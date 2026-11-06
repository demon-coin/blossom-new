//! VFS 挂载拓扑、路径逐组件解析（REF-walk）与目录命名空间事务操作。
//!
//! # 模块职责与架构定位
//! 本模块负责连接进程视角的文件路径与底层具体文件系统实例（`ext4` 或各类内存文件系统 `tmpfs`/`devfs`/`proc`）：
//! - **上游调用者**：系统调用层（`openat`、`mkdirat`、`unlinkat`、`linkat`、`symlinkat`、`renameat2`、`getcwd`、`chdir`、`chroot`、`mount`/`umount2` 等）、进程管理（`FsContext` 继承与切换）以及内核启动挂载流程。
//! - **下游依赖**：`DentryCache` / `DentryRef`（维护单挂载域内的目录项拓扑与名字缓存）、`InodeCache` / `InodeRef`（执行实际目录项读写、权限快照获取与节点分配）以及 `Filesystem`（提供跨实例重命名互斥与磁盘同步）。
//!
//! # 核心设计与挂载拓扑模型
//! ## 1. 路径身份（`Path`）与单向挂载覆盖树（`Mount`）
//! 一个活跃路径由 `(Arc<Mount>, DentryRef)` 二元组唯一标识：即使同一个底层文件系统被绑定到不同位置，各挂载实例仍拥有独立的读写模式（`AccessMode`）和拓扑位置。
//! 子挂载通过 `Mount::covered` 单向强引用父挂载中被覆盖的目录 `Path`，父挂载不反向持有子挂载，从而保证无环引用计数；所有活跃挂载统一登记在 `Vfs::mounts` 表中：
//! ```text
//! Vfs::mounts[0] (根挂载 Mount, covered = None)
//!   └── root Dentry ("/")
//!         └── child Dentry ("mnt")  <─── covered ─── Vfs::mounts[k] (子挂载 Mount)
//!               [在父视图中被遮蔽]                      └── root Dentry ("/")
//!                                                             └── child Dentry ("file")
//!
//! 穿越方向：
//! - 向下解析 (follow_mounts)：命中有子挂载覆盖的目录时，在 Vfs::mounts 自旋锁内循环跳转到最顶层子挂载的 root。
//! - 向上回退 (parent_within)：处理 ".." 时，若当前位于子挂载 root，先沿 covered 退回父挂载被覆盖目录，再取其父 Dentry（遇到 FsContext::root 边界则原地停止）。
//! ```
//!
//! ## 2. 逐组件引用行走（REF-walk）与“先准备、后无失败发布”目录事务
//! - **逐组件查找**：`resolve_from_with_links` 不对 `"."` 或 `".."` 做纯字符串词法折叠，而是每走一步都先验证当前位置确实是目录并检查搜索权限（`MAY_EXEC`），避免 `"non_dir/.."` 或 `"missing/.."` 绕过目录存在性与权限校验；符号链接展开在单次查找共享的 `MAXSYMLINKS`（40 次）预算内执行。
//! - **命名空间修改事务**：`open_with(O_CREAT)`、`mkdir_from`、`symlink_from`、`unlink_from`、`link_from`、`rmdir_from` 与 `rename_from` 均先解析到父目录（`LookupParent`），再获取父目录睡眠写锁重查目标名字，先完成所有可能失败的内存预留、Inode 分配与磁盘块编辑准备（`prepare_*`），最后依次执行零失败的提交发布（`publish*`）并更新 `DentryCache`。
//!
//! # 锁获取顺序（Lock Ordering）与并发契约
//! 1. **文件系统实例级重命名睡眠互斥锁（`Filesystem::lock_rename`）**：
//!    - 同挂载 `rename_from` 先锁住该挂载所属 `Filesystem` 以固定目录树祖先关系；
//!    - 跨挂载反向拼装路径（`path_bytes`）时，先收集沿途所有去重后的 `Filesystem`，并严格按底层 `Arc` 实例地址升序（`Filesystem::identity_address`）依次获取 `lock_rename`。
//! 2. **目录 Inode 睡眠读写锁（`directory_write` / `directory_read`）**：
//!    - 双目录重命名时，若一端是另一端的祖先，必须**先锁祖先父目录、再锁后代父目录**；互不隶属时固定先锁源父目录、再锁目标父目录；
//!    - 父目录写锁必须在子目录写锁或普通子 Inode 元数据写锁（`inode_write`）之前获取。
//! 3. **非目录子 Inode 睡眠写锁（`inode_write`）**：
//!    - 当同一卷内需同时锁定两个非目录子 Inode（如文件覆盖重命名）时，严格按卷内 Inode 编号升序（`source.number() < target.number()`）获取。
//! 4. **`Vfs::mounts` 挂载表自旋锁**：
//!    - 叶子级短临界区锁（可在持有目录睡眠读/写锁期间短暂获取，如 `is_mountpoint` 和 `mount`）；
//!    - **严禁**在持有 `mounts` 自旋锁期间执行内存分配、路径解析、块设备同步（`sync`）、睡眠等待或析构可能级联释放底层资源的 `Path` / `Arc<Mount>`。
//! 5. **`DentryCache` 索引自旋锁与 `Dentry` 拓扑自旋锁**：位于目录锁或 `lock_rename` 保护之下，遵循 `dentry` 模块内部锁序。

use alloc::borrow::Cow;
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::block::AccessMode;
use crate::fs::memfs::{Filesystem, MemoryFsKind};
use crate::fs::types::{
    MAY_EXEC, MAY_READ, MAY_WRITE, NAME_MAX, PATH_MAX, S_IALLUGO, S_IRWXUGO, S_ISVTX, S_IXUGO,
};
use crate::fs::{
    DentryCache, DentryRef, DirectoryWriteGuard, FsError, InodeCache, InodeKind, InodeMetadata,
    InodeRef,
};
use crate::sync::SpinLock;

/// 单次路径解析允许跟随符号链接的最大累计次数（超过则返回 `FsError::TooManySymlinks`，对应 `ELOOP`）。
const MAXSYMLINKS: usize = 40;
/// 新建 `FsContext` 时进程文件创建模式掩码的初始默认值（八进制 `0022`，屏蔽同组与其他用户的写权限）。
const DEFAULT_UMASK: u16 = 0o022;

/// POSIX 权限位中文件属主（Owner）读/写/执行三位相对于低 3 位的右移位数。
const OWNER_PERMISSION_SHIFT: u32 = 6;
/// POSIX 权限位中文件属组（Group）读/写/执行三位相对于低 3 位的右移位数。
const GROUP_PERMISSION_SHIFT: u32 = 3;

/// 控制路径解析遇到**末尾组件为符号链接且无尾随斜杠**时的处理策略。
///
/// 路径中间的符号链接组件、以及带有尾随斜杠 `.../` 的末尾符号链接组件始终会被继续跟随。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ResolveMode {
    /// 跟随末尾符号链接并继续解析其目标路径。
    Follow,
    /// 不跟随末尾符号链接，直接返回该符号链接自身的 `Path`（供 `lstat`、`readlink` 等使用）。
    NoFollow,
}

/// VFS 命名空间中的一个具体位置，由所属挂载实例与目录项引用共同决定。
///
/// 持有 `Path` 会通过引用计数同时保活所在挂载实例（及其关联的底层文件系统和 `DentryCache`）与当前 `DentryRef`。
#[derive(Clone)]
pub struct Path {
    /// 该路径所在的挂载实例引用，决定挂载级读写权限与向上穿越覆盖点的拓扑边。
    mount: Arc<Mount>,
    /// 该路径在当前挂载域内的目录项引用。
    dentry: DentryRef,
}

/// 文件系统挂载实例，绑定一个底层文件系统后端、独立的挂载读写模式与共享目录项缓存。
///
/// 子挂载通过 `covered` 单向强引用父挂载视图中的被覆盖目录，父挂载不反向持有子挂载，防止循环引用。
struct Mount {
    /// 挂载承载的底层文件系统实例（`Ext4` 磁盘卷或内存文件系统）。
    filesystem: Filesystem,
    /// 本挂载点独立的访问模式（`ReadOnly` 或 `ReadWrite`），可比底层文件系统自身模式更严格。
    mode: AccessMode,
    /// 该挂载使用的目录项缓存（同一后端域的多个挂载可共享该缓存实例）。
    dcache: Arc<DentryCache>,
    /// 该挂载根目录对应的目录项引用。
    root: DentryRef,
    /// 父挂载视图中被本挂载覆盖的挂载点目录路径；根挂载（`mounts[0]`）此字段为 `None`。
    covered: Option<Path>,
}

/// 进程或任务级的文件系统路径解析上下文。
///
/// 保存绝对路径与 `".."` 上溯的根目录边界、相对路径的当前工作目录起点，以及创建文件/目录时的权限掩码。
#[derive(Clone)]
pub struct FsContext {
    /// 当前上下文的根目录边界（支持 `chroot` 隔离，`resolve` 与 `".."` 均不会越过该目录向上回退）。
    root: Path,
    /// 相对路径解析的默认起始工作目录。
    cwd: Path,
    /// 进程级文件创建权限屏蔽位（仅保留低 9 位 `S_IRWXUGO`）。
    umask: u16,
}

/// 父目录查找（`lookup_parent_from`）拆分出的路径末尾原始组件分类。
///
/// 末尾组件在父目录查找阶段尚未实际进入 `dcache` 解析，留给具体的 `open`/`mkdir`/`unlink`/`rename` 按各自语义处理。
#[derive(Clone)]
pub enum LastComponent {
    /// 普通文件或目录名组件（非空、不含 `'/'` 与 `'\0'`，且长度不超过 `NAME_MAX` 字节）。
    Name(Arc<Vec<u8>>),
    /// 当前目录特殊组件 `"."`。
    Dot,
    /// 父目录特殊组件 `".."`。
    DotDot,
    /// 整条路径仅由一个或多个 `'/'` 组成，代表上下文根目录本身。
    Root,
}

/// 父目录解析结果，供创建、删除、硬链接与重命名等目录修改操作在加锁前定位父目录。
#[derive(Clone)]
pub struct LookupParent {
    /// 已完成挂载穿越与目录类型校验的父目录路径（当 `component` 为 `Root` 时即为 `context.root`）。
    pub parent: Path,
    /// 尚未在 `parent` 下查找或发布的末尾路径组件。
    pub component: LastComponent,
    /// 原始路径末尾是否带有至少一个 `'/'`（用于强制要求目标为目录或拒绝创建普通文件）。
    pub trailing_slash: bool,
}

/// 全局虚拟文件系统（VFS）实例，统一管理挂载表、根命名空间缓存与共享磁盘 Inode 缓存。
///
/// ponytail: 有限挂载预算内线性查找，预算显著增大时改用有限散列表。
pub struct Vfs {
    /// 根文件系统初始化时建立的目录项缓存。
    dcache: Arc<DentryCache>,
    /// 供所有磁盘文件系统（`ext4`）共用的全局 Inode 缓存池。
    inodes: Arc<InodeCache>,
    /// 受自旋锁保护的活动挂载实例列表；槽位 `0` 固定存放不可卸载的根挂载，路径穿越与卸载忙检查共用此短临界区锁。
    mounts: SpinLock<Vec<Arc<Mount>>>,
    /// 挂载表允许同时登记的最大挂载实例数量（包含槽位 `0` 的根挂载，单位：个）。
    mount_capacity: usize,
}

impl Path {
    /// 借用当前路径持有的目录项引用（仅供 `ktest` 断言缓存淘汰不会改变已有 `Path` 的对象身份）。
    #[cfg(feature = "ktest")]
    pub fn dentry(&self) -> &DentryRef {
        &self.dentry
    }

    /// 等待底层目录项完成单飞（Singleflight）绑定，并借用其关联的共享 `InodeRef`。
    pub fn inode(&self) -> Result<&InodeRef, FsError> {
        self.dentry.inode()
    }

    /// 判断两个 `Path` 是否指向完全相同的挂载实例与目录项实例（同一底层目录在不同绑定挂载中的位置视为不相等）。
    pub fn ptr_eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.mount, &other.mount) && self.dentry.ptr_eq(&other.dentry)
    }

    /// 借用当前路径所属挂载承载的底层文件系统实例。
    pub fn filesystem(&self) -> &Filesystem {
        &self.mount.filesystem
    }

    /// 返回当前路径所属挂载实例自身的读写访问模式（不受同一后端上其他挂载点模式的影响）。
    pub fn mount_mode(&self) -> AccessMode {
        self.mount.mode
    }

    /// 原地重新配置 `tmpfs` 挂载根目录的运行期选项（如容量或 Inode 配额上限）。
    ///
    /// 仅允许对 `tmpfs` 挂载根调用，且暂不支持动态切换挂载的只读/读写（`RO`/`RW`）模式；
    /// 调用期间当前 `&Path` 强引用原 `Mount`，可阻止并发 `unmount` 将该挂载实例摘除。
    pub fn reconfigure_tmpfs(&self, mode: AccessMode, options: &[u8]) -> Result<(), FsError> {
        if !self.dentry.ptr_eq(&self.mount.root) || mode != self.mount.mode {
            return Err(FsError::InvalidOperation);
        }
        match &self.mount.filesystem {
            Filesystem::Memory(fs) => fs.reconfigure(options),
            Filesystem::Ext4(_) => Err(FsError::InvalidOperation),
        }
    }

    /// 复用当前路径的挂载实例，为同一文件系统上的匿名（未链接）`Inode` 构造一个未入 `dcache` 哈希索引的 `Path`。
    ///
    /// 调用上下文：普通可睡眠任务上下文，且 `inode` 必须属于与当前挂载相同的底层文件系统设备。
    #[cfg(feature = "ktest")]
    pub(crate) fn unhashed_inode(&self, inode: InodeRef) -> Result<Self, FsError> {
        if inode.device() != self.mount.filesystem.device() {
            return Err(FsError::InvalidOperation);
        }
        let dentry = DentryRef::try_unhashed(inode)?;
        Ok(Self {
            mount: self.mount.clone(),
            dentry,
        })
    }

    /// 按给定的用户凭据（`uid`/`gid`）与访问掩码（`MAY_READ` | `MAY_WRITE` | `MAY_EXEC`）检查当前路径的访问权限。
    ///
    /// 普通文件、目录和符号链接的写请求同时受挂载与后端读写模式限制；字符设备与块设备节点的数据读写仅检查节点自身的自主访问控制（DAC）权限位。
    pub fn check_access(&self, uid: u32, gid: u32, mask: u16) -> Result<(), FsError> {
        if mask & !(MAY_READ | MAY_WRITE | MAY_EXEC) != 0 {
            return Err(FsError::InvalidOperation);
        }
        let data = self.inode()?.metadata_snapshot()?;
        check_access_metadata(self, &data, uid, gid, mask)
    }

    /// 检查当前路径所在的挂载实例及底层文件系统后端是否均处于可写模式（`AccessMode::ReadWrite`）。
    pub(crate) fn check_writable(&self) -> Result<(), FsError> {
        if self.mount.mode != AccessMode::ReadWrite
            || self.mount.filesystem.mode() != AccessMode::ReadWrite
        {
            return Err(FsError::ReadOnly);
        }
        Ok(())
    }

    /// 沿挂载与目录树向上走一步拓扑边：若当前位于子挂载根则返回其覆盖点 `covered`，否则返回同挂载内的父目录项。
    fn topology_parent(&self) -> Option<Self> {
        if self.dentry.ptr_eq(&self.mount.root) {
            return self.mount.covered.clone();
        }
        self.dentry.parent().map(|parent| Self {
            mount: self.mount.clone(),
            dentry: parent,
        })
    }

    /// 在不越过根目录边界 `root` 的前提下解析 `".."` 对应的父目录路径。
    ///
    /// 若当前处于多层嵌套子挂载的根目录，会先沿 `covered` 链持续退回到被覆盖的底层目录，再向上跨越一级普通父目录；途中一旦遇到 `root` 立即停止并返回 `root`。
    fn parent_within(&self, root: &Self) -> Self {
        let mut current = self.clone();
        loop {
            if current.ptr_eq(root) {
                return current;
            }
            let at_mount_root = current.dentry.ptr_eq(&current.mount.root);
            let Some(parent) = current.topology_parent() else {
                return current;
            };
            if !at_mount_root {
                return parent;
            }
            current = parent;
        }
    }
}

impl FsContext {
    /// 校验 `root` 与 `cwd` 均为有效目录且 `root` 是 `cwd` 的拓扑祖先，并以默认掩码 `0o022` 创建上下文。
    pub fn new(root: Path, cwd: Path) -> Result<Self, FsError> {
        require_directory(&root)?;
        require_directory(&cwd)?;
        let mut ancestor = cwd.clone();
        loop {
            if ancestor.ptr_eq(&root) {
                return Ok(Self {
                    root,
                    cwd,
                    umask: DEFAULT_UMASK,
                });
            }
            ancestor = ancestor
                .topology_parent()
                .ok_or(FsError::InvalidOperation)?;
        }
    }

    /// 借用当前上下文的根目录路径边界。
    pub fn root(&self) -> &Path {
        &self.root
    }

    /// 借用当前上下文的相对路径起始工作目录。
    pub fn cwd(&self) -> &Path {
        &self.cwd
    }

    /// 读取当前进程的文件创建模式屏蔽位快照。
    pub fn umask(&self) -> u16 {
        self.umask
    }

    /// 在持有外部 `FsContext` 锁的临界区内替换 `umask`（仅保留低 9 位权限掩码），并返回旧值。
    pub fn exchange_umask(&mut self, mask: u16) -> u16 {
        core::mem::replace(&mut self.umask, mask & S_IRWXUGO)
    }

    /// 在持有外部 `FsContext` 锁的临界区内替换当前工作目录 `cwd`，并返回旧 `Path` 以便调用方在锁外释放。
    ///
    /// 调用方必须在加锁前完成新 `cwd` 的目录类型与搜索权限校验。
    pub fn exchange_cwd(&mut self, cwd: Path) -> Path {
        core::mem::replace(&mut self.cwd, cwd)
    }

    /// 在持有外部 `FsContext` 锁的临界区内替换根目录 `root`（保留 `cwd` 与 `umask`），并返回旧 `Path` 供锁外释放。
    ///
    /// 调用方必须在加锁前完成新 `root` 的目录类型、搜索权限及换根特权校验。
    pub fn exchange_root(&mut self, root: Path) -> Path {
        core::mem::replace(&mut self.root, root)
    }
}

impl Vfs {
    /// 使用指定的根文件系统、共享 `InodeCache`、缓存容量及挂载槽位上限初始化 `Vfs`，并在槽位 `0` 登记根挂载。
    pub fn try_new(
        filesystem: Filesystem,
        inodes: Arc<InodeCache>,
        dentry_capacity: usize,
        mount_capacity: usize,
        root_mode: AccessMode,
    ) -> Result<Self, FsError> {
        if mount_capacity == 0 {
            return Err(FsError::InvalidOperation);
        }
        if root_mode == AccessMode::ReadWrite && filesystem.mode() != AccessMode::ReadWrite {
            return Err(FsError::ReadOnly);
        }
        if dentry_capacity == 0 {
            return Err(FsError::InvalidOperation);
        }

        let mut mounts = Vec::new();
        mounts
            .try_reserve_exact(mount_capacity)
            .map_err(|_| FsError::NoMemory)?;
        let (dcache, root) = filesystem.dentry_namespace(&inodes, dentry_capacity)?;
        let mount = Arc::try_new(Mount {
            filesystem,
            mode: root_mode,
            dcache: dcache.clone(),
            root,
            covered: None,
        })
        .map_err(|_| FsError::NoMemory)?;
        mounts.push(mount);
        Ok(Self {
            dcache,
            inodes,
            mounts: SpinLock::new(mounts),
            mount_capacity,
        })
    }

    /// 在挂载表自旋锁内克隆并返回 VFS 全局根挂载的根目录 `Path`。
    pub fn root(&self) -> Path {
        let mounts = self.mounts.lock();
        let mount = &mounts[0];
        Path {
            mount: mount.clone(),
            dentry: mount.root.clone(),
        }
    }

    /// 借用根挂载域的目录项缓存（仅供 `ktest` 观察缓存状态）。
    #[cfg(feature = "ktest")]
    pub fn dcache(&self) -> &DentryCache {
        self.dcache.as_ref()
    }

    /// 借用所有磁盘后端共用的全局 `InodeCache`（仅供 `ktest` 观察缓存状态）。
    #[cfg(feature = "ktest")]
    pub fn inodes(&self) -> &Arc<InodeCache> {
        &self.inodes
    }

    /// 在沿途文件系统的 `lock_rename` 互斥保护下，拼装 `context.cwd` 相对于 `context.root` 的规范绝对路径字节串（不含末尾 `'\0'`，总长度上限保留调用方 `'\0'` 预算，符合 `PATH_MAX` 4096 字节限制）。
    pub fn cwd_bytes(&self, context: &FsContext) -> Result<Vec<u8>, FsError> {
        path_bytes(context.cwd(), context.root())
    }

    /// 从 `context.cwd` 出发、不附加用户 DAC 凭据（内核态内部查找）解析路径字符串 `pathname`。
    pub fn resolve(
        &self,
        context: &FsContext,
        pathname: &[u8],
        mode: ResolveMode,
    ) -> Result<Path, FsError> {
        self.resolve_from(context, context.cwd(), pathname, mode, None)
    }

    /// 从指定起点 `start`（相对路径）或 `context.root`（绝对路径）逐组件解析路径 `pathname`。
    ///
    /// 若传入 `credentials = Some((uid, gid))`，则在进入每一级目录前都会校验目录搜索权限（`MAY_EXEC`）。
    pub fn resolve_from(
        &self,
        context: &FsContext,
        start: &Path,
        pathname: &[u8],
        mode: ResolveMode,
        credentials: Option<(u32, u32)>,
    ) -> Result<Path, FsError> {
        let mut links = 0;
        self.resolve_from_with_links(context, start, pathname, mode, credentials, &mut links)
    }

    /// 在调用链共享的符号链接展开计数器 `links` 预算内，执行逐组件路径解析（REF-walk）。
    fn resolve_from_with_links(
        &self,
        context: &FsContext,
        start: &Path,
        pathname: &[u8],
        mode: ResolveMode,
        credentials: Option<(u32, u32)>,
        links: &mut usize,
    ) -> Result<Path, FsError> {
        // Step 1: 拒绝空路径与内嵌 NUL 字节，并校验解析起点所在的挂载实例仍登记在当前挂载表中。
        if pathname.is_empty() {
            return Err(FsError::NotFound);
        }
        if pathname.contains(&0) {
            return Err(FsError::InvalidName);
        }

        self.validate_start(context, start, pathname[0] == b'/')?;
        let mut current = if pathname[0] == b'/' {
            context.root.clone()
        } else {
            start.clone()
        };
        let mut pending = Cow::Borrowed(pathname);
        let mut offset = 0;

        // Step 2: 逐个斜杠分隔组件步进行走。
        while offset < pending.len() {
            // 先确认当前位置确为目录并检查搜索权限，再跳过斜杠或处理 "." / ".."，防止 "file/.." 词法折叠绕过检查。
            require_directory(&current)?;
            if let Some((uid, gid)) = credentials {
                current.check_access(uid, gid, MAY_EXEC)?;
            }
            while offset < pending.len() && pending[offset] == b'/' {
                offset += 1;
            }
            if offset == pending.len() {
                return Ok(current);
            }
            let end = pending[offset..]
                .iter()
                .position(|byte| *byte == b'/')
                .map_or(pending.len(), |length| offset + length);
            let name = &pending[offset..end];
            if name == b"." {
                offset = end;
                continue;
            }
            if name == b".." {
                current = self.follow_mounts(current.parent_within(&context.root));
                offset = end;
                continue;
            }

            // Step 3: 在当前挂载的 dcache 中查找子项并向下跨越任何覆盖在该目录上的子挂载。
            let dentry = current
                .mount
                .dcache
                .lookup(&self.inodes, &current.dentry, name)?;
            let next = self.follow_mounts(Path {
                mount: current.mount.clone(),
                dentry,
            });
            let inode = next.inode()?;

            // Step 4: 若命中符号链接，且属于中间组件（end < pending.len()）或调用方要求跟随末尾链接，则在预算内展开链接目标并拼接剩余后缀。
            let follow = inode.metadata_snapshot()?.kind == InodeKind::Symlink
                && (mode == ResolveMode::Follow || end < pending.len());
            if follow {
                if *links == MAXSYMLINKS {
                    return Err(FsError::TooManySymlinks);
                }
                *links += 1;
                let mut target = inode.read_link()?;
                if target.is_empty() {
                    return Err(FsError::NotFound);
                }
                let absolute = target.first() == Some(&b'/');
                // 将未消费的原路径后缀（含可能存在的尾随斜杠）原样拼接到链接目标之后，确保尾斜杠依然强制校验目标为目录。
                target
                    .try_reserve_exact(pending.len() - end)
                    .map_err(|_| FsError::NoMemory)?;
                target.extend_from_slice(&pending[end..]);
                pending = Cow::Owned(target);
                offset = 0;
                if absolute {
                    current = context.root.clone();
                }
            } else {
                current = next;
                offset = end;
            }
        }
        Ok(current)
    }

    /// 解析路径的父目录部分，同时原样保留末尾组件名称、点组件语义（`.` / `..` / `/`）及尾随斜杠标志。
    pub fn lookup_parent_from(
        &self,
        context: &FsContext,
        start: &Path,
        pathname: &[u8],
        credentials: Option<(u32, u32)>,
    ) -> Result<LookupParent, FsError> {
        let mut links = 0;
        self.lookup_parent_with_links(context, start, pathname, credentials, &mut links)
    }

    /// 在共享符号链接计数 `links`（上限 `MAXSYMLINKS`）下解析父目录并在最后一个组件前停下。
    fn lookup_parent_with_links(
        &self,
        context: &FsContext,
        start: &Path,
        pathname: &[u8],
        credentials: Option<(u32, u32)>,
        links: &mut usize,
    ) -> Result<LookupParent, FsError> {
        // Step 1: 校验非空且无 NUL 字节，剥离末尾连续斜杠并识别纯根路径（如 "/" 或 "///"）。
        if pathname.is_empty() {
            return Err(FsError::NotFound);
        }
        if pathname.contains(&0) {
            return Err(FsError::InvalidName);
        }

        let trailing_slash = pathname.last() == Some(&b'/');
        let mut end = pathname.len();
        while 0 < end && pathname[end - 1] == b'/' {
            end -= 1;
        }
        if end == 0 {
            self.validate_start(context, start, true)?;
            require_directory(&context.root)?;
            if let Some((uid, gid)) = credentials {
                context.root.check_access(uid, gid, 1)?;
            }
            return Ok(LookupParent {
                parent: context.root.clone(),
                component: LastComponent::Root,
                trailing_slash: true,
            });
        }

        // Step 2: 切分前缀目录路径与最后一个原始组件；若无前缀则直接校验相对起点 start 作为父目录。
        let start_at = pathname[..end]
            .iter()
            .rposition(|byte| *byte == b'/')
            .map_or(0, |position| position + 1);
        let raw_name = &pathname[start_at..end];
        let prefix = &pathname[..start_at];
        let parent = if prefix.is_empty() {
            self.validate_start(context, start, false)?;
            require_directory(start)?;
            if let Some((uid, gid)) = credentials {
                start.check_access(uid, gid, 1)?;
            }
            start.clone()
        } else {
            self.resolve_from_with_links(
                context,
                start,
                prefix,
                ResolveMode::Follow,
                credentials,
                links,
            )?
        };

        // Step 3: 将末尾组件分类为 "."、".." 或经长度与字符校验后的普通名字。
        let component = if raw_name == b"." {
            LastComponent::Dot
        } else if raw_name == b".." {
            LastComponent::DotDot
        } else {
            LastComponent::Name(try_raw_name(raw_name)?)
        };
        Ok(LookupParent {
            parent,
            component,
            trailing_slash,
        })
    }

    /// 在挂载表自旋锁临界区内校验 `context.root`（以及相对路径下的起点 `start`）所属挂载仍登记在活跃挂载表中；绝对路径直接忽略 `start`。
    fn validate_start(
        &self,
        context: &FsContext,
        start: &Path,
        absolute: bool,
    ) -> Result<(), FsError> {
        let mounts = self.mounts.lock();
        if !contains_mount(&mounts, &context.root.mount)
            || (!absolute && !contains_mount(&mounts, &start.mount))
        {
            return Err(FsError::InvalidOperation);
        }
        Ok(())
    }

    /// 在已持有父目录睡眠写锁的前提下，为父目录下已查到的现有 `inode` 获取或建立正绑定的子 `Path`。
    ///
    /// 若 `dcache` 中已有指向同一 `inode` 的缓存项则直接复用；若缓存未命中或残留旧负目录项，则在 `dcache` 中预留并发布正目录项。
    fn existing_child_locked(
        &self,
        parent: &Path,
        name: Arc<Vec<u8>>,
        inode: InodeRef,
    ) -> Result<Path, FsError> {
        if let Some(dentry) = parent
            .mount
            .dcache
            .cached_locked(&parent.dentry, name.as_slice())
        {
            match dentry.inode() {
                Ok(cached) if cached.ptr_eq(&inode) => {
                    return Ok(Path {
                        mount: parent.mount.clone(),
                        dentry,
                    });
                }
                Ok(_) => return Err(FsError::InvalidOperation),
                Err(FsError::NotFound) => {}
                Err(error) => return Err(error),
            }
        }

        let prepared = parent
            .mount
            .dcache
            .try_prepare_positive(&parent.dentry, name, inode)?;
        let dentry = prepared.dentry().clone();
        prepared.publish();
        Ok(Path {
            mount: parent.mount.clone(),
            dentry,
        })
    }

    /// 判断 `path` 是否为挂载点（即当前挂载自身的根目录，或在挂载表自旋锁下发现被某个子挂载的 `covered` 覆盖）。
    fn is_mountpoint(&self, path: &Path) -> bool {
        if path.dentry.ptr_eq(&path.mount.root) {
            return true;
        }
        self.mounts.lock().iter().any(|mount| {
            mount
                .covered
                .as_ref()
                .is_some_and(|covered| covered.ptr_eq(path))
        })
    }

    /// 解析路径并打开已有目标，或在持有最终父目录写锁期间原子地完成新常规文件创建、打开文件描述（OFD）准备与目录项发布。
    ///
    /// 参数说明：
    /// - `create_mode`：`Some((requested_mode, umask_snapshot))` 表示携带 `O_CREAT`，仅在实际新建文件时应用权限与掩码；`None` 表示仅打开现存路径。
    /// - `exclusive`：对应 `O_EXCL`（若未指定 `create_mode` 则返回 `InvalidOperation`）。
    /// - `no_follow`：对应 `O_NOFOLLOW`，末尾组件为符号链接时拒绝跟随并返回 `TooManySymlinks`（映射为 `ELOOP`）。
    /// - `prepare(path, created)`：在目录项对其他并发查找可见之前构造上层句柄（如 `FileRef`），若回调返回错误则新文件与目录项均不会发布。
    #[expect(
        clippy::too_many_arguments,
        reason = "open 的解析起点、创建与链接选项及 OFD 准备回调共同约束最终目录锁内的单次发表"
    )]
    pub fn open_with<T>(
        &self,
        context: &FsContext,
        start: &Path,
        pathname: &[u8],
        credentials: (u32, u32),
        create_mode: Option<(u16, u16)>,
        exclusive: bool,
        no_follow: bool,
        prepare: impl FnOnce(Path, bool) -> Result<T, FsError>,
    ) -> Result<T, FsError> {
        if exclusive && create_mode.is_none() {
            return Err(FsError::InvalidOperation);
        }
        let mut prepare = Some(prepare);
        // Step 1: 未请求 O_CREAT 时直接走常规路径解析，校验 O_NOFOLLOW 后交给回调打开。
        if create_mode.is_none() {
            let path = self.resolve_from(
                context,
                start,
                pathname,
                if no_follow {
                    ResolveMode::NoFollow
                } else {
                    ResolveMode::Follow
                },
                Some(credentials),
            )?;
            if no_follow && path.inode()?.metadata_snapshot()?.kind == InodeKind::Symlink {
                return Err(FsError::TooManySymlinks);
            }
            return prepare
                .take()
                .expect("open preparation callback was consumed")(path, false);
        }

        // Step 2: 请求 O_CREAT 时循环解析到父目录；若末尾命中已有符号链接且允许跟随，则在释放父目录锁后继续展开链接目标。
        let (requested_mode, umask) = create_mode.expect("create mode was checked above");
        let mut current_start = start.clone();
        let mut pending = Cow::Borrowed(pathname);
        let mut links = 0;
        loop {
            let lookup = self.lookup_parent_with_links(
                context,
                &current_start,
                pending.as_ref(),
                Some(credentials),
                &mut links,
            )?;
            if lookup.trailing_slash {
                return Err(FsError::IsDirectory);
            }
            match lookup.component {
                LastComponent::Name(name) => {
                    // Step 2a: 锁住父目录并二次查询目标名字，处理并发创建或已存在的文件/符号链接。
                    let parent_inode = lookup.parent.inode()?.clone();
                    let directory = parent_inode.directory_write()?;
                    let parent_metadata = directory.metadata_snapshot();
                    check_access_metadata(
                        &lookup.parent,
                        &parent_metadata,
                        credentials.0,
                        credentials.1,
                        MAY_EXEC,
                    )?;
                    if let Some(inode) = directory.lookup(&self.inodes, name.as_slice())? {
                        if exclusive {
                            return Err(FsError::Exists);
                        }
                        let metadata = inode.metadata_snapshot()?;
                        if metadata.kind == InodeKind::Symlink {
                            if no_follow {
                                return Err(FsError::TooManySymlinks);
                            }
                            if links == MAXSYMLINKS {
                                return Err(FsError::TooManySymlinks);
                            }
                            links += 1;
                            drop(directory);
                            let target = inode.read_link()?;
                            if target.is_empty() {
                                return Err(FsError::NotFound);
                            }
                            let absolute = target.first() == Some(&b'/');
                            current_start = if absolute {
                                context.root.clone()
                            } else {
                                lookup.parent
                            };
                            pending = Cow::Owned(target);
                            continue;
                        }
                        let path = self.existing_child_locked(&lookup.parent, name, inode)?;
                        drop(directory);
                        return prepare
                            .take()
                            .expect("open preparation callback was consumed")(
                            self.follow_mounts(path),
                            false,
                        );
                    }

                    // Step 2b: 目标名字不存在：校验父目录写权限，先完成 dcache 槽位预留、新 Inode 分配、OFD 回调准备与目录项插入准备。
                    check_parent_write(
                        &lookup.parent,
                        &parent_metadata,
                        credentials.0,
                        credentials.1,
                    )?;
                    let prepared_dentry = lookup
                        .parent
                        .mount
                        .dcache
                        .try_prepare_child(&lookup.parent.dentry, name.clone())?;
                    if !parent_inode.is_memory() && self.inodes.is_full() {
                        lookup.parent.mount.dcache.prune_unused();
                    }
                    let now_ns = crate::time::realtime();
                    let new_inode = self.inodes.prepare_regular_inode(
                        &parent_inode,
                        &parent_metadata,
                        requested_mode & S_IALLUGO,
                        umask & S_IRWXUGO,
                        credentials.0,
                        credentials.1,
                        now_ns,
                    )?;
                    let inode = new_inode.inode().clone();
                    let prepared_dentry = prepared_dentry.bind_positive(inode.clone());
                    let path = Path {
                        mount: lookup.parent.mount.clone(),
                        dentry: prepared_dentry.dentry().clone(),
                    };
                    let file = prepare
                        .take()
                        .expect("open preparation callback was consumed")(
                        path, true
                    )?;

                    let edit = directory.prepare_insert(
                        name.as_slice(),
                        inode.number(),
                        InodeKind::Regular,
                        0,
                        now_ns,
                    )?;

                    // Step 2c: 所有可能失败的步骤均已就绪，依次执行无失败发布：Inode -> 目录内容 -> dcache。
                    let _published_inode = new_inode.publish();
                    directory.publish(edit);
                    prepared_dentry.publish();
                    return Ok(file);
                }
                LastComponent::Dot => {
                    if exclusive {
                        return Err(FsError::Exists);
                    }
                    return prepare
                        .take()
                        .expect("open preparation callback was consumed")(
                        lookup.parent, false
                    );
                }
                LastComponent::DotDot => {
                    if exclusive {
                        return Err(FsError::Exists);
                    }
                    let path = self.follow_mounts(lookup.parent.parent_within(&context.root));
                    return prepare
                        .take()
                        .expect("open preparation callback was consumed")(
                        path, false
                    );
                }
                LastComponent::Root => {
                    if exclusive {
                        return Err(FsError::Exists);
                    }
                    return prepare
                        .take()
                        .expect("open preparation callback was consumed")(
                        context.root.clone(),
                        false,
                    );
                }
            }
        }
    }

    /// 在目标父目录睡眠写锁下创建新子目录。
    ///
    /// 先解析父目录并在写锁内重查名字，随后依次完成 `dcache` 槽位预留、新目录 Inode 分配与父目录插入编辑准备（父目录硬链接数 `+1`），最后一次性发布。
    pub fn mkdir_from(
        &self,
        context: &FsContext,
        start: &Path,
        pathname: &[u8],
        mode: u16,
        credentials: (u32, u32),
        umask: u16,
    ) -> Result<(), FsError> {
        let lookup = self.lookup_parent_from(context, start, pathname, Some(credentials))?;
        let name = match lookup.component {
            LastComponent::Name(name) => name,
            LastComponent::Dot | LastComponent::DotDot | LastComponent::Root => {
                return Err(FsError::Exists);
            }
        };

        // Step 1: 获取父目录写锁，确认目标名字尚不存在且当前凭据对父目录拥有写与搜索权限。
        let parent_inode = lookup.parent.inode()?.clone();
        let directory = parent_inode.directory_write()?;
        let parent_metadata = directory.metadata_snapshot();
        if directory.lookup(&self.inodes, name.as_slice())?.is_some() {
            check_access_metadata(
                &lookup.parent,
                &parent_metadata,
                credentials.0,
                credentials.1,
                1,
            )?;
            return Err(FsError::Exists);
        }
        check_parent_write(
            &lookup.parent,
            &parent_metadata,
            credentials.0,
            credentials.1,
        )?;

        // Step 2: 预留缓存槽位、分配目录 Inode 并准备父目录项插入编辑；全部就绪后执行无失败发布。
        let prepared_dentry = lookup
            .parent
            .mount
            .dcache
            .try_prepare_child(&lookup.parent.dentry, name.clone())?;
        if !parent_inode.is_memory() && self.inodes.is_full() {
            lookup.parent.mount.dcache.prune_unused();
        }
        let now_ns = crate::time::realtime();
        let new_inode = self.inodes.prepare_directory_inode(
            &parent_inode,
            &parent_metadata,
            mode & (S_ISVTX | S_IRWXUGO),
            umask & S_IRWXUGO,
            credentials.0,
            credentials.1,
            now_ns,
        )?;
        let inode = new_inode.inode().clone();
        let prepared_dentry = prepared_dentry.bind_positive(inode.clone());
        let edit = directory.prepare_insert(
            name.as_slice(),
            inode.number(),
            InodeKind::Directory,
            1,
            now_ns,
        )?;

        let _published_inode = new_inode.publish();
        directory.publish(edit);
        prepared_dentry.publish();
        Ok(())
    }

    /// 在目标父目录睡眠写锁下创建指向原始字节串 `target` 的符号链接（短内联链接或长块存储链接由底层文件系统决定）。
    pub fn symlink_from(
        &self,
        context: &FsContext,
        start: &Path,
        pathname: &[u8],
        target: &[u8],
        credentials: (u32, u32),
    ) -> Result<(), FsError> {
        let lookup = self.lookup_parent_from(context, start, pathname, Some(credentials))?;
        let name = match lookup.component {
            LastComponent::Name(name) => name,
            LastComponent::Dot | LastComponent::DotDot | LastComponent::Root => {
                return Err(FsError::Exists);
            }
        };

        // Step 1: 获取父目录写锁，检查重名、尾随斜杠与父目录写权限。
        let parent_inode = lookup.parent.inode()?.clone();
        let directory = parent_inode.directory_write()?;
        let parent_metadata = directory.metadata_snapshot();
        if directory.lookup(&self.inodes, name.as_slice())?.is_some() {
            check_access_metadata(
                &lookup.parent,
                &parent_metadata,
                credentials.0,
                credentials.1,
                1,
            )?;
            return Err(FsError::Exists);
        }
        if lookup.trailing_slash {
            return Err(FsError::NotFound);
        }
        check_parent_write(
            &lookup.parent,
            &parent_metadata,
            credentials.0,
            credentials.1,
        )?;

        // Step 2: 预留 dcache 槽位、分配符号链接 Inode 并准备父目录插入，随后统一发布。
        let prepared_dentry = lookup
            .parent
            .mount
            .dcache
            .try_prepare_child(&lookup.parent.dentry, name.clone())?;
        if !parent_inode.is_memory() && self.inodes.is_full() {
            lookup.parent.mount.dcache.prune_unused();
        }
        let now_ns = crate::time::realtime();
        let new_inode = self.inodes.prepare_symlink_inode(
            &parent_inode,
            &parent_metadata,
            target,
            credentials.0,
            credentials.1,
            now_ns,
        )?;
        let inode = new_inode.inode().clone();
        let prepared_dentry = prepared_dentry.bind_positive(inode.clone());
        let edit = directory.prepare_insert(
            name.as_slice(),
            inode.number(),
            InodeKind::Symlink,
            0,
            now_ns,
        )?;

        let _published_inode = new_inode.publish();
        directory.publish(edit);
        prepared_dentry.publish();
        Ok(())
    }

    /// 从父目录中删除指定的非目录名字（`unlink`），递减目标 Inode 硬链接数并从 `dcache` 摘除该目录项。
    ///
    /// 已被其他任务持有的 `Path` 或打开文件描述符（OFD）继续保活底层共享 `Inode`，直至最后一个引用释放后才回收磁盘资源。
    pub fn unlink_from(
        &self,
        context: &FsContext,
        start: &Path,
        pathname: &[u8],
        credentials: (u32, u32),
    ) -> Result<(), FsError> {
        let lookup = self.lookup_parent_from(context, start, pathname, Some(credentials))?;
        let name = match lookup.component {
            LastComponent::Name(name) => name,
            LastComponent::Dot | LastComponent::DotDot | LastComponent::Root => {
                return Err(FsError::IsDirectory);
            }
        };

        // Step 1: 获取父目录写锁并重查目标，校验尾随斜杠、父目录权限、粘滞位（Sticky Bit）、非目录类型及非挂载点约束。
        let parent_inode = lookup.parent.inode()?.clone();
        let directory = parent_inode.directory_write()?;
        let parent_metadata = directory.metadata_snapshot();
        lookup.parent.check_writable()?;
        let inode = directory
            .lookup(&self.inodes, name.as_slice())?
            .ok_or(FsError::NotFound)?;
        let metadata = inode.metadata_snapshot()?;
        if lookup.trailing_slash {
            return Err(if metadata.kind == InodeKind::Directory {
                FsError::IsDirectory
            } else {
                FsError::NotDirectory
            });
        }
        check_parent_permissions(&parent_metadata, credentials.0, credentials.1)?;
        check_sticky_directory(&parent_metadata, &metadata, credentials.0)?;
        if metadata.kind == InodeKind::Directory {
            return Err(FsError::IsDirectory);
        }
        let target = self.existing_child_locked(&lookup.parent, name.clone(), inode.clone())?;
        if self.is_mountpoint(&target) {
            return Err(FsError::Busy);
        }

        // Step 2: 获取子 Inode 元数据写锁，准备硬链接数 -1 与父目录项删除编辑，随后提交并从 dcache 取消哈希。
        let child = inode.inode_write()?;
        let child_metadata = child.metadata_snapshot();
        if child_metadata.links == 0 {
            return Err(FsError::NotFound);
        }
        let now_ns = crate::time::realtime();
        let child_edit = child.prepare_link_delta(-1, now_ns)?;
        let directory_edit = directory.prepare_remove(name.as_slice(), 0, now_ns)?;

        child.publish_metadata(child_edit);
        directory.publish(directory_edit);
        target.mount.dcache.unhash(&target.dentry);
        Ok(())
    }

    /// 在目标父目录写锁下为现有非目录路径 `source` 创建同一挂载实例内的硬链接（Hard Link）。
    pub fn link_from(
        &self,
        source: &Path,
        context: &FsContext,
        start: &Path,
        pathname: &[u8],
        credentials: (u32, u32),
    ) -> Result<(), FsError> {
        let lookup = self.lookup_parent_from(context, start, pathname, Some(credentials))?;
        let name = match lookup.component {
            LastComponent::Name(name) => name,
            LastComponent::Dot | LastComponent::DotDot | LastComponent::Root => {
                return Err(FsError::Exists);
            }
        };

        // Step 1: 锁定目标父目录，重查名字不存在并校验父目录写权限、同挂载点约束与非目录类型。
        let source_inode = source.inode()?.clone();
        let parent_inode = lookup.parent.inode()?.clone();
        let directory = parent_inode.directory_write()?;
        let parent_metadata = directory.metadata_snapshot();
        if directory.lookup(&self.inodes, name.as_slice())?.is_some() {
            check_access_metadata(
                &lookup.parent,
                &parent_metadata,
                credentials.0,
                credentials.1,
                1,
            )?;
            return Err(FsError::Exists);
        }
        if lookup.trailing_slash {
            return Err(FsError::NotFound);
        }
        check_parent_write(
            &lookup.parent,
            &parent_metadata,
            credentials.0,
            credentials.1,
        )?;
        if !Arc::ptr_eq(&source.mount, &lookup.parent.mount) {
            return Err(FsError::CrossDevice);
        }
        // Inode 类型创建后固定不变；先无锁读取类型并拒绝目录，避免 source 恰好就是父目录时重复获取写锁造成死锁。
        let source_kind = source_inode.metadata_snapshot()?.kind;
        if source_kind == InodeKind::Directory {
            return Err(FsError::NotPermitted);
        }
        // Step 2: 锁定源 Inode，预留 dcache 槽位并准备链接数 +1 与目录插入，最后统一发布。
        let source_guard = source_inode.inode_write()?;
        let source_metadata = source_guard.metadata_snapshot();
        if source_metadata.links == 0 {
            return Err(FsError::NotFound);
        }

        let prepared_dentry = lookup
            .parent
            .mount
            .dcache
            .try_prepare_child(&lookup.parent.dentry, name.clone())?;
        let now_ns = crate::time::realtime();
        let source_edit = source_guard.prepare_link_delta(1, now_ns)?;
        let directory_edit = directory.prepare_insert(
            name.as_slice(),
            source_inode.number(),
            source_metadata.kind,
            0,
            now_ns,
        )?;
        let prepared_dentry = prepared_dentry.bind_positive(source_inode.clone());

        source_guard.publish_metadata(source_edit);
        directory.publish(directory_edit);
        prepared_dentry.publish();
        Ok(())
    }

    /// 删除指定的空子目录（`rmdir`），将其自身链接数减 2、父目录链接数减 1，并从 `dcache` 摘除。
    ///
    /// 若仍有进程的 `cwd` 或 `Path` 指向该目录，内存中的零链接目录对象保持存活但拒绝后续新建子项。
    pub fn rmdir_from(
        &self,
        context: &FsContext,
        start: &Path,
        pathname: &[u8],
        credentials: (u32, u32),
    ) -> Result<(), FsError> {
        let lookup = self.lookup_parent_from(context, start, pathname, Some(credentials))?;
        let name = match lookup.component {
            LastComponent::Name(name) => name,
            LastComponent::Dot => return Err(FsError::InvalidOperation),
            LastComponent::DotDot => return Err(FsError::NotEmpty),
            LastComponent::Root => return Err(FsError::Busy),
        };

        // Step 1: 按“先父目录写锁、后子目录写锁”的顺序加锁，校验权限、粘滞位、非挂载点及子目录为空。
        let parent_inode = lookup.parent.inode()?.clone();
        let directory = parent_inode.directory_write()?;
        let parent_metadata = directory.metadata_snapshot();
        lookup.parent.check_writable()?;
        let inode = directory
            .lookup(&self.inodes, name.as_slice())?
            .ok_or(FsError::NotFound)?;
        let metadata = inode.metadata_snapshot()?;
        check_parent_permissions(&parent_metadata, credentials.0, credentials.1)?;
        check_sticky_directory(&parent_metadata, &metadata, credentials.0)?;
        if metadata.kind != InodeKind::Directory {
            return Err(FsError::NotDirectory);
        }
        let target = self.existing_child_locked(&lookup.parent, name.clone(), inode.clone())?;
        let child = inode.directory_write()?;
        if self.is_mountpoint(&target) {
            return Err(FsError::Busy);
        }
        let child_metadata = child.metadata_snapshot();
        if child_metadata.links == 0 {
            return Err(FsError::NotFound);
        }
        if !child.is_empty()? {
            return Err(FsError::NotEmpty);
        }

        // Step 2: 准备子目录链接数 -2 与父目录项移除（父链接数 -1），提交变更并从 dcache 摘除。
        let now_ns = crate::time::realtime();
        let child_edit = child.prepare_link_delta(-2, now_ns)?;
        let directory_edit = directory.prepare_remove(name.as_slice(), -1, now_ns)?;

        child.publish_metadata(child_edit);
        directory.publish(directory_edit);
        target.mount.dcache.unhash(&target.dentry);
        Ok(())
    }

    /// 在同一挂载实例内原子重命名或移动目录项（支持标准覆盖语义与 `RENAME_NOREPLACE` 不覆盖模式）。
    ///
    /// 先获取底层文件系统的 `lock_rename` 互斥锁固定整个目录树拓扑，再严格按“祖先父目录优先”顺序获取两端父目录写锁，防止并发交叉重命名死锁。
    #[expect(
        clippy::too_many_arguments,
        reason = "rename 独立解析两端起点与名字，并将 NOREPLACE 和身份快照交给同一父锁协议"
    )]
    pub fn rename_from(
        &self,
        context: &FsContext,
        old_start: &Path,
        old_pathname: &[u8],
        new_start: &Path,
        new_pathname: &[u8],
        noreplace: bool,
        credentials: (u32, u32),
    ) -> Result<(), FsError> {
        let old = self.lookup_parent_from(context, old_start, old_pathname, Some(credentials))?;
        let new = self.lookup_parent_from(context, new_start, new_pathname, Some(credentials))?;
        if !Arc::ptr_eq(&old.parent.mount, &new.parent.mount) {
            return Err(FsError::CrossDevice);
        }
        let old_name = match &old.component {
            LastComponent::Name(name) => name.clone(),
            LastComponent::Dot | LastComponent::DotDot | LastComponent::Root => {
                return Err(FsError::Busy);
            }
        };
        let new_name = match &new.component {
            LastComponent::Name(name) => name.clone(),
            LastComponent::Dot | LastComponent::DotDot | LastComponent::Root => {
                return Err(if noreplace {
                    FsError::Exists
                } else {
                    FsError::Busy
                });
            }
        };

        // Step 1: 获取卷级 rename 互斥锁固定祖先关系，并按祖先优先顺序锁定一个或两个父目录后转入 rename_locked。
        let same_parent = old.parent.ptr_eq(&new.parent);
        let _rename_guard = old.parent.filesystem().lock_rename();
        let source_first = same_parent
            || is_path_within(&old.parent, &new.parent)
            || !is_path_within(&new.parent, &old.parent);
        let old_parent_inode = old.parent.inode()?.clone();
        let new_parent_inode = new.parent.inode()?.clone();
        if same_parent {
            let directory = old_parent_inode.directory_write()?;
            return self.rename_locked(
                &old,
                &new,
                old_name,
                new_name,
                noreplace,
                credentials,
                &directory,
                None,
            );
        }
        if source_first {
            let source_directory = old_parent_inode.directory_write()?;
            let target_directory = new_parent_inode.directory_write()?;
            self.rename_locked(
                &old,
                &new,
                old_name,
                new_name,
                noreplace,
                credentials,
                &source_directory,
                Some(&target_directory),
            )
        } else {
            let target_directory = new_parent_inode.directory_write()?;
            let source_directory = old_parent_inode.directory_write()?;
            self.rename_locked(
                &old,
                &new,
                old_name,
                new_name,
                noreplace,
                credentials,
                &source_directory,
                Some(&target_directory),
            )
        }
    }

    /// 在已持有文件系统 `lock_rename` 锁及有序父目录写锁的前提下，执行重命名的权限与拓扑校验、子项加锁、编辑预准备及原子发布。
    #[expect(
        clippy::too_many_arguments,
        reason = "两端名字与已持父目录 guard 共同完成重查、可失败准备和无失败提交"
    )]
    fn rename_locked(
        &self,
        old: &LookupParent,
        new: &LookupParent,
        old_name: Arc<Vec<u8>>,
        new_name: Arc<Vec<u8>>,
        noreplace: bool,
        credentials: (u32, u32),
        source_parent: &DirectoryWriteGuard<'_>,
        target_parent: Option<&DirectoryWriteGuard<'_>>,
    ) -> Result<(), FsError> {
        // Step 1: 校验源与目标父目录的搜索权限与挂载可写状态，并在父目录锁内重新查找源与目标 Inode。
        let same_parent = target_parent.is_none();
        let destination_parent = target_parent.unwrap_or(source_parent);
        let old_metadata = source_parent.metadata_snapshot();
        let new_metadata = destination_parent.metadata_snapshot();
        check_access_metadata(
            &old.parent,
            &old_metadata,
            credentials.0,
            credentials.1,
            MAY_EXEC,
        )?;
        check_access_metadata(
            &new.parent,
            &new_metadata,
            credentials.0,
            credentials.1,
            MAY_EXEC,
        )?;
        old.parent.check_writable()?;
        new.parent.check_writable()?;

        let source_inode = source_parent
            .lookup(&self.inodes, old_name.as_slice())?
            .ok_or(FsError::NotFound)?;
        let target_inode = destination_parent.lookup(&self.inodes, new_name.as_slice())?;
        if noreplace && target_inode.is_some() {
            return Err(FsError::Exists);
        }
        let source_metadata = source_inode.metadata_snapshot()?;
        let target_metadata = target_inode
            .as_ref()
            .map(|inode| inode.metadata_snapshot())
            .transpose()?;
        if old.trailing_slash && source_metadata.kind != InodeKind::Directory {
            return Err(FsError::NotDirectory);
        }
        if new.trailing_slash && source_metadata.kind != InodeKind::Directory {
            return Err(FsError::NotDirectory);
        }
        // POSIX 语义：若源与目标解析到同一个硬链接 Inode，重命名直接作为空操作成功返回。
        if target_inode
            .as_ref()
            .is_some_and(|target| target.ptr_eq(&source_inode))
        {
            return Ok(());
        }
        check_parent_permissions(&old_metadata, credentials.0, credentials.1)?;
        check_parent_permissions(&new_metadata, credentials.0, credentials.1)?;
        check_sticky_directory(&old_metadata, &source_metadata, credentials.0)?;
        if let Some(metadata) = target_metadata.as_ref() {
            check_sticky_directory(&new_metadata, metadata, credentials.0)?;
        }
        if source_metadata.kind == InodeKind::Directory
            && target_metadata
                .as_ref()
                .is_some_and(|metadata| metadata.kind != InodeKind::Directory)
        {
            return Err(FsError::NotDirectory);
        }
        if source_metadata.kind != InodeKind::Directory
            && target_metadata
                .as_ref()
                .is_some_and(|metadata| metadata.kind == InodeKind::Directory)
        {
            return Err(FsError::IsDirectory);
        }

        // Step 2: 卷级 rename 锁与父目录写锁已冻结目录树拓扑；在尝试获取子目录写锁之前先排除祖先包含关系（如将目录移入自身子孙，或目标覆盖源的祖先），否则当子项恰为已持锁的另一端父目录时会触发同锁重入死锁。
        let source_is_directory = source_metadata.kind == InodeKind::Directory;
        let source_path =
            self.existing_child_locked(&old.parent, old_name.clone(), source_inode.clone())?;
        let target_path = target_inode
            .as_ref()
            .map(|inode| self.existing_child_locked(&new.parent, new_name.clone(), inode.clone()))
            .transpose()?;
        if source_is_directory && is_path_within(&source_path, &new.parent) {
            return Err(FsError::InvalidOperation);
        }
        if target_path
            .as_ref()
            .is_some_and(|path| is_path_within(path, &old.parent))
        {
            return Err(FsError::NotEmpty);
        }

        // Step 3: 锁定源子项与（可能存在的）被覆盖目标子项；跨父目录移动子目录时须校验源目录写权限（用于更新其内部 ".." 条目）。
        let source_directory = if source_is_directory {
            Some(source_inode.directory_write()?)
        } else {
            None
        };
        let source_directory_metadata = source_directory
            .as_ref()
            .map(|directory| directory.metadata_snapshot());
        if let Some(metadata) = source_directory_metadata.as_ref() {
            if metadata.links == 0 {
                return Err(FsError::NotFound);
            }
            if !same_parent {
                check_inode_mode_access(metadata, credentials.0, credentials.1, MAY_WRITE)?;
            }
        }
        let target_is_directory = target_metadata
            .as_ref()
            .is_some_and(|metadata| metadata.kind == InodeKind::Directory);
        let (source_file, target_directory, target_file) = if source_is_directory {
            let target_directory = match target_inode.as_ref() {
                Some(inode) if target_is_directory => Some(inode.directory_write()?),
                _ => None,
            };
            let target_file = match target_inode.as_ref() {
                Some(inode) if !target_is_directory => Some(inode.inode_write()?),
                _ => None,
            };
            (None, target_directory, target_file)
        } else if target_is_directory {
            let source_file = Some(source_inode.inode_write()?);
            let target_directory = Some(
                target_inode
                    .as_ref()
                    .expect("directory target has no canonical inode")
                    .directory_write()?,
            );
            (source_file, target_directory, None)
        } else {
            match target_inode.as_ref() {
                Some(target) => {
                    // 父目录写锁已按拓扑偏序持有；两个同卷非目录子 Inode 严格按 Inode 编号升序加锁以避免 ABBA 死锁。
                    if source_inode.number() < target.number() {
                        let source = Some(source_inode.inode_write()?);
                        let destination = Some(target.inode_write()?);
                        (source, None, destination)
                    } else {
                        let destination = Some(target.inode_write()?);
                        let source = Some(source_inode.inode_write()?);
                        (source, None, destination)
                    }
                }
                None => (Some(source_inode.inode_write()?), None, None),
            }
        };

        // Step 4: 二次确认源与目标均未被并发断链、均非活跃挂载点，且被覆盖的目标目录（若有）必须为空。
        let source_current = match source_directory_metadata {
            Some(metadata) => metadata,
            None => source_file
                .as_ref()
                .expect("non-directory source has no inode write guard")
                .metadata_snapshot(),
        };
        if !source_is_directory && source_current.links == 0 {
            return Err(FsError::NotFound);
        }
        let target_current = target_inode
            .as_ref()
            .map(|_| match target_directory.as_ref() {
                Some(directory) => directory.metadata_snapshot(),
                None => target_file
                    .as_ref()
                    .expect("non-directory target has no inode write guard")
                    .metadata_snapshot(),
            });
        if target_current
            .as_ref()
            .is_some_and(|metadata| metadata.links == 0)
        {
            return Err(FsError::NotFound);
        }
        if self.is_mountpoint(&source_path)
            || target_path
                .as_ref()
                .is_some_and(|path| self.is_mountpoint(path))
        {
            return Err(FsError::Busy);
        }
        if let Some(directory) = target_directory.as_ref() {
            if !directory.is_empty()? {
                return Err(FsError::NotEmpty);
            }
        }

        // Step 5: 预先构造源 ctime/dotdot 更新、被覆盖目标链接扣减以及源/目标父目录项编辑（所有可能失败的分配与校验在此完成），确保后续提交不会中途失败。
        let now_ns = crate::time::realtime();
        let source_metadata_edit = if source_is_directory && !same_parent {
            None
        } else if let Some(directory) = source_directory.as_ref() {
            Some(directory.prepare_ctime(now_ns)?)
        } else {
            Some(
                source_file
                    .as_ref()
                    .expect("non-directory source has no inode write guard")
                    .prepare_ctime(now_ns)?,
            )
        };
        let dotdot_edit = if source_is_directory && !same_parent {
            let parent_number = new.parent.inode()?.number();
            Some(
                source_directory
                    .as_ref()
                    .expect("moved directory has no directory guard")
                    .prepare_dotdot(parent_number, now_ns)?,
            )
        } else {
            None
        };
        let target_metadata_edit = match target_current.as_ref() {
            Some(metadata) if metadata.kind == InodeKind::Directory => Some(
                target_directory
                    .as_ref()
                    .expect("directory target has no directory guard")
                    .prepare_link_delta(-2, now_ns)?,
            ),
            Some(_) => Some(
                target_file
                    .as_ref()
                    .expect("non-directory target has no inode guard")
                    .prepare_link_delta(-1, now_ns)?,
            ),
            None => None,
        };
        let target_identity = target_inode
            .as_ref()
            .zip(target_current.as_ref())
            .map(|(inode, metadata)| (inode.number(), metadata.kind));
        let old_parent_links = if source_is_directory && !same_parent {
            -1
        } else {
            0
        };
        let (source_parent_edit, target_parent_edit) = if same_parent {
            let parent_links = if source_is_directory && target_is_directory {
                -1
            } else {
                0
            };
            (
                source_parent.prepare_move(
                    old_name.as_slice(),
                    new_name.as_slice(),
                    source_inode.number(),
                    source_metadata.kind,
                    target_identity,
                    parent_links,
                    now_ns,
                )?,
                None,
            )
        } else {
            let old_edit =
                source_parent.prepare_remove(old_name.as_slice(), old_parent_links, now_ns)?;
            let new_edit = match target_identity {
                Some(_) => destination_parent.prepare_replace(
                    new_name.as_slice(),
                    source_inode.number(),
                    source_metadata.kind,
                    0,
                    now_ns,
                )?,
                None => destination_parent.prepare_insert(
                    new_name.as_slice(),
                    source_inode.number(),
                    source_metadata.kind,
                    if source_is_directory { 1 } else { 0 },
                    now_ns,
                )?,
            };
            (old_edit, Some(new_edit))
        };

        // Step 6: 依次提交发布源与目标元数据、子目录 ".."、父目录项变更，最后同步迁移 dcache 拓扑。
        if let Some(edit) = source_metadata_edit {
            if let Some(directory) = source_directory.as_ref() {
                directory.publish_metadata(edit);
            } else {
                source_file
                    .as_ref()
                    .expect("non-directory source has no inode write guard")
                    .publish_metadata(edit);
            }
        }
        if let Some(edit) = target_metadata_edit {
            if let Some(directory) = target_directory.as_ref() {
                directory.publish_metadata(edit);
            } else {
                target_file
                    .as_ref()
                    .expect("non-directory target has no inode write guard")
                    .publish_metadata(edit);
            }
        }
        if let Some(edit) = dotdot_edit {
            source_directory
                .as_ref()
                .expect("moved directory has no directory guard")
                .publish(edit);
        }
        source_parent.publish(source_parent_edit);
        if let Some(edit) = target_parent_edit {
            destination_parent.publish(edit);
        }

        source_path.mount.dcache.move_entry(
            &source_path.dentry,
            target_path.as_ref().map(|path| &path.dentry),
            &new.parent.dentry,
            new_name,
        );
        Ok(())
    }

    /// 在现有目录 `covered` 上挂载一个具体文件系统实例 `filesystem`。
    ///
    /// 目录项命名空间分配与 Inode 锁获取均在 `mounts` 自旋锁外执行；最后在持有挂载点目录睡眠读锁的同时获取 `mounts` 自旋锁完成二次冲突检查与登记。
    pub fn mount(
        &self,
        covered: &Path,
        filesystem: Filesystem,
        mode: AccessMode,
    ) -> Result<(), FsError> {
        // Step 1: 校验覆盖点为目录、挂载读写模式不宽于后端自身模式，并做一次快速挂载表冲突预检。
        require_directory(covered)?;
        if mode == AccessMode::ReadWrite && filesystem.mode() != AccessMode::ReadWrite {
            return Err(FsError::ReadOnly);
        }
        {
            let mounts = self.mounts.lock();
            self.check_mount(&mounts, covered, &filesystem)?;
        }

        // Step 2: 在挂载表自旋锁外初始化新挂载的 dcache/根 Dentry 并分配 Mount 实例。
        let (dcache, root) = filesystem.dentry_namespace(&self.inodes, self.dcache.capacity())?;
        let candidate = Arc::try_new(Mount {
            filesystem,
            mode,
            dcache,
            root,
            covered: Some(covered.clone()),
        })
        .map_err(|_| FsError::NoMemory)?;

        // Step 3: 持有被覆盖目录的睡眠读锁确认其仍处于已链接和已哈希状态（与 rmdir/rename 的子目录写锁互斥），随后加挂载表自旋锁完成最终登记。
        let inode = covered.inode()?;
        let directory = inode.directory_read()?;
        if directory.metadata_snapshot().links == 0
            || (!covered.dentry.ptr_eq(&covered.mount.root) && !covered.dentry.is_hashed())
        {
            return Err(FsError::NotFound);
        }
        // 目录睡眠读锁与并发 rmdir/rename 所需的子目录写锁互斥，确保登记瞬间挂载点目录不会被并发删除或移走。
        let mut mounts = self.mounts.lock();
        self.check_mount(&mounts, covered, &candidate.filesystem)?;
        mounts.push(candidate);
        Ok(())
    }

    /// 按被覆盖的父目录位置 `covered` 查找并卸载其上的子挂载（供 `ktest` 直接按覆盖点卸载）。
    #[cfg(feature = "ktest")]
    pub fn unmount(&self, covered: &Path) -> Result<(), FsError> {
        let mount = {
            let mounts = self.mounts.lock();
            let position = unmount_position(&mounts, covered, None)?;
            mounts[position].clone()
        };
        self.retire_mount(mount)
    }

    /// 消费调用方解析得到的子挂载根目录 `path` 并执行卸载（供系统调用 `umount2` 使用）。
    ///
    /// 先释放 `path.dentry` 只保留唯一的 `Arc<Mount>` 句柄作为 `held` 引用基数，校验无其他活跃 `Path` 或下级子挂载后执行同步与摘除。
    pub fn unmount_path(&self, path: Path) -> Result<(), FsError> {
        if !path.dentry.ptr_eq(&path.mount.root) {
            return Err(FsError::InvalidOperation);
        }
        let Path { mount, dentry } = path;
        drop(dentry);
        let covered = mount.covered.as_ref().ok_or(FsError::InvalidOperation)?;
        {
            let mounts = self.mounts.lock();
            unmount_position(&mounts, covered, Some(&mount))?;
        }
        self.retire_mount(mount)
    }

    /// 同步待卸载挂载的底层文件系统，二次确认无并发新增引用或子挂载后从挂载表移除，并在锁外回收空闲缓存与卷绑定。
    fn retire_mount(&self, mount: Arc<Mount>) -> Result<(), FsError> {
        // Step 1: 在挂载表自旋锁外刷回底层文件系统的全部脏数据与元数据。
        let covered = mount.covered.as_ref().ok_or(FsError::InvalidOperation)?;
        mount.filesystem.sync()?;
        // Step 2: 重新获取挂载表自旋锁，复核同一 Mount 实例仍无其他活跃引用或子挂载后将其摘除。
        let victim = {
            let mut mounts = self.mounts.lock();
            let position = unmount_position(&mounts, covered, Some(&mount))?;
            mounts.swap_remove(position)
        };

        // Step 3: 在自旋锁外释放 Mount 强引用，并依次清理 dcache、全局 InodeCache 及 ext4 卷残留缓存。
        let dcache = victim.dcache.clone();
        let filesystem = victim.filesystem.clone();
        drop(victim);
        drop(mount);
        dcache.prune_unused();
        self.inodes.prune_unused();
        if let Filesystem::Ext4(volume) = filesystem {
            volume.prune_unused();
            self.inodes.retire_volume(&volume);
        }
        Ok(())
    }

    /// 同步当前挂载表中所有已挂载文件系统的脏页、元数据与块设备缓存（对应 `sync` 系统调用）。
    ///
    /// 在挂载表自旋锁外按后端实例地址去重并逐一调用 `sync()`；若途中某个卷同步出错，仍会继续同步其余卷并返回遇到的第一个错误。
    pub fn sync(&self) -> Result<(), FsError> {
        let mut filesystems = Vec::new();
        filesystems
            .try_reserve_exact(self.mount_capacity)
            .map_err(|_| FsError::NoMemory)?;
        {
            let mounts = self.mounts.lock();
            filesystems.extend(mounts.iter().map(|mount| mount.filesystem.clone()));
        }
        filesystems.sort_unstable_by_key(Filesystem::identity_address);
        filesystems.dedup_by(|a, b| a.ptr_eq(b));

        let mut result = Ok(());
        for filesystem in &filesystems {
            if let Err(error) = filesystem.sync() {
                if result.is_ok() {
                    result = Err(error);
                }
            }
        }
        result
    }

    /// 主动收缩所有活跃挂载的空闲 `dcache`、磁盘卷元数据缓存与全局 `InodeCache`，返回累计回收的对象总数。
    ///
    /// 内存文件系统（`tmpfs`/`devfs`/`proc`）的权威目录项与文件内容驻留在其自身结构中，不属于可丢弃缓存，不会被此操作清除。
    pub fn prune_unused(&self) -> usize {
        let count = self.mounts.lock().len();
        let mut removed = 0;
        for position in 0..count {
            let mount = {
                let mounts = self.mounts.lock();
                mounts.get(position).cloned()
            };
            if let Some(mount) = mount {
                removed += mount.dcache.prune_unused();
                if let Filesystem::Ext4(volume) = &mount.filesystem {
                    removed += volume.prune_unused();
                }
            }
        }
        removed + self.inodes.prune_unused()
    }

    /// 生成相对于根边界 `root` 可见的 `/proc/mounts` 六列格式挂载表文本。
    ///
    /// 位于 `root` 子树之外的挂载点会被自动跳过；所有路径反解与字符串拼装均在 `mounts` 自旋锁外执行。
    pub fn mounts_text(&self, root: &Path) -> Result<Vec<u8>, FsError> {
        let mut snapshot = Vec::new();
        snapshot
            .try_reserve_exact(self.mount_capacity)
            .map_err(|_| FsError::NoMemory)?;
        {
            let mounts = self.mounts.lock();
            snapshot.extend(mounts.iter().cloned());
        }

        let mut text = Vec::new();
        for mount in &snapshot {
            let path = Path {
                mount: mount.clone(),
                dentry: mount.root.clone(),
            };
            let pathname = match path_bytes(&path, root) {
                Ok(pathname) => pathname,
                Err(FsError::NotFound) => continue,
                Err(error) => return Err(error),
            };
            let kind: &[u8] = match &mount.filesystem {
                Filesystem::Ext4(_) => b"ext4",
                Filesystem::Memory(filesystem) => match filesystem.kind() {
                    MemoryFsKind::Tmpfs => b"tmpfs",
                    MemoryFsKind::Devfs => b"devfs",
                    MemoryFsKind::Procfs => b"proc",
                },
            };
            append_mount_bytes(&mut text, kind)?;
            append_mount_bytes(&mut text, b":")?;
            append_mount_device(&mut text, mount.filesystem.device())?;
            append_mount_bytes(&mut text, b" ")?;
            append_mount_component(&mut text, &pathname)?;
            append_mount_bytes(&mut text, b" ")?;
            append_mount_bytes(&mut text, kind)?;
            let writable = mount.mode == AccessMode::ReadWrite
                && mount.filesystem.mode() == AccessMode::ReadWrite;
            append_mount_bytes(
                &mut text,
                if writable { b" rw 0 0\n" } else { b" ro 0 0\n" },
            )?;
        }
        Ok(text)
    }

    /// 在已持有 `mounts` 自旋锁的前提下，校验被覆盖目录所属父挂载有效、同一覆盖点未被重复挂载、同一物理块区间未被另一个 `Ext4Volume` 实例占用，且挂载总数未达上限。
    fn check_mount(
        &self,
        mounts: &[Arc<Mount>],
        covered: &Path,
        filesystem: &Filesystem,
    ) -> Result<(), FsError> {
        if !contains_mount(mounts, &covered.mount) {
            return Err(FsError::InvalidOperation);
        }
        let disk_conflict = mounts
            .iter()
            .any(|mount| match (&mount.filesystem, filesystem) {
                (Filesystem::Ext4(mounted), Filesystem::Ext4(candidate)) => {
                    !Arc::ptr_eq(mounted, candidate) && mounted.view().same_range(candidate.view())
                }
                _ => false,
            });
        if find_mount(mounts, &covered.mount, &covered.dentry).is_some() || disk_conflict {
            return Err(FsError::Busy);
        }
        if self.mount_capacity <= mounts.len() {
            return Err(FsError::NoMemory);
        }
        Ok(())
    }

    /// 在单次 `mounts` 自旋锁临界区内沿着挂载覆盖关系一直向下穿越到最顶层子挂载的根目录；被替换的旧 `Path` 留在锁外析构。
    fn follow_mounts(&self, path: Path) -> Path {
        let crossed = {
            let mounts = self.mounts.lock();
            let mut mount = &path.mount;
            let mut dentry = &path.dentry;
            while let Some(child) = find_mount(&mounts, mount, dentry) {
                mount = child;
                dentry = &child.root;
            }
            if Arc::ptr_eq(mount, &path.mount) {
                None
            } else {
                Some(Path {
                    mount: mount.clone(),
                    dentry: dentry.clone(),
                })
            }
        };
        crossed.unwrap_or(path)
    }
}

/// 向挂载表输出缓冲区追加字节切片，预先检查长度溢出与内存容量，避免内存不足时留下半截行数据。
fn append_mount_bytes(text: &mut Vec<u8>, bytes: &[u8]) -> Result<(), FsError> {
    text.len()
        .checked_add(bytes.len())
        .ok_or(FsError::FileTooLarge)?;
    text.try_reserve(bytes.len())
        .map_err(|_| FsError::NoMemory)?;
    text.extend_from_slice(bytes);
    Ok(())
}

/// 使用栈上固定缓冲区将文件系统内部 64 位设备标识格式化为十进制 ASCII 数字并追加到挂载表文本中（该标识仅区分文件系统实例，并非字符/块特殊文件的 `rdev` 号）。
fn append_mount_device(text: &mut Vec<u8>, mut device: u64) -> Result<(), FsError> {
    let mut digits = [0; 20];
    let mut start = digits.len();
    loop {
        start -= 1;
        digits[start] = b'0' + (device % 10) as u8;
        device /= 10;
        if device == 0 {
            return append_mount_bytes(text, &digits[start..]);
        }
    }
}

/// 按 `/proc/mounts` 规范将路径中的空白字符、`'\'` 与 `'#'` 转义为 `\ooo` 四字节八进制序列后追加到输出缓冲。
fn append_mount_component(text: &mut Vec<u8>, name: &[u8]) -> Result<(), FsError> {
    let mut start = 0;
    for (position, &byte) in name.iter().enumerate() {
        if byte.is_ascii_whitespace() || byte == b'\\' || byte == b'#' {
            append_mount_bytes(text, &name[start..position])?;
            let escaped = [
                b'\\',
                b'0' + (byte >> 6),
                b'0' + ((byte >> 3) & 7),
                b'0' + (byte & 7),
            ];
            append_mount_bytes(text, &escaped)?;
            start = position + 1;
        }
    }
    append_mount_bytes(text, &name[start..])
}

/// 在跨挂载拓扑稳定的前提下，反向拼装 `path` 相对于祖先边界 `root` 的规范绝对路径字节串（不含末尾 NUL）。
///
/// 先沿不可变的 `Mount::covered` 链收集沿途涉及的全部 `Filesystem` 实例，按底层 `Arc` 地址升序依次获取 `lock_rename` 睡眠互斥锁以冻结所有跨挂载目录父子关系，再从缓冲末尾向前填充路径组件。
fn path_bytes(path: &Path, root: &Path) -> Result<Vec<u8>, FsError> {
    // Step 1: 沿 covered 链向上确认 root.mount 可达，收集沿途文件系统并按实例地址升序去重。
    let mut count = 1;
    let mut mount = &path.mount;
    while !Arc::ptr_eq(mount, &root.mount) {
        mount = &mount.covered.as_ref().ok_or(FsError::NotFound)?.mount;
        count += 1;
    }

    let mut filesystems = Vec::new();
    filesystems
        .try_reserve_exact(count)
        .map_err(|_| FsError::NoMemory)?;
    mount = &path.mount;
    loop {
        filesystems.push(mount.filesystem.clone());
        if Arc::ptr_eq(mount, &root.mount) {
            break;
        }
        mount = &mount.covered.as_ref().ok_or(FsError::NotFound)?.mount;
    }
    filesystems.sort_unstable_by_key(Filesystem::identity_address);
    filesystems.dedup_by(|a, b| a.ptr_eq(b));

    // Step 2: 预分配锁守卫列表与 PATH_MAX（4096 字节）输出缓冲，按全局一致顺序获取所有 lock_rename。
    let mut guards = Vec::new();
    guards
        .try_reserve_exact(filesystems.len())
        .map_err(|_| FsError::NoMemory)?;
    let mut bytes = Vec::new();
    bytes
        .try_reserve_exact(PATH_MAX)
        .map_err(|_| FsError::NoMemory)?;
    bytes.resize(PATH_MAX, 0);

    for filesystem in &filesystems {
        guards.push(filesystem.lock_rename());
    }
    let result = collect_path_bytes(path, root, &mut bytes);
    drop(guards);

    // Step 3: 释放所有 rename 锁后，将缓冲末尾拼好的路径前移至起始位置并截断多余尾部空间。
    let start = result?;
    let length = PATH_MAX - 1 - start;
    bytes.copy_within(start..PATH_MAX - 1, 0);
    bytes.truncate(length);
    Ok(bytes)
}

/// 在已持有沿途全部 `lock_rename` 的前提下，从固定缓冲区 `bytes` 的倒数第二个字节开始由右向左写入各层目录名与 `'/'`。
///
/// 保留最后一个字节额度给上层调用方追加 C 字符串结尾 `'\0'`；若途中遇到已从 `dcache` 摘除（unhashed）的目录或未能到达 `root`，则返回 `FsError::NotFound`。
fn collect_path_bytes(path: &Path, root: &Path, bytes: &mut [u8]) -> Result<usize, FsError> {
    let mut current = path.clone();
    let mut start = bytes.len() - 1;
    while !current.ptr_eq(root) {
        if current.dentry.ptr_eq(&current.mount.root) {
            current = current.mount.covered.clone().ok_or(FsError::NotFound)?;
            continue;
        }
        if !current.dentry.is_hashed() {
            return Err(FsError::NotFound);
        }
        let (parent, name) = current.dentry.topology_snapshot();
        let required = name.len() + 1;
        if start < required {
            return Err(FsError::NameTooLong);
        }
        start -= name.len();
        bytes[start..start + name.len()].copy_from_slice(&name);
        start -= 1;
        bytes[start] = b'/';
        current.dentry = parent.ok_or(FsError::NotFound)?;
    }
    if start == bytes.len() - 1 {
        start -= 1;
        bytes[start] = b'/';
    }
    Ok(start)
}

/// 校验给定 `path` 绑定的底层 `Inode` 确实是目录类型（`InodeKind::Directory`），否则返回 `FsError::NotDirectory`。
fn require_directory(path: &Path) -> Result<(), FsError> {
    if path.inode()?.metadata_snapshot()?.kind == InodeKind::Directory {
        Ok(())
    } else {
        Err(FsError::NotDirectory)
    }
}

/// 结合路径挂载写权限与 `Inode` 元数据快照校验访问权限。
///
/// 对常规文件、目录及符号链接请求 `MAY_WRITE` 时，要求所在挂载与底层文件系统均为读写模式；字符设备与块设备节点的数据读写不受只读挂载限制，仅检查自身 DAC 位。
fn check_access_metadata(
    path: &Path,
    metadata: &InodeMetadata,
    uid: u32,
    gid: u32,
    mask: u16,
) -> Result<(), FsError> {
    if mask & MAY_WRITE != 0
        && matches!(
            metadata.kind,
            InodeKind::Regular | InodeKind::Directory | InodeKind::Symlink
        )
    {
        path.check_writable()?;
    }
    check_inode_mode_access(metadata, uid, gid, mask)
}

/// 基于 `InodeMetadata` 快照中的 `uid`、`gid` 与 `mode` 九位权限掩码执行 POSIX 自主访问控制（DAC）检查。
///
/// 特权用户（`uid == 0`）自动拥有读写与目录搜索权限，但对非目录检查执行权限（`MAY_EXEC`）时仍要求属主/属组/其他至少有一位设置了可执行位（`S_IXUGO`）。
fn check_inode_mode_access(
    metadata: &InodeMetadata,
    uid: u32,
    gid: u32,
    mask: u16,
) -> Result<(), FsError> {
    if mask & !(MAY_READ | MAY_WRITE | MAY_EXEC) != 0 {
        return Err(FsError::InvalidOperation);
    }
    if uid == 0 {
        if mask & MAY_EXEC != 0
            && metadata.kind != InodeKind::Directory
            && metadata.mode & S_IXUGO == 0
        {
            return Err(FsError::Permission);
        }
        return Ok(());
    }

    let shift = if uid == metadata.uid {
        OWNER_PERMISSION_SHIFT
    } else if gid == metadata.gid {
        GROUP_PERMISSION_SHIFT
    } else {
        0
    };
    if (metadata.mode >> shift) & mask == mask {
        Ok(())
    } else {
        Err(FsError::Permission)
    }
}

/// 校验在父目录 `path` 下新建目录项所需的前置条件：挂载与后端可写、父目录尚未被删除（`links > 0`）、类型为目录，且调用方凭据同时具备写与搜索权限（`MAY_WRITE | MAY_EXEC`）。
fn check_parent_write(
    path: &Path,
    metadata: &InodeMetadata,
    uid: u32,
    gid: u32,
) -> Result<(), FsError> {
    path.check_writable()?;
    if metadata.links == 0 {
        return Err(FsError::NotFound);
    }
    if metadata.kind != InodeKind::Directory {
        return Err(FsError::NotDirectory);
    }
    check_inode_mode_access(metadata, uid, gid, MAY_WRITE | MAY_EXEC)
}

/// 校验在已确认挂载可写的父目录中删除或重命名子项所需的父目录状态与 DAC 权限（`links > 0`、目录类型、`MAY_WRITE | MAY_EXEC`）。
fn check_parent_permissions(metadata: &InodeMetadata, uid: u32, gid: u32) -> Result<(), FsError> {
    if metadata.links == 0 {
        return Err(FsError::NotFound);
    }
    if metadata.kind != InodeKind::Directory {
        return Err(FsError::NotDirectory);
    }
    check_inode_mode_access(metadata, uid, gid, MAY_WRITE | MAY_EXEC)
}

/// 校验目录粘滞位（Sticky Bit，`S_ISVTX`，如 `/tmp`）删除/重命名限制：当父目录带有 `S_ISVTX` 时，仅允许 `root`（`uid == 0`）、父目录属主或目标条目属主移除该条目。
fn check_sticky_directory(
    parent: &InodeMetadata,
    target: &InodeMetadata,
    uid: u32,
) -> Result<(), FsError> {
    if parent.mode & S_ISVTX != 0 && uid != 0 && uid != parent.uid && uid != target.uid {
        return Err(FsError::NotPermitted);
    }
    Ok(())
}

/// 沿目录项父指针与挂载 `covered` 链向上追溯，判断 `path` 是否等于 `ancestor` 或位于 `ancestor` 的子树内部（用于防止 `rename` 将目录移入其自身后代）。
fn is_path_within(ancestor: &Path, path: &Path) -> bool {
    let mut current = path.clone();
    loop {
        if current.ptr_eq(ancestor) {
            return true;
        }
        let Some(parent) = current.topology_parent() else {
            return false;
        };
        current = parent;
    }
}

/// 校验单个路径组件名称合法（非空、长度不超过 `NAME_MAX`、不含 `'\0'` 与 `'/'`），并将其拷贝封装为引用计数的 `Arc<Vec<u8>>`。
fn try_raw_name(name: &[u8]) -> Result<Arc<Vec<u8>>, FsError> {
    if NAME_MAX < name.len() {
        return Err(FsError::NameTooLong);
    }
    if name.is_empty() || name.contains(&0) || name.contains(&b'/') {
        return Err(FsError::InvalidName);
    }

    let mut bytes = Vec::new();
    bytes
        .try_reserve_exact(name.len())
        .map_err(|_| FsError::NoMemory)?;
    bytes.extend_from_slice(name);
    Arc::try_new(bytes).map_err(|_| FsError::NoMemory)
}

/// 在已持有 `mounts` 自旋锁的前提下定位覆盖在 `covered` 上的子挂载下标，并校验其不处于忙碌（`Busy`）状态。
///
/// `held` 表示当前卸载流程自身额外持有的那一份 `Arc<Mount>` 引用（`unmount_path` 与 `retire_mount` 为 `Some`，`unmount` 初次查找为 `None`）；
/// 若该挂载之上还挂有下级子挂载，或者其强引用计数超过“挂载表自身 1 份 + `held` 持有的 0/1 份”（说明仍有活跃的打开文件、`cwd` 或正在解析的 `Path`），则返回 `FsError::Busy`。
fn unmount_position(
    mounts: &[Arc<Mount>],
    covered: &Path,
    held: Option<&Arc<Mount>>,
) -> Result<usize, FsError> {
    if !contains_mount(mounts, &covered.mount) {
        return Err(FsError::InvalidOperation);
    }
    let position = mounts
        .iter()
        .position(|mount| {
            mount
                .covered
                .as_ref()
                .is_some_and(|path| path.ptr_eq(covered))
        })
        .ok_or(FsError::NotFound)?;
    let mount = &mounts[position];
    if held.is_some_and(|expected| !Arc::ptr_eq(expected, mount)) {
        return Err(FsError::NotFound);
    }
    let has_children = mounts.iter().any(|child| {
        child
            .covered
            .as_ref()
            .is_some_and(|path| Arc::ptr_eq(&path.mount, mount))
    });
    // 挂载表槽位自身占 1 个强引用，本次卸载显式持有的 held（若有）占 1 个；超出即代表外部仍持有活跃 Path 或子挂载。
    if has_children || Arc::strong_count(mount) != 1 + usize::from(held.is_some()) {
        return Err(FsError::Busy);
    }
    Ok(position)
}

/// 在已持有 `mounts` 自旋锁的前提下，按 `Arc<Mount>` 指针身份判断该挂载实例是否仍登记在当前挂载表中。
fn contains_mount(mounts: &[Arc<Mount>], mount: &Arc<Mount>) -> bool {
    mounts.iter().any(|entry| Arc::ptr_eq(entry, mount))
}

/// 在已持有 `mounts` 自旋锁的前提下，查找直接覆盖在 `(parent, dentry)` 目录位置上的子挂载引用（不在自旋锁内克隆或释放任何 `Path`）。
fn find_mount<'a>(
    mounts: &'a [Arc<Mount>],
    parent: &Arc<Mount>,
    dentry: &DentryRef,
) -> Option<&'a Arc<Mount>> {
    mounts.iter().find(|mount| {
        mount.covered.as_ref().is_some_and(|covered| {
            Arc::ptr_eq(&covered.mount, parent) && covered.dentry.ptr_eq(dentry)
        })
    })
}
