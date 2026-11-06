//! VFS 命名空间集成测试公共视图、目录逻辑块读取辅助与子套件注册锚点。
//!
//! # 模块职责与架构定位
//! 本模块位于 `crate::fs::ktest::user::namespace`，是用户态文件系统命名空间相关内核测试的顶层聚合模块。
//! - **上游调用者**：由 `crate::fs::ktest::user` 测试入口调用 [`anchor`]，将本模块下属的测试套件接入全局 `ktest` 运行器。
//! - **下游消费者**：包含三个子测试模块：
//!   - `directory`：验证 ext4 HTree 索引目录分裂、传统线性目录向 HTree 升级、目录游标（cookie）定位以及跨挂载点命名空间边界。
//!   - `loading`：验证 Inode 并发加载、单飞（Singleflight）合并与缓存淘汰回收。
//!   - `metadata`：验证文件与目录元数据变更、时间戳更新及写回持久化。
//!
//! # 核心设计与数据流布局
//! 本模块提取了子测试共用的命名空间环境初始化（[`NamespaceView`]）与底层目录块读取（[`directory_block`]）逻辑：
//!
//! ```text
//! WriteOwner (可读写测试夹具)
//!   │
//!   ├─► Vfs + FsContext ──► root   ("/")
//!   │                   └─► source ("/pattern"，作为硬链接与目录项测试的公共源 Inode)
//!   │
//!   └─► Ext4Volume
//!         └─► directory_block(volume, metadata, map_root, logical)
//!               ├─► 1. volume.map_block(...) ──► BlockMapping::Mapped { physical, .. }
//!               ├─► 2. volume.metadata_block(physical) ──► 获取元数据块缓存 BufferRef
//!               └─► 3. buffer.with_read(...) ──► 拷贝至栈上 [u8; PAGE_SIZE] 后立即释放借用
//! ```
//!
//! # 锁获取顺序与并发契约
//! - [`prepare_namespace_view`] 运行在可睡眠的测试线程上下文中，按 VFS 路径解析规则获取根目录与 `/pattern` 路径，调用返回时不持有任何自旋锁或睡眠锁。
//! - [`directory_block`] 在读取单个目录块时，仅在 `BufferRef::with_read` 闭包内部短暂持有块缓存读锁，数据拷贝至本地栈数组后立即释放，之后才返回调用方，避免调用方在遍历多级 HTree 节点时发生跨块缓存死锁。

use alloc::sync::Arc;

use crate::fs::ext4::{BlockMapping, Ext4Volume};
use crate::fs::ktest::user::write::WriteOwner;
use crate::fs::ktest::{checked, TestContext, TestError};
use crate::fs::{FsContext, FsError, InodeMetadata, Path, ResolveMode, Vfs};
use crate::mm;

mod directory;
mod loading;
mod metadata;

/// 命名空间测试的公共上下文视图。
///
/// 聚合了单次命名空间测试所需的 VFS 实例、路径解析上下文、常用基准路径以及底层 ext4 卷引用，
/// 避免各子测试用例重复执行挂载准备与基准路径查找。
struct NamespaceView {
    /// 当前测试挂载的可读写虚拟文件系统（`Vfs`）实例。
    vfs: Arc<Vfs>,
    /// 绑定到测试根目录与当前工作目录的文件系统解析上下文（含 `umask` 等属性）。
    context: FsContext,
    /// 虚拟文件系统的根路径（`"/"`）。
    root: Path,
    /// 预置样本文件 `"/pattern"` 的已解析路径，常作为创建硬链接或校验数据一致性的源路径。
    source: Path,
    /// 底层挂载的 ext4 卷实例，用于直接检查磁盘块映射、HTree 索引结构与元数据块缓存。
    volume: Arc<Ext4Volume>,
}

/// 初始化命名空间测试夹具并构造 [`NamespaceView`] 公共视图。
///
/// # 调用上下文
/// 在普通可睡眠的测试线程上下文中调用，会触发底层块设备 I/O 以准备测试卷与解析 `"/pattern"` 路径。
///
/// # 参数与返回值
/// - `context`：当前测试断言与诊断上下文。
/// - `fixture`：可读写文件系统测试夹具（[`WriteOwner`]），负责管理底层块设备、Inode 缓存与挂载生命周期。
/// - 成功时返回包含根路径、`/pattern` 源路径及底层 `Ext4Volume` 的 [`NamespaceView`]；若挂载准备或路径解析失败则返回 [`TestError`]。
fn prepare_namespace_view(
    context: &TestContext<'_>,
    fixture: &mut WriteOwner,
) -> Result<NamespaceView, TestError> {
    fixture.prepare_namespace(context)?;
    let vfs = fixture.user.path.vfs().clone();
    let fs_context = fixture.user.path.context().clone();
    let root = vfs.root();
    let source = checked(
        context,
        vfs.resolve(&fs_context, b"/pattern", ResolveMode::Follow),
    )?;
    Ok(NamespaceView {
        vfs,
        context: fs_context,
        root,
        source,
        volume: fixture.volume(),
    })
}

/// 根据 Inode 块映射信息读取指定逻辑块号对应的目录原始字节块。
///
/// # 调用上下文
/// 在可睡眠的测试线程上下文中调用。内部通过 [`Ext4Volume::map_block`] 将逻辑块号映射为物理块号，
/// 并通过 [`Ext4Volume::metadata_block`] 读取元数据块缓存。每个 [`BufferRef`](crate::fs::ext4::BufferRef)
/// 的读借用在当前函数返回前已释放，调用方可以安全地连续读取多个目录块而不会产生缓存借用冲突。
///
/// # 参数与返回值
/// - `volume`：目标 ext4 卷实例。
/// - `metadata`：目标目录 Inode 的元数据快照。
/// - `map_root`：目标目录 Inode 的 60 字节块映射根数据（`i_block`）。
/// - `logical`：要读取的目录逻辑块号（以文件系统块大小为单位，从 `0` 开始）。
/// - 成功时返回长度为 [`mm::PAGE_SIZE`] 的字节数组（前半部分填充实际块内容）；若该逻辑块未映射或底层 I/O 失败则返回对应的 [`FsError`]。
fn directory_block(
    volume: &Ext4Volume,
    metadata: &InodeMetadata,
    map_root: &[u8; 60],
    logical: u64,
) -> Result<[u8; mm::PAGE_SIZE], FsError> {
    let physical = match volume.map_block(metadata, map_root, logical, None)? {
        BlockMapping::Mapped { physical, .. } => physical,
        _ => return Err(FsError::Corrupt),
    };
    let buffer = volume.metadata_block(physical)?;
    let mut bytes = [0u8; mm::PAGE_SIZE];
    buffer
        .with_read(|data| bytes[..data.len()].copy_from_slice(data))
        .map_err(FsError::from)?;
    Ok(bytes)
}

/// 保持本模块下属三个子测试套件的链接存活，并返回供父模块注册的套件槽引用。
///
/// # 设计原因
/// `metadata`、`directory` 与 `loading` 三个子模块各自声明了独立的 `#[ktest::suite]`。
/// 通过 `core::hint::black_box` 引用全部三个子模块的锚点，确保链接器不会在死代码消除阶段将它们剥离；
/// 同时向父级 `user` 模块返回代表性的注册槽引用。
pub(super) fn anchor() -> &'static ::ktest::SuiteSlot {
    core::hint::black_box([
        metadata::cases::__ktest_anchor(),
        directory::cases::__ktest_anchor(),
        loading::cases::__ktest_anchor(),
    ]);
    metadata::cases::__ktest_anchor()
}
