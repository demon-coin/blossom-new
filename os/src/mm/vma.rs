//! 用户虚拟内存区域（VMA, Virtual Memory Area）与地址区间索引管理。
//!
//! # 模块职责与架构定位
//! 本模块定义用户进程地址空间中的连续虚拟内存区域（[`Vma`]）及其有序集合（[`MmapState`]）。
//! 上游由 [`crate::mm::space::Mm`] 在持有地址空间睡眠读写锁（`mmap: SleepRwLock<MmapState>`）时调用；
//! 下游关联共享匿名页对象（[`SharedAnon`]）、虚拟文件系统打开实例与索引节点（[`FileRef`] / [`InodeRef`]），
//! 以及页缓存的文件映射反向映射登记（[`FileRangeRegistration`]）。
//!
//! # 核心设计与不可变 VMA 身份
//! 已发布到 [`MmapState`] 中的 [`Vma`] 采用**不可变身份（Immutable Identity）**设计，通过 `Arc<Vma>` 共享：
//! 1. **锁外缺页准备与重查**：缺页处理（Page Fault）先在读锁下克隆 `Arc<Vma>`，释放锁后执行磁盘读取或物理页分配，
//!    最后重新获取写锁并用 `Arc::ptr_eq` 比对当前地址处的 VMA 是否仍是同一个 `Arc<Vma>` 实例。
//! 2. **写时复制式的区间变更**：拆分（`slice`）、合并（`coalesce`）、修改权限（`rewrite`）或覆盖映射（`replacement`）
//!    绝不原地修改已发布的 `Vma` 字段，而是先在候选 `Vec<Arc<Vma>>` 中构造全新的 `Arc<Vma>` 节点；
//!    若中途内存分配失败，原 `MmapState` 保持完好，满足事务性（全成功或全不改）语义。
//!
//! ```text
//! 低地址                                                                    高地址
//! +------------------+------------------+       +------------------+-------------+
//! | 普通 VMA (代码/堆) |   未映射空闲区   | ...   | 栈防护间隙 (256页) | 栈 VMA (↓) |
//! +------------------+------------------+       +------------------+-------------+
//!                                               ^                  ^
//!                                        vma.start_gap()    vma.range.start
//! ```
//!
//! # 锁获取顺序与并发契约
//! - [`MmapState`] 本身不包含锁，由所属 `Mm` 的 `mmap` 睡眠读写锁保护。
//! - 构造文件后备 `Vma` 时，[`FileRangeRegistration::try_new`] 会在内部获取该文件 `AddressSpace` 的范围自旋锁；
//!   因此调用本模块构造或销毁 `Vma` 时，**严禁持有页表自旋锁（`PteDomain`）**，必须遵守 `mmap 睡眠锁 -> 文件范围自旋锁 -> PTE 自旋锁` 的全局层级。

use alloc::sync::Arc;
use alloc::vec::Vec;
use core::ops::Range;

use crate::arch::{self, UserAccess};
use crate::cache::FileRangeRegistration;
use crate::fs::FileRef;
use crate::fs::InodeRef;
use crate::mm::anon::SharedAnon;
use crate::mm::space::{PteDomain, VmError};
use crate::mm::user_table::PagePermissions;
use crate::mm::PAGE_SIZE;

/// 向下增长的用户栈 VMA 与其低地址方向相邻可访问 VMA 之间保留的防护间隙大小（256 个物理页，4 KiB 页下为 1 MiB）。
///
/// 该间隙用于防止自动选址（`mmap` 传 `Hint`）、堆扩展（`brk`）或栈自动向下增长直接撞上相邻映射区。
const STACK_GUARD_GAP: usize = 256 * PAGE_SIZE;

/// 虚拟内存区域的后备存储类型（Backing Store）。
///
/// 文件映射直接持有完整的文件打开实例（[`FileRef`]），因此用户态关闭文件描述符（`close(fd)`）不会使已建立的映射失效；
/// 私有映射（含私有匿名与私有文件 COW 副本）实际分配的匿名物理页所有权保存在页表层的驻留页链表中。
#[derive(Clone)]
pub(crate) enum VmaBacking {
    /// 私有匿名映射（`MAP_PRIVATE | MAP_ANONYMOUS`），缺页时按需分配零初始化物理页。
    Anonymous,
    /// 共享匿名映射（`MAP_SHARED | MAP_ANONYMOUS`），由跨进程共享的匿名页容器提供物理页。
    SharedAnonymous {
        /// 共享匿名页集合对象，按页索引保管物理页。
        owner: Arc<SharedAnon>,
        /// 当前 `Vma` 起始虚拟地址（`range.start`）在 `owner` 中的起始字节偏移（页大小对齐）。
        offset: u64,
    },
    /// 文件后备映射（`MAP_SHARED` 或 `MAP_PRIVATE` 文件映射）。
    File {
        /// 建立映射时的文件打开实例，用于保持底层文件系统引用及后续 `msync` 回写。
        file: FileRef,
        /// 被映射文件的索引节点引用，用于缺页时读取页缓存（Page Cache）及校验文件大小。
        inode: InodeRef,
        /// 当前 `Vma` 起始虚拟地址（`range.start`）对应的文件内起始字节偏移（页大小对齐）。
        offset: u64,
        /// 是否为私有文件映射（`MAP_PRIVATE`）：为 `true` 时写入触发写时复制（COW），不回写底层文件；为 `false` 时为共享文件映射。
        private: bool,
    },
}

/// 一段连续用户虚拟地址空间的内存区域描述符（Virtual Memory Area）。
///
/// 一旦发布到地址空间的 `MmapState` 中，`Vma` 的所有字段均视为不可变；
/// 任何区间拆分、合并、权限修改或重映射都会创建并发布新的 `Arc<Vma>` 实例。
pub(crate) struct Vma {
    /// 虚拟地址半开区间 `[start, end)`，单位为字节，两端均按 [`PAGE_SIZE`] 对齐。
    pub(crate) range: Range<usize>,
    /// 当前生效的页面访问权限（读、写、执行）。
    pub(crate) permissions: PagePermissions,
    /// 允许通过 `mprotect` 提升到的最大页面访问权限上限（例如只读打开的文件映射禁止将最大写权限设为 `true`）。
    pub(crate) maximum: PagePermissions,
    /// 是否为允许随缺页自动向低地址方向扩展的用户栈区域（`MAP_GROWSDOWN`）。
    pub(crate) growdown: bool,
    /// 该虚拟内存区域的后备数据源。
    pub(crate) backing: VmaBacking,
    /// 当后备源为文件时，在文件 `AddressSpace` 中注册的反向映射范围句柄；销毁时自动从文件反向映射表中注销。
    pub(crate) file_range: Option<FileRangeRegistration>,
}

impl Vma {
    /// 在普通可睡眠任务上下文中构造一个非向下增长的普通 [`Vma`] 实例。
    ///
    /// 若权限不合法、超出 `maximum` 上限或内存分配失败，直接返回错误且不改变现有地址空间状态。
    pub(crate) fn try_new(
        range: Range<usize>,
        permissions: PagePermissions,
        maximum: PagePermissions,
        backing: VmaBacking,
        domain: Option<&Arc<PteDomain>>,
    ) -> Result<Arc<Self>, VmError> {
        Self::try_new_with_growth(range, permissions, maximum, backing, domain, false)
    }

    /// 构造一个允许向低地址方向自动扩展的用户栈 [`Vma`]（后备类型为私有匿名映射）。
    ///
    /// 初始仅登记已准备好的实际栈范围；运行期触发缺页向下扩展时，再结合当前线程的 `RLIMIT_STACK` 软限额动态裁决。
    pub(crate) fn try_stack(
        range: Range<usize>,
        permissions: PagePermissions,
        maximum: PagePermissions,
    ) -> Result<Arc<Self>, VmError> {
        Self::try_new_with_growth(
            range,
            permissions,
            maximum,
            VmaBacking::Anonymous,
            None,
            true,
        )
    }

    /// 内部通用构造函数：校验权限合法性与上限约束，为文件映射注册反向映射条目，并分配 `Arc<Vma>`。
    fn try_new_with_growth(
        range: Range<usize>,
        permissions: PagePermissions,
        maximum: PagePermissions,
        backing: VmaBacking,
        domain: Option<&Arc<PteDomain>>,
        growdown: bool,
    ) -> Result<Arc<Self>, VmError> {
        validate_permissions(permissions)?;
        validate_permissions(maximum)?;
        if !permissions_within(permissions, maximum) {
            return Err(VmError::Permission);
        }

        let file_range = match &backing {
            VmaBacking::File {
                inode,
                offset,
                private,
                ..
            } => Some(
                FileRangeRegistration::try_new(
                    inode.clone(),
                    Arc::clone(domain.expect("file VMA requires its PTE domain")),
                    range.clone(),
                    *offset,
                    *private,
                )
                .map_err(|error| VmError::File(error.into()))?,
            ),
            _ => None,
        };

        Arc::try_new(Self {
            range,
            permissions,
            maximum,
            growdown,
            backing,
            file_range,
        })
        .map_err(|_| VmError::NoMemory)
    }

    /// 检查当前 `Vma` 的软件权限是否允许指定的用户态访问类型（读、写或取指）。
    pub(crate) fn allows(&self, access: UserAccess) -> bool {
        allows(self.permissions, access)
    }

    /// 判断该区域是否为私有映射（包含私有匿名映射与 `MAP_PRIVATE` 文件映射）。
    ///
    /// 私有映射在 `fork` 复制地址空间后，父子进程对应的物理页必须启用写时复制（COW）保护（即使当前权限暂时只读但最大权限允许后续通过 `mprotect` 开启写权限）。
    pub(crate) fn is_private(&self) -> bool {
        match &self.backing {
            VmaBacking::Anonymous => true,
            VmaBacking::SharedAnonymous { .. } => false,
            VmaBacking::File { private, .. } => *private,
        }
    }

    /// 判断该区域是否为共享文件映射（`MAP_SHARED` 文件映射）。
    ///
    /// 共享文件映射的首次写入必须经过页缓存的 `mkwrite` 脏页准备流程；而共享匿名映射与私有写时复制映射不需要走文件脏页授权。
    pub(crate) fn is_shared_file(&self) -> bool {
        matches!(&self.backing, VmaBacking::File { private: false, .. })
    }

    /// 返回自动选址和堆扩展时需要避开的低地址起始边界（字节地址）。
    ///
    /// 若当前 `Vma` 为向下增长的用户栈，则在 `range.start` 下方额外预留 [`STACK_GUARD_GAP`] 字节的防护间隙；
    /// 显式指定固定地址的映射（`MAP_FIXED` / `MAP_FIXED_NOREPLACE`）仍只检查实际 `range`。
    fn start_gap(&self) -> usize {
        if self.growdown {
            self.range.start.saturating_sub(STACK_GUARD_GAP)
        } else {
            self.range.start
        }
    }

    /// 从当前 `Vma` 中切分出子区间 `range` 并赋予新权限 `permissions`，构造全新的 `Arc<Vma>`。
    ///
    /// 子区间保留原有的 `maximum` 权限上限与 `growdown` 属性；若后备为共享匿名或文件映射，其内部字节偏移随子区间左边界 `range.start` 的右移量同步增加。
    fn slice(
        &self,
        range: Range<usize>,
        permissions: PagePermissions,
    ) -> Result<Arc<Self>, VmError> {
        let backing = match &self.backing {
            VmaBacking::Anonymous => VmaBacking::Anonymous,
            VmaBacking::SharedAnonymous { owner, offset } => VmaBacking::SharedAnonymous {
                owner: Arc::clone(owner),
                offset: offset
                    .checked_add((range.start - self.range.start) as u64)
                    .ok_or(VmError::InvalidRange)?,
            },
            VmaBacking::File {
                file,
                inode,
                offset,
                private,
            } => VmaBacking::File {
                file: Arc::clone(file),
                inode: inode.clone(),
                offset: offset
                    .checked_add((range.start - self.range.start) as u64)
                    .ok_or(VmError::InvalidRange)?,
                private: *private,
            },
        };
        Self::try_new_with_growth(
            range,
            permissions,
            self.maximum,
            backing,
            self.file_range.as_ref().map(|range| &range.data().domain),
            self.growdown,
        )
    }

    /// 判断当前 `Vma` 与紧邻的高地址 `next` 区域是否可以无损合并为单个 `Vma`。
    ///
    /// 仅当两者首尾相接、当前权限与最大权限完全一致、`growdown` 属性相同、至少具备一项访问权限（保留 `PROT_NONE` 防护区独立边界），
    /// 且底层后备对象相同、文件或共享匿名偏移严格连续时，才允许合并。
    fn mergeable(&self, next: &Self) -> bool {
        if self.range.end != next.range.start
            || self.permissions != next.permissions
            || self.maximum != next.maximum
            || self.growdown != next.growdown
            || !(self.permissions.read || self.permissions.write || self.permissions.execute)
        {
            return false;
        }

        let length = (self.range.end - self.range.start) as u64;
        match (&self.backing, &next.backing) {
            (VmaBacking::Anonymous, VmaBacking::Anonymous) => true,
            (
                VmaBacking::SharedAnonymous { owner, offset },
                VmaBacking::SharedAnonymous {
                    owner: next_owner,
                    offset: next_offset,
                },
            ) => Arc::ptr_eq(owner, next_owner) && offset.checked_add(length) == Some(*next_offset),
            (
                VmaBacking::File {
                    file,
                    inode,
                    offset,
                    private,
                },
                VmaBacking::File {
                    file: next_file,
                    inode: next_inode,
                    offset: next_offset,
                    private: next_private,
                },
            ) => {
                private == next_private
                    && Arc::ptr_eq(file, next_file)
                    && inode.ptr_eq(next_inode)
                    && offset.checked_add(length) == Some(*next_offset)
            }
            _ => false,
        }
    }
}

/// 地址空间的虚拟内存区域集合与堆边界状态，由所属 [`crate::mm::space::Mm`] 的睡眠读写锁保护。
///
/// 硬件页表项（PTE）与驻留物理页所有权由 `PteDomain` 自旋锁独立保护，不在此结构体中直接存放。
#[derive(Default)]
pub(crate) struct MmapState {
    /// 按 `range.start` 严格升序排列且互不重叠的已发布 `Vma` 列表。
    pub(crate) vmas: Vec<Arc<Vma>>,
    /// 用户堆区的起始虚拟地址（由加载器在 `execve` 装载完 ELF 段后设定，单位：字节）。
    pub(crate) heap_start: usize,
    /// 当前用户堆顶（Program Break）的精确字节地址。
    pub(crate) heap_end: usize,
}

impl MmapState {
    /// 在有序且互不重叠的 `vmas` 列表中二分查找包含虚拟地址 `address` 的 [`Vma`]。
    pub(crate) fn find(&self, address: usize) -> Option<&Arc<Vma>> {
        let after = self.vmas.partition_point(|vma| vma.range.start <= address);
        after
            .checked_sub(1)
            .and_then(|index| self.vmas.get(index))
            .filter(|vma| address < vma.range.end)
    }

    /// 检查候选区间 `range` 是否与所有已发布的 `Vma` 均无交集（允许区间端点紧邻）。
    pub(crate) fn vacant(&self, range: &Range<usize>) -> bool {
        let index = self
            .vmas
            .partition_point(|vma| vma.range.end <= range.start);
        self.vmas
            .get(index)
            .is_none_or(|vma| range.end <= vma.range.start)
    }

    /// 检查候选区间 `range` 是否与所有已发布的 `Vma` 及其栈下方防护间隙（[`STACK_GUARD_GAP`]）均无交集。
    ///
    /// 用于非固定地址分配（`choose_address`）和 `brk` 堆扩展，防止新映射或堆增长侵占相邻用户栈下方的防护区。
    pub(crate) fn vacant_with_guard(&self, range: &Range<usize>) -> bool {
        let index = self
            .vmas
            .partition_point(|vma| vma.range.end <= range.start);
        self.vmas
            .get(index)
            .is_none_or(|vma| range.end <= vma.start_gap())
    }

    /// 验证位于未映射空洞中的缺页地址 `address` 是否允许通过紧邻高地址栈 VMA 向下扩展来满足，并构造扩展后的新栈 [`Vma`]。
    ///
    /// 在持有 `mmap` 写锁时调用，依次校验：
    /// 1. `address` 上方紧邻的 VMA 必须带有 `growdown` 标志，且其权限允许本次访问 `access`；
    /// 2. 扩展后的栈底 `base` 不得越过前一个 VMA 的末尾，且若前一个 VMA 是可访问的非栈映射，两者之间必须保留至少 [`STACK_GUARD_GAP`] 字节的防护间隙；
    /// 3. 扩展后的栈总字节数不得超过当前线程的 `RLIMIT_STACK` 软限额 `limit`。
    ///
    /// 前邻冲突或超出软限额仍返回 [`VmError::Unmapped`]（触发段错误），仅当候选 `Vma` 内存分配失败时返回 [`VmError::NoMemory`]。
    pub(crate) fn stack_growth(
        &self,
        address: usize,
        access: UserAccess,
        limit: u64,
    ) -> Result<Arc<Vma>, VmError> {
        // Step 1: 将缺页地址向下对齐到页边界，并定位其上方紧邻的候选栈 VMA。
        let base = address & !(PAGE_SIZE - 1);
        checked_range(base, PAGE_SIZE)?;
        let index = self.vmas.partition_point(|vma| vma.range.start <= address);
        let stack = self.vmas.get(index).ok_or(VmError::Unmapped)?;
        if !stack.growdown {
            return Err(VmError::Unmapped);
        }
        if !stack.allows(access) {
            return Err(VmError::Permission);
        }
        // Step 2: 校验与低地址前邻 VMA 的边界及栈防护间隙（STACK_GUARD_GAP）约束。
        if let Some(previous) = index.checked_sub(1).map(|index| &self.vmas[index]) {
            if base < previous.range.end {
                return Err(VmError::Unmapped);
            }
            if !previous.growdown
                && (previous.permissions.read
                    || previous.permissions.write
                    || previous.permissions.execute)
                && base - previous.range.end < STACK_GUARD_GAP
            {
                return Err(VmError::Unmapped);
            }
        }
        // Step 3: 校验扩展后的栈总长度是否超出当前线程的 RLIMIT_STACK 软限额，并构造新栈 VMA。
        if limit < (stack.range.end - base) as u64 {
            return Err(VmError::Unmapped);
        }

        Vma::try_stack(base..stack.range.end, stack.permissions, stack.maximum)
    }

    /// 为 `fork` 出的子进程地址空间克隆当前 VMA 列表与堆边界快照。
    ///
    /// 文件映射 VMA 需要绑定到子进程自己的页表域 `domain` 以建立独立的反向映射登记；
    /// 匿名映射与共享匿名映射不含页表域绑定，直接通过 `Arc::clone` 共享不可变 `Vma` 实例。
    pub(crate) fn snapshot(&self, domain: &Arc<PteDomain>) -> Result<Self, VmError> {
        let mut vmas = Vec::new();
        vmas.try_reserve_exact(self.vmas.len())
            .map_err(|_| VmError::NoMemory)?;
        for vma in &self.vmas {
            if vma.file_range.is_some() {
                vmas.push(Vma::try_new_with_growth(
                    vma.range.clone(),
                    vma.permissions,
                    vma.maximum,
                    vma.backing.clone(),
                    Some(domain),
                    vma.growdown,
                )?);
            } else {
                vmas.push(Arc::clone(vma));
            }
        }
        Ok(Self {
            vmas,
            heap_start: self.heap_start,
            heap_end: self.heap_end,
        })
    }

    /// 预检 `mprotect` 请求区间 `range` 是否被已映射 VMA 连续完整覆盖，且每段 VMA 的最大权限 `maximum` 均允许目标权限 `permissions`。
    ///
    /// 若区间内存在未映射空洞则返回 [`VmError::Unmapped`]，若权限超出上限则返回 [`VmError::Permission`]，不修改任何现有状态。
    pub(crate) fn check_protection(
        &self,
        range: &Range<usize>,
        permissions: PagePermissions,
    ) -> Result<(), VmError> {
        validate_permissions(permissions)?;
        let mut cursor = range.start;
        for vma in &self.vmas {
            if vma.range.end <= cursor {
                continue;
            }
            if cursor < vma.range.start {
                return Err(VmError::Unmapped);
            }
            if !permissions_within(permissions, vma.maximum) {
                return Err(VmError::Permission);
            }
            cursor = vma.range.end.min(range.end);
            if cursor == range.end {
                return Ok(());
            }
        }
        Err(VmError::Unmapped)
    }

    /// 针对目标区间 `range` 构造重写后的完整新 VMA 列表（在获取页表锁前于锁外完成全部分配）。
    ///
    /// - 当 `permissions` 为 `None` 时（如 `munmap` 或覆盖映射前清理），移除所有与 `range` 相交的子区间；
    /// - 当 `permissions` 为 `Some(perm)` 时（如 `mprotect`），将相交子区间的权限替换为 `perm`；
    /// - 相交区间两侧未被覆盖的头尾剩余部分通过 [`Vma::slice`] 拆分为独立节点，最后对全表执行相邻合并（[`coalesce`]）。
    pub(crate) fn rewrite(
        &self,
        range: &Range<usize>,
        permissions: Option<PagePermissions>,
    ) -> Result<Vec<Arc<Vma>>, VmError> {
        let mut vmas = Vec::new();
        vmas.try_reserve_exact(self.vmas.len().checked_add(2).ok_or(VmError::NoMemory)?)
            .map_err(|_| VmError::NoMemory)?;
        for vma in &self.vmas {
            if vma.range.end <= range.start || range.end <= vma.range.start {
                vmas.push(Arc::clone(vma));
                continue;
            }
            if vma.range.start < range.start {
                vmas.push(vma.slice(vma.range.start..range.start, vma.permissions)?);
            }
            let start = vma.range.start.max(range.start);
            let end = vma.range.end.min(range.end);
            if let Some(permissions) = permissions {
                vmas.push(vma.slice(start..end, permissions)?);
            }
            if range.end < vma.range.end {
                vmas.push(vma.slice(range.end..vma.range.end, vma.permissions)?);
            }
        }
        coalesce(&mut vmas)?;
        Ok(vmas)
    }

    /// 构造将区间 `range` 内的旧映射移除、并可选插入新映射 `replacement` 后的完整候选 VMA 列表。
    ///
    /// 所有的内存分配、区间拆分、相邻合并与文件偏移溢出检查均在解除旧硬件页表项之前完成。
    pub(crate) fn replacement(
        &self,
        range: &Range<usize>,
        replacement: Option<Arc<Vma>>,
    ) -> Result<Vec<Arc<Vma>>, VmError> {
        let mut vmas = self.rewrite(range, None)?;
        if let Some(replacement) = replacement {
            vmas.try_reserve(1).map_err(|_| VmError::NoMemory)?;
            let index = vmas.partition_point(|vma| vma.range.start < range.start);
            vmas.insert(index, replacement);
            coalesce(&mut vmas)?;
        }
        Ok(vmas)
    }

    /// 为未指定固定地址的 `mmap` 请求寻找一段长度为 `length` 字节的空闲虚拟地址区间起点。
    ///
    /// 优先尝试向下页对齐且不低于 `64 KiB` 的 `hint` 地址；若不可用，则从 `64 KiB` 起按低地址向高地址线性扫描第一个满足长度要求且避开栈防护间隙（[`STACK_GUARD_GAP`]）的空隙。
    pub(crate) fn choose_address(&self, hint: usize, length: usize) -> Result<usize, VmError> {
        const MINIMUM: usize = 64 * 1024;
        let hint = hint & !(PAGE_SIZE - 1);
        if MINIMUM <= hint {
            if let Ok(range) = checked_range(hint, length) {
                if self.vacant_with_guard(&range) {
                    return Ok(hint);
                }
            }
        }

        let mut cursor = MINIMUM;
        for vma in &self.vmas {
            if vma.range.end <= cursor {
                continue;
            }
            let range = checked_range(cursor, length).map_err(|_| VmError::NoMemory)?;
            if range.end <= vma.start_gap() {
                return Ok(cursor);
            }
            cursor = vma.range.end;
        }
        checked_range(cursor, length).map_err(|_| VmError::NoMemory)?;
        Ok(cursor)
    }
}

/// 在尚未发布的候选 VMA 列表中原地合并所有相邻且满足 [`Vma::mergeable`] 条件的区域。
///
/// 若合并过程中的新节点分配失败，直接返回错误，不会污染当前已生效的 `MmapState`。
fn coalesce(vmas: &mut Vec<Arc<Vma>>) -> Result<(), VmError> {
    let mut kept = 0;
    for index in 0..vmas.len() {
        if 0 < kept && vmas[kept - 1].mergeable(&vmas[index]) {
            let previous = &vmas[kept - 1];
            let merged = Vma::try_new_with_growth(
                previous.range.start..vmas[index].range.end,
                previous.permissions,
                previous.maximum,
                previous.backing.clone(),
                previous
                    .file_range
                    .as_ref()
                    .map(|range| &range.data().domain),
                previous.growdown,
            )?;
            vmas[kept - 1] = merged;
        } else {
            if kept != index {
                vmas[kept] = Arc::clone(&vmas[index]);
            }
            kept += 1;
        }
    }
    vmas.truncate(kept);
    Ok(())
}

/// 校验并构造用户虚拟地址区间 `[address, address + length)`。
///
/// 拒绝长度为 0、起始地址或长度未按 [`PAGE_SIZE`] 对齐、加法溢出或超出架构用户地址空间上限（[`arch::USER_ADDRESS_END`]）的非法范围。
pub(crate) fn checked_range(address: usize, length: usize) -> Result<Range<usize>, VmError> {
    let end = address.checked_add(length).ok_or(VmError::InvalidRange)?;
    if length == 0
        || address & (PAGE_SIZE - 1) != 0
        || length & (PAGE_SIZE - 1) != 0
        || arch::USER_ADDRESS_END < end
    {
        return Err(VmError::InvalidRange);
    }
    Ok(address..end)
}

/// 判断请求的页面权限 `request` 的每一项能力是否均不超过上限权限 `maximum`。
pub(crate) fn permissions_within(request: PagePermissions, maximum: PagePermissions) -> bool {
    (!request.read || maximum.read)
        && (!request.write || maximum.write)
        && (!request.execute || maximum.execute)
}

/// 判断页面权限 `permissions` 是否允许硬件访问类型 `access`（读、写或执行）。
pub(crate) fn allows(permissions: PagePermissions, access: UserAccess) -> bool {
    match access {
        UserAccess::Read => permissions.read,
        UserAccess::Write => permissions.write,
        UserAccess::Execute => permissions.execute,
    }
}

/// 校验目标权限组合是否能够被当前硬件架构原生表示，拒绝将不支持的权限组合静默提升为更宽的权限。
///
/// 例如在 RISC-V Sv39 架构下，硬件页表不支持“仅可写不可读（W=1, R=0）”编码，此类请求直接返回 [`VmError::InvalidRange`]。
pub(crate) fn validate_permissions(permissions: PagePermissions) -> Result<(), VmError> {
    #[cfg(target_arch = "riscv64")]
    if permissions.write && !permissions.read {
        return Err(VmError::InvalidRange);
    }
    let _ = permissions;
    Ok(())
}
