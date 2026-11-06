//! 进程文件描述符表（`FileTable`）与两阶段槽位预留机制。
//!
//! # 模块职责与架构定位
//! 本模块管理进程（或通过 `CLONE_FILES` 共享描述符表的多个线程组）中从非负整数文件描述符（`fd`）到打开文件描述（OFD，即 `FileRef`）及描述符级标志（如 `FD_CLOEXEC`）的映射：
//! - **上游调用者**：系统调用层（`openat`、`close`、`dup`/`dup3`、`fcntl`、`select` 等）、进程生命周期管理（`fork`/`clone` 复制或共享、`execve` 清理 `CLOEXEC`、进程退出释放）以及内核测试。
//! - **下游依赖**：抽象打开文件接口 `FileRef`（`Arc<dyn FileLike>`）。`FileTable` 只维护整数编号映射，文件的读写游标、打开状态标志与底层 `Path`/`Inode` 生命周期由各 `FileRef` 独立持有。
//!
//! # 核心设计与两级分页槽位布局
//! 为避免在自旋锁临界区内执行堆内存分配或大块拷贝，`Slots` 采用按 64 槽分页的目录结构，配合每页 `u64` 空闲位图实现 $O(1)$ 页内空槽查找：
//! ```text
//!   FileTable
//!   ├── users: AtomicUsize  (绑定到此表的活跃任务/线程计数，用于判断是否跨任务共享)
//!   └── slots: SpinLock<Slots>
//!                └── pages: Vec<Box<SlotPage>>
//!                             ├── [0] -> Box<SlotPage> (fd 0..63)
//!                             │            ├── slots: [Slot; 64]
//!                             │            └── free:  u64 (第 i 位为 1 表示槽 Vacant，0 表示 Reserved 或 Open)
//!                             ├── [1] -> Box<SlotPage> (fd 64..127)
//!                             └── ...
//!
//!   单个描述符槽（Slot）的三态生命周期：
//!     reserve() 占位                    publish() 填入 FileRef
//!   [ Vacant ] ──────────────> [ Reserved ] ──────────────────────> [ Open { file, flags } ]
//!       ^                           │                                       │
//!       │    FileReservation 析构   │                                       │ close() / dup3 覆盖 /
//!       +───────────────────────────+                                       │ close_on_exec()
//!       +───────────────────────────────────────────────────────────────────+
//! ```
//! - **额度解耦**：描述符上限（`RLIMIT_NOFILE`）归属于调用者所在的进程/线程组，不存储在可能跨线程组共享的 `FileTable` 内，每次分配描述符时由调用者传入 `limit`。
//! - **两阶段预留（`FileReservation`）**：打开文件可能阻塞于磁盘 I/O，因此先持自旋锁分配 `Reserved` 槽位拿到 `fd`，释放自旋锁后执行路径解析与文件构造，成功后再 `publish` 为 `Open`；若中途失败，`FileReservation::drop` 自动将槽位恢复为 `Vacant`。
//!
//! # 锁获取顺序（Lock Ordering）与并发契约
//! 1. **`FileTable::slots` 自旋锁是最短叶级映射锁**：持锁期间严禁执行堆分配（扩容所需的新 `Box<SlotPage>` 和目录 `Vec` 均在锁外预先分配）、严禁调用任何可能睡眠的 VFS/Inode 操作。
//! 2. **锁外释放 `FileRef`（严禁持锁 Drop）**：关闭或替换槽位时，只能在自旋锁临界区内将旧 `Slot` 移出，必须在释放 `slots` 自旋锁**之后**再 `drop` 旧 `Slot`。因为释放最后一个 `Arc<dyn FileLike>` 可能触发管道唤醒、epoll 反注册或底层文件系统资源回收，若在自旋锁内执行会导致死锁。
//! 3. **`try_clone` 双表锁顺序**：复制描述符表时，子表 `child` 为局部新建且尚未对外暴露，固定先锁 `child.slots` 再锁源表 `self.slots`。

use alloc::boxed::Box;
use alloc::vec::Vec;
use core::mem;
use core::sync::atomic::{AtomicUsize, Ordering};

#[cfg(feature = "ktest")]
use crate::fs::file::{O_DIRECTORY, O_NOFOLLOW};
#[cfg(feature = "ktest")]
use crate::fs::{
    open_path_with_flags, DirectoryEntry, FileIoContext, FsContext, ResolveMode, Vfs, WriteBuffer,
};
use crate::fs::{FileRef, FileStat, FsError, SeekFrom};
use crate::sync::SpinLock;

/// 单个文件描述符私有的标志位（对应 POSIX 描述符级标志，如 `FD_CLOEXEC`）。
///
/// 该标志存放在描述符表槽位上，通过 `dup`/`fork` 复制出的不同描述符各自拥有独立的 `DescriptorFlags`，不会随共享的 `FileRef` 传播。
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct DescriptorFlags {
    /// 是否在进程执行 `execve` 成功加载新程序时自动关闭该文件描述符（`FD_CLOEXEC`）。
    pub close_on_exec: bool,
}

/// 内核测试使用的只读打开选项，封装路径解析约束与新描述符初始标志。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct OpenOptions {
    /// 要求目标必须是目录（对应 `O_DIRECTORY`），否则返回 `NotDirectory`。
    pub directory: bool,
    /// 不跟随路径末端符号链接（对应 `O_NOFOLLOW`）。
    pub no_follow: bool,
    /// 新分配的描述符是否初始带有 `FD_CLOEXEC` 标志。
    pub close_on_exec: bool,
}

/// 描述符表内单个槽位的状态。
///
/// 槽位处于 `Vacant` 或 `Reserved` 时不持有任何文件对象引用；`Reserved` 仅由唯一对应的 `FileReservation` 负责完成发布或回滚。
enum Slot {
    /// 空闲槽位，可被 `reserve` 或 `dup` 分配（对应 `SlotPage::free` 位图中相应位为 `1`）。
    Vacant,
    /// 已预留但尚未绑定具体文件的占位状态（位图相应位为 `0`），对 `lookup`/`close` 仍表现为未打开。
    Reserved,
    /// 已正式发布的打开描述符，持有底层打开文件对象与描述符私有标志（位图相应位为 `0`）。
    Open {
        /// 指向底层打开文件描述（OFD）的共享强引用。
        file: FileRef,
        /// 当前描述符槽位独有的标志（如 `close_on_exec`）。
        flags: DescriptorFlags,
    },
}

/// 单个槽位页容纳的描述符槽数量（64 个，恰好对应一个 `u64` 位图的全部比特位）。
const PAGE_SLOTS: usize = u64::BITS as usize;

/// 固定包含 64 个描述符槽及对应空闲位图的分配页。
///
/// 页通过 `Box` 独立分配在堆上；目录 `Vec` 扩容时只移动 `Box` 指针，避免整块搬移已打开的 `Slot` 与 `Arc` 引用。
struct SlotPage {
    /// 当前页内的 64 个描述符槽数组，由外层 `FileTable::slots` 自旋锁保护。
    slots: [Slot; PAGE_SLOTS],
    /// 64 位空闲位图：第 `i` 位为 `1` 当且仅当 `slots[i]` 为 `Slot::Vacant`。
    free: u64,
}

/// 受 `FileTable::slots` 自旋锁保护的分页槽位集合。
struct Slots {
    #[expect(
        clippy::vec_box,
        reason = "槽页在短锁外独立准备，目录扩容只移动页拥有，避免搬动每页全部槽和 OFD"
    )]
    /// 按页索引排列的槽位页指针表，仅在扩容时追加新页。
    pages: Vec<Box<SlotPage>>,
}

impl Slots {
    /// 返回当前已分配的槽位总容量（槽位页数乘以 64）。
    ///
    /// 该值仅反映历史扩容水位，不代表任何调用方进程的 `RLIMIT_NOFILE` 软硬限额。
    fn len(&self) -> usize {
        self.pages.len() * PAGE_SLOTS
    }

    /// 返回覆盖当前所有已打开（`Slot::Open`）描述符所需的最小槽位跨度（即最高打开 `fd + 1`，全空时为 `0`）。
    ///
    /// `fork` 复制描述符表时借此确定子表初始分配大小，已关闭的历史高编号槽位或临时的 `Reserved` 槽位不会拉大子表容量。
    fn open_span(&self) -> usize {
        (0..self.len())
            .rev()
            .find(|fd| matches!(self.get(*fd), Some(Slot::Open { .. })))
            .map_or(0, |fd| fd + 1)
    }

    /// 按描述符编号借用已分配范围内的槽位引用；若 `fd` 超出当前已分配页范围则返回 `None`。
    fn get(&self, fd: usize) -> Option<&Slot> {
        self.pages
            .get(fd / PAGE_SLOTS)
            .map(|page| &page.slots[fd % PAGE_SLOTS])
    }

    /// 在持有表自旋锁的前提下替换 `fd` 处的槽位并同步更新所在页的 `free` 位图，返回被替换出的旧槽位。
    ///
    /// 调用方必须保证 `fd < self.len()`，且返回的旧 `Slot` 必须在释放表自旋锁之后再析构。
    fn replace(&mut self, fd: usize, slot: Slot) -> Slot {
        let page = &mut self.pages[fd / PAGE_SLOTS];
        let bit = 1u64 << (fd % PAGE_SLOTS);
        if matches!(slot, Slot::Vacant) {
            page.free |= bit;
        } else {
            page.free &= !bit;
        }
        mem::replace(&mut page.slots[fd % PAGE_SLOTS], slot)
    }

    /// 利用每页 `free` 位图查找满足 `minimum <= fd < limit` 的最小空闲（`Vacant`）描述符编号。
    ///
    /// 若在当前已分配页的 `[minimum, limit)` 区间内不存在空闲槽位，或 `limit <= minimum`，则返回 `None`。
    fn vacant(&self, minimum: usize, limit: usize) -> Option<usize> {
        if limit <= minimum {
            return None;
        }
        for index in minimum / PAGE_SLOTS..self.pages.len().min(limit.div_ceil(PAGE_SLOTS)) {
            let base = index * PAGE_SLOTS;
            let mut free = self.pages[index].free;
            if base < minimum {
                free &= u64::MAX << (minimum - base);
            }
            if limit - base < PAGE_SLOTS {
                free &= (1u64 << (limit - base)) - 1;
            }
            if free != 0 {
                return Some(base + free.trailing_zeros() as usize);
            }
        }
        None
    }
}

/// 进程打开文件描述符表。
///
/// 支持单进程独占或通过 `CLONE_FILES` 在多个任务之间并发共享；内部槽位通过自旋锁保护，具体文件 I/O 一律在锁外通过克隆出的 `FileRef` 执行。
pub struct FileTable {
    /// 保护分页槽位目录、各槽状态以及空闲位图的自旋锁（临界区极短、不可睡眠）。
    slots: SpinLock<Slots>,
    /// 当前绑定并运行在此描述符表上的活跃任务数量（用于 `execve` 等场景判断表是否被多个任务共享）。
    users: AtomicUsize,
}

/// 描述符槽位的单次预留守卫（RAII Guard）。
///
/// 持有对所属 `FileTable` 的借用和一个处于 `Slot::Reserved` 状态的 `fd`；调用 `publish` 会将其正式转换为 `Slot::Open`，若未调用 `publish` 即被丢弃，`Drop` 实现会自动将槽位归还为 `Slot::Vacant`。
pub struct FileReservation<'a> {
    /// 预留槽位所属的文件描述符表。
    table: &'a FileTable,
    /// 已预留占位的非负整数描述符编号。
    fd: usize,
    /// 该预留是否仍未发布（`true` 表示析构时需回滚槽位为 `Vacant`）。
    active: bool,
}

impl FileTable {
    /// 创建一张初始仅包含 1 个槽位页（64 个空闲槽）的新描述符表，初始绑定任务计数为 0。
    ///
    /// 若初始槽位页分配失败则返回 `FsError::NoMemory`。
    pub fn try_new() -> Result<Self, FsError> {
        let table = Self {
            slots: SpinLock::new(Slots { pages: Vec::new() }),
            users: AtomicUsize::new(0),
        };
        table.grow(PAGE_SLOTS)?;
        Ok(table)
    }

    /// 确保描述符表至少拥有能够容纳 `required` 个槽位的页数，不足时在自旋锁外预分配内存并扩容。
    ///
    /// 若并发线程已抢先完成扩容，锁外预分配的多余临时页会在离开循环体时安全释放。
    fn grow(&self, required: usize) -> Result<(), FsError> {
        let needed = required.div_ceil(PAGE_SLOTS);
        loop {
            // Step 1: 在自旋锁临界区内读取当前页数与目录 Vec 容量；若已满足需求则直接返回。
            let (current, capacity) = {
                let slots = self.slots.lock();
                (slots.pages.len(), slots.pages.capacity())
            };
            if needed <= current {
                return Ok(());
            }

            // Step 2: 在自旋锁外预先分配扩容后的目录缓冲区与全部新增的空 SlotPage，避免持自旋锁分配内存。
            let mut directory = Vec::new();
            if capacity < needed {
                directory
                    .try_reserve_exact(needed.max(capacity.saturating_mul(2)))
                    .map_err(|_| FsError::NoMemory)?;
            }
            let mut added = Vec::new();
            added
                .try_reserve_exact(needed - current)
                .map_err(|_| FsError::NoMemory)?;
            for _ in current..needed {
                added.push(
                    Box::try_new(SlotPage {
                        slots: [const { Slot::Vacant }; PAGE_SLOTS],
                        free: u64::MAX,
                    })
                    .map_err(|_| FsError::NoMemory)?,
                );
            }

            // Step 3: 重新获取自旋锁校验页数未发生并发变化；若仍一致则仅转移 Box 指针所有权完成扩容。
            {
                let mut slots = self.slots.lock();
                if slots.pages.len() != current {
                    continue;
                }
                if capacity < needed {
                    directory.append(&mut slots.pages);
                    mem::swap(&mut slots.pages, &mut directory);
                }
                slots.pages.append(&mut added);
            }
            return Ok(());
        }
    }

    /// 克隆当前描述符表中所有已打开（`Slot::Open`）的描述符以构造子进程独立的描述符表（用于不带 `CLONE_FILES` 的 `fork`）。
    ///
    /// 源表中处于 `Slot::Reserved` 的未完成打开槽位不会复制到子表（在子表中保持 `Vacant`）；克隆出的每个打开槽位与父表共享同一个底层 `FileRef`，但各自独立持有 `DescriptorFlags` 副本。
    pub fn try_clone(&self) -> Result<Self, FsError> {
        let child = Self::try_new()?;
        loop {
            let required = self.slots.lock().open_span();
            child.grow(required)?;

            let mut target = child.slots.lock();
            let source = self.slots.lock();
            let highest = source.open_span();
            if target.len() < highest {
                continue;
            }

            for fd in 0..highest {
                if let Some(Slot::Open { file, flags }) = source.get(fd) {
                    target.replace(
                        fd,
                        Slot::Open {
                            file: file.clone(),
                            flags: *flags,
                        },
                    );
                }
            }
            break;
        }
        Ok(child)
    }

    /// 递增绑定到此描述符表的活跃运行任务计数（在任务创建共享此表或切换绑定时调用）。
    pub(crate) fn add_user(&self) {
        self.users
            .fetch_update(Ordering::AcqRel, Ordering::Acquire, |count| {
                count.checked_add(1)
            })
            .expect("file table user count overflow");
    }

    /// 递减绑定到此描述符表的活跃运行任务计数（在任务退出或 `execve` 脱离原共享表时调用）。
    pub(crate) fn remove_user(&self) {
        let previous = self.users.fetch_sub(1, Ordering::AcqRel);
        assert!(previous != 0, "file table user count underflow");
    }

    /// 读取当前绑定到此描述符表的活跃运行任务数量（临时借用或内核诊断观察不计入此数）。
    pub(crate) fn user_count(&self) -> usize {
        self.users.load(Ordering::Acquire)
    }

    /// 返回当前已分配的描述符槽位总上限（供 `select` 等多路复用接口按当前表跨度裁剪位图扫描范围）。
    pub fn allocated_slots(&self) -> usize {
        self.slots.lock().len()
    }

    /// 在 `[0, limit)` 范围内分配最小可用描述符编号并将其置为 `Slot::Reserved`，返回对应的 `FileReservation` 守卫。
    ///
    /// 若当前已分配槽位已满但尚未达到 `limit`，会在自旋锁外按页扩容后重试；若 `[0, limit)` 全部被占用则返回 `FsError::TooManyFiles`（`EMFILE`）。
    pub fn reserve(&self, limit: usize) -> Result<FileReservation<'_>, FsError> {
        loop {
            let required = {
                let mut slots = self.slots.lock();
                if let Some(fd) = slots.vacant(0, limit) {
                    slots.replace(fd, Slot::Reserved);
                    return Ok(FileReservation {
                        table: self,
                        fd,
                        active: true,
                    });
                }
                if limit <= slots.len() {
                    return Err(FsError::TooManyFiles);
                }
                (slots.len() + PAGE_SLOTS).min(limit)
            };
            self.grow(required)?;
        }
    }

    /// 根据整数编号 `fd` 查找已打开的文件，在自旋锁临界区内克隆并返回底层 `FileRef` 强引用。
    ///
    /// 若 `fd` 越界、处于 `Vacant` 或仍处于未发布的 `Reserved` 状态，均返回 `FsError::BadFd`（`EBADF`）。
    /// 调用方拿到 `FileRef` 后即可在表锁之外安全执行阻塞 I/O，即使其他线程随后关闭或复用了同一个整数 `fd`，也不会影响本次持有的文件对象。
    pub fn lookup(&self, fd: usize) -> Result<FileRef, FsError> {
        let slots = self.slots.lock();
        match slots.get(fd) {
            Some(Slot::Open { file, .. }) => Ok(file.clone()),
            _ => Err(FsError::BadFd),
        }
    }

    /// 读取已打开描述符 `fd` 的私有标志（如 `FD_CLOEXEC`）；空槽或未发布的预留槽返回 `FsError::BadFd`。
    pub fn flags(&self, fd: usize) -> Result<DescriptorFlags, FsError> {
        let slots = self.slots.lock();
        match slots.get(fd) {
            Some(Slot::Open { flags, .. }) => Ok(*flags),
            _ => Err(FsError::BadFd),
        }
    }

    /// 原地更新已打开描述符 `fd` 的私有标志（对应 `fcntl(F_SETFD)`），不影响底层共享的 `FileRef`。
    pub fn set_flags(&self, fd: usize, replacement: DescriptorFlags) -> Result<(), FsError> {
        let mut slots = self.slots.lock();
        match slots
            .pages
            .get_mut(fd / PAGE_SLOTS)
            .map(|page| &mut page.slots[fd % PAGE_SLOTS])
        {
            Some(Slot::Open { flags, .. }) => {
                *flags = replacement;
                Ok(())
            }
            _ => Err(FsError::BadFd),
        }
    }

    /// 在普通可睡眠任务上下文中解析路径、打开文件并安装到此描述符表（供 `ktest` 内核测试便捷调用）。
    ///
    /// 先预留最小可用槽位，在表自旋锁外完成路径解析与打开文件构造；若中途出错，预留守卫析构时自动回收槽位。
    #[cfg(feature = "ktest")]
    pub fn open(
        &self,
        vfs: &Vfs,
        context: &FsContext,
        pathname: &[u8],
        options: OpenOptions,
        limit: usize,
    ) -> Result<usize, FsError> {
        let reservation = self.reserve(limit)?;
        let mode = if options.no_follow {
            ResolveMode::NoFollow
        } else {
            ResolveMode::Follow
        };
        let path = vfs.resolve(context, pathname, mode)?;
        let flags = if options.directory { O_DIRECTORY } else { 0 }
            | if options.no_follow { O_NOFOLLOW } else { 0 };
        let file = open_path_with_flags(vfs, path, flags, FileIoContext::Kernel)?;
        Ok(reservation.publish(
            file,
            DescriptorFlags {
                close_on_exec: options.close_on_exec,
            },
        ))
    }

    /// 查找 `fd` 并在表自旋锁外顺序读取数据到内核缓冲区 `buffer`，推进该文件的共享偏移（供 `ktest` 使用）。
    #[cfg(feature = "ktest")]
    pub fn read(&self, fd: usize, buffer: &mut [u8]) -> Result<usize, FsError> {
        let file = self.lookup(fd)?;
        file.read(buffer)
    }

    /// 查找 `fd` 并在表自旋锁外从指定字节偏移 `offset` 处读取数据，不修改该文件的共享偏移（供 `ktest` 使用）。
    #[cfg(feature = "ktest")]
    pub fn read_at(&self, fd: usize, offset: u64, buffer: &mut [u8]) -> Result<usize, FsError> {
        let file = self.lookup(fd)?;
        file.read_at(offset, buffer)
    }

    /// 查找 `fd` 并在表自旋锁外调整其底层打开文件对象的共享读写偏移（对应 `lseek`）。
    pub fn seek(&self, fd: usize, from: SeekFrom) -> Result<u64, FsError> {
        let file = self.lookup(fd)?;
        file.seek(from)
    }

    /// 查找 `fd` 并在表自旋锁外获取其底层 `Inode` 的元数据属性快照（对应 `fstat`）。
    pub fn stat(&self, fd: usize) -> Result<FileStat, FsError> {
        let file = self.lookup(fd)?;
        file.stat()
    }

    /// 查找目录描述符 `fd` 并在表自旋锁外读取下一条目录项（供 `ktest` 使用）。
    #[cfg(feature = "ktest")]
    pub fn readdir(&self, fd: usize) -> Result<Option<DirectoryEntry>, FsError> {
        let file = self.lookup(fd)?;
        file.readdir()
    }

    /// 查找 `fd` 并在表自旋锁外将内核切片 `buffer` 顺序写入文件（供 `ktest` 使用）。
    #[cfg(feature = "ktest")]
    pub fn write(&self, fd: usize, buffer: &[u8]) -> Result<usize, FsError> {
        self.write_from(fd, &mut WriteBuffer::kernel(buffer))
    }

    /// 查找 `fd` 并在表自旋锁外将内核切片 `buffer` 写入指定偏移 `offset`，不修改共享偏移（供 `ktest` 使用）。
    #[cfg(feature = "ktest")]
    pub fn write_at(&self, fd: usize, offset: u64, buffer: &[u8]) -> Result<usize, FsError> {
        self.write_at_from(fd, offset, &mut WriteBuffer::kernel(buffer))
    }

    /// 查找 `fd` 并在表自旋锁外从通用写入源 `source` 执行顺序写（供 `ktest` 使用）。
    #[cfg(feature = "ktest")]
    pub fn write_from(&self, fd: usize, source: &mut WriteBuffer<'_>) -> Result<usize, FsError> {
        self.lookup(fd)?.write_from(source, FileIoContext::Kernel)
    }

    /// 查找 `fd` 并在表自旋锁外从通用写入源 `source` 向指定偏移 `offset` 执行定位写（供 `ktest` 使用）。
    #[cfg(feature = "ktest")]
    pub fn write_at_from(
        &self,
        fd: usize,
        offset: u64,
        source: &mut WriteBuffer<'_>,
    ) -> Result<usize, FsError> {
        self.lookup(fd)?
            .write_at_from(offset, source, FileIoContext::Kernel)
    }

    /// 关闭已打开的文件描述符 `fd`，使其编号立即可被重用，并在释放表自旋锁后析构移出的 `FileRef`。
    ///
    /// 若 `fd` 未打开（为 `Vacant`、`Reserved` 或越界），返回 `FsError::BadFd`。
    pub fn close(&self, fd: usize) -> Result<(), FsError> {
        let old = {
            let mut slots = self.slots.lock();
            if !matches!(slots.get(fd), Some(Slot::Open { .. })) {
                return Err(FsError::BadFd);
            }
            slots.replace(fd, Slot::Vacant)
        };
        drop(old);
        Ok(())
    }

    /// 遍历并关闭所有带有 `close_on_exec` 标志的打开描述符（在 `execve` 已独占或私有化当前表后调用）。
    ///
    /// 每次仅在自旋锁临界区内移出单个槽位，解锁后再执行 `drop`，防止文件最后一个强引用析构时在自旋锁内阻塞。
    pub fn close_on_exec(&self) {
        for fd in 0..self.allocated_slots() {
            let old = {
                let mut slots = self.slots.lock();
                if matches!(slots.get(fd), Some(Slot::Open { flags, .. }) if flags.close_on_exec) {
                    Some(slots.replace(fd, Slot::Vacant))
                } else {
                    None
                }
            };
            drop(old);
        }
    }

    /// 复制已打开描述符 `fd` 到 `[0, limit)` 范围内的最小可用槽位，新描述符默认清除 `FD_CLOEXEC` 并共享同一个 `FileRef`（对应 `dup`）。
    pub fn dup(&self, fd: usize, limit: usize) -> Result<usize, FsError> {
        self.dup_min(fd, 0, DescriptorFlags::default(), limit)
    }

    /// 复制已打开描述符 `fd` 到 `[min, limit)` 范围内的最小可用槽位，并为新描述符设置指定的 `flags`（对应 `fcntl(F_DUPFD / F_DUPFD_CLOEXEC)`）。
    ///
    /// 若需要扩容槽位页，会在自旋锁外完成分配后重新加锁校验源 `fd` 是否仍然有效且目标空槽可用。
    pub fn dup_min(
        &self,
        fd: usize,
        min: usize,
        flags: DescriptorFlags,
        limit: usize,
    ) -> Result<usize, FsError> {
        loop {
            let required = {
                let mut slots = self.slots.lock();
                let Some(Slot::Open { file, .. }) = slots.get(fd) else {
                    return Err(FsError::BadFd);
                };
                if limit <= min {
                    return Err(FsError::InvalidOperation);
                }
                if let Some(target) = slots.vacant(min, limit) {
                    let file = file.clone();
                    slots.replace(target, Slot::Open { file, flags });
                    return Ok(target);
                }
                if limit <= slots.len() {
                    return Err(FsError::TooManyFiles);
                }
                (slots.len() + PAGE_SLOTS).max(min + 1).min(limit)
            };
            self.grow(required)?;
        }
    }

    /// 将已打开描述符 `source` 复制到指定编号 `target`（对应 `dup2`，供 `ktest` 使用）：若 `source == target` 则保持原描述符及标志不变，否则覆盖目标槽并清除 `FD_CLOEXEC`。
    #[cfg(feature = "ktest")]
    pub fn dup_exact(&self, source: usize, target: usize, limit: usize) -> Result<usize, FsError> {
        self.replace_duplicate(source, target, DescriptorFlags::default(), true, limit)
    }

    /// 将已打开描述符 `source` 复制到指定编号 `target` 并赋予 `flags`（对应 `dup3`）：`source == target` 时按规范返回 `FsError::InvalidOperation`（`EINVAL`）。
    pub fn dup_exact_flags(
        &self,
        source: usize,
        target: usize,
        flags: DescriptorFlags,
        limit: usize,
    ) -> Result<usize, FsError> {
        self.replace_duplicate(source, target, flags, false, limit)
    }

    /// `dup2` / `dup3` 的内部公共实现：支持按需扩容目标页、校验源描述符有效性与目标预留冲突，原子替换目标槽并在表自旋锁外释放被覆盖的旧文件引用。
    fn replace_duplicate(
        &self,
        source: usize,
        target: usize,
        flags: DescriptorFlags,
        allow_same: bool,
        limit: usize,
    ) -> Result<usize, FsError> {
        if !allow_same && source == target {
            return Err(FsError::InvalidOperation);
        }
        loop {
            let old = {
                let mut slots = self.slots.lock();
                let Some(Slot::Open { file, .. }) = slots.get(source) else {
                    return Err(FsError::BadFd);
                };
                if source == target && allow_same {
                    return Ok(target);
                }
                if limit <= target {
                    return Err(FsError::BadFd);
                }
                if slots.len() <= target {
                    drop(slots);
                    self.grow(target + 1)?;
                    continue;
                }
                if matches!(slots.get(target), Some(Slot::Reserved)) {
                    return Err(FsError::Busy);
                }
                let replacement = Slot::Open {
                    file: file.clone(),
                    flags,
                };
                slots.replace(target, replacement)
            };

            drop(old);
            return Ok(target);
        }
    }
}

impl FileReservation<'_> {
    /// 返回本次预留占位的整数文件描述符编号（在调用 `publish` 之前，该编号对 `lookup`/`close` 仍不可见）。
    pub fn fd(&self) -> usize {
        self.fd
    }

    /// 消费该预留守卫，将构造好的 `file` 与描述符标志 `flags` 正式写入预留槽位，使其转为对外可见的 `Slot::Open` 状态并返回 `fd`。
    pub fn publish(mut self, file: FileRef, flags: DescriptorFlags) -> usize {
        {
            let mut slots = self.table.slots.lock();
            assert!(matches!(slots.get(self.fd), Some(Slot::Reserved)));
            slots.replace(self.fd, Slot::Open { file, flags });
            self.active = false;
        }
        self.fd
    }
}

impl Drop for FileReservation<'_> {
    /// 若预留守卫在调用 `publish` 前被提前丢弃（例如路径解析或打开失败），自动将占用的 `Slot::Reserved` 槽位恢复为 `Slot::Vacant`。
    fn drop(&mut self) {
        if self.active {
            let mut slots = self.table.slots.lock();
            assert!(matches!(slots.get(self.fd), Some(Slot::Reserved)));
            slots.replace(self.fd, Slot::Vacant);
        }
    }
}
