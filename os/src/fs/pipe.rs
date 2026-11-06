//! 匿名管道（Pipe）与按需分配物理页的环形字节流缓冲区。
//!
//! # 模块职责与架构定位
//! 本模块实现两部分核心功能：
//! 1. **通用物理页环形 FIFO 缓冲区（[`PipeBuffer<N>`]）**：按需分配单个 4 KiB 物理页槽位，支持尾部写合并、
//!    消费完即刻归还物理页以及整页零拷贝转移；除本模块的匿名管道外，也供 `AF_UNIX` 流式套接字（`net/unix.rs`）复用。
//! 2. **匿名管道打开文件对（[`PipeEnd`] 与 [`try_new`]）**：向上层系统调用（`pipe2`、`read`、`write`、
//!    `splice`、`poll` / `epoll`）提供符合 POSIX 原子写（`PIPE_BUF`）与对端关闭语义（EOF / `SIGPIPE` + `EPIPE`）
//!    的单向流式打开文件描述（OFD）。通过 `dup` 或 `fork` 复制描述符只会增加同一个 [`PipeEnd`] 的 `Arc` 引用计数，
//!    只有最后一个强引用被销毁（Drop）时才会真正关闭该方向端点。
//!
//! # 核心设计与内存布局
//!
//! ## 1. 环形物理页槽位布局（[`PipeBuffer<N>`]）
//! 环形数组共 `N` 个槽位（匿名管道默认 `SLOTS = 16`，总容量 64 KiB），空管道不预分配任何物理页：
//!
//! ```text
//! PipeBuffer<16> 环形数组:
//!   [ .. 空 .. | Slot(tail, 最早未读) | ... 中间满槽 ... | Slot(head-1, 最新写入) | .. 空 .. ]
//!                 │                                          │
//!                 ▼ 4096B 物理页                              ▼ 4096B 物理页
//!   +-------------+------------------+         +-------------+----------+------------+
//!   | 已消费前缀   | 未读有效字节      |         | 已消费/偏移  | 有效字节  | merge_space|
//!   | 0..offset   | offset..+length  |         | 0..offset   | length   | 可追加后缀  |
//!   +-------------+------------------+         +-------------+----------+------------+
//! ```
//! - **尾部合并写入**：当 `head - 1` 槽位的物理页尾部仍有剩余空间（`merge_space() > 0`）时，新写入的数据直接追加到该页末尾；
//!   前端已被读取过的 `0..offset` 区域不会回绕复用，整槽的 `length` 归零后立即释放该物理页并推进 `tail`。
//! - **整页零拷贝 `splice`**：管道到管道传输（[`PipeBuffer::splice_into`]）时，若 `tail` 槽位的数据全部位于本次请求长度内，
//!   直接将整个 [`Slot`]（含底层 `OwnedPages` 所有权）移交给目标环形缓冲区，避免字节拷贝与页面重分配。
//!
//! ## 2. 无锁 `poll` 观察字（[`Pipe::observation`]）位域编码
//! 为使高频的 `poll` / `epoll` 状态复查无需争抢 `Pipe::ring` 睡眠互斥锁，`Pipe` 维护一个 64 位原子快照，
//! 每次在持有 `ring` 锁完成读写修改后通过 [`Pipe::publish`] 以 `Release` 内存序更新：
//!
//! ```text
//! 63                           SPACE_SHIFT (17) 16                          0
//! +--------------------------------------------+----------------------------+
//! |    空闲物理槽字节容量 empty_slot_capacity     |   可读有效字节数 ring.len()  |
//! |         ((16 - used) * 4096)               |        (0 ..= 65536)       |
//! +--------------------------------------------+----------------------------+
//! ```
//! 注意：`poll` 写就绪（`POLLOUT`）仅依据是否存在至少一个完整空闲物理槽（`empty_slot_capacity > 0`）判定，
//! 不因全满槽位的末页仍剩少量尾部合并空间而报告 `POLLOUT`，从而保证收到 `POLLOUT` 后的用户至少能无阻塞写入 `PIPE_BUF` 字节。
//!
//! # 锁获取顺序与并发契约
//! 1. **跨文件与管道锁顺序**：普通文件偏移锁（由 `fs/transfer.rs` 在外层持有） -> `Pipe::ring` 睡眠互斥锁。
//! 2. **双管道 `splice_to_pipe` 锁顺序**：当同时锁定输入与输出两个不同管道的 `ring` 睡眠互斥锁时，
//!    严格按底层 `Arc::as_ptr(&pipe)` 指针地址从小到大顺序加锁，防止双向并发 `splice` 产生 AB-BA 死锁。
//! 3. **通知与阻塞边界**：`Pipe::publish` 在持有 `ring` 睡眠锁期间仅调用无阻塞的 [`PollWaitHead::notify`]（内部仅持自旋锁）；
//!    线程进入 [`PollContext::park_until`] 睡眠前必须释放所有 `Pipe::ring` 睡眠锁。
//! 4. **反向监控注销顺序**：[`PipeEnd::drop`] 先调用 `WatchHead::release` 同步清理所有 `epoll` 监听项，
//!    随后才将 `reader_open` / `writer_open` 置为 `false` 并唤醒对端等待队列。

use alloc::sync::Arc;
use core::ops::Range;
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};

use crate::fs::epoll::WatchHead;
use crate::fs::file::{
    set_nonblocking, set_status_flags, FileIoContext, FileLike, FileRef, FileStat, InodeFile,
    ReadBuffer, SeekFrom, WriteBuffer,
};
use crate::fs::poll::{PollContext, PollMask, PollNotify, PollTable, PollWaitHead, Pollable};
use crate::fs::{
    DirectoryEntry, FsError, InodeKind, InodeRef, InodeTime, Path, O_CLOEXEC, O_NONBLOCK,
};
use crate::mm::{self, OwnedPages};
use crate::sync::SleepMutex;
use crate::task::signal::SIGPIPE;
use crate::wait::WaitResult;

/// POSIX 保证单次 `write` 原子不交错的最大字节数（等于单个物理页大小 4096 字节）。
pub const PIPE_BUF: usize = 4096;
/// 匿名管道默认分配全部物理槽位时的最大总字节容量（16 页，共 65536 字节）。
pub const PIPE_CAPACITY: usize = SLOTS * PIPE_BUF;
/// 匿名管道环形缓冲区包含的物理页槽位数量。
const SLOTS: usize = 16;

/// 在 `Pipe::observation` 64 位原子状态字中存放高段“空闲槽字节容量”的左移位数（容纳 `0..=PIPE_CAPACITY` 所需的位宽，即 17 位）。
const SPACE_SHIFT: u32 = usize::BITS - PIPE_CAPACITY.leading_zeros();
/// 从 `Pipe::observation` 低位提取当前可读字节数（`0..=PIPE_CAPACITY`）的位掩码。
const BYTE_MASK: u64 = (1 << SPACE_SHIFT) - 1;

/// 为每个新创建的匿名管道分配进程内唯一伪 inode 编号的全局单调递增计数器。
static NEXT_INODE: AtomicU64 = AtomicU64::new(1);

/// 环形缓冲区中的单个数据槽位，独占持有一整页（4096 字节）物理内存。
struct Slot {
    /// 槽位独占拥有的底层物理页（阶数 0，即单页 `PAGE_SIZE == PIPE_BUF`）。
    pages: OwnedPages,
    /// 页内尚未被消费的有效数据起始字节偏移量（范围 `0..PIPE_BUF`）。
    offset: usize,
    /// 从 `offset` 开始已写入且尚未被消费的有效字节长度（范围 `1..=PIPE_BUF - offset`，归零时立即释放本槽位）。
    length: usize,
}

/// 按需分配物理页的定长槽位环形 FIFO 缓冲区（由匿名管道和 `AF_UNIX` 流式套接字共享使用）。
///
/// 本结构本身不包含内部锁，调用方需通过外层互斥锁（如 `Pipe::ring`）串行化访问。
pub(crate) struct PipeBuffer<const N: usize> {
    /// 环形物理页槽位数组，仅在槽位存有未读数据时为 `Some(Slot)`。
    slots: [Option<Slot>; N],
    /// 下一个待分配新物理页槽位的环形写下标（范围 `0..N`）。
    head: usize,
    /// 最早包含未消费数据的物理页槽位环形读下标（范围 `0..N`；当 `used == 0` 时与 `head` 相等）。
    tail: usize,
    /// 当前环形数组中处于占用状态的非空物理页槽位数量（范围 `0..=N`）。
    used: usize,
    /// 所有占用槽位中尚未消费的有效数据总字节数。
    bytes: usize,
}

impl<const N: usize> PipeBuffer<N> {
    /// 创建一个空的环形页缓冲区（初始不分配任何物理页；编译/运行期要求槽位数 `N > 0` 且系统页大小等于 `PIPE_BUF`）。
    pub(crate) fn new() -> Self {
        assert!(0 < N);
        assert_eq!(mm::PAGE_SIZE, PIPE_BUF);
        Self {
            slots: core::array::from_fn(|_| None),
            head: 0,
            tail: 0,
            used: 0,
            bytes: 0,
        }
    }

    /// 返回缓冲区内已成功写入且尚未被读取消费的总字节数。
    pub(crate) fn len(&self) -> usize {
        self.bytes
    }

    /// 判断缓冲区当前是否没有任何可读字节。
    pub(crate) fn is_empty(&self) -> bool {
        self.bytes == 0
    }

    /// 返回全部 `N` 个物理页槽位均写满时的理论最大字节容量（`N * PIPE_BUF`）。
    pub(crate) fn capacity(&self) -> usize {
        N * PIPE_BUF
    }

    /// 计算最新写入槽位（`head - 1`）的物理页尾部剩余可合并追加字节数；缓冲区为空时返回 `0`。
    fn merge_space(&self) -> usize {
        if self.used == 0 {
            return 0;
        }
        let slot = self.slots[(self.head + N - 1) % N].as_ref().unwrap();
        PIPE_BUF - slot.offset - slot.length
    }

    /// 返回当前实际可立即写入的总字节空间（等于全部空闲物理槽容量加上最后一个占用页的尾部可合并空间）。
    pub(crate) fn space(&self) -> usize {
        self.empty_slot_capacity() + self.merge_space()
    }

    /// 返回尚未分配使用的空闲物理槽所对应的整页字节容量（`(N - used) * PIPE_BUF`，供 `poll` 判定写就绪）。
    pub(crate) fn empty_slot_capacity(&self) -> usize {
        self.capacity() - self.used * PIPE_BUF
    }

    /// 立即释放全部已占用的物理页，并将环形队列的读写游标与字节计数重置为空状态。
    pub(crate) fn clear(&mut self) {
        self.slots.iter_mut().for_each(|slot| drop(slot.take()));
        self.head = 0;
        self.tail = 0;
        self.used = 0;
        self.bytes = 0;
    }

    /// 从最早的数据槽位（`tail`）消费前 `count` 个字节；若该槽位的有效字节全部消费完毕，立即释放其物理页并推进 `tail`。
    fn consume(&mut self, count: usize) {
        let slot = self.slots[self.tail].as_mut().unwrap();
        assert!(count <= slot.length);
        slot.offset += count;
        slot.length -= count;
        self.bytes -= count;
        if slot.length == 0 {
            drop(self.slots[self.tail].take());
            self.tail = (self.tail + 1) % N;
            self.used -= 1;
        }
    }

    /// 借用最早槽位（`tail`）中最多 `limit` 字节的已初始化未读数据切片（调用方须在借用结束后才能调用 `consume` 消费或释放该槽位）。
    fn front(&mut self, limit: usize) -> &[u8] {
        if self.is_empty() {
            return &[];
        }
        let slot = self.slots[self.tail].as_mut().unwrap();
        let count = limit.min(slot.length);
        // SAFETY:
        // 1. `slot.pages` 为当前槽位独占持有的 4096 字节已清零物理页，`slot.offset + count <= slot.offset + slot.length <= PIPE_BUF` 保证指针运算与切片长度不越界且已初始化；
        // 2. 返回切片的生命周期绑定在 `&mut self` 独占借用上，期间调用方无法修改该槽位数据或释放物理页。
        unsafe {
            core::slice::from_raw_parts(
                slot.pages.as_mut_ptr().cast::<u8>().add(slot.offset),
                count,
            )
        }
    }

    /// 将当前缓冲区头部最多 `limit` 字节移入目标缓冲区 `target`：若整个 `tail` 槽位不超过 `limit` 则直接零拷贝转移物理页槽位，否则拷贝前缀并按实际接收量消费。
    fn splice_into(&mut self, target: &mut Self, limit: usize) -> Result<usize, FsError> {
        assert!(!self.is_empty());
        assert!(0 < target.empty_slot_capacity());
        let length = self.slots[self.tail].as_ref().unwrap().length;
        // 快速路径：最早槽位的全部有效数据均在本次传输上限内，直接将整页 Slot 移交给目标空闲槽位。
        if length <= limit {
            let slot = self.slots[self.tail].take().unwrap();
            self.tail = (self.tail + 1) % N;
            self.used -= 1;
            self.bytes -= length;
            assert!(target.slots[target.head].is_none());
            target.slots[target.head] = Some(slot);
            target.head = (target.head + 1) % N;
            target.used += 1;
            target.bytes += length;
            return Ok(length);
        }

        // 慢速路径：请求长度小于当前整槽数据量，拷贝前缀追加到目标缓冲区，并仅消费目标实际接收的字节数。
        let accepted = target.append(&mut WriteBuffer::kernel(self.front(limit)))?;
        self.consume(accepted);
        Ok(accepted)
    }

    /// 从写入源 `source` 拉取数据追加到最后一个槽位的尾部空闲区（若无尾部空间则分配新物理页槽位）；单次最多填满一个物理页后缀，若拷贝失败且新槽位为空则立即回收该空页。
    pub(crate) fn append(&mut self, source: &mut WriteBuffer<'_>) -> Result<usize, FsError> {
        // Step 1: 若最新槽位无尾部剩余空间，分配一个已清零的新物理页挂入 `head` 槽位。
        if self.merge_space() == 0 {
            assert!(self.used < N);
            let pages = mm::alloc_pages(0, true).ok_or(FsError::NoMemory)?;
            self.slots[self.head] = Some(Slot {
                pages,
                offset: 0,
                length: 0,
            });
            self.head = (self.head + 1) % N;
            self.used += 1;
        }

        // Step 2: 构造指向该页尾部可写区间 `offset..PIPE_BUF` 的可变字节切片，并从 `source` 拷贝数据。
        let index = (self.head + N - 1) % N;
        let slot = self.slots[index].as_mut().unwrap();
        let offset = slot.offset + slot.length;
        // SAFETY:
        // 1. `slot.pages` 为当前槽位独占持有的已清零物理页（大小 `PIPE_BUF`），且 `offset <= PIPE_BUF`，`offset..PIPE_BUF` 完全处于该页有效且已初始化范围内；
        // 2. `&mut self` 保证无其他并发切片借用该页，`source.pull(dst)` 在返回前即结束对 `dst` 的借用，之后才更新 `slot.length` 或回收空槽。
        let dst = unsafe {
            core::slice::from_raw_parts_mut(
                slot.pages.as_mut_ptr().cast::<u8>().add(offset),
                PIPE_BUF - offset,
            )
        };

        // Step 3: 提交成功拷贝的字节长度；若新分配槽位因用户态缺页错误等原因导致实际长度为 0，回滚释放该空物理页。
        let result = source.pull(dst);
        if let Ok(count) = result {
            slot.length += count;
            self.bytes += count;
        }
        if slot.length == 0 {
            drop(self.slots[index].take());
            self.head = index;
            self.used -= 1;
        }
        result
    }

    /// 按 FIFO 顺序向读取目标 `target` 逐槽写出数据：当 `peek == false` 时实时消费已成功拷贝的字节并回收空页；当 `peek == true` 时仅移动局部游标而不消费缓冲区数据。
    ///
    /// 若中途发生用户态拷贝错误（如 `EFAULT`），若此前已成功传输了部分字节（Partial Progress）则返回已传输字节数，否则返回该错误。
    pub(crate) fn receive(
        &mut self,
        target: &mut ReadBuffer<'_>,
        peek: bool,
    ) -> Result<usize, FsError> {
        let start = target.copied();
        let available = self.len();
        let mut index = self.tail;
        let mut offset = 0;
        while target.copied() - start < available && target.copied() < target.len() {
            let slot = self.slots[index].as_mut().unwrap();
            let length = slot.length;
            let count = (length - offset).min(target.len() - target.copied());
            // SAFETY:
            // 1. `slot.pages` 为当前槽位独占持有的存活物理页，区间 `slot.offset + offset .. +count` 完全落在已提交的有效初始化数据 `slot.offset .. +slot.length` 内；
            // 2. `&mut self` 排除了并发写入或页面释放，`target.push(bytes)` 返回后不再保留 `bytes` 引用，随后才调用 `self.consume` 修改槽位。
            let bytes = unsafe {
                core::slice::from_raw_parts(
                    slot.pages
                        .as_mut_ptr()
                        .cast::<u8>()
                        .add(slot.offset + offset),
                    count,
                )
            };
            let before = target.copied();
            let result = target.push(bytes);
            let delivered = target.copied() - before;
            if peek {
                offset += delivered;
                if offset == length {
                    index = (index + 1) % N;
                    offset = 0;
                }
            } else {
                self.consume(delivered);
                index = self.tail;
            }
            if let Err(error) = result {
                let copied = target.copied() - start;
                return if copied == 0 { Err(error) } else { Ok(copied) };
            }
            assert!(0 < delivered, "FIFO copyout made no progress");
        }

        Ok(target.copied() - start)
    }
}

/// 匿名管道读写两端共同持有的底层共享管道实体。
struct Pipe {
    /// 保护物理页环形缓冲区读写与槽位分配释放的睡眠互斥锁。
    ring: SleepMutex<PipeBuffer<SLOTS>>,
    /// 供 `poll` 无锁读取的 64 位原子状态快照（低 `SPACE_SHIFT` 位为可读字节数，高位为空闲物理槽字节容量）。
    observation: AtomicU64,
    /// 读端打开文件描述（[`PipeEnd`]，`read == true`）是否仍然存活。
    reader_open: AtomicBool,
    /// 写端打开文件描述（[`PipeEnd`]，`read == false`）是否仍然存活。
    writer_open: AtomicBool,
    /// 通知等待“管道变为可读或写端挂断”订阅者的持续等待队列头。
    read_head: Arc<PollWaitHead>,
    /// 通知等待“管道变为可写或读端关闭”订阅者的持续等待队列头。
    write_head: Arc<PollWaitHead>,
    /// 两端 `fstat` 共享报告的进程内唯一匿名 FIFO 伪 inode 编号。
    inode: u64,
    /// 创建该管道时的所有者用户 ID（供 `fstat` 报告）。
    uid: u32,
    /// 创建该管道时的所有者组 ID（供 `fstat` 报告）。
    gid: u32,
}

impl Pipe {
    /// 在持有 `ring` 锁完成缓冲区状态变更后，发布最新原子容量快照并按需唤醒读/写端等待队列。
    fn publish(&self, ring: &PipeBuffer<SLOTS>, read_changed: bool, write_changed: bool) {
        self.observation.store(
            (ring.len() as u64) | ((ring.empty_slot_capacity() as u64) << SPACE_SHIFT),
            Ordering::Release,
        );
        if read_changed {
            self.read_head
                .notify(PollMask::POLLIN | PollMask::POLLRDNORM);
        }
        if write_changed {
            self.write_head
                .notify(PollMask::POLLOUT | PollMask::POLLWRNORM);
        }
    }
}

/// 匿名管道单侧（只读端或只写端）的打开文件描述对象（OFD）。
///
/// 每个管道创建时仅生成一读一写两个 `PipeEnd` 实例；`dup` / `fork` 共享同一个 `Arc<PipeEnd>`，不会产生额外的端点。
pub struct PipeEnd {
    /// 指向读写两端共享的底层管道实体。
    pipe: Arc<Pipe>,
    /// 端点传输方向：`true` 表示只读端（`O_RDONLY`），`false` 表示只写端（`O_WRONLY`）。
    read: bool,
    /// 该端点的打开文件状态标志（包含只读/只写访问模式及可通过 `fcntl` 动态修改的 `O_NONBLOCK` 等标志）。
    flags: AtomicU32,
    /// 挂载在该端点上的 `epoll` 反向监控链表头，在端点最终关闭（Drop）时自动注销所有关联监听项。
    watches: Arc<WatchHead>,
}

/// 按指定标志 `flags`（仅允许 `O_NONBLOCK` 与 `O_CLOEXEC`）和创建者凭据 `(uid, gid)` 创建一对相连的匿名管道 `(读端 FileRef, 写端 FileRef)`。
///
/// `O_NONBLOCK` 直接写入读写两个打开文件描述的状态字；`O_CLOEXEC` 由上层系统调用在安装文件描述符槽位时设置。
pub fn try_new(flags: u32, uid: u32, gid: u32) -> Result<(FileRef, FileRef), FsError> {
    if flags & !(O_NONBLOCK | O_CLOEXEC) != 0 {
        return Err(FsError::InvalidOperation);
    }

    let read_head = Arc::try_new(PollWaitHead::new()).map_err(|_| FsError::NoMemory)?;
    let write_head = Arc::try_new(PollWaitHead::new()).map_err(|_| FsError::NoMemory)?;
    let inode = NEXT_INODE
        .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |id| id.checked_add(1))
        .expect("pipe inode identity exhausted");
    let pipe = Arc::try_new(Pipe {
        ring: SleepMutex::new(PipeBuffer::new()),
        observation: AtomicU64::new((PIPE_CAPACITY as u64) << SPACE_SHIFT),
        reader_open: AtomicBool::new(true),
        writer_open: AtomicBool::new(true),
        read_head,
        write_head,
        inode,
        uid,
        gid,
    })
    .map_err(|_| FsError::NoMemory)?;

    let read_watches = Arc::try_new(WatchHead::new()).map_err(|_| FsError::NoMemory)?;
    let write_watches = Arc::try_new(WatchHead::new()).map_err(|_| FsError::NoMemory)?;
    let reader: FileRef = Arc::try_new(PipeEnd {
        pipe: pipe.clone(),
        read: true,
        flags: AtomicU32::new(flags & O_NONBLOCK),
        watches: read_watches,
    })
    .map_err(|_| FsError::NoMemory)?;
    let writer: FileRef = Arc::try_new(PipeEnd {
        pipe,
        read: false,
        flags: AtomicU32::new(1 | (flags & O_NONBLOCK)),
        watches: write_watches,
    })
    .map_err(|_| FsError::NoMemory)?;
    Ok((reader, writer))
}

impl PipeEnd {
    /// 在首次确认需要阻塞等待时创建 `PollContext` 与单槽位 `PollTable` 并注册当前端点的等待队列头（返回后调用方须先复查状态再决定是否睡眠）。
    fn prepare_wait(&self) -> Result<(Arc<PollContext>, PollTable), FsError> {
        let context = PollContext::try_new()?;
        let sink: Arc<dyn PollNotify> = context.clone();
        let mut table = PollTable::new(sink, 1)?;
        self.poll(Some(&mut table))?;
        Ok((context, table))
    }

    /// 统一处理部分传输后的错误返回：若尚未传输任何字节（`count == 0`）则返回 `Err(error)`，否则优先保留并返回已成功传输的字节数 `Ok(count)`。
    fn progress(count: usize, error: FsError) -> Result<usize, FsError> {
        if count == 0 {
            Err(error)
        } else {
            Ok(count)
        }
    }

    /// 处理读端已关闭时的管道写入：若调用者为用户态线程则向其发送线程定向的 `SIGPIPE` 信号，随后按已有传输进度返回已写字节数或 `FsError::BrokenPipe`（`EPIPE`）。
    fn broken_pipe(&self, caller: FileIoContext<'_>, count: usize) -> Result<usize, FsError> {
        if let FileIoContext::User(user) = caller {
            user.send_kernel_signal(SIGPIPE);
        }
        Self::progress(count, FsError::BrokenPipe)
    }

    /// 判断两个管道端点是否连接在同一个底层 `Pipe` 实例上（同一管道的读端与写端比较也会返回 `true`）。
    pub(crate) fn same_pipe(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.pipe, &other.pipe)
    }

    /// 判断一次 `splice` 操作在当前管道端点上是否应以非阻塞模式运行（合并 `SPLICE_F_NONBLOCK` 请求参数与本端点的 `O_NONBLOCK` 状态位）。
    fn splice_nonblocking(&self, requested: bool) -> bool {
        requested || self.flags() & O_NONBLOCK != 0
    }

    /// 实现“普通文件 -> 管道写端”的 `splice` 传输：从 `file` 的指定偏移 `offset` 处分块读取数据写入当前管道写端（外层调用方已预先锁定文件偏移并在返回后按实际写入管道字节数推进偏移）。
    pub(crate) fn splice_from_file(
        &self,
        file: &InodeFile,
        offset: u64,
        length: usize,
        nonblocking: bool,
        caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        if self.read {
            return Err(FsError::BadFd);
        }

        let mut waiting: Option<(Arc<PollContext>, PollTable)> = None;
        let mut interrupted = false;
        let mut copied = 0;
        let mut scratch = [0u8; PIPE_BUF];
        loop {
            // Step 1: 清除上一轮遗留的通知标志，并锁定管道环形缓冲区推进文件读取与管道写入。
            if let Some((context, _)) = &waiting {
                context.clear_triggered();
            }
            {
                let mut ring = self.pipe.ring.lock();
                if !self.pipe.reader_open.load(Ordering::Acquire) {
                    return self.broken_pipe(caller, copied);
                }
                while copied < length && 0 < ring.empty_slot_capacity() {
                    let room = if ring.merge_space() == 0 {
                        PIPE_BUF
                    } else {
                        ring.merge_space()
                    };
                    let request = (length - copied).min(room);
                    let read = match file.read_at(offset + copied as u64, &mut scratch[..request]) {
                        Ok(read) => read,
                        Err(error) => return Self::progress(copied, error),
                    };
                    if read == 0 {
                        return Ok(copied);
                    }

                    if !self.pipe.reader_open.load(Ordering::Acquire) {
                        return self.broken_pipe(caller, copied);
                    }
                    let result = ring.append(&mut WriteBuffer::kernel(&scratch[..read]));
                    self.pipe
                        .publish(&ring, result.as_ref().is_ok_and(|count| 0 < *count), false);
                    let accepted = match result {
                        Ok(accepted) => accepted,
                        Err(error) => return Self::progress(copied, error),
                    };
                    copied += accepted;
                    if accepted < read || read < request {
                        return Ok(copied);
                    }
                }
                if 0 < copied || copied == length {
                    return Ok(copied);
                }
                if self.splice_nonblocking(nonblocking) {
                    return Err(FsError::WouldBlock);
                }
                if interrupted {
                    return Err(FsError::Interrupted);
                }
            }

            // Step 2: 管道已满且尚未写入任何字节，在释放 `ring` 锁后注册等待表或进入可中断睡眠等待读端腾出空槽。
            if let Some((context, _)) = &waiting {
                interrupted = context.park_until(matches!(caller, FileIoContext::User(_)), None)
                    == WaitResult::Interrupted;
            } else {
                waiting = Some(self.prepare_wait()?);
            }
        }
    }

    /// 实现“管道读端 -> 普通文件”的 `splice` 传输：借用当前管道头部已提交切片写入目标文件 `offset` 处，仅在目标文件成功接收后才消费管道字节（文件元数据同步由外层在释放锁后完成）。
    pub(crate) fn splice_to_file(
        &self,
        file: &InodeFile,
        offset: u64,
        length: usize,
        nonblocking: bool,
        caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        if !self.read {
            return Err(FsError::BadFd);
        }

        let mut waiting: Option<(Arc<PollContext>, PollTable)> = None;
        let mut interrupted = false;
        let mut copied = 0;
        loop {
            // Step 1: 检查待处理信号并锁定管道环形缓冲区，将头部槽位数据切片直接写入目标文件。
            if crate::fs::transfer::interruption_pending(caller) {
                return Self::progress(copied, FsError::Interrupted);
            }
            if let Some((context, _)) = &waiting {
                context.clear_triggered();
            }
            {
                let mut ring = self.pipe.ring.lock();
                while copied < length && !ring.is_empty() {
                    if crate::fs::transfer::interruption_pending(caller) {
                        return Self::progress(copied, FsError::Interrupted);
                    }
                    let bytes = ring.front(length - copied);
                    let expected = bytes.len();
                    let result = file.transfer_write_at(
                        offset + copied as u64,
                        bytes,
                        FsError::InvalidOperation,
                    );
                    let accepted = match result {
                        Ok(accepted) => accepted,
                        Err(error) => return Self::progress(copied, error),
                    };
                    ring.consume(accepted);
                    copied += accepted;
                    self.pipe.publish(&ring, false, 0 < accepted);
                    if accepted < expected {
                        return Ok(copied);
                    }
                }
                if 0 < copied || copied == length || !self.pipe.writer_open.load(Ordering::Acquire)
                {
                    return Ok(copied);
                }
                if self.splice_nonblocking(nonblocking) {
                    return Err(FsError::WouldBlock);
                }
                if interrupted {
                    return Err(FsError::Interrupted);
                }
            }

            // Step 2: 管道为空且写端仍存活，在释放 `ring` 锁后注册等待表或睡眠等待写端写入数据。
            if let Some((context, _)) = &waiting {
                interrupted = context.park_until(matches!(caller, FileIoContext::User(_)), None)
                    == WaitResult::Interrupted;
            } else {
                waiting = Some(self.prepare_wait()?);
            }
        }
    }

    /// 实现“源管道读端 -> 目标管道写端”的 `splice` 传输：按底层 `Pipe` 指针地址升序锁定双环，整槽满足上限时直接零拷贝转移物理页所有权，阻塞前释放双环锁。
    pub(crate) fn splice_to_pipe(
        &self,
        target: &Self,
        length: usize,
        nonblocking: bool,
        caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        if !self.read || target.read {
            return Err(FsError::BadFd);
        }
        if self.same_pipe(target) {
            return Err(FsError::InvalidOperation);
        }

        let mut waiting: Option<(bool, Arc<PollContext>, PollTable)> = None;
        let mut interrupted = false;
        loop {
            if let Some((_, context, _)) = &waiting {
                context.clear_triggered();
            }
            // Step 1: 严格按底层 `Pipe` 地址从小到大顺序获取两个管道的 `ring` 睡眠锁，防止双向并发 `splice` 产生 AB-BA 死锁。
            let wait_input = {
                let (mut input, mut output) = if Arc::as_ptr(&self.pipe) < Arc::as_ptr(&target.pipe)
                {
                    let input = self.pipe.ring.lock();
                    let output = target.pipe.ring.lock();
                    (input, output)
                } else {
                    let output = target.pipe.ring.lock();
                    let input = self.pipe.ring.lock();
                    (input, output)
                };

                // Step 2: 在双锁保护下循环将源管道槽位转移或前缀拷贝至目标管道，并实时发布两端状态快照。
                let mut copied = 0;
                while copied < length {
                    if !target.pipe.reader_open.load(Ordering::Acquire) {
                        return target.broken_pipe(caller, copied);
                    }
                    if input.is_empty() || output.empty_slot_capacity() == 0 {
                        break;
                    }
                    let result = input.splice_into(&mut output, length - copied);
                    let accepted = match result {
                        Ok(accepted) => accepted,
                        Err(error) => return Self::progress(copied, error),
                    };
                    copied += accepted;
                    self.pipe.publish(&input, false, 0 < accepted);
                    target.pipe.publish(&output, 0 < accepted, false);
                }
                if 0 < copied || copied == length {
                    return Ok(copied);
                }
                if input.is_empty() && !self.pipe.writer_open.load(Ordering::Acquire) {
                    return Ok(0);
                }
                if self.splice_nonblocking(nonblocking) || target.splice_nonblocking(nonblocking) {
                    return Err(FsError::WouldBlock);
                }
                if interrupted {
                    return Err(FsError::Interrupted);
                }
                input.is_empty()
            };

            // Step 3: 释放双环锁后，仅针对当前真正导致阻塞的那一端（源空等输入 `wait_input == true`，或目标满等输出）注册等待或进入睡眠；若阻塞方向切换则重建订阅并回到循环顶部重扫。
            if waiting
                .as_ref()
                .is_some_and(|(input, _, _)| *input == wait_input)
            {
                let (_, context, _) = waiting.as_ref().unwrap();
                interrupted = context.park_until(matches!(caller, FileIoContext::User(_)), None)
                    == WaitResult::Interrupted;
            } else {
                drop(waiting.take());
                let end = if wait_input { self } else { target };
                let (context, table) = end.prepare_wait()?;
                waiting = Some((wait_input, context, table));
            }
        }
    }
}

impl Pollable for PipeEnd {
    /// 查询管道端点的即时就绪事件掩码：首轮将对应方向的等待队列头注册到 `table`，随后无锁读取 `observation` 原子快照与对端存活状态。
    fn poll(&self, table: Option<&mut PollTable>) -> Result<PollMask, FsError> {
        if let Some(table) = table {
            table.register(if self.read {
                &self.pipe.read_head
            } else {
                &self.pipe.write_head
            })?;
        }

        let state = self.pipe.observation.load(Ordering::Acquire);
        let mut mask = PollMask::default();
        if self.read {
            if 0 < state & BYTE_MASK {
                mask |= PollMask::POLLIN | PollMask::POLLRDNORM;
            }
            if !self.pipe.writer_open.load(Ordering::Acquire) {
                mask |= PollMask::POLLHUP;
            }
        } else {
            if 0 < state >> SPACE_SHIFT {
                mask |= PollMask::POLLOUT | PollMask::POLLWRNORM;
            }
            if !self.pipe.reader_open.load(Ordering::Acquire) {
                mask |= PollMask::POLLERR;
            }
        }
        Ok(mask)
    }
}

impl FileLike for PipeEnd {
    /// 返回对当前 [`PipeEnd`] 的引用，供 `splice` 等跨文件传输路径直接识别并操作匿名管道端点。
    fn pipe_end(&self) -> Option<&PipeEnd> {
        Some(self)
    }

    /// 从管道读端读取数据写入 `target`：只要读到至少 1 个字节或写端已全部关闭（返回 0 表示 EOF）即立即返回；否则按阻塞/非阻塞策略等待写端写入。
    fn read_into(
        &self,
        target: &mut ReadBuffer<'_>,
        caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        if !self.read {
            return Err(FsError::BadFd);
        }
        let start = target.copied();
        if target.len() == start {
            return Ok(0);
        }

        let mut waiting: Option<(Arc<PollContext>, PollTable)> = None;
        let mut interrupted = false;
        loop {
            if let Some((context, _)) = &waiting {
                context.clear_triggered();
            }
            {
                let mut ring = self.pipe.ring.lock();
                let before = target.copied();
                let result = ring.receive(target, false);
                self.pipe.publish(&ring, false, before < target.copied());
                if let Err(error) = result {
                    return Self::progress(target.copied() - start, error);
                }
                let copied = target.copied() - start;
                if 0 < copied || !self.pipe.writer_open.load(Ordering::Acquire) {
                    return Ok(copied);
                }
                if self.flags() & O_NONBLOCK != 0 {
                    return Err(FsError::WouldBlock);
                }
                if interrupted {
                    return Err(FsError::Interrupted);
                }
            }

            if let Some((context, _)) = &waiting {
                interrupted = context.park_until(matches!(caller, FileIoContext::User(_)), None)
                    == WaitResult::Interrupted;
            } else {
                waiting = Some(self.prepare_wait()?);
            }
        }
    }

    /// 匿名管道是字节流，无文件偏移概念，`pread` 固定返回 `FsError::IllegalSeek`（`ESPIPE`）。
    fn read_at_into(
        &self,
        _offset: u64,
        _target: &mut ReadBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::IllegalSeek)
    }

    /// 匿名管道端点没有关联的文件系统路径。
    fn path(&self) -> Option<&Path> {
        None
    }
    /// 匿名管道不支持 `mmap` 文件内存映射。
    fn mapping_inode(&self) -> Option<&InodeRef> {
        None
    }
    /// 暴露 `Pollable` 接口以支持 `poll` / `select` / `epoll` 事件监听。
    fn pollable(&self) -> Option<&dyn Pollable> {
        Some(self)
    }
    /// 返回该打开端点的 `epoll` 反向监控链表头（复制出的文件描述符共享同一个端点与析构注销时机）。
    fn watch_head(&self) -> Option<&Arc<WatchHead>> {
        Some(&self.watches)
    }

    /// 读取当前打开文件描述的访问模式与状态标志（不含描述符槽位级别的 `O_CLOEXEC`）。
    fn flags(&self) -> u32 {
        self.flags.load(Ordering::Acquire)
    }
    /// 按 `fcntl(F_SETFL)` 规则原子更新可变状态标志，保持端点固有的只读/只写访问模式不变。
    fn set_status_flags(&self, raw: u32) -> Result<(), FsError> {
        set_status_flags(&self.flags, raw)
    }
    /// 原子设置或清除该管道端点的 `O_NONBLOCK` 标志（通过 `dup` / `fork` 共享该端点的描述符同步生效）。
    fn set_nonblocking(&self, enabled: bool) -> Result<(), FsError> {
        set_nonblocking(&self.flags, enabled)
    }

    /// 管道字节流不支持 `ftruncate` 截断。
    fn truncate(&self, _new_size: u64) -> Result<(), FsError> {
        Err(FsError::InvalidOperation)
    }
    /// 匿名管道驻留内存，无底层持久化存储需要 `fsync` 同步。
    fn sync_range(&self, _range: Range<u64>, _data_only: bool) -> Result<(), FsError> {
        Err(FsError::InvalidOperation)
    }
    /// 匿名管道不支持 `lseek` 定位，固定返回 `FsError::IllegalSeek`（`ESPIPE`）。
    fn seek(&self, _from: SeekFrom) -> Result<u64, FsError> {
        Err(FsError::IllegalSeek)
    }

    /// 返回匿名管道的 `fstat` 属性（读写两端共享相同的伪 inode 编号，类型为 `S_IFIFO | 0600`，块大小为 `PIPE_BUF`）。
    fn stat(&self) -> Result<FileStat, FsError> {
        let zero = InodeTime {
            seconds: 0,
            nanoseconds: 0,
        };
        Ok(FileStat {
            inode: self.pipe.inode,
            kind: InodeKind::Fifo,
            mode: crate::fs::types::S_IFIFO | 0o600,
            size: 0,
            uid: self.pipe.uid,
            gid: self.pipe.gid,
            links: 1,
            blocks: 0,
            device: 0,
            rdev: 0,
            block_size: PIPE_BUF as u32,
            atime: zero,
            mtime: zero,
            ctime: zero,
        })
    }

    /// 管道端点不是目录，不支持 `readdir`。
    fn readdir(&self) -> Result<Option<DirectoryEntry>, FsError> {
        Err(FsError::NotDirectory)
    }
    /// 管道端点不是目录，不支持目录项枚举。
    fn for_each_dirent(
        &self,
        _emit: &mut dyn FnMut(&DirectoryEntry) -> Result<bool, FsError>,
    ) -> Result<(), FsError> {
        Err(FsError::NotDirectory)
    }

    /// 向管道写端写入数据：总长度不超过 `PIPE_BUF` 的小请求遵循 POSIX 原子写语义（剩余空间不足以容纳整笔请求时先等待而非拆分写入）；大于 `PIPE_BUF` 的请求分段写入并在中途出错或阻塞时保留已成功写入的字节数。
    fn write_from(
        &self,
        source: &mut WriteBuffer<'_>,
        caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        if self.read {
            return Err(FsError::BadFd);
        }
        let start = source.copied();
        if source.len() == start {
            return Ok(0);
        }

        let small = source.len() <= PIPE_BUF;
        let mut written = 0;
        let mut waiting: Option<(Arc<PollContext>, PollTable)> = None;
        let mut interrupted = false;
        loop {
            // Step 1: 清除旧通知标志并获取管道 `ring` 锁，校验读端仍处于打开状态（若读端已关闭则触发 `SIGPIPE` 并返回 `EPIPE`）。
            if let Some((context, _)) = &waiting {
                context.clear_triggered();
            }
            {
                let mut ring = self.pipe.ring.lock();
                if !self.pipe.reader_open.load(Ordering::Acquire) {
                    return self.broken_pipe(caller, written);
                }
                let remaining = source.len() - source.copied();
                // Step 2: 若为大于 `PIPE_BUF` 的普通写，或当前可用总空间足以一次性容纳整个小请求（`<= PIPE_BUF`），则在本次持锁期内循环追加写入并唤醒读端。
                if !small || remaining <= ring.space() {
                    while source.copied() < source.len() && 0 < ring.space() {
                        if !self.pipe.reader_open.load(Ordering::Acquire) {
                            return self.broken_pipe(caller, written);
                        }
                        let expected =
                            (source.len() - source.copied()).min(if ring.merge_space() == 0 {
                                PIPE_BUF
                            } else {
                                ring.merge_space()
                            });
                        let result = ring.append(source);
                        self.pipe.publish(
                            &ring,
                            result.as_ref().is_ok_and(|count| 0 < *count),
                            false,
                        );
                        match result {
                            Ok(count) => {
                                assert!(0 < count, "pipe copyin made no progress");
                                written += count;
                                if count < expected {
                                    let error = source
                                        .pull(&mut [])
                                        .expect_err("short pipe copyin without error");
                                    return Self::progress(written, error);
                                }
                            }
                            Err(error) => return Self::progress(written, error),
                        }
                    }
                }
                if source.copied() == source.len() {
                    return Ok(written);
                }
                if self.flags() & O_NONBLOCK != 0 {
                    return Self::progress(written, FsError::WouldBlock);
                }
                if interrupted {
                    return Self::progress(written, FsError::Interrupted);
                }
            }

            // Step 3: 缓冲区空间不足以完成本次写入，释放 `ring` 锁后注册等待队列或进入睡眠等待读端消费腾出空间。
            if let Some((context, _)) = &waiting {
                interrupted = context.park_until(matches!(caller, FileIoContext::User(_)), None)
                    == WaitResult::Interrupted;
            } else {
                waiting = Some(match self.prepare_wait() {
                    Ok(waiting) => waiting,
                    Err(error) => return Self::progress(written, error),
                });
            }
        }
    }

    /// 匿名管道不支持带偏移量的随机写入，`pwrite` 固定返回 `FsError::IllegalSeek`（`ESPIPE`）。
    fn write_at_from(
        &self,
        _offset: u64,
        _source: &mut WriteBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::IllegalSeek)
    }
}

impl Drop for PipeEnd {
    /// 当该方向打开文件描述的最后一个强引用被释放时：先同步注销挂载在本端点上的所有 `epoll` 监听项，再将对应端点的打开标志置为 `false` 并向对端广播 `POLLERR`（读端关闭）或 `POLLHUP`（写端关闭）。
    fn drop(&mut self) {
        self.watches.release();
        if self.read {
            self.pipe.reader_open.store(false, Ordering::Release);
            self.pipe.write_head.notify(PollMask::POLLERR);
        } else {
            self.pipe.writer_open.store(false, Ordering::Release);
            self.pipe.read_head.notify(PollMask::POLLHUP);
        }
    }
}
