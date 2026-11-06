//! 匿名 UNIX 域套接字对（`socketpair`）的双工通道实现，支持字节流（`SOCK_STREAM`）与有序数据包（`SOCK_SEQPACKET`）。
//!
//! ## 模块职责与架构定位
//! 本模块实现 `AF_UNIX` 匿名套接字对的底层数据通道（[`UnixPair`]）与单侧端点（[`UnixEndpoint`]）：
//! - 上游由 [`SocketFile::try_new_unix_pair`](crate::net::socket::SocketFile::try_new_unix_pair) 与 [`NetStack::try_new_unix_pair`] 调用以创建成对的文件描述符，并接收来自 [`SocketFile`](crate::net::socket::SocketFile) 的读写、轮询、半关闭（`shutdown`）和套接字选项请求。
//! - 下游复用 [`PipeBuffer`] 管理流式物理页环形缓冲区，使用 [`PollWaitHead`] 驱动事件唤醒，并通过 [`NetStack`] 弱引用在双端均关闭后从 [`NetTables`](crate::net::table::NetTables) 注销自身。
//! - [`NetTables`](crate::net::table::NetTables) 仅持有 [`UnixPair`] 共享核心，不反向持有 [`UnixEndpoint`] 或 [`SocketFile`](crate::net::socket::SocketFile)，避免循环引用导致内存泄漏。
//!
//! ## 核心设计与位域布局
//! ### 1. 双工通道与方向隔离
//! ```text
//! UnixEndpoint (side = 0)                             UnixEndpoint (side = 1)
//!   |-- send() ---------> directions[0] (0 -> 1) ---------> receive() --|
//!   |-- receive() <------ directions[1] (1 -> 0) <--------- send() -----|
//! ```
//! `directions[s]` 专门负责以 `s` 为发送方、`1 - s` 为接收方的单向数据流。两个方向的 `SleepMutex` 完全独立，一侧在拷贝用户态内存发生缺页睡眠时不会阻塞反方向的数据收发。
//!
//! ### 2. `Direction::snapshot` 原子状态字位域编码
//! 为了让 `poll` 在无需获取 `Direction::state` 睡眠锁的情况下读取就绪状态，每个方向将缓冲区状态编码在单个 `AtomicUsize` 中（`STREAM_CAPACITY = 16384`，`SPACE_SHIFT = 15`）：
//! ```text
//!  usize 高位                                                      Bit 0
//! +------------------+-----------------+---------------+----------------+
//! |       ...        | RECEIVER_CLOSED | SENDER_CLOSED |   space (15b)  | readable (15b) |
//! +------------------+-----------------+---------------+----------------+
//!                      Bit 31            Bit 30          Bits 29..15      Bits 14..0
//! ```
//! - `readable`（`BYTES_MASK`，Bits `0..SPACE_SHIFT`）：流模式下为可读字节数，有序记录模式下为队列中的消息条数（使空记录也能触发可读事件）。
//! - `space`（`SPACE_MASK`，Bits `SPACE_SHIFT..2*SPACE_SHIFT`）：当前方向剩余可写字节额度（当记录槽满时强制为 `0`）。
//! - `SENDER_CLOSED`（Bit `2*SPACE_SHIFT`）：发送端已关闭写方向。
//! - `RECEIVER_CLOSED`（Bit `2*SPACE_SHIFT + 1`）：接收端已关闭读方向。
//!
//! ## 锁获取顺序（Lock Ordering）与并发契约
//! 1. **单向锁不嵌套**：任何线程在同一时刻最多持有一个方向的 `Direction::state`（`SleepMutex`）。`shutdown(SHUT_RDWR)` 或 `close_side` 需要关闭两个方向时，必须先释放第一个方向的锁并完成事件唤醒，再获取第二个方向的锁，从而消除双端同时关闭时的 AB-BA 死锁。
//! 2. **方向睡眠锁优先于错误自旋锁**：`Direction::state`（`SleepMutex`） ➔ `UnixPair::errors[side]`（`SpinLock`）。
//! 3. **锁外通知与注销**：`PollWaitHead::notify` 以及向 `NetTables` 注销 `UnixPair` 的操作必须在释放 `Direction::state` 之后执行。

use alloc::collections::VecDeque;
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use crate::fs::pipe::{PipeBuffer, PIPE_BUF};
use crate::fs::poll::{PollMask, PollTable, PollWaitHead};
use crate::fs::{FsError, ReadBuffer, WriteBuffer};
use crate::net::address::SocketAddress;
use crate::net::socket::{
    SocketTimeouts, MSG_TRUNC, SHUT_RD, SHUT_RDWR, SHUT_WR, SOCK_SEQPACKET, SOCK_STREAM,
};
use crate::net::{EndpointSlot, IoProgress, MessageResult, NetStack, NetStackRef};
use crate::sync::{SleepMutex, SpinLock};

/// 单个流式方向包含的管道页槽数量。
const STREAM_PIPE_BUFFERS: usize = 4;
/// 单个方向的最大字节容量（单位：字节，等于 `4 * PIPE_BUF = 16384`）。
const STREAM_CAPACITY: usize = STREAM_PIPE_BUFFERS * PIPE_BUF;
/// `SOCK_SEQPACKET` 模式下单个方向最多可缓存的离散记录条数。
const RECORD_CAPACITY: usize = 32;

/// 匿名 UNIX 套接字对的传输语义类型，底层判别值与系统调用 ABI 常量一致。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u32)]
pub enum UnixSocketType {
    /// 面向连接的无边界字节流（对应 `SOCK_STREAM`）。
    Stream = SOCK_STREAM,
    /// 面向连接、保留消息边界且按序递交的数据包（对应 `SOCK_SEQPACKET`）。
    SeqPacket = SOCK_SEQPACKET,
}

/// `Direction::snapshot` 中编码 `readable` 与 `space` 字段各自占用的位宽（足以容纳 `0..=STREAM_CAPACITY`）。
const SPACE_SHIFT: u32 = usize::BITS - STREAM_CAPACITY.leading_zeros();
/// 提取 `Direction::snapshot` 低位可读计数（字节数或记录数）的位掩码。
const BYTES_MASK: usize = (1 << SPACE_SHIFT) - 1;
/// 提取 `Direction::snapshot` 中部剩余可写字节空间的位掩码。
const SPACE_MASK: usize = BYTES_MASK << SPACE_SHIFT;
/// `Direction::snapshot` 中标记发送侧已关闭的标志位。
const SENDER_CLOSED: usize = 1 << (2 * SPACE_SHIFT);
/// `Direction::snapshot` 中标记接收侧已关闭的标志位。
const RECEIVER_CLOSED: usize = 1 << (2 * SPACE_SHIFT + 1);
/// `Direction::snapshot` 中任意一端已关闭的组合掩码（`SENDER_CLOSED | RECEIVER_CLOSED`）。
const CLOSED_MASK: usize = SENDER_CLOSED | RECEIVER_CLOSED;
const _: () = {
    assert!(STREAM_CAPACITY <= BYTES_MASK);
    assert!(SPACE_MASK & BYTES_MASK == 0);
    assert!(CLOSED_MASK & (BYTES_MASK | SPACE_MASK) == 0);
    assert!(2 * SPACE_SHIFT + 1 < usize::BITS);
};

/// 匿名 UNIX 套接字对中的单向传输通道，独立串行化该方向的数据写入、读取及半关闭状态变更。
struct Direction {
    /// 保护本方向底层缓冲区与关闭标志的睡眠互斥锁。
    state: SleepMutex<DirectionState>,
    /// 供 `poll` 无锁读取的原子状态快照，按位编码可读量、可写空间与关闭标志。
    snapshot: AtomicUsize,
}

/// 单向通道受 `Direction::state` 睡眠锁保护的可变内部状态。
struct DirectionState {
    /// 承载字节流或有序记录队列的底层存储。
    buffer: DirectionBuffer,
    /// 发送侧是否已关闭本方向的写入（例如调用了 `SHUT_WR` 或关闭了发送侧端点）。
    sender_closed: bool,
    /// 接收侧是否已关闭本方向的读取（例如调用了 `SHUT_RD` 或关闭了接收侧端点）。
    receiver_closed: bool,
}

/// 单向通道的底层数据存储实现，在创建时按套接字类型固定为字节流或记录队列。
enum DirectionBuffer {
    /// `SOCK_STREAM` 字节流缓冲区，直接复用基于物理页的管道环形队列。
    Stream(PipeBuffer<STREAM_PIPE_BUFFERS>),
    /// `SOCK_SEQPACKET` 有序数据包队列，按条保存消息并统计已占用的字节额度。
    SeqPacket {
        /// 先进先出的离散数据包队列（容量固定预分配为 `RECORD_CAPACITY`）。
        records: VecDeque<Vec<u8>>,
        /// 当前队列中所有数据包占用的总字节预算（单位：字节；空包按 1 字节计费以防无限堆积）。
        charge: usize,
    },
}

impl DirectionBuffer {
    /// 返回当前方向的可读计量（流模式返回可读字节数，记录模式返回队列中的消息条数，使零长记录也能表示为非零可读）。
    fn readable(&self) -> usize {
        match self {
            Self::Stream(ring) => ring.len(),
            Self::SeqPacket { records, .. } => records.len(),
        }
    }

    /// 返回当前方向剩余可写字节空间（单位：字节）；记录模式下同时受 `RECORD_CAPACITY` 条数上限与 `STREAM_CAPACITY` 字节预算限制。
    fn space(&self) -> usize {
        match self {
            Self::Stream(ring) => ring.space(),
            Self::SeqPacket { records, charge } if records.len() < RECORD_CAPACITY => {
                STREAM_CAPACITY - charge
            }
            Self::SeqPacket { .. } => 0,
        }
    }

    /// 清空本方向所有尚未被读取的数据并重置计费额度（在接收侧关闭连接时调用）。
    fn clear(&mut self) {
        match self {
            Self::Stream(ring) => ring.clear(),
            Self::SeqPacket { records, charge } => {
                records.clear();
                *charge = 0;
            }
        }
    }
}

impl Direction {
    /// 初始化单向传输通道；流模式按需延迟分配物理页，记录模式预先分配固定长度的队列槽位。
    fn new(kind: UnixSocketType) -> Result<Self, FsError> {
        let buffer = match kind {
            UnixSocketType::Stream => DirectionBuffer::Stream(PipeBuffer::new()),
            UnixSocketType::SeqPacket => {
                let mut records = VecDeque::new();
                records
                    .try_reserve_exact(RECORD_CAPACITY)
                    .map_err(|_| FsError::NoMemory)?;
                DirectionBuffer::SeqPacket { records, charge: 0 }
            }
        };
        Ok(Self {
            state: SleepMutex::new(DirectionState {
                buffer,
                sender_closed: false,
                receiver_closed: false,
            }),
            snapshot: AtomicUsize::new(STREAM_CAPACITY << SPACE_SHIFT),
        })
    }

    /// 在持有 `Direction::state` 锁期间，将当前可读量、剩余空间与关闭位打包发布到 `snapshot` 原子变量中。
    fn publish(&self, state: &DirectionState) {
        let snapshot = state.buffer.readable()
            | (state.buffer.space() << SPACE_SHIFT)
            | if state.sender_closed {
                SENDER_CLOSED
            } else {
                0
            }
            | if state.receiver_closed {
                RECEIVER_CLOSED
            } else {
                0
            };
        self.snapshot.store(snapshot, Ordering::Release);
    }
}

/// 匿名 UNIX 套接字对的共享核心对象，持有双向通道、事件等待队列头与双端生命周期状态。
pub struct UnixPair {
    /// 该套接字对在网络域中的唯一注册 ID（取第一侧端点的 ID）。
    id: u64,
    /// 所属网络域的弱引用，用于在双端均关闭后从 `NetTables` 移除自身，同时避免与网络域形成强引用环。
    net: Weak<NetStack>,
    /// 传输语义类型（`Stream` 或 `SeqPacket`）。
    kind: UnixSocketType,
    /// 两条方向独立的单向数据通道：`directions[0]` 为 `0 -> 1`，`directions[1]` 为 `1 -> 0`。
    directions: [Direction; 2],
    /// 双端各自的轮询事件等待队列头，`sources[side]` 唤醒等待在 `side` 端点上的任务。
    sources: [Arc<PollWaitHead>; 2],
    /// 双端各自待由 `SO_ERROR` 或下次收发取走的异步错误（如对端丢弃未读数据关闭时产生的 `ECONNRESET`）。
    errors: [SpinLock<Option<FsError>>; 2],
    /// 标记双端各自的 [`UnixEndpoint`] 是否已执行关闭流程。
    closed: [AtomicBool; 2],
    /// 标记所属网络域是否已执行全局同步停机（`stop_sync`）。
    stopped: AtomicBool,
}

/// 匿名 UNIX 套接字对的单侧协议端点，由对应的 [`SocketFile`](crate::net::socket::SocketFile) 独占持有。
pub struct UnixEndpoint {
    /// 本端点的网络域唯一 ID（同时作为匿名套接字的 `st_ino`）。
    id: u64,
    /// 本端点在 [`UnixPair`] 中的侧别下标（`0` 或 `1`）。
    side: usize,
    /// 指向所属共享套接字对核心的强引用。
    pair: Arc<UnixPair>,
    /// 保存 `SO_REUSEADDR` 选项的布尔状态。
    reuse: AtomicBool,
    /// 保护本侧收发超时配置（`SO_RCVTIMEO` / `SO_SNDTIMEO`）的自旋锁。
    timeouts: SpinLock<SocketTimeouts>,
    /// 本端点占用的网络域活跃端点配额槽，在 `UnixEndpoint` 最终析构时自动归还。
    _slot: EndpointSlot,
}

impl UnixPair {
    /// 分配并初始化共享的 [`UnixPair`] 及其两侧 [`UnixEndpoint`]；若任一内存分配失败则整体回滚，不会向网络域遗留半成品。
    pub fn try_new(
        net: &NetStackRef,
        kind: UnixSocketType,
        ids: [u64; 2],
        slots: [EndpointSlot; 2],
    ) -> Result<(Arc<Self>, [Arc<UnixEndpoint>; 2]), FsError> {
        let pair = Arc::try_new(Self {
            id: ids[0],
            kind,
            net: Arc::downgrade(net),
            directions: [Direction::new(kind)?, Direction::new(kind)?],
            sources: [
                Arc::try_new(PollWaitHead::new()).map_err(|_| FsError::NoMemory)?,
                Arc::try_new(PollWaitHead::new()).map_err(|_| FsError::NoMemory)?,
            ],
            errors: [SpinLock::new(None), SpinLock::new(None)],
            closed: [AtomicBool::new(false), AtomicBool::new(false)],
            stopped: AtomicBool::new(false),
        })
        .map_err(|_| FsError::NoMemory)?;

        let [first_slot, second_slot] = slots;
        let first = Arc::try_new(UnixEndpoint {
            id: ids[0],
            side: 0,
            pair: Arc::clone(&pair),
            reuse: AtomicBool::new(false),
            timeouts: SpinLock::new(SocketTimeouts::default()),
            _slot: first_slot,
        })
        .map_err(|_| FsError::NoMemory)?;
        let second = Arc::try_new(UnixEndpoint {
            id: ids[1],
            side: 1,
            pair: Arc::clone(&pair),
            reuse: AtomicBool::new(false),
            timeouts: SpinLock::new(SocketTimeouts::default()),
            _slot: second_slot,
        })
        .map_err(|_| FsError::NoMemory)?;
        Ok((pair, [first, second]))
    }

    /// 返回该套接字对在 [`NetTables`](crate::net::table::NetTables) 中登记的唯一 ID（等于第 0 侧端点的 ID）。
    pub fn id(&self) -> u64 {
        self.id
    }

    /// 根据双向通道的原子快照（`snapshot`）与本侧错误槽计算当前就绪的 `PollMask`，全程无需获取方向睡眠锁。
    fn mask(&self, side: usize) -> PollMask {
        let incoming = self.directions[1 - side].snapshot.load(Ordering::Acquire);
        let outgoing = self.directions[side].snapshot.load(Ordering::Acquire);
        let read_closed = incoming & CLOSED_MASK != 0;
        let write_closed = outgoing & CLOSED_MASK != 0;
        let mut mask = PollMask::default();
        if incoming & BYTES_MASK != 0 || read_closed {
            mask |= PollMask::POLLIN | PollMask::POLLRDNORM;
        }
        if read_closed {
            mask |= PollMask::POLLRDHUP;
        }
        // 即使写方向已关闭，也必须报告 POLLOUT，以便唤醒阻塞在写等待上的任务并立即返回 EPIPE。
        if 0 < (outgoing & SPACE_MASK) >> SPACE_SHIFT || write_closed {
            mask |= PollMask::POLLOUT | PollMask::POLLWRNORM;
        }
        if read_closed && write_closed {
            mask |= PollMask::POLLHUP;
        }
        if self.errors[side].lock().is_some() {
            mask |= PollMask::POLLERR;
        }
        mask
    }

    /// 关闭 `sender` 发送方向的发送端或接收端标志，按需丢弃未读数据并在释放方向睡眠锁后触发事件通知。
    fn close_direction(
        &self,
        sender: usize,
        close_sender: bool,
        close_receiver: bool,
        discard: bool,
    ) {
        let receiver = 1 - sender;
        let direction = &self.directions[sender];
        let (changed, reset) = {
            let mut state = direction.state.lock();
            let was_closed = state.sender_closed || state.receiver_closed;
            let reset = discard && state.buffer.readable() != 0;
            state.sender_closed |= close_sender;
            state.receiver_closed |= close_receiver;
            if discard {
                state.buffer.clear();
            }
            if reset {
                self.errors[sender]
                    .lock()
                    .get_or_insert(FsError::ConnectionReset);
            }
            direction.publish(&state);
            (
                !was_closed && (state.sender_closed || state.receiver_closed),
                reset,
            )
        };

        if changed {
            self.sources[receiver]
                .notify(PollMask::POLLIN | PollMask::POLLRDNORM | PollMask::POLLRDHUP);
            self.sources[sender].notify(PollMask::POLLOUT | PollMask::POLLWRNORM);
            if self.mask(receiver).contains(PollMask::POLLHUP) {
                self.sources[receiver].notify(PollMask::POLLHUP);
            }
            if self.mask(sender).contains(PollMask::POLLHUP) {
                self.sources[sender].notify(PollMask::POLLHUP);
            }
        }
        if reset {
            self.sources[sender].notify(PollMask::POLLERR);
        }
    }

    /// 关闭 `side` 侧端点：丢弃对端发往本侧的未读数据（必要时向对端记录 `ECONNRESET`），关闭本侧发送方向，并在双端均关闭后从网络域表中注销。
    fn close_side(&self, side: usize) {
        self.close_direction(1 - side, true, true, true);
        self.close_direction(side, true, true, false);
        self.closed[side].store(true, Ordering::Release);
        if self.closed[1 - side].load(Ordering::Acquire) {
            if let Some(net) = self.net.upgrade() {
                let removed = net.tables().take_unix(self.id);
                drop(removed);
            }
        }
    }

    /// 在网络域停机时同步停止该套接字对：设置 `stopped` 标志并依次关闭两侧方向，确保所有正在执行的拷贝与快照更新均已完成。
    pub(crate) fn stop_sync(&self) {
        self.stopped.store(true, Ordering::Release);
        self.close_side(0);
        self.close_side(1);
    }
}

impl UnixEndpoint {
    /// 读取本侧端点当前的收发超时配置快照，无需等待数据通道睡眠锁。
    pub(crate) fn timeouts(&self) -> SocketTimeouts {
        *self.timeouts.lock()
    }

    /// 更新本侧端点的接收或发送超时配置（通过 `dup` / `fork` 共享同一打开文件描述的描述符共用此设置，不影响对端）。
    pub(crate) fn set_timeout(&self, receive: bool, timeout: Option<u64>) {
        self.timeouts.lock().set(receive, timeout);
    }

    /// 返回本侧端点的唯一 ID（双工通道的两侧各自拥有不同的 ID）。
    pub fn id(&self) -> u64 {
        self.id
    }

    /// 返回本端点所属套接字对的传输类型（`Stream` 或 `SeqPacket`）。
    pub fn socket_type(&self) -> UnixSocketType {
        self.pair.kind
    }

    /// 从入站方向（`1 - self.side`）读取字节流数据到 `target`；若非 `MSG_PEEK` 且释放了缓冲空间，则在锁外唤醒对端的写等待任务。
    pub fn receive(&self, target: &mut ReadBuffer<'_>, peek: bool) -> Result<IoProgress, FsError> {
        let direction = &self.pair.directions[1 - self.side];
        let mut state = direction.state.lock();
        self.check_open()?;
        if target.copied() == target.len() {
            return Ok(IoProgress::Ready(0));
        }
        let DirectionBuffer::Stream(ring) = &mut state.buffer else {
            unreachable!("record receive uses complete messages");
        };
        if ring.is_empty() {
            if let Some(error) = self.pair.errors[self.side].lock().take() {
                return Err(error);
            }
            return Ok(if state.sender_closed || state.receiver_closed {
                IoProgress::Eof
            } else {
                IoProgress::Blocked
            });
        }

        let space = ring.space();
        let before = target.copied();
        let result = ring.receive(target, peek);
        let copied = target.copied() - before;
        if !peek && 0 < copied {
            direction.publish(&state);
        }
        let writable =
            !state.sender_closed && !state.receiver_closed && space < state.buffer.space();
        drop(state);

        if !peek && writable {
            self.pair.sources[1 - self.side].notify(PollMask::POLLOUT | PollMask::POLLWRNORM);
        }
        result.map(IoProgress::Ready)
    }

    /// 将已在锁外拷贝至内核的字节切片写入出站方向（`self.side`）的环形缓冲区，并在锁外通知对端可读。
    pub fn send(&self, source: &[u8]) -> Result<IoProgress, FsError> {
        let direction = &self.pair.directions[self.side];
        let mut state = direction.state.lock();
        self.check_open()?;
        if state.sender_closed || state.receiver_closed {
            return Err(FsError::BrokenPipe);
        }
        if let Some(error) = self.pair.errors[self.side].lock().take() {
            return Err(error);
        }
        if source.is_empty() {
            return Ok(IoProgress::Ready(0));
        }

        let DirectionBuffer::Stream(ring) = &mut state.buffer else {
            unreachable!("record send uses owned messages");
        };
        let mut source = WriteBuffer::kernel(source);
        let mut accepted = 0;
        while source.copied() < source.len() && 0 < ring.space() {
            match ring.append(&mut source) {
                Ok(count) => {
                    assert!(0 < count, "UNIX copyin made no progress");
                    accepted += count;
                }
                Err(error) => {
                    if accepted == 0 {
                        return Err(error);
                    }
                    break;
                }
            }
        }
        if accepted == 0 {
            return Ok(IoProgress::Blocked);
        }
        direction.publish(&state);
        drop(state);

        self.pair.sources[1 - self.side].notify(PollMask::POLLIN | PollMask::POLLRDNORM);
        Ok(IoProgress::Ready(accepted))
    }

    /// 在 `SOCK_SEQPACKET` 模式下发送一条完整的数据包；仅当出站队列有足够空间时才通过 `take()` 移交 `payload` 所有权。
    pub fn send_record(&self, payload: &mut Option<Vec<u8>>) -> Result<IoProgress, FsError> {
        let direction = &self.pair.directions[self.side];
        let mut state = direction.state.lock();
        self.check_open()?;
        if state.sender_closed || state.receiver_closed {
            return Err(FsError::BrokenPipe);
        }
        if let Some(error) = self.pair.errors[self.side].lock().take() {
            return Err(error);
        }

        let length = payload
            .as_ref()
            .expect("UNIX record already submitted")
            .len();
        if STREAM_CAPACITY < length {
            return Err(FsError::MessageTooLarge);
        }
        let needed = length.max(1);
        if state.buffer.space() < needed {
            return Ok(IoProgress::Blocked);
        }

        let DirectionBuffer::SeqPacket { records, charge } = &mut state.buffer else {
            unreachable!("stream send uses byte prefixes");
        };
        records.push_back(payload.take().expect("UNIX record already submitted"));
        *charge += needed;
        direction.publish(&state);
        drop(state);

        self.pair.sources[1 - self.side].notify(PollMask::POLLIN | PollMask::POLLRDNORM);
        Ok(IoProgress::Ready(length))
    }

    /// 在 `SOCK_SEQPACKET` 模式下从入站队列接收一条完整记录；若目标缓冲区不足则截断并标记 `MSG_TRUNC`，非 `MSG_PEEK` 时弹出整条记录并归还配额。
    pub fn receive_record(
        &self,
        target: &mut ReadBuffer<'_>,
        peek: bool,
    ) -> Result<Option<MessageResult>, FsError> {
        let direction = &self.pair.directions[1 - self.side];
        let mut state = direction.state.lock();
        self.check_open()?;
        if state.buffer.readable() == 0 {
            if let Some(error) = self.pair.errors[self.side].lock().take() {
                return Err(error);
            }
            return Ok(
                (state.sender_closed || state.receiver_closed).then_some(MessageResult {
                    copied: 0,
                    original_len: 0,
                    source: None,
                    flags: 0,
                }),
            );
        }

        let writable = !state.sender_closed && !state.receiver_closed;
        let DirectionBuffer::SeqPacket { records, charge } = &mut state.buffer else {
            unreachable!("stream receive uses byte prefixes");
        };
        let payload = records.front().expect("UNIX readable record");
        let original_len = payload.len();
        let copied = target.push(payload);
        if !peek {
            *charge -= original_len.max(1);
            records.pop_front();
            direction.publish(&state);
        }
        drop(state);

        if !peek && writable {
            self.pair.sources[1 - self.side].notify(PollMask::POLLOUT | PollMask::POLLWRNORM);
        }
        copied.map(|copied| {
            Some(MessageResult {
                copied,
                original_len,
                source: None,
                flags: if copied < original_len { MSG_TRUNC } else { 0 },
            })
        })
    }

    /// 按 `how`（`SHUT_RD`、`SHUT_WR` 或 `SHUT_RDWR`）关闭本端点的读和/或写方向；分步获取单向锁以避免双向死锁。
    pub fn shutdown(&self, how: u32) -> Result<(), FsError> {
        if SHUT_RDWR < how {
            return Err(FsError::InvalidOperation);
        }
        self.check_open()?;
        if how == SHUT_RD || how == SHUT_RDWR {
            self.pair.close_direction(1 - self.side, false, true, false);
        }
        if how == SHUT_WR || how == SHUT_RDWR {
            self.pair.close_direction(self.side, true, false, false);
        }
        Ok(())
    }

    /// 返回本侧的套接字名称；匿名 UNIX 套接字固定返回未命名标识 `SocketAddress::UnixUnnamed`。
    pub fn local_address(&self) -> Result<SocketAddress, FsError> {
        self.check_open()?;
        Ok(SocketAddress::UnixUnnamed)
    }

    /// 返回对端的套接字名称；只要本侧尚未关闭，即使对端已关闭也仍返回 `SocketAddress::UnixUnnamed`。
    pub fn peer_address(&self) -> Result<SocketAddress, FsError> {
        self.check_open()?;
        Ok(SocketAddress::UnixUnnamed)
    }

    /// 向 `PollTable` 注册本侧的事件等待队列头，并无锁读取当前的就绪事件掩码。
    pub fn poll(&self, table: Option<&mut PollTable>) -> Result<PollMask, FsError> {
        if let Some(table) = table {
            table.register(&self.pair.sources[self.side])?;
        }
        Ok(self.pair.mask(self.side))
    }

    /// 取出并清空本侧挂起的异步错误（供 `getsockopt(SO_ERROR)` 调用）。
    pub fn take_error(&self) -> Option<FsError> {
        self.pair.errors[self.side].lock().take()
    }

    /// 返回接收缓冲区的固定字节容量（`STREAM_CAPACITY`）。
    pub fn receive_capacity(&self) -> usize {
        STREAM_CAPACITY
    }

    /// 返回发送缓冲区的固定字节容量（`STREAM_CAPACITY`）。
    pub fn send_capacity(&self) -> usize {
        STREAM_CAPACITY
    }

    /// 读取本侧保存的 `SO_REUSEADDR` 选项状态。
    pub fn reuse_address(&self) -> bool {
        self.reuse.load(Ordering::Acquire)
    }

    /// 在方向锁保护下校验端点未关闭，并更新本侧的 `SO_REUSEADDR` 选项状态。
    pub fn set_reuse_address(&self, reuse: bool) -> Result<(), FsError> {
        let _direction = self.pair.directions[self.side].state.lock();
        self.check_open()?;
        self.reuse.store(reuse, Ordering::Release);
        Ok(())
    }

    /// 检查本侧端点及所属网络域是否仍处于活跃打开状态；若已关闭或已停机则返回 `FsError::NotConnected`。
    fn check_open(&self) -> Result<(), FsError> {
        if self.pair.stopped.load(Ordering::Acquire)
            || self.pair.closed[self.side].load(Ordering::Acquire)
        {
            return Err(FsError::NotConnected);
        }
        Ok(())
    }

    /// 关闭本侧端点并唤醒双端等待队列；当最后一个 `SocketFile` 释放时调用，不阻塞等待对端关闭。
    pub fn close(&self) {
        self.pair.close_side(self.side);
    }
}

impl Drop for UnixEndpoint {
    /// 析构时确保本侧通道已关闭，随后由字段析构自动归还 `EndpointSlot` 配额与 `UnixPair` 强引用。
    fn drop(&mut self) {
        self.close();
    }
}
