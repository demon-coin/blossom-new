//! 匿名 UNIX pair 由两条独立方向拥有字节流或有序记录，根不反持端点或打开对象。

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

const STREAM_PIPE_BUFFERS: usize = 4;
const STREAM_CAPACITY: usize = STREAM_PIPE_BUFFERS * PIPE_BUF;
const RECORD_CAPACITY: usize = 32;

/// 匿名 pair 的真实传输类型，数值与 socket ABI 一致。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u32)]
pub enum UnixSocketType {
    Stream = SOCK_STREAM,
    SeqPacket = SOCK_SEQPACKET,
}

const SPACE_SHIFT: u32 = usize::BITS - STREAM_CAPACITY.leading_zeros();
const BYTES_MASK: usize = (1 << SPACE_SHIFT) - 1;
const SPACE_MASK: usize = BYTES_MASK << SPACE_SHIFT;
const SENDER_CLOSED: usize = 1 << (2 * SPACE_SHIFT);
const RECEIVER_CLOSED: usize = 1 << (2 * SPACE_SHIFT + 1);
const CLOSED_MASK: usize = SENDER_CLOSED | RECEIVER_CLOSED;
const _: () = {
    assert!(STREAM_CAPACITY <= BYTES_MASK);
    assert!(SPACE_MASK & BYTES_MASK == 0);
    assert!(CLOSED_MASK & (BYTES_MASK | SPACE_MASK) == 0);
    assert!(2 * SPACE_SHIFT + 1 < usize::BITS);
};

/// 每个方向只串行该方向的生产、交付及关闭，用户故障不占反向锁。
struct Direction {
    state: SleepMutex<DirectionState>,
    snapshot: AtomicUsize,
}

struct DirectionState {
    buffer: DirectionBuffer,
    sender_closed: bool,
    receiver_closed: bool,
}

/// 流保留原页 FIFO，记录方向仅在创建时准备有限队列槽。
enum DirectionBuffer {
    Stream(PipeBuffer<STREAM_PIPE_BUFFERS>),
    SeqPacket {
        records: VecDeque<Vec<u8>>,
        charge: usize,
    },
}

impl DirectionBuffer {
    /// 返回可读占用；记录数使空消息也能发布读就绪。
    fn readable(&self) -> usize {
        match self {
            Self::Stream(ring) => ring.len(),
            Self::SeqPacket { records, .. } => records.len(),
        }
    }

    /// 记录槽与字节预算共同限制接纳，空消息仍占一个字节额度。
    fn space(&self) -> usize {
        match self {
            Self::Stream(ring) => ring.space(),
            Self::SeqPacket { records, charge } if records.len() < RECORD_CAPACITY => {
                STREAM_CAPACITY - charge
            }
            Self::SeqPacket { .. } => 0,
        }
    }

    /// 放弃本方向全部未读拥有，恢复对应存储的空状态。
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
    /// 流不预分配数据页，记录方向只预留有限消息槽，准备失败不发表 pair。
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

    /// 持方向保护发布可读占用、真实空间及关闭位，poll 不等待数据交付。
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

/// 两个端点共享方向存储与持续源；弱域归属不形成根到打开对象的强引用环。
pub struct UnixPair {
    id: u64,
    net: Weak<NetStack>,
    kind: UnixSocketType,
    /// 下标是生产侧，接收侧为另一个下标。
    directions: [Direction; 2],
    sources: [Arc<PollWaitHead>; 2],
    errors: [SpinLock<Option<FsError>>; 2],
    closed: [AtomicBool; 2],
    stopped: AtomicBool,
}

/// 每侧节点持自己的真实额度，最后节点拥有结束才归还该额度。
pub struct UnixEndpoint {
    id: u64,
    side: usize,
    pair: Arc<UnixPair>,
    reuse: AtomicBool,
    timeouts: SpinLock<SocketTimeouts>,
    _slot: EndpointSlot,
}

impl UnixPair {
    /// 完整准备两个方向 FIFO、持续源及节点；根只在全部成功后取得 pair 拥有。
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

    /// 根表以第一侧端点的本域身份登记这份 pair。
    pub fn id(&self) -> u64 {
        self.id
    }

    /// 从真实方向存储的占用、空间和关闭快照查询就绪，不获取睡眠锁或通知。
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
        // 关闭后的发送会立即报错，仍须允许等待发送的任务重新查询。
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

    /// 一次只取得一个方向保护，提交关闭后解锁再报告真实方向变化。
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

    /// 末端放弃自己的未读字节，保留已交给 peer 环的字节；撤根拥有在方向锁外。
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

    /// 根停止拒绝后继数据操作，逐方向等待当前 copyout、消费及快照提交，再锁外通知关闭。
    /// 外部调用的锁外通知可与停止交错，其端点强拥有保活 pair 和源；这里不等待外部 Arc 或调用返回。
    /// slot 随各节点最后强拥有归还，环随 pair 最后强拥有归还；资源基线须在外部线程真实结束后核对。
    pub(crate) fn stop_sync(&self) {
        self.stopped.store(true, Ordering::Release);
        self.close_side(0);
        self.close_side(1);
    }
}

impl UnixEndpoint {
    /// 每侧端点保存独立控制值，查询不等待任一方向的数据交付锁。
    pub(crate) fn timeouts(&self) -> SocketTimeouts {
        *self.timeouts.lock()
    }

    /// dup 与 fork 共享此端点短锁值，另一侧 pair 的期限保持不变。
    pub(crate) fn set_timeout(&self, receive: bool, timeout: Option<u64>) {
        self.timeouts.lock().set(receive, timeout);
    }

    /// 匿名 inode 身份来自本域端点编号，两侧不会共享同一个编号。
    pub fn id(&self) -> u64 {
        self.id
    }

    /// 两侧始终使用创建 pair 时确定的同一种传输语义。
    pub fn socket_type(&self) -> UnixSocketType {
        self.pair.kind
    }

    /// 接收只占 incoming 方向，共同 FIFO 按实际差量消费；PEEK 不产生容量通知。
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

    /// 已准备的内核前缀逐页提交，接受量只取 append 结果；空输入仍先检查发送关闭。
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

    /// 只在完整记录可接纳时移动同份 payload，等待期间调用者保留原拥有。
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

    /// 接收一条有序记录；PEEK 保留拥有，短复制或 fault 消费整条并归还其额度。
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

    /// 指定方向关闭保留已有 incoming；任一路径均先释放一方向再取得另一方向。
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

    /// 匿名本地名称仅编码 AF_UNIX，不制造路径或 IPv4 地址。
    pub fn local_address(&self) -> Result<SocketAddress, FsError> {
        self.check_open()?;
        Ok(SocketAddress::UnixUnnamed)
    }

    /// pair 的匿名 peer 名称不随对端末关闭改变，本侧拥有结束前仍保留关联。
    pub fn peer_address(&self) -> Result<SocketAddress, FsError> {
        self.check_open()?;
        Ok(SocketAddress::UnixUnnamed)
    }

    /// 首轮只登记本侧持续源，查询不等待任何用户复制或改变事件事实。
    pub fn poll(&self, table: Option<&mut PollTable>) -> Result<PollMask, FsError> {
        if let Some(table) = table {
            table.register(&self.pair.sources[self.side])?;
        }
        Ok(self.pair.mask(self.side))
    }

    /// SO_ERROR 领取真正未读关闭造成的错误，查询消费本身不产生通知。
    pub fn take_error(&self) -> Option<FsError> {
        self.pair.errors[self.side].lock().take()
    }

    /// 查询当前真实固定接收存储容量，不承诺动态修改缓冲。
    pub fn receive_capacity(&self) -> usize {
        STREAM_CAPACITY
    }

    /// 查询 outgoing 环的固定容量，不套用 skb 记账翻倍。
    pub fn send_capacity(&self) -> usize {
        STREAM_CAPACITY
    }

    /// 匿名 socket 保留一般复用选项事实，本版本没有 UNIX 绑定索引。
    pub fn reuse_address(&self) -> bool {
        self.reuse.load(Ordering::Acquire)
    }

    /// 设置本侧一般 socket 选项，不据此发表路径或改变 pair 连接。
    pub fn set_reuse_address(&self, reuse: bool) -> Result<(), FsError> {
        let _direction = self.pair.directions[self.side].state.lock();
        self.check_open()?;
        self.reuse.store(reuse, Ordering::Release);
        Ok(())
    }

    /// 拒绝已经完成末关闭或根停止的对象，根停止不能被残留 OFD 重新使用。
    fn check_open(&self) -> Result<(), FsError> {
        if self.pair.stopped.load(Ordering::Acquire)
            || self.pair.closed[self.side].load(Ordering::Acquire)
        {
            return Err(FsError::NotConnected);
        }
        Ok(())
    }

    /// 末 OFD 在撤 watch 后关闭本侧；此处不等待 peer 节点拥有结束。
    pub fn close(&self) {
        self.pair.close_side(self.side);
    }
}

impl Drop for UnixEndpoint {
    /// 未发表节点也结束自己的方向，最后 slot 与 pair 拥有由字段自然归还。
    fn drop(&mut self) {
        self.close();
    }
}
