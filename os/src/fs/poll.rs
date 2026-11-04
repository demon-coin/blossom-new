use alloc::{sync::Arc, vec::Vec};
use core::{
    ops::{BitAnd, BitAndAssign, BitOr, BitOrAssign, BitXor, BitXorAssign, Not},
    pin::Pin,
};

use crate::time::{self, MonotonicInstant};
use crate::timer::{Timer, TimerKind};
use crate::wait::{WaitResult, Waiter, Waker};
use crate::{fs::FsError, sync::SpinLock};

/// 持续登记只保存通知事实，唯一一次等待仅在真正 park 的窗口存在。
pub struct PollContext {
    state: SpinLock<PollContextState>,
}

struct PollContextState {
    triggered: bool,
    waker: Option<Waker>,
}

impl PollContext {
    /// 在确定需要等待后准备通知事实；快速 I/O 不需要建立本对象。
    pub fn try_new() -> Result<Arc<Self>, FsError> {
        Arc::try_new(Self {
            state: SpinLock::new(PollContextState {
                triggered: false,
                waker: None,
            }),
        })
        .map_err(|_| FsError::NoMemory)
    }

    /// 在下一轮业务扫描之前清除旧事实，不得在扫描之后调用。
    pub fn clear_triggered(&self) {
        self.state.lock().triggered = false;
    }

    /// 观察通知事实，不查询 provider，也不建立一次等待。
    #[cfg(feature = "ktest")]
    pub fn triggered(&self) -> bool {
        self.state.lock().triggered
    }

    /// 在无业务锁、无用户复制的窗口安装唯一 Waiter；期限和通知只唤醒本轮重查。
    /// 无期限不建立定时器；返回先撤上下文 Waker、结束 Waiter，再确认实际 Timer 的回调尾部。
    pub fn park_until(
        &self,
        interruptible: bool,
        deadline: Option<MonotonicInstant>,
    ) -> WaitResult {
        let waiter = if interruptible {
            Waiter::new_interruptible()
        } else {
            Waiter::new()
        };
        let waker = core::pin::pin!(deadline.map(|_| waiter.waker()));
        let timer = core::pin::pin!(waker.as_ref().as_pin_ref().map(|waker| {
            Timer::new(
                crate::cpu::current().id,
                TimerKind::Precise,
                poll_timeout,
                waker.get_ref() as *const Waker as usize,
            )
        }));
        if let Some(limit) = deadline {
            let timer = timer
                .as_ref()
                .as_pin_ref()
                .expect("poll deadline has no timer");
            // SAFETY: Timer 和 Waker 均固定在本栈，回调只读取 Waker；每条返回路径
            // 先撤上下文并结束活等待，再同步确认回调尾部，期间两份存储不移动或退栈。
            unsafe { timer.arm_borrowed(limit) }.expect("fresh precise poll timer rejected");
        }
        {
            let mut state = self.state.lock();
            assert!(state.waker.is_none(), "poll context already parked");
            state.waker = Some(waiter.waker());
            if state.triggered || deadline.is_some_and(|limit| limit <= time::now()) {
                state.waker.as_ref().unwrap().notify();
            }
        }
        let result = if interruptible {
            waiter.park_signal()
        } else {
            waiter.park();
            WaitResult::Woken
        };
        let installed = self.state.lock().waker.take();
        drop(installed);
        drop(waiter);
        if let Some(timer) = timer.as_ref().as_pin_ref() {
            timer.cancel_sync();
        }
        result
    }
}

/// 精确硬 IRQ 只通知固定栈 Waker，调用者同步停止节点后才结束上下文存储。
fn poll_timeout(timer: Pin<&Timer>) {
    // SAFETY: park_until 以固定栈 Waker 地址构造 context；同步 cancel_sync 确认
    // 真实回调尾部以前保持该存储有效，回调不取得 provider 或用户访问锁。
    let waker = unsafe { &*(timer.context() as *const Waker) };
    waker.notify();
}

impl PollNotify for PollContext {
    /// 源短锁内先保存事实，再唤醒当前窗口的等待者；没有 Waiter 时事实继续保留。
    fn notify(&self, _mask: PollMask) {
        let mut state = self.state.lock();
        state.triggered = true;
        if let Some(waker) = &state.waker {
            waker.notify();
        }
    }
}

/// 保存 poll/select 共同使用的目标 UAPI 位，并支持直接位运算。
#[repr(transparent)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct PollMask(pub u32);

impl PollMask {
    /// 表示有数据可读。
    pub const POLLIN: Self = Self(0x001);
    /// 表示有带外数据可读。
    pub const POLLPRI: Self = Self(0x002);
    /// 表示写入不会阻塞。
    pub const POLLOUT: Self = Self(0x004);
    /// 表示文件对象发生错误。
    pub const POLLERR: Self = Self(0x008);
    /// 表示文件对象已挂断。
    pub const POLLHUP: Self = Self(0x010);
    /// 表示文件描述符无效。
    pub const POLLNVAL: Self = Self(0x020);
    /// 表示普通数据可读。
    pub const POLLRDNORM: Self = Self(0x040);
    /// 表示带优先级数据可读。
    pub const POLLRDBAND: Self = Self(0x080);
    /// 表示普通写入不会阻塞。
    pub const POLLWRNORM: Self = Self(0x100);
    /// 表示带优先级写入不会阻塞。
    pub const POLLWRBAND: Self = Self(0x200);
    /// 表示消息事件。
    pub const POLLMSG: Self = Self(0x400);
    /// 表示删除事件。
    pub const POLLREMOVE: Self = Self(0x1000);
    /// 表示对端已半关闭写侧。
    pub const POLLRDHUP: Self = Self(0x2000);

    /// 返回该掩码是否包含给定的全部位。
    pub const fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }

    /// 返回该掩码是否没有任何位。
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }
}

impl BitOr for PollMask {
    type Output = Self;

    /// 合并两个 poll 位集合。
    fn bitor(self, rhs: Self) -> Self::Output {
        Self(self.0 | rhs.0)
    }
}

impl BitOrAssign for PollMask {
    /// 将右侧 poll 位加入当前集合。
    fn bitor_assign(&mut self, rhs: Self) {
        self.0 |= rhs.0;
    }
}

impl BitAnd for PollMask {
    type Output = Self;

    /// 保留两个 poll 位集合的交集。
    fn bitand(self, rhs: Self) -> Self::Output {
        Self(self.0 & rhs.0)
    }
}
impl BitAndAssign for PollMask {
    /// 将当前集合限制为与右侧集合的交集。
    fn bitand_assign(&mut self, rhs: Self) {
        self.0 &= rhs.0;
    }
}

impl BitXor for PollMask {
    type Output = Self;

    /// 保留两个 poll 位集合中只出现一侧的位。
    fn bitxor(self, rhs: Self) -> Self::Output {
        Self(self.0 ^ rhs.0)
    }
}

impl BitXorAssign for PollMask {
    /// 将当前集合与右侧集合做异或。
    fn bitxor_assign(&mut self, rhs: Self) {
        self.0 ^= rhs.0;
    }
}

impl Not for PollMask {
    type Output = Self;

    /// 反转 poll 位集合中的全部 UAPI 位。
    fn not(self) -> Self::Output {
        Self(!self.0)
    }
}

/// 提供实际就绪状态和持续等待源，普通文件的默认 mask 不属于该能力。
pub trait Pollable: Send + Sync {
    /// 首轮登记全部源后观察状态，后续无 table 的调用只查询，不创建一次等待者。
    fn poll(&self, table: Option<&mut PollTable>) -> Result<PollMask, FsError>;
}

/// 接收源就绪短通知的角色；实现不得在回调中等待或依赖源锁。
pub trait PollNotify: Send + Sync {
    /// 接收一次源状态变化通知。
    fn notify(&self, mask: PollMask);
}

struct PollNode {
    sink: Arc<dyn PollNotify>,
    next: SpinLock<Option<Pin<Arc<PollNode>>>>,
}

struct PollWaitState {
    chain: Option<Pin<Arc<PollNode>>>,
}

/// 由实际文件对象持有、串行管理持续 poll 登记链的源头。
pub struct PollWaitHead {
    state: SpinLock<PollWaitState>,
}

impl PollWaitHead {
    /// 建立一个尚未登记任何消费者的持续源头。
    pub const fn new() -> Self {
        Self {
            state: SpinLock::new(PollWaitState { chain: None }),
        }
    }

    /// 在源短锁内顺序通知全部登记者；该路径不分配、不查询对象状态也不取睡眠锁。
    pub fn notify(&self, mask: PollMask) {
        let state = self.state.lock();
        let mut current = state.chain.as_ref().cloned();
        while let Some(node) = current {
            node.sink.notify(mask);
            current = {
                let next = node.next.lock();
                next.as_ref().cloned()
            };
        }
    }

    /// 登记一个持续通知接收者并返回持有源头、节点和源链责任的句柄。
    pub fn register(
        self: &Arc<Self>,
        sink: Arc<dyn PollNotify>,
    ) -> Result<PollRegistration, FsError> {
        let node = Pin::new(
            Arc::try_new(PollNode {
                sink,
                next: SpinLock::new(None),
            })
            .map_err(|_| FsError::NoMemory)?,
        );
        let mut state = self.state.lock();
        {
            let mut next = node.next.lock();
            *next = state.chain.take();
        }
        state.chain = Some(node.clone());
        Ok(PollRegistration {
            head: Arc::clone(self),
            node,
        })
    }

    /// 在源锁内同步摘除目标节点，并将被摘下的源链 ticket 留到锁外释放。
    fn unlink(&self, target: &Pin<Arc<PollNode>>) {
        let detached = {
            let mut state = self.state.lock();
            if state
                .chain
                .as_ref()
                .is_some_and(|node| same_node(node, target))
            {
                let detached = state.chain.take();
                let successor = detached.as_ref().and_then(|node| {
                    let mut next = node.next.lock();
                    next.take()
                });
                state.chain = successor;
                detached
            } else {
                let mut current = state.chain.as_ref().cloned();
                let mut detached = None;
                while let Some(node) = current {
                    let mut next = node.next.lock();
                    if next
                        .as_ref()
                        .is_some_and(|candidate| same_node(candidate, target))
                    {
                        let removed = next.take();
                        let successor = removed.as_ref().and_then(|candidate| {
                            let mut following = candidate.next.lock();
                            following.take()
                        });
                        *next = successor;
                        detached = removed;
                        break;
                    }
                    current = next.as_ref().cloned();
                }
                detached
            }
        };
        drop(detached);
    }
}

/// 一个持续 poll 登记的同步注销句柄。
pub struct PollRegistration {
    head: Arc<PollWaitHead>,
    node: Pin<Arc<PollNode>>,
}

impl Drop for PollRegistration {
    /// 先在源锁内摘链，再在锁外释放源链 ticket 和节点拥有。
    fn drop(&mut self) {
        self.head.unlink(&self.node);
    }
}

/// 比较两个稳定节点的实际地址，不从节点内容重建任何拥有引用。
fn same_node(left: &Pin<Arc<PollNode>>, right: &Pin<Arc<PollNode>>) -> bool {
    core::ptr::eq(Pin::as_ref(left).get_ref(), Pin::as_ref(right).get_ref())
}

/// 负责一次 provider 轮询的有限、可失败源登记集合。
pub struct PollTable {
    sink: Arc<dyn PollNotify>,
    registrations: Vec<PollRegistration>,
}

impl PollTable {
    /// 预留固定登记容量；容量不足时不会在后续首轮登记中隐式扩容。
    pub fn new(sink: Arc<dyn PollNotify>, capacity: usize) -> Result<Self, FsError> {
        let mut registrations = Vec::new();
        registrations
            .try_reserve_exact(capacity)
            .map_err(|_| FsError::NoMemory)?;
        Ok(Self {
            sink,
            registrations,
        })
    }

    /// 首轮登记一个源头；同一源头在本表中只保留一个登记。
    pub fn register(&mut self, head: &Arc<PollWaitHead>) -> Result<(), FsError> {
        if self
            .registrations
            .iter()
            .any(|registration| Arc::ptr_eq(&registration.head, head))
        {
            return Ok(());
        }
        if self.registrations.len() == self.registrations.capacity() {
            return Err(FsError::NoMemory);
        }
        let registration = head.register(Arc::clone(&self.sink))?;
        self.registrations.push(registration);
        Ok(())
    }

    /// 返回本表中已经登记的不同源头数量。
    #[cfg(feature = "ktest")]
    pub fn len(&self) -> usize {
        self.registrations.len()
    }

    /// 返回本表是否尚未登记任何源头。
    #[cfg(feature = "ktest")]
    pub fn is_empty(&self) -> bool {
        self.registrations.is_empty()
    }
}

impl Drop for PollTable {
    /// 逆序销毁登记集合；每个登记先同步撤链，再释放节点、源头和 sink 引用。
    fn drop(&mut self) {
        while let Some(registration) = self.registrations.pop() {
            drop(registration);
        }
    }
}
