//! 粗粒度定时器（[`super::TimerKind::Coarse`]）的八层非级联时间轮实现。
//!
//! # 模块职责与架构定位
//! 本模块作为每核定时器调度基座（`TimerBase::state`）中的粗粒度队列后端，为对微秒/纳秒级精度不敏感、
//! 但高频注册与取消的超时事件（如网络重传、连接保活、看门狗等）提供 $O(1)$ 入桶（[`Wheel::insert`]）、
//! $O(1)$ 摘链（[`Wheel::remove`]）以及基于位图的常数时间无空转到期扫描（[`Wheel::next`] / [`Wheel::pop_due`]）。
//!
//! # 核心设计与非级联时间轮布局
//! 不同于传统需要从高层向低层搬迁（Re-cascade）节点的级联时间轮，本模块采用**固定 8 层、每层 64 桶的非级联设计**：
//! 节点在入轮时根据距离当前轮时钟 `clk` 的相对跨度一次性确定层级并向上对齐到该层粒度边界，直到触发或取消前
//! 始终停留在同一个桶中，无需后台级联搬迁。
//!
//! ```text
//! 基础粒度: WHEEL_HZ = 100 (1 tick = 10 ms = 10_000_000 ns)
//! 层级总数: LEVELS = 8, 每层桶数: BUCKETS_PER_LEVEL = 64 (pending[level]: u64 位图)
//! 层间步进: LEVEL_SHIFT = 3 (每上升一层，单桶时间粒度放大 8 倍 = 2^3)
//!
//! +-------+-------------------+-----------------------+--------------------------+
//! | Level | 桶粒度 (Ticks)    | 单桶时长              | 本层覆盖跨度 (64 桶)     |
//! +-------+-------------------+-----------------------+--------------------------+
//! |   0   | 8^0 = 1           | 10 ms                 | 640 ms                   |
//! |   1   | 8^1 = 8           | 80 ms                 | 5.12 s                   |
//! |   2   | 8^2 = 64          | 640 ms                | 40.96 s                  |
//! |   3   | 8^3 = 512         | 5.12 s                | ~5.46 min                |
//! |   4   | 8^4 = 4_096       | 40.96 s               | ~43.69 min               |
//! |   5   | 8^5 = 32_768      | ~5.46 min             | ~5.82 hours              |
//! |   6   | 8^6 = 262_144     | ~43.69 min            | ~46.60 hours             |
//! |   7   | 8^7 = 2_097_152   | ~5.82 hours           | ~15.53 days              |
//! +-------+-------------------+-----------------------+--------------------------+
//!
//! 桶索引布局 (heads: [*const Timer; 512]):
//!   bucket = level * 64 + ((effective_tick >> (level * 3)) & 63)
//!   每个非空桶在 pending[level] 的对应位 (0..64) 置 1，桶内通过 Links 维护侵入式双向链表。
//! ```
//! - **向上取整（Never Early）**：任意未过期的请求截止时间先按 `expires = deadline_ns / TICK_NS` 折算，
//!   再对齐到目标层级的**下一个粒度边界** `effective_tick = ((expires >> shift) + 1) << shift`，
//!   保证粗粒度定时器绝不在请求的 `deadline` 之前提前触发。
//! - **位图旋转扫描（Tickless Advancement）**：查询最早到期桶（[`Wheel::next_bucket`]）时，对每层 64 位
//!   `pending[level]` 位图按当前 `clk` 在该层的起始桶位置执行循环右移（`rotate_right`）并统计尾随零（`trailing_zeros`），
//!   直接跳过所有空 tick，无需逐 tick 递增推进。
//!
//! # 锁与并发契约
//! - 本模块自身不包含锁；生产环境下对 `Wheel` 及其关联 [`Links`] 的一切读写操作，调用方必须已持有所属 CPU
//!   的 `TimerBase::state` 自旋锁（内核自检 [`self_check`] 中则由当前线程独占局部未发布的 `Wheel` 与节点保证安全）。
//! - 挂入时间轮的所有 `Timer` 节点在通过 `remove` 或 `pop_due` 摘除前，其内存地址必须保持固定且有效。

use core::ptr;

use crate::time::MonotonicInstant;

use super::{ArmError, Timer};

/// 粗粒度时间轮单个基础 tick 的纳秒长度（10ms，对应 `WHEEL_HZ = 100`）。
pub(crate) const TICK_NS: u64 = crate::time::NSEC_PER_SEC / WHEEL_HZ;
/// 粗粒度时间轮的基础滴答频率（每秒 100 个 tick）。
const WHEEL_HZ: u64 = 100;

/// 时间轮的总层级数（8 层）。
const LEVELS: usize = 8;
/// 每层桶数量的以 2 为底对数（$2^6 = 64$ 个桶，恰好对应一个 `u64` 位图）。
const BUCKET_BITS: usize = 6;
/// 每一层包含的桶数量（64 个）。
const BUCKETS_PER_LEVEL: usize = 1 << BUCKET_BITS;
/// 层内桶索引的位掩码（`0x3f`）。
const BUCKET_MASK: usize = BUCKETS_PER_LEVEL - 1;
/// 相邻两层之间的时间粒度移位步长（每层放大 $2^3 = 8$ 倍）。
const LEVEL_SHIFT: usize = 3;
/// 8 层时间轮的桶数组总长度（$8 \times 64 = 512$ 个桶）。
const BUCKETS: usize = LEVELS * BUCKETS_PER_LEVEL;

/// 标识节点当前未挂入任何时间轮桶的哨兵值。
const UNLINKED: usize = usize::MAX;

/// 时间轮允许注册的最大相对超时跨度（单位：基础 tick）。
///
/// 最高层（第 7 层）预留出 2 个桶的余量（`64 - 2 = 62`），确保向上取整到下一个层粒度边界后不会在同一层发生回绕碰撞。
pub(super) const MAX_TIMEOUT_TICKS: u64 =
    (BUCKETS_PER_LEVEL as u64 - 2) << ((LEVELS - 1) * LEVEL_SHIFT);

/// 内嵌在每个 `Timer` 节点 `Node` 中的时间轮双向桶链表节点。
///
/// 由所属 CPU 的 `TimerBase::state` 自旋锁（或自检线程的局部独占权）保护。
pub(super) struct Links {
    /// 同一个桶双向链表中的前驱节点指针；若当前节点为桶头或未入桶，则为 `null`。
    previous: *const Timer,
    /// 同一个桶双向链表中的后继节点指针；若当前节点为桶尾或未入桶，则为 `null`。
    next: *const Timer,
    /// 当前节点所在的全局桶下标（`0..BUCKETS`）；未入桶时为 [`UNLINKED`]。
    bucket: usize,
    /// 节点按所在层级粒度向上取整后的实际到期 tick 编号。
    effective_tick: u64,
}

impl Links {
    /// 构造尚未挂入任何时间轮桶的初始空链节。
    pub(super) const fn new() -> Self {
        Self {
            previous: ptr::null(),
            next: ptr::null(),
            bucket: UNLINKED,
            effective_tick: 0,
        }
    }
}

/// 固定 8 层、每层 64 桶的非级联粗粒度时间轮。
pub(super) struct Wheel {
    /// 全部 512 个桶的侵入式双向链表头指针数组。
    heads: [*const Timer; BUCKETS],
    /// 每一层的 64 位非空桶位图：第 `i` 位为 `1` 表示该层第 `i` 个桶的链表非空。
    pending: [u64; LEVELS],
    /// 时间轮当前已推进到的基准 tick 时刻（若某个到期桶尚未被 `pop_due` 完全清空，则停留在那个桶的 tick）。
    clk: u64,
}

impl Wheel {
    /// 构造一个所有桶为空、基准时钟为 0 的时间轮。
    pub(super) const fn new() -> Self {
        Self {
            heads: [ptr::null(); BUCKETS],
            pending: [0; LEVELS],
            clk: 0,
        }
    }

    /// 扫描 8 层非空位图，返回自 `self.clk` 起最早到期的 `(全局桶下标, 到期 tick)`；若时间轮全空则返回 `None`。
    fn next_bucket(&self) -> Option<(usize, u64)> {
        let mut first = None;
        for level in 0..LEVELS {
            let bits = self.pending[level];
            if bits == 0 {
                continue;
            }
            let shift = level * LEVEL_SHIFT;
            let granularity = 1u64 << shift;
            let level_clk = self.clk.div_ceil(granularity);
            let start = (level_clk & BUCKET_MASK as u64) as u32;
            let distance = bits.rotate_right(start).trailing_zeros() as u64;
            let tick = (level_clk + distance) << shift;
            let bucket =
                level * BUCKETS_PER_LEVEL + ((start as usize + distance as usize) & BUCKET_MASK);
            if first.is_none_or(|(_, earliest)| tick < earliest) {
                first = Some((bucket, tick));
            }
        }
        first
    }

    /// 返回时间轮中最早到期桶对应的单调时钟绝对时刻（按该桶向上取整后的生效 tick 计算）；若轮为空则返回 `None`。
    pub(super) fn next(&self) -> Option<MonotonicInstant> {
        self.next_bucket()
            .map(|(_, tick)| MonotonicInstant::from_nanos(tick * TICK_NS))
    }

    /// 计算在不越过任何现存未处理桶的前提下，基准时钟 `clk` 可向当前时刻 `now` 推进到的目标 tick。
    fn forward_clock(&self, now: MonotonicInstant) -> u64 {
        let now_tick = now.as_nanos() / TICK_NS;
        let limit = self
            .next_bucket()
            .map_or(now_tick, |(_, first)| first.min(now_tick));
        self.clk.max(limit)
    }

    /// 纯函数计算给定截止时间 `deadline` 在当前时刻 `now` 下的入轮参数：`(推进后的 clk, 目标全局桶下标, 向上取整后的生效 tick)`。
    ///
    /// 若请求的超时跨度超出 [`MAX_TIMEOUT_TICKS`] 或纳秒换算溢出，则返回 `Err(ArmError::OutOfRange)`，且不修改时间轮状态。
    fn placement(
        &self,
        deadline: MonotonicInstant,
        now: MonotonicInstant,
    ) -> Result<(u64, usize, u64), ArmError> {
        // Step 1: 计算可推进的基准时钟 clk；若 deadline 已到期，直接放入第 0 层当前 clk 对应的桶以立即触发。
        let clk = self.forward_clock(now);
        if deadline <= now {
            return Ok((clk, (clk & BUCKET_MASK as u64) as usize, clk));
        }

        // Step 2: 校验相对超时跨度是否超出时间轮最大可表示范围 MAX_TIMEOUT_TICKS。
        let deadline_ns = deadline.as_nanos();
        let last_tick = deadline_ns.div_ceil(TICK_NS);
        if MAX_TIMEOUT_TICKS < last_tick.saturating_sub(clk) {
            return Err(ArmError::OutOfRange);
        }

        // Step 3: 根据相对 tick 跨度 delta 选择能够容纳该跨度的最低层级 level。
        let expires = deadline_ns / TICK_NS;
        let delta = expires.saturating_sub(clk);
        let mut level = 0;
        while level + 1 < LEVELS && ((BUCKET_MASK as u64) << (level * LEVEL_SHIFT)) <= delta {
            level += 1;
        }

        // Step 4: 将到期 tick 严格向上取整到目标层级的下一个粒度边界 effective_tick，确保永不早于请求期限触发。
        let shift = level * LEVEL_SHIFT;
        let effective_tick = ((expires >> shift) + 1) << shift;
        effective_tick
            .checked_mul(TICK_NS)
            .ok_or(ArmError::OutOfRange)?;
        let bucket =
            level * BUCKETS_PER_LEVEL + ((effective_tick >> shift) & BUCKET_MASK as u64) as usize;
        Ok((clk, bucket, effective_tick))
    }

    /// 预先校验 `deadline` 在当前时刻 `now` 下是否可被本时间轮接受（不修改任何状态，供重注册前预检使用）。
    pub(super) fn accepts(
        &self,
        deadline: MonotonicInstant,
        now: MonotonicInstant,
    ) -> Result<(), ArmError> {
        self.placement(deadline, now).map(|_| ())
    }

    /// 将未入轮的节点 `timer` 插入计算出的目标桶头部，并置位对应层级的 `pending` 位图。
    ///
    /// # Safety
    /// 1. 调用方必须独占访问本 `Wheel`、桶内所有已入轮节点的 [`Links`] 以及 `timer` 的 `Node`（生产环境由持有
    ///    `TimerBase::state` 自旋锁保证，自检环境由线程局部独占保证）；
    /// 2. `timer` 必须非空、未挂入任何时间轮桶（`bucket == UNLINKED`），且其 `node.expires` 已写入目标截止时间；
    /// 3. 所有入轮节点在通过 [`Wheel::remove`] 或 [`Wheel::pop_due`] 摘除前必须保持内存有效且地址固定。
    pub(super) unsafe fn insert(
        &mut self,
        timer: *const Timer,
        now: MonotonicInstant,
    ) -> Result<(), ArmError> {
        // SAFETY: 调用方保证 `timer` 非空有效且已独占访问其 `Node::expires`。
        let deadline = unsafe { (*(*timer).node.get()).expires };
        let (clk, bucket, effective_tick) = self.placement(deadline, now)?;

        let head = self.heads[bucket];
        // SAFETY: 调用方独占访问本轮及所有节点的 `wheel` 链节；`timer` 当前未入桶，原桶头 `head` 若非空则是轮中有效存活节点。
        unsafe {
            let links = &mut (*(*timer).node.get()).wheel;
            assert_eq!(links.bucket, UNLINKED);
            links.previous = ptr::null();
            links.next = head;
            links.bucket = bucket;
            links.effective_tick = effective_tick;
            if !head.is_null() {
                debug_assert_eq!((*(*head).node.get()).wheel.effective_tick, effective_tick);
                (*(*head).node.get()).wheel.previous = timer;
            }
        }
        self.clk = clk;
        self.heads[bucket] = timer;
        self.pending[bucket / BUCKETS_PER_LEVEL] |= 1u64 << (bucket % BUCKETS_PER_LEVEL);
        Ok(())
    }

    /// 将已知在时间轮中的节点 `timer` 从其所在桶的双向链表中 $O(1)$ 摘除；若摘除后桶变空则清除对应 `pending` 位。
    ///
    /// # Safety
    /// 1. 调用方必须独占访问本 `Wheel` 及所有已入轮节点的 [`Links`]（持有 `TimerBase::state` 自旋锁或自检线程独占）；
    /// 2. `timer` 必须非空且确实是当前时间轮中的已入桶成员（`links.bucket < BUCKETS`）。
    pub(super) unsafe fn remove(&mut self, timer: *const Timer) {
        // SAFETY: 调用方已独占本轮并保证 `timer` 及其在桶链表中的前后驱指针均为有效存活成员。
        unsafe {
            let links = &mut (*(*timer).node.get()).wheel;
            let bucket = links.bucket;
            assert!(bucket < BUCKETS);
            if links.previous.is_null() {
                self.heads[bucket] = links.next;
            } else {
                (*(*links.previous).node.get()).wheel.next = links.next;
            }
            if !links.next.is_null() {
                (*(*links.next).node.get()).wheel.previous = links.previous;
            }
            if self.heads[bucket].is_null() {
                self.pending[bucket / BUCKETS_PER_LEVEL] &= !(1u64 << (bucket % BUCKETS_PER_LEVEL));
            }
            *links = Links::new();
        }
    }

    /// 推进基准时钟 `clk` 并从最早到期的桶中弹出一个截止时间不晚于 `now` 的节点；若无到期桶则返回 `None`。
    ///
    /// 若同一个桶内还有其他到期节点，该桶的 `pending` 位保持置位且 `self.clk` 停留在该桶的 `tick`，供后续调用继续弹出。
    ///
    /// # Safety
    /// 1. 调用方必须独占访问本 `Wheel` 及所有已入轮节点的 [`Links`]（持有 `TimerBase::state` 自旋锁或自检线程独占）；
    /// 2. 轮中所有已入桶节点均保持有效且地址稳定。
    pub(super) unsafe fn pop_due(&mut self, now: MonotonicInstant) -> Option<*const Timer> {
        let now_tick = now.as_nanos() / TICK_NS;
        let Some((bucket, tick)) = self.next_bucket() else {
            self.clk = self.clk.max(now_tick);
            return None;
        };
        if now_tick < tick {
            self.clk = self.clk.max(now_tick.min(tick));
            return None;
        }
        self.clk = tick;
        let timer = self.heads[bucket];
        // SAFETY: `next_bucket()` 仅返回 `pending` 位图中置位的非空桶，故 `self.heads[bucket]` 必然是非空有效成员；
        // 调用方已独占本轮及全部链节，满足 `self.remove` 的安全前置条件。
        unsafe { self.remove(timer) };
        Some(timer)
    }
}

#[cfg(feature = "ktest")]
/// 执行粗粒度时间轮的单元测试套件：覆盖 8 层入桶定位、向上取整边界、过去时刻处理、最大跨度上限、同桶多节点删除与超大时钟推进。
///
/// 返回执行通过的测试子用例总数。
pub(super) fn self_check() -> usize {
    use core::pin::{pin, Pin};

    use super::TimerKind;

    /// 自检占位回调函数（本测试仅验证时间轮数据结构，不触发回调执行）。
    fn callback(_: Pin<&Timer>) {}

    /// 将 tick 编号换算为对应的单调时钟绝对时刻。
    fn instant(tick: u64) -> MonotonicInstant {
        MonotonicInstant::from_nanos(tick * TICK_NS)
    }

    /// 在自检节点未挂入时间轮时修改其内部 `expires` 截止时间。
    ///
    /// # Safety
    /// `timer` 必须是当前线程局部独占、未挂入任何队列的测试节点。
    unsafe fn set_deadline(timer: Pin<&Timer>, deadline: MonotonicInstant) {
        // SAFETY: 调用方保证该测试节点由当前线程独占且未入轮，直接修改 `Node::expires` 无并发竞争。
        unsafe { (*timer.node.get()).expires = deadline };
    }
    let first = pin!(Timer::new(0, TimerKind::Coarse, callback, 0));
    let second = pin!(Timer::new(0, TimerKind::Coarse, callback, 0));
    let third = pin!(Timer::new(0, TimerKind::Coarse, callback, 0));
    let a = first.as_ref().get_ref() as *const Timer;
    let b = second.as_ref().get_ref() as *const Timer;
    let c = third.as_ref().get_ref() as *const Timer;
    let mut cases = 0;

    // SAFETY:
    // 1. 下方测试全程在当前单线程中操作局部 `Wheel` 实例与栈上通过 `pin!` 固定的三个私有节点 `first`/`second`/`third`；
    // 2. 每次调用 `set_deadline` 与 `insert` 前，节点均处于未入轮状态；每个子用例结束前均通过 `pop_due` 或 `remove` 将节点摘除。
    unsafe {
        // Case 1: 遍历验证全部 8 个层级的边界入桶、向上取整到期时刻计算与精确到期弹出。
        for level in 0..LEVELS {
            let mut wheel = Wheel::new();
            let expires = if level == 0 {
                62
            } else {
                63u64 << ((level - 1) * LEVEL_SHIFT)
            };
            let effective = ((expires >> (level * LEVEL_SHIFT)) + 1) << (level * LEVEL_SHIFT);
            set_deadline(first.as_ref(), instant(expires));
            wheel.insert(a, instant(0)).unwrap();
            assert_eq!((*first.node.get()).wheel.bucket / BUCKETS_PER_LEVEL, level);
            assert_eq!(wheel.next(), Some(instant(effective)));
            assert!(instant(expires) <= wheel.next().unwrap());
            assert_eq!(
                wheel.pop_due(MonotonicInstant::from_nanos(effective * TICK_NS - 1)),
                None
            );
            assert_eq!(wheel.next(), Some(instant(effective)));
            assert_eq!(wheel.pop_due(instant(effective)), Some(a));
            assert_eq!(wheel.next(), None);
            cases += 1;
        }

        // Case 2: 验证已过期或恰好等于当前时刻的截止时间均被立即放入当前 tick 桶并在当刻弹出。
        for (deadline_ns, now_ns) in [
            (0, 0),
            (0, 13 * TICK_NS + 7),
            (13 * TICK_NS + 7, 13 * TICK_NS + 7),
        ] {
            let mut wheel = Wheel::new();
            let now = MonotonicInstant::from_nanos(now_ns);
            set_deadline(first.as_ref(), MonotonicInstant::from_nanos(deadline_ns));
            wheel.insert(a, now).unwrap();
            assert_eq!(wheel.next(), Some(instant(now_ns / TICK_NS)));
            assert_eq!(wheel.pop_due(now), Some(a));
            assert_eq!(wheel.next(), None);
            cases += 1;
        }

        // Case 3: 验证非整 tick 纳秒偏移与跨层边界时的严格向上取整（永不早于请求时刻到期）。
        for (nanos, effective) in [(1, 1), (64 * TICK_NS, 72), (65 * TICK_NS + 1, 72)] {
            let mut wheel = Wheel::new();
            set_deadline(first.as_ref(), MonotonicInstant::from_nanos(nanos));
            wheel.insert(a, instant(0)).unwrap();
            assert_eq!(wheel.next(), Some(instant(effective)));
            assert_eq!(
                wheel.pop_due(MonotonicInstant::from_nanos(effective * TICK_NS - 1)),
                None
            );
            assert_eq!(wheel.pop_due(instant(effective)), Some(a));
            cases += 1;
        }

        // Case 4: 验证最大支持相对跨度 MAX_TIMEOUT_TICKS 的正常入桶与到期弹出。
        let mut wheel = Wheel::new();
        set_deadline(first.as_ref(), instant(MAX_TIMEOUT_TICKS));
        wheel.insert(a, instant(0)).unwrap();
        let last = wheel.next().unwrap();
        assert!(instant(MAX_TIMEOUT_TICKS) <= last);
        assert_eq!(
            wheel.pop_due(MonotonicInstant::from_nanos(last.as_nanos() - 1)),
            None
        );
        assert_eq!(wheel.pop_due(last), Some(a));
        assert_eq!(wheel.next(), None);
        cases += 1;

        // Case 5: 验证超出 MAX_TIMEOUT_TICKS 的预检返回 OutOfRange，且不破坏节点原有入轮状态。
        let mut wheel = Wheel::new();
        set_deadline(first.as_ref(), instant(1));
        wheel.insert(a, instant(0)).unwrap();
        let original = wheel.next();
        assert!(matches!(
            wheel.accepts(instant(MAX_TIMEOUT_TICKS + 1), instant(0)),
            Err(ArmError::OutOfRange)
        ));
        assert_eq!(wheel.next(), original);
        assert_eq!((*first.node.get()).expires, instant(1));
        wheel.remove(a);
        assert_eq!(wheel.next(), None);
        cases += 1;

        // Case 6: 验证不同层级双节点的独立摘除（remove）与最早到期时间重算。
        let mut wheel = Wheel::new();
        set_deadline(first.as_ref(), instant(65));
        set_deadline(second.as_ref(), instant(500));
        wheel.insert(a, instant(0)).unwrap();
        wheel.insert(b, instant(0)).unwrap();
        wheel.remove(a);
        assert_eq!(wheel.next(), Some(instant(504)));
        wheel.remove(b);
        assert_eq!(wheel.next(), None);
        assert_eq!(wheel.pop_due(instant(MAX_TIMEOUT_TICKS)), None);
        cases += 1;

        // Case 7: 验证同一桶内挂载多个节点时的头插顺序、中间节点摘除（remove）与逐个弹出。
        let mut wheel = Wheel::new();
        for timer in [first.as_ref(), second.as_ref(), third.as_ref()] {
            set_deadline(timer, instant(65));
            wheel.insert(timer.get_ref(), instant(0)).unwrap();
        }
        assert_eq!(wheel.pop_due(instant(1000)), Some(c));
        assert_eq!(wheel.next(), Some(instant(72)));
        wheel.remove(a);
        assert_eq!(wheel.next(), Some(instant(72)));
        assert_eq!(wheel.pop_due(instant(1000)), Some(b));
        assert_eq!(wheel.next(), None);
        cases += 1;

        // Case 8: 验证跨高层级的多节点按先后到期顺序依次弹出。
        let mut wheel = Wheel::new();
        set_deadline(first.as_ref(), instant(4096));
        set_deadline(second.as_ref(), instant(1_000_000));
        wheel.insert(a, instant(0)).unwrap();
        wheel.insert(b, instant(0)).unwrap();
        let first_due = wheel.next().unwrap();
        assert_eq!(wheel.pop_due(instant(MAX_TIMEOUT_TICKS)), Some(a));
        let second_due = wheel.next().unwrap();
        assert!(first_due < second_due);
        assert!(instant(1_000_000) <= second_due);
        assert_eq!(wheel.pop_due(instant(MAX_TIMEOUT_TICKS)), Some(b));
        assert_eq!(wheel.next(), None);
        cases += 1;

        // Case 9: 验证非零初始基准时钟（now = 60 ticks）下的相对层级选择与弹出。
        let mut wheel = Wheel::new();
        set_deadline(first.as_ref(), instant(61));
        set_deadline(second.as_ref(), instant(64));
        wheel.insert(a, instant(60)).unwrap();
        wheel.insert(b, instant(60)).unwrap();
        assert_eq!(wheel.next(), Some(instant(62)));
        assert_eq!(wheel.pop_due(instant(1000)), Some(a));
        assert_eq!(wheel.next(), Some(instant(65)));
        assert_eq!(wheel.pop_due(instant(1000)), Some(b));
        assert_eq!(wheel.next(), None);
        cases += 1;

        // Case 10: 验证时间轮中已有早期未消费桶时，forward_clock 不会越过该早期桶丢弃已到期节点。
        let mut wheel = Wheel::new();
        set_deadline(first.as_ref(), instant(65));
        wheel.insert(a, instant(0)).unwrap();
        set_deadline(second.as_ref(), instant(999));
        wheel.insert(b, instant(1000)).unwrap();
        assert_eq!(wheel.next(), Some(instant(72)));
        assert_eq!(wheel.pop_due(instant(1000)), Some(b));
        assert_eq!(wheel.next(), Some(instant(72)));
        assert_eq!(wheel.pop_due(instant(1000)), Some(a));
        assert_eq!(wheel.next(), None);
        cases += 1;

        // Case 11: 验证空轮在经历大幅度时钟跳跃（100 倍 MAX_TIMEOUT_TICKS）后仍能正确推进 clk 并接纳新节点。
        let mut wheel = Wheel::new();
        let now = instant(MAX_TIMEOUT_TICKS * 100);
        assert_eq!(wheel.pop_due(now), None);
        set_deadline(first.as_ref(), instant(MAX_TIMEOUT_TICKS * 100 + 63));
        wheel.insert(a, now).unwrap();
        let due = wheel.next().unwrap();
        assert!(instant(MAX_TIMEOUT_TICKS * 100 + 63) <= due);
        assert_eq!(
            wheel.pop_due(MonotonicInstant::from_nanos(due.as_nanos() - 1)),
            None
        );
        assert_eq!(wheel.pop_due(due), Some(a));
        assert_eq!(wheel.next(), None);
        cases += 1;

        // Case 12: 验证接近 u64::MAX 纳秒上限时，向上取整导致的纳秒乘法溢出被安全拦截并返回 OutOfRange。
        let mut wheel = Wheel::new();
        set_deadline(first.as_ref(), MonotonicInstant::from_nanos(u64::MAX));
        assert!(matches!(
            wheel.insert(a, MonotonicInstant::from_nanos(u64::MAX - 1)),
            Err(ArmError::OutOfRange)
        ));
        assert_eq!(wheel.next(), None);
        assert_eq!((*first.node.get()).wheel.bucket, UNLINKED);
        cases += 1;
    }
    cases
}
