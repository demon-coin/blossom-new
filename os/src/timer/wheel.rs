use core::ptr;

use crate::time::MonotonicInstant;

use super::{ArmError, Timer};

/// 粗定时器的一个基础 tick，对应 HZ=100。
pub(crate) const TICK_NS: u64 = crate::time::NSEC_PER_SEC / WHEEL_HZ;
const WHEEL_HZ: u64 = 100;

const LEVELS: usize = 8;
const BUCKET_BITS: usize = 6;
const BUCKETS_PER_LEVEL: usize = 1 << BUCKET_BITS;
const BUCKET_MASK: usize = BUCKETS_PER_LEVEL - 1;
const LEVEL_SHIFT: usize = 3;
const BUCKETS: usize = LEVELS * BUCKETS_PER_LEVEL;

const UNLINKED: usize = usize::MAX;

/// 允许的最大相对期限；末层仍为向上取整留出一个粒度。
pub(super) const MAX_TIMEOUT_TICKS: u64 =
    (BUCKETS_PER_LEVEL as u64 - 2) << ((LEVELS - 1) * LEVEL_SHIFT);

/// 地址稳定节点的双向桶链；生产由所属 base 锁保护，自检仅访问未发表的独占节点。
pub(super) struct Links {
    previous: *const Timer,
    next: *const Timer,
    bucket: usize,
    effective_tick: u64,
}

impl Links {
    /// 构造尚未登记到任何桶的链节。
    pub(super) const fn new() -> Self {
        Self {
            previous: ptr::null(),
            next: ptr::null(),
            bucket: UNLINKED,
            effective_tick: 0,
        }
    }
}

/// 固定八层非级联轮；每层位图跳过空桶，不逐 tick 推进。
pub(super) struct Wheel {
    heads: [*const Timer; BUCKETS],
    pending: [u64; LEVELS],
    // 下一个未处理时刻；部分消费的桶仍留在这个时刻。
    clk: u64,
}

impl Wheel {
    /// 构造空轮，不申请节点或控制块。
    pub(super) const fn new() -> Self {
        Self {
            heads: [ptr::null(); BUCKETS],
            pending: [0; LEVELS],
            clk: 0,
        }
    }

    /// 每层一次旋转位扫描，利用 clk 起一圈内的桶期限选择真实最早处理点。
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

    /// 返回实际最早桶的处理期限，而非未取整的请求期限。
    pub(super) fn next(&self) -> Option<MonotonicInstant> {
        self.next_bucket()
            .map(|(_, tick)| MonotonicInstant::from_nanos(tick * TICK_NS))
    }

    /// 仅推进到当前时刻或首个未消费桶，保留仍待执行的到期节点。
    fn forward_clock(&self, now: MonotonicInstant) -> u64 {
        let now_tick = now.as_nanos() / TICK_NS;
        let limit = self
            .next_bucket()
            .map_or(now_tick, |(_, first)| first.min(now_tick));
        self.clk.max(limit)
    }

    /// 纯计算受支持的层、桶和上取整期限，范围失败不修改原登记。
    fn placement(
        &self,
        deadline: MonotonicInstant,
        now: MonotonicInstant,
    ) -> Result<(u64, usize, u64), ArmError> {
        let clk = self.forward_clock(now);
        if deadline <= now {
            return Ok((clk, (clk & BUCKET_MASK as u64) as usize, clk));
        }
        let deadline_ns = deadline.as_nanos();
        let last_tick = deadline_ns.div_ceil(TICK_NS);
        if MAX_TIMEOUT_TICKS < last_tick.saturating_sub(clk) {
            return Err(ArmError::OutOfRange);
        }
        let expires = deadline_ns / TICK_NS;
        let delta = expires.saturating_sub(clk);
        let mut level = 0;
        while level + 1 < LEVELS && ((BUCKET_MASK as u64) << (level * LEVEL_SHIFT)) <= delta {
            level += 1;
        }

        // 请求先落在 tick 内，再严格取下一个层粒度边界，避免 tick
        // 边缘和上层截断导致提前；不把长期限截短到支持范围。
        let shift = level * LEVEL_SHIFT;
        let effective_tick = ((expires >> shift) + 1) << shift;
        effective_tick
            .checked_mul(TICK_NS)
            .ok_or(ArmError::OutOfRange)?;
        let bucket =
            level * BUCKETS_PER_LEVEL + ((effective_tick >> shift) & BUCKET_MASK as u64) as usize;
        Ok((clk, bucket, effective_tick))
    }

    /// 纯预检新期限；更新旧登记时必须在同一 base 锁内先检查再撤链。
    pub(super) fn accepts(
        &self,
        deadline: MonotonicInstant,
        now: MonotonicInstant,
    ) -> Result<(), ArmError> {
        self.placement(deadline, now).map(|_| ())
    }

    /// 将未链接节点入桶；范围错误不改变轮或节点。
    ///
    /// # Safety
    /// 调用者必须独占访问本轮、全部入轮节点的链节及 timer 的链节和 expires；
    /// 生产由所属 base 锁满足，单线程自检由不发表轮和节点满足，不允许共享并发无锁访问。
    /// timer 必须有效、未链接，expires 已在该独占访问下设置；全部入轮节点
    /// 的存储必须保持有效且地址稳定直到各自撤链。
    pub(super) unsafe fn insert(
        &mut self,
        timer: *const Timer,
        now: MonotonicInstant,
    ) -> Result<(), ArmError> {
        // SAFETY: 调用者保证 timer 有效，并独占其 expires，排除并发读写。
        let deadline = unsafe { (*(*timer).node.get()).expires };
        let (clk, bucket, effective_tick) = self.placement(deadline, now)?;

        let head = self.heads[bucket];
        // SAFETY: timer 未链接，已有 head 是本轮内仍存活的稳定节点；全部链节由调用者独占。
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

    /// 由已知链节常数时间撤链；仅末节点清除桶位，不扫描其他桶。
    ///
    /// # Safety
    /// 调用者必须独占访问本轮及全部入轮节点的链节；生产由所属 base 锁满足，
    /// 单线程自检由不发表轮和节点满足，不允许共享并发无锁访问。timer 必须
    /// 是本轮已链接成员，全部成员的存储必须有效且地址稳定直到各自撤链。
    pub(super) unsafe fn remove(&mut self, timer: *const Timer) {
        // SAFETY: 调用者保证 timer 属于本轮，全部成员稳定存活且链节处于独占访问下。
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

    /// 最多检查八层位图并摘一个到期节点；原桶未空时继续保留其处理点。
    ///
    /// # Safety
    /// 调用者必须独占访问本轮及全部入轮节点的链节；生产由所属 base 锁满足，
    /// 单线程自检由不发表轮和节点满足，不允许共享并发无锁访问。全部成员
    /// 的存储必须有效且地址稳定直到各自撤链；返回节点的运行状态和引用转移
    /// 由调用者处理，轮不访问生命周期字段。
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
        // SAFETY: 置位桶必有本轮有效稳定成员，调用者独占本轮及全部链节，满足 remove 契约。
        unsafe { self.remove(timer) };
        Some(timer)
    }
}

#[cfg(feature = "ktest")]
/// 对生产入桶、位图、撤链和推进执行边界断言；不替代真实客体回调计数。
pub(super) fn self_check() -> usize {
    use core::pin::{pin, Pin};

    use super::TimerKind;

    /// 仅满足自检节点的构造接口；本自检只操作轮，不执行回调。
    fn callback(_: Pin<&Timer>) {}

    /// 将自检中受常量边界约束的 tick 换算为单调时刻。
    fn instant(tick: u64) -> MonotonicInstant {
        MonotonicInstant::from_nanos(tick * TICK_NS)
    }

    /// 在自检节点未链接且独占时设置下一次入轮期限。
    ///
    /// # Safety
    /// timer 必须是未链接、未向其他 CPU 或服务发表的自检专属节点；
    /// 调用者独占其 Node，存储保持有效且地址稳定。
    unsafe fn set_deadline(timer: Pin<&Timer>, deadline: MonotonicInstant) {
        // SAFETY: 调用者保证该自检节点未链接、未发表，并独占其 Node。
        unsafe { (*timer.node.get()).expires = deadline };
    }

    let first = pin!(Timer::new(0, TimerKind::Coarse, callback, 0));
    let second = pin!(Timer::new(0, TimerKind::Coarse, callback, 0));
    let third = pin!(Timer::new(0, TimerKind::Coarse, callback, 0));
    let a = first.as_ref().get_ref() as *const Timer;
    let b = second.as_ref().get_ref() as *const Timer;
    let c = third.as_ref().get_ref() as *const Timer;
    let mut cases = 0;

    // SAFETY: 本线程独占所有局部 Wheel 和三个未发表节点的 Node，pin! 保持存储稳定。
    // 每次 set_deadline 都发生在节点首次入轮前，或前例已由 pop_due/remove 撤链后；
    // 每个成功登记均在进入下一例或退栈前撤链，范围失败不登记，故所有成员始终存活。
    unsafe {
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
