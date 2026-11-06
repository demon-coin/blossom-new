//! 系统单调时钟（Monotonic Clock）、墙钟时间（Realtime）与每核时钟事件（Clockevent）管理。
//!
//! # 模块职责与架构定位
//! - **职责**：将底层硬件计数器周期（Cycles）精确换算为单调递增的纳秒时间（[`MonotonicInstant`]），结合平台实时时钟（RTC）提供 Unix 墙钟时间，并统一管理每个 CPU 核心的硬件定时器下一次触发期限。
//! - **上游调用者**：启动初始化（`initialize`）、调度器（`activate_current_cpu`、`task_switch_in`、`task_switch_out`）、时钟中断处理入口（`handle_timer`）、软件定时器子系统（[`crate::timer`]）以及时间相关系统调用与内核日志时间戳。
//! - **下游依赖**：架构层硬件计数器读取与定时器编程（[`crate::arch`]）、板级 RTC 采样（`arch::board_rtc_calendar`）、每核本地状态（[`crate::cpu`]）及软件定时器最近到期查询（`crate::timer::with_next_deadline`）。
//!
//! # 核心机制、算术模型与寄存器布局
//! 1. **定点换算与小数余数保留（[`Scale`]）**：
//!    为避免 64 位整数乘法溢出并消除多次维护采样之间的舍入累积误差，硬件周期差值 $\Delta \text{cycles}$ 通过定点参数 $(\text{mult}, \text{shift})$ 换算为纳秒：
//!    $$\text{scaled} = \Delta \text{cycles} \times \text{mult} + \text{fraction}$$
//!    $$\Delta \text{ns} = \text{scaled} \gg \text{shift}, \quad \text{fraction}' = \text{scaled} \ \& \ ((1 \ll \text{shift}) - 1)$$
//!    每次维护间隔 `maintenance_delta` 取最大安全差值 `max_delta` 的一半，为中断延迟预留充足余量。
//!
//! 2. **顺序锁（SeqLock）无锁读取与单写者更新（[`Timekeeper`]）**：
//!
//! ```text
//! 维护写者 (Timekeeper::update)                     并发读者 (Timekeeper::sample)
//! ─────────────────────────────                     ─────────────────────────────
//! 1. 获取 writer 自旋锁 (关中断)                    1. 读 sequence (Acquire)
//! 2. 读取硬件 counter 并计算新基准                     └─ 若为奇数 (写者正在更新) -> spin_loop 重试
//! 3. sequence 置为奇数 (sequence + 1)               2. 依次读取 base_ns, cycle_last, fraction
//! 4. fence(Release) ───────────────────────────────► 3. 读取硬件 counter
//! 5. 写入新的 base_ns, cycle_last, fraction         4. fence(Acquire)
//! 6. sequence 置为新偶数 (Release)                  5. 再次读 sequence：若与步骤 1 不相等则重试
//! 7. 释放 writer 自旋锁                             6. 用一致的基准与 counter 计算当前纳秒
//! ```
//!
//! 3. **LoongArch LS7A TOY（Time of Year）寄存器位域布局（[`toy`]）**：
//!
//! ```text
//! TOY_READ0_REG (offset 0x2c) - 32 位日历低字寄存器：
//! ┌──────────┬──────────┬──────────┬───────────┬───────────┬──────────┐
//! │ 31 .. 26 │ 25 .. 21 │ 20 .. 16 │ 15 .. 10  │  9 .. 4   │  3 .. 0  │
//! │  Month   │   Day    │   Hour   │  Minute   │  Second   │ Sub-sec  │
//! │ (6 bits) │ (5 bits) │ (5 bits) │ (6 bits)  │ (6 bits)  │ (4 bits) │
//! └──────────┴──────────┴──────────┴───────────┴───────────┴──────────┘
//! TOY_READ1_REG (offset 0x30)：32 位年份偏移值（实际公历年份 = 1900 + TOY_READ1_REG）
//! RTC_CTRL_REG  (offset 0x40)：Bit 11 = TOY_ENABLE，Bit 8 = TOY_32KHZ_ENABLE
//! ```
//!
//! 4. **每核时钟事件（[`LocalEvent`]）三合一期限仲裁**：
//!    每个 CPU 在编程底层硬件比较器（`program`）时，从以下三类期限中取最早值（最小值）：
//!    - `maintenance`：防止硬件计数器差值超出 `Scale::max_delta` 的周期性时间基准维护期限；
//!    - `quantum`：当前运行任务的时间片轮转到期期限（`QUANTUM = 10ms`）；
//!    - `timer_deadline`：当前 CPU 上挂载的硬定时器与睡眠定时器的最早到期时间。
//!
//! # 锁获取顺序与并发契约
//! - `LocalEvent::with_current` 要求在关中断（IRQ-off）状态下仅访问当前 CPU 自身的 `LocalEvent`，且闭包内严禁触发上下文切换或递归调用 `with_current`。
//! - `handle_timer` 先结束 `LocalEvent::with_current` 的可变借用，再调用 `crate::timer::handle_interrupt()`（允许定时器回调重新注册定时器并触发重编程），最后调用 `reprogram_current()`。
//! - 硬件编程路径锁顺序：先按值拷贝出 `EventState`，再进入 `crate::timer::with_next_deadline` 获取本核定时器底座自旋锁，在锁内调用 `Timekeeper::sample`（无锁顺序读）与 `arch::program_timer` 完成硬件下发，确保不会把已被并发移除或更新的旧期限写入硬件。

pub use core::time::Duration;

use core::cell::UnsafeCell;
use core::convert::TryFrom;
use core::hint::spin_loop;
#[cfg(feature = "ktest")]
use core::sync::atomic::AtomicUsize;
use core::sync::atomic::{compiler_fence, fence, AtomicU64, Ordering};
use spin::Once;

use crate::arch;
use crate::boot::{BootInfo, Platform, Region};
use crate::cpu;
use crate::sync::SpinLock;

/// 每秒包含的纳秒数（$10^9\text{ ns}$）。
pub(crate) const NSEC_PER_SEC: u64 = 1_000_000_000;
/// 每毫秒包含的纳秒数（$10^6\text{ ns}$）。
pub(crate) const NSEC_PER_MSEC: u64 = NSEC_PER_SEC / 1_000;
/// 每微秒包含的纳秒数（$10^3\text{ ns}$）。
pub(crate) const NSEC_PER_USEC: u64 = NSEC_PER_MSEC / 1_000;
/// 每秒包含的微秒数（$10^6\ \mu\text{s}$）。
pub(crate) const USEC_PER_SEC: u64 = NSEC_PER_SEC / NSEC_PER_USEC;

/// 选择定点换算系数时要求无溢出覆盖的最小时间跨度（3600 秒，即 1 小时）。
const CONVERSION_SECONDS: u64 = 3600;

/// 调度器任务时间片长度（10 毫秒）。
const QUANTUM: Duration = Duration::from_millis(10);

/// LoongArch LS7A TOY（Time of Year）实时时钟寄存器偏移、控制位与日历位域定义。
#[cfg(any(target_arch = "loongarch64", feature = "ktest"))]
pub(crate) mod toy {
    /// TOY 日历低字只读寄存器偏移（存放月、日、时、分、秒字段）。
    #[cfg(target_arch = "loongarch64")]
    pub(crate) const TOY_READ0_REG: usize = 0x2c;
    /// TOY 日历高字只读寄存器偏移（存放相对于 1900 年的年份偏移）。
    #[cfg(target_arch = "loongarch64")]
    pub(crate) const TOY_READ1_REG: usize = 0x30;
    /// RTC 控制寄存器偏移。
    #[cfg(target_arch = "loongarch64")]
    pub(crate) const RTC_CTRL_REG: usize = 0x40;
    /// `RTC_CTRL_REG` 中的 TOY 计数器使能位（Bit 11）。
    #[cfg(target_arch = "loongarch64")]
    pub(crate) const TOY_ENABLE: u32 = 1 << 11;
    /// `RTC_CTRL_REG` 中的外部 32.768kHz 晶振使能位（Bit 8）。
    #[cfg(target_arch = "loongarch64")]
    pub(crate) const TOY_32KHZ_ENABLE: u32 = 1 << 8;

    /// TOY 年份寄存器的基准年份（1900 年）。
    pub(crate) const TOY_YEAR_BASE: u32 = 1900;
    /// `TOY_READ0_REG` 中月份字段的起始位偏移（Bit 26）。
    pub(crate) const TOY_MONTH_SHIFT: u32 = 26;
    /// `TOY_READ0_REG` 中月份字段的位掩码（6 位）。
    pub(crate) const TOY_MONTH_MASK: u32 = (1 << 6) - 1;
    /// `TOY_READ0_REG` 中日期（Day of Month）字段的起始位偏移（Bit 21）。
    pub(crate) const TOY_DAY_SHIFT: u32 = 21;
    /// `TOY_READ0_REG` 中日期字段的位掩码（5 位）。
    pub(crate) const TOY_DAY_MASK: u32 = (1 << 5) - 1;
    /// `TOY_READ0_REG` 中小时字段的起始位偏移（Bit 16）。
    pub(crate) const TOY_HOUR_SHIFT: u32 = 16;
    /// `TOY_READ0_REG` 中小时字段的位掩码（5 位）。
    pub(crate) const TOY_HOUR_MASK: u32 = (1 << 5) - 1;
    /// `TOY_READ0_REG` 中分钟字段的起始位偏移（Bit 10）。
    pub(crate) const TOY_MINUTE_SHIFT: u32 = 10;
    /// `TOY_READ0_REG` 中分钟字段的位掩码（6 位）。
    pub(crate) const TOY_MINUTE_MASK: u32 = (1 << 6) - 1;
    /// `TOY_READ0_REG` 中秒字段的起始位偏移（Bit 4）。
    pub(crate) const TOY_SECOND_SHIFT: u32 = 4;
    /// `TOY_READ0_REG` 中秒字段的位掩码（6 位）。
    pub(crate) const TOY_SECOND_MASK: u32 = (1 << 6) - 1;
}

/// 自系统时间基准初始化起累计的单调递增时刻（单位：纳秒）。
///
/// 该时间戳仅反映系统运行经过的相对单调时间，不受墙钟（RTC）起点影响。
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct MonotonicInstant(u64);

impl MonotonicInstant {
    /// 由自时间基准起点起算的纳秒数直接构造 [`MonotonicInstant`]。
    pub const fn from_nanos(nanos: u64) -> Self {
        Self(nanos)
    }

    /// 返回自时间基准起点起算的纳秒数。
    pub const fn as_nanos(self) -> u64 {
        self.0
    }

    /// 计算当前时刻加上 `duration` 后的目标时刻；若 `duration` 或加法结果超出 `u64` 纳秒表示范围则返回 `None`。
    pub fn checked_add(self, duration: Duration) -> Option<Self> {
        let nanos = u64::try_from(duration.as_nanos()).ok()?;
        self.0.checked_add(nanos).map(Self)
    }

    /// 计算从 `earlier` 到当前时刻 `self` 经过的时间间隔；若 `self < earlier` 则返回 `None`。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "执行计账测试以单调墙钟核对阻塞和迁核间隔，逆序仍须返回 None"
        )
    )]
    pub fn checked_duration_since(self, earlier: Self) -> Option<Duration> {
        self.0.checked_sub(earlier.0).map(Duration::from_nanos)
    }
}

/// 硬件计数器周期与纳秒之间的定点换算参数及安全采样界限。
///
/// 在 [`Timekeeper`] 初始化时根据硬件频率一次性计算生成，运行期只读不变。
#[derive(Clone, Copy)]
pub(crate) struct Scale {
    /// 硬件计数器的有效位掩码（连续低位全 1，如 64 位计数器为 `u64::MAX`）。
    pub(crate) mask: u64,
    /// 周期转纳秒的定点乘数（满足 $\text{ns} \approx (\text{cycles} \times \text{mult}) \gg \text{shift}$）。
    pub(crate) mult: u64,
    /// 周期转纳秒的定点右移位数。
    pub(crate) shift: u32,
    /// 单次采样允许的最大硬件计数器周期差值（同时受 64 位乘法不溢出与计数器半周期回绕边界 `mask >> 1` 限制）。
    pub(crate) max_delta: u64,
    /// 周期性维护基准的硬件周期差值阈值（取 `max_delta / 2`，为关中断延迟预留一半安全窗口）。
    pub(crate) maintenance_delta: u64,
}

impl Scale {
    /// 校验硬件计数器频率与位宽掩码，并搜索能无溢出覆盖 1 小时跨度（`CONVERSION_SECONDS`）的最高精度定点换算参数。
    fn new(frequency: u64, mask: u64) -> Self {
        assert!(
            0 < frequency && frequency <= u32::MAX as u64,
            "invalid counter frequency"
        );
        assert!(
            3 <= mask && mask & mask.wrapping_add(1) == 0,
            "invalid counter mask"
        );

        // Step 1: 从最大移位值向小搜索，寻找满足 1 小时周期数乘 `mult` 加余数不溢出的最高精度 `(mult, shift)`。
        let required_cycles = frequency * CONVERSION_SECONDS;
        let (mult, shift) = (0..=u32::BITS)
            .rev()
            .find_map(|shift| {
                let mult = ((NSEC_PER_SEC << shift) + frequency / 2) / frequency;
                let residue_mask = (1u64 << shift) - 1;
                if 0 < mult
                    && mult <= u32::MAX as u64
                    && required_cycles <= (u64::MAX - residue_mask) / mult
                {
                    Some((mult, shift))
                } else {
                    None
                }
            })
            .expect("counter has no valid scale");

        // Step 2: 计算最大安全周期差值 `max_delta`（不超过计数器半量程）与维护间隔 `maintenance_delta`。
        let residue_mask = (1u64 << shift) - 1;
        let max_delta = ((u64::MAX - residue_mask) / mult).min(mask >> 1);
        let maintenance_delta = max_delta / 2;
        let scale = Self {
            mask,
            mult,
            shift,
            max_delta,
            maintenance_delta,
        };
        assert!(
            0 < scale.maintenance_ns(),
            "counter maintenance interval is below one nanosecond"
        );
        scale
    }

    /// 将硬件周期差值 `delta` 与上一次保留的小数余数 `fraction` 换算为 `(纳秒增量, 新小数余数)`。
    ///
    /// 若 `delta` 超过 `max_delta` 或 `fraction` 超出移位余数范围，返回 `None`。
    pub(crate) fn convert(self, delta: u64, fraction: u64) -> Option<(u64, u64)> {
        let residue_mask = (1u64 << self.shift) - 1;
        if self.max_delta < delta || residue_mask < fraction {
            return None;
        }
        let scaled = delta.checked_mul(self.mult)?.checked_add(fraction)?;
        Some((scaled >> self.shift, scaled & residue_mask))
    }

    /// 返回周期性基准维护间隔对应的纳秒数。
    fn maintenance_ns(self) -> u64 {
        self.convert(self.maintenance_delta, 0)
            .expect("invalid maintenance delta")
            .0
    }

    /// 根据当前采样点 `sample` 将目标单调时刻 `deadline` 换算为底层硬件定时器的绝对比较周期值（向上取整至硬件 tick）。
    fn deadline_cycles(self, sample: Sample, deadline: MonotonicInstant) -> u64 {
        if deadline <= sample.instant {
            return sample.cycle;
        }
        let nanos = deadline.0 - sample.instant.0;
        let scaled = nanos
            .checked_mul(1u64 << self.shift)
            .and_then(|value| value.checked_sub(sample.fraction))
            .expect("clockevent deadline conversion overflows");
        let ticks = scaled / self.mult + u64::from(scaled % self.mult != 0);
        assert!(
            ticks <= self.max_delta,
            "clockevent deadline exceeds safe delta"
        );
        sample
            .cycle
            .checked_add(ticks)
            .expect("clockevent absolute counter overflows")
    }
}

/// 单次一致性时间采样的结果快照。
#[derive(Clone, Copy)]
struct Sample {
    /// 换算得到的当前单调纳秒时刻。
    instant: MonotonicInstant,
    /// 本次采样读取并掩码截断后的硬件计数器周期值。
    cycle: u64,
    /// 定点换算后不足 1 纳秒的小数余数（范围 `0 .. (1 << shift)`）。
    fraction: u64,
}

/// 全局单调时间维护器（基于顺序锁 SeqLock 实现多核无锁读取与单写者周期推进）。
pub(crate) struct Timekeeper {
    /// 硬件计数器到纳秒的定点换算参数（初始化后只读）。
    scale: Scale,
    /// 串行化多核维护更新操作的写者自旋锁（获取时自动关闭本地中断）。
    writer: SpinLock<()>,
    /// 顺序锁代次序号：偶数值表示基准快照一致可读，奇数值表示写者正在更新基准字段。
    sequence: AtomicU64,
    /// 当前基准点对应的单调纳秒时间（由 `writer` 锁与 `sequence` 保护）。
    base_ns: AtomicU64,
    /// 当前基准点对应的硬件计数器周期读数（由 `writer` 锁与 `sequence` 保护）。
    cycle_last: AtomicU64,
    /// 当前基准点保留的定点换算小数余数（由 `writer` 锁与 `sequence` 保护）。
    fraction: AtomicU64,
    /// 测试辅助字段：记录基准成功更新的累计次数。
    #[cfg(feature = "ktest")]
    updates: AtomicU64,
}

impl Timekeeper {
    /// 使用指定的硬件计数器频率、位掩码与初始周期读数构造 [`Timekeeper`]。
    pub(crate) fn new(frequency: u64, mask: u64, initial_cycle: u64) -> Self {
        Self {
            scale: Scale::new(frequency, mask),
            writer: SpinLock::new(()),
            sequence: AtomicU64::new(0),
            base_ns: AtomicU64::new(0),
            cycle_last: AtomicU64::new(initial_cycle & mask),
            fraction: AtomicU64::new(0),
            #[cfg(feature = "ktest")]
            updates: AtomicU64::new(0),
        }
    }

    /// 通过顺序锁（SeqLock）无锁读取一致的基准字段并采样硬件计数器，外推计算当前时刻快照 [`Sample`]。
    fn sample(&self, counter: impl Fn() -> u64) -> Sample {
        loop {
            // Step 1: 读取当前序号；若为奇数说明写者正在更新基准，自旋等待下一轮。
            let sequence = self.sequence.load(Ordering::Acquire);
            if sequence & 1 != 0 {
                spin_loop();
                continue;
            }
            // Step 2: 读取基准纳秒、基准周期与小数余数。
            let base = self.base_ns.load(Ordering::Relaxed);
            #[cfg(feature = "ktest")]
            let overlap = testing::pause_snapshot(self);
            let last = self.cycle_last.load(Ordering::Relaxed);
            let fraction = self.fraction.load(Ordering::Relaxed);
            // Step 3: 使用编译器屏障约束无内存副作用的硬件计数器读取指令，确保采样严格落在两次序号检查之间。
            compiler_fence(Ordering::SeqCst);
            let cycle = counter() & self.scale.mask;
            compiler_fence(Ordering::SeqCst);
            // Step 4: 执行 Acquire 内存屏障并与初始序号比对；若期间写者修改了任一字段，写侧 Release 屏障保证此处观测到序号变化并重试。
            fence(Ordering::Acquire);
            if self.sequence.load(Ordering::Relaxed) != sequence {
                #[cfg(feature = "ktest")]
                if overlap {
                    testing::reject_snapshot();
                }
                continue;
            }
            return self.advance(base, last, fraction, cycle);
        }
    }

    /// 在确认顺序锁快照一致后，根据基准字段 `(base, last, fraction)` 与当前周期 `cycle` 计算最新 [`Sample`]。
    fn advance(&self, base: u64, last: u64, fraction: u64, cycle: u64) -> Sample {
        let delta = cycle.wrapping_sub(last) & self.scale.mask;
        let (nanos, fraction) = self
            .scale
            .convert(delta, fraction)
            .expect("counter delta exceeds safe sampling interval");
        Sample {
            instant: MonotonicInstant(
                base.checked_add(nanos)
                    .expect("monotonic nanoseconds exhausted"),
            ),
            cycle,
            fraction,
        }
    }

    /// 读取当前单调纳秒时刻；传入的 `counter` 闭包必须无阻塞地读取同一硬件计数源且不得重入 [`Timekeeper`]。
    pub(crate) fn read(&self, counter: impl Fn() -> u64) -> MonotonicInstant {
        self.sample(counter).instant
    }

    /// 获取写者自旋锁（关中断），采样硬件计数器并将单调时间基准向前推进。
    ///
    /// 传入的 `counter` 闭包必须无阻塞、无内存分配且不重入当前 [`Timekeeper`]。
    pub(crate) fn update(&self, counter: impl Fn() -> u64) -> MonotonicInstant {
        // Step 1: 获取关中断写锁，采样当前硬件周期并在锁内基于现有基准计算出新基准值。
        let _writer = self.writer.lock();
        compiler_fence(Ordering::SeqCst);
        let cycle = counter() & self.scale.mask;
        compiler_fence(Ordering::SeqCst);
        let sample = self.advance(
            self.base_ns.load(Ordering::Relaxed),
            self.cycle_last.load(Ordering::Relaxed),
            self.fraction.load(Ordering::Relaxed),
            cycle,
        );
        let sequence = self.sequence.load(Ordering::Relaxed);
        let complete = sequence
            .checked_add(2)
            .expect("timekeeper sequence exhausted");
        #[cfg(feature = "ktest")]
        let updates = self
            .updates
            .load(Ordering::Relaxed)
            .checked_add(1)
            .expect("timekeeper update count exhausted");

        // Step 2: 所有可能触发 panic 的算术检查均已在上方完成；将序号置为奇数通知并发读者进入更新临界区，写入新基准后再以 Release 发布新偶数序号。
        self.sequence.store(sequence + 1, Ordering::Relaxed);
        fence(Ordering::Release);
        #[cfg(feature = "ktest")]
        testing::panic_writer(self, cycle);
        self.base_ns.store(sample.instant.0, Ordering::Relaxed);
        self.cycle_last.store(sample.cycle, Ordering::Relaxed);
        self.fraction.store(sample.fraction, Ordering::Relaxed);
        #[cfg(feature = "ktest")]
        self.updates.store(updates, Ordering::Relaxed);
        self.sequence.store(complete, Ordering::Release);
        #[cfg(feature = "ktest")]
        testing::publish_snapshot_update(self);
        sample.instant
    }

    /// 测试辅助接口：返回当前 [`Timekeeper`] 使用的定点换算参数 [`Scale`]。
    #[cfg(feature = "ktest")]
    pub(crate) fn scale(&self) -> Scale {
        self.scale
    }

    /// 测试辅助接口：通过顺序锁一致性读取当前基准中保存的小数余数 `fraction`。
    #[cfg(feature = "ktest")]
    pub(crate) fn fraction(&self) -> u64 {
        loop {
            let sequence = self.sequence.load(Ordering::Acquire);
            if sequence & 1 != 0 {
                spin_loop();
                continue;
            }
            let fraction = self.fraction.load(Ordering::Relaxed);
            fence(Ordering::Acquire);
            if self.sequence.load(Ordering::Relaxed) == sequence {
                return fraction;
            }
        }
    }
}

/// 全局唯一单调时间维护器实例（由 BSP 在启动阶段初始化一次）。
static TIMEKEEPER: Once<Timekeeper> = Once::new();
/// 系统启动时刻对应的 Unix 墙钟基准时间（单位：纳秒；未校时状态下为 `0`）。
static REALTIME_BASE: Once<u64> = Once::new();

/// 读取已映射 RTC 设备窗口内 4 字节对齐的 32 位寄存器，并在读取后插入设备内存屏障。
///
/// # Safety
/// 1. `region` 必须来自当前平台已验证的 RTC 设备资源描述，且对应的永久 MMIO 虚拟映射已建立。
/// 2. `offset` 必须对应设备内合法的 32 位可读寄存器偏移。
/// 3. 调用者必须独占当前 RTC 设备的采样序列，避免与其他并发访问交错破坏硬件锁存状态。
unsafe fn rtc_register(region: Region, offset: usize) -> u32 {
    assert!(
        offset % core::mem::size_of::<u32>() == 0
            && offset
                .checked_add(core::mem::size_of::<u32>())
                .is_some_and(|end| end <= region.end - region.start),
        "RTC register exceeds its resource window"
    );
    let address = arch::mmio_address(region.start)
        .checked_add(offset)
        .expect("RTC virtual register address overflows");
    // SAFETY:
    // 1. 调用方契约保证 `region` 对应的 RTC MMIO 区域已永久映射且当前独占访问；
    // 2. 上方 `assert!` 已验证 `offset` 按 4 字节对齐且不超出 `region` 边界，因此 `address as *const u32` 对齐且有效；
    // 3. `read_volatile` 仅执行单次 MMIO 读指令，随后的 `arch::dma_barrier()` 确保该设备读操作在后续寄存器访问或计数器采样前完成。
    unsafe {
        let value = core::ptr::read_volatile(address as *const u32);
        arch::dma_barrier();
        value
    }
}

/// 解析板级 RTC 日历采样结果：若日历有效则转换为 Unix 纳秒起点，否则回退至 `0`（Unix 纪元零点）并返回诊断原因字符串。
fn rtc_realtime_base(
    sample: &Result<Option<[u32; 6]>, &'static str>,
) -> (u64, Option<&'static str>) {
    match sample {
        Ok(Some(calendar)) => match calendar_realtime(*calendar) {
            Some(realtime) => (realtime, None),
            None => (0, Some("RTC calendar is invalid or out of range")),
        },
        Ok(None) => (0, Some("no persistent RTC is available")),
        Err(error) => (0, Some(*error)),
    }
}

/// 在 BSP 启动阶段根据平台类型采样硬件 RTC，确定系统墙钟时间的初始 Unix 纳秒基准。
///
/// 若平台无持久化 RTC 或日历值无效，则打印启动诊断信息并将墙钟起点设为 `0`（不影响单调时钟初始化）。
///
/// # Safety
/// 1. `info` 必须是当前机器已验证的启动信息（[`BootInfo`]），其中描述的 RTC 及其依赖的时钟/复位控制器 MMIO 区域已完成永久映射。
/// 2. 必须在关中断（IRQ-off）且从核（AP）尚未唤醒的 BSP 单核阶段调用，保证无其他执行流并发访问 RTC 硬件。
unsafe fn boot_realtime(info: &BootInfo) -> u64 {
    match info.platform {
        Platform::QemuVirt => {
            // SAFETY: `info.platform` 已确认为 `QemuVirt`，且本函数调用方保证 BSP 关中断独占已永久映射的 QEMU RTC。
            unsafe { qemu_realtime(info) }
        }
        Platform::VisionFive2 | Platform::Ls2k1000la => {
            // SAFETY: 本函数调用方保证 BSP 处于关中断单核启动阶段，且 `info` 中的板级 RTC 及其时钟/复位控制器已完成永久 MMIO 映射。
            let sample = unsafe { arch::board_rtc_calendar(info) };
            let (realtime, unavailable) = rtc_realtime_base(&sample);
            if let Some(reason) = unavailable {
                crate::early_println!(
                    "RTC platform={:?} source=unset reason={} sample={:?}; realtime starts at Unix epoch 0",
                    info.platform,
                    reason,
                    sample
                );
            }
            realtime
        }
    }
}

/// 读取 QEMU `virt` 平台的虚拟 RTC（RISC-V Goldfish RTC 或 LoongArch LS7A TOY），返回 Unix 纳秒时间戳。
///
/// # Safety
/// 1. `info` 必须属于 QEMU `virt` 平台，且 `info.rtc` 指向的 MMIO 区域已完成永久映射。
/// 2. 必须在关中断（IRQ-off）且 AP 尚未释放的 BSP 初始化阶段调用，保证独占访问 Goldfish 锁存寄存器或 LS7A TOY 控制寄存器。
unsafe fn qemu_realtime(info: &BootInfo) -> u64 {
    #[cfg(target_arch = "riscv64")]
    {
        const TIME_LOW: usize = 0;
        const TIME_HIGH: usize = TIME_LOW + core::mem::size_of::<u32>();
        // Goldfish RTC 规定读取 `TIME_LOW` 会同时锁存同一时刻的 `TIME_HIGH`；因此必须先读低 32 位再读高 32 位。
        // SAFETY:
        // 1. 调用方保证当前独占已永久映射的 Goldfish RTC MMIO 窗口；
        // 2. `TIME_LOW` (0x0) 与 `TIME_HIGH` (0x4) 均为合法的 32 位只读寄存器偏移，`rtc_register` 内部会校验窗口边界并插入设备屏障。
        unsafe {
            let low = rtc_register(info.rtc, TIME_LOW);
            let high = rtc_register(info.rtc, TIME_HIGH);
            (u64::from(high) << u32::BITS) | u64::from(low)
        }
    }
    #[cfg(target_arch = "loongarch64")]
    {
        use crate::time::toy::{RTC_CTRL_REG, TOY_READ0_REG, TOY_READ1_REG};
        const TOY_ENABLE_MASK: u32 = toy::TOY_ENABLE | toy::TOY_32KHZ_ENABLE;
        // SAFETY:
        // 1. 调用方保证 BSP 在关中断状态下独占访问已永久映射的 LS7A TOY 寄存器窗口；
        // 2. 先通过 `rtc_register` 验证 `RTC_CTRL_REG` 落在 `info.rtc` 窗口内，再按位置位 `TOY_ENABLE` 与 `TOY_32KHZ_ENABLE` 并插入设备屏障；
        // 3. 随后循环读取 `TOY_READ1_REG`（年）与 `TOY_READ0_REG`（月日时分秒），均为合法对齐的只读寄存器。
        unsafe {
            let control = rtc_register(info.rtc, RTC_CTRL_REG);
            let address = arch::mmio_address(info.rtc.start)
                .checked_add(RTC_CTRL_REG)
                .expect("RTC control address overflows");
            core::ptr::write_volatile(address as *mut u32, control | TOY_ENABLE_MASK);
            arch::dma_barrier();
            assert_eq!(
                rtc_register(info.rtc, RTC_CTRL_REG) & TOY_ENABLE_MASK,
                TOY_ENABLE_MASK,
                "RTC TOY could not be enabled"
            );

            // `TOY_READ1_REG`（年份）不会硬件锁存 `TOY_READ0_REG`；若读取低字前后年份发生变化（跨年边界），则重新采样整组寄存器。
            loop {
                let year = rtc_register(info.rtc, TOY_READ1_REG);
                let low = rtc_register(info.rtc, TOY_READ0_REG);
                if year == rtc_register(info.rtc, TOY_READ1_REG) {
                    return rtc_calendar(year, low)
                        .expect("RTC calendar is invalid or out of range");
                }
            }
        }
    }
}

/// 将 LoongArch LS7A TOY 寄存器的年份偏移（基于 1900 年）与打包日历低字解码并换算为 Unix 纳秒时间戳。
#[cfg(any(target_arch = "loongarch64", feature = "ktest"))]
fn rtc_calendar(toy_year: u32, low: u32) -> Option<u64> {
    calendar_realtime([
        toy_year.checked_add(toy::TOY_YEAR_BASE)?,
        (low >> toy::TOY_MONTH_SHIFT) & toy::TOY_MONTH_MASK,
        (low >> toy::TOY_DAY_SHIFT) & toy::TOY_DAY_MASK,
        (low >> toy::TOY_HOUR_SHIFT) & toy::TOY_HOUR_MASK,
        (low >> toy::TOY_MINUTE_SHIFT) & toy::TOY_MINUTE_MASK,
        (low >> toy::TOY_SECOND_SHIFT) & toy::TOY_SECOND_MASK,
    ])
}

/// 将公历 `[年, 月, 日, 时, 分, 秒]` 校验并转换为自 1970-01-01 00:00:00 UTC 起的 Unix 纳秒数；日期非法或溢出时返回 `None`。
fn calendar_realtime(calendar: [u32; 6]) -> Option<u64> {
    const HOURS_PER_DAY: i64 = 24;
    const MINUTES_PER_HOUR: i64 = 60;
    const SECONDS_PER_MINUTE: i64 = 60;

    // 以 3 月 1 日为年度起点的公历天数公式相对于 1970-01-01（Unix Epoch）的天数偏移常数。
    const UNIX_EPOCH_DAY_OFFSET: i64 = 719_499;

    // Step 1: 校验月、时、分、秒的基本范围，并根据格里高利历闰年规则校验当月天数。
    let [year, month, day, hour, minute, second] = calendar.map(i64::from);
    if !(1..=12).contains(&month)
        || HOURS_PER_DAY <= hour
        || MINUTES_PER_HOUR <= minute
        || SECONDS_PER_MINUTE <= second
    {
        return None;
    }

    let leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    let days = [
        31,
        28 + i64::from(leap),
        31,
        30,
        31,
        30,
        31,
        31,
        30,
        31,
        30,
        31,
    ];
    if !(1..=days[(month - 1) as usize]).contains(&day) {
        return None;
    }

    // Step 2: 将 1 月和 2 月视作上一年的第 13、14 月，使闰日（2 月 29 日）固定落在每年末尾，从而统一计算自 1970-01-01 起的天数与纳秒数。
    let (year, month) = if month <= 2 {
        (year - 1, month + 10)
    } else {
        (year, month - 2)
    };
    let days = year / 4 - year / 100 + year / 400 + 367 * month / 12 + day + year * 365
        - UNIX_EPOCH_DAY_OFFSET;
    let seconds =
        ((days * HOURS_PER_DAY + hour) * MINUTES_PER_HOUR + minute) * SECONDS_PER_MINUTE + second;
    u64::try_from(seconds).ok()?.checked_mul(NSEC_PER_SEC)
}

/// 初始化全局时间子系统：采样平台 RTC 建立墙钟基准 [`REALTIME_BASE`]，并注册全局单调时间维护器 [`TIMEKEEPER`]。
///
/// # Safety
/// 1. 仅允许由主核（BSP）在内存管理初始化完成后、本地中断关闭（IRQ-off）且从核（AP）尚未启动时调用一次。
/// 2. `info` 必须是经校验的当前机器启动信息，其中 RTC 及其依赖的时钟、复位控制器 MMIO 区域已完成永久映射且无并发访问。
/// 3. `info.counter_frequency` 必须准确反映当前全核同步的 64 位硬件计数器频率，且初始化后不得切换底层计数源。
pub unsafe fn initialize(info: &BootInfo) {
    assert!(!arch::irq_enabled(), "time initialization requires IRQ-off");
    assert!(
        TIMEKEEPER.get().is_none(),
        "timekeeper is already initialized"
    );
    TIMEKEEPER.call_once(|| {
        // SAFETY: 当前处于 `initialize` 入口校验后的 BSP 关中断单核上下文，直接满足 `boot_realtime` 对 `info` 与永久 MMIO 映射的安全前提。
        let realtime = unsafe { boot_realtime(info) };
        compiler_fence(Ordering::SeqCst);
        let cycle = arch::read_counter();
        compiler_fence(Ordering::SeqCst);
        REALTIME_BASE.call_once(|| realtime);
        Timekeeper::new(info.counter_frequency, u64::MAX, cycle)
    });
}

/// 获取已初始化的全局 [`Timekeeper`] 引用；若在 [`initialize`] 完成前调用则直接 panic。
fn keeper() -> &'static Timekeeper {
    TIMEKEEPER
        .get()
        .expect("timekeeper has not been initialized")
}

/// 读取当前系统单调时间 [`MonotonicInstant`]（自 [`initialize`] 起算的纳秒数）。
pub fn now() -> MonotonicInstant {
    keeper().read(arch::read_counter)
}

/// 读取当前 Unix 墙钟时间（单位：纳秒，等于启动墙钟基准 [`REALTIME_BASE`] 加上当前单调时间 [`now()`]）。
pub fn realtime() -> u64 {
    REALTIME_BASE
        .get()
        .expect("realtime has not been initialized")
        .checked_add(now().as_nanos())
        .expect("realtime nanoseconds exhausted")
}

/// 将 Unix 墙钟时间域的绝对纳秒期限转换为系统单调时间域的 [`MonotonicInstant`]（早于启动墙钟基准的时刻饱和归零）。
pub(crate) fn realtime_deadline(nanos: u64) -> MonotonicInstant {
    let base = *REALTIME_BASE
        .get()
        .expect("realtime has not been initialized");
    MonotonicInstant::from_nanos(nanos.saturating_sub(base))
}

/// 每核时钟事件（Clockevent）的内部可变状态。
#[derive(Clone, Copy)]
struct EventState {
    /// 标记当前 CPU 的时钟事件是否已由调度器激活接管。
    active: bool,
    /// 当前 CPU 下一次必须执行 [`Timekeeper::update`] 的单调时间期限。
    maintenance: MonotonicInstant,
    /// 当前 CPU 上正在运行任务的时间片到期期限；若当前无任务时间片（如切出期间）则为 `None`。
    quantum: Option<MonotonicInstant>,
}

/// 存放在每核本地存储（`CpuLocal`）中的本核时钟事件控制器。
///
/// 内部的 `state` 仅由所属 CPU 在关中断（IRQ-off）上下文中通过 [`LocalEvent::with_current`] 访问。
pub(crate) struct LocalEvent {
    /// 本核时钟事件状态（激活标志、维护期限与任务时间片期限），仅限本核关中断访问。
    state: UnsafeCell<EventState>,
    /// 测试辅助字段：控制本核硬件定时器编程入口的暂停状态机（`0` / `ARMED` / `PAUSED` / `PUBLISHED`）。
    #[cfg(feature = "ktest")]
    program_pause: AtomicUsize,
    /// 测试辅助字段：记录本核最近一次成功下发到硬件定时器的目标单调纳秒期限。
    #[cfg(feature = "ktest")]
    last_programmed: AtomicU64,
}

// SAFETY:
// 1. `LocalEvent::state` 仅通过私有方法 `with_current` 访问，该方法强制断言本地中断已关闭（`!arch::irq_enabled()`）且调用方就是所属 CPU（`ptr::eq(self, &cpu::current().time_event)`）；
// 2. 跨核操作仅在测试模式下访问原子字段 `program_pause` 与 `last_programmed`，不会触碰 `state`，因此 `LocalEvent` 实现 `Sync` 是安全的。
unsafe impl Sync for LocalEvent {}

impl LocalEvent {
    /// 创建未激活的初始每核时钟事件控制器（支持静态初始化）。
    pub(crate) const fn new() -> Self {
        Self {
            state: UnsafeCell::new(EventState {
                active: false,
                maintenance: MonotonicInstant::from_nanos(0),
                quantum: None,
            }),
            #[cfg(feature = "ktest")]
            program_pause: AtomicUsize::new(0),
            #[cfg(feature = "ktest")]
            last_programmed: AtomicU64::new(0),
        }
    }

    /// 在校验本地中断已关闭且归属当前 CPU 后，对本核 [`EventState`] 执行可变借用操作。
    ///
    /// 传入的 `operation` 闭包不得触发任务上下文切换或重入 `with_current`。
    fn with_current<R>(&self, operation: impl FnOnce(&mut EventState) -> R) -> R {
        assert!(!arch::irq_enabled(), "local clockevent requires IRQ-off");
        assert!(
            core::ptr::eq(self, &cpu::current().time_event),
            "foreign CPU clockevent access"
        );
        // SAFETY:
        // 1. 上方两个 `assert!` 已确认当前处于关中断状态且 `self` 即为当前 CPU 的本地实例，不存在跨核并发访问或本地中断抢占；
        // 2. 可变引用 `&mut EventState` 的生命周期被严格限制在非重入的同步闭包 `operation` 内，不会逃逸到外部或跨越上下文切换。
        unsafe { operation(&mut *self.state.get()) }
    }
}

/// 在当前 CPU 启动调度器时激活本核时钟事件：推进一次全局时间基准、设定首次维护期限并下发硬件定时器。
pub(crate) fn activate_current_cpu() {
    cpu::current().time_event.with_current(|event| {
        assert!(!event.active, "local clockevent already activated");
        let instant = keeper().update(arch::read_counter);
        event.active = true;
        event.maintenance = maintenance_deadline(instant);
    });
    reprogram_current();
}

/// 在任务切入当前 CPU 执行时（关中断上下文），为其分配新的调度时间片（`QUANTUM`）并重新编程本核硬件定时器。
pub(crate) fn task_switch_in() {
    cpu::current().time_event.with_current(|event| {
        assert!(
            event.active && event.quantum.is_none(),
            "invalid task clockevent entry"
        );
        arm_quantum(event);
    });
    reprogram_current();
}

/// 在当前任务切出 CPU 时（关中断上下文），清除任务时间片期限并按剩余维护/定时器事件重新编程本核硬件定时器。
pub(crate) fn task_switch_out() {
    cpu::current().time_event.with_current(|event| {
        assert!(event.active, "task clockevent is not activated");
        event.quantum = None;
    });
    reprogram_current();
}

/// 时钟中断处理函数：检查并推进到期的基准维护事件、清理到期的任务时间片，随后触发软件定时器回调并重编程下一次硬件中断。
pub(crate) fn handle_timer() {
    // Step 1: 在本核 `EventState` 短暂可变借用内检查维护期限与时间片期限，并在借用结束前释放可变引用。
    let active = cpu::current().time_event.with_current(|event| {
        if !event.active {
            return false;
        }
        let instant = now();
        if event.maintenance <= instant {
            event.maintenance = maintenance_deadline(keeper().update(arch::read_counter));
        }
        if event.quantum.is_some_and(|deadline| deadline <= instant) {
            event.quantum = None;
        }
        true
    });

    // Step 2: 在释放 `EventState` 借用后执行软件定时器到期回调（允许回调内部重武装定时器），最后统一重新编程本核硬件定时器。
    if active {
        crate::timer::handle_interrupt();
        reprogram_current();
    }
}

/// 重新查询当前 CPU 的维护期限、任务时间片与软件定时器最早到期时间，并将最早期限写入本核硬件定时器。
pub(crate) fn reprogram_current() {
    let event = cpu::current()
        .time_event
        .with_current(|event| event.active.then_some(*event));
    if let Some(event) = event {
        program(event);
    }
}

/// 根据当前维护完成时刻 `instant` 计算下一次基准维护的单调时间期限。
fn maintenance_deadline(instant: MonotonicInstant) -> MonotonicInstant {
    instant
        .checked_add(Duration::from_nanos(keeper().scale.maintenance_ns()))
        .expect("counter maintenance deadline overflows")
}

/// 为指定事件状态设置从当前时刻起算的一个完整任务时间片期限（`now() + QUANTUM`）。
fn arm_quantum(event: &mut EventState) {
    event.quantum = Some(now().checked_add(QUANTUM).expect("task quantum overflows"));
}

/// 在持有本核软件定时器底座锁的临界区内，综合比较维护期限、任务时间片与软件定时器期限，将最早到期时间换算为硬件周期并写入比较器。
fn program(event: EventState) {
    #[cfg(feature = "ktest")]
    testing::pause_program();
    crate::timer::with_next_deadline(|timer_deadline| {
        let deadline = event
            .quantum
            .map_or(event.maintenance, |quantum| quantum.min(event.maintenance));
        let deadline = timer_deadline.map_or(deadline, |timer| timer.min(deadline));
        let timekeeper = keeper();
        let sample = timekeeper.sample(arch::read_counter);
        arch::program_timer(timekeeper.scale.deadline_cycles(sample, deadline));
        #[cfg(feature = "ktest")]
        cpu::current()
            .time_event
            .last_programmed
            .store(deadline.as_nanos(), Ordering::Release);
    });
}

/// 内核单元测试（`ktest`）专用的时间子系统白盒观测与并发交错控制辅助模块。
#[cfg(feature = "ktest")]
pub(crate) mod testing {
    pub(crate) use super::Timekeeper;
    use super::{arm_quantum, keeper, reprogram_current, MonotonicInstant};
    use crate::arch;
    use crate::cpu;
    use crate::task::{self, TaskId};
    use core::hint::spin_loop;
    use core::sync::atomic::{AtomicUsize, Ordering};

    /// 测试辅助接口：调用 [`super::rtc_calendar`] 对指定的 TOY 年份偏移与打包低字进行公历换算。
    pub(crate) fn rtc_calendar(year: u32, low: u32) -> Option<u64> {
        super::rtc_calendar(year, low)
    }

    /// 测试辅助接口：调用 [`super::rtc_realtime_base`] 验证板级 RTC 日历样本到 Unix 纳秒起点的解析逻辑。
    pub(crate) fn rtc_realtime_base(
        sample: Result<Option<[u32; 6]>, &'static str>,
    ) -> (u64, Option<&'static str>) {
        super::rtc_realtime_base(&sample)
    }

    const ARMED: usize = 1;
    const PAUSED: usize = 2;
    const PUBLISHED: usize = 3;
    const REJECTED: usize = 4;
    const PAUSE_TIMEOUT_SECONDS: u64 = 5;
    static OVERLAP_PHASE: AtomicUsize = AtomicUsize::new(0);
    static OVERLAP_READER: AtomicUsize = AtomicUsize::new(0);
    static OVERLAP_WRITER: AtomicUsize = AtomicUsize::new(0);
    static OVERLAPS: AtomicUsize = AtomicUsize::new(0);
    static RETRIES: AtomicUsize = AtomicUsize::new(0);
    #[cfg(feature = "ktest")]
    static FAULT_WRITER: AtomicUsize = AtomicUsize::new(0);

    /// 测试辅助接口：将顺序锁写者故障注入目标绑定到当前普通任务（定时器维护中断不会消费该故障注入点）。
    #[cfg(feature = "ktest")]
    pub(crate) fn arm_writer_fault() {
        FAULT_WRITER.store(task::current_task().id().as_usize(), Ordering::Release);
    }

    /// 测试辅助钩子：当目标普通任务持有全局 [`Timekeeper`] 写锁且已将 `sequence` 置为奇数时，触发致命故障诊断事件。
    #[cfg(feature = "ktest")]
    pub(super) fn panic_writer(timekeeper: &Timekeeper, raw: u64) {
        if core::ptr::eq(timekeeper, keeper())
            && cpu::current().hardirq_depth.load(Ordering::Relaxed) == 0
            && task::current_task_id().map(|id| id.as_usize())
                == Some(FAULT_WRITER.load(Ordering::Acquire))
        {
            let sequence = timekeeper.sequence.load(Ordering::Relaxed);
            assert_eq!(sequence & 1, 1);
            crate::ktest::fatal_event(
                "timekeeper",
                format_args!(
                    "execution timekeeper writer fault sequence={} sequence_odd=1 raw={}",
                    sequence, raw
                ),
            );
        }
    }

    /// 测试辅助接口：为目标 CPU `target` 的下一次硬件定时器编程武装锁前暂停点（必须由另一 CPU 调用并释放）。
    ///
    /// 调用前必须确保上一轮暂停点的硬件编程及中断处理函数已执行完毕，且相关测试子任务已汇合（join）。
    pub(crate) fn arm_program_pause(target: usize) {
        assert_ne!(
            target,
            cpu::current().id,
            "program pause requires another CPU"
        );
        cpu::get(target)
            .time_event
            .program_pause
            .compare_exchange(0, ARMED, Ordering::AcqRel, Ordering::Acquire)
            .expect("clockevent program pause already armed");
    }

    /// 测试辅助接口：查询目标 CPU `target` 是否已进入 `program` 并在获取定时器底座锁前处于暂停状态（`PAUSED`）。
    pub(crate) fn program_paused(target: usize) -> bool {
        cpu::get(target)
            .time_event
            .program_pause
            .load(Ordering::Acquire)
            == PAUSED
    }

    /// 测试辅助接口：放行处于 `PAUSED` 状态的目标 CPU `target`，使其继续获取定时器底座锁并完成硬件定时器编程。
    pub(crate) fn release_program_pause(target: usize) {
        cpu::get(target)
            .time_event
            .program_pause
            .compare_exchange(PAUSED, PUBLISHED, Ordering::Release, Ordering::Relaxed)
            .expect("clockevent program pause has not been reached");
    }

    /// 测试辅助接口：撤销目标 CPU `target` 尚未触发的编程暂停点，或放行已暂停的编程流程。
    ///
    /// 调用者必须在其他 CPU 上保证目标 CPU 及关联定时器对象存活，并在锁外完成任务汇合（join）与中断退出确认后才能再次武装。
    pub(crate) fn cancel_program_pause(target: usize) {
        let state = &cpu::get(target).time_event.program_pause;
        loop {
            let phase = state.load(Ordering::Acquire);
            let next = match phase {
                0 | PUBLISHED => return,
                ARMED => 0,
                PAUSED => PUBLISHED,
                _ => panic!("invalid clockevent program pause state"),
            };
            if state
                .compare_exchange(phase, next, Ordering::AcqRel, Ordering::Acquire)
                .is_ok()
            {
                return;
            }
        }
    }

    /// 测试辅助接口：读取目标 CPU `target` 最近一次完成硬件定时器编程时写入的单调纳秒期限。
    pub(crate) fn last_programmed_deadline(target: usize) -> MonotonicInstant {
        MonotonicInstant::from_nanos(
            cpu::get(target)
                .time_event
                .last_programmed
                .load(Ordering::Acquire),
        )
    }

    /// 测试辅助钩子：在 `program` 获取本核软件定时器底座锁之前检查暂停状态，若已武装则使用原始硬件计数器自旋等待放行。
    pub(super) fn pause_program() {
        let state = &cpu::current().time_event.program_pause;
        if state
            .compare_exchange(ARMED, PAUSED, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            return;
        }
        assert!(
            !arch::irq_enabled(),
            "clockevent program pause requires IRQ-off"
        );
        let limit = arch::counter_frequency()
            .checked_mul(PAUSE_TIMEOUT_SECONDS)
            .expect("clockevent program raw timeout overflows");
        let start = arch::read_counter();
        while state.load(Ordering::Acquire) == PAUSED {
            assert!(
                arch::read_counter().wrapping_sub(start) < limit,
                "clockevent program pause timed out"
            );
            spin_loop();
        }
        assert_eq!(state.load(Ordering::Acquire), PUBLISHED);
        state.store(0, Ordering::Release);
    }

    /// 测试辅助接口：在当前普通任务的关中断临界区内重置任务时间片期限并重新编程本核硬件定时器。
    ///
    /// # Safety
    /// 1. 调用者必须运行在已由调度器激活时钟事件的当前 CPU 普通任务上下文中，且本地中断已关闭（IRQ-off）、期间不发生任务切换。
    /// 2. 调用期间不得持有任何会在重编程路径中被再次获取的自旋锁或可变借用。
    pub(crate) unsafe fn rearm_current_quantum() {
        assert_ordinary_task();
        cpu::current().time_event.with_current(|event| {
            assert!(event.active, "task clockevent is not activated");
            arm_quantum(event);
        });
        reprogram_current();
    }

    /// 测试辅助接口：为全局 [`Timekeeper`] 配置一次指定读者任务 `reader` 与写者任务 `writer` 之间的跨核顺序锁并发交错握手。
    ///
    /// # Safety
    /// 1. 必须由 `reader` 任务在本地关中断（IRQ-off）的普通任务上下文中调用，且调用后须在同一关中断段内立即调用 [`super::now()`]。
    /// 2. `writer` 任务必须已在另一 CPU 上就绪运行，通过硬件计数器轮询等待 [`reader_paused`] 为真后调用 [`maintain`] 推进时间基准。
    /// 3. 读写双方不得持有阻碍对方前进的锁；必须在 `reader` 从 [`super::now()`] 返回且双方任务汇合后才允许开启下一轮握手。
    pub(crate) unsafe fn arm_snapshot_overlap(reader: TaskId, writer: TaskId) {
        assert_ordinary_task();
        assert_eq!(task::current_task_id(), Some(reader));
        assert_ne!(reader, writer, "snapshot overlap requires distinct tasks");
        assert_eq!(
            OVERLAP_PHASE.load(Ordering::Acquire),
            0,
            "snapshot overlap is already armed"
        );
        OVERLAP_READER.store(reader.as_usize(), Ordering::Relaxed);
        OVERLAP_WRITER.store(writer.as_usize(), Ordering::Relaxed);
        OVERLAPS.store(0, Ordering::Relaxed);
        RETRIES.store(0, Ordering::Relaxed);
        OVERLAP_PHASE.store(ARMED, Ordering::Release);
    }

    /// 测试辅助接口：无锁查询 `reader` 任务是否已在读取旧偶数序号与 `base_ns` 后进入暂停状态（`PAUSED`）。
    pub(crate) fn reader_paused() -> bool {
        OVERLAP_PHASE.load(Ordering::Acquire) == PAUSED
    }

    /// 测试辅助接口：返回并发交错测试期间写者完成重叠更新的次数与读者因序号变化触发重试的次数 `(overlaps, retries)`。
    pub(crate) fn snapshot_overlap_result() -> (usize, usize) {
        (
            OVERLAPS.load(Ordering::Acquire),
            RETRIES.load(Ordering::Acquire),
        )
    }

    /// 测试辅助接口：在 `reader` 任务验证完重试并从 [`super::now()`] 返回后，重置跨核交错握手状态机。
    pub(crate) fn finish_snapshot_overlap() {
        assert_eq!(
            OVERLAP_PHASE.load(Ordering::Acquire),
            REJECTED,
            "snapshot overlap has not rejected the old sequence"
        );
        OVERLAP_PHASE.store(0, Ordering::Release);
    }

    /// 测试辅助接口：在 `reader` 离开 `Timekeeper::sample` 后取消当前跨核交错握手状态机（幂等）。
    ///
    /// # Safety
    /// 1. 调用者必须独占当前测试用例的控制权，并确保 `reader` 任务已退出 `Timekeeper::sample`（例如已捕获栈展开或已完成子任务 join）。
    /// 2. 若 `reader` 仍处于 `PAUSED` 自旋状态，必须先由指定 `writer` 调用 [`maintain`] 完成更新将其放行，严禁直接取消导致 `reader` 断言失败。
    /// 3. 取消后必须等待本轮所有 `reader` 与 `writer` 任务完全退出汇合后，才可重新调用 [`arm_snapshot_overlap`]。
    pub(crate) unsafe fn cancel_snapshot_overlap() {
        loop {
            let phase = OVERLAP_PHASE.load(Ordering::Acquire);
            assert!(phase <= REJECTED, "invalid snapshot overlap state");
            if phase == 0
                || OVERLAP_PHASE
                    .compare_exchange(phase, 0, Ordering::AcqRel, Ordering::Acquire)
                    .is_ok()
            {
                return;
            }
        }
    }

    /// 断言当前执行流处于关中断（IRQ-off）的普通任务上下文（非硬中断或软中断下半部）。
    fn assert_ordinary_task() {
        assert!(!arch::irq_enabled(), "snapshot test requires IRQ-off");
        let local = cpu::current();
        assert_eq!(local.hardirq_depth.load(Ordering::Relaxed), 0);
        assert_eq!(local.bh_depth.load(Ordering::Relaxed), 0);
        assert!(
            task::current_task_id().is_some(),
            "snapshot test requires an ordinary task"
        );
    }

    /// 检查 `timekeeper` 是否为全局唯一实例，且当前执行流为非中断上下文中匹配 `target` 任务 ID 的普通任务。
    fn matches_task(timekeeper: &Timekeeper, target: &AtomicUsize) -> bool {
        if !core::ptr::eq(timekeeper, keeper()) {
            return false;
        }
        let local = cpu::current();
        local.hardirq_depth.load(Ordering::Relaxed) == 0
            && local.bh_depth.load(Ordering::Relaxed) == 0
            && task::current_task_id()
                .is_some_and(|id| id.as_usize() == target.load(Ordering::Relaxed))
    }

    /// 测试辅助钩子：当匹配的 `reader` 任务在 `Timekeeper::sample` 中读取 `base_ns` 后，将其暂停直到另一核上的 `writer` 完成一次基准更新。
    pub(super) fn pause_snapshot(timekeeper: &Timekeeper) -> bool {
        if OVERLAP_PHASE.load(Ordering::Acquire) != ARMED
            || !matches_task(timekeeper, &OVERLAP_READER)
        {
            return false;
        }
        assert_ordinary_task();
        let limit = arch::counter_frequency()
            .checked_mul(PAUSE_TIMEOUT_SECONDS)
            .expect("snapshot raw timeout overflows");
        let start = arch::read_counter();
        if OVERLAP_PHASE
            .compare_exchange(ARMED, PAUSED, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            return false;
        }
        while OVERLAP_PHASE.load(Ordering::Acquire) == PAUSED {
            assert!(
                arch::read_counter().wrapping_sub(start) < limit,
                "snapshot overlapping update timed out"
            );
            spin_loop();
        }
        assert_eq!(OVERLAP_PHASE.load(Ordering::Acquire), PUBLISHED);
        true
    }

    /// 测试辅助钩子：当匹配的 `writer` 任务在 `Timekeeper::update` 中完成字段更新并发布新偶数序号后，将握手状态推进至 `PUBLISHED` 以唤醒暂停的 `reader`。
    pub(super) fn publish_snapshot_update(timekeeper: &Timekeeper) {
        if OVERLAP_PHASE.load(Ordering::Acquire) == PAUSED
            && matches_task(timekeeper, &OVERLAP_WRITER)
        {
            OVERLAPS.fetch_add(1, Ordering::Relaxed);
            let _ = OVERLAP_PHASE.compare_exchange(
                PAUSED,
                PUBLISHED,
                Ordering::Release,
                Ordering::Relaxed,
            );
        }
    }

    /// 测试辅助钩子：当 `reader` 在 `Timekeeper::sample` 末尾检测到序号变化并准备重试时，递增重试计数并将状态标记为 `REJECTED`。
    pub(super) fn reject_snapshot() {
        RETRIES.fetch_add(1, Ordering::Relaxed);
        OVERLAP_PHASE.store(REJECTED, Ordering::Release);
    }

    /// 测试辅助接口：在测试中主动调用全局 [`Timekeeper::update`] 推进一次单调时间基准。
    pub(crate) fn maintain() -> MonotonicInstant {
        keeper().update(arch::read_counter)
    }

    /// 测试辅助接口：返回全局 [`Timekeeper`] 自启动以来完成的基准维护更新总次数。
    pub(crate) fn updates() -> u64 {
        keeper().updates.load(Ordering::Acquire)
    }
}
