//! 有限硬件差值维护单调时间；每核事件统一选择维护、时间片和两类 timer 的最早期限。

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

const NSEC_PER_SEC: u64 = 1_000_000_000;
const CONVERSION_SECONDS: u64 = 3600;
const QUANTUM: Duration = Duration::from_millis(10);

/// 自时间源注册起累计的单调纳秒，不表示日历时间。
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct MonotonicInstant(u64);

impl MonotonicInstant {
    /// 构造内部绝对纳秒期限，不赋予墙钟含义。
    pub const fn from_nanos(nanos: u64) -> Self {
        Self(nanos)
    }

    /// 返回统一时间域的纳秒值。
    pub const fn as_nanos(self) -> u64 {
        self.0
    }

    /// 计算有限期限，拒绝 Duration 或结果超出内部纳秒空间。
    pub fn checked_add(self, duration: Duration) -> Option<Self> {
        let nanos = u64::try_from(duration.as_nanos()).ok()?;
        self.0.checked_add(nanos).map(Self)
    }

    /// 返回两个单调时刻的间隔，逆序时刻不产生负 Duration。
    pub fn checked_duration_since(self, earlier: Self) -> Option<Duration> {
        self.0.checked_sub(earlier.0).map(Duration::from_nanos)
    }
}

/// 注册后不变的定点换算与安全采样界限。
#[derive(Clone, Copy)]
pub(crate) struct Scale {
    pub(crate) mask: u64,
    pub(crate) mult: u64,
    pub(crate) shift: u32,
    pub(crate) max_delta: u64,
    pub(crate) maintenance_delta: u64,
}

impl Scale {
    /// 验证连续低位 mask 和频率，并选择覆盖一小时乘法的最高精度。
    fn new(frequency: u64, mask: u64) -> Self {
        assert!(
            0 < frequency && frequency <= u32::MAX as u64,
            "invalid counter frequency"
        );
        assert!(
            3 <= mask && mask & mask.wrapping_add(1) == 0,
            "invalid counter mask"
        );
        let required_cycles = frequency * CONVERSION_SECONDS;
        let (mult, shift) = (0..=32)
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

    /// 对有限差值保留小数余数；超界输入不会返回饱和的伪时间。
    pub(crate) fn convert(self, delta: u64, fraction: u64) -> Option<(u64, u64)> {
        let residue_mask = (1u64 << self.shift) - 1;
        if self.max_delta < delta || residue_mask < fraction {
            return None;
        }
        let scaled = delta.checked_mul(self.mult)?.checked_add(fraction)?;
        Some((scaled >> self.shift, scaled & residue_mask))
    }

    /// 返回提前于最大安全差值的维护间隔，留出一半差值空间供 IRQ 延迟。
    fn maintenance_ns(self) -> u64 {
        self.convert(self.maintenance_delta, 0)
            .expect("invalid maintenance delta")
            .0
    }

    /// 从同一读数的小数位置向上取整至硬件 tick，并校验最大转换范围。
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

#[derive(Clone, Copy)]
struct Sample {
    instant: MonotonicInstant,
    cycle: u64,
    fraction: u64,
}

/// 原子基准字段与序号共同提供合法的一代快照；写侧短锁只串行维护。
pub(crate) struct Timekeeper {
    scale: Scale,
    writer: SpinLock<()>,
    sequence: AtomicU64,
    base_ns: AtomicU64,
    cycle_last: AtomicU64,
    fraction: AtomicU64,
    #[cfg(feature = "ktest")]
    updates: AtomicU64,
}

impl Timekeeper {
    /// 建立相对原始计数起点，完整验证配置后才可向读者发表。
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

    /// 从匹配基准外推当前纳秒；采样必须在序号验证区间内完成。
    fn sample(&self, counter: impl Fn() -> u64) -> Sample {
        loop {
            let sequence = self.sequence.load(Ordering::Acquire);
            if sequence & 1 != 0 {
                spin_loop();
                continue;
            }
            let base = self.base_ns.load(Ordering::Relaxed);
            #[cfg(feature = "ktest")]
            let overlap = testing::pause_snapshot(self);
            let last = self.cycle_last.load(Ordering::Relaxed);
            let fraction = self.fraction.load(Ordering::Relaxed);
            // 原始 counter 指令声明 nomem；编译器屏障将它保留在两次序号读取之间。
            compiler_fence(Ordering::SeqCst);
            let cycle = counter() & self.scale.mask;
            compiler_fence(Ordering::SeqCst);
            // 若任一字段读到新写，写侧 Release fence 与此 Acquire fence 同步，
            // 奇序号先于最后序号读取；原子一致性禁止后者仍接受旧偶序号。
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

    /// 对已经稳定的基准做算术，避免把重试期间的混合字段误报为超界。
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

    /// 读取同一算法的当前时刻；采样函数只能非阻塞读取同一计数源，不能递归维护。
    pub(crate) fn read(&self, counter: impl Fn() -> u64) -> MonotonicInstant {
        self.sample(counter).instant
    }

    /// 在 IRQ-safe 短锁内重新采样并推进基准，小数余数跨维护保留。
    /// 采样函数必须非阻塞且无分配，不能重入此基准；原始采样不得在调用前缓存。
    pub(crate) fn update(&self, counter: impl Fn() -> u64) -> MonotonicInstant {
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
        // 全部可能失败的算术在奇序号前结束；写侧关 IRQ，不能被本核 reader 打断。
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

    /// 向专用验收公开注册后的真实换算参数。
    #[cfg(feature = "ktest")]
    pub(crate) fn scale(&self) -> Scale {
        self.scale
    }

    /// 返回稳定基准中的小数余数，不把观察操作当作维护更新。
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

static TIMEKEEPER: Once<Timekeeper> = Once::new();
static REALTIME_BASE: Once<u64> = Once::new();

/// 读取已映射 RTC 内完整对齐的 32 位寄存器，设备屏障维持连续采样的先后顺序。
///
/// # Safety
/// region 必须来自当前平台已验证的 RTC，且其永久 MMIO 映射已经建立；
/// offset 必须对应可读寄存器，调用者独占设备采样协议。
unsafe fn rtc_register(region: Region, offset: usize) -> u32 {
    assert!(
        offset % 4 == 0
            && offset
                .checked_add(4)
                .is_some_and(|end| end <= region.end - region.start),
        "RTC register exceeds its resource window"
    );
    let address = arch::mmio_address(region.start)
        .checked_add(offset)
        .expect("RTC virtual register address overflows");
    // SAFETY: 调用者保证实际 RTC 与永久映射；窗口校验保证完整对齐访问，
    // volatile 不建立设备内存引用，完整屏障在后续寄存器或 counter 采样前完成该读。
    unsafe {
        let value = core::ptr::read_volatile(address as *const u32);
        arch::dma_barrier();
        value
    }
}

/// RTC 只能以有效日历提供墙钟起点；不可用样本保留未校时零点及诊断原因。
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

/// BSP 按实际平台尝试RTC授时；未取得有效起点时保持未校时状态，不阻止单调计数器注册。
///
/// # Safety
/// info 必须是已验证的当前机器启动资源；存在的 RTC 及其时钟、复位 provider
/// 已永久映射，IRQ 关闭且 AP 尚未释放，没有其它 RTC 访问者。
unsafe fn boot_realtime(info: &BootInfo) -> u64 {
    match info.platform {
        Platform::QemuVirt => {
            // SAFETY: 平台身份来自已验证 BootInfo；初始化独占已映射的 QEMU RTC。
            unsafe { qemu_realtime(info) }
        }
        Platform::VisionFive2 | Platform::Ls2k1000la => {
            // SAFETY: 继承 BSP IRQ-off、AP 未释放、真实资源与 provider 永久映射的合同。
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

/// 保留 QEMU Goldfish 的纳秒锁存与 TOY 的启用、稳定采样协议。
///
/// # Safety
/// info 必须属于 QEMU virt；RTC 已永久映射，IRQ 关闭且 AP 尚未释放，
/// 调用者独占锁存和 TOY 控制寄存器。
unsafe fn qemu_realtime(info: &BootInfo) -> u64 {
    #[cfg(target_arch = "riscv64")]
    {
        // TIME_LOW 锁存同一采样的高字；必须先读低字再读取该高字。
        // SAFETY: 初始化调用者独占实际 Goldfish RTC；两个偏移均为可读寄存器，
        // rtc_register 验证宽度并在两次访问间执行设备屏障。
        unsafe {
            let low = rtc_register(info.rtc, 0);
            let high = rtc_register(info.rtc, 4);
            (u64::from(high) << 32) | u64::from(low)
        }
    }
    #[cfg(target_arch = "loongarch64")]
    {
        const TOY_ENABLE: u32 = (1 << 11) | (1 << 8);
        // SAFETY: BSP 独占 LS7A TOY；控制字只增加计数和晶振使能位，
        // 不改日历或 alarm，窗口已由平台验证，强序 DMW 与设备屏障排序读改写。
        unsafe {
            let control = rtc_register(info.rtc, 0x40);
            let address = arch::mmio_address(info.rtc.start)
                .checked_add(0x40)
                .expect("RTC control address overflows");
            core::ptr::write_volatile(address as *mut u32, control | TOY_ENABLE);
            arch::dma_barrier();
            assert_eq!(
                rtc_register(info.rtc, 0x40) & TOY_ENABLE,
                TOY_ENABLE,
                "RTC TOY could not be enabled"
            );
            // 年寄存器不锁存低字；跨年时整组重新采样，避免合成两年的日历。
            loop {
                let year = rtc_register(info.rtc, 0x30);
                let low = rtc_register(info.rtc, 0x2c);
                if year == rtc_register(info.rtc, 0x30) {
                    return rtc_calendar(year, low)
                        .expect("RTC calendar is invalid or out of range");
                }
            }
        }
    }
}

/// 解码以 1900 为起点的 TOY 年份及日历寄存器，复用完整公历校验。
#[cfg(any(target_arch = "loongarch64", feature = "ktest"))]
fn rtc_calendar(toy_year: u32, low: u32) -> Option<u64> {
    calendar_realtime([
        toy_year.checked_add(1900)?,
        (low >> 26) & 0x3f,
        (low >> 21) & 0x1f,
        (low >> 16) & 0x1f,
        (low >> 10) & 0x3f,
        (low >> 4) & 0x3f,
    ])
}

/// 将绝对公历年月日时分秒转换为 Unix 纳秒，拒绝无效日期和容量之外的样本。
fn calendar_realtime(calendar: [u32; 6]) -> Option<u64> {
    let [year, month, day, hour, minute, second] = calendar.map(i64::from);
    if !(1..=12).contains(&month) || 24 <= hour || 60 <= minute || 60 <= second {
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
    // 将一、二月移至前一年末尾，闰日统一落在最后一月。
    let (year, month) = if month <= 2 {
        (year - 1, month + 10)
    } else {
        (year, month - 2)
    };
    let days = year / 4 - year / 100 + year / 400 + 367 * month / 12 + day + year * 365 - 719499;
    let seconds = ((days * 24 + hour) * 60 + minute) * 60 + second;
    u64::try_from(seconds).ok()?.checked_mul(NSEC_PER_SEC)
}

/// BSP 一次确定持久时钟或未校时零点，并注册后续共同使用的稳定计数器。
///
/// # Safety
/// 仅 BSP 在 MM 初始化后、IRQ 关闭且 AP 尚未释放时调用一次；BootInfo 必须是
/// 当前机器已验证的资源，存在的 RTC 及其时钟、复位 provider 已永久映射且无其它
/// 访问者；频率属于当前跨 CPU 一致的 64 位硬件 counter，之后不得切换计数源。
pub unsafe fn initialize(info: &BootInfo) {
    assert!(!arch::irq_enabled(), "time initialization requires IRQ-off");
    assert!(
        TIMEKEEPER.get().is_none(),
        "timekeeper is already initialized"
    );
    TIMEKEEPER.call_once(|| {
        // SAFETY: 本入口继承 BSP 独占、IRQ-off、AP 未释放和实际 RTC/provider 已映射的启动合同。
        let realtime = unsafe { boot_realtime(info) };
        compiler_fence(Ordering::SeqCst);
        let cycle = arch::read_counter();
        compiler_fence(Ordering::SeqCst);
        REALTIME_BASE.call_once(|| realtime);
        Timekeeper::new(info.counter_frequency, u64::MAX, cycle)
    });
}

/// 借用已经完整发表的唯一时间基准，注册前调用直接失败。
fn keeper() -> &'static Timekeeper {
    TIMEKEEPER
        .get()
        .expect("timekeeper has not been initialized")
}

/// 读取生产单调时间；panic 与不可屏蔽异常不得等待可能被其打断的写者。
pub fn now() -> MonotonicInstant {
    keeper().read(arch::read_counter)
}

/// 将 RTC 起点或未校时零点与原单调经过纳秒相加，日历空间耗尽时直接失败。
pub fn realtime() -> u64 {
    REALTIME_BASE
        .get()
        .expect("realtime has not been initialized")
        .checked_add(now().as_nanos())
        .expect("realtime nanoseconds exhausted")
}

/// 将当前日历域的绝对睡眠期限转换到原单调域；起点之前的期限对应已经过去的零时刻。
pub(crate) fn realtime_deadline(nanos: u64) -> MonotonicInstant {
    let base = *REALTIME_BASE
        .get()
        .expect("realtime has not been initialized");
    MonotonicInstant::from_nanos(nanos.saturating_sub(base))
}

#[derive(Clone, Copy)]
struct EventState {
    active: bool,
    maintenance: MonotonicInstant,
    quantum: Option<MonotonicInstant>,
}

/// CpuLocal 中预备的本核事件状态，只由所属 CPU 在 IRQ-off 边界访问。
pub(crate) struct LocalEvent {
    state: UnsafeCell<EventState>,
    #[cfg(feature = "ktest")]
    program_pause: AtomicUsize,
    #[cfg(feature = "ktest")]
    last_programmed: AtomicU64,
}

// SAFETY: state 只经 with_current 访问；它校验所属 CPU 和 IRQ-off，远端只发送重编程提示。
unsafe impl Sync for LocalEvent {}

impl LocalEvent {
    /// 静态准备尚未接管硬件的本核事件，早期手动 timer 保持原行为。
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

    /// 将可变事件借用限制在所属 CPU 的 IRQ-off 调用内，不跨任务切换。
    fn with_current<R>(&self, operation: impl FnOnce(&mut EventState) -> R) -> R {
        assert!(!arch::irq_enabled(), "local clockevent requires IRQ-off");
        assert!(
            core::ptr::eq(self, &cpu::current().time_event),
            "foreign CPU clockevent access"
        );
        // SAFETY: 对象为当前核的内嵌事件，IRQ 已关闭，私有方法不逃逸引用或调用调度。
        unsafe { operation(&mut *self.state.get()) }
    }
}

/// 在本核 scheduler 开始时接管一次事件；每个已激活 CPU 独立保证维护期限。
pub(crate) fn activate_current_cpu() {
    cpu::current().time_event.with_current(|event| {
        assert!(!event.active, "local clockevent already activated");
        let instant = keeper().update(arch::read_counter);
        event.active = true;
        event.maintenance = maintenance_deadline(instant);
    });
    reprogram_current();
}

/// 当前任务真实切入时建立新时间片，不继承旧 CPU 或旧执行段的期限。
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

/// 当前任务真实切出时移除时间片，保留本核计数器维护事件。
pub(crate) fn task_switch_out() {
    cpu::current().time_event.with_current(|event| {
        assert!(event.active, "task clockevent is not activated");
        event.quantum = None;
    });
    reprogram_current();
}

/// 先结束事件维护借用，再执行允许重新武装 timer 的硬回调，最后重查本核期限。
pub(crate) fn handle_timer() {
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
    if active {
        crate::timer::handle_interrupt();
        reprogram_current();
    }
}

/// 重查本核真实最早事件并编程；远端提示不携带可能失效的旧期限。
pub(crate) fn reprogram_current() {
    let event = cpu::current()
        .time_event
        .with_current(|event| event.active.then_some(*event));
    if let Some(event) = event {
        program(event);
    }
}

/// 从本次真实维护基准建立下一次期限，纳秒空间耗尽直接失败。
fn maintenance_deadline(instant: MonotonicInstant) -> MonotonicInstant {
    instant
        .checked_add(Duration::from_nanos(keeper().scale.maintenance_ns()))
        .expect("counter maintenance deadline overflows")
}

/// 对当前执行段重建一次真实调度期限，提交前由调用者结束可变事件借用。
fn arm_quantum(event: &mut EventState) {
    event.quantum = Some(now().checked_add(QUANTUM).expect("task quantum overflows"));
}

/// 在同一本核 base 锁下最终重查两类 timer，并保持锁直到真实硬件编程结束。
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

/// 专用验收只公开生产对象与同一算法，不维护第二套时间模型。
#[cfg(feature = "ktest")]
pub(crate) mod testing {
    pub(crate) use super::Timekeeper;
    use super::{arm_quantum, keeper, reprogram_current, MonotonicInstant};
    use crate::arch;
    use crate::cpu;
    use crate::task::{self, TaskId};
    use core::hint::spin_loop;
    use core::sync::atomic::{AtomicUsize, Ordering};

    /// 对独立字段调用生产日历换算，不访问或改写实际 RTC。
    pub(crate) fn rtc_calendar(year: u32, low: u32) -> Option<u64> {
        super::rtc_calendar(year, low)
    }

    /// 对独立RTC样本执行生产启动授时规则，不访问硬件或改写已发布的时间基准。
    pub(crate) fn rtc_realtime_base(
        sample: Result<Option<[u32; 6]>, &'static str>,
    ) -> (u64, Option<&'static str>) {
        super::rtc_realtime_base(&sample)
    }

    const ARMED: usize = 1;
    const PAUSED: usize = 2;
    const PUBLISHED: usize = 3;
    const REJECTED: usize = 4;
    static OVERLAP_PHASE: AtomicUsize = AtomicUsize::new(0);
    static OVERLAP_READER: AtomicUsize = AtomicUsize::new(0);
    static OVERLAP_WRITER: AtomicUsize = AtomicUsize::new(0);
    static OVERLAPS: AtomicUsize = AtomicUsize::new(0);
    static RETRIES: AtomicUsize = AtomicUsize::new(0);
    #[cfg(feature = "ktest")]
    static FAULT_WRITER: AtomicUsize = AtomicUsize::new(0);

    /// 将故障限定到已等待 printer 真阻塞的当前普通 Task，维护 IRQ 不消费停点。
    #[cfg(feature = "ktest")]
    pub(crate) fn arm_writer_fault() {
        FAULT_WRITER.store(task::current_task().id().as_usize(), Ordering::Release);
    }

    /// 仅在生产写锁和真实奇序号内报告写者故障；终结路径不读取单调时间或普通日志。
    #[cfg(feature = "ktest")]
    pub(super) fn panic_writer(timekeeper: &Timekeeper, raw: u64) {
        if core::ptr::eq(timekeeper, keeper())
            && cpu::current().hardirq_depth.load(Ordering::Relaxed) == 0
            && task::current_task_id().map(|id| id.as_usize())
                == Some(FAULT_WRITER.load(Ordering::Acquire))
        {
            let sequence = timekeeper.sequence.load(Ordering::Relaxed);
            assert_eq!(sequence & 1, 1);
            #[cfg(feature = "ktest")]
            crate::ktest::fatal_event(
                "timekeeper",
                format_args!(
                    "execution timekeeper writer fault sequence={} sequence_odd=1 raw={}",
                    sequence, raw
                ),
            );
            #[cfg(not(feature = "ktest"))]
            {
                crate::console::testing::emergency_write(format_args!(
                    "EXECUTION X27 state fault=timekeeper sequence={} sequence_odd=1 raw={}\n",
                    sequence, raw
                ));
                panic!("execution timekeeper writer fault");
            }
        }
    }

    /// 为指定 CPU 的下一次真实硬件提交武装锁前停点，须由另一 CPU 释放。
    /// 上一次停点的真实提交与 handler 已返回、相关生产子任务已 join 后才可重新武装。
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

    /// 观察目标已进入真实提交入口，但尚未持有 timer base 锁。
    pub(crate) fn program_paused(target: usize) -> bool {
        cpu::get(target)
            .time_event
            .program_pause
            .load(Ordering::Acquire)
            == PAUSED
    }

    /// 释放真实提交停点；目标随后在 base 锁内重新选择已发表的最早节点。
    pub(crate) fn release_program_pause(target: usize) {
        cpu::get(target)
            .time_event
            .program_pause
            .compare_exchange(PAUSED, PUBLISHED, Ordering::Release, Ordering::Relaxed)
            .expect("clockevent program pause has not been reached");
    }

    /// 撤销未到达的提交停点，或放行已暂停的真实提交；不撤销硬件事件且不等待 handler。
    /// 控制者须在另一 CPU 保持目标与相关 timer 存储有效，锁外完成真实 close/join 和
    /// handler 尾部确认后才能重新武装；状态归零本身不证明提交或中断已结束。
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

    /// 读取真实硬件编程结束后发表的统一纳秒期限，不把候选期限冒充提交证据。
    pub(crate) fn last_programmed_deadline(target: usize) -> MonotonicInstant {
        MonotonicInstant::from_nanos(
            cpu::get(target)
                .time_event
                .last_programmed
                .load(Ordering::Acquire),
        )
    }

    /// 只在真实提交取最终 base 锁之前暂停；原始计数器限制等待且不重入时间事件。
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
            .checked_mul(5)
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

    /// 在当前普通任务的 IRQ-off 段重建时间片，沿用生产事件和硬件提交。
    ///
    /// # Safety
    /// 调用者必须为本核普通任务，IRQ 已关闭且不跨任务切换；本核事件已由
    /// scheduler 激活，调用期间不得持有会重入时间事件的锁或借用。
    pub(crate) unsafe fn rearm_current_quantum() {
        assert_ordinary_task();
        cpu::current().time_event.with_current(|event| {
            assert!(event.active, "task clockevent is not activated");
            arm_quantum(event);
        });
        reprogram_current();
    }

    /// 为唯一生产基准准备一次指定任务的跨核快照交错。
    ///
    /// # Safety
    /// reader 必须在本核普通任务的 IRQ-off 段调用，并在同一段立即调用 now，
    /// 期间不切换或先调用其他时间读接口；writer 必须已在另一 CPU 可运行，
    /// 以原始 counter 有限等待 reader_paused 后调用 maintain。双方不得持有
    /// 对方推进所需的锁；reader 的 now 返回且双方真实 join 后才可重新注册。
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

    /// 无锁观察 reader 是否已经读取旧偶序号和 base，写者不调用 now 等待。
    pub(crate) fn reader_paused() -> bool {
        OVERLAP_PHASE.load(Ordering::Acquire) == PAUSED
    }

    /// 返回真实重叠更新数和由末序号不匹配产生的拒绝重试数。
    pub(crate) fn snapshot_overlap_result() -> (usize, usize) {
        (
            OVERLAPS.load(Ordering::Acquire),
            RETRIES.load(Ordering::Acquire),
        )
    }

    /// 在 reader 的 now 已返回后关闭一次完成的握手，不留下生产停点。
    pub(crate) fn finish_snapshot_overlap() {
        assert_eq!(
            OVERLAP_PHASE.load(Ordering::Acquire),
            REJECTED,
            "snapshot overlap has not rejected the old sequence"
        );
        OVERLAP_PHASE.store(0, Ordering::Release);
    }

    /// 原 reader 离开 sample 后撤销本例快照握手；重复取消空状态不改变生产基准。
    /// 只修改停点，不跳过 writer 的真实字段写或伪造 PUBLISHED，不等待 reader/writer。
    ///
    /// # Safety
    /// 调用者须独占本例控制权，并证明原 reader 已退出本次 sample 或不会再进入它：
    /// 本体 catch 退栈或 reader 子任务真实 join 可提供该证明。仍暂停的 reader 必须
    /// 先由指定 writer 真实 maintain 发布 even 并放行，再在原同 CPU IRQ-off 段退出
    /// sample；不得以取消替代这次生产写。取消后仍保活双方，真实 join 全部 writer
    /// 和 reader 以前不得重新武装，避免旧 producer 的 CAS 消费下一例的同值状态。
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

    /// 校验停点只位于本核普通执行段，IRQ 不能在等待期间重入时间读取。
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

    /// 只识别唯一生产基准与指定普通任务，独立算术实例不访问 CPU 状态。
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

    /// 在旧代 even 和 base 之后暂停一次 reader；等待只依赖原始 counter。
    pub(super) fn pause_snapshot(timekeeper: &Timekeeper) -> bool {
        if OVERLAP_PHASE.load(Ordering::Acquire) != ARMED
            || !matches_task(timekeeper, &OVERLAP_READER)
        {
            return false;
        }
        assert_ordinary_task();
        let limit = arch::counter_frequency()
            .checked_mul(5)
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

    /// 写者完成真实字段写和 even 发布以后才释放持旧 base 的 reader。
    /// 原 reader 已退出时取消可先关闭握手；CAS 防止仍在真实 update 尾部的旧 writer 复活停点。
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

    /// 只在原 sample 的末序号实际不匹配时记录拒绝，不把暂停当成重试。
    pub(super) fn reject_snapshot() {
        RETRIES.fetch_add(1, Ordering::Relaxed);
        OVERLAP_PHASE.store(REJECTED, Ordering::Release);
    }

    /// 真实推进生产基准，写锁与序号协议和本地维护 IRQ 完全相同。
    pub(crate) fn maintain() -> MonotonicInstant {
        keeper().update(arch::read_counter)
    }

    /// 返回生产基准真实完成的维护更新次数。
    pub(crate) fn updates() -> u64 {
        keeper().updates.load(Ordering::Acquire)
    }
}
