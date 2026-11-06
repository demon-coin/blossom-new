//! LoongArch64 TOY（Time of Year）实时时钟（RTC）日历采样模块。
//!
//! ## 模块职责与架构定位
//! 本模块位于 `arch::loongarch64` 架构层，负责在系统启动阶段从 LoongArch 平台的 TOY 实时时钟硬件寄存器中读取年、月、日、时、分、秒日历值，供上层时间子系统建立墙钟时间（`CLOCK_REALTIME`）的初始基准。
//! - **上游调用者**：`time` 子系统在引导初始化阶段调用 [`board_rtc_calendar`]。若设备树未提供 RTC 资源则返回 `Ok(None)`；若硬件寄存器中的年月日尚未配置（例如全零），由上层 `time` 模块在校验日历合法性时判定为无有效墙钟基准。
//! - **下游依赖**：使用 `arch::mmio_address` 将 RTC 物理基址转换为非缓存强序 `DMW0` 虚拟地址，使用 `arch::dma_barrier`（`dbar 0`）保证多次 MMIO 读取之间的严格硬件顺序，并引用 `time::toy` 中定义的寄存器偏移与位域常量。
//!
//! ## 核心设计与 TOY 寄存器布局
//! LoongArch TOY 实时时钟将“年份”和“月/日/时/分/秒”拆分存放在两个独立的 32 位只读寄存器（`TOY_READ1_REG` 与 `TOY_READ0_REG`）中：
//!
//! ```text
//! RTC MMIO 寄存器窗口 (base = mmio_address(info.rtc.start))：
//! +------------------+------------------+-----------------------------------------------+
//! | 偏移量 (Offset)  | 寄存器名称       | 位域结构与含义                                |
//! +------------------+------------------+-----------------------------------------------+
//! | 0x2C             | TOY_READ0_REG    | [31:26] Month  (6 bits, 1..=12)               |
//! |                  |                  | [25:21] Day    (5 bits, 1..=31)               |
//! |                  |                  | [20:16] Hour   (5 bits, 0..=23)               |
//! |                  |                  | [15:10] Minute (6 bits, 0..=59)               |
//! |                  |                  | [ 9:4 ] Second (6 bits, 0..=59)               |
//! |                  |                  | [ 3:0 ] Sub-second / 保留位                   |
//! +------------------+------------------+-----------------------------------------------+
//! | 0x30             | TOY_READ1_REG    | [31:0 ] Year offset (相对于 1900 年的偏移量)  |
//! +------------------+------------------+-----------------------------------------------+
//! | 0x40             | RTC_CTRL_REG     | bit[11] TOY_ENABLE (写 1 使能 TOY 计数器)     |
//! +------------------+------------------+-----------------------------------------------+
//!
//! 防跨年撕裂的一致性重采样流程（最多循环 MAX_CALENDAR_SAMPLES = 4 次）：
//!   read(TOY_READ1_REG) -> year_before
//!        |  (dma_barrier)
//!        v
//!   read(TOY_READ0_REG) -> low (month/day/hour/min/sec)
//!        |  (dma_barrier)
//!        v
//!   read(TOY_READ1_REG) -> year_after
//!   若 year_before == year_after，说明采样期间未发生跨年进位，解包并返回；否则重试。
//! ```
//!
//! ## 同步与并发契约
//! - 本模块不设内部锁，[`board_rtc_calendar`] 仅在主核（BSP）启动阶段（本地中断关闭、从核尚未释放）调用一次，由调用方通过 `# Safety` 契约保证对 RTC MMIO 窗口的独占访问。

use crate::arch::{dma_barrier, mmio_address};
use crate::boot::BootInfo;
use crate::time::toy::*;

/// 读取日历寄存器时允许的最大一致性重试次数（防止在跨年进位瞬间读到新旧混合的年份与月日时分秒）。
const MAX_CALENDAR_SAMPLES: usize = 4;

/// 在主核（BSP）启动阶段使能 TOY 实时时钟并采样一致的 `[年, 月, 日, 时, 分, 秒]` 日历数组。
///
/// - **调用上下文**：由主核在本地中断关闭、从核（AP）尚未唤醒的启动初始化阶段独占调用。
/// - **参数 `info`**：全局启动信息快照，其中 `info.rtc` 描述 RTC 控制器的物理地址区间。
/// - **返回值**：
///   - `Ok(None)`：当前平台未提供 RTC 硬件资源（`info.rtc` 为空区间）。
///   - `Ok(Some([year, month, day, hour, minute, second]))`：成功采样到一组无跨年撕裂的原始日历字段（年份已加上 `1900` 基数）。
///   - `Err(&'static str)`：RTC 资源窗口未按 4 字节对齐或长度不足、年份加法溢出、或连续重采样均遇到跨年跳变。
///
/// # Safety
/// 1. `info` 必须是已通过启动校验的平台启动信息，且非缓存强序直接映射窗口（`DMW0`）必须已建立并覆盖 `info.rtc` 物理地址区间。
/// 2. 调用时必须处于主核（BSP）关中断且从核（AP）尚未启动的单线程环境中，期间不得有任何其他执行流并发访问同一 RTC 控制器的 MMIO 寄存器。
pub unsafe fn board_rtc_calendar(info: &BootInfo) -> Result<Option<[u32; 6]>, &'static str> {
    // Step 1: 检查平台是否声明了 RTC 硬件区间；若存在，校验物理起始地址是否满足 4 字节对齐，且窗口大小足以覆盖最高偏移的 RTC_CTRL_REG (0x40 + 4)。
    let region = info.rtc;
    if region.start == region.end {
        return Ok(None);
    }
    if region.start & (core::mem::align_of::<u32>() - 1) != 0
        || region.end - region.start < RTC_CTRL_REG + core::mem::size_of::<u32>()
    {
        return Err("TOY RTC resource window is invalid");
    }

    // Step 2: 将 RTC 物理基址转换为强序 DMW0 虚拟地址，置位 RTC_CTRL_REG 中的 TOY_ENABLE 位以确保 TOY 计数器处于工作状态。
    let base = mmio_address(region.start);
    // SAFETY:
    // 1. 上方边界检查已确认 region.start 按 4 字节对齐且窗口长度 >= RTC_CTRL_REG + 4，因此 base + RTC_CTRL_REG 是合法且对齐的 32 位 MMIO 寄存器地址。
    // 2. 调用方契约保证了 DMW0 强序映射已建立且主核在关中断状态下独占访问 RTC 控制器。
    unsafe {
        let control = (base + RTC_CTRL_REG) as *mut u32;
        control.write_volatile(control.read_volatile() | TOY_ENABLE);
    }
    dma_barrier();

    // Step 3: 采用“读年份 -> 读月日时分秒 -> 再读年份”的三明治采样法，防止在 12 月 31 日 23:59:59 跨年瞬间读到撕裂的两组日历值。
    for _ in 0..MAX_CALENDAR_SAMPLES {
        // SAFETY:
        // 1. TOY_READ0_REG (0x2c) 与 TOY_READ1_REG (0x30) 均小于已验证的窗口上界 RTC_CTRL_REG (0x40)，且均为 4 字节对齐的只读寄存器偏移。
        // 2. 调用方契约保证主核独占 RTC 硬件访问，中间插入 dma_barrier 防止 CPU 或总线重排两次年份读取与月日时分秒读取的顺序。
        let (year, low, after) = unsafe {
            let year = ((base + TOY_READ1_REG) as *const u32).read_volatile();
            dma_barrier();
            let low = ((base + TOY_READ0_REG) as *const u32).read_volatile();
            dma_barrier();
            let after = ((base + TOY_READ1_REG) as *const u32).read_volatile();
            (year, low, after)
        };
        // Step 4: 若前后两次读取的年份一致，说明采样期间未发生跨年进位，将年份加上 1900 基准并按位域提取月、日、时、分、秒返回。
        if year == after {
            let year = year
                .checked_add(TOY_YEAR_BASE)
                .ok_or("TOY RTC year overflows")?;
            return Ok(Some([
                year,
                (low >> TOY_MONTH_SHIFT) & TOY_MONTH_MASK,
                (low >> TOY_DAY_SHIFT) & TOY_DAY_MASK,
                (low >> TOY_HOUR_SHIFT) & TOY_HOUR_MASK,
                (low >> TOY_MINUTE_SHIFT) & TOY_MINUTE_MASK,
                (low >> TOY_SECOND_SHIFT) & TOY_SECOND_MASK,
            ]));
        }
    }
    Err("TOY RTC calendar did not stabilize")
}
