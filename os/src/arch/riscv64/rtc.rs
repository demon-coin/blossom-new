//! RISC-V 64 板级实时时钟（StarFive JH7110 RTC）初始化与日历读取驱动。
//!
//! # 模块职责与架构定位
//! 本模块位于 `arch::riscv64` 架构抽象层，在系统启动早期由主核（BSP）的时间子系统初始化路径调用，
//! 负责根据设备树（FDT）解析出的 `BootInfo` 硬件资源使能 AON（Always-On）域的 RTC 时钟与复位控制器，
//! 并从板载 RTC 硬件寄存器读取当前的年、月、日、时、分、秒日历值，用于建立内核真实世界时间（Wall-clock Epoch）。
//! 若目标平台（如未配置 AON RTC 的虚拟平台）未声明相应资源，则返回 `Ok(None)`。
//!
//! # 核心设计与寄存器位域布局
//! StarFive JH7110 RTC 位于 AON 电源域，其时间（`RTC_TIME`，偏移 `0x3c`）与日期（`RTC_DATE`，偏移 `0x40`）
//! 寄存器均采用二进码十进数（BCD，每个十进制数位占 4 比特）编码：
//!
//! ```text
//! RTC_DATE 寄存器 (偏移 0x40) 位域布局:
//!  31              19 18            11 10         6 5          0
//! +------------------+----------------+------------+------------+
//! |     Reserved     |  Year (8-bit)  | Month(5-b) |  Day (6-b) |
//! |                  | BCD 00..99     | BCD 01..12 | BCD 01..31 |
//! +------------------+----------------+------------+------------+
//!                      实际年份 = 2000 + bcd(Year)
//!
//! RTC_TIME 寄存器 (偏移 0x3c) 位域布局:
//!  31              21 20            14 13         7 6          0
//! +------------------+----------------+------------+------------+
//! |     Reserved     |  Hour (7-bit)  | Minute(7-b)| Second(7-b)|
//! |                  | BCD 00..23     | BCD 00..59 | BCD 00..59 |
//! +------------------+----------------+------------+------------+
//!
//! 初始化与无撕裂读取状态机:
//!   [1. 检查并使能 AON RTC 时钟门控与分频器]
//!          │
//!          ▼
//!   [2. 解除 AON_RESET_ASSERT 复位位并轮询 AON_RESET_STATUS 就绪]
//!          │
//!          ▼
//!   [3. 置位 RTC_CONTROL 启动硬件日历并选用 24 小时制]
//!          │
//!          ▼
//!   [4. 循环读取 RTC_TIME(before) -> RTC_DATE -> RTC_TIME(after)]
//!       若 before == after 则解码 BCD 返回，避免跨秒进位导致日期/时间撕裂
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 本模块不持有任何锁。`board_rtc_calendar` 仅在引导阶段由主核（BSP）在本地中断关闭、从核（AP）尚未唤醒前单线程调用，
//!   依靠启动阶段的单核执行顺序天然保证对 AON CRG 与 RTC MMIO 寄存器的独占访问。

use crate::arch::{dma_barrier, mmio_address};
use crate::boot::{BootInfo, Region};
use crate::drivers::board_setup::{
    AON_RESET_ASSERT, AON_RESET_STATUS, CLOCK_MUX_SHIFT, DIV_MASK, ENABLE, MUX_MASK,
};

/// JH7110 AON CRG 中 RTC APB 总线时钟的设备树绑定 ID。
const AONCLK_RTC_APB: u32 = 10;
/// JH7110 AON CRG 中 RTC 内部振荡分频时钟的设备树绑定 ID。
const AONCLK_RTC_INTERNAL: u32 = 11;
/// JH7110 AON CRG 中 RTC 32.768 kHz 工作时钟选择器（Mux）的设备树绑定 ID。
const AONCLK_RTC_32K: u32 = 12;
/// JH7110 AON CRG 中 RTC 日历计数门控时钟的设备树绑定 ID。
const AONCLK_RTC_CAL: u32 = 13;
/// JH7110 AON 复位控制器中 RTC APB 接口的复位线 ID（起始值）。
const AONRST_RTC_APB: u32 = 5;
/// JH7110 AON 复位控制器中 RTC 日历模块的复位线 ID（上界值）。
const AONRST_RTC_CAL: u32 = 7;

/// AON CRG 单个时钟控制寄存器的字节宽度（4 字节）。
const CRG_REGISTER_BYTES: usize = core::mem::size_of::<u32>();
/// `AONCLK_RTC_INTERNAL` 时钟寄存器在 AON CRG 内的字节偏移。
const RTC_INTERNAL_CLOCK: usize = AONCLK_RTC_INTERNAL as usize * CRG_REGISTER_BYTES;
/// `AONCLK_RTC_32K` 时钟寄存器在 AON CRG 内的字节偏移。
const RTC_32K_CLOCK: usize = AONCLK_RTC_32K as usize * CRG_REGISTER_BYTES;
/// `AONCLK_RTC_32K` 父时钟选择位的值上界：`0` 为外部 32K 晶振，`1` 为内部 `AONCLK_RTC_INTERNAL` 分频源。
const RTC_32K_PARENT_INTERNAL: u32 = 1;
/// 解除 AON RTC 复位后轮询复位状态寄存器的最大自旋次数。
const RESET_POLL_ATTEMPTS: usize = 100_000;
/// 读取日历寄存器时为防止跨秒进位撕裂而重试的最大次数。
const CALENDAR_READ_ATTEMPTS: usize = 4;

/// RTC 控制寄存器（Control Register）的字节偏移。
const RTC_CONTROL: usize = 0;
/// RTC 时间寄存器（Time Register，含时/分/秒 BCD 码）的字节偏移。
const RTC_TIME: usize = 0x3c;
/// RTC 日期寄存器（Date Register，含年/月/日 BCD 码）的字节偏移。
const RTC_DATE: usize = 0x40;
/// `RTC_CONTROL` 启用位掩码：位 0 使能 RTC 硬件计数器，位 3 选用 24 小时制模式。
const RTC_CALENDAR_CONTROL: u32 = (1 << 0) | (1 << 3);

/// RTC 硬件两位 BCD 年份所相对的公历基准年份（2000 年）。
const RTC_YEAR_BASE: u32 = 2000;
/// `RTC_DATE` 寄存器中年份字段的起始位偏移（位 11）。
const RTC_YEAR_SHIFT: u32 = 11;
/// `RTC_DATE` 寄存器中年份字段的 8 比特掩码。
const RTC_YEAR_MASK: u32 = (1 << 8) - 1;
/// `RTC_DATE` 寄存器中月份字段的起始位偏移（位 6）。
const RTC_MONTH_SHIFT: u32 = 6;
/// `RTC_DATE` 寄存器中月份字段的 5 比特掩码。
const RTC_MONTH_MASK: u32 = (1 << 5) - 1;
/// `RTC_DATE` 寄存器中日期（日）字段的 6 比特掩码（位 `0..6`）。
const RTC_DAY_MASK: u32 = (1 << RTC_MONTH_SHIFT) - 1;
/// `RTC_TIME` 寄存器中分钟字段的起始位偏移（位 7）。
const RTC_MINUTE_SHIFT: u32 = 7;
/// `RTC_TIME` 寄存器中小时字段的起始位偏移（位 14）。
const RTC_HOUR_SHIFT: u32 = 2 * RTC_MINUTE_SHIFT;
/// `RTC_TIME` 寄存器中小时字段的 7 比特掩码。
const RTC_HOUR_MASK: u32 = (1 << 7) - 1;
/// `RTC_TIME` 寄存器中分钟字段的 7 比特掩码。
const RTC_MINUTE_MASK: u32 = (1 << RTC_MINUTE_SHIFT) - 1;
/// `RTC_TIME` 寄存器中秒字段的 7 比特掩码（位 `0..7`）。
const RTC_SECOND_MASK: u32 = (1 << RTC_MINUTE_SHIFT) - 1;

/// 从已建立内核 MMIO 映射的设备区域读取一个 4 字节对齐的 32 位寄存器。
///
/// 调用前必须处于 `board_rtc_calendar` 的启动期上下文，若 `offset` 未按 4 字节对齐或越出 `region` 边界则触发断言。
fn read(region: Region, offset: usize) -> u32 {
    assert!(offset % 4 == 0 && offset + 4 <= region.end - region.start);
    // SAFETY:
    // 1. 前置 `assert!` 已验证 `offset` 按 4 字节对齐且 `[offset, offset + 4)` 完全落在 `region` 物理范围内；
    // 2. `board_rtc_calendar` 的调用契约保证 `region` 对应的 MMIO 窗口已完成永久映射，且无并发访问。
    unsafe { ((mmio_address(region.start) + offset) as *const u32).read_volatile() }
}

/// 向已建立内核 MMIO 映射的设备区域写入一个 4 字节对齐的 32 位寄存器，并执行设备 I/O 屏障。
///
/// 仅在主核（BSP）单核启动阶段调用，写入后通过 `dma_barrier()` 确保寄存器更新在后续 MMIO 读取前生效。
fn write(region: Region, offset: usize, value: u32) {
    assert!(offset % 4 == 0 && offset + 4 <= region.end - region.start);
    // SAFETY:
    // 1. 前置 `assert!` 已验证 `offset` 按 4 字节对齐且 `[offset, offset + 4)` 完全落在 `region` 物理范围内；
    // 2. `board_rtc_calendar` 的调用契约保证 `region` 对应的 MMIO 窗口已完成永久映射，且当前由 BSP 独占初始化。
    unsafe { ((mmio_address(region.start) + offset) as *mut u32).write_volatile(value) };
    dma_barrier();
}

/// 将两位 BCD 编码字节（高 4 位为十位，低 4 位为个位）转换为十进制整数。
///
/// 若高 4 位或低 4 位出现 `10..=15` 的非十进制半字节（例如硬件日历尚未初始化），返回 `Err`。
fn bcd(value: u32) -> Result<u32, &'static str> {
    let low = value & 15;
    let high = value >> 4;
    if low < 10 && high < 10 {
        Ok(high * 10 + low)
    } else {
        Err("RTC BCD digit is invalid")
    }
}

/// 初始化板载 AON RTC 的时钟与复位线，并读取硬件日历的 `[年, 月, 日, 时, 分, 秒]`。
///
/// - 若设备树未提供 RTC 寄存器区域、时钟或复位资源，返回 `Ok(None)`，通知上层无可用的硬件挂钟时间基准。
/// - 若硬件时钟分频配置非法、复位超时、日历读取无法稳定或 BCD 字段无效，返回 `Err` 描述具体硬件异常。
///
/// # Safety
/// 1. 仅允许由主核（BSP）在本地中断关闭（IRQ-off）且从核（AP）尚未启动的单核引导阶段调用。
/// 2. `info` 必须指向已完成解析与校验的 `BootInfo`，其中 `info.rtc` 及 `info.board.rtc_setup` 引用的全部时钟/复位控制器
///    物理区间必须已建立永久内核 MMIO 映射。
/// 3. 调用期间不得有任何其他执行流并发访问 RTC 或 AON CRG 寄存器。
pub unsafe fn board_rtc_calendar(info: &BootInfo) -> Result<Option<[u32; 6]>, &'static str> {
    // Step 1: 检查设备树是否声明了完整的 RTC MMIO 区域、时钟与复位资源；若缺失则直接返回 Ok(None)。
    if info.rtc.start == info.rtc.end {
        return Ok(None);
    }
    let setup = &info.board.rtc_setup;
    if setup.clocks.count == 0 || setup.reset_count == 0 {
        return Ok(None);
    }

    // Step 2: 遍历 RTC 关联的 AON 时钟项，使能 APB/CAL 门控时钟，并校验内部时钟与 32K 时钟源的分频及父节点配置。
    for clock in &setup.clocks.entries[..setup.clocks.count] {
        let reference = clock.reference;
        match reference.id {
            AONCLK_RTC_APB | AONCLK_RTC_CAL => {
                let offset = reference.id as usize * CRG_REGISTER_BYTES;
                write(
                    reference.provider,
                    offset,
                    read(reference.provider, offset) | ENABLE,
                );
            }
            AONCLK_RTC_INTERNAL => {
                if read(reference.provider, RTC_INTERNAL_CLOCK) & DIV_MASK == 0 {
                    return Err("RTC internal clock divider is zero");
                }
            }
            AONCLK_RTC_32K => {
                let parent = (read(reference.provider, RTC_32K_CLOCK) >> CLOCK_MUX_SHIFT)
                    & (MUX_MASK >> CLOCK_MUX_SHIFT);
                if RTC_32K_PARENT_INTERNAL < parent {
                    return Err("RTC 32K clock parent is invalid");
                }
                if parent == RTC_32K_PARENT_INTERNAL
                    && read(reference.provider, RTC_INTERNAL_CLOCK) & DIV_MASK == 0
                {
                    return Err("RTC selected internal clock divider is zero");
                }
            }
            _ => return Err("RTC clock is outside the supported AON binding"),
        }
    }

    // Step 3: 逐个清除 AON 复位断言位以解除 RTC 复位，并轮询 AON_RESET_STATUS 直至所有复位线完成释放。
    for reset in &setup.resets[..setup.reset_count] {
        if !(AONRST_RTC_APB..=AONRST_RTC_CAL).contains(&reset.id) {
            return Err("RTC reset is outside the AON binding");
        }
        write(
            reset.provider,
            AON_RESET_ASSERT,
            read(reset.provider, AON_RESET_ASSERT) & !(1 << reset.id),
        );
    }
    let mut ready = false;
    for _ in 0..RESET_POLL_ATTEMPTS {
        if setup.resets[..setup.reset_count]
            .iter()
            .all(|reset| read(reset.provider, AON_RESET_STATUS) & (1 << reset.id) != 0)
        {
            ready = true;
            break;
        }
        core::hint::spin_loop();
    }
    if !ready {
        return Err("RTC AON reset deassertion timed out");
    }

    // Step 4: 置位 RTC_CONTROL 启动日历计数器与 24 小时模式，并使用“时间-日期-时间”夹心重读法消除跨秒进位撕裂，解码 BCD 后返回。
    let config = read(info.rtc, RTC_CONTROL);
    write(info.rtc, RTC_CONTROL, config | RTC_CALENDAR_CONTROL);
    for _ in 0..CALENDAR_READ_ATTEMPTS {
        let before = read(info.rtc, RTC_TIME);
        let date = read(info.rtc, RTC_DATE);
        let after = read(info.rtc, RTC_TIME);
        if before == after {
            return Ok(Some([
                RTC_YEAR_BASE + bcd((date >> RTC_YEAR_SHIFT) & RTC_YEAR_MASK)?,
                bcd((date >> RTC_MONTH_SHIFT) & RTC_MONTH_MASK)?,
                bcd(date & RTC_DAY_MASK)?,
                bcd((before >> RTC_HOUR_SHIFT) & RTC_HOUR_MASK)?,
                bcd((before >> RTC_MINUTE_SHIFT) & RTC_MINUTE_MASK)?,
                bcd(before & RTC_SECOND_MASK)?,
            ]));
        }
    }
    Err("RTC calendar did not stabilize")
}
