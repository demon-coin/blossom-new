//! 板 RTC 只消费固件树声明的 AON 时钟、复位和日历，不写入构建时间。

use crate::arch::{dma_barrier, mmio_address};
use crate::boot::{BootInfo, Region};
use crate::drivers::board_setup::{
    AON_RESET_ASSERT, AON_RESET_STATUS, CLOCK_MUX_SHIFT, DIV_MASK, ENABLE, MUX_MASK,
};

const AONCLK_RTC_APB: u32 = 10;
const AONCLK_RTC_INTERNAL: u32 = 11;
const AONCLK_RTC_32K: u32 = 12;
const AONCLK_RTC_CAL: u32 = 13;
const AONRST_RTC_APB: u32 = 5;
const AONRST_RTC_CAL: u32 = 7;

const CRG_REGISTER_BYTES: usize = core::mem::size_of::<u32>();
const RTC_INTERNAL_CLOCK: usize = AONCLK_RTC_INTERNAL as usize * CRG_REGISTER_BYTES;
const RTC_32K_CLOCK: usize = AONCLK_RTC_32K as usize * CRG_REGISTER_BYTES;
const RTC_32K_PARENT_INTERNAL: u32 = 1;
const RESET_POLL_ATTEMPTS: usize = 100_000;
const CALENDAR_READ_ATTEMPTS: usize = 4;

const RTC_CONTROL: usize = 0;
const RTC_TIME: usize = 0x3c;
const RTC_DATE: usize = 0x40;
const RTC_CALENDAR_CONTROL: u32 = (1 << 0) | (1 << 3);

const RTC_YEAR_BASE: u32 = 2000;
const RTC_YEAR_SHIFT: u32 = 11;
const RTC_YEAR_MASK: u32 = (1 << 8) - 1;
const RTC_MONTH_SHIFT: u32 = 6;
const RTC_MONTH_MASK: u32 = (1 << 5) - 1;
const RTC_DAY_MASK: u32 = (1 << RTC_MONTH_SHIFT) - 1;
const RTC_MINUTE_SHIFT: u32 = 7;
const RTC_HOUR_SHIFT: u32 = 2 * RTC_MINUTE_SHIFT;
const RTC_HOUR_MASK: u32 = (1 << 7) - 1;
const RTC_MINUTE_MASK: u32 = (1 << RTC_MINUTE_SHIFT) - 1;
const RTC_SECOND_MASK: u32 = (1 << RTC_MINUTE_SHIFT) - 1;

/// 读取已映射、完整对齐的设备寄存器，调用者已验证资源来源。
fn read(region: Region, offset: usize) -> u32 {
    assert!(offset % 4 == 0 && offset + 4 <= region.end - region.start);
    // SAFETY: board_rtc_calendar 的调用契约保证实际资源与永久映射，范围检查覆盖完整 u32。
    unsafe { ((mmio_address(region.start) + offset) as *const u32).read_volatile() }
}

/// BSP 独占初始化时写设备寄存器，并在下一次访问前完成设备排序。
fn write(region: Region, offset: usize, value: u32) {
    assert!(offset % 4 == 0 && offset + 4 <= region.end - region.start);
    // SAFETY: 资源来自已验证的 BootInfo，BSP 独占初始化；寄存器偏移及四字节宽度有效。
    unsafe { ((mmio_address(region.start) + offset) as *mut u32).write_volatile(value) };
    dma_barrier();
}

/// 拒绝非十进制 BCD 数位，保留硬件未设置日历的失败状态。
fn bcd(value: u32) -> Result<u32, &'static str> {
    let low = value & 15;
    let high = value >> 4;
    if low < 10 && high < 10 {
        Ok(high * 10 + low)
    } else {
        Err("RTC BCD digit is invalid")
    }
}

/// 读取实际板日历的年月日时分秒；无启用 RTC 时明确没有可信 epoch。
///
/// # Safety
/// 只能由 BSP 在 IRQ 关闭、AP 释放前调用，info 必须是已经发布并验证的真实资源；
/// RTC 与所有时钟/复位 provider 的永久 MMIO 映射必须已建立，且没有其它 RTC 使用者。
pub unsafe fn board_rtc_calendar(info: &BootInfo) -> Result<Option<[u32; 6]>, &'static str> {
    if info.rtc.start == info.rtc.end {
        return Ok(None);
    }
    let setup = &info.board.rtc_setup;
    if setup.clocks.count == 0 || setup.reset_count == 0 {
        return Ok(None);
    }

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
