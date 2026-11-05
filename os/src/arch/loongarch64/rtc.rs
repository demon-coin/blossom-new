//! TOY 日历只读取真实寄存器，未设置的年月日由时间层判定为无可信 epoch。

use crate::arch::{dma_barrier, mmio_address};
use crate::boot::BootInfo;
use crate::time::toy::*;

const MAX_CALENDAR_SAMPLES: usize = 4;

/// BSP 独占 RTC 时取得年月日时分秒，有限重采样避免跨年混合两组日历。
///
/// # Safety
/// info 必须是已验证且已发布的实际板资源，RTC 永久强序 DMW 映射已建立；
/// 仅由 IRQ 关闭、AP 尚未释放的 BSP 调用，没有其它 RTC 寄存器访问者。
pub unsafe fn board_rtc_calendar(info: &BootInfo) -> Result<Option<[u32; 6]>, &'static str> {
    let region = info.rtc;
    if region.start == region.end {
        return Ok(None);
    }
    if region.start & (core::mem::align_of::<u32>() - 1) != 0
        || region.end - region.start < RTC_CTRL_REG + core::mem::size_of::<u32>()
    {
        return Err("TOY RTC resource window is invalid");
    }

    let base = mmio_address(region.start);
    // SAFETY: 调用契约保证真实 RTC 资源和永久映射，上面的检查覆盖全部 u32 寄存器；
    // BSP 独占控制寄存器，TOY enable 不写日历，不生成默认时间。
    unsafe {
        let control = (base + RTC_CTRL_REG) as *mut u32;
        control.write_volatile(control.read_volatile() | TOY_ENABLE);
    }
    dma_barrier();

    for _ in 0..MAX_CALENDAR_SAMPLES {
        // SAFETY: year/read0 寄存器均在已验证且对齐的资源内，只有 BSP 访问；
        // 每组 volatile 读之间设备屏障保证硬件采样顺序，年变化时放弃该组。
        let (year, low, after) = unsafe {
            let year = ((base + TOY_READ1_REG) as *const u32).read_volatile();
            dma_barrier();
            let low = ((base + TOY_READ0_REG) as *const u32).read_volatile();
            dma_barrier();
            let after = ((base + TOY_READ1_REG) as *const u32).read_volatile();
            (year, low, after)
        };
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
