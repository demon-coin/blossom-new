//! 启动阶段（Boot）地址空间映射、页表/DMW 窗口与多核（SMP）硬件探测测试套件。
//!
//! # 模块定位
//! 本模块隶属于内核自检框架（`ktest`）的启动阶段用例集（`boot_cases`），负责验证：
//! 1. **启动与最终内核地址映射（`mapping` / `rv_mapping`）**：高地址代码与栈映射、早期低地址恒等映射撤销、
//!    `no-map` 物理页与普通 RAM 页的页表属性（RISC-V Sv39 三级页表遍历或 LoongArch64 CRMD/DMW CSR 配置）、
//!    引导根页表（`RV_BOOT_ROOT`）布局、DMA 缓存刷新边界以及串口应急输出预算。
//! 2. **多核硬件状态与中断现场（`cpus` / `rv_fp`）**：通过固定核子任务（`spawn_on`）在系统中每一个在线 CPU 上
//!    并发检验 `CpuLocal` 身份与启动栈/任务栈范围、单核硬件定时器中断编程与恢复、通用/浮点寄存器跨中断保存恢复、
//!    嵌套关中断守卫（`IrqSaveGuard`）与自旋锁交互，以及自核处理器间中断（Self-IPI）。
//!
//! # 核心机制与地址/多核探测架构
//!
//! ```text
//! 1. 地址空间映射检验
//!    - RISC-V 64 (Sv39):
//!      SATP (MODE=8) / RV_BOOT_ROOT ──► L2 ──► L1 ──► L0 ──► (PPN, Flags)
//!        ├─ 低地址恒等映射 (0x8000_0000..)  : 最终页表中已卸载 (None)
//!        ├─ 内核直映区 (phys_to_virt)       : 普通 RAM 映射为 RW (0x6)，no-map 页无映射 (None)
//!        └─ MMIO 映射区 (mmio_address)      : UART 映射为 RW (0x6)
//!    - LoongArch64 (DMW):
//!      CRMD (PG=1, DA=0) + DMW1 (Cached, PLV0) + DMW0 (Uncached/Device, PLV0)
//!
//! 2. 多核固定任务探测 (CpuFixture)
//!    主测试线程 (every_cpu)
//!      ├─► spawn_on(CPU 0) ──► PreemptGuard ──► identity / timer / registers / irq_save / self_ipi ──► results[0] = 1
//!      ├─► spawn_on(CPU 1) ──► PreemptGuard ──► ...                                                ──► results[1] = 1
//!      └─► ...
//!      └─► wait(try_join) 收集所有 CPU 子任务离栈并校验 results[i] == 1
//! ```
//!
//! # 锁与并发契约
//! - 在固定核子任务内部操作本核定时器、IPI 或校验 `CpuLocal` 身份时，必须先持有 `PreemptGuard` 禁止内核抢占与跨核迁移。
//! - 临时改写本核硬件比较定时器（`arch::program_timer`）的测试函数必须绑定 `TimerRestore` 守卫，确保退出前在关中断状态下
//!   调用 `time::reprogram_current()` 恢复内核时间子系统的下一个到期事件。

use crate::ktest::{spawn_on, ChildHandle};
#[cfg(target_arch = "riscv64")]
use crate::mm;
use crate::sync::{IrqSaveGuard, PreemptGuard, SpinLock};
use crate::{arch, boot, cpu, sched, time};
use ::ktest::{Fixture, TestContext, TestError, TestResult};
use alloc::sync::Arc;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

pub(super) mod stacks;

#[cfg(target_arch = "loongarch64")]
pub(crate) mod fcc;

/// 在硬件计数器（Counter）的 5 秒超时窗口内轮询谓词 `predicate`；每次未满足时主动调用 `sched::yield_now()` 让出 CPU，
/// 若超时仍未满足则返回携带 `reason` 的测试错误。
pub(super) fn wait(mut predicate: impl FnMut() -> bool, reason: &'static str) -> TestResult {
    let deadline = arch::read_counter().wrapping_add(arch::counter_frequency() * 5);
    while !predicate() {
        if deadline <= arch::read_counter() {
            return Err(TestError(reason));
        }
        sched::yield_now();
    }
    Ok(())
}

/// 读取当前 RISC-V `satp` 寄存器，断言其处于 Sv39 分页模式（`MODE == 8`），并从当前活动根页表物理基址开始查询虚拟地址 `address` 的映射结果。
///
/// 若映射有效，返回 `Some((物理地址, 低 8 位 PTE 标志位))`；若任一级页表项无效则返回 `None`。
#[cfg(target_arch = "riscv64")]
fn translation(address: usize) -> Option<(usize, usize)> {
    let satp: usize;
    // SAFETY: 当前运行在 RISC-V S-mode 下，`csrr satp` 仅读取页表基址寄存器，不修改任何内存或地址翻译状态。
    unsafe {
        core::arch::asm!("csrr {value}, satp", value = out(reg) satp, options(nomem, nostack));
    }
    assert_eq!(satp >> 60, 8, "final SATP is not Sv39");
    translation_from((satp & arch::SATP_PPN_MASK) << mm::PAGE_SHIFT, address)
}

/// 从指定的 Sv39 根页表物理地址 `physical` 出发，只读遍历三级页表查询虚拟地址 `address` 对应的物理地址与页表项低 8 位属性。
///
/// 遍历过程中会断言每一级页表页均位于内核镜像永久保留的物理内存区间内。
#[cfg(target_arch = "riscv64")]
fn translation_from(mut physical: usize, address: usize) -> Option<(usize, usize)> {
    for level in (0..arch::SV39_LEVELS).rev() {
        let kernel = boot::kernel_region();
        assert!(
            kernel.start <= physical && physical + mm::PAGE_SIZE <= kernel.end,
            "page table outside kernel reservation"
        );
        let index =
            (address >> (mm::PAGE_SHIFT + level * arch::PT_INDEX_BITS)) & arch::PT_INDEX_MASK;
        // SAFETY:
        // 1. 上方 `assert!` 已确认当前页表物理页 `physical..physical + PAGE_SIZE` 完全落在内核永久保留区内；
        // 2. `index` 经 `PT_INDEX_MASK`（`0x1ff`）掩码截断，偏移不超出单个 4 KiB 页表页；
        // 3. 内核启动页表与最终根页表在启动完成后只读，通过直接映射区读取不会产生并发数据竞争。
        let entry = unsafe {
            core::ptr::read_volatile((arch::phys_to_virt(physical) as *const usize).add(index))
        };
        if entry & arch::PTE_VALID == 0 {
            return None;
        }
        if entry & (arch::PTE_READ | arch::PTE_WRITE | arch::PTE_EXEC) != 0 {
            let mask = (1usize << (mm::PAGE_SHIFT + level * arch::PT_INDEX_BITS)) - 1;
            return Some((
                ((entry >> arch::PTE_PPN_SHIFT) << mm::PAGE_SHIFT) | (address & mask),
                entry & u8::MAX as usize,
            ));
        }
        physical = (entry >> arch::PTE_PPN_SHIFT) << mm::PAGE_SHIFT;
    }
    None
}

/// 检查物理地址 `address` 在最终内核页表中的直接映射状态：
/// 若 `address` 位于 RAM 内且所属物理页包含 `no-map` 保留区，则断言其直映虚拟地址未建立映射（`None`）；
/// 若属于普通可映射 RAM 页，则断言其直映虚拟地址映射到相同物理地址且具备可读写、不可执行权限（`R|W == 0x6`）。
#[cfg(target_arch = "riscv64")]
fn expect_ram_mapping(context: &mut TestContext<'_>, info: &boot::BootInfo, address: usize) {
    if !info.memory[..info.memory_count]
        .iter()
        .any(|ram| ram.start <= address && address < ram.end)
    {
        return;
    }
    let no_map = info.reserved[..info.reserved_count]
        .iter()
        .zip(&info.reserved_no_map[..info.reserved_count])
        .any(|(region, no_map)| {
            *no_map
                && region.start / mm::PAGE_SIZE <= address / mm::PAGE_SIZE
                && address / mm::PAGE_SIZE < region.end.div_ceil(mm::PAGE_SIZE)
        });
    if no_map {
        context.expect_eq(translation(arch::phys_to_virt(address)), None);
    } else {
        let (physical, flags) =
            translation(arch::phys_to_virt(address)).expect("actual RAM direct map missing");
        context.expect_eq(physical, address);
        context.expect_eq(flags & 0xe, 0x6);
    }
}

#[cfg(target_arch = "loongarch64")]
/// 在 LoongArch64 PLV0 特权级下读取当前模式信息寄存器 `CRMD` 以及直接映射配置窗口寄存器 `DMW1`（Cached）、`DMW0`（Device）。
fn mapping_csrs() -> [usize; 3] {
    let crmd;
    let cached;
    let device;
    // SAFETY: 当前运行在 LoongArch64 PLV0 内核态，`csrrd` 指令仅读取架构标准控制寄存器，不修改地址映射模式或栈状态。
    unsafe {
        core::arch::asm!("csrrd {crmd}, {csr_crmd}", "csrrd {cached}, {dmw1}", "csrrd {device}, {dmw0}",
        crmd = out(reg) crmd, cached = out(reg) cached, device = out(reg) device,
        csr_crmd = const arch::CSR_CRMD, dmw1 = const arch::CSR_DMW1,
        dmw0 = const arch::CSR_DMW0, options(nomem, nostack));
    }
    [crmd, cached, device]
}

#[::ktest::suite(context = "Boot", tags("boot", "mapping"), timeout_ms = 1000)]
pub(crate) mod mapping {
    use super::*;

    /// 测试目标：验证当前正在执行的内核代码地址与栈变量地址均位于最终高地址内核空间，且能正确与物理地址双向转换。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 取当前栈上局部变量地址 `stack` 与测试函数入口地址 `code`。
    /// - When: 将两者通过 `arch::virt_to_phys` 转换为物理地址，并在 RISC-V 下通过页表遍历核对映射，在 LoongArch64 下通过 `phys_to_virt` 核对 DMW 映射。
    /// - Then: 转换所得物理地址均落在 `boot::kernel_region()` 范围内，且虚拟地址到物理地址的映射关系完全一致。
    #[test]
    fn final_address(context: &mut TestContext<'_>) {
        let probe = 0usize;
        let stack = &probe as *const usize as usize;
        let code = final_address as usize;
        let kernel = boot::kernel_region();
        for address in [stack, code] {
            let physical = arch::virt_to_phys(address);
            assert!(kernel.start <= physical && physical < kernel.end);
            #[cfg(target_arch = "loongarch64")]
            context.expect_eq(address, arch::phys_to_virt(physical));
            #[cfg(target_arch = "riscv64")]
            {
                let (mapped, _) = translation(address).expect("current address not mapped");
                context.expect_eq(mapped, physical);
            }
        }
        context.expect_eq(
            arch::virt_to_phys(arch::phys_to_virt(kernel.start)),
            kernel.start,
        );
        context.diagnostic(format_args!(
            "final-address pc={:#x} sp={:#x} kernel={:#x}..{:#x}",
            code, stack, kernel.start, kernel.end
        ));
    }

    /// 测试目标：验证早期启动字清零函数 `arch::boot_clear_words` 严格只清零半开区间 `[start, end)`，不越界破坏空区间或区间两端的相邻字。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 在栈上初始化一个包含 10 个哨兵字（`SENTINEL`）的数组 `words`。
    /// - When: 先对空区间 `[start, start)` 调用 `boot_clear_words`；再对内部子区间 `[start + word_size, start + 9 * word_size)` 调用 `boot_clear_words`。
    /// - Then: 空区间调用后全部 10 个元素保持 `SENTINEL`；内部子区间调用后首尾元素 `words[0]`、`words[9]` 仍为 `SENTINEL`，中间 `words[1..9]` 全部被清零。
    #[test]
    fn boot_clear_interval() {
        const SENTINEL: usize = 0x9d37_51ab_c462_e806;
        let mut words = [SENTINEL; 10];
        let start = core::ptr::addr_of_mut!(words).cast::<usize>() as usize;
        let word_size = core::mem::size_of::<usize>();
        // SAFETY: `start` 指向当前函数栈上独占且按 `usize` 对齐的数组起始处，半开空区间 `[start, start)` 长度为 0，不会写入任何内存。
        unsafe { arch::boot_clear_words(start, start) };
        assert!(words.iter().all(|&value| value == SENTINEL));
        // SAFETY: `[start + word_size, start + 9 * word_size)` 严格落在 10 元素数组 `words[1..9]` 内部，且按机器字对齐，不会越界覆盖相邻栈帧。
        unsafe { arch::boot_clear_words(start + word_size, start + 9 * word_size) };
        assert_eq!(words[0], SENTINEL);
        assert_eq!(words[9], SENTINEL);
        assert!(words[1..9].iter().all(|&value| value == 0));
    }

    /// 测试目标：验证启动完成后早期低地址恒等映射已被撤销，同时验证 `no-map` 保留页、普通保留页、启动活动对象及 UART MMIO 的最终映射属性（或 LoongArch64 DMW 配置）。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 已完成页表切换或 DMW 窗口初始化的启动环境。
    /// - When:
    ///   - 在 RISC-V 64 下：检查各 RAM 段物理起始地址的恒等映射、首尾页直映、所有保留区边界页及相邻页直映、内核/DTB/initrd 直映以及 UART MMIO 映射；
    ///   - 在 LoongArch64 下：读取 `CRMD`、`DMW1`、`DMW0` 寄存器。
    /// - Then:
    ///   - RISC-V 64 下低地址恒等映射返回 `None`，`no-map` 页直映返回 `None`，其余 RAM 页与 UART 映射具备 `R|W`（`0x6`）属性；
    ///   - LoongArch64 下 `CRMD` 处于分页模式（`PG=1, DA=0`），`DMW1` 与 `DMW0` 分别匹配 Cached 直映窗口与 Uncached 设备窗口。
    #[test]
    fn low_alias_removed(context: &mut TestContext<'_>) {
        #[cfg(target_arch = "riscv64")]
        {
            let info = boot::info();
            for region in &info.memory[..info.memory_count] {
                context.expect_eq(translation(region.start), None);
                expect_ram_mapping(context, info, region.start);
                expect_ram_mapping(context, info, region.end - 1);
            }

            for (region, no_map) in info.reserved[..info.reserved_count]
                .iter()
                .zip(&info.reserved_no_map[..info.reserved_count])
            {
                let start = region.start & !(crate::mm::PAGE_SIZE - 1);
                let end = region
                    .end
                    .checked_add(crate::mm::PAGE_SIZE - 1)
                    .expect("reservation page rounding overflow")
                    & !(crate::mm::PAGE_SIZE - 1);
                for address in [Some(start), Some(end - 1), start.checked_sub(1), Some(end)]
                    .iter()
                    .flatten()
                    .copied()
                {
                    expect_ram_mapping(context, info, address);
                }
                if *no_map {
                    expect_ram_mapping(context, info, start + (end - start) / 2);
                }
            }

            for region in [Some(boot::kernel_region()), Some(info.dtb), info.initrd]
                .iter()
                .flatten()
            {
                for address in [region.start, region.end - 1] {
                    let (physical, _) = translation(arch::phys_to_virt(address))
                        .expect("live boot object direct map missing");
                    context.expect_eq(physical, address);
                }
            }

            let uart = info.interrupts.uart.start;
            let (physical, flags) =
                translation(arch::mmio_address(uart)).expect("UART MMIO map missing");
            context.expect_eq(physical, uart);
            context.expect_eq(flags & 0xe, 0x6);
        }
        #[cfg(target_arch = "loongarch64")]
        {
            let [crmd, cached, device] = mapping_csrs();
            context.expect_eq(crmd & 0x18, 0x10);
            context.expect_eq(cached, arch::phys_to_virt(0) | 0x11);
            context.expect_eq(device, arch::mmio_address(0) | 1);
            context.diagnostic(format_args!(
                "final-csr crmd={:#x} dmw1={:#x} dmw0={:#x}",
                crmd, cached, device
            ));
        }
    }

    /// 测试目标：验证非一致性 DMA 缓存刷新接口 `arch::dma_cache_flush` 能够拒绝未按缓存行对齐的起始地址、算术溢出区间以及不支持硬件缓存维护指令的平台。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 当前运行平台。
    /// - When: 传入未按 64 字节对齐的地址 `1`、会导致末尾地址溢出的 `(usize::MAX & !63, 64)`，以及在非 VisionFive2 平台上传入 `(0, 0)`。
    /// - Then: 非对齐与溢出调用均返回 `Err`；非 VisionFive2 平台因无非一致性缓存控制器同样返回 `Err`。
    #[test]
    fn dma_cache_boundary() {
        assert!(arch::dma_cache_flush(1, 64).is_err());
        assert!(arch::dma_cache_flush(usize::MAX & !63, 64).is_err());
        if boot::info().platform != boot::Platform::VisionFive2 {
            assert!(arch::dma_cache_flush(0, 0).is_err());
        }
    }

    /// 测试目标：验证串口应急输出路径（Emergency Console）在发送 FIFO 持续繁忙（`stalled`）、并发争用（`shared`）与正常就绪（`ready`）三种状态下的轮询预算行为。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 指定模式编号 `mode`（0: `stalled`，1: `shared`，2: `ready`）。
    /// - When: 调用 `crate::console::testing::emergency_budget(mode)` 执行单次应急输出预算测试。
    /// - Then: 返回 `true`，表明各模式下的等待上限与退出条件均符合预期。
    #[test]
    #[case("stalled", 0u8)]
    #[case("shared", 1u8)]
    #[case("ready", 2u8)]
    fn emergency_output_budget(mode: u8) {
        assert!(crate::console::testing::emergency_budget(mode));
    }
}

#[cfg(target_arch = "riscv64")]
#[::ktest::suite(context = "Boot", tags("boot", "mapping"), timeout_ms = 1000)]
pub(crate) mod rv_mapping {
    use super::*;

    /// 测试目标：验证在生产级本地时钟事件（`LocalEvent`）尚未接管前，BSP 能够触发精确一次 S-mode 时钟中断，且 Trap 前后全部通用寄存器（及可选浮点寄存器）保持完整无损。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 运行在 BSP（CPU 0）启动阶段，本地中断处于关闭状态。
    /// - When: 记录当前 CPU 的 `timer_interrupts` 计数，调用汇编测试桩 `arch::ktest_register_pattern(fp, 0)`（内部填入特征值、短暂开中断等待一次时钟中断后再关中断校验寄存器）。
    /// - Then: 汇编桩返回 `true`，`timer_interrupts` 精确增加 1，且返回后本地中断仍保持关闭状态。
    #[test]
    #[case("fp_off", false)]
    #[case("fp_on", true)]
    fn register_exact_irq(context: &mut TestContext<'_>, fp: bool) {
        assert_eq!(cpu::current().id, 0);
        assert!(!arch::irq_enabled());
        let _irq = IrqSaveGuard::new();
        let counter = &cpu::current().timer_interrupts;
        let before = counter.load(Ordering::Acquire);
        // SAFETY:
        // 1. 当前在 CPU 0 上执行，早期启动流程已初始化本核 Trap 入口、`CpuLocal` 及 SBI 时钟源；
        // 2. 从核、调度器与动态内存尚未激活，本地中断已关闭且当前栈帧无活跃浮点状态；
        // 3. `ktest_register_pattern` 汇编桩会在触发一次时钟中断后恢复全部调用者保存寄存器与原始中断掩码。
        assert!(unsafe { arch::ktest_register_pattern(fp, 0) });
        context.expect_eq(counter.load(Ordering::Acquire), before + 1);
        assert!(!arch::irq_enabled());
    }

    /// 测试目标：验证 RISC-V 64 早期引导页表（`RV_BOOT_ROOT`）中恒等映射、MMIO 窗口、64 GiB 物理直映区与内核高地址镜像区的页表项映射关系及未映射边界。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 获取静态引导根页表 `RV_BOOT_ROOT` 的物理地址与链接布局常量 `LINK_LAYOUT`。
    /// - When: 使用 `translation_from` 只读遍历引导页表，依次检查首个 1 GiB 恒等映射与 MMIO 窗口、64 个 1 GiB 直映大页、512 个 2 MiB 内核镜像大页以及区间外的未映射边界地址。
    /// - Then: 内存映射叶表项属性为 `0xcf`（`V|R|W|X|A|D`），MMIO 叶表项属性为 `0xc7`（`V|R|W|A|D`），越界地址均返回 `None`。
    #[test]
    fn bootstrap_mappings(context: &mut TestContext<'_>) {
        extern "C" {
            static RV_BOOT_ROOT: u8;
        }
        const GIB: usize = 1 << 30;
        const IMAGE_LEAF: usize = 1 << 21;
        let layout = crate::platform::LINK_LAYOUT;
        let root = arch::virt_to_phys(core::ptr::addr_of!(RV_BOOT_ROOT) as usize);
        let ram = layout.ram_phys_base as usize;
        for offset in [0, GIB - 1] {
            context.expect_eq(
                translation_from(root, ram + offset),
                Some((ram + offset, 0xcf)),
            );
            context.expect_eq(
                translation_from(root, layout.mmio_virt_base as usize + offset),
                Some((offset, 0xc7)),
            );
        }

        for leaf in 0..64 {
            for within in [0, GIB - 1] {
                let offset = leaf * GIB + within;
                context.expect_eq(
                    translation_from(root, layout.direct_map_virt_base as usize + offset),
                    Some((ram + offset, 0xcf)),
                );
            }
        }

        for leaf in 0..512 {
            for within in [0, IMAGE_LEAF - 1] {
                let offset = leaf * IMAGE_LEAF + within;
                context.expect_eq(
                    translation_from(root, layout.kernel_link_virt as usize + offset),
                    Some((layout.kernel_load_phys as usize + offset, 0xcf)),
                );
            }
        }

        for address in [
            0,
            GIB - 1,
            layout.direct_map_virt_base as usize + 64 * GIB,
            layout.kernel_link_virt as usize + GIB,
        ] {
            context.expect_eq(translation_from(root, address), None);
        }
    }
}

/// 多核（SMP）固定 CPU 子任务测试夹具。
///
/// 为每个逻辑 CPU 维护一个派发任务句柄、完成标志与共享停止信号，确保用例结束或失败时所有子任务均已安全退出内核栈。
struct CpuFixture {
    /// 按逻辑 CPU 编号索引的子任务句柄槽位。
    handles: [Option<ChildHandle>; boot::MAX_CPUS],
    /// 按逻辑 CPU 编号索引的子任务执行成功标志数组（`1` 表示对应核上的测试操作已顺利通过）。
    results: Arc<[AtomicUsize; boot::MAX_CPUS]>,
    /// 通知所有正在自旋等待硬件事件的 CPU 子任务提前退出的协作式停止标志。
    stop: Arc<AtomicBool>,
}

impl Fixture for CpuFixture {
    /// 初始化所有 CPU 的句柄槽位为空，并分配清零的结果数组与停止标志。
    fn set_up(_: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        Ok(Self {
            handles: core::array::from_fn(|_| None),
            results: Arc::try_new(core::array::from_fn(|_| AtomicUsize::new(0)))
                .map_err(|_| TestError("NoMemory: CPU probe"))?,
            stop: Arc::try_new(AtomicBool::new(false))
                .map_err(|_| TestError("NoMemory: CPU stop"))?,
        })
    }

    /// 在用例收尾阶段置位协作式停止标志 `stop`，通知仍在自旋等待的子任务尽快退出。
    fn quiesce(&mut self, _: &mut TestContext<'_>) -> TestResult {
        self.stop.store(true, Ordering::Release);
        Ok(())
    }

    /// 析构夹具前断言所有已派发的子任务均已彻底执行完毕并离开各自任务栈（`try_join().is_some()`）。
    fn tear_down(self, _: &mut TestContext<'_>) -> TestResult {
        assert!(self
            .handles
            .iter()
            .flatten()
            .all(|h| h.try_join().is_some()));
        Ok(())
    }
}

impl CpuFixture {
    /// 在指定的逻辑核 `target` 上派发一个绑定 CPU 的测试子任务，执行 `operation` 并在成功后将 `results[target]` 置为 1。
    fn run(
        &mut self,
        target: usize,
        operation: fn(usize, bool, u8, &AtomicBool) -> TestResult,
        fp: bool,
        fcc: u8,
    ) -> TestResult {
        let result = Arc::clone(&self.results);
        let stop = Arc::clone(&self.stop);
        self.handles[target] = Some(spawn_on(target, move |token| {
            if token.stop_requested() || stop.load(Ordering::Acquire) {
                return Ok(());
            }
            operation(target, fp, fcc, &stop)?;
            result[target].store(1, Ordering::Release);
            Ok(())
        })?);
        Ok(())
    }

    /// 等待逻辑核 `target` 上的子任务彻底结束并离开任务栈，同时校验其执行结果标志是否为 1。
    fn complete(&self, target: usize) -> TestResult {
        wait(
            || self.handles[target].as_ref().unwrap().try_join().is_some(),
            "CPU probe did not leave stack",
        )?;
        if self.results[target].load(Ordering::Acquire) != 1 {
            return Err(TestError("CPU probe failed"));
        }
        Ok(())
    }
}

/// 恢复当前 CPU 生产级本地时钟事件（`LocalEvent`）的 RAII 守卫。
///
/// 当测试函数直接调用 `arch::program_timer` 改写了底层硬件比较寄存器后，由本守卫在函数退出时重新按内核时间队列编程下一次到期时间。
struct TimerRestore;
impl Drop for TimerRestore {
    /// 在关中断保护下调用 `time::reprogram_current()` 恢复当前 CPU 的生产时钟事件。
    fn drop(&mut self) {
        let _irq = IrqSaveGuard::new();
        time::reprogram_current();
    }
}

/// 校验目标逻辑核 `target` 的 `CpuLocal` 身份信息、早期启动栈探针、当前任务栈物理范围以及与其他 CPU 的隔离性。
fn identity(target: usize, _: bool, _: u8, _: &AtomicBool) -> TestResult {
    let _preempt = PreemptGuard::new();
    let local = cpu::current();
    assert_eq!(local.id, target);
    assert!(core::ptr::eq(local, cpu::get(target)));
    assert!(local.online.load(Ordering::Acquire));
    let boot_stack = local.boot_stack_probe.load(Ordering::Acquire);
    let top = local.stack_top();
    assert!(top - cpu::STACK_SIZE <= boot_stack && boot_stack < top);
    assert_eq!(
        local.boot_local_probe.load(Ordering::Acquire),
        local as *const _ as usize
    );
    assert!(local.boot_irq_ready.load(Ordering::Acquire));
    assert_eq!(local.hardware_id, boot::info().hardware_ids[target]);
    if target == 0 {
        assert_eq!(local.hardware_id, boot::info().bsp_hardware_id);
    }

    let stack = 0usize;
    let address = arch::virt_to_phys(&stack as *const usize as usize);
    let (start, bytes) = crate::task::test_current_stack_range();
    assert!(start <= address && address < start + bytes);
    for other in 0..target {
        assert!(!core::ptr::eq(local, cpu::get(other)));
        assert!(cpu::STACK_SIZE <= top.abs_diff(cpu::get(other).stack_top()));
        assert_ne!(local.hardware_id, cpu::get(other).hardware_id);
    }
    Ok(())
}

/// 在目标逻辑核 `target` 上编程一次 1 毫秒后的硬件定时器中断，自旋等待中断计数递增并校验触发时刻不早于设定的 `deadline`。
fn timer(target: usize, _: bool, _: u8, stop: &AtomicBool) -> TestResult {
    let _preempt = PreemptGuard::new();
    let _restore = TimerRestore;
    let local = cpu::current();
    assert_eq!(local.id, target);
    assert!(arch::irq_enabled());
    let before = local.timer_interrupts.load(Ordering::Acquire);
    let deadline = arch::read_counter() + (arch::counter_frequency() / 1000).max(1);
    arch::program_timer(deadline);
    let end = arch::read_counter() + arch::counter_frequency() * 5;
    while local.timer_interrupts.load(Ordering::Acquire) == before {
        if stop.load(Ordering::Acquire) {
            return Ok(());
        }
        if end <= arch::read_counter() {
            return Err(TestError("programmed timer did not interrupt"));
        }
        core::hint::spin_loop();
    }
    assert!(deadline <= arch::read_counter());
    Ok(())
}

/// 在目标逻辑核 `target` 上调用汇编测试桩 `arch::ktest_register_pattern`，验证时钟中断 Trap 前后通用寄存器、栈指针、`CpuLocal` 基址及可选浮点/FCC 状态的完整性。
fn registers(target: usize, fp: bool, fcc: u8, _: &AtomicBool) -> TestResult {
    let _preempt = PreemptGuard::new();
    let _restore = TimerRestore;
    assert_eq!(cpu::current().id, target);
    let before = cpu::current().timer_interrupts.load(Ordering::Acquire);
    // SAFETY:
    // 1. `_preempt` 已禁止内核抢占，保证当前任务固定在逻辑核 `target` 上运行不会发生跨核迁移；
    // 2. 当前处于普通内核线程上下文，未持有自旋锁且无活跃浮点计算状态；
    // 3. `_restore` 守卫保证汇编桩改写的硬件定时器在退出本函数时恢复为生产级 `LocalEvent`。
    assert!(
        unsafe { arch::ktest_register_pattern(fp, fcc) },
        "register pattern fp={} fcc={}",
        fp,
        fcc
    );
    assert!(before < cpu::current().timer_interrupts.load(Ordering::Acquire));
    assert!(arch::irq_enabled());
    Ok(())
}

/// 在目标逻辑核 `target` 上验证 `IrqSaveGuard` 的多层嵌套中断状态保存与逆序恢复，以及 `SpinLock` 释放后中断状态的恢复与可重入加锁。
fn irq_save(target: usize, _: bool, _: u8, _: &AtomicBool) -> TestResult {
    let _preempt = PreemptGuard::new();
    assert_eq!(cpu::current().id, target);
    assert!(arch::irq_enabled());
    let lock = SpinLock::new(0usize);
    {
        let outer = IrqSaveGuard::new();
        assert!(!arch::irq_enabled());
        {
            let _inner = IrqSaveGuard::new();
            assert!(!arch::irq_enabled());
        }
        assert!(!arch::irq_enabled());
        drop(outer);
    }
    assert!(arch::irq_enabled());

    {
        let _initial = IrqSaveGuard::new();
        {
            let _outer = IrqSaveGuard::new();
            {
                let _inner = IrqSaveGuard::new();
            }
            assert!(!arch::irq_enabled());
        }
        assert!(!arch::irq_enabled());
    }
    assert!(arch::irq_enabled());

    {
        let mut guard = lock.lock();
        *guard = 7;
        assert!(!arch::irq_enabled());
    }
    assert!(arch::irq_enabled());
    assert_eq!(
        *lock.try_lock().expect("lock remained held after release"),
        7
    );
    assert!(arch::irq_enabled());
    Ok(())
}

/// 向当前逻辑核 `target` 的硬件 ID 发送一次真实的处理器间中断（Self-IPI），并自旋等待本核 `ipi_interrupts` 计数递增。
fn self_ipi(target: usize, _: bool, _: u8, stop: &AtomicBool) -> TestResult {
    let _preempt = PreemptGuard::new();
    let local = cpu::current();
    assert_eq!(local.id, target);
    let before = local.ipi_interrupts.load(Ordering::Acquire);
    arch::send_ipi(local.hardware_id);
    let end = arch::read_counter() + arch::counter_frequency() * 5;
    while local.ipi_interrupts.load(Ordering::Acquire) == before {
        if stop.load(Ordering::Acquire) {
            return Ok(());
        }
        if end <= arch::read_counter() {
            return Err(TestError("self IPI did not interrupt"));
        }
        core::hint::spin_loop();
    }
    assert!(arch::irq_enabled());
    Ok(())
}

/// 在当前系统的所有在线逻辑 CPU（`0..cpu::count()`）上并发派发 `operation` 探测任务，等待全部核完成并输出各核诊断信息。
fn every_cpu(
    context: &mut TestContext<'_>,
    fixture: &mut CpuFixture,
    operation: fn(usize, bool, u8, &AtomicBool) -> TestResult,
    fp: bool,
    fcc: u8,
) -> TestResult {
    for target in 0..cpu::count() {
        fixture.run(target, operation, fp, fcc)?;
    }

    for target in 0..cpu::count() {
        fixture.complete(target)?;
        context.diagnostic(format_args!(
            "CPU probe cpu={} hw={} stack={:#x} local={:#x} timer={} ipi={}",
            target,
            cpu::get(target).hardware_id,
            cpu::get(target).boot_stack_probe.load(Ordering::Acquire),
            cpu::get(target) as *const _ as usize,
            cpu::get(target).timer_interrupts.load(Ordering::Acquire),
            cpu::get(target).ipi_interrupts.load(Ordering::Acquire)
        ));
    }
    Ok(())
}

#[::ktest::suite(context = "Thread", fixture = CpuFixture, tags("boot", "cpu"), timeout_ms = 30000)]
pub(crate) mod cpus {
    use super::*;

    /// 测试目标：验证所有在线 CPU 的 `CpuLocal` 身份、硬件 ID、启动栈与任务栈范围均正确且互不重叠。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 多核线程测试环境与 `CpuFixture`。
    /// - When: 通过 `every_cpu` 在每个在线 CPU 上运行 `identity` 检验函数。
    /// - Then: 所有 CPU 上的身份、栈范围与隔离性断言均通过。
    #[test]
    fn cpu_identity(context: &mut TestContext<'_>, fixture: &mut CpuFixture) -> TestResult {
        every_cpu(context, fixture, identity, false, 0)
    }

    /// 测试目标：验证每个在线 CPU 的硬件定时器均可独立编程、按期触发中断并在测试后恢复生产时钟事件。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 多核线程测试环境与 `CpuFixture`。
    /// - When: 通过 `every_cpu` 在每个在线 CPU 上运行 `timer` 检验函数。
    /// - Then: 每个 CPU 均在达到设定 `deadline` 后收到时钟中断并正常返回。
    #[test]
    fn programmed_timer(context: &mut TestContext<'_>, fixture: &mut CpuFixture) -> TestResult {
        every_cpu(context, fixture, timer, false, 0)
    }

    /// 测试目标：验证每个在线 CPU 在浮点关闭（`fp_off`）状态下经历真实时钟中断 Trap 时，所有通用寄存器与控制寄存器均能完整保存与恢复。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 多核线程测试环境与 `CpuFixture`，`fp == false`。
    /// - When: 通过 `every_cpu` 在每个在线 CPU 上运行 `registers` 检验函数。
    /// - Then: 所有 CPU 的寄存器图样校验均返回成功且时钟中断计数递增。
    #[test]
    #[case("fp_off", false)]
    fn register_pattern(
        context: &mut TestContext<'_>,
        fixture: &mut CpuFixture,
        fp: bool,
    ) -> TestResult {
        every_cpu(context, fixture, registers, fp, 0)
    }

    /// 测试目标：验证每个在线 CPU 上的嵌套 `IrqSaveGuard` 与 `SpinLock` 在加锁/解锁全过程中能准确保存并恢复本地中断状态。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 多核线程测试环境与 `CpuFixture`。
    /// - When: 通过 `every_cpu` 在每个在线 CPU 上运行 `irq_save` 检验函数。
    /// - Then: 嵌套守卫与自旋锁在最外层释放前保持关中断，释放后立即恢复开中断且锁可再次获取。
    #[test]
    fn irq_save_restore(context: &mut TestContext<'_>, fixture: &mut CpuFixture) -> TestResult {
        every_cpu(context, fixture, irq_save, false, 0)
    }

    /// 测试目标：验证每个在线 CPU 均能向自身发送硬件 IPI 中断并由本核中断处理程序正确接收响应。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 多核线程测试环境与 `CpuFixture`。
    /// - When: 通过 `every_cpu` 在每个在线 CPU 上运行 `self_ipi` 检验函数。
    /// - Then: 每个 CPU 的 `ipi_interrupts` 计数器均成功递增且返回时中断保持开启。
    #[test]
    fn hardware_self_ipi(context: &mut TestContext<'_>, fixture: &mut CpuFixture) -> TestResult {
        every_cpu(context, fixture, self_ipi, false, 0)
    }
}

#[cfg(target_arch = "riscv64")]
#[::ktest::suite(context = "Thread", fixture = CpuFixture, tags("boot", "cpu", "fp"), timeout_ms = 30000)]
pub(crate) mod rv_fp {
    use super::*;
    /// 测试目标：验证 RISC-V 64 架构下每个在线 CPU 在开启浮点单元（`fp == true`）时，经历时钟中断 Trap 后全部 FPR 浮点寄存器与 `fcsr` 均完整保存并恢复。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 多核线程测试环境与 `CpuFixture`。
    /// - When: 通过 `every_cpu` 在每个在线 CPU 上以 `fp = true` 运行 `registers` 检验函数。
    /// - Then: 所有 CPU 的通用寄存器与浮点寄存器图样校验均通过。
    #[test]
    fn fp_enabled(context: &mut TestContext<'_>, fixture: &mut CpuFixture) -> TestResult {
        every_cpu(context, fixture, registers, true, 0)
    }
}
