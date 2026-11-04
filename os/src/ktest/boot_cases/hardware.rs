//! 映射读取与真实固定 CPU 子任务；timer 覆盖后交还生产 LocalEvent。

use crate::ktest::{spawn_on, ChildHandle};
use crate::sync::{IrqSaveGuard, PreemptGuard, SpinLock};
use crate::{arch, boot, cpu, sched, time};
use ::ktest::{Fixture, TestContext, TestError, TestResult};
use alloc::sync::Arc;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

pub(super) mod stacks;

#[cfg(target_arch = "loongarch64")]
pub(crate) mod fcc;

/// 在硬件 counter 的有限区间观察状态；普通 Task 自愿让出真实 CPU。
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

/// 读取实际 SATP 并确认 Sv39 模式，再查询当前最终页表。
#[cfg(target_arch = "riscv64")]
fn translation(address: usize) -> Option<(usize, usize)> {
    let satp: usize;
    // SAFETY: S-mode 读取当前 SATP，不改写翻译状态。
    unsafe {
        core::arch::asm!("csrr {value}, satp", value = out(reg) satp, options(nomem, nostack));
    }
    assert_eq!(satp >> 60, 8, "final SATP is not Sv39");
    translation_from((satp & ((1usize << 44) - 1)) << 12, address)
}

/// 只读遍历指定 Sv39 根，当前表与 bootstrap 表都必须属于永久内核保留 RAM。
#[cfg(target_arch = "riscv64")]
fn translation_from(mut physical: usize, address: usize) -> Option<(usize, usize)> {
    for level in (0..3).rev() {
        let kernel = boot::kernel_region();
        assert!(
            kernel.start <= physical && physical + 4096 <= kernel.end,
            "page table outside kernel reservation"
        );
        let index = (address >> (12 + level * 9)) & 511;
        // SAFETY: 完整页表处于实际内核保留 RAM，直映永久有效；引导后页表不修改。
        let entry = unsafe {
            core::ptr::read_volatile((arch::phys_to_virt(physical) as *const usize).add(index))
        };
        if entry & 1 == 0 {
            return None;
        }
        if entry & 0xe != 0 {
            let mask = (1usize << (12 + level * 9)) - 1;
            return Some((((entry >> 10) << 12) | (address & mask), entry & 0xff));
        }
        physical = (entry >> 10) << 12;
    }
    None
}

/// 只遍历最终页表核对 RAM 页属性，不解引用被 no-map 禁止访问的物理页。
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
                && region.start / 4096 <= address / 4096
                && address / 4096 < region.end.div_ceil(4096)
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
/// PLV0 读取实际控制与 DMW CSR，不依赖 GDB 的 CSR 名字支持。
fn mapping_csrs() -> [usize; 3] {
    let crmd;
    let cached;
    let device;
    // SAFETY: 三个已存在 CSR 只读且不改写 DMW 或地址模式。
    unsafe {
        core::arch::asm!("csrrd {crmd}, 0", "csrrd {cached}, 0x181", "csrrd {device}, 0x180",
        crmd = out(reg) crmd, cached = out(reg) cached, device = out(reg) device, options(nomem, nostack));
    }
    [crmd, cached, device]
}

#[::ktest::suite(context = "Boot", tags("boot", "mapping"), timeout_ms = 1000)]
pub(crate) mod mapping {
    use super::*;

    /// 当前执行代码和局部栈均来自最终高地址映射，RAM直映保持不同地址角色。
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

    /// 清零只改写半开区间；空区间和两端的有效数据必须保持。
    #[test]
    fn boot_clear_interval() {
        const SENTINEL: usize = 0x9d37_51ab_c462_e806;
        let mut words = [SENTINEL; 10];
        let start = core::ptr::addr_of_mut!(words).cast::<usize>() as usize;
        let word_size = core::mem::size_of::<usize>();
        // SAFETY: 空区间端点指向本例独占且对齐的数组，不改写任何对象。
        unsafe { arch::boot_clear_words(start, start) };
        assert!(words.iter().all(|&value| value == SENTINEL));
        // SAFETY: 内部八个机器字完全落在本例独占数组中，不覆盖两端数据或调用保存区。
        unsafe { arch::boot_clear_words(start + word_size, start + 9 * word_size) };
        assert_eq!(words[0], SENTINEL);
        assert_eq!(words[9], SENTINEL);
        assert!(words[1..9].iter().all(|&value| value == 0));
    }

    /// 低 RAM 别名已撤销；no-map 页与相邻普通保留页的最终映射按真实属性核对。
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
                let start = region.start & !4095;
                let end = region
                    .end
                    .checked_add(4095)
                    .expect("reservation page rounding overflow")
                    & !4095;
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
                .into_iter()
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

    /// 非一致维护必须拒绝未知机制与不完整 cacheline，不能把内存屏障记为 cache flush。
    #[test]
    fn dma_cache_boundary() {
        assert!(arch::dma_cache_flush(1, 64).is_err());
        assert!(arch::dma_cache_flush(usize::MAX & !63, 64).is_err());
        if boot::info().platform != boot::Platform::VisionFive2 {
            assert!(arch::dma_cache_flush(0, 0).is_err());
        }
    }

    /// 单个真实应急 writer 的预算场景，不把可控LSR输入冒称硬件输入。
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

    /// 生产 LocalEvent 尚未激活时保留原精确一次 timer IRQ 与完整寄存器图样断言。
    #[test]
    #[case("fp_off", false)]
    #[case("fp_on", true)]
    fn register_exact_irq(context: &mut TestContext<'_>, fp: bool) {
        assert_eq!(cpu::current().id, 0);
        assert!(!arch::irq_enabled());
        let _irq = IrqSaveGuard::new();
        let counter = &cpu::current().timer_interrupts;
        let before = counter.load(Ordering::Acquire);
        // SAFETY: 既有 Boot 路径已安装本核 trap/timer 源和永久 CpuLocal；AP/MM/调度
        // 尚未启动，IRQ 关闭且无活跃 FP，原汇编暂开 IRQ 后恢复同核现场。
        assert!(unsafe { arch::ktest_register_pattern(fp, 0) });
        context.expect_eq(counter.load(Ordering::Acquire), before + 1);
        assert!(!arch::irq_enabled());
    }

    /// 从 BSP 实际保留的 bootstrap 根检查全部直映和镜像叶，不读取叶所映射的数据。
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

struct CpuFixture {
    handles: [Option<ChildHandle>; boot::MAX_CPUS],
    results: Arc<[AtomicUsize; boot::MAX_CPUS]>,
    stop: Arc<AtomicBool>,
}
impl Fixture for CpuFixture {
    /// 每例拥有独立结果与停止门；真实子任务的离栈责任登记到当前框架 scope。
    fn set_up(_: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        Ok(Self {
            handles: core::array::from_fn(|_| None),
            results: Arc::try_new(core::array::from_fn(|_| AtomicUsize::new(0)))
                .map_err(|_| TestError("NoMemory: CPU probe"))?,
            stop: Arc::try_new(AtomicBool::new(false))
                .map_err(|_| TestError("NoMemory: CPU stop"))?,
        })
    }
    /// 主体失败也合作关闭每个具体探测；join 在框架中确认真实 Task 已离栈。
    fn quiesce(&mut self, _: &mut TestContext<'_>) -> TestResult {
        self.stop.store(true, Ordering::Release);
        Ok(())
    }
    /// 只有真实 join 之后才释放结果 owner 和观察 handle。
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
    /// 固定核子任务拥有结果对象；无父 fixture 借用逃逸到另一条栈。
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
    /// 等待观测到全部实际完成，不把框架 child 失败当作成功结果。
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

struct TimerRestore;
impl Drop for TimerRestore {
    /// 同核IRQ关闭段按真实 LocalEvent 重装下一deadline，再恢复进入IRQ态。
    fn drop(&mut self) {
        let _irq = IrqSaveGuard::new();
        time::reprogram_current();
    }
}

/// 在固定 Task 上读取真实基址；启动栈来自进入调度前的永久观测。
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

/// 真实 programmed timer 不能提前；完成或失败后都交还生产事件。
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

/// 原始 trap 汇编检查 GPR/SP/CPU基址与指定 FP/FCSR/FCC 图样。
fn registers(target: usize, fp: bool, fcc: u8, _: &AtomicBool) -> TestResult {
    let _preempt = PreemptGuard::new();
    let _restore = TimerRestore;
    assert_eq!(cpu::current().id, target);
    let before = cpu::current().timer_interrupts.load(Ordering::Acquire);
    // SAFETY: 固定核普通 Task、无活跃 FP 上下文、不持 IRQ 锁；真实 timer/trap 已初始化，
    // PreemptGuard 排除迁移，汇编继承生产 trap 与永久本核计数器。
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

/// 保存初始开关、嵌套逆序恢复，并证明真实锁 release 后同核可以再取得。
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

/// 向当前硬件 CPU 发送真正 IPI，而非普通函数直接调用 handler。
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

/// 每个图样都遍历本次真实 CPU 表，不对缺核填入结果。
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
    /// 全部请求核数的基址、真实启动栈、Task栈与上线顺序。
    #[test]
    fn cpu_identity(context: &mut TestContext<'_>, fixture: &mut CpuFixture) -> TestResult {
        every_cpu(context, fixture, identity, false, 0)
    }
    /// 逐核真实 timer 的 deadline 和实际返回。
    #[test]
    fn programmed_timer(context: &mut TestContext<'_>, fixture: &mut CpuFixture) -> TestResult {
        every_cpu(context, fixture, timer, false, 0)
    }
    /// 浮点关闭现场独立验证，开启图样按目标架构分别注册。
    #[test]
    #[case("fp_off", false)]
    fn register_pattern(
        context: &mut TestContext<'_>,
        fixture: &mut CpuFixture,
        fp: bool,
    ) -> TestResult {
        every_cpu(context, fixture, registers, fp, 0)
    }
    /// 每核真实嵌套IRQ-save与锁释放后恢复。
    #[test]
    fn irq_save_restore(context: &mut TestContext<'_>, fixture: &mut CpuFixture) -> TestResult {
        every_cpu(context, fixture, irq_save, false, 0)
    }
    /// 每核实际硬件 IPI 返回。
    #[test]
    fn hardware_self_ipi(context: &mut TestContext<'_>, fixture: &mut CpuFixture) -> TestResult {
        every_cpu(context, fixture, self_ipi, false, 0)
    }
}

#[cfg(target_arch = "riscv64")]
#[::ktest::suite(context = "Thread", fixture = CpuFixture, tags("boot", "cpu", "fp"), timeout_ms = 30000)]
pub(crate) mod rv_fp {
    use super::*;
    /// RV代表FPR/FCSR图样在每个实际CPU上经过真实timer返回。
    #[test]
    fn fp_enabled(context: &mut TestContext<'_>, fixture: &mut CpuFixture) -> TestResult {
        every_cpu(context, fixture, registers, true, 0)
    }
}
