//! QEMU 9.2.1 virt 的 DMW、异常、一次定时器及 PCH-PIC/EXTIOI 机制。
use crate::boot::BootInfo;
use crate::platform::LINK_LAYOUT;
use crate::sched::context::TaskContext;
use crate::sync::SpinLock;
use crate::syscall::Errno;
use core::arch::asm;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

pub const NAME: &str = "loongarch64";

// PCH-PIC 的共享 word、源路由与极性寄存器。
const PCH_PIC_MASK: usize = 0x20;
const PCH_PIC_HTMSI_ENABLE: usize = 0x40;
const PCH_PIC_EDGE: usize = 0x60;
const PCH_PIC_CLEAR: usize = 0x80;
const PCH_PIC_ROUTE: usize = 0x100;
const PCH_PIC_VECTOR: usize = 0x200;
const PCH_PIC_POLARITY: usize = 0x3e0;
const IO_WORD_BYTES: usize = core::mem::size_of::<u32>();
const NODEMAP_ENTRIES_PER_WORD: usize = IO_WORD_BYTES / core::mem::size_of::<u16>();
const PCH_PIC_SOURCE_COUNT: u32 = 2 * u32::BITS;
const PCH_PIC_ROUTE_HT: u8 = 1;

// EXTIOI 偏移相对于固件给出的 parent_controller，不能当作绝对 IOCSR 编号。
const EXTIOI_NODEMAP: usize = 0xa0;
const EXTIOI_IPMAP: usize = 0xc0;
const EXTIOI_ENABLE: usize = 0x200;
const EXTIOI_BOUNCE: usize = 0x280;
const EXTIOI_ISR: usize = 0x400;
const EXTIOI_ROUTE: usize = 0x800;
const EXTIOI_CORES_PER_NODE: usize = 4;
const EXTIOI_ROUTE_NODE_SHIFT: usize = 4;
const EXTIOI_NODE_COUNT: usize = 1 << EXTIOI_ROUTE_NODE_SHIFT;
const EXTIOI_ROOT_INTERRUPT: u32 = reg::CPU_HWI1;
const EXTIOI_IPMAP_HWI1: u32 = 2;
const IOCSR_MISC_FUNCTION: usize = 0x420;
const IOCSR_EXTIOI_ENABLE: u64 = 1 << 48;

const GED_BASE: usize = 0x100e001c;
const GED_SLEEP_CONTROL: usize = 0;
#[cfg(not(feature = "ktest"))]
const GED_RESET: usize = 2;
const GED_SLEEP_ENABLE: u8 = 1 << 5;
const GED_SLEEP_S5: u8 = 5 << 2;
#[cfg(not(feature = "ktest"))]
const GED_RESET_VALUE: u8 = 0x42;

pub mod ale;
mod counter;
mod entry;
mod idle;
mod liointc;
mod mmu;
mod register;
#[cfg(feature = "ktest")]
mod register_test;
mod rtc;
use register as reg;
#[cfg(feature = "ktest")]
pub(crate) use register::{CRMD_IE, CSR_CRMD, CSR_DMW0, CSR_DMW1, CSR_EUEN, CSR_KS0};
pub(crate) use register::{
    ECODE_ADE, ECODE_ALE, ECODE_BCE, ECODE_BRK, ECODE_FPE, ECODE_PPI, EUEN_FP, EUEN_LASX, EUEN_LSX,
    FCSR_CAUSE_SHIFT, FCSR_ENABLE_MASK,
};
mod signal;
mod switch;
mod trap;
mod uaccess;
mod user;
use crate::arch::loongarch64::trap::{la_kernel_refill, la_kernel_trap};
pub use counter::{apply_counter_epoch, establish_counter_epoch};
pub use rtc::board_rtc_calendar;
use switch::la_switch_context;
#[cfg(feature = "ktest")]
use {register_test::la_test_register_preservation, switch::la_test_task_context_preservation};

pub use mmu::{
    activate_kernel_root, activate_user_root, flush_user_translations, initialize_user_root,
    sync_user_icache, user_empty_entry, user_entry_is_table, user_entry_physical, user_leaf_dirty,
    user_leaf_entry, user_leaf_mkclean, user_leaf_permissions, user_table_entry,
    SIGNAL_RETURN_PAGE, USER_ADDRESS_END, USER_LEVELS, USER_STACK_TOP,
};
pub(crate) use signal::SIGNAL_MIN_STACK;
pub use uaccess::{copy_user_bytes, load_user_u32, store_user_u32};
pub use user::{run_user, signal_return_code, UserContext};

#[cfg(feature = "ktest")]
pub(crate) use user::{
    exit_offset, fault_offset, loop_offset, loop_spin_offset, user_code, FpRegister,
};

#[repr(C, align(16))]
struct KernelTrapFrame {
    gpr: [usize; 32],
    era: usize,
    prmd: usize,
    estat: usize,
    badv: usize,
    euen: usize,
    cpu_base: usize,
    // EUEN.FPE 关闭时汇编跳过这些字段，Rust 不得把它们视为已初始化值。
    fpr: MaybeUninit<[u64; 32]>,
    fcsr: MaybeUninit<usize>,
    fcc: MaybeUninit<[usize; 8]>,
}

// refill 有独立 CSR 现场，不借用普通异常的 ERA/PRMD/BADV。
#[repr(C, align(16))]
struct RefillFrame {
    gpr: [usize; 32],
    era: usize,
    prmd: usize,
    badv: usize,
}

extern "C" {
    static __secondary_entry_phys: u8;
    /// 在启 IE 到 IDLE 的显式窗口内复查提示；窗口中断后退出等待并返回调度器。
    fn la_idle_once_asm(
        kick_pending: *const AtomicUsize,
        wake_pending: *const AtomicBool,
        #[cfg(feature = "ktest")] wait_completions: *const AtomicUsize,
    );
    static la_idle_window_start: u8;
    static la_idle_window_end: u8;
    static la_idle_window_exit: u8;
}

/// 在引导入口确立的 PLV0 环境读取本核 CSR；寄存器编号由本模块常量指定。
#[inline]
fn csr_read<const R: u16>() -> usize {
    let value;
    // SAFETY: 两种 Rust 引导入口均处于 PLV0；调用点只选择本架构可读的 CSR。
    unsafe {
        asm!("csrrd {value}, {reg}", value = out(reg) value, reg = const R, options(nomem, nostack));
    }
    value
}
/// 写入本核 CSR，调用点负责保持寄存器对应的异常、映射和中断协议。
///
/// # Safety
/// 必须处于 PLV0，R 必须是可写 CSR，写入值不得破坏当前代码、栈和 per-CPU 对象的可达性。
#[inline]
unsafe fn csr_write<const R: u16>(value: usize) {
    // SAFETY: 调用者保证 CSR 可写、特权级正确，并保持当前执行现场所依赖的映射。
    unsafe {
        asm!("csrwr {value}, {reg}", value = inout(reg) value => _, reg = const R, options(nostack));
    }
}
/// 读取本模块选定的 QEMU IOCSR 状态，不通过普通内存指针访问设备。
#[inline]
fn iocsr_read32(reg: usize) -> u32 {
    let value: usize;
    // SAFETY: 调用点只传递 QEMU virt 的有效 IOCSR 编号，入口保证当前执行级为 PLV0。
    unsafe {
        asm!("iocsrrd.w {value}, {reg}", value = out(reg) value, reg = in(reg) reg, options(nostack));
    }
    value as u32
}
/// 读取 QEMU virt 的 64 位 IOCSR 状态。
#[inline]
fn iocsr_read64(reg: usize) -> u64 {
    let value;
    // SAFETY: 调用点只选择有效的 64 位 IOCSR，PLV0 已由引导入口建立。
    unsafe {
        asm!("iocsrrd.d {value}, {reg}", value = out(reg) value, reg = in(reg) reg, options(nostack));
    }
    value
}
/// 写入选定的 32 位 IOCSR；共享路由读改写由独立控制器锁串行化。
#[inline]
fn iocsr_write32(reg: usize, value: u32) {
    // SAFETY: 本模块调用点使用已验证平台的有效 IOCSR；设备状态不承载 Rust 引用。
    unsafe {
        asm!("iocsrwr.w {value}, {reg}", value = in(reg) value as usize, reg = in(reg) reg, options(nostack));
    }
}
/// 写入 QEMU mailbox 或 EXTIOI 的 64 位 IOCSR。
#[inline]
fn iocsr_write64(reg: usize, value: u64) {
    // SAFETY: 调用点选择 QEMU virt 的有效 IOCSR，mailbox 指针的生命周期由启动接口约定。
    unsafe {
        asm!("iocsrwr.d {value}, {reg}", value = in(reg) value, reg = in(reg) reg, options(nostack));
    }
}
/// 排序本核设备访问和已发表的共享启动记录。
fn barrier() {
    // SAFETY: dbar 在引导入口建立的 PLV0 可执行，不解引用任何指针。
    unsafe {
        asm!("dbar 0", options(nostack));
    }
}

/// 以完成型写屏障排序 coherent DMA 内存写入。
pub(super) fn dma_write_barrier() {
    // SAFETY: 引导入口已建立 PLV0，dbar 0x0a 排序前后写访问；
    // 汇编保留 memory 副作用，不声明 nomem/readonly。
    unsafe { asm!("dbar {hint}", hint = const reg::DBAR_COMPLETION_WRITE, options(nostack)) };
}

/// 在观察设备完成后，以完成型读屏障排序 coherent 数据读取。
pub(super) fn dma_read_barrier() {
    // SAFETY: PLV0 可执行 dbar 0x05，完成前读并排序后读，不访问额外地址；
    // 汇编的 memory 副作用同时约束编译器读取顺序。
    unsafe { asm!("dbar {hint}", hint = const reg::DBAR_COMPLETION_READ, options(nostack)) };
}

/// 对 coherent 内存和设备 MMIO 执行完成型双向排序。
pub(super) fn dma_barrier() {
    // SAFETY: PLV0 可执行 dbar 0x00，覆盖全部前后访问；
    // 不声明 nomem/readonly，禁止编译器将内存访问移过此完整屏障。
    unsafe { asm!("dbar 0x00", options(nostack)) };
}

/// 当前平台仅公开 coherent DMA；缺少已核非一致维护机制时拒绝假成功。
pub fn dma_cache_flush(_: usize, _: usize) -> Result<(), &'static str> {
    Err("LoongArch noncoherent DMA cache maintenance is unsupported")
}

/// 从 CPUId CSR 取得当前硬件身份，不解释 QEMU 的引导参数。
pub fn boot_hardware_id(_: [usize; 4]) -> usize {
    csr_read::<{ reg::CSR_CPUID }>() & reg::HARDWARE_CPU_ID_MASK
}
/// 将真实 RAM 物理地址转换为缓存 DMW 别名。
pub fn phys_to_virt(pa: usize) -> usize {
    pa | LINK_LAYOUT.direct_map_virt_base as usize
}
/// 从缓存 DMW 内的 RAM 地址取得物理地址；其他地址类别不适用此换算。
pub fn virt_to_phys(va: usize) -> usize {
    va & !(LINK_LAYOUT.direct_map_virt_base as usize)
}
/// 将设备物理地址转换为强序 DMW 地址；返回整数地址，不验证设备资源范围。
pub fn mmio_address(pa: usize) -> usize {
    pa | LINK_LAYOUT.mmio_virt_base as usize
}
/// 确认最终 PLV0 DMW 并建立永久 invalid 页表链；普通内核不再使用 DA 低地址别名。
///
/// # Safety
/// 必须由 BSP 在 IRQ 关闭、辅核尚未释放时调用一次；入口已建立 PG、缓存与强序 DMW。
/// BootInfo 必须是已发布的真实平台对象，调用期间不得修改这些映射。
pub unsafe fn finish_boot_mappings(_: &BootInfo) {
    assert_eq!(
        csr_read::<{ reg::CSR_CRMD }>() & reg::CRMD_ADDRESS_MODE,
        reg::CRMD_PG
    );
    assert_eq!(
        csr_read::<{ reg::CSR_DMW1 }>(),
        phys_to_virt(0) | reg::DMW_KERNEL_RAM
    );
    assert_eq!(
        csr_read::<{ reg::CSR_DMW0 }>(),
        mmio_address(0) | reg::DMW_PLV0
    );
    // SAFETY: 本函数的 BSP 独占启动契约保证 AP 尚未发布，静态 DMW 映射已通过上面的断言。
    unsafe { mmu::initialize_invalid_tables() };
    crate::console::configure_resources(crate::boot::info());
}
/// 将稳定的内核 per-CPU 指针放入 KS0；用户 TLS 寄存器不参与此约定。
///
/// # Safety
/// local 必须非空、满足 CpuLocal 对齐、已完整初始化且属于本核；对象和 DMW 映射必须永久有效。
/// 必须在本核 IRQ 关闭时安装，不得把同一个对象安装给另一个 CPU。
pub unsafe fn install_cpu_local(local: *const crate::cpu::CpuLocal) {
    // SAFETY: 调用者保证本核 IRQ 已关闭，指针是本核永久有效对象，KS0 只保存该基址。
    unsafe { csr_write::<{ reg::CSR_KS0 }>(local as usize) };
}
/// 读取当前 CPU 的 KS0 基址。
pub fn cpu_local() -> *const crate::cpu::CpuLocal {
    csr_read::<{ reg::CSR_KS0 }>() as *const _
}

/// 在调度专用 IRQ 关闭段切换整数现场；恢复后 KS0 仍属于实际执行的 CPU。
///
/// # Safety
/// 调用者必须独占 prev、next 的上下文访问，两者非空、对齐且互不重叠。
/// prev 可写，next 已初始化为合法函数返回现场，其独占栈满足 16 字节 ABI 对齐；
/// 两份现场及其栈必须保留到各自恢复。当前 IRQ 必须关闭、EUEN 必须为零，
/// 不得持有普通锁、IRQ/preempt guard 或跨切换保存本核借用。
pub unsafe fn switch_context(prev: *mut TaskContext, next: *const TaskContext) {
    assert!(!irq_enabled(), "context switch requires IRQ off");
    assert_eq!(
        csr_read::<{ reg::CSR_EUEN }>(),
        0,
        "context switch requires EUEN off"
    );
    // SAFETY: 调用者独占两个有效现场及栈，断言确认 IRQ 与扩展单元关闭；
    // 汇编仅按 Rust 提供的布局访问整数字段，不恢复 CPU 基址或扩展单元现场。
    unsafe { la_switch_context(prev, next) };
}
/// 安装本核异常与用户 walker，按实际平台开放 timer、IPI 和外部父 cause。
pub fn init_current_cpu() {
    irq_save();
    // SAFETY: 入口已建立本核 PLV0/DMW 和有效栈；IRQ 已关闭，异常代码在永久内核映射内。
    unsafe {
        csr_write::<{ reg::CSR_EENTRY }>(la_kernel_trap as usize);
        csr_write::<{ reg::CSR_TLBRENTRY }>(virt_to_phys(la_kernel_refill as usize));
        let external = if crate::platform::current() == crate::boot::Platform::Ls2k1000la {
            reg::IRQ_HWI0 | reg::IRQ_HWI1
        } else {
            reg::IRQ_HWI1
        };
        csr_write::<{ reg::CSR_ECFG }>(reg::IRQ_TIMER | reg::IRQ_IPI | external);
        csr_write::<{ reg::CSR_EUEN }>(0);
    }
    mmu::initialize_current_cpu();
    disable_timer();
    iocsr_write32(reg::IOCSR_IPI_CLEAR, u32::MAX);
    iocsr_write32(reg::IOCSR_IPI_ENABLE, u32::MAX);
    barrier();
}
/// 在 PLV0 原子关闭本核 IRQ 并返回原 CRMD；编译器内存屏障保持临界区访问位于关 IRQ 之后。
pub fn irq_save() -> usize {
    let old;
    // SAFETY: 入口保证 PLV0；固定输入 $t0/r12 仅承载 IE 掩码 4，避开 csrxchg 禁用的 r0/r1。
    // csrxchg 不改写 $t0，inout 声明旧 CRMD 的回写并排除与 $t0 重叠；指令不访问栈。
    // 仅清除 IE；不使用 nomem/readonly，保留普通访存不能跨越 IRQ 边界的编译器屏障。
    unsafe {
        asm!("csrxchg {old}, $t0, {crmd}", old = inout(reg) 0usize => old, in("$t0") reg::CRMD_IE, crmd = const reg::CSR_CRMD, options(nostack));
    }
    old
}
/// 在 PLV0 只恢复原来的 IE 位；编译器内存屏障保持临界区访问位于恢复 IRQ 之前。
///
/// # Safety
/// saved 必须来自本核匹配的 irq_save，期间不能迁移 CPU；嵌套保存必须逆序恢复，
/// 恢复 IE 前必须释放依赖 IRQ 关闭的锁和借用。
pub unsafe fn irq_restore(saved: usize) {
    // SAFETY: 调用者保证同核恢复且临界区已结束；入口保证 PLV0，固定 $t0/r12 为合法掩码寄存器。
    // 只恢复掩码 4 指定的 IE；$t0 保持不变，inout 声明旧 CRMD 回写并排除与 $t0 重叠。
    // 指令不访问栈；不使用 nomem/readonly，保留锁释放及借用结束不能越过恢复 IRQ 的编译器屏障。
    unsafe {
        asm!("csrxchg {value}, $t0, {crmd}", value = inout(reg) (saved & reg::CRMD_IE) => _, in("$t0") reg::CRMD_IE, crmd = const reg::CSR_CRMD, options(nostack));
    }
}
/// 查询本核全局 IRQ 开关。
pub fn irq_enabled() -> bool {
    csr_read::<{ reg::CSR_CRMD }>() & reg::CRMD_IE != 0
}
/// 在 PLV0 打开本核全局 IRQ；编译器内存屏障保持初始化写入和 online 发布位于各自边界。
pub fn enable_interrupts() {
    // SAFETY: 入口保证 PLV0，本核异常及 IRQ 资源已安装；固定 $t0/r12 避开禁用的 r0/r1。
    // 掩码 4 只使能 IE；$t0 保持不变，inout 声明旧 CRMD 回写并排除与 $t0 重叠。
    // 指令不访问栈；不使用 nomem/readonly，保留普通访存不能跨越开 IRQ 边界的编译器屏障。
    unsafe {
        asm!("csrxchg {value}, $t0, {crmd}", value = inout(reg) reg::CRMD_IE => _, in("$t0") reg::CRMD_IE, crmd = const reg::CSR_CRMD, options(nostack));
    }
}
/// 等待真实硬件中断。
pub fn wait_for_interrupt() {
    // SAFETY: 当前在 PLV0，idle 不解引用指针，也不改变已建立的映射。
    unsafe {
        asm!("idle 0", options(nostack));
    }
}

/// 启 IE 后重查提示，窗口中断直接退出等待，由调度器重新检查队列；返回保持 IRQ 关闭。
///
/// # Safety
/// 仅允许本核永久 boot/scheduler 栈上的最终等待边界调用：IRQ 关闭、无普通任务、
/// 锁或 IRQ/BH/preempt guard，允许临时服务 IRQ，栈不得迁移或回收。
/// kick_pending 与测试计数是本核永久字段；wake_pending 在本次调用期间地址稳定，
/// 所有原子只能原子访问，唤醒方按所属协议发表条件并通知本核；trap 与独立 IRQ 源已使能。
pub(crate) unsafe fn idle_once(
    kick_pending: &AtomicUsize,
    wake_pending: &AtomicBool,
    #[cfg(feature = "ktest")] wait_completions: &AtomicUsize,
) {
    assert!(!irq_enabled(), "idle entry requires IRQ off");
    // SAFETY: 调用者保证本核永久 boot/scheduler 栈、稳定原子及无锁 IRQ-off 边界；汇编遵守整数 ABI，
    // 窗口内中断转到关闭 IE 的返回边界，调度器随后重查完整队列；不改 FP/KS0。
    // 测试计数指针经 a2 传入；amadd_db.d 的输出 t1、增量 t0、地址 a2 互不重叠，
    // 64 位原子 RMW 的完整屏障与远端 Acquire 读取匹配，trap 保留全部参数和临时寄存器。
    unsafe {
        #[cfg(feature = "ktest")]
        la_idle_once_asm(kick_pending, wake_pending, wait_completions);
        #[cfg(not(feature = "ktest"))]
        la_idle_once_asm(kick_pending, wake_pending);
    };
    assert!(!irq_enabled(), "idle exit has IRQ enabled");
}
/// 读取稳定硬件计数器。
pub fn read_counter() -> u64 {
    let value;
    // SAFETY: rdtime 仅读取硬件计数器，入口已建立可执行该指令的环境。
    unsafe {
        asm!("rdtime.d {value}, $zero", value = out(reg) value, options(nomem, nostack));
    }
    value
}
/// 读取只读 CPUCFG 参数，供计数器频率与用户扩展能力选择使用。
pub(crate) fn cpucfg(index: usize) -> u64 {
    let value: usize;
    // SAFETY: CPUCFG 是架构只读配置指令，不访问普通内存或改变特权状态。
    unsafe {
        asm!("cpucfg {value}, {index}", value = out(reg) value, index = in(reg) index, options(nomem, nostack));
    }
    value as u64 & u64::from(u32::MAX)
}

/// 返回用户现场完整支持的 EUEN 位；扩展依赖必须同时存在于实际 CPU。
pub(crate) fn user_euen() -> usize {
    let config = cpucfg(reg::CPUCFG_FEATURES);
    if config & reg::CPUCFG_FP == 0 {
        return 0;
    }
    if config & reg::CPUCFG_LSX == 0 {
        return reg::EUEN_SCALAR;
    }
    if config & reg::CPUCFG_LASX == 0 {
        return reg::EUEN_SIMD;
    }
    reg::EUEN_WIDE_SIMD
}

/// 给出实际启用的每寄存器 u64 lane 数，零表示 CPU 没有浮点扩展。
pub(crate) fn user_fp_lanes() -> usize {
    match user_euen() {
        reg::EUEN_WIDE_SIMD => 4,
        reg::EUEN_SIMD => 2,
        reg::EUEN_SCALAR => 1,
        _ => 0,
    }
}
/// 仅资源发现前从 CPUCFG4/5 取得实际 counter 频率，不作为运行期读数路径。
pub fn boot_counter_frequency() -> u64 {
    let base = cpucfg(reg::CPUCFG_COUNTER_BASE);
    let ratio = cpucfg(reg::CPUCFG_COUNTER_RATIO);
    let mul = ratio & reg::CPUCFG_COUNTER_MUL_MASK;
    let div = ratio >> reg::CPUCFG_COUNTER_DIV_SHIFT;
    assert!(
        base != 0 && mul != 0 && div != 0,
        "invalid CPUCFG counter frequency"
    );
    base * mul / div
}
/// 返回一次启动资源发现发布的 counter 频率。
#[cfg(feature = "ktest")]
pub fn counter_frequency() -> u64 {
    crate::boot::info().counter_frequency
}
/// 将绝对 deadline 向上量化到四个 tick，并编程一次事件。
pub fn program_timer(deadline: u64) {
    disable_timer();
    let delta = deadline
        .saturating_sub(read_counter())
        .max(reg::TIMER_QUANTUM);
    let value = delta
        .saturating_add(reg::TIMER_QUANTUM - 1)
        .min(reg::TIMER_MAX_DELTA)
        & !(reg::TIMER_QUANTUM - 1);
    // SAFETY: 在 PLV0 写本核 TCFG；上面的量化保证只写有效 tick 位与一次事件使能位。
    unsafe {
        csr_write::<{ reg::CSR_TCFG }>(value as usize | reg::TCFG_ENABLE);
    }
}
/// 关闭一次事件并清除真实 timer pending。
pub fn disable_timer() {
    // SAFETY: TCFG/TICLR 是本核定时器 CSR，写零关闭事件、写一清 pending 均不影响映射。
    unsafe {
        csr_write::<{ reg::CSR_TCFG }>(0);
        csr_write::<{ reg::CSR_TICLR }>(reg::TICLR_CLEAR);
    }
}
/// 编码 IOCSR 的十位真实 CPU target；编译逻辑 CPU 容量不参与硬件寻址。
pub(crate) const fn ipi_target(hardware_id: usize) -> Option<u32> {
    if hardware_id < reg::HARDWARE_CPU_ID_COUNT {
        Some(reg::IPI_SEND_BLOCKING | ((hardware_id as u32) << reg::IPI_SEND_CPU_SHIFT))
    } else {
        None
    }
}

/// 用已编码的 target 向四个硬件 mailbox 之一发表完整 64 位值。
fn mail_send(target: u32, mailbox: usize, data: usize) {
    assert!(
        mailbox < reg::MAILBOX_COUNT,
        "IOCSR mailbox index is out of range"
    );
    let target = u64::from(target);
    iocsr_write64(
        reg::IOCSR_MAIL_SEND,
        (data as u64 & reg::MAIL_SEND_HIGH_MASK)
            | target
            | (((mailbox * 2 + 1) as u64) << reg::MAIL_SEND_BOX_SHIFT),
    );
    iocsr_write64(
        reg::IOCSR_MAIL_SEND,
        ((data as u64) << reg::MAIL_SEND_DATA_SHIFT)
            | target
            | (((mailbox * 2) as u64) << reg::MAIL_SEND_BOX_SHIFT),
    );
}
/// 板 mailbox1 发表真实栈、mailbox2 发表 CpuLocal，最后发表入口并发送启动 IPI。
///
/// # Safety
/// 必须由 BSP 串行启动尚未运行的已枚举辅核；物理地址须指向对齐、完整初始化且永久保留的 CpuBoot。
/// 记录中的栈须独占、满足 16 字节 ABI 对齐，CpuLocal 须属于目标核且完整初始化；两者与 DMW 均须永久有效。
/// 发布后不得再修改启动记录。
pub unsafe fn start_cpu(hardware_id: usize, boot_record_physical: usize) -> Result<(), isize> {
    let target = ipi_target(hardware_id).ok_or(-(Errno::InvalidArgument as isize))?;
    barrier();
    if crate::platform::current() == crate::boot::Platform::Ls2k1000la {
        // SAFETY: 调用者保证物理记录完整、对齐且永久有效；缓存 DMW 映射覆盖该内核静态对象。
        let record =
            unsafe { &*(phys_to_virt(boot_record_physical) as *const crate::cpu::CpuBoot) };
        mail_send(target, 1, record.stack_top);
        mail_send(target, 2, record.local);
    } else {
        mail_send(target, 1, boot_record_physical);
    }
    let entry: usize;
    // SAFETY: 链接脚本给出可执行的物理辅核入口；这里只取符号值，记录契约由调用者保证。
    unsafe {
        asm!("la.abs {entry}, {symbol}", entry = out(reg) entry, symbol = sym __secondary_entry_phys, options(nomem, nostack));
    }
    mail_send(target, 0, entry);
    barrier();
    iocsr_write32(reg::IOCSR_IPI_SEND, target);
    Ok(())
}
/// 队列发表屏障后发送 bit1 的真实 IOCSR IPI。
pub fn send_ipi(hardware_id: usize) {
    let target = ipi_target(hardware_id).expect("IOCSR IPI target is out of range");
    barrier();
    iocsr_write32(reg::IOCSR_IPI_SEND, target | 1);
}
/// 写入已验证 PCH-PIC 范围的 32 位寄存器，共享 word 由控制器锁保护。
fn pic_write(offset: usize, value: u32) {
    // SAFETY: 已发布 BootInfo 的控制器范围覆盖调用点的对齐 offset；入口保持强序 DMW 永久有效。
    unsafe {
        core::ptr::write_volatile(
            (mmio_address(crate::boot::info().interrupts.controller.start) + offset) as *mut u32,
            value,
        );
    }
}
/// 读取已验证 PCH-PIC 范围的 32 位寄存器。
fn pic_read(offset: usize) -> u32 {
    // SAFETY: BootInfo getter 返回已发布对象，调用点 offset 对齐且位于控制器范围，强序 DMW 永久有效。
    unsafe {
        core::ptr::read_volatile(
            (mmio_address(crate::boot::info().interrupts.controller.start) + offset) as *const u32,
        )
    }
}
/// 将已验证的 EXTIOI 基址与本模块寄存器偏移组合成 IOCSR 编号。
fn ext_reg(offset: usize) -> usize {
    crate::boot::info().interrupts.parent_controller.start + offset
}
static EXTERNAL_CONTROLLER: SpinLock<()> = SpinLock::new(());

/// 定位实际源所在的共享 32 位 word 与位掩码。
fn source_word(source: u32) -> (usize, u32) {
    assert!(source < crate::boot::info().interrupts.source_count);
    (
        source as usize / u32::BITS as usize * IO_WORD_BYTES,
        1 << (source % u32::BITS),
    )
}

/// EXTIOI 路由字节用高半表示 node、低半表示该 node 的四核位图。
pub(crate) const fn extioi_route(hardware_id: usize) -> Option<u8> {
    let node = hardware_id / EXTIOI_CORES_PER_NODE;
    let core = hardware_id % EXTIOI_CORES_PER_NODE;
    if node < EXTIOI_NODE_COUNT {
        Some(((node as u8) << EXTIOI_ROUTE_NODE_SHIFT) | (1u8 << core))
    } else {
        None
    }
}

/// 按平台快照初始化 PCH/EXTIOI 或两组 LIOINTC，所有 level 源固定路由实际 BSP。
///
/// # Safety
/// BSP 必须在本核 IRQ 关闭且 AP 尚未释放时独占初始化；info 是永久发布的平台对象，
/// 所选控制器的 DMW/IOCSR 范围已经验证且永久有效。
pub unsafe fn external_init(info: &BootInfo) {
    if info.platform == crate::boot::Platform::Ls2k1000la {
        liointc::initialize(info);
        return;
    }
    assert_eq!(info.interrupts.source_count, PCH_PIC_SOURCE_COUNT);
    let route = extioi_route(info.bsp_hardware_id).expect("EXTIOI target node is out of range");
    // QEMU 的传统 EXTIOI 只解码 node 0 的四核位图；其余核仍可正常启动和接收 IPI。
    // 高 node 的外部 IRQ 需要另行配置 CPU_ENCODE，不能让模拟器静默投递到 core 0。
    assert_eq!(
        info.bsp_hardware_id / EXTIOI_CORES_PER_NODE,
        0,
        "EXTIOI bitmap mode cannot route to a remote node"
    );
    assert_eq!(info.interrupts.root_interrupt, EXTIOI_ROOT_INTERRUPT);
    let _controller = EXTERNAL_CONTROLLER.lock();
    for source in 0..info.interrupts.source_count {
        let (word, bit) = source_word(source);
        pic_write(PCH_PIC_MASK + word, pic_read(PCH_PIC_MASK + word) | bit);
        pic_write(
            PCH_PIC_HTMSI_ENABLE + word,
            pic_read(PCH_PIC_HTMSI_ENABLE + word) | bit,
        );
        pic_write(PCH_PIC_EDGE + word, pic_read(PCH_PIC_EDGE + word) & !bit);
        pic_write(
            PCH_PIC_POLARITY + word,
            pic_read(PCH_PIC_POLARITY + word) & !bit,
        );
        // SAFETY: 实际源小于 64，两个字节寄存器完整落在已验证 PCH 窗口；
        // 控制器锁与 AP 前初始化保证独占，各源路由到同号 EXTIOI 向量。
        unsafe {
            let base = mmio_address(info.interrupts.controller.start);
            core::ptr::write_volatile(
                (base + PCH_PIC_ROUTE + source as usize) as *mut u8,
                PCH_PIC_ROUTE_HT,
            );
            core::ptr::write_volatile(
                (base + PCH_PIC_VECTOR + source as usize) as *mut u8,
                source as u8,
            );
        }
        let route_reg = ext_reg(EXTIOI_ROUTE + (source as usize & !(IO_WORD_BYTES - 1)));
        let shift = source % IO_WORD_BYTES as u32 * u8::BITS;
        let route = u32::from(route) << shift;
        iocsr_write32(
            route_reg,
            (iocsr_read32(route_reg) & !(u32::from(u8::MAX) << shift)) | route,
        );
        iocsr_write32(
            ext_reg(EXTIOI_ENABLE + word),
            iocsr_read32(ext_reg(EXTIOI_ENABLE + word)) & !bit,
        );
        iocsr_write32(
            ext_reg(EXTIOI_BOUNCE + word),
            iocsr_read32(ext_reg(EXTIOI_BOUNCE + word)) | bit,
        );
        iocsr_write32(ext_reg(EXTIOI_ISR + word), bit);
    }
    // QEMU 的 IPMAP 每个字节选择一组 32 向量的 CPU IP，NODEMAP 每组占 16 位。
    for group in 0..(PCH_PIC_SOURCE_COUNT / u32::BITS) as usize {
        let ip_reg = ext_reg(EXTIOI_IPMAP + group / IO_WORD_BYTES * IO_WORD_BYTES);
        let shift = group % IO_WORD_BYTES * u8::BITS as usize;
        iocsr_write32(
            ip_reg,
            (iocsr_read32(ip_reg) & !(u32::from(u8::MAX) << shift)) | (EXTIOI_IPMAP_HWI1 << shift),
        );
        let node_reg = ext_reg(EXTIOI_NODEMAP + group / NODEMAP_ENTRIES_PER_WORD * IO_WORD_BYTES);
        let shift = group % NODEMAP_ENTRIES_PER_WORD * u16::BITS as usize;
        iocsr_write32(
            node_reg,
            (iocsr_read32(node_reg) & !(u32::from(u16::MAX) << shift)) | ((1 << group) << shift),
        );
    }
    iocsr_write64(
        IOCSR_MISC_FUNCTION,
        iocsr_read64(IOCSR_MISC_FUNCTION) | IOCSR_EXTIOI_ENABLE,
    );
    barrier();
}

/// 在控制器短锁内只屏蔽实际源，不改变相邻设备。
pub fn external_mask(source: u32) {
    if crate::platform::current() == crate::boot::Platform::Ls2k1000la {
        liointc::set_enabled(source, false);
        return;
    }
    let (word, bit) = source_word(source);
    let _controller = EXTERNAL_CONTROLLER.lock();
    pic_write(PCH_PIC_MASK + word, pic_read(PCH_PIC_MASK + word) | bit);
    let reg = ext_reg(EXTIOI_ENABLE + word);
    iocsr_write32(reg, iocsr_read32(reg) & !bit);
    barrier();
}

/// 设备已准备后开放实际 level 源，保持父控制器路由与相邻源不变。
pub fn external_unmask(source: u32) {
    if crate::platform::current() == crate::boot::Platform::Ls2k1000la {
        liointc::set_enabled(source, true);
        return;
    }
    let (word, bit) = source_word(source);
    let _controller = EXTERNAL_CONTROLLER.lock();
    pic_write(PCH_PIC_CLEAR + word, bit);
    let reg = ext_reg(EXTIOI_ENABLE + word);
    iocsr_write32(reg, iocsr_read32(reg) | bit);
    pic_write(PCH_PIC_MASK + word, pic_read(PCH_PIC_MASK + word) & !bit);
    barrier();
}

/// 读取本核实际父控制器的 pending 和准入位，返回一个规范化设备源。
pub fn external_claim() -> Option<u32> {
    if crate::platform::current() == crate::boot::Platform::Ls2k1000la {
        return liointc::claim();
    }
    let _controller = EXTERNAL_CONTROLLER.lock();
    let enabled = iocsr_read32(ext_reg(EXTIOI_ENABLE)) as u64
        | ((iocsr_read32(ext_reg(EXTIOI_ENABLE + IO_WORD_BYTES)) as u64) << u32::BITS);
    let pending = iocsr_read64(ext_reg(EXTIOI_ISR)) & enabled;
    if pending == 0 {
        None
    } else {
        Some(pending.trailing_zeros())
    }
}

/// 完成本核真实 claim；PCH 清对应锁存，LIOINTC 在设备应答后完成访问排序。
///
/// # Safety
/// source 必须来自本核尚未完成的真实 claim；源已屏蔽，处理者已退出，
/// 本核不可迁移，调用者持该源 descriptor 锁保护完成与重新开放。
pub unsafe fn external_complete(source: u32) {
    if crate::platform::current() == crate::boot::Platform::Ls2k1000la {
        liointc::complete(source);
        return;
    }
    let (word, bit) = source_word(source);
    let _controller = EXTERNAL_CONTROLLER.lock();
    iocsr_write32(ext_reg(EXTIOI_ISR + word), bit);
    barrier();
}
/// 用户与 FS 生命周期已停稳后停其它 online 核，再执行实际 PMC 或 virt GED 断电请求。
/// 调用者在普通 IRQ 可服务上下文中，不持锁且没有需要继续运行的 CSD 责任。
#[cfg(not(feature = "ktest"))]
pub fn shutdown() -> ! {
    let _stop = crate::cpu::stop_other_online_cpus();
    if crate::platform::current() == crate::boot::Platform::Ls2k1000la {
        board_action(crate::boot::info().board.poweroff);
    }
    ged_action(GED_SLEEP_CONTROL, GED_SLEEP_ENABLE | GED_SLEEP_S5)
}

/// 用户与 FS 生命周期已停稳后永久停全部核，不请求 PMC 或 GED 断电。
/// 调用环境与 shutdown 相同，永久停核确认不能用于恢复 CPU。
#[cfg(not(feature = "ktest"))]
pub fn halt() -> ! {
    let _stop = crate::cpu::stop_other_online_cpus();
    disable_stop_sources();
    crate::console::board_stop();
    permanent_idle()
}

/// 用户与 FS 生命周期已停稳后停其它 online 核，再执行实际 PMC 或 virt GED 重启请求。
/// 调用环境与 shutdown 相同，寄存器宽度与 mask 由平台资源确定。
#[cfg(not(feature = "ktest"))]
pub fn restart() -> ! {
    let _stop = crate::cpu::stop_other_online_cpus();
    if crate::platform::current() == crate::boot::Platform::Ls2k1000la {
        board_action(crate::boot::info().board.reboot);
    }
    ged_action(GED_RESET, GED_RESET_VALUE)
}

/// 致命错误不等待 AP/CSD 或文件生命周期；板只停当前核，不发布全机清理终点。
#[cfg(feature = "ktest")]
pub fn emergency_shutdown() -> ! {
    if crate::platform::current() == crate::boot::Platform::Ls2k1000la {
        disable_stop_sources();
        crate::console::board_stop_current_cpu();
        permanent_idle();
    }
    ged_action(GED_SLEEP_CONTROL, GED_SLEEP_ENABLE | GED_SLEEP_S5)
}

/// QEMU 的 catalog、Boot 或 Thread 测试结束后直接请求 GED 断电。
/// 测试构建只允许 QEMU；早期退出不要求其它核上线或生产服务初始化。
#[cfg(feature = "ktest")]
pub fn test_shutdown() -> ! {
    ged_action(GED_SLEEP_CONTROL, GED_SLEEP_ENABLE | GED_SLEEP_S5)
}

/// 关闭本核可唤醒源，发布永久确认后只执行 IDLE；只供已完成 CSD 的停核 IPI 使用。
pub(crate) fn stop_current_cpu(stopped: &AtomicBool) -> ! {
    disable_stop_sources();
    stopped.store(true, Ordering::Release);
    permanent_idle()
}

/// 永久停核前关闭本核 timer、全局 IRQ 与 ECFG 使能源，不恢复被中断的执行者。
fn disable_stop_sources() {
    irq_save();
    disable_timer();
    // SAFETY: 入口运行于 PLV0，ECFG 为本核 CSR；清使能位只关闭此核 IRQ 源，
    // 停核已不再需要 trap 返回或调度，本函数不改变共享控制器及内核映射。
    unsafe { csr_write::<{ reg::CSR_ECFG }>(0) };
    barrier();
}

/// 对固定 GED 字节寄存器执行最终平台动作，之后本核不再运行生产者。
fn ged_action(offset: usize, value: u8) -> ! {
    disable_stop_sources();
    // SAFETY: 所选 virt 平台的 GED 基址 0x100e001c 在永久强序 DMW 中；
    // 私有调用仅传 sleep_ctl 偏移 0/值 0x34 或 reset 偏移 2/值 0x42，设备要求单字节访问。
    unsafe {
        core::ptr::write_volatile(mmio_address(GED_BASE + offset) as *mut u8, value);
    }
    barrier();
    permanent_idle()
}

/// 执行实际 DT syscon 的最终 RMW；无对应动作时只停核，不声称物理断电。
#[cfg(not(feature = "ktest"))]
fn board_action(action: Option<crate::boot::SysconAction>) -> ! {
    disable_stop_sources();
    crate::console::board_stop();
    if let Some(action) = action {
        let address = mmio_address(action.registers.start) + action.offset as usize;
        // SAFETY: 启动边界已验证 syscon 范围、对齐及 mask/value，DMW 永久有效；
        // 其它 CPU 已永久停稳，本核 IRQ 已关闭，不与其它 PMC 操作并发。
        unsafe {
            let pointer = address as *mut u32;
            let old = pointer.read_volatile();
            pointer.write_volatile((old & !action.mask) | (action.value & action.mask));
        }
        barrier();
    }
    permanent_idle()
}

/// IRQ 与本核使能源已关闭后永久 IDLE，不返回 trap 或普通执行栈。
fn permanent_idle() -> ! {
    loop {
        wait_for_interrupt();
    }
}
#[cfg(feature = "ktest")]
/// 触发明确的非法指令，保留真实 kernel fault 现场。
pub fn trigger_fault() -> ! {
    // SAFETY: 此接口有意执行非法指令并由已安装的内核异常入口终止，不访问 Rust 内存。
    unsafe {
        asm!(".word 0xffffffff", options(noreturn));
    }
}

/// 读取 DMW 之外的低虚拟地址，触发真实 TLB refill 并保留 TLBRBADV 现场。
#[cfg(feature = "ktest")]
pub fn trigger_unmapped_fault() -> ! {
    // SAFETY: 两种引导入口均启用 PG；0x1000 不属于任何 DMW，内核没有安装低地址 TLB 映射。
    // 使用汇编执行故障 load，避免用 Rust 指针解引用表达无效地址访问。
    unsafe {
        asm!("ld.d {value}, {address}, 0", value = out(reg) _, address = in(reg) 0x1000usize, options(nostack));
    }
    panic!("unmapped load unexpectedly returned");
}

/// 分派真实硬中断并记录本核深度；异常直接终止，FP 关闭时不读取条件字段。
/// 等待窗口被中断时先退出汇编等待，避免原因已消费后再次进入 IDLE。
#[no_mangle]
extern "C" fn loongarch_trap(frame: &mut KernelTrapFrame) {
    if (frame.estat >> reg::ESTAT_ECODE_SHIFT) & reg::ESTAT_ECODE_MASK != 0 {
        #[cfg(feature = "ktest")]
        crate::ktest::fatal_event(
            if (frame.estat >> reg::ESTAT_ECODE_SHIFT) & reg::ESTAT_ECODE_MASK == reg::ECODE_INE {
                "illegal"
            } else {
                "panic"
            },
            format_args!(
                "synchronous trap pc={:#x} sp={:#x} cause={:#x} bad={:#x}",
                frame.era, frame.gpr[3], frame.estat, frame.badv
            ),
        );
        #[cfg(not(feature = "ktest"))]
        crate::console::fatal(frame.era, frame.gpr[3], frame.estat, frame.badv);
    }
    // 本次中断可能已消费纯唤醒原因，必须先返回调度器重查队列，不能重新进入等待窗口。
    let (idle_start, idle_end) = (
        core::ptr::addr_of!(la_idle_window_start) as usize,
        core::ptr::addr_of!(la_idle_window_end) as usize,
    );
    if idle_start <= frame.era && frame.era < idle_end {
        frame.era = core::ptr::addr_of!(la_idle_window_exit) as usize;
    }
    handle_hardware_interrupt(frame.estat);
}

/// 在当前 IRQ-off 核完成真实硬中断；返回前 guard 已退出，调用者才能请求普通调度。
fn handle_hardware_interrupt(estat: usize) {
    let _hardirq = crate::cpu::HardIrqGuard::enter();
    let pending = estat & csr_read::<{ reg::CSR_ECFG }>() & reg::ESTAT_IRQ_MASK;
    if pending & reg::IRQ_TIMER != 0 {
        disable_timer();
        crate::cpu::handle_timer();
    }
    if pending & reg::IRQ_IPI != 0 {
        let bits = iocsr_read32(reg::IOCSR_IPI_STATUS);
        iocsr_write32(reg::IOCSR_IPI_CLEAR, bits);
        barrier();
        crate::cpu::handle_ipi();
    }
    let external = if crate::platform::current() == crate::boot::Platform::Ls2k1000la {
        reg::IRQ_HWI0 | reg::IRQ_HWI1
    } else {
        reg::IRQ_HWI1
    };
    if pending & external != 0 {
        crate::irq::handle_external();
    }
}
/// 接收 refill 专用 CSR 现场并终止；TLBRERA 的 IsTLBR 位保留为真实 cause。
#[no_mangle]
extern "C" fn loongarch_refill(frame: &RefillFrame) -> ! {
    // refill 不产生普通 ESTAT.Ecode；cause 记录 TLBRERA.IsTLBR 的真实值。
    #[cfg(feature = "ktest")]
    crate::ktest::fatal_event(
        "refill",
        format_args!(
            "refill trap pc={:#x} sp={:#x} cause={:#x} bad={:#x}",
            frame.era & !reg::TLBRERA_IS_TLBR,
            frame.gpr[3],
            frame.era & reg::TLBRERA_IS_TLBR,
            frame.badv
        ),
    );
    #[cfg(not(feature = "ktest"))]
    crate::console::fatal(
        frame.era & !reg::TLBRERA_IS_TLBR,
        frame.gpr[3],
        frame.era & reg::TLBRERA_IS_TLBR,
        frame.badv,
    )
}

/// 以一次真实本核 timer IRQ 检查指定 FP 模式与八位 FCC 图样，不遍历其他参数。
///
/// # Safety
/// 本核异常入口、timer IRQ 源、永久 CpuLocal 和当前栈必须已安装且可服务中断；
/// 调用期间必须固定 CPU、禁止调度迁移，不得持有 IRQ-save 锁或 handler 所需锁，
/// 也不得有活跃 FP 上下文。原汇编恢复被触及的 f0/f24、FCSR、全部 FCC、EUEN、
/// KS0 和整数 ABI 现场；fp=false 跳过所有 FP 指令并忽略 fcc。
/// 函数恢复进入时的 IRQ 位；调用者必须在同核 IRQ-off 下用 time::reprogram_current
/// 交还生产 deadline（包括断言失败后的清理），不能把临时测试事件当成生产配置。
#[cfg(feature = "ktest")]
pub(crate) unsafe fn ktest_register_pattern(fp: bool, fcc: u8) -> bool {
    use core::sync::atomic::Ordering;
    let _irq = crate::sync::IrqSaveGuard::new();
    let counter = &crate::cpu::current().timer_interrupts;
    let before = counter.load(Ordering::Acquire);
    let ticks = ((counter_frequency() / 10000).max(0x600) as usize
        + (reg::TIMER_QUANTUM as usize - 1))
        & !(reg::TIMER_QUANTUM as usize - 1);
    // SAFETY: 调用者保证固定本核、真实异常/timer 与栈有效、无活跃 FP 或持锁现场；
    // 永久 counter 属于本核，ticks 为原测试对齐周期，fcc 为八位值，fp 仅取 0/1。
    let good =
        unsafe { la_test_register_preservation(counter, ticks, usize::from(fcc), usize::from(fp)) }
            != 0;
    good && before < counter.load(Ordering::Acquire)
}

/// 在真实普通 Task 中检查 yield 的 callee-saved、栈及返回路径，并恢复调用者现场。
#[cfg(feature = "ktest")]
pub fn test_task_context_preservation() -> bool {
    assert!(
        crate::task::current_task_id().is_some(),
        "context probe requires a running task"
    );
    // SAFETY: 仅向汇编传递真实 yield 函数入口；汇编保持 ABI 栈对齐及调用者现场，
    // yield 自身检查可调度边界，探针不启用 FP、不恢复旧 CPU 基址。
    unsafe { la_test_task_context_preservation(crate::sched::yield_now as usize) != 0 }
}

/// 用一次真实 timer trap 检验整数、SP、KS0 与返回路径，不执行任何 FP 指令。
///
/// # Safety
/// 本核异常入口、timer IRQ、CpuLocal 已安装；不得持锁或 guard，不得有活跃 FP 现场。
/// 测试临时开放 IRQ 并覆盖本核 timer，期间不得调度或迁移。
#[cfg(feature = "ktest")]
pub(crate) unsafe fn test_integer_register_preservation() -> usize {
    let saved = irq_save();
    let counter = &crate::cpu::current().timer_interrupts;
    let before = counter.load(core::sync::atomic::Ordering::Acquire);
    let ticks = ((counter_frequency() / 1000).max(reg::TIMER_QUANTUM) as usize
        + (reg::TIMER_QUANTUM as usize - 1))
        & !(reg::TIMER_QUANTUM as usize - 1);
    // SAFETY: 调用者保证本核 trap/timer 与栈可服务；counter 永久有效。
    // fp=0 使汇编跳过全部 FP/FCSR/FCC 访问，返回恢复同核整数与 IRQ-off 现场。
    let good = unsafe { la_test_register_preservation(counter, ticks, 0, 0) };
    assert!(good != 0, "integer timer trap register preservation failed");
    assert!(
        before < counter.load(core::sync::atomic::Ordering::Acquire),
        "integer timer trap was not observed"
    );
    crate::time::reprogram_current();
    // SAFETY: saved 来自同核，汇编已恢复现场，测试期间没有迁移或留存受 IRQ 保护的锁。
    unsafe { irq_restore(saved) };
    1
}
