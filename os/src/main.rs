//! 内核根模块与生命周期主入口（Kernel Entry & System Lifecycle）。
//!
//! # 模块职责与架构定位
//! 本文件是 Blossom 内核的 crate 根模块，负责：
//! 1. 声明全部内核子系统模块，并提供全局 `#[panic_handler]` 与 `#[alloc_error_handler]`；
//! 2. 承接架构早期汇编跳转而来的主核入口 [`primary_main`] 与从核入口 [`secondary_main`]；
//! 3. 按严格的单核（BSP）到多核（SMP）、再到初始内核线程 [`init`] 的阶段顺序，
//!    依次拉起固件解析、物理内存与页表管理、多核 CPU 上下文、时间与定时器、后台服务线程、
//!    中断控制器与串口、存储与文件系统、网络协议栈以及用户态初始进程（PID 1）；
//! 4. 在 PID 1 退出并提交关机/重启请求后，按逆序排空子进程、停止网络、同步并卸载根文件系统，
//!    最终调用底层硬件停机、断电或重启。
//!
//! # 核心设计与启动/关机生命周期状态机
//! ```text
//! [架构汇编 _start]
//!       │ (仅 BSP 进入，关中断，使用早期启动栈与早期页表/DMW)
//!       ▼
//! primary_main(arg0..arg3)
//!       ├──► initialize_boot()       [独立栈帧，返回后释放局部变量栈空间]
//!       │      ├── console::initialize()
//!       │      ├── platform::discover()  (解析 DTB / EFI，填充静态 BootInfo 槽)
//!       │      ├── boot::publish()       (将 BootInfo 冻结为全局只读引用)
//!       │      ├── arch::finish_boot_mappings() & cpu::initialize()
//!       │      └── arch::init_current_cpu()
//!       │
//!       ├──► (可选 ktest) 若请求 Boot 上下文测试则直接执行并退出
//!       │
//!       └──► run_kernel()
//!              ├── mm::initialize() & cpu::prepare_secondary_stacks()
//!              ├── arch::establish_counter_epoch() & time::initialize()
//!              ├── 启动内核服务任务 (timer / workqueue / inode_eviction / klog / tty)
//!              ├── 派生初始任务 init (或 ktest 线程)
//!              ├── irq::initialize() & 注册 UART 中断 -> 开启本核中断 -> 标记 BSP 在线
//!              ├── cpu::start_secondaries() ──► [唤醒各 AP 进入 secondary_main]
//!              ├── EXECUTION_READY = true 并发送 IPI 唤醒已上线 AP
//!              └── enter_scheduler() ◄────────── [AP 完成本核初始化后在此汇合]
//!                        │
//!                        ▼ (调度器开始分发任务，BSP 运行 init 任务)
//!                      init()
//!                        ├── drivers::discover() & storage::Storage::initialize()
//!                        ├── init_filesystems()
//!                        │     ├── 挂载根文件系统与 /dev，初始化网络栈 (NIC 或 Loopback)
//!                        │     ├── 启动用户态 PID 1 并阻塞等待其退出
//!                        │     ├── 循环回收残余孤儿进程 (wait_root)
//!                        │     └── 停止网络 -> vfs.sync() -> 淘汰缓存 -> 等待异步回收任务退出
//!                        └── 执行硬件电源动作 (Halt / PowerOff / Restart)
//! ```
//!
//! # 同步与并发契约
//! - **单核独占期（Pre-SMP）**：从 `primary_main` 开始直至 `cpu::start_secondaries()` 之前，
//!   只有主核（BSP）运行且本地中断关闭（直至 `arch::enable_interrupts()`），此时可安全完成全局只写一次的静态初始化。
//! - **多核起跑屏障（[`EXECUTION_READY`]）**：主核与从核进入 [`enter_scheduler`] 后，
//!   均须等待 `EXECUTION_READY` 以 `Release`/`Acquire` 语义置为 `true` 后才开始调度任务；
//!   等待期间从核已开启本地中断，可正常响应 IPI 与外设中断。

#![no_std]
#![no_main]
#![feature(asm_experimental_arch)]
#![feature(alloc_error_handler)]
#![feature(allocator_api)]
#![feature(naked_functions)]
#![deny(unsafe_op_in_unsafe_fn)]
#![cfg_attr(feature = "ktest", feature(panic_can_unwind))]

extern crate alloc;
extern crate rlibc;

mod accounting;
mod arch;
mod block;
mod boot;
mod cache;
mod console;
mod contest_hooks;
mod cpu;
mod drivers;
mod fs;
mod irq;
mod klog;
#[cfg(feature = "ktest")]
mod ktest;
mod mm;
mod net;
mod platform;
mod random;
mod sched;
mod storage;
mod sync;
mod syscall;
mod task;
mod time;
mod timer;
mod trace;
mod wait;
mod workqueue;

#[cfg(not(feature = "ktest"))]
use alloc::sync::Arc;

#[cfg(not(feature = "ktest"))]
use crate::drivers::DeviceInventory;

/// 多核调度起跑屏障：主核（BSP）完成全部启动阶段与从核唤醒后将其置为 `true`，
/// 所有核心在 [`enter_scheduler`] 中等待该标志置位后才正式进入任务调度循环。
static EXECUTION_READY: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

/// 在主核独立栈帧中完成早期控制台、固件资源发现、永久页表收尾与主核 CPU 本地状态初始化。
///
/// 使用 `#[inline(never)]` 让本函数拥有独立栈帧并在返回时回收，避免 LTO 将早期解析期间的临时对象
/// 留在 `primary_main` 或 `ktest` Boot 用例的 64 KiB 启动栈上。
///
/// # Safety
/// - 必须仅由主核（BSP）在 `primary_main` 最早期调用一次，此时本地中断关闭且从核（AP）尚未唤醒；
/// - `arguments` 必须为固件传入的原始四参数，早期汇编已建立内核镜像、RAM、DTB 与早期串口映射；
/// - 全局 `BootInfo` 静态存储槽处于未初始化且无任何并发引用的独占状态。
#[inline(never)]
unsafe fn initialize_boot(arguments: [usize; 4]) {
    // Step 1: 初始化早期串口输出，读取当前主核硬件 ID，解析固件 DTB/EFI 填充静态 BootInfo 槽。
    console::initialize();
    let hardware_id = arch::boot_hardware_id(arguments);
    // SAFETY:
    // 1. 当前处于主核单核关中断阶段，`arguments` 保存了固件原始入口参数，DTB 物理页已建立早期只读映射；
    // 2. `BOOT_STORAGE` 静态槽尚未发布，由当前主核唯一独占初始化。
    let resources = unsafe { platform::discover(arguments, hardware_id) }.unwrap_or_else(|error| {
        panic!(
            "boot resource discovery failed: {} args={:#x?}",
            error, arguments
        )
    });

    // Step 2: 校验内核镜像物理区间必须完全落在真实物理内存（RAM）范围内，并将其登记为保留区以防被页分配器分配。
    let kernel = boot::kernel_region();
    assert!(
        resources.memory[..resources.memory_count]
            .iter()
            .any(|region| { region.start <= kernel.start && kernel.end <= region.end }),
        "kernel load region is outside actual RAM"
    );
    resources
        .reserve(kernel)
        .expect("kernel reservation failed");

    // Step 3: 冻结并发布全局只读 BootInfo，切换至完整内核页表映射，并初始化主核 CpuLocal 与异常入口。
    // SAFETY:
    // 1. `resources` 是由 `platform::discover` 返回的唯一可变静态引用，已完成固件解析与内核区间保留；
    // 2. 从核尚未唤醒且本核关中断，调用 `publish` 消费该可变引用后，全局仅存在不可变引用。
    unsafe { boot::publish(resources) };
    let resources = boot::info();
    // SAFETY: 仍处于主核单核关中断引导阶段，`resources` 中的物理内存与设备窗口已校验完毕，当前代码、启动栈与静态页表均位于实际 RAM 内。
    unsafe { arch::finish_boot_mappings(resources) };
    // SAFETY: 当前为主核且中断关闭、从核未释放；启动栈与 `resources` 中的 CPU 硬件 ID 列表均有效。
    unsafe { cpu::initialize(resources) };
    arch::init_current_cpu();
}

/// 内核主核（BSP）C ABI 入口：由架构早期启动汇编在关中断状态下跳转进入，永不返回。
///
/// 负责协调两阶段引导（[`initialize_boot`] 与 [`run_kernel`]），在启用 `ktest` 时拦截并执行
/// `Boot` 上下文测试用例。
///
/// # Safety
/// - 必须仅由选定平台的主核（BSP）引导汇编调用一次，调用时本地中断关闭且所有从核（AP）处于暂停/未唤醒状态；
/// - 早期汇编必须已清零 `.bss` 段、设置好独占的 64 KiB 启动栈，并建立内核高地址镜像、RAM 与早期 UART 的地址映射；
/// - `arg0..arg3` 必须为固件传递的原始寄存器参数，DTB 镜像、EFI 系统表及命令行字符串所在物理内存必须可读且未与内核或 `BootInfo` 静态槽重叠。
#[no_mangle]
pub unsafe extern "C" fn primary_main(arg0: usize, arg1: usize, arg2: usize, arg3: usize) -> ! {
    // 先在独立栈帧中完成早期发现并真实返回，确保临时栈帧空间在进入 Boot 测试或主服务前释放。
    // SAFETY: 架构汇编入口保证了主核独占、中断关闭、早期映射就绪以及固件四参数原样传入。
    unsafe { initialize_boot([arg0, arg1, arg2, arg3]) };
    #[cfg(feature = "ktest")]
    let test_command = {
        let command = ktest::early();
        if ktest::requested_context(&command) == ::ktest::Context::Boot {
            ktest::run_boot(command);
        }
        command
    };
    #[cfg(feature = "ktest")]
    // SAFETY: `initialize_boot` 已发布全局 `BootInfo`、永久页表与本核 `CpuLocal`；`Boot` 级测试已提前分支退出，此处仅执行 `Thread` 级测试路径。
    unsafe {
        run_kernel(test_command)
    }
    #[cfg(not(feature = "ktest"))]
    // SAFETY: `initialize_boot` 已发布全局 `BootInfo`、永久页表与本核 `CpuLocal`，且当前中断仍关闭、从核尚未释放。
    unsafe {
        run_kernel()
    }
}

/// 主核引导后半段：初始化物理内存与堆分配器、从核启动栈、全局时间基准、后台内核线程、
/// 中断控制器与从核唤醒，最后进入调度器循环。
///
/// 使用 `#[inline(never)]` 隔离栈帧，防止本函数的大栈帧挤占 `primary_main` 中 `Boot` 测试用例的栈空间。
///
/// # Safety
/// - 必须在 [`initialize_boot`] 完成后由主核（BSP）调用一次；
/// - 全局 `BootInfo`、永久地址映射与主核 `CpuLocal` 必须已就绪，且调用时中断关闭、从核尚未唤醒。
#[inline(never)]
unsafe fn run_kernel(#[cfg(feature = "ktest")] test_command: ktest::RunCommand) -> ! {
    // Step 1: 打印引导摘要日志，初始化物理页分配器与内核堆，并为所有从核分配内核启动栈。
    let resources = boot::info();
    early_println!(
        "BOOT arch={} platform={:?} cpus={} bsp={} args={:#x},{:#x},{:#x},{:#x} dtb={:#x}..{:#x}",
        arch::NAME,
        resources.platform,
        resources.cpu_count,
        resources.bsp_hardware_id,
        resources.raw_args[0],
        resources.raw_args[1],
        resources.raw_args[2],
        resources.raw_args[3],
        resources.dtb.start,
        resources.dtb.end
    );
    // SAFETY:
    // 1. 当前为主核单核关中断阶段，从核尚未唤醒，系统中不存在其他并发物理内存分配者；
    // 2. 完整 RAM 线性直映已由 `arch::finish_boot_mappings` 建立，`resources` 中已将内核镜像、启动栈、DTB 与 initrd 标记为保留区。
    unsafe { mm::initialize(resources) };
    // SAFETY:
    // 1. 主核关中断且从核未唤醒，物理页分配器与线性直映已就绪；
    // 2. 全局 CPU 表已由 `cpu::initialize` 初始化，各从核槽位尚无并发读取者。
    unsafe { cpu::prepare_secondary_stacks() }.expect("AP stack allocation failed");

    // Step 2: 建立多核统一硬件计数器零点，初始化墙钟时间（RTC）与内核各后台服务任务，并派生初始任务。
    arch::establish_counter_epoch();
    // SAFETY: 主核在内存管理就绪后、中断与从核开放前唯一初始化时间子系统；`resources` 中的计数器频率与 RTC 寄存器区间已校验且永久 MMIO 映射有效。
    unsafe { time::initialize(resources) };

    timer::initialize_services().expect("timer service task allocation failed");
    workqueue::initialize_services().expect("worker pool service task allocation failed");
    fs::initialize_inode_eviction().expect("inode eviction service allocation failed");
    klog::initialize_service().expect("log printer task allocation failed");
    fs::tty::initialize_service().expect("TTY worker preparation failed");
    #[cfg(not(feature = "ktest"))]
    task::spawn_kernel_on(0, init).expect("initial kernel task allocation failed");
    #[cfg(feature = "ktest")]
    let _test_task = ktest::spawn_thread(test_command).expect("test suite Task allocation failed");

    // Step 3: 初始化中断控制器，注册并使能串口接收中断，开启主核中断并标记主核在线。
    // SAFETY: 主核异常入口与 `CpuLocal` 已安装，中断控制器 MMIO 映射已建立，且由主核唯一初始化一次全局中断路由。
    unsafe { irq::initialize(resources) };
    let uart_number = irq::map_source(resources.interrupts.uart_source)
        .expect("UART interrupt source is outside the IRQ domain");
    // SAFETY:
    // 1. `console::uart_handler` 不解引用 `context` 指针（传入 `null_mut()` 安全）；
    // 2. 串口寄存器映射与处理函数在内核生命周期内永久有效，此时该中断源尚未使能；
    // 3. 返回的注册凭证 `_uart_registration` 保活在永不返回的 `run_kernel` 栈帧中（或交由 `ktest` 辅助模块托管）。
    let _uart_registration =
        unsafe { irq::register(uart_number, console::uart_handler, core::ptr::null_mut()) }
            .expect("UART IRQ action registration failed");
    #[cfg(feature = "ktest")]
    console::testing::restore_uart_registration(_uart_registration)
        .expect("default UART registration cannot be installed");
    console::enable_receive_interrupts();
    #[cfg(not(feature = "ktest"))]
    _uart_registration.enable();
    arch::enable_interrupts();
    cpu::publish_online();

    // Step 4: 打印物理内存布局、保留区与 PCI 窗口信息，唤醒所有从核（AP）并发布 `EXECUTION_READY` 调度起跑信号。

    for region in &resources.memory[..resources.memory_count] {
        early_println!("RAM start={:#x} end={:#x}", region.start, region.end);
    }
    for region in &resources.reserved[..resources.reserved_count] {
        early_println!("RESERVED start={:#x} end={:#x}", region.start, region.end);
    }
    early_println!(
        "PCI config={:#x}..{:#x} windows={}",
        resources.pci.configuration.start,
        resources.pci.configuration.end,
        resources.pci.range_count
    );
    for window in &resources.pci.ranges[..resources.pci.range_count] {
        early_println!(
            "PCI WINDOW flags={:#x} bus={:#x} physical={:#x}..{:#x}",
            window.flags,
            window.bus_start,
            window.cpu.start,
            window.cpu.end
        );
    }

    // SAFETY: 所有 CPU 槽位与从核独占启动栈均已分配并初始化完毕，主核已开启中断可正常处理 IPI。
    unsafe { cpu::start_secondaries() };
    for id in 0..cpu::count() {
        let local = cpu::get(id);
        early_println!(
            "CPU id={} hardware={} local={:#x} stack={:#x}",
            local.id,
            local.hardware_id,
            local as *const _ as usize,
            local.stack_top()
        );
    }

    early_println!("READY arch={} cpus={}", arch::NAME, cpu::count());
    EXECUTION_READY.store(true, core::sync::atomic::Ordering::Release);
    for id in 0..cpu::count() {
        let local = cpu::get(id);
        if id != cpu::current().id && local.online.load(core::sync::atomic::Ordering::Acquire) {
            cpu::kick(id, cpu::KICK_RESCHEDULE);
        }
    }
    enter_scheduler();
}

/// 从核（AP）C ABI 入口：由从核启动汇编在关中断状态下跳转进入，完成本核初始化后进入调度器，永不返回。
///
/// # Safety
/// - 必须仅由从核引导汇编在主核调用 `cpu::start_secondaries()` 后调用一次，调用时本核中断关闭；
/// - `local` 必须指向主核为当前从核分配并已完成初始化、对齐且永久有效的 [`cpu::CpuLocal`] 实例；
/// - 本核专属启动栈、内核页表/DMW 映射以及主核发布的时间基准（counter epoch）必须已就绪。
#[no_mangle]
pub unsafe extern "C" fn secondary_main(local: *const cpu::CpuLocal) -> ! {
    // SAFETY: `local` 由从核启动汇编从主核静态发布的启动记录中取出，指向当前核心独占的 `CpuLocal`，此时本核中断关闭无并发访问。
    unsafe { arch::install_cpu_local(local) };
    // SAFETY: 本核尚未进行任何时间读取；主核已在唤醒从核前通过 `establish_counter_epoch` 建立了全局计数器零点。
    unsafe { arch::apply_counter_epoch() };
    arch::init_current_cpu();
    arch::enable_interrupts();
    cpu::publish_online();
    enter_scheduler();
}

/// 等待全局 [`EXECUTION_READY`] 起跑信号置位，随后正式进入本核任务调度主循环。
///
/// 在等待期间从核已开启本地中断，可以正常响应核间中断（IPI）与外设中断。
fn enter_scheduler() -> ! {
    cpu::wait_until_started(&EXECUTION_READY);
    sched::run()
}

/// 0 号核初始内核任务（`init`）：负责枚举总线与外设、初始化块存储层、挂载文件系统并运行用户态 PID 1，
/// 在用户态生命周期结束后执行底层关机、断电或重启操作。
#[cfg(not(feature = "ktest"))]
fn init() -> i32 {
    // Step 1: 扫描 MMIO 与 PCI 总线生成设备清单，并根据内核启动参数（bootargs）解析根设备与附加盘选择。
    let inventory = drivers::discover().expect("device resource discovery failed");
    report_device_resources(&inventory);
    let selected =
        fs::system::boot_devices(boot::info().bootargs()).expect("boot device selection failed");
    // Step 2: 初始化块设备与 RAM Disk 存储表。
    // SAFETY:
    // 1. `init` 是系统唯一的设备初始化任务，独占消费 `inventory` 中的块设备资源，且内核页表中的设备强序 MMIO 映射永久有效；
    // 2. 若启用了 `initrd`，其物理页已在 `BootInfo` 中登记为保留区且建立了永久线性直映，不会与页分配器或用户态映射冲突，其内存生命周期覆盖整个 RAM Disk 运行与停机过程。
    let storage = unsafe {
        storage::Storage::initialize(
            &inventory,
            selected.root.device,
            selected.extra.map(|selector| selector.device),
        )
    }
    .expect("storage initialization failed");

    crate::klog!(
        klog::Level::Info,
        "STORAGE online devices={}",
        storage.count()
    );
    // Step 3: 挂载文件系统、初始化网络栈并运行用户态初始进程（PID 1），待其退出并完成同步卸载后执行硬件关机/重启。
    // SAFETY:
    // 1. `inventory` 来自上方同一次设备枚举，`Storage::initialize` 仅消费了块设备候选，网络设备及板级 MAC/PHY/时钟复位资源仍由当前 `init` 任务独占；
    // 2. `inventory` 与永久设备 MMIO 映射在 `init_filesystems` 整个运行及停机清理期间保持有效。
    let operation = unsafe { init_filesystems(storage, &inventory, selected) }
        .expect("system lifecycle failed");
    drop(inventory);
    match operation {
        task::process::ShutdownOperation::Halt => arch::halt(),
        task::process::ShutdownOperation::PowerOff => arch::shutdown(),
        task::process::ShutdownOperation::Restart => arch::restart(),
    }
}

/// 挂载根文件系统与 `/dev`、初始化网络协议栈、启动用户态初始进程（PID 1），
/// 并在 PID 1 退出后回收全部残留子进程、关闭网络、刷回并卸载文件系统。
///
/// # Safety
/// - `inventory` 必须与初始化 `storage` 时使用的是同一次 `drivers::discover()` 结果；
/// - 调用方必须保证 `inventory` 中的网络设备（VirtIO-Net / GMAC）以及板级时钟、复位、引脚、syscon 寄存器仍处于未初始化的独占状态；
/// - `storage`、`inventory`、`BootInfo` 及内核强序设备 MMIO 映射必须在整个用户进程、网络栈与文件系统的运行及关闭期间保持有效。
#[cfg(not(feature = "ktest"))]
unsafe fn init_filesystems(
    storage: &storage::Storage,
    inventory: &drivers::DeviceInventory,
    selected: fs::system::BootDevices<'_>,
) -> Result<task::process::ShutdownOperation, task::ProgramError> {
    const INODE_CACHE_CAPACITY: usize = 64;

    // Step 1: 查找根块设备与可选附加块设备，创建 Inode 缓存并挂载根文件系统与 /dev 设备文件系统。
    let root = storage
        .lookup(selected.root)
        .ok_or(fs::FsError::MissingRoot)?;
    let extra = selected
        .extra
        .map(|selector| storage.lookup(selector).ok_or(fs::FsError::MissingRoot))
        .transpose()?;
    let inodes = Arc::try_new(fs::InodeCache::try_new(INODE_CACHE_CAPACITY)?)
        .map_err(|_| fs::FsError::NoMemory)?;
    let inode_eviction = inodes.watch();
    let (vfs, context) =
        fs::system::mount_disks(root, extra, inodes, selected.root_mode, selected.rootdir)?;
    let vfs = Arc::try_new(vfs).map_err(|_| fs::FsError::NoMemory)?;

    {
        let mountpoint = vfs.resolve(&context, b"/dev", fs::ResolveMode::Follow)?;
        let devfs = fs::devfs::try_new(0o755)?;
        vfs.mount(
            &mountpoint,
            fs::memfs::Filesystem::Memory(devfs),
            block::AccessMode::ReadOnly,
        )?;
    }
    crate::klog!(
        klog::Level::Info,
        "ROOTFS selected root={} extra={:?} mode={:?}",
        selected.root,
        selected.extra,
        selected.root_mode
    );

    // Step 2: 初始化网络协议栈（若发现网卡候选则初始化真实 NIC 驱动，否则回退为纯回环接口 lo）。
    let net_owner = if drivers::net_device::NetDevice::has_candidate(inventory) {
        // SAFETY:
        // 1. 调用方契约保证了 `inventory` 中的网卡候选及板级时钟/复位/PHY 资源由当前上下文唯一独占，且设备 MMIO 映射永久有效；
        // 2. 返回的 `net_owner` 保活底层网卡驱动，并在下方所有用户进程退出且回收完毕后（`drop(net_owner)`）才同步停止网卡。
        unsafe { net::NetStack::try_new_nic(inventory) }?
    } else {
        net::NetStack::try_new_lo()?
    };
    // Step 3: 构建系统进程关系图（TaskGraph），加载初始程序镜像并作为 PID 1 发布运行。
    let graph =
        task::process::TaskGraph::try_new(task::process::MAX_PID, task::process::RootRole::System)?;
    let arguments = task::exec::Arguments::from_kernel(
        selected.init,
        &[selected.init],
        &[
            b"PATH=/usr/sbin:/usr/bin:/sbin:/bin",
            b"HOME=/root",
            b"TERM=vt102",
        ],
        task::exec::DEFAULT_STACK_BYTES,
    )?;
    let image = task::exec::prepare(&vfs, &context, arguments, task::Credentials::ROOT)?;
    let runtime = task::UserRuntime::try_new(
        Arc::clone(&vfs),
        context.clone(),
        task::Credentials::ROOT,
        Some(Arc::clone(net_owner.stack())),
    )?;
    runtime.install_console()?;

    let prepared = task::prepare_program(&graph, None, image, runtime, task::Placement::Any)?;
    let pid = prepared.pid();
    assert_eq!(pid, 1, "system root must be PID1");
    let child = prepared.publish()?;
    let task_id = child.task_id();
    early_println!("INIT running pid={} task={}", pid, task_id.as_usize());

    // Step 4: 阻塞等待 PID 1 退出，读取其提交的系统关机动作（Halt / PowerOff / Restart）。
    let event = graph.wait(None, pid)?;
    let task_status = child.join();
    assert_eq!(event.pid, pid, "system wait returned another process");
    assert_eq!(event.task_id, task_id, "system wait returned another task");
    assert_eq!(task_status, event.status, "system join disagrees with wait");
    let operation = graph
        .shutdown_operation()
        .expect("system root exited without a shutdown request");
    early_println!(
        "INIT exited pid={} status={} operation={:?}",
        pid,
        event.status,
        operation
    );

    // Step 5: 此时新进程准入已随关机请求关闭，循环等待并回收进程树中所有残余子进程。
    loop {
        match graph.wait_root() {
            Ok(event) => early_println!(
                "INIT child-reaped pid={} task={} status={}",
                event.pid,
                event.task_id.as_usize(),
                event.status
            ),
            Err(task::process::ProcessError::NoChild) => break,
            Err(error) => return Err(error.into()),
        }
    }

    // Step 6: 停止网络协议栈与网卡 DMA，将文件系统脏数据同步落盘，清理 VFS 缓存并等待异步 Inode 回收线程彻底退出。
    drop(net_owner);
    early_println!("NET stopped");

    vfs.sync()?;
    drop(graph);
    drop(context);
    vfs.prune_unused();
    drop(vfs);
    inode_eviction.wait_retired()?;
    drop(inode_eviction);
    early_println!("ROOTFS synchronized and retired operation={:?}", operation);
    Ok(operation)
}

/// 将固件声明的 VirtIO-MMIO 槽位以及总线扫描发现的 MMIO 块设备、PCI 设备及 BAR 窗口记录到内核日志。
///
/// 仅输出已发现的静态资源描述信息，不触发设备队列初始化或磁盘读写。
#[cfg(not(feature = "ktest"))]
fn report_device_resources(inventory: &DeviceInventory) {
    let resources = boot::info();
    for (slot, resource) in resources.virtio_mmio[..resources.virtio_mmio_count]
        .iter()
        .enumerate()
    {
        crate::klog!(
            klog::Level::Info,
            "MMIO slot={} physical={:#x}..{:#x} source={}",
            slot,
            resource.registers.start,
            resource.registers.end,
            resource.interrupt_source
        );
    }

    for candidate in &inventory.mmio_blocks {
        crate::klog!(
            klog::Level::Info,
            "MMIO BLOCK physical={:#x}..{:#x} version={} vendor={:#x} irq={:?}",
            candidate.registers.start,
            candidate.registers.end,
            candidate.version,
            candidate.vendor_id,
            candidate.interrupt
        );
    }

    for device in inventory.pci.devices() {
        crate::klog!(
            klog::Level::Info,
            "PCI BDF={:02x}:{:02x}.{} vendor={:04x} device={:04x} header={:#x} caps={} irq={:?}",
            device.bus,
            device.device,
            device.function,
            device.vendor_id,
            device.device_id,
            device.header_type,
            device.capability_count,
            device.interrupt
        );
        for (index, bar) in device.bars.iter().enumerate() {
            if let Some(bar) = bar {
                crate::klog!(
                    klog::Level::Info,
                    "PCI BAR BDF={:02x}:{:02x}.{} index={} bus={:#x} physical={:#x}..{:#x} prefetch={}",
                    device.bus,
                    device.device,
                    device.function,
                    index,
                    bar.bus_address,
                    bar.physical.start,
                    bar.physical.end,
                    bar.prefetchable
                );
            }
        }
    }

    crate::klog!(
        klog::Level::Info,
        "DEVICE discovery mmio-blocks={} pci-functions={}",
        inventory.mmio_blocks.len(),
        inventory.pci.devices().len()
    );
}

/// 全局内核 Panic 处理入口：在启用 `ktest` 且支持栈展开时触发测试异常捕获或输出结构化失败事件；
/// 在普通运行模式下通过控制台打印崩溃位置与原因并暂停当前 CPU 核心。
#[panic_handler]
fn panic(info: &core::panic::PanicInfo<'_>) -> ! {
    #[cfg(feature = "ktest")]
    {
        if cfg!(panic = "unwind") && !arch::cpu_local().is_null() {
            ktest::panic::raise(info);
        }
        let kind = if info.can_unwind() { "panic" } else { "abi" };
        ktest::fatal_event(kind, format_args!("{}", info));
    }
    #[cfg(not(feature = "ktest"))]
    console::panic(info)
}

/// 全局内核堆内存分配失败（OOM）处理入口：输出请求分配的大小与对齐要求，并终止当前执行流（测试模式下上报 `oom` 事件，普通模式下触发 panic）。
#[alloc_error_handler]
fn allocation_error(layout: core::alloc::Layout) -> ! {
    #[cfg(feature = "ktest")]
    ktest::fatal_event(
        "oom",
        format_args!(
            "heap allocation failed: size={} align={}",
            layout.size(),
            layout.align()
        ),
    );
    #[cfg(not(feature = "ktest"))]
    panic!(
        "heap allocation failed: size={} align={}",
        layout.size(),
        layout.align()
    )
}
