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

static EXECUTION_READY: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

/// 在永久槽中完成一次性固件资源与 CPU 准备，发布后仅保留只读资源借用。
///
/// # Safety
/// 调用者须满足 primary_main 的 BSP、映射和固件参数约束。
#[inline(never)]
unsafe fn initialize_boot(arguments: [usize; 4]) {
    console::initialize();
    let hardware_id = arch::boot_hardware_id(arguments);
    // SAFETY: BSP 保留实际固件参数并建立 DTB 映射；本核独占尚未发布的永久资源槽。
    let resources = unsafe { platform::discover(arguments, hardware_id) }.unwrap_or_else(|error| {
        panic!(
            "boot resource discovery failed: {} args={:#x?}",
            error, arguments
        )
    });
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
    // SAFETY: 永久槽的独占借用已完成 DTB 解析和内核保留；AP 未释放，发布消费借用后仅允许只读访问。
    unsafe { boot::publish(resources) };
    let resources = boot::info();
    // SAFETY: 仍在独占 bootstrap 执行阶段；实际 RAM 包含当前镜像、栈和静态页表。
    unsafe { arch::finish_boot_mappings(resources) };
    // SAFETY: 当前为 BSP，IRQ 关闭、AP 未释放；静态栈和全部 CPU 输入均有效。
    unsafe { cpu::initialize(resources) };
    arch::init_current_cpu();
}

/// 使用固件资源准备 CPU、MM、统一时间与中断，再发表内核任务并释放 AP。
///
/// # Safety
/// 仅由选定机器的 BSP 引导入口调用一次，IRQ 关闭且 AP 未释放。
/// 早期初始化已建立内核/RAM/早期 UART 映射与独占启动栈，并清理普通 BSS。
/// 原始四参数在内存初始化前已保存，DTB 头部及声明全文可读且不覆盖内核。
/// 板固件提供的 EFI 系统表及配置表数组须在解析期间可读且不变；
/// 所选命令行须保活到快照复制结束，相关存储均不得与永久 BootInfo 槽重叠。
#[no_mangle]
pub unsafe extern "C" fn primary_main(arg0: usize, arg1: usize, arg2: usize, arg3: usize) -> ! {
    // 引导构造帧先真实返回，避免 LTO 把大数组留在 Boot 展开的 64KiB 栈上。
    // SAFETY: 机器入口与早期初始化满足本入口的独占状态、实际映射和固件参数约束。
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
    // SAFETY: 引导资源与本核状态已发表；命令已确认只走 Thread 路径。
    unsafe {
        run_kernel(test_command)
    }
    #[cfg(not(feature = "ktest"))]
    // SAFETY: 引导资源与本核状态已发表，IRQ 关闭且 AP 尚未释放。
    unsafe {
        run_kernel()
    }
}

/// 从已完成的引导进入完整服务；其长期构造帧不得覆盖 Boot 用例。
///
/// # Safety
/// 固件资源、永久映射和本核 CPU 状态已发表；IRQ 关闭且 AP 尚未释放。
#[inline(never)]
unsafe fn run_kernel(#[cfg(feature = "ktest")] test_command: ktest::RunCommand) -> ! {
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
    // SAFETY: BSP 尚未释放 AP，IRQ 仍关闭；实际 RAM 永久可写的一一直映已完成，
    // BootInfo 已保留内核、启动栈、页表与固件资源。此时没有其他空闲 RAM 使用者，
    // 后续物理页使用以此描述符表为唯一边界，不通过别名修改或回收元数据 backing。
    unsafe { mm::initialize(resources) };
    // SAFETY: 唯一 BSP 仍保持 IRQ 关闭且 AP 未释放；MM 已完成页分配器和永久直映，
    // CPU 表已提前构造，辅核记录尚无固件读者，全部动态槽尚为空。
    unsafe { cpu::prepare_secondary_stacks() }.expect("AP stack allocation failed");
    // 统一各核计数器零点后才能开放时间服务与 AP 采样。
    arch::establish_counter_epoch();
    // SAFETY: 唯一 BSP 在 MM 初始化后、IRQ 与 AP 开放前初始化时间源；
    // BootInfo 的频率与 RTC 资源已验证，永久 MMIO 映射有效，尚无其他 RTC 访问者。
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
    // SAFETY: 本核基址/异常入口已安装，实际控制器已映射；仅 BSP 初始化路由一次。
    unsafe { irq::initialize(resources) };
    let uart_number = irq::map_source(resources.interrupts.uart_source)
        .expect("UART interrupt source is outside the IRQ domain");
    // SAFETY: handler 不使用 context；端口、映射和函数永久存在，尚未开放源。
    // token 在普通启动栈中保活到调度器停机，测试镜像将其移动给同步注销拥有者。
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
    // SAFETY: 全部记录已构造，本核能服务 IPI；每个 AP 的独占栈和 CPU 对象永久保留。
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

/// 使用 BSP 发布的 CpuLocal 安装本核执行状态，只初始化本地中断。
///
/// # Safety
/// 仅由 AP 引导汇编调用一次；local 是本核已构造、对齐且永久有效的 CpuLocal。
/// 独占栈和所需映射已安装，记录来自 BSP，当前 IRQ 关闭。
#[no_mangle]
pub unsafe extern "C" fn secondary_main(local: *const cpu::CpuLocal) -> ! {
    // SAFETY: AP 汇编从 BSP 的静态记录读出本核对象，尚无本核 IRQ 访问。
    unsafe { arch::install_cpu_local(local) };
    // SAFETY: 本核尚无时间采样；BSP 已在开放 AP 前建立统一补偿。
    unsafe { arch::apply_counter_epoch() };
    arch::init_current_cpu();
    arch::enable_interrupts();
    cpu::publish_online();
    enter_scheduler();
}

/// 全部 CPU 已上线且启动输出完成后才允许分派，等待中的 AP 仍能服务 IRQ/IPI。
fn enter_scheduler() -> ! {
    cpu::wait_until_started(&EXECUTION_READY);
    sched::run()
}

/// 在普通任务栈一次解析启动选择并发现设备；等待用户、网络和文件系统退休后执行硬件动作。
#[cfg(not(feature = "ktest"))]
fn init() -> i32 {
    let inventory = drivers::discover().expect("device resource discovery failed");
    report_device_resources(&inventory);
    let selected =
        fs::system::boot_devices(boot::info().bootargs()).expect("boot device selection failed");
    // SAFETY: init 是本次唯一发现后全部块设备的唯一准备者；启动页表的完整强序映射永久有效。
    // 固定 Storage 表在发表前持全部 owner，库存保留到用户、网络和文件系统全部结束。
    // initrd 已完整保留并永久直映；本轮唯一 Storage 取得其字节拥有，固件及其他设备
    // 不再访问该区间，用户映射不包含这些保留页，拥有持续到 RAM Disk 停止全尾结束。
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
    // SAFETY: 库存来自上述唯一发现，Storage 只拥有块设备，网络候选仍由 init 独占。
    // 库存保留到本调用结束，启动页表的完整强序设备映射在全部消费者和停止尾部期间永久有效。
    let operation = unsafe { init_filesystems(storage, &inventory, selected) }
        .expect("system lifecycle failed");
    drop(inventory);
    match operation {
        task::process::ShutdownOperation::Halt => arch::halt(),
        task::process::ShutdownOperation::PowerOff => arch::shutdown(),
        task::process::ShutdownOperation::Restart => arch::restart(),
    }
}

/// 复用初始化 Storage 前解析的目录和设备选择启动 PID1；用户退出后结束网络并同步退休文件系统。
///
/// # Safety
/// inventory 须来自初始化 storage 的同一次唯一发现，调用者仍独占其中的网络设备及板 MAC、PHY 和设置资源。
/// 库存、启动资源与完整永久强序设备映射须覆盖本次用户、网络和文件系统的全部寿命。
#[cfg(not(feature = "ktest"))]
unsafe fn init_filesystems(
    storage: &storage::Storage,
    inventory: &drivers::DeviceInventory,
    selected: fs::system::BootDevices<'_>,
) -> Result<task::process::ShutdownOperation, task::ProgramError> {
    const INODE_CACHE_CAPACITY: usize = 64;

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
    let net_owner = if drivers::net_device::NetDevice::has_candidate(inventory) {
        // SAFETY: 调用者传递同次库存、启动资源、网络及板设置独占和完整永久映射义务。
        // 原 NetStackOwner 在发表前保活 NIC，用户全部 join/reap 后才同步停止；失败不改用 lo。
        unsafe { net::NetStack::try_new_nic(inventory) }?
    } else {
        net::NetStack::try_new_lo()?
    };
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
    // 关闭准入已随关机请求提交；此后只回收残余组，合法信号退出状态原样保留。
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

/// 只记录启用 MMIO 槽、实际 block 候选与 PCI 功能资源，不触碰设备队列或读取磁盘。
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

/// 在不可继续的内核错误中记录真实 panic 信息并停止当前核。
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

/// 记录不可恢复的标准堆分配错误，复用不分配内存的有界应急 panic 输出。
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
