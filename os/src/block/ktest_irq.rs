//! 双 VirtIO 块设备与串口（UART）并发外部硬中断及共享 INTx 中断链测试模块。
//!
//! # 模块职责与架构定位
//! 本模块验证块设备子系统与外部中断控制器在多设备并发中断场景下的端到端协作：
//! 1. 在 RISC-V 64 平台上，两台 VirtIO MMIO 块设备拥有各自独立的 PLIC 外部中断源；在 LoongArch64 平台上，两台 VirtIO PCI 块设备（槽位 `0x5` 与 `0x9`）共享同一条 PCI INTx 中断线。
//! 2. 测试同时驱动两台块设备的只读 `Bio` 请求与控制台 UART 的单字节硬件接收中断，验证中断门控关闭期间三者同时挂起（pending）、门控开放后经外部中断控制器统一派发完成、共享中断线上无关设备 ISR 返回 `false`（空中断）时不截断后续设备派发，以及中途停机注销其中一台块设备的 ISR 后另一台块设备与 UART 仍能正常响应外部中断。
//!
//! # 测试场景与中断拓扑示意图
//! ```text
//!   [RISC-V 64: 独立 PLIC 中断源]          [LoongArch64: 共享 PCI INTx 中断线]
//!     Disk 0 (MMIO) ---> IRQ S0              Disk 0 (PCI) ---+
//!     Disk 1 (MMIO) ---> IRQ S1                              +---> 同一 IRQ S0 (共享 action 链)
//!     UART          ---> IRQ Su              Disk 1 (PCI) ---+
//!                                            UART            ---> 独立 IRQ Su
//!
//!   阶段 1 (并发挂起与派发):
//!     禁用 Disk0/Disk1/UART token -> 提交 Read0/Read1 + 注入 UART 'I' -> 轮询确认三者硬件均已 pending
//!     -> 开放全部 token -> 验证外部中断触发完成，收回物理页 DMA 访问权。
//!
//!   阶段 2 (共享中断线空中断穿透):
//!     仅向 Disk1 提交 Read2 -> 在共享 INTx 下验证 Disk0 的 ISR 被调用且记入 `irq_empty`，Disk1 正常完成。
//!
//!   阶段 3 (邻居设备停机注销后的独立运行):
//!     调用 `Disk0::stop()` 同步注销 Disk0 的中断 action -> 向 Disk1 提交 Read3 + 注入 UART 'J'
//!     -> 验证 Disk0 的 ISR 不再被调用，而 Disk1 与 UART 继续通过外部硬中断正常完成。
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 当同时操作两台块设备的中断注册表锁时（如 [`MultiIrq::enable_disks`]），固定按 `disks[0].irq` -> `disks[1].irq` 的顺序获取自旋锁。
//! - `disk.irq` 自旋锁仅用于读取或克隆 `IrqRegistration` 门控凭据，不与 `disk.queue` 自旋锁交叉嵌套；磁盘中断处理函数（ISR）内部只获取 `disk.queue` 锁，不获取 `disk.irq` 锁。

mod uart;

use super::{Bio, BioAllocator, BioCompletion, BlkOperation, Disk, DiskRef, IoResult};
use crate::drivers::virtio::{
    VirtioDevice, VirtioIdentity, VIRTIO_PCI_BLOCK_LEGACY, VIRTIO_PCI_BLOCK_MODERN,
    VIRTIO_VENDOR_ID,
};
use crate::drivers::{self, DeviceInventory};
use crate::mm::dma::MappedPages;
use crate::workqueue::{WorkQueue, WorkQueueConfig};
use crate::{cpu, irq, mm, sched, time, timer};
use ::ktest::{Fixture, SuiteSlot, TestContext, TestError, TestResult};
use alloc::vec::Vec;
use core::sync::atomic::Ordering;
use virtio_drivers::{BufferDirection, MappedSegment};

/// 在 5 秒单调时钟期限内轮询等待条件 `ready` 成立，每次未就绪时睡眠 1 毫秒。
fn until(label: &'static str, mut ready: impl FnMut() -> bool) -> TestResult {
    let deadline = time::now()
        .checked_add(time::Duration::from_secs(5))
        .ok_or(TestError("block IRQ deadline overflow"))?;
    while !ready() {
        if deadline <= time::now() {
            return Err(TestError(label));
        }
        timer::sleep(time::Duration::from_millis(1));
    }
    Ok(())
}

/// 等待所有工作队列服务线程进入阻塞空闲态、所有其他 CPU 核处于 idle 状态，并回收全局缓存与每 CPU 页池后返回内存统计快照。
fn settled() -> Result<mm::MemoryStats, TestError> {
    crate::workqueue::testing::wait_services_blocked();
    let deadline = time::now()
        .checked_add(time::Duration::from_secs(5))
        .ok_or(TestError("block IRQ settle deadline overflow"))?;
    while !(0..cpu::count())
        .all(|id| id == cpu::current().id || cpu::get(id).idle.load(Ordering::Acquire))
    {
        if deadline <= time::now() {
            return Err(TestError("block IRQ services did not settle"));
        }
        sched::yield_now();
    }
    mm::shrink_all();
    mm::drain_all_pages();
    Ok(mm::stats())
}

/// 从设备发现清单中选取恰好两台 VirtIO 块设备（两台 MMIO 设备或两台 PCI 设备），并初始化各自的底层传输层实例。
fn devices(inventory: &DeviceInventory) -> Result<[VirtioDevice; 2], TestError> {
    let mut pci = inventory.pci.devices().iter().filter(|device| {
        device.vendor_id == VIRTIO_VENDOR_ID
            && matches!(
                device.device_id,
                VIRTIO_PCI_BLOCK_LEGACY | VIRTIO_PCI_BLOCK_MODERN
            )
    });
    let pci_count = pci.clone().count();
    if inventory.mmio_blocks.len() + pci_count != 2 {
        return Err(TestError(
            "multi-device IRQ requires exactly two attached root/extra devices",
        ));
    }

    if inventory.mmio_blocks.len() == 2 {
        // SAFETY:
        // 1. 宿主机测试环境为本用例独占挂载两台块设备，`inventory.mmio_blocks[0]` 的 MMIO 窗口映射在内核整个生命周期内有效。
        // 2. 当前处于独立测试用例初始化阶段，尚无其他驱动程序或中断处理函数持有该设备的所有权。
        let first = unsafe { VirtioDevice::prepare_mmio(&inventory.mmio_blocks[0]) }
            .map_err(|_| TestError("IRQ first MMIO preparation failed"))?;
        // SAFETY:
        // 1. `inventory.mmio_blocks[1]` 指向第二台块设备独立的 MMIO 物理地址窗口，与 `first` 的寄存器及 DMA 控制页互不重叠。
        // 2. 该设备同样仅由当前测试夹具独占初始化一次。
        let second = unsafe { VirtioDevice::prepare_mmio(&inventory.mmio_blocks[1]) }
            .map_err(|_| TestError("IRQ second MMIO preparation failed"))?;
        Ok([first, second])
    } else if pci_count == 2 {
        // SAFETY:
        // 1. 第一个 PCI 块设备 BDF 来自当前有效的 PCI 枚举清单，其 BAR 空间已完成永久映射。
        // 2. 当前测试夹具是该 PCI 函数的唯一所有者。
        let first = unsafe { VirtioDevice::prepare_pci(&inventory.pci, pci.next().unwrap()) }
            .map_err(|_| TestError("IRQ first PCI preparation failed"))?;
        // SAFETY:
        // 1. 迭代器返回的第二个 PCI 设备具有与 `first` 不同的独立 BDF 地址和 BAR 映射窗口。
        // 2. 当前测试夹具独占拥有该 PCI 设备实例。
        let second = unsafe { VirtioDevice::prepare_pci(&inventory.pci, pci.next().unwrap()) }
            .map_err(|_| TestError("IRQ second PCI preparation failed"))?;
        Ok([first, second])
    } else {
        Err(TestError(
            "mixed MMIO/PCI IRQ topology is not the selected hardware profile",
        ))
    }
}

/// 单块只读测试请求的封装，持有预构造的 `Bio`、完成通知句柄以及用于校验的原始物理页 DMA 映射段信息。
struct Request {
    /// 待提交的只读块请求；提交给磁盘后被 `take()` 置为 `None`。
    bio: Option<Bio>,
    /// 与 `bio` 关联的完成跟踪句柄，用于等待 I/O 完成并收回物理页。
    completion: BioCompletion,
    /// 请求提交前记录的物理页 DMA 映射段信息（地址、长度、传输方向），用于完成后核对归还的物理页。
    original: MappedSegment,
    /// 本次读取的字节长度（等于目标磁盘的 1 个逻辑块大小）。
    unit: usize,
}
impl Request {
    /// 为指定磁盘预分配 1 个清零物理页并构造读取第 0 扇区（1 个逻辑块）的 `Bio` 请求。
    fn new(disk: &Disk) -> Result<Self, TestError> {
        let unit = disk.limits().logical_block_size as usize;
        if mm::PAGE_SIZE < unit {
            return Err(TestError("IRQ logical block exceeds original page"));
        }

        let pages = mm::alloc_pages(0, true)
            .ok_or(TestError("IRQ original Read page allocation failed"))?;
        let mapped = MappedPages::new(pages, 0, unit, BufferDirection::DeviceToDriver)
            .map_err(|_| TestError("IRQ original Read page mapping failed"))?;
        let original = mapped
            .segment(crate::mm::dma::DmaDomain::qemu_coherent())
            .expect("QEMU fixture mapping must be reachable");
        let mut memory = Vec::new_in(BioAllocator::Heap);
        memory
            .try_reserve_exact(1)
            .map_err(|_| TestError("IRQ Read owner storage allocation failed"))?;
        memory.push(mapped);
        // SAFETY:
        // 1. 物理页刚刚通过 `mm::alloc_pages(0, true)` 分配并清零，且唯一所有权已移交至 `memory` 向量中的 `MappedPages`，不存在任何 CPU 侧裸指针别名。
        // 2. 操作类型为无回调的只读请求（`BlkOperation::Read`），测试夹具会一直等待该请求完成或设备停机复位后才收回物理页。
        let bio = unsafe { Bio::try_new(disk, BlkOperation::Read, 0, memory, 0, None) }
            .map_err(|_| TestError("IRQ Read Bio preparation failed"))?;
        let completion = bio.completion();
        Ok(Self {
            bio: Some(bio),
            completion,
            original,
            unit,
        })
    }

    /// 等待该只读请求完成，校验完成字节数，从 `BioCompletion` 收回物理页的 DMA 访问权并核对 DMA 地址、长度、方向及单次领取约束。
    fn reclaim(&self, context: &mut TestContext<'_>) -> TestResult {
        until("IRQ Read did not complete", || {
            self.completion.is_complete()
        })?;
        context.expect_eq(
            self.completion.result(),
            Some(IoResult {
                completed: self.unit,
                error: None,
            }),
        );
        let memory = self
            .completion
            .take_memory()
            .map_err(|_| TestError("IRQ completed original page not reclaimable"))?;
        if memory.len() != 1 {
            return Err(TestError("IRQ original page owner count changed"));
        }
        let mapped = &memory[0];
        let segment = mapped
            .segment(crate::mm::dma::DmaDomain::qemu_coherent())
            .expect("QEMU fixture mapping must be reachable");
        context.expect_eq(
            (segment.address, segment.length, segment.direction),
            (
                self.original.address,
                self.original.length,
                self.original.direction,
            ),
        );
        context.expect_eq(mapped.is_device_owned(), false);
        context.expect_eq(
            self.completion.take_memory().err(),
            Some(super::BlockError::Busy),
        );
        drop(memory);
        Ok(())
    }
}

/// 双块设备与串口并发外部中断测试夹具（Fixture）。
struct MultiIrq {
    /// 测试开始前的六域物理内存统计基线，用于在 `tear_down` 时校验无内存泄漏。
    baseline: mm::MemoryStats,
    /// 测试开始前的工作队列元数据统计基线。
    metadata: crate::workqueue::testing::MetadataStats,
    /// 硬件设备发现清单，保存在夹具中以维持底层资源生命周期。
    _inventory: DeviceInventory,
    /// 两台被测 VirtIO 块设备的强引用句柄。
    disks: [DiskRef; 2],
    /// 分别为两台块设备提供完成回调执行上下文的独立工作队列。
    queues: [WorkQueue; 2],
    /// 两台块设备各自对应的外部中断源编号（LoongArch64 PCI INTx 下两者相同，RISC-V PLIC 下两者不同）。
    sources: [irq::IrqNumber; 2],
    /// 两台块设备的硬件身份标识（用于诊断输出与 Legacy MMIO 判别）。
    identities: [VirtioIdentity; 2],
    /// 标记每台块设备是否为 Legacy（v1）VirtIO MMIO 设备，用于计算 Virtqueue `used` 环偏移。
    legacy: [bool; 2],
    /// 测试设备注册中断前，两个中断源上已有的中断处理函数（action）注册数量基线。
    actions: [usize; 2],
    /// 预先分配并构造好的 4 个只读块请求（`requests[0]` 发往 `disks[0]`，`requests[1..4]` 发往 `disks[1]`）。
    requests: [Request; 4],
    /// 接管系统控制台串口中断的测试驱动器。
    uart: uart::Uart,
    /// 标记在 `set_up` 阶段调用 `Disk::start()` 是否发生错误（若出错则保留夹具以执行安全停机清理）。
    activation_error: bool,
}
impl Fixture for MultiIrq {
    /// 初始化双块设备与 UART 中断测试夹具：发现设备、校验平台中断拓扑、创建工作队列与 `Disk` 实例、预分配 4 个只读请求并启动块设备。
    fn set_up(context: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        // Step 1: 记录测试前的物理内存与工作队列元数据基线，并探测平台上的两台 VirtIO 块设备。
        let baseline = settled()?;
        let metadata = crate::workqueue::testing::metadata_stats();
        let inventory =
            drivers::discover().map_err(|_| TestError("IRQ inventory discovery failed"))?;
        let devices = devices(&inventory)?;
        let sources = [devices[0].interrupt(), devices[1].interrupt()];
        let identities = [devices[0].identity(), devices[1].identity()];
        let legacy =
            identities.map(|identity| matches!(identity, VirtioIdentity::Mmio { version: 1, .. }));
        let actions = [
            irq::test_action_count(sources[0]),
            irq::test_action_count(sources[1]),
        ];
        // Step 2: 校验架构中断拓扑约束（LoongArch64 的两台 PCI 设备必须共享同一条 INTx，RISC-V 的两台 MMIO 设备必须使用不同的 PLIC 中断源）。
        if cfg!(target_arch = "loongarch64") && sources[0] != sources[1] {
            return Err(TestError(
                "LA two real PCI devices do not share INTx; require slots 0x5/0x9",
            ));
        }
        if cfg!(target_arch = "riscv64") && sources[0] == sources[1] {
            return Err(TestError(
                "RV two real MMIO devices must have distinct PLIC sources",
            ));
        }

        // Step 3: 为两台块设备分别创建独立的完成工作队列与 `Disk` 实例，任一步骤失败时按逆序同步关闭已创建的队列。
        let queue0 = WorkQueue::try_new(WorkQueueConfig {
            max_active: usize::from(devices[0].info().tag_count) + 1,
            ordered: false,
            reclaim: true,
        })
        .map_err(|_| TestError("IRQ first completion domain preparation failed"))?;
        let queue1 = match WorkQueue::try_new(WorkQueueConfig {
            max_active: usize::from(devices[1].info().tag_count) + 1,
            ordered: false,
            reclaim: true,
        }) {
            Ok(queue) => queue,
            Err(_) => {
                queue0.close_sync();
                return Err(TestError("IRQ second completion domain preparation failed"));
            }
        };
        let queues = [queue0, queue1];
        let mut prepared = core::iter::IntoIterator::into_iter(devices);
        let disk0 = match Disk::prepare(prepared.next().unwrap(), &queues[0]) {
            Ok(disk) => disk,
            Err(_) => {
                for queue in &queues {
                    queue.close_sync();
                }
                return Err(TestError("IRQ first Disk preparation failed"));
            }
        };
        let disk1 = match Disk::prepare(prepared.next().unwrap(), &queues[1]) {
            Ok(disk) => disk,
            Err(_) => {
                drop(disk0);
                for queue in &queues {
                    queue.close_sync();
                }
                return Err(TestError("IRQ second Disk preparation failed"));
            }
        };
        let disks = [disk0, disk1];

        // Step 4: 在设备启动前预先分配全部 4 个只读 `Request` 并接管 UART 中断，最后依次启动两台块设备。
        let requests = (|| {
            Ok([
                Request::new(&disks[0])?,
                Request::new(&disks[1])?,
                Request::new(&disks[1])?,
                Request::new(&disks[1])?,
            ])
        })();
        let requests = match requests {
            Ok(requests) => requests,
            Err(error) => {
                drop(disks);
                for queue in &queues {
                    queue.close_sync();
                }
                return Err(error);
            }
        };
        let uart = match uart::Uart::prepare() {
            Ok(uart) => uart,
            Err(error) => {
                drop(disks);
                for queue in &queues {
                    queue.close_sync();
                }
                return Err(error);
            }
        };

        let mut fixture = Self {
            baseline,
            metadata,
            _inventory: inventory,
            disks,
            queues,
            sources,
            identities,
            legacy,
            actions,
            requests,
            uart,
            activation_error: false,
        };
        for disk in &fixture.disks {
            if let Err(error) = disk.start() {
                fixture.activation_error = true;
                context.diagnostic(format_args!(
                    "multi-device IRQ activation failed: {:?}; retaining original owner",
                    error
                ));
                break;
            }
        }
        Ok(fixture)
    }

    /// 停机清理阶段：重新开放块设备中断门控，依次停止两台块设备的 DMA 与中断注册，恢复默认 UART 中断，并同步关闭两个工作队列。
    fn quiesce(&mut self, _: &mut TestContext<'_>) -> TestResult {
        self.enable_disks()?;
        for disk in &self.disks {
            disk.stop().map_err(|_| {
                TestError("multi-device IRQ stop unconfirmed; retaining original DMA/domain")
            })?;
        }

        self.uart.quiesce()?;
        for queue in &self.queues {
            queue.close_sync();
        }
        Ok(())
    }

    /// 销毁测试夹具并校验中断 action 注册数量、全局六域物理页统计以及工作队列元数据统计均已完全恢复至测试前基线。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        let baseline = self.baseline;
        let metadata = self.metadata;
        let sources = self.sources;
        let actions = self.actions;
        drop(self);
        for index in 0..2 {
            context.expect_eq(irq::test_action_count(sources[index]), actions[index]);
        }
        context.expect_eq(settled()?, baseline);
        context.expect_eq(crate::workqueue::testing::metadata_stats(), metadata);
        Ok(())
    }
}

/// 禁用指定块设备的外部中断门控（等待正在执行的 handler 退出），阻止外部中断控制器向该设备的 ISR 派发中断。
fn disable(disk: &Disk) {
    if let Some(token) = disk.irq.lock().as_ref() {
        token.disable_sync();
    }
}
/// 重新开放指定块设备的外部中断门控；若设备已停机并注销了中断 token，则不执行任何操作。
fn enable(disk: &Disk) {
    if let Some(token) = disk.irq.lock().as_ref() {
        token.enable();
    }
}

impl MultiIrq {
    /// 开放两台块设备的中断门控：若两台设备共享同一条 INTx 中断线，则通过 `irq::testing::enable_pair` 原子地同时开放两个注册项并解除中断线屏蔽，防止出现只有一个设备的 handler 被开放的竞争窗口。
    fn enable_disks(&self) -> TestResult {
        if self.sources[0] == self.sources[1] {
            let first = self.disks[0].irq.lock();
            let second = self.disks[1].irq.lock();
            match (first.as_ref(), second.as_ref()) {
                (Some(first), Some(second)) => {
                    if !irq::testing::enable_pair(first, second) {
                        return Err(TestError(
                            "shared INTx original actions cannot be enabled together",
                        ));
                    }
                }
                (Some(token), None) | (None, Some(token)) => token.enable(),
                (None, None) => {}
            }
        } else {
            for disk in &self.disks {
                enable(disk);
            }
        }
        Ok(())
    }
    /// 将预先构造的第 `request` 号只读 `Bio` 请求提交给第 `disk` 台块设备。
    fn submit(&mut self, request: usize, disk: usize) {
        self.disks[disk].submit_bio(
            self.requests[request]
                .bio
                .take()
                .expect("IRQ request submitted twice"),
        );
    }

    /// 在中断门控关闭期间，轮询等待两台块设备的 Virtqueue 硬件 `used.idx` 均达到 1 且 UART 接收 FIFO 已有待读字节，同时断言软件侧尚未处理任何完成中断。
    fn pending(&self, context: &mut TestContext<'_>) -> TestResult {
        until(
            "two hardware used rings and UART FIFO never became pending together",
            || {
                super::tests::used_index(&self.disks[0], self.legacy[0]) == 1
                    && super::tests::used_index(&self.disks[1], self.legacy[1]) == 1
                    && crate::console::receive_pending()
            },
        )?;
        for disk in &self.disks {
            context.expect_eq(disk.test_snapshot().irq_queue, 0);
            context.expect_eq(disk.test_snapshot().completed, 0);
        }
        Ok(())
    }

    /// 执行双块设备与 UART 并发外部中断、共享 INTx 空中断穿透以及单设备停机注销后的独立中断响应测试。
    fn exercise(&mut self, context: &mut TestContext<'_>) -> TestResult {
        if self.activation_error {
            return Err(TestError("multi-device IRQ activation failed"));
        }
        // Step 1: 校验 UART 中断源与块设备中断源互不相同，并校验两台块设备启动后中断源上的 action 注册数符合预期（共享中断线增加 2，独立中断源各增加 1）。
        let uart_source = self.uart.source();
        context.expect_eq(
            uart_source != self.sources[0] && uart_source != self.sources[1],
            true,
        );
        let dispatch = [
            irq::testing::dispatch_count(self.sources[0]),
            irq::testing::dispatch_count(self.sources[1]),
            irq::testing::dispatch_count(uart_source),
        ];
        let added = if self.sources[0] == self.sources[1] {
            2
        } else {
            1
        };
        for index in 0..2 {
            context.expect_eq(
                irq::test_action_count(self.sources[index]),
                self.actions[index] + added,
            );
        }

        // Step 2: 禁用两台块设备与 UART 的中断门控，同时发起两台块设备的读请求与 UART 字节 `'I'` 注入，等待三者在硬件层同时进入挂起（pending）状态。
        disable(&self.disks[0]);
        disable(&self.disks[1]);
        self.uart.begin(b'I')?;
        self.submit(0, 0);
        self.submit(1, 1);
        self.pending(context)?;

        // Step 3: 开放 UART 与两台块设备的中断门控，等待硬件中断派发完成所有 I/O，收回物理页并验证三个中断源的派发计数均已递增。
        self.uart.enable();
        self.enable_disks()?;
        self.requests[0].reclaim(context)?;
        self.requests[1].reclaim(context)?;
        self.uart.receive(context, b'I')?;
        for (index, previous) in dispatch[..2].iter().enumerate() {
            context.expect_eq(0 < self.disks[index].test_snapshot().irq_queue, true);
            context.expect_eq(
                *previous < irq::testing::dispatch_count(self.sources[index]),
                true,
            );
        }
        context.expect_eq(
            dispatch[2] < irq::testing::dispatch_count(uart_source),
            true,
        );

        // Step 4: 仅向第二台块设备（disks[1]）提交读请求；若两台设备共享同一条 INTx 中断线，验证第一台块设备（disks[0]）的 ISR 也被派发调用且正确递增空中断计数（`irq_empty`），同时不影响 disks[1] 完成请求。
        {
            let queue = self.disks[0].queue.lock();
            context.expect_eq(queue.phase, super::disk::QueuePhase::Live);
            context.expect_eq(queue.failure, None);
        }
        let empty = self.disks[0].test_snapshot().irq_empty;
        let queue_irqs = self.disks[1].test_snapshot().irq_queue;
        self.submit(2, 1);
        self.requests[2].reclaim(context)?;
        context.expect_eq(queue_irqs < self.disks[1].test_snapshot().irq_queue, true);
        if self.sources[0] == self.sources[1] {
            context.expect_eq(empty < self.disks[0].test_snapshot().irq_empty, true);
            let queue = self.disks[0].queue.lock();
            context.expect_eq(queue.phase, super::disk::QueuePhase::Live);
            context.expect_eq(queue.failure, None);
        }

        // Step 5: 停止第一台块设备（disks[0]），验证其中断 token 已注销且共享中断源上的活跃 action 数量减 1。
        self.disks[0]
            .stop()
            .map_err(|_| TestError("first shared action removal cannot safely stop"))?;
        let stopped_calls = self.disks[0].test_snapshot().irq_calls;
        context.expect_eq(self.disks[0].irq.lock().is_none(), true);
        context.expect_eq(
            irq::test_action_count(self.sources[0]),
            self.actions[0]
                + if self.sources[0] == self.sources[1] {
                    1
                } else {
                    0
                },
        );

        // Step 6: 在第一台块设备已停机注销的状态下，再次禁用 disks[1] 与 UART 门控，提交读请求并注入字节 `'J'`，等待两者同时挂起后再开放门控，验证剩余块设备与 UART 的外部中断依然正常工作且已停机的 disks[0] 不再收到任何 ISR 调用。
        let queue_irqs = self.disks[1].test_snapshot().irq_queue;
        let neighbor_dispatch = irq::testing::dispatch_count(self.sources[1]);
        disable(&self.disks[1]);
        self.uart.begin(b'J')?;
        self.submit(3, 1);
        until(
            "remaining hardware used ring and UART FIFO did not become pending",
            || {
                super::tests::used_index(&self.disks[1], self.legacy[1]) == 3
                    && crate::console::receive_pending()
            },
        )?;

        self.uart.enable();
        enable(&self.disks[1]);
        self.requests[3].reclaim(context)?;
        self.uart.receive(context, b'J')?;
        context.expect_eq(queue_irqs < self.disks[1].test_snapshot().irq_queue, true);
        context.expect_eq(
            neighbor_dispatch < irq::testing::dispatch_count(self.sources[1]),
            true,
        );
        context.expect_eq(self.disks[0].test_snapshot().irq_calls, stopped_calls);
        context.expect_eq(self.disks[1].test_snapshot().completed, 3);
        context.diagnostic(format_args!("hardware-irq identities={:?} sources={}/{} uart-source={} shared-intx={} shared-empty-isr-checked={} actual-disk-snapshots={:?}/{:?} read-only=1 root-business-identity=not-covered", self.identities, self.sources[0].source(), self.sources[1].source(), uart_source.source(), (self.sources[0] == self.sources[1]) as usize, (self.sources[0] == self.sources[1]) as usize, self.disks[0].test_snapshot(), self.disks[1].test_snapshot()));
        Ok(())
    }
}

#[::ktest::suite(fixture = MultiIrq, isolation = "Case", tags("block", "irq", "B02"), resources("root", "extra", "uart"), timeout_ms = 30000)]
mod hardware {
    use super::*;
    /// **测试目标**：验证两台 VirtIO 块设备（RISC-V 下为不同 PLIC 中断源，LoongArch64 下共享同一条 PCI INTx 中断线）与控制台 UART 在并发挂起、共享中断线空中断穿透以及动态停机注销其中一台设备后的外部中断派发正确性。
    /// **场景步骤**：
    /// 1. 门控关闭期间同时向两台磁盘提交读请求并触发 UART 接收字节 `'I'`，等待三者硬件状态同时挂起后开放门控，验证中断完成与物理页回收。
    /// 2. 单独向第二台磁盘提交读请求，验证共享 INTx 下第一台磁盘的 ISR 返回空中断而不阻断第二台磁盘完成。
    /// 3. 调用第一台磁盘的 `stop()` 同步注销其 ISR，随后并发触发第二台磁盘读请求与 UART 接收字节 `'J'`，验证剩余设备中断正常且已停机设备不再被调用。
    /// **预期结果**：所有读请求与 UART 接收均通过硬件中断准确完成，物理页 DMA 访问权正常收回，停机清理后中断 action 计数与六域内存统计完全回到基线。
    #[test]
    fn uart_two_devices_unregister_neighbor(
        context: &mut TestContext<'_>,
        fixture: &mut MultiIrq,
    ) -> TestResult {
        fixture.exercise(context)
    }
}

/// 返回多设备硬件中断测试套件的注册锚点，供上层块设备模块汇总导出。
pub(crate) fn anchors() -> [&'static SuiteSlot; 1] {
    [hardware::__ktest_anchor()]
}
