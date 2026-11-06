//! QEMU `virt` 平台 VirtIO 网络设备驱动（`VirtioNet`）。
//!
//! ## 模块职责与架构定位
//! 本模块实现基于双 Split VirtQueue（RX 接收队列与 TX 发送队列）及预分配单页缓冲池的 VirtIO 网卡驱动。
//! 上游由 [`crate::drivers::net_device::NetDevice`] 持有强引用 [`Arc<VirtioNet>`] 并调度生命周期与发送请求；
//! 下游依赖 [`crate::drivers::virtio`] 构造的 MMIO（RISC-V）或 PCI（LoongArch）传输层、[`crate::mm::dma`]
//! 提供的物理页 DMA 访问权状态机、[`crate::irq`] 外部中断注册机制以及 [`crate::workqueue::Work`] 后台工作队列。
//! 当收到以太网帧或回收发送槽位后，驱动通过 `Weak<NetStack>` 弱引用回调协议栈完成收包分发与阻塞发送者唤醒。
//!
//! ## 核心设计与关键状态机
//!
//! ### 1. 驱动生命周期状态机（`Phase`）
//! ```text
//!   prepare() ──► Prepared (已复位协商、预分配双环与缓冲池，尚未向硬件写入队列地址)
//!                    │
//!                 start()
//!                    ├──► 发布或提交初始 RX 失败 ──► Faulted (屏蔽队列中断，等待 stop_sync 复位清理)
//!                    ▼
//!                  Live  (中断已使能，允许并发 TX 提交与后台 Work 收发批处理)
//!                    │
//!                    ├──► 运行期 DMA / 队列协议异常 ──► Faulted
//!                    │
//!                stop_sync()
//!                    ▼
//!                Stopping (屏蔽队列中断 ──► close_sync 等待 Work 退出 ──► 轮询硬件 reset 归零
//!                    │     ──► unregister_sync 注销 IRQ ──► 收回所有缓冲页 DMA 访问权并释放物理页)
//!                    ▼
//!                 Stopped (硬件与后台执行流彻底静止，允许安全 Drop 最后一个 Arc)
//! ```
//!
//! ### 2. 预分配单页缓冲池与槽位状态机（`SlotState`）
//! 每个 RX / TX 槽位独占一页物理内存（[`MappedPages`]），页内前 `header_len` 字节存放 `virtio_net_hdr`，
//! 随后紧接最长 1514 字节的以太网帧，在 VirtQueue 中编码为 2 个描述符组成的链：
//! ```text
//!   ┌────────────────────┬───────────────────────────────────────────┐
//!   │ virtio_net_hdr     │ Ethernet Frame (14..=1514 Bytes)          │  (单张 4 KiB 物理页)
//!   └────────────────────┴───────────────────────────────────────────┘
//!     Descriptor 0 (头) ──► Descriptor 1 (帧载荷)
//!
//!   TX 槽位流转：Free ──(try_transmit 认领)──► Preparing (锁外 CPU 写入以太网帧；异常退出由 PreparingTx 自动归还 Free)
//!                     ──(begin_device_access + submit_tx)──► DeviceOwned (移交设备 DMA 读取)
//!                     ──(complete_tx 收到 Used 并 finish_device_access)──► Free
//!
//!   RX 槽位流转：Free ──(start / repost_rx 移交设备并 submit_rx)──► DeviceOwned (设备 DMA 写入收包数据)
//!                     ──(take_rx 收到 Used 并收回 CPU 访问权)──► Processing (Work 线程独占解析并上交协议栈)
//!                     ──(repost_rx 归还槽位)──► DeviceOwned (若设备仍 Live 且无协议错) / Free (停机或协议错)
//! ```
//!
//! ## 锁获取顺序（Lock Ordering）与并发契约
//! 本模块涉及三把锁，严格遵循以下获取顺序以消除死锁：
//! 1. `control: SleepMutex<()>`（最外层，仅在可睡眠任务上下文的 `start` 与 `stop_sync` 中获取，串行化启停状态转换）
//! 2. `registration: SpinLock<Option<IrqRegistration>>` 或 `state: SpinLock<State>`（同级内层自旋锁，**严禁嵌套同时持有**！
//!    代码中始终先释放一把自旋锁，再获取另一把自旋锁）
//! - **自旋锁临界区约束**：持有 `state` 或 `registration` 自旋锁期间严禁睡眠、动态分配内存、触发 Doorbell MMIO 通知
//!   （`notify` 通过 `notify_active` 计数移至锁外执行）或回调 `NetStack`（`receive_ethernet` / `kick_egress` 均在锁外执行）。

#[cfg(feature = "ktest")]
pub mod ktest;

use alloc::{
    sync::{Arc, Weak},
    vec::Vec,
};
use core::{convert::TryFrom, mem::ManuallyDrop, pin::Pin, slice};
use spin::Once;
use virtio_drivers::{
    device::net::{rx_frame, NetInfo, VirtIONet},
    queue::{RingLayout, VirtQueue},
    transport::{DeviceStatus, DeviceType, Notification, SomeTransport, Transport},
    BufferDirection, Error, Hal,
};

#[cfg(target_arch = "loongarch64")]
use crate::drivers::virtio::{VIRTIO_PCI_NET_LEGACY, VIRTIO_PCI_NET_MODERN, VIRTIO_VENDOR_ID};

use crate::{
    drivers::{
        virtio::{dma_protocol, qemu_dma_domain, VirtioIdentity, VirtioInitError},
        DeviceInventory, ProbeError,
    },
    irq::{self, IrqNumber, IrqRegistration},
    mm::{
        self,
        dma::{CoherentDma, DmaDomain, KernelHal, MappedPages},
    },
    net::{stats::InterfaceStats, NetStack, NetStackRef, ETHERNET_FRAME_CAPACITY},
    sync::{SleepMutex, SpinLock},
    time::{self, Duration},
    timer, trace,
    workqueue::Work,
};

/// 单个以太网帧允许的最大字节长度（14 字节以太网头 + 1500 字节 MTU = 1514 字节，不含 `virtio_net_hdr`）。
const FRAME_MAX: usize = ETHERNET_FRAME_CAPACITY;
/// 单个以太网帧允许的最小字节长度（14 字节标准以太网首部，不含 `virtio_net_hdr`）。
const FRAME_MIN: usize = smoltcp::wire::ETHERNET_HEADER_LEN;
/// 单次后台 `Work` 轮询最多连续处理的接收帧数量上限（防止高流量收包饿死同核其他任务，类似 NAPI weight）。
const RX_BUDGET: usize = 16;
/// 单次后台 `Work` 轮询最多连续回收的已发送描述符链数量上限。
const TX_BUDGET: usize = 32;
/// 每个收发以太网帧在 Split VirtQueue 中占用的描述符个数（1 个 VirtIO 头部描述符 + 1 个以太网帧载荷描述符）。
const FRAME_DESCRIPTOR_COUNT: u16 = 2;
/// RX 与 TX 各自的预分配单页缓冲池最大槽位数上限（最多 32 个槽位，即每方向最多 32 张 4 KiB 物理页）。
const POOL_MAX_SLOTS: usize = 32;

/// 绑定内核 `KernelHal` 内存屏障与静态擦除传输层的 `virtio-drivers` 网卡驱动类型别名。
type Driver = VirtIONet<KernelHal, SomeTransport<'static>>;

/// VirtIO 网卡驱动的运行阶段状态机（受 `VirtioNet::state` 自旋锁保护）。
#[derive(Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// 已完成复位、特性协商与 DMA 队列/缓冲池预分配，但尚未向硬件写入队列地址或启用中断。
    Prepared,
    /// 队列已向硬件发布且中断已启用，允许正常的并发帧发送与后台收发轮询。
    Live,
    /// 启动发布或运行期间检测到 DMA 或 VirtIO 队列协议违规，已关闭队列中断并拒绝新操作，等待 `stop_sync` 复位回收。
    Faulted,
    /// 正在执行 `stop_sync` 同步停机流程（排空后台 `Work`、复位硬件并注销中断）。
    Stopping,
    /// 硬件已确认复位、中断已注销且全部 DMA 缓冲页与控制环物理页均已归还，可安全析构。
    Stopped,
}

/// 收发缓冲池中单个固定槽位的所有权状态（受 `VirtioNet::state` 自旋锁保护）。
#[derive(Clone, Copy, PartialEq, Eq)]
enum SlotState {
    /// 槽位空闲，物理页保存在 `Slot::page` 中且归 CPU 所有，可被新的发送请求或接收投递认领。
    Free,
    /// 发送路径已认领该槽位并将物理页临时移交给栈上 [`PreparingTx`]，正在锁外填充待发送帧数据。
    Preparing,
    /// 物理页保存在 `Slot::page` 中，但 DMA 访问权已移交给网卡硬件（已提交至 VirtQueue 且尚未从 Used 环收回）。
    DeviceOwned,
    /// 接收路径已从 Used 环收回完成项并解除 DMA 访问，物理页临时移交给后台 `Work` 在锁外解析并上送协议栈。
    Processing,
}

/// 预分配收发缓冲池中的单个槽位元数据（存放在 `Inner::rx` 或 `Inner::tx` 中，受 `VirtioNet::state` 自旋锁保护）。
struct Slot {
    /// 当前槽位所处的所有权流转状态。
    state: SlotState,
    /// 槽位绑定的单张 4 KiB 物理页封装；仅在 `Preparing`（TX 锁外写帧）或 `Processing`（RX 锁外收包）期间被短暂 `take` 为 `None`。
    page: Option<MappedPages>,
    /// 当 `state == SlotState::DeviceOwned` 时，记录本次提交在 VirtQueue 中分配的描述符链头部索引（`0..depth`）。
    head: Option<u16>,
    /// 发送槽位记录本次提交的以太网帧字节长度（不含 `virtio_net_hdr`），用于在 TX 完成时统计发送字节数；接收槽位恒为 `0`。
    frame_len: usize,
}

/// 网卡底层协议驱动、一致性队列环物理页与收发缓冲池的聚合拥有者。
///
/// 字段析构顺序由 [`Drop`] 手工控制：必须先销毁持有裸地址视图的 `driver`，再释放 `rx`/`tx` 载荷缓冲页，
/// 最后释放 `tx_ring`/`rx_ring` 队列控制页。外层再包裹一层 [`ManuallyDrop`]，以便在硬件未确认复位时阻止自动析构。
struct Inner {
    /// `virtio-drivers` 网卡协议状态机，内部借用 `rx_ring` 与 `tx_ring` 的虚拟/DMA 地址。
    driver: ManuallyDrop<Driver>,
    /// 当前网卡所属的平台 DMA 地址转换与缓存同步域（QEMU `virt` 恒等一致性域）。
    domain: DmaDomain,
    /// 接收队列（RX Split VirtQueue）的描述符表、Available 环与 Used 环连续物理页。
    rx_ring: ManuallyDrop<CoherentDma>,
    /// 发送队列（TX Split VirtQueue）的描述符表、Available 环与 Used 环连续物理页。
    tx_ring: ManuallyDrop<CoherentDma>,
    /// 固定容量的接收帧物理页槽位池（最多 [`POOL_MAX_SLOTS`] 项）。
    rx: Vec<Slot>,
    /// 固定容量的发送帧物理页槽位池（最多 [`POOL_MAX_SLOTS`] 项）。
    tx: Vec<Slot>,
    /// RX VirtQueue 描述符头索引（`head`）到 `rx` 槽位下标的反查表（长度等于 `rx_depth`），用于校验并定位设备返回的 Used 项。
    rx_heads: Vec<Option<usize>>,
    /// TX VirtQueue 描述符头索引（`head`）到 `tx` 槽位下标的反查表（长度等于 `tx_depth`），用于校验并定位设备返回的 Used 项。
    tx_heads: Vec<Option<usize>>,
}

impl Drop for Inner {
    /// 按“协议驱动视图 -> 收发数据缓冲页 -> 队列控制环物理页”的安全顺序释放全部底层资源。
    ///
    /// 仅允许在队列从未发布，或 `stop_sync` 已确认硬件复位完成并收回全部缓冲页 DMA 访问权后调用。
    fn drop(&mut self) {
        assert!(
            self.rx.iter().chain(self.tx.iter()).all(|slot| {
                slot.page
                    .as_ref()
                    .is_some_and(|page| !page.is_device_owned())
            }),
            "dropping active NIC payloads"
        );
        // SAFETY:
        // 1. 外层仅在尚未向硬件发布队列，或 `stop_sync` 已排空全部发送者、`Work`、IRQ 并确认硬件 `status == 0` 后才析构 `Inner`。
        // 2. 此时没有任何并发执行流访问 `driver`，且底层 `rx_ring`/`tx_ring` 尚未释放，销毁 `driver` 视图完全安全。
        unsafe { ManuallyDrop::drop(&mut self.driver) };
        self.rx.clear();
        self.tx.clear();
        // SAFETY:
        // 1. 上方 `driver` 已完成析构，不再存在指向 `tx_ring` 或 `rx_ring` 的 CPU 侧裸指针视图，且硬件已停止访问。
        // 2. 两个 `ManuallyDrop<CoherentDma>` 字段在此处各执行一次析构，归还底层伙伴系统物理页。
        unsafe {
            ManuallyDrop::drop(&mut self.tx_ring);
            ManuallyDrop::drop(&mut self.rx_ring);
        }
    }
}

/// 受 `VirtioNet::state` 自旋锁保护的可变运行状态与统计信息。
struct State {
    /// 网卡当前的生命周期阶段。
    phase: Phase,
    /// 底层驱动与全部 DMA 物理页拥有者；仅在 `stop_sync` 完成资源回收或未发布直接 `Drop` 时被 `take` 取出销毁。
    inner: Option<ManuallyDrop<Inner>>,
    /// 硬件是否可能已知晓队列 DMA 地址（`start` 开始发布队列前即置为 `true`，确认硬件复位归零后清为 `false`）。
    must_reset: bool,
    /// 后台收发 `Work` 是否已执行过 `close_sync`（保证多次调用 `stop_sync` 时的幂等性）。
    progress_closed: bool,
    /// 当前正在自旋锁外执行硬件 Doorbell 寄存器写入的线程计数；`stop_sync` 复位硬件前断言该计数必须为 `0`。
    notify_active: usize,
    /// 首次记录到的队列或 DMA 协议错误原因，供后续拒绝请求时返回准确错误码。
    failure: Option<Error>,
    /// 接口累计收发包数、字节数、错包数与丢包数统计。
    statistics: InterfaceStats,
}

/// VirtIO 网络设备驱动主体结构。
///
/// 上层协议栈（`NetStack` -> `NetDevice`）持有稳定的强引用 `Arc<VirtioNet>`；注册到中断子系统与工作队列的上下文
/// 均借用该 `Arc` 的原始地址，而回指协议栈的 `net` 字段采用 [`Weak`] 弱引用以打破引用环。
/// 一旦调用过 [`Self::start`]，最后一个强引用必须在 [`Self::stop_sync`] 成功返回后才能释放。
pub struct VirtioNet {
    /// 保护运行阶段、VirtQueue、缓冲池槽位状态与接口统计的自旋锁（临界区极短、不可睡眠）。
    state: SpinLock<State>,
    /// 串行化 [`Self::start`] 与 [`Self::stop_sync`] 生命周期控制操作的睡眠互斥锁。
    control: SleepMutex<()>,
    /// 保存外部中断注册句柄的自旋锁（与 `state` 同级，严禁嵌套持锁）。
    registration: SpinLock<Option<IrqRegistration>>,
    /// 绑定在 CPU 0 网络工作队列上的单实例收发批处理 `Work`，在 `prepare` 阶段一次性完成初始化。
    progress: Once<Pin<Arc<Work>>>,
    /// 指向所属网络协议栈根对象的弱引用，用于上送接收帧与唤醒等待发送槽位的任务。
    net: Weak<NetStack>,
    /// 特性协商阶段确定的不可变设备参数（6 字节 MAC 地址、`virtio_net_hdr` 长度、RX/TX 队列深度）。
    info: NetInfo,
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "ktest 校验 NIC 与库存的 MMIO 或 PCI 身份，生产路径不再读取该快照"
        )
    )]
    /// 设备在总线上的物理身份标识（MMIO 基址或 PCI BDF）。
    identity: VirtioIdentity,
    /// 设备绑定的外部中断号。
    interrupt: IrqNumber,
    /// 内核自测（`ktest`）专用的运行指标与同步检查点观测器。
    #[cfg(feature = "ktest")]
    observation: ktest::Observation,
}

impl VirtioNet {
    /// 在可睡眠的普通任务上下文中探测唯一 VirtIO 网卡，完成复位、特性协商、DMA 队列/缓冲池预分配及后台 `Work` 创建。
    ///
    /// 返回时网卡处于 `Phase::Prepared` 状态，尚未注册中断或向硬件发布任何队列地址。
    /// - `net`：所属协议栈强引用（内部降级为 `Weak` 保存，并借用其工作队列创建 `Work`）。
    /// - `inventory`：启动枚举生成的硬件资源库存。
    /// - `cpu`：运行收发批处理 `Work` 的目标 CPU 编号（当前固定要求为 CPU 0）。
    ///
    /// # Safety
    /// 1. `inventory` 必须来自真实的单次硬件枚举，其中的网卡寄存器窗口已完成永久、强序（Uncached Device）映射，且调用方独占该硬件。
    /// 2. 调用方在调用 [`Self::start`] 前必须将返回的 `Arc<Self>` 存入协议栈根结构持久保活，直至所有外部发送者退出且 [`Self::stop_sync`] 成功完成。
    pub unsafe fn prepare(
        net: &NetStackRef,
        inventory: &DeviceInventory,
        cpu: usize,
    ) -> Result<Arc<Self>, VirtioInitError> {
        // Step 1: 确认当前处于可调度任务上下文并固定绑定 CPU 0，然后从库存中选取唯一 VirtIO 网卡传输层。
        crate::sched::assert_switchable();
        assert_eq!(cpu, 0, "NIC progress must use CPU0");
        let domain = qemu_dma_domain()?;
        // SAFETY: 调用方契约保证了 `inventory` 来自真实硬件枚举、寄存器窗口永久强序映射有效且该网卡由当前实例独占。
        let (mut transport, identity, interrupt) = unsafe { select_transport(inventory) }?;
        if transport.device_type() != DeviceType::Network {
            return Err(ProbeError::Resource("selected VirtIO device is not network").into());
        }

        // Step 2: 同步复位硬件并插入全内存屏障，随后执行特性协商并分配 RX/TX 队列环与单页缓冲池。
        transport.request_reset();
        if !transport.reset_complete() {
            return Err(Error::ResetPending.into());
        }
        KernelHal::dma_barrier();

        let info = Driver::negotiate(&mut transport)?;
        let status = transport.get_status();
        let inner = prepare_inner(transport, info, domain)?;
        // Step 3: 构造 `Arc<VirtioNet>` 并将其实例裸指针绑定到网络工作队列的后台 `Work` 上。
        let nic = Arc::try_new(Self {
            state: SpinLock::new(State {
                phase: Phase::Prepared,
                inner: Some(ManuallyDrop::new(inner)),
                must_reset: false,
                progress_closed: false,
                notify_active: 0,
                failure: None,
                statistics: InterfaceStats::default(),
            }),
            control: SleepMutex::new(()),
            registration: SpinLock::new(None),
            progress: Once::new(),
            net: Arc::downgrade(net),
            info,
            identity,
            interrupt,
            #[cfg(feature = "ktest")]
            observation: ktest::Observation::new(),
        })
        .map_err(|_| VirtioInitError::NoMemory { status })?;

        let work = Work::try_new(
            net.work_queue(),
            cpu,
            progress_callback,
            Arc::as_ptr(&nic) as usize,
        )
        .map_err(|_| VirtioInitError::NoMemory { status })?;
        nic.progress.call_once(|| work);
        Ok(nic)
    }

    /// 注册中断处理函数、向硬件发布 RX/TX 队列、预投递全部接收缓冲页并启用中断，使网卡进入 `Phase::Live`。
    ///
    /// 必须在可睡眠的普通任务上下文调用，且上层协议栈已持有本 `Arc<Self>`。
    /// 一旦开始尝试向硬件发布队列（`must_reset = true`），若中途发生错误，DMA 内存与中断注册将保留在实例内，
    /// 调用方必须在停止外部生产者后调用 [`Self::stop_sync`] 执行复位清理。
    pub fn start(self: &Arc<Self>) -> Result<(), VirtioInitError> {
        let _control = self.control.lock();
        if self.state.lock().phase != Phase::Prepared {
            return Err(Error::AlreadyUsed.into());
        }

        // Step 1: 先以未使能（disabled）状态注册共享/独占中断处理函数，确保队列就绪前不会提前触发回调。
        // SAFETY:
        // 1. `Arc::as_ptr(self)` 指向由上层 `NetStack` 强引用持久保活的 `VirtioNet` 实例，直到 `stop_sync` 调用 `unregister_sync` 返回后才会释放。
        // 2. `irq_callback` 内部仅获取短临界区自旋锁确认中断并投递预分配的 `Work`，不会睡眠或动态分配内存。
        let registration =
            unsafe { irq::register(self.interrupt, irq_callback, Arc::as_ptr(self) as *mut ()) }
                .map_err(ProbeError::Irq)?;
        *self.registration.lock() = Some(registration);

        // Step 2: 在队列锁内标记复位义务，向硬件发布双队列并将所有预分配 RX 缓冲页提交到接收环。
        let result = {
            let mut state = self.state.lock();
            state.must_reset = true;
            let inner: &mut Inner = state.inner.as_mut().expect("prepared NIC has no DMA owner");
            let result = publish_rx(inner);
            match result {
                Ok(pending) => {
                    state.phase = Phase::Live;
                    Ok(pending)
                }
                Err(error) => {
                    state.failure = Some(error);
                    state.phase = Phase::Faulted;
                    state.inner.as_mut().unwrap().driver.set_interrupts(false);
                    Err(VirtioInitError::Protocol(error))
                }
            }
        };

        // Step 3: 使能外部中断线，敲响 RX Doorbell 通知设备接收缓冲已就绪；若发布瞬间已有包到达则立即调度后台 `Work`。
        let pending = result?;
        self.registration.lock().as_ref().unwrap().enable();
        self.notify(true);
        if pending {
            self.queue_progress();
        }
        Ok(())
    }

    /// 返回特性协商确定的网卡静态参数（MAC 地址、`virtio_net_hdr` 字节长度及收发队列深度）。
    pub fn info(&self) -> NetInfo {
        self.info
    }

    /// 返回设备枚举阶段记录的总线物理身份（MMIO 基址或 PCI BDF），供 `ktest` 核对设备绑定关系。
    #[cfg(feature = "ktest")]
    pub fn identity(&self) -> VirtioIdentity {
        self.identity
    }

    /// 返回本网卡注册的外部中断号，供 `ktest` 验证中断路由与共享中断行为。
    #[cfg(feature = "ktest")]
    pub fn interrupt(&self) -> IrqNumber {
        self.interrupt
    }

    /// 在队列自旋锁临界区内复制并返回当前的接口收发统计快照。
    pub fn statistics(&self) -> InterfaceStats {
        self.state.lock().statistics
    }

    /// 非阻塞尝试发送一帧以太网数据：锁内认领空闲槽位 -> 锁外编码帧内容 -> 锁内移交 DMA 访问权并提交 TX 队列 -> 锁外触发 Doorbell。
    ///
    /// - `frame_len`：以太网帧长度（必须在 `14..=1514` 字节之间，不含前导 `virtio_net_hdr`）。
    /// - `write_frame`：在自旋锁外执行的帧编码闭包，严禁睡眠、动态分配内存、访问用户空间或重入网络栈。
    /// - 返回值：`Ok(true)` 表示已提交硬件发送队列；`Ok(false)` 表示所有预分配发送槽位均繁忙（不会调用 `write_frame`）；
    ///   `Err(...)` 表示帧长越界或网卡已脱离 `Live` 状态。
    pub fn try_transmit(
        &self,
        frame_len: usize,
        write_frame: impl FnOnce(&mut [u8]),
    ) -> Result<bool, VirtioInitError> {
        if !(FRAME_MIN..=FRAME_MAX).contains(&frame_len) {
            return Err(Error::InvalidParam.into());
        }
        // Step 1: 在队列自旋锁内认领一个 `Free` 发送槽位并取出其物理页，标记为 `Preparing` 后释放锁。
        let (slot, page) = {
            let mut state = self.state.lock();
            if state.phase != Phase::Live {
                return Err(state.failure.unwrap_or(Error::DeviceNeedsReset).into());
            }
            let inner: &mut Inner = state.inner.as_mut().unwrap();
            let Some(index) = inner
                .tx
                .iter()
                .position(|slot| slot.state == SlotState::Free)
            else {
                return Ok(false);
            };
            let slot = &mut inner.tx[index];
            slot.state = SlotState::Preparing;
            (index, slot.page.take().expect("free TX slot has no page"))
        };

        // Step 2: 用 `PreparingTx` 守卫在栈上持有物理页（异常提前返回时自动归还槽位），在自旋锁外调用 `write_frame` 填充以太网帧载荷。
        let mut prepared = PreparingTx {
            nic: self,
            slot,
            page: Some(page),
        };
        {
            let bytes = prepared.page.as_mut().unwrap().bytes_mut();
            // SAFETY:
            // 1. 缓冲池中的每张物理页在 `prepare_pool` 中均由 `alloc_pages(0, true)` 完整清零初始化，不存在未初始化字节（`MaybeUninit` 可安全转为 `u8`）。
            // 2. 当前物理页已从槽位 `take` 到栈局部变量 `prepared` 中且处于 CPU 拥有状态，没有任何并发 CPU 线程或设备 DMA 别名。
            // 3. 可变切片借用在本代码块结束时立即终止，随后才会在 Step 3 中将物理页移交设备 DMA。
            let bytes =
                unsafe { slice::from_raw_parts_mut(bytes.as_mut_ptr().cast::<u8>(), bytes.len()) };
            write_frame(&mut bytes[self.info.header_len..self.info.header_len + frame_len]);
        }

        // Step 3: 重新获取队列自旋锁，将物理页放回槽位、移交设备 DMA 访问权（ToDevice）、提交 TX VirtQueue 并登记 `tx_heads` 映射。
        let result = {
            let mut state = self.state.lock();
            if state.phase != Phase::Live {
                return Err(state.failure.unwrap_or(Error::DeviceNeedsReset).into());
            }
            let inner: &mut Inner = state.inner.as_mut().unwrap();
            assert!(
                usize::from(FRAME_DESCRIPTOR_COUNT) <= inner.driver.available_tx_descriptors(),
                "TX preparation lost reserved descriptors"
            );
            let domain = inner.domain;
            let segment = {
                let slot = &mut inner.tx[prepared.slot];
                assert!(slot.state == SlotState::Preparing && slot.page.is_none());
                slot.page = prepared.page.take();
                slot.frame_len = frame_len;
                slot.state = SlotState::Free;
                let page = slot.page.as_mut().unwrap();
                match page.segment(domain).map_err(dma_protocol) {
                    Ok(segment) => {
                        // SAFETY:
                        // 1. Step 2 中 `write_frame` 对帧数据的 CPU 可变借用已结束，且页首 `header_len` 字节自初始清零后从未被设备改写、始终保持全零。
                        // 2. 物理页已放回 `inner.tx[prepared.slot]` 并受 `state` 自旋锁保护，在设备返回对应 TX Used 项或 `stop_sync` 确认硬件复位前绝不会释放。
                        match unsafe { page.begin_device_access(domain) }.map_err(dma_protocol) {
                            Ok(()) => {
                                slot.state = SlotState::DeviceOwned;
                                Ok(segment)
                            }
                            Err(error) => Err(error),
                        }
                    }
                    Err(error) => Err(error),
                }
            };
            let result = match segment {
                Ok(segment) => {
                    // SAFETY:
                    // 1. `segment` 来自方向为 `DriverToDevice` 的已同步 `MappedPages`，映射长度覆盖 `header_len + FRAME_MAX`，足以容纳本次 `frame_len`。
                    // 2. 该物理页由 `inner.tx[prepared.slot]` 在整个设备访问期内保活，且持有 `state` 自旋锁保证了在 `install_head` 完成前不会与 `complete_tx` 发生竞态。
                    let head = unsafe { inner.driver.submit_tx(segment, frame_len) };
                    match head {
                        Ok(head) => {
                            install_head(&mut inner.tx, &mut inner.tx_heads, prepared.slot, head)
                        }
                        Err(error) => Err(error),
                    }
                }
                Err(error) => Err(error),
            };
            if let Err(error) = result {
                state.failure = Some(error);
                state.phase = Phase::Faulted;
                state.inner.as_mut().unwrap().driver.set_interrupts(false);
            }
            result
        };
        // Step 4: 在自旋锁外按需敲响 TX Doorbell 寄存器通知硬件发送。
        result?;
        self.notify(false);
        Ok(true)
    }

    /// 在所有外部发送生产者均已退出后，按“关闭后台 `Work` -> 硬件复位并轮询确认 -> 同步注销 IRQ -> 收回缓冲页并释放”的严格顺序停机。
    ///
    /// 必须在可睡眠的普通任务上下文调用。若硬件在 2 秒超时内未确认复位完成（返回 [`VirtioInitError::StopUnconfirmed`]），
    /// 本方法会保持中断注册与全部 DMA 物理页不释放，防止仍在失控运行的硬件通过残留 DMA 地址踩坏已归还的内存页。
    pub fn stop_sync(&self) -> Result<(), VirtioInitError> {
        let _control = self.control.lock();
        // Step 1: 切换阶段为 `Stopping`，抑制 VirtQueue 通知，并在锁外同步等待后台收发 `Work` 彻底执行完毕并关闭。
        let close_work = {
            let mut state = self.state.lock();
            if state.phase == Phase::Stopped {
                return Ok(());
            }
            state.phase = Phase::Stopping;
            state.inner.as_mut().unwrap().driver.set_interrupts(false);
            let close = !state.progress_closed;
            state.progress_closed = true;
            close
        };
        if close_work {
            self.progress
                .get()
                .expect("prepared NIC has no progress Work")
                .as_ref()
                .close_sync();
        }

        // Step 2: 校验所有 TX 写帧线程、RX 处理任务及锁外 Doorbell 通知均已归零；若曾向硬件发布过队列，则发起复位并轮询等待状态归零。
        let must_reset = {
            let state = self.state.lock();
            let inner = state.inner.as_ref().unwrap();
            assert!(
                inner
                    .tx
                    .iter()
                    .all(|slot| slot.state != SlotState::Preparing),
                "NIC producer still preparing TX at stop"
            );
            assert!(
                inner.rx.iter().all(|slot| slot.page.is_some()),
                "NIC RX worker still owns a page at stop"
            );
            assert_eq!(
                state.notify_active, 0,
                "NIC producer still notifying at stop"
            );
            state.must_reset
        };
        if must_reset {
            self.state
                .lock()
                .inner
                .as_mut()
                .unwrap()
                .driver
                .request_reset();
            let deadline = time::now()
                .checked_add(Duration::from_secs(2))
                .expect("NIC reset deadline overflow");
            loop {
                let status = {
                    let state = self.state.lock();
                    let driver = &state.inner.as_ref().unwrap().driver;
                    if driver.reset_complete() {
                        let status = driver.status();
                        if status.is_empty() {
                            KernelHal::dma_barrier();
                            break;
                        }
                        status
                    } else {
                        driver.status()
                    }
                };
                if deadline <= time::now() {
                    return Err(VirtioInitError::StopUnconfirmed { status });
                }
                timer::sleep(Duration::from_millis(1));
            }
        }

        #[cfg(feature = "ktest")]
        if must_reset {
            self.observation.reset_confirmed();
            self.observation.after_reset.stop();
        }
        // Step 3: 硬件已停止产生新事件，同步注销外部中断处理函数并等待正在执行的 IRQ 回调彻底退出。
        let registration = self.registration.lock().take();
        if let Some(registration) = registration {
            registration.unregister_sync();
        }
        #[cfg(feature = "ktest")]
        self.observation
            .unregistered
            .store(true, core::sync::atomic::Ordering::Release);

        // Step 4: 收回所有仍处于设备拥有状态的缓冲页 DMA 访问权，随后按安全逆序销毁驱动视图、缓冲池物理页与双环控制页。
        let owner = {
            let mut state = self.state.lock();
            let inner: &mut Inner = state.inner.as_mut().unwrap();
            let status = inner.driver.status();
            for slot in inner.rx.iter_mut().chain(inner.tx.iter_mut()) {
                let page = slot
                    .page
                    .as_mut()
                    .expect("NIC CPU owner missing after all tails");
                if page.is_device_owned() {
                    // SAFETY:
                    // 1. 外部发送者、锁外 Doorbell 通知、后台 `Work`（Step 1 `close_sync`）及中断处理函数（Step 3 `unregister_sync`）均已彻底退出。
                    // 2. Step 2 已确认硬件状态寄存器读回 `0` 并执行了 `KernelHal::dma_barrier()`，硬件不再对任何 RX/TX 物理页执行 DMA 读写。
                    // 3. 当前槽位独占持有该 `MappedPages`，收回 CPU 访问权无任何并发别名冲突。
                    unsafe { page.finish_device_access() }
                        .map_err(|error| VirtioInitError::dma_prepare(error, status))?;
                }
            }
            // 若上方缓存同步失败而提前返回，`Inner` 仍保留在 `state.inner` 中，不会误释放异常页面。
            state.must_reset = false;
            state.inner.take().unwrap()
        };
        // 触发 `Inner::drop`：先析构 `driver` 队列视图，再释放 `rx`/`tx` 缓冲池页与 `tx_ring`/`rx_ring` 控制页。
        drop(ManuallyDrop::into_inner(owner));
        #[cfg(feature = "ktest")]
        self.observation
            .pages_returned
            .store(true, core::sync::atomic::Ordering::Release);
        self.state.lock().phase = Phase::Stopped;
        Ok(())
    }

    /// 顶半部外部中断处理逻辑（在关中断上下文中运行）：确认本设备 ISR 中断原因、屏蔽队列后续中断，并调度后台收发 `Work`。
    ///
    /// 返回 `true` 表示中断确由本网卡产生并已应答；返回 `false` 表示共享中断线上本次中断不属于本设备。
    fn handle_interrupt(&self) -> bool {
        let (handled, schedule) = {
            let mut state = self.state.lock();
            let inner: &mut Inner = state.inner.as_mut().expect("registered NIC lost DMA owner");
            let reason = inner.driver.ack_interrupt();
            if reason.is_empty() {
                return false;
            }
            inner.driver.set_interrupts(false);
            (true, state.phase == Phase::Live)
        };
        #[cfg(feature = "ktest")]
        self.observation.interrupt(schedule);
        if schedule {
            self.queue_progress();
        }
        handled
    }

    /// 后台 `Work` 独占执行的软中断收发批处理循环（运行于 CPU 0 普通任务上下文）。
    ///
    /// 采用类 NAPI 轮询抑制模型：先按 `TX_BUDGET` 回收已发送槽位并唤醒等待发送的协议栈任务，
    /// 再按 `RX_BUDGET` 批量收包上送协议栈并重投接收缓冲页，最后开启队列中断并二次复查避免漏通知。
    fn progress(&self) -> Result<(), Error> {
        if self.state.lock().phase != Phase::Live {
            return Ok(());
        }
        // Step 1: 批量回收最多 `TX_BUDGET` 个已完成发送的槽位；若有槽位释放则唤醒协议栈重推待发报文。
        let mut completed = 0;
        for _ in 0..TX_BUDGET {
            if !self.complete_tx()? {
                break;
            }
            completed += 1;
        }
        if 0 < completed {
            trace::record(trace::Kind::NetTxCompleteBatch, 1, completed);
            if let Some(net) = self.net.upgrade() {
                net.kick_egress();
                #[cfg(feature = "ktest")]
                self.observation
                    .space_kicks
                    .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            }
        }

        // Step 2: 批量消费最多 `RX_BUDGET` 个已接收帧，在锁外解析上送协议栈后将同一物理页重新投递回接收环。
        let mut received = 0;
        while received < RX_BUDGET {
            let Some((index, page, len)) = self.take_rx().inspect_err(|_| {
                let mut state = self.state.lock();
                state.statistics.rx_errors = state.statistics.rx_errors.wrapping_add(1);
            })?
            else {
                break;
            };
            let result = self.receive_frame(&page, len);
            // 即使协议栈因校验和或路由原因丢弃该帧（`receive_frame` 返回 `Ok`），该缓冲页在 VirtIO 层仍有效，继续重投给硬件复用；
            // 仅当 VirtIO 头部本身损坏（`result.is_err()`）时不再重投，随后向上层返回协议错误。
            self.repost_rx(index, page, result.is_ok())?;
            result?;
            received += 1;
        }
        if 0 < received {
            trace::record(trace::Kind::NetRxBatch, 1, received as u64);
            self.notify(true);
        }
        #[cfg(feature = "ktest")]
        {
            if received == RX_BUDGET {
                self.observation
                    .budget_exhausted
                    .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            }
            if received < RX_BUDGET {
                self.observation.before_rearm.stop();
            }
        }

        // Step 3: 重新开启 VirtQueue 中断并立即复查 Used 环；若复查发现新完成项或本轮 RX 预算耗尽，则再次屏蔽中断并自调度下一轮 `Work`。
        let again = {
            let mut state = self.state.lock();
            if state.phase != Phase::Live {
                false
            } else {
                let pending = state
                    .inner
                    .as_mut()
                    .unwrap()
                    .driver
                    .enable_interrupts_and_recheck();
                #[cfg(feature = "ktest")]
                if pending {
                    self.observation
                        .rearm_pending
                        .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                }
                if pending || received == RX_BUDGET {
                    state.inner.as_mut().unwrap().driver.set_interrupts(false);
                    true
                } else {
                    false
                }
            }
        };
        if again {
            self.queue_progress();
        }
        Ok(())
    }

    /// 从 TX Used 环弹出一个已完成项，校验描述符头归属后收回物理页的 CPU 访问权、释放槽位并累加发送统计。
    ///
    /// 注意：VirtIO 规范中设备写回的 TX `used.len` 通常为 `0`（设备只读不写发送帧），因此实际发送字节数取槽位提交时保存的 `slot.frame_len`。
    fn complete_tx(&self) -> Result<bool, Error> {
        let mut state = self.state.lock();
        if state.phase != Phase::Live {
            return Ok(false);
        }
        let inner: &mut Inner = state.inner.as_mut().unwrap();
        let Some(used) = inner.driver.pop_tx_used()? else {
            return Ok(false);
        };
        let index = owned_slot(&inner.tx, &inner.tx_heads, used.head)?;
        let slot = &mut inner.tx[index];
        let page = slot.page.as_mut().ok_or(Error::IoError)?;
        if !page.is_device_owned() || !(FRAME_MIN..=FRAME_MAX).contains(&slot.frame_len) {
            return Err(Error::IoError);
        }
        // SAFETY:
        // 1. `pop_tx_used` 与 `owned_slot` 共同确认该 `used.head` 对应的 2 段描述符链已由硬件处理完毕，且全队列仅有该链引用此物理页。
        // 2. 当前持有 `state` 自旋锁串行化队列回收，收回 DMA 访问权后槽位恢复为 `SlotState::Free` 供后续发送复用。
        unsafe { page.finish_device_access() }.map_err(dma_protocol)?;
        inner.tx_heads[usize::from(used.head)] = None;
        slot.head = None;
        slot.state = SlotState::Free;
        let frame_len = slot.frame_len as u64;
        state.statistics.tx_packets = state.statistics.tx_packets.wrapping_add(1);
        state.statistics.tx_bytes = state.statistics.tx_bytes.wrapping_add(frame_len);
        #[cfg(feature = "ktest")]
        self.observation
            .tx_completed
            .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        Ok(true)
    }

    /// 从 RX Used 环弹出一个已接收项，校验描述符头与写入字节数合法性，收回物理页 CPU 访问权并将该页移交给调用方（后台 `Work`）。
    fn take_rx(&self) -> Result<Option<(usize, MappedPages, usize)>, Error> {
        let mut state = self.state.lock();
        if state.phase != Phase::Live {
            return Ok(None);
        }
        let inner: &mut Inner = state.inner.as_mut().unwrap();
        let Some(used) = inner.driver.pop_rx_used()? else {
            return Ok(None);
        };
        let index = owned_slot(&inner.rx, &inner.rx_heads, used.head)?;
        let slot = &mut inner.rx[index];
        let page = slot.page.as_mut().ok_or(Error::IoError)?;
        let len = usize::try_from(used.len).map_err(|_| Error::IoError)?;
        if !page.is_device_owned()
            || len < self.info.header_len + FRAME_MIN
            || self.info.header_len + FRAME_MAX < len
            || page.len() < len
        {
            return Err(Error::IoError);
        }
        // SAFETY:
        // 1. `pop_rx_used` 与 `owned_slot` 确认该接收描述符链已由硬件写完返回，且上方条件已验证 `len` 落在 `[header_len + 14, header_len + 1514]` 及页容量范围内。
        // 2. 持有 `state` 自旋锁期间收回设备 DMA 访问权并将槽位转为 `SlotState::Processing`，在调用 `repost_rx` 重新投递前硬件不再写入该页。
        unsafe { page.finish_device_access() }.map_err(dma_protocol)?;
        inner.rx_heads[usize::from(used.head)] = None;
        slot.head = None;
        slot.state = SlotState::Processing;
        #[cfg(feature = "ktest")]
        self.observation
            .rx_completed
            .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        Ok(Some((index, slot.page.take().unwrap(), len)))
    }

    /// 剥离并校验接收缓冲页前部的 `virtio_net_hdr`，更新接收包/字节统计，并将以太网帧切片上送给 [`NetStack`]。
    ///
    /// 在后台 `Work` 上下文、不持有 `state` 自旋锁的状态下调用。若上层协议栈未就绪或拒绝输入帧，仅递增 `rx_dropped` 计数而不视为硬件协议故障。
    fn receive_frame(&self, page: &MappedPages, len: usize) -> Result<(), Error> {
        // SAFETY:
        // 1. 该物理页在分配时已整页清零，且在 `take_rx` 中已通过 `finish_device_access` 收回 CPU 访问权。
        // 2. 单实例 `Work` 独占持有此 `page` 引用，没有任何并发写入者；后续 `rx_frame` 仅读取硬件本轮实际写入的前 `len` 字节，且切片借用在本函数返回前结束。
        let bytes = unsafe { page.bytes() };
        let frame = match rx_frame(&bytes[..len], self.info.header_len) {
            Ok(frame) => frame,
            Err(error) => {
                let mut state = self.state.lock();
                state.statistics.rx_errors = state.statistics.rx_errors.wrapping_add(1);
                state.statistics.rx_dropped = state.statistics.rx_dropped.wrapping_add(1);
                return Err(error);
            }
        };
        {
            let mut state = self.state.lock();
            state.statistics.rx_packets = state.statistics.rx_packets.wrapping_add(1);
            state.statistics.rx_bytes = state.statistics.rx_bytes.wrapping_add(frame.len() as u64);
        }
        if let Some(net) = self.net.upgrade() {
            if let Err(error) = net.receive_ethernet(frame) {
                {
                    let mut state = self.state.lock();
                    state.statistics.rx_dropped = state.statistics.rx_dropped.wrapping_add(1);
                }
                crate::klog!(
                    crate::klog::Level::Debug,
                    "Ethernet input rejected: {:?}",
                    error
                );
            }
        } else {
            let mut state = self.state.lock();
            state.statistics.rx_dropped = state.statistics.rx_dropped.wrapping_add(1);
        }
        Ok(())
    }

    /// 将后台 `Work` 处理完毕的接收物理页归还给原 `rx[index]` 槽位；若网卡仍处于 `Live` 且未发生 VirtIO 协议错误，则重新移交 DMA 访问权并投递至 RX 环。
    fn repost_rx(&self, index: usize, page: MappedPages, valid: bool) -> Result<(), Error> {
        let mut state = self.state.lock();
        let live = state.phase == Phase::Live;
        let inner: &mut Inner = state.inner.as_mut().unwrap();
        let slot = &mut inner.rx[index];
        assert!(slot.state == SlotState::Processing && slot.page.is_none());
        slot.page = Some(page);
        slot.state = SlotState::Free;
        if !live || !valid {
            return Ok(());
        }
        let page = slot.page.as_mut().unwrap();
        let segment = page.segment(inner.domain).map_err(dma_protocol)?;
        // SAFETY:
        // 1. `receive_frame` 已返回，CPU 侧对上一帧数据的只读切片借用已全部结束。
        // 2. 物理页已放回 `inner.rx[index]` 并受 `state` 自旋锁保护，在下一轮收到 RX Used 项或确认硬件复位前保持存活。
        unsafe { page.begin_device_access(inner.domain) }.map_err(dma_protocol)?;
        slot.state = SlotState::DeviceOwned;
        // SAFETY:
        // 1. `segment` 来自方向为 `DeviceToDriver`、容量至少为 `header_len + FRAME_MAX` 的单页缓冲，2 个接收描述符完全落在该页内。
        // 2. 同一 `state` 自旋锁临界区内紧接着调用 `install_head` 建立映射，确保中断或下一轮 `Work` 消费前元数据已一致。
        let head = unsafe { inner.driver.submit_rx(segment) }?;
        install_head(&mut inner.rx, &mut inner.rx_heads, index, head)
    }

    /// 按 VirtQueue 通知抑制标志判断是否需要敲响硬件 Doorbell：在锁内递增 `notify_active` 计数，在锁外写入通知寄存器并执行全内存屏障。
    ///
    /// `receive` 为 `true` 时通知 RX 队列（补充了新接收页），为 `false` 时通知 TX 队列（提交了新发送帧）。
    fn notify(&self, receive: bool) {
        let notification: Option<Notification> = {
            let mut state = self.state.lock();
            if state.phase != Phase::Live {
                return;
            }
            let driver = &state.inner.as_ref().unwrap().driver;
            let notification = if receive {
                driver.rx_should_notify().then(|| driver.rx_notification())
            } else {
                driver.tx_should_notify().then(|| driver.tx_notification())
            };
            if notification.is_some() {
                state.notify_active += 1;
            }
            notification
        };
        if let Some(notification) = notification {
            // SAFETY:
            // 1. 锁内已递增 `notify_active`；`stop_sync` 必须在外部发送者全部退出且 `Work` 关闭后才会运行，并在复位硬件前断言 `notify_active == 0`。
            // 2. 调用 `prepare` 时的安全契约保证了硬件 Doorbell 寄存器映射在整个生命周期内永久有效。
            unsafe { notification.notify::<KernelHal>() };
            KernelHal::dma_barrier();
            self.state.lock().notify_active -= 1;
        }
    }

    /// 将预分配的收发批处理 `Work` 加入网络工作队列（若已在队列中或正在运行，`Work::queue` 会自动合并；若已关闭则静默忽略）。
    fn queue_progress(&self) {
        let _ = self
            .progress
            .get()
            .expect("NIC progress Work missing")
            .as_ref()
            .queue();
    }

    /// 记录首次发生的运行期队列/DMA 协议错误，将网卡状态切换至 `Phase::Faulted` 并屏蔽队列中断，防止继续处理损坏的描述符环。
    fn record_failure(&self, error: Error) {
        let mut state = self.state.lock();
        if state.failure.is_none() {
            state.failure = Some(error);
        }
        if state.phase == Phase::Live {
            state.phase = Phase::Faulted;
            state.inner.as_mut().unwrap().driver.set_interrupts(false);
        }
    }
}

impl Drop for VirtioNet {
    /// 析构 `VirtioNet` 实例并释放剩余内部资源。
    ///
    /// 仅允许在准备阶段未发布即丢弃（`Phase::Prepared` 且 `!must_reset`），或已通过 [`Self::stop_sync`] 彻底停机（`Phase::Stopped`）后执行；
    /// 若硬件仍处于活动或未确认复位状态被提前释放，将立即触发断言防止 DMA 悬垂访问。
    fn drop(&mut self) {
        let owner = {
            let mut state = self.state.lock();
            assert!(!state.must_reset, "dropping NIC before confirmed reset");
            assert!(
                matches!(state.phase, Phase::Prepared | Phase::Stopped),
                "dropping live NIC context"
            );
            state.inner.take()
        };
        if let Some(owner) = owner {
            drop(ManuallyDrop::into_inner(owner));
        }
    }
}

/// 发送路径锁外填充以太网帧期间的栈上 RAII 守卫，临时持有从 `tx[slot]` 取出的物理页。
///
/// 若 `try_transmit` 在正式将页面移交设备 DMA 前因 panic 或状态切换（如网卡进入非 `Live` 状态）提前退出，
/// [`Drop`] 实现会自动重新获取队列锁并将该物理页归还回原槽位，避免缓冲池物理页泄漏。
struct PreparingTx<'a> {
    /// 所属的 VirtIO 网卡实例引用。
    nic: &'a VirtioNet,
    /// 被认领的发送槽位在 `Inner::tx` 中的下标。
    slot: usize,
    /// 临时由当前发送线程独占持有的物理页；一旦在队列锁内成功放回槽位则被 `take` 为 `None`。
    page: Option<MappedPages>,
}

impl Drop for PreparingTx<'_> {
    /// 当发送准备未完成提交即退出作用域时，将仍归 CPU 所有的物理页放回原发送槽位并恢复为 `SlotState::Free`。
    fn drop(&mut self) {
        if let Some(page) = self.page.take() {
            let mut state = self.nic.state.lock();
            let slot = &mut state.inner.as_mut().unwrap().tx[self.slot];
            assert!(slot.state == SlotState::Preparing && slot.page.is_none());
            slot.page = Some(page);
            slot.state = SlotState::Free;
        }
    }
}

/// 在向硬件发布任何地址之前，预先分配 RX/TX Split Ring 控制页、单页缓冲池与描述符头反查表，并构建 `VirtIONet` 驱动实例。
///
/// 若任何一步内存分配失败，函数内局部变量按逆序析构（`VirtQueue` 先于 `CoherentDma` 析构），所有物理页安全归还伙伴系统。
fn prepare_inner(
    transport: SomeTransport<'static>,
    info: NetInfo,
    domain: DmaDomain,
) -> Result<Inner, VirtioInitError> {
    let status = transport.get_status();
    let legacy = transport.requires_legacy_layout();
    let rx_layout = RingLayout::new(info.rx_depth, legacy)?;
    let tx_layout = RingLayout::new(info.tx_depth, legacy)?;
    let rx_ring = CoherentDma::allocate(domain, rx_layout.byte_len())
        .map_err(|error| VirtioInitError::dma_prepare(error, status))?;
    let tx_ring = CoherentDma::allocate(domain, tx_layout.byte_len())
        .map_err(|error| VirtioInitError::dma_prepare(error, status))?;
    let rx = prepare_pool(
        domain,
        info.rx_depth,
        info.header_len,
        BufferDirection::DeviceToDriver,
        status,
    )?;
    let tx = prepare_pool(
        domain,
        info.tx_depth,
        info.header_len,
        BufferDirection::DriverToDevice,
        status,
    )?;
    let rx_heads = prepare_heads(info.rx_depth, status)?;
    let tx_heads = prepare_heads(info.tx_depth, status)?;

    // SAFETY:
    // 1. `rx_ring` 是刚分配并清零的一致性连续物理页，容量满足 `rx_layout`，且此时尚未向硬件发布地址。
    // 2. 若后续步骤失败返回，`rx_queue` 局部变量先于 `rx_ring` 逆序析构，不会产生悬垂指针。
    let rx_queue = unsafe { VirtQueue::<KernelHal>::new(rx_layout, rx_ring.region()) }
        .map_err(|error| VirtioInitError::queue_prepare(error, status))?;
    // SAFETY:
    // 1. `tx_ring` 与 `rx_ring` 相互独立，容量满足 `tx_layout`，同样尚未向硬件发布地址。
    // 2. 错误返回时 `tx_queue` 先于 `tx_ring` 析构。
    let tx_queue = unsafe { VirtQueue::<KernelHal>::new(tx_layout, tx_ring.region()) }
        .map_err(|error| VirtioInitError::queue_prepare(error, status))?;
    // SAFETY:
    // 1. `rx_queue` 与 `tx_queue` 绑定的底层物理页 `rx_ring`、`tx_ring` 与 `driver` 一同封装进 `Inner`，且 `Inner::drop` 保证 `driver` 先析构。
    // 2. 外层 `VirtioNet` 一旦开始发布队列即要求在确认硬件 `reset` 完成后才允许析构 `Inner`。
    let driver = unsafe { Driver::new(transport, info, rx_queue, tx_queue) }?;
    Ok(Inner {
        driver: ManuallyDrop::new(driver),
        domain,
        rx_ring: ManuallyDrop::new(rx_ring),
        tx_ring: ManuallyDrop::new(tx_ring),
        rx,
        tx,
        rx_heads,
        tx_heads,
    })
}

/// 为单个队列方向（RX 或 TX）预分配固定数量（`min(depth / 2, 32)`）的单张清零物理页，并预先验证其在 `domain` 下的 DMA 可映射性。
///
/// 每张页的有效映射长度固定为 `header_len + FRAME_MAX`（覆盖 VirtIO 网络头与最大 1514 字节以太网帧）。
fn prepare_pool(
    domain: DmaDomain,
    depth: u16,
    header_len: usize,
    direction: BufferDirection,
    status: DeviceStatus,
) -> Result<Vec<Slot>, VirtioInitError> {
    let count = usize::from(depth / FRAME_DESCRIPTOR_COUNT).min(POOL_MAX_SLOTS);
    let len = header_len
        .checked_add(FRAME_MAX)
        .ok_or(Error::InvalidParam)?;
    let mut slots = Vec::new();
    slots
        .try_reserve_exact(count)
        .map_err(|_| VirtioInitError::NoMemory { status })?;
    for _ in 0..count {
        let pages = mm::alloc_pages(0, true).ok_or(VirtioInitError::NoMemory { status })?;
        let page = MappedPages::new(pages, 0, len, direction)
            .map_err(|error| VirtioInitError::dma_prepare(error, status))?;
        page.segment(domain)
            .map_err(|error| VirtioInitError::dma_prepare(error, status))?;
        slots.push(Slot {
            state: SlotState::Free,
            page: Some(page),
            head: None,
            frame_len: 0,
        });
    }
    Ok(slots)
}

/// 在普通任务上下文中预分配长度为 `depth` 的描述符头到槽位下标反查表，确保中断与收发路径无需动态扩容。
fn prepare_heads(depth: u16, status: DeviceStatus) -> Result<Vec<Option<usize>>, VirtioInitError> {
    let mut heads = Vec::new();
    heads
        .try_reserve_exact(usize::from(depth))
        .map_err(|_| VirtioInitError::NoMemory { status })?;
    heads.resize(usize::from(depth), None);
    Ok(heads)
}

/// 向硬件发布双队列（置位 `DRIVER_OK`），将所有预分配 RX 槽位物理页移交设备 DMA 并提交到接收环，最后检查是否已有到达事件。
///
/// 调用方必须持有 `state` 自旋锁，且已将 `state.must_reset` 置为 `true`。
fn publish_rx(inner: &mut Inner) -> Result<bool, Error> {
    // Step 1: 向硬件寄存器写入 RX/TX 队列地址并置位 `DRIVER_OK`，先屏蔽队列通知以批量装填接收环。
    inner.driver.publish()?;
    inner.driver.set_interrupts(false);

    // Step 2: 将池中所有空闲 RX 物理页逐一移交设备 DMA 访问权，提交至 RX VirtQueue 并记录描述符头映射。
    for index in 0..inner.rx.len() {
        let slot = &mut inner.rx[index];
        let page = slot.page.as_mut().ok_or(Error::IoError)?;
        let segment = page.segment(inner.domain).map_err(dma_protocol)?;
        // SAFETY:
        // 1. 池内物理页在 `prepare_pool` 中已整页清零，当前无任何 CPU 切片借用，且由 `inner.rx[index]` 持久保活。
        // 2. 调用方持有 `state` 自旋锁并已置位 `must_reset = true`，中途若失败也会保留页面直到 `stop_sync` 确认硬件复位。
        unsafe { page.begin_device_access(inner.domain) }.map_err(dma_protocol)?;
        slot.state = SlotState::DeviceOwned;
        // SAFETY:
        // 1. `segment` 为 `DeviceToDriver` 方向且长度覆盖 `header_len + 1514` 字节，满足 `submit_rx` 对头+帧双描述符的要求。
        // 2. 同一把 `state` 锁内立即调用 `install_head` 记录 `head` 归属，在此之前没有任何消费者会弹出 Used 环。
        let head = unsafe { inner.driver.submit_rx(segment) }?;
        install_head(&mut inner.rx, &mut inner.rx_heads, index, head)?;
    }

    let pending = inner.driver.enable_interrupts_and_recheck();
    if pending {
        inner.driver.set_interrupts(false);
    }
    Ok(pending)
}

/// 在队列锁保护下双向绑定 VirtQueue 描述符头索引 `head` 与缓冲池槽位下标 `index`。
///
/// 绝不直接假设硬件返回的 `head` 等于槽位下标；若检测到 `head` 越界或重复占用，立即返回 `Error::IoError` 阻止破坏已有 DMA 页归属。
fn install_head(
    slots: &mut [Slot],
    heads: &mut [Option<usize>],
    index: usize,
    head: u16,
) -> Result<(), Error> {
    let entry = heads.get_mut(usize::from(head)).ok_or(Error::IoError)?;
    let slot = slots.get_mut(index).ok_or(Error::IoError)?;
    if entry.is_some() || slot.head.is_some() || slot.state != SlotState::DeviceOwned {
        return Err(Error::IoError);
    }
    *entry = Some(index);
    slot.head = Some(head);
    Ok(())
}

/// 校验硬件 Used 环返回的描述符头 `head` 是否对应一个真实处于 `DeviceOwned` 状态的软件槽位，并返回槽位下标。
///
/// 若硬件返回了伪造、重复或越界的 `head`，仅返回 `Error::IoError` 供上层转入 `Faulted` 状态，绝不误释放其他槽位的 DMA 页面。
fn owned_slot(slots: &[Slot], heads: &[Option<usize>], head: u16) -> Result<usize, Error> {
    let index = heads
        .get(usize::from(head))
        .copied()
        .flatten()
        .ok_or(Error::IoError)?;
    let slot = slots.get(index).ok_or(Error::IoError)?;
    if slot.state != SlotState::DeviceOwned || slot.head != Some(head) || slot.page.is_none() {
        return Err(Error::IoError);
    }
    Ok(index)
}

/// 从设备枚举库存中按当前 CPU 架构选取唯一合法的 VirtIO 网卡候选并构造传输层（RISC-V 使用 MMIO，LoongArch 使用 PCI）。
///
/// 若库存中不存在网卡或存在多张网卡（无法无歧义绑定默认接口），均返回 [`ProbeError::Resource`]。
///
/// # Safety
/// 调用方必须满足 [`VirtioNet::prepare`] 的安全契约：`inventory` 来自真实单次枚举，选定网卡寄存器窗口已完成永久强序映射且由本驱动独占。
unsafe fn select_transport(
    inventory: &DeviceInventory,
) -> Result<(SomeTransport<'static>, VirtioIdentity, IrqNumber), VirtioInitError> {
    #[cfg(target_arch = "riscv64")]
    {
        let [candidate] = inventory.mmio_networks.as_slice() else {
            return Err(
                ProbeError::Resource("network inventory requires exactly one MMIO NIC").into(),
            );
        };
        // SAFETY: `candidate` 来自同一 `inventory` 的唯一 MMIO 网卡记录，调用方保证了寄存器永久强序映射与硬件独占。
        unsafe {
            crate::drivers::virtio::prepare_mmio_transport(
                candidate.registers,
                candidate.version,
                candidate.vendor_id,
                candidate.interrupt,
            )
        }
    }
    #[cfg(target_arch = "loongarch64")]
    {
        let mut candidates = inventory.pci.devices().iter().filter(|device| {
            device.vendor_id == VIRTIO_VENDOR_ID
                && matches!(
                    device.device_id,
                    VIRTIO_PCI_NET_LEGACY | VIRTIO_PCI_NET_MODERN
                )
        });
        let candidate = candidates
            .next()
            .ok_or(ProbeError::Resource("network inventory has no PCI NIC"))?;
        if candidates.next().is_some() {
            return Err(ProbeError::Resource("network inventory has multiple PCI NICs").into());
        }
        // SAFETY: `candidate` 来自同一 `inventory.pci` 的唯一 VirtIO 网卡功能，调用方保证了 BAR 永久强序映射与硬件独占。
        unsafe { crate::drivers::virtio::prepare_pci_transport(&inventory.pci, candidate) }
    }
}

/// 注册到外部中断子系统的 C 风格类型擦除回调函数，将 `context` 还原为 `&VirtioNet` 并委托给 [`VirtioNet::handle_interrupt`]。
///
/// # Safety
/// 1. `context` 必须是 [`VirtioNet::start`] 注册中断时传入的 `Arc::as_ptr(self) as *mut ()`。
/// 2. 上层 `NetStack` 必须在调用 `unregister_sync` 完成同步注销前始终持有该 `Arc<VirtioNet>` 的强引用，确保指针指向的内存有效且不可移动。
unsafe fn irq_callback(context: *mut ()) -> bool {
    // SAFETY: 调用方与 `NetStack` 生命周期契约保证了在 `unregister_sync` 返回前，`context` 始终指向有效且存活的 `VirtioNet` 实例。
    unsafe { &*context.cast::<VirtioNet>() }.handle_interrupt()
}

/// 网络工作队列调度的后台收发回调入口（单实例 `Work` 自身保证不可重入，从而保证 RX 消费者唯一）。
///
/// 若轮询过程中检测到硬件返回的队列元数据违反 VirtIO 协议，先将错误记录至 `Faulted` 状态再触发 panic 快速失败，保留现场供诊断。
fn progress_callback(work: Pin<&Work>) {
    // SAFETY: `work.context()` 在 `VirtioNet::prepare` 中由 `Arc::as_ptr(&nic)` 初始化；上层 `NetStack` 在 `stop_sync` 调用 `close_sync` 等到本回调彻底退出前始终持有该 `Arc`。
    let nic = unsafe { &*(work.context() as *const VirtioNet) };
    #[cfg(feature = "ktest")]
    let _tail = nic.observation.enter_work();
    if let Err(error) = nic.progress() {
        nic.record_failure(error);
        panic!("VirtIO network queue protocol failed: {:?}", error);
    }
}
