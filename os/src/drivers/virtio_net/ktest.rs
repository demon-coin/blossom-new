//! `VirtioNet` 网卡驱动的内核集成测试（`ktest`）与并发时序观测点。
//!
//! ## 模块职责与架构定位
//! 本模块作为 [`super::VirtioNet`] 的内联测试子模块（仅在 `cfg(feature = "ktest")` 下编译），
//! 提供两部分能力：
//! 1. **非侵入式并发停点与观测计数器**（[`Gate`]、[`Observation`]、[`WorkTail`]）：嵌入到 `VirtioNet`
//!    结构体内，允许测试用例在后台 `Work` 轮询或 `stop_sync` 停机流程的关键窗口暂停执行流，并原子采样收发完成数、
//!    预算耗尽次数、中断重装二次复查及 DMA 物理页回收状态。
//! 2. **基于 QEMU `virt` 硬件与宿主 UDP 对端的端到端测试套件**（[`native`]）：通过 [`Inventory`] 与 [`Owner`]
//!    两级测试夹具（Fixture）独占初始化 VirtIO 网卡（及多块 VirtIO 磁盘），验证 TX/RX 描述符槽位流转、
//!    NAPI 式收包预算续跑、中断重装（Rearm）竞态窗口、共享中断线（INTx）多设备共存、停机复位时序屏障及 OOM 恢复。
//!
//! ## 核心设计与关键状态机
//!
//! ### 1. 单次触发停点门控状态机（`Gate`）
//! ```text
//!   GATE_UNSET (0, 默认直通，不阻塞后台 Work 或停机路径)
//!       │
//!       │ 测试线程调用 arm()
//!       ▼
//!   GATE_ARMED (1, 已布防，等待目标执行路径命停点)
//!       │
//!       │ 目标线程调用 stop() 并 CAS 命中
//!       ▼
//!   GATE_ENTERED (2, 目标线程在无自旋锁上下文中每次 sleep 1ms 等待；硬件继续向 Used 环生产完成项)
//!       │
//!       │ 测试线程或清理路径调用 release()
//!       ▼
//!   GATE_RELEASED (3, 门控放行，目标线程退出循环并继续执行后续逻辑)
//! ```
//!
//! ### 2. 驱动内部注入的三个停点位置（`Observation`）
//! ```text
//!   后台 Work 回调 (`VirtioNet::work`)：
//!     enter_work() ──► [停点 1: entry] (Work 刚进入、尚未获取 state 锁处理 RX/TX Used 环前)
//!       │
//!       ├──► 循环处理 TX/RX Used 环（受 TX_BUDGET=32 / RX_BUDGET=16 限制）
//!       ├──► 若首次判空准备重装队列中断 ──► [停点 2: before_rearm] (释放 state 锁后、打开队列中断并二次复查前)
//!       └──► 函数退出时析构 WorkTail (`work_exited += 1`)
//!
//!   同步停机路径 (`VirtioNet::stop_sync`)：
//!     关闭队列中断 ──► close_sync 等待 Work 彻底退出 ──► 轮询硬件 reset 归零 (`reset_confirmed`)
//!       ──► [停点 3: after_reset] (硬件已确认复位、尚未注销 IRQ 及收回 DMA 物理页前)
//!       ──► 注销 IRQ (`unregistered = true`) ──► 收回并释放所有缓冲页 (`pages_returned = true`)
//! ```
//!
//! ## 锁获取顺序（Lock Ordering）与并发契约
//! - **严禁持自旋锁进入 `Gate::stop`**：所有 `Gate::stop()` 调用点均位于 `VirtioNet::state` 与 `registration`
//!   自旋锁临界区之外，因为 `stop()` 内部通过 `timer::sleep` 主动让出 CPU。
//! - **快照采样（`VirtioNet::snapshot`）与 `Owner` 锁**：`snapshot()` 短暂获取 `VirtioNet::state` 自旋锁校验槽位与
//!   `head` 映射表的一致性；测试辅助结构中的 `Owner::owner` 自旋锁仅用于跨任务移交 `NetStackOwner`，不与 `VirtioNet::state` 嵌套。

use super::{Phase, SlotState, VirtioNet};
use crate::block::{Bio, BioAllocator, BlkOperation, Disk, DiskIdentity, DiskRef, IoResult};
use crate::fs::{FsError, PollMask, ReadBuffer};
use crate::ktest::execution_services::{baseline, Baseline, SuiteBaseline};
use crate::mm::dma::{DmaDomain, MappedPages};
use crate::net::{
    address::{AddressFamily, InetEndpoint, SocketAddress},
    packet::OwnedPacket,
    route::{EthernetConfig, InterfaceId},
    udp::UdpEndpoint,
    IoProgress, NetStack, NetStackOwner, NetStackRef,
};
use crate::sync::SpinLock;
use crate::workqueue::{WorkQueue, WorkQueueConfig};
use crate::{
    drivers::{
        virtio::{
            VirtioDevice, VirtioIdentity, VirtioInitError, VIRTIO_PCI_BLOCK_LEGACY,
            VIRTIO_PCI_BLOCK_MODERN, VIRTIO_VENDOR_ID,
        },
        DeviceInventory,
    },
    irq, mm, sched, task, time, timer,
};
use ::ktest::{self, Fixture, TestContext, TestError, TestResult};
use alloc::{sync::Arc, vec::Vec};
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicUsize, Ordering};
use smoltcp::wire::{
    ArpOperation, ArpPacket, ArpRepr, EthernetAddress, EthernetFrame, EthernetProtocol,
    EthernetRepr, IpAddress, Ipv4Address,
};
use virtio_drivers::{transport::DeviceStatus, BufferDirection, Error};

/// 门控初始未布防状态：目标路径调用 `stop()` 时直接放行。
const GATE_UNSET: u8 = 0;
/// 门控已布防状态：下一次目标路径调用 `stop()` 时将进入阻塞等待。
const GATE_ARMED: u8 = 1;
/// 目标执行流已命停点并正在休眠等待放行。
const GATE_ENTERED: u8 = 2;
/// 测试控制线程或清理路径已放行该停点。
const GATE_RELEASED: u8 = 3;

/// 用于在测试线程与后台驱动执行流之间同步时序窗口的四态原子门控（元组字段 `0` 为保存当前门控状态的 `AtomicU8`）。
pub(super) struct Gate(AtomicU8);

impl Gate {
    /// 创建处于 [`GATE_UNSET`] 默认直通状态的门控，不阻塞正常的驱动执行路径。
    const fn new() -> Self {
        Self(AtomicU8::new(GATE_UNSET))
    }

    /// 由测试控制线程调用，将门控设为 [`GATE_ARMED`]，使目标路径下一次经过该停点时暂停。
    fn arm(&self) {
        self.0.store(GATE_ARMED, Ordering::Release);
    }

    /// 查询目标执行流是否已命停点并处于 [`GATE_ENTERED`] 等待状态。
    fn entered(&self) -> bool {
        self.0.load(Ordering::Acquire) == GATE_ENTERED
    }

    /// 将门控设为 [`GATE_RELEASED`]，唤醒正在该停点休眠等待的目标执行流（正常测试步骤与异常清理共用）。
    fn release(&self) {
        self.0.store(GATE_RELEASED, Ordering::Release);
    }

    /// 若门控处于 [`GATE_ARMED`] 状态，则将其切换为 [`GATE_ENTERED`] 并以 1ms 为周期休眠等待，直至被 [`Self::release`] 放行。
    ///
    /// 调用方必须确保当前未持有任何自旋锁，以便在休眠等待期间允许时钟中断、调度器与硬件队列继续运行。
    pub(super) fn stop(&self) {
        if self
            .0
            .compare_exchange(
                GATE_ARMED,
                GATE_ENTERED,
                Ordering::AcqRel,
                Ordering::Acquire,
            )
            .is_ok()
        {
            while self.0.load(Ordering::Acquire) == GATE_ENTERED {
                timer::sleep(time::Duration::from_millis(1));
            }
        }
    }
}

/// 嵌入在 [`VirtioNet`] 内部的测试停点与运行期统计观测器。
pub(super) struct Observation {
    /// 后台 `Work` 回调刚进入、尚未获取锁处理队列完成项前的停点。
    pub(super) entry: Gate,
    /// 后台 `Work` 首轮判空后、重新启用队列中断并执行二次复查（Recheck）前的停点。
    pub(super) before_rearm: Gate,
    /// `stop_sync` 确认硬件 `status == 0` 复位完成后、注销中断与回收物理页前的停点。
    pub(super) after_reset: Gate,
    /// 累计已从 TX Used 环收回并释放回空闲池的发送槽位次数。
    pub(super) tx_completed: AtomicUsize,
    /// 累计已从 RX Used 环取出的接收完成项次数。
    pub(super) rx_completed: AtomicUsize,
    /// 发送槽位从满变为有空闲后，回调 `NetStack::kick_egress` 唤醒阻塞发送者的累计次数。
    pub(super) space_kicks: AtomicUsize,
    /// 单轮后台 `Work` 因达到收发预算上限（`RX_BUDGET` 或 `TX_BUDGET`）而主动自续跑的累计次数。
    pub(super) budget_exhausted: AtomicUsize,
    /// 重新打开队列中断后二次复查发现新的待处理 Used 项、从而重新压入 `Work` 的累计次数。
    pub(super) rearm_pending: AtomicUsize,
    /// 后台 `Work` 回调已进入执行的累计次数。
    work_entered: AtomicUsize,
    /// 后台 `Work` 回调已彻底执行完毕并退出的累计次数。
    work_exited: AtomicUsize,
    /// 网卡中断服务例程（ISR）确认本设备产生中断的累计次数。
    interrupts: AtomicUsize,
    /// 驱动处于非 `Live`（如 `Stopping`）阶段期间收到的本设备中断累计次数。
    stopping_interrupts: AtomicUsize,
    /// 标记 `stop_sync` 是否已完成硬件复位状态字回读确认与 DMA 内存屏障。
    reset_barrier: AtomicBool,
    /// 标记 `stop_sync` 是否已同步注销外部中断处理函数。
    pub(super) unregistered: AtomicBool,
    /// 标记 `stop_sync` 是否已收回全部 RX/TX 缓冲页的 DMA 访问权并释放底层物理页。
    pub(super) pages_returned: AtomicBool,
}

impl Observation {
    /// 创建所有停点未布防、全部事件计数器归零的初始观测器。
    pub(super) const fn new() -> Self {
        Self {
            entry: Gate::new(),
            before_rearm: Gate::new(),
            after_reset: Gate::new(),
            tx_completed: AtomicUsize::new(0),
            rx_completed: AtomicUsize::new(0),
            space_kicks: AtomicUsize::new(0),
            budget_exhausted: AtomicUsize::new(0),
            rearm_pending: AtomicUsize::new(0),
            work_entered: AtomicUsize::new(0),
            work_exited: AtomicUsize::new(0),
            interrupts: AtomicUsize::new(0),
            stopping_interrupts: AtomicUsize::new(0),
            reset_barrier: AtomicBool::new(false),
            unregistered: AtomicBool::new(false),
            pages_returned: AtomicBool::new(false),
        }
    }

    /// 记录后台 `Work` 开始执行，并在返回作用域守卫 [`WorkTail`] 前检查 [`Self::entry`] 停点。
    pub(super) fn enter_work(&self) -> WorkTail<'_> {
        self.work_entered.fetch_add(1, Ordering::Relaxed);
        let tail = WorkTail(self);
        self.entry.stop();
        tail
    }

    /// 记录一次已确认属于本网卡的硬件中断；若当前驱动不处于 `Live` 阶段，同时递增 `stopping_interrupts`。
    pub(super) fn interrupt(&self, live: bool) {
        self.interrupts.fetch_add(1, Ordering::Relaxed);
        if !live {
            self.stopping_interrupts.fetch_add(1, Ordering::Relaxed);
        }
    }

    /// 在硬件复位状态字读回为 0 且执行完完整 DMA 内存屏障后调用，记录 `reset_barrier` 已达成。
    pub(super) fn reset_confirmed(&self) {
        self.reset_barrier.store(true, Ordering::Release);
    }

    /// 放行全部测试停点（`entry`、`before_rearm`、`after_reset`），确保阻塞中的后台 `Work` 或停机任务能够继续退出。
    fn release_all(&self) {
        self.entry.release();
        self.before_rearm.release();
        self.after_reset.release();
    }
}

/// 后台 `Work` 回调的作用域退出守卫（元组字段 `0` 为所属网卡观测器借用），确保无论回调正常返回还是提前退出都会递增 `work_exited`。
pub(super) struct WorkTail<'a>(&'a Observation);

impl Drop for WorkTail<'_> {
    /// 在后台 `Work` 回调作用域结束时递增 `work_exited` 计数。
    fn drop(&mut self) {
        self.0.work_exited.fetch_add(1, Ordering::Release);
    }
}

/// 单个收发方向（RX 或 TX）缓冲池的槽位状态统计与一致性快照。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Pool {
    /// 缓冲池中的槽位总数。
    count: usize,
    /// 处于 [`SlotState::Free`] 空闲状态的槽位数。
    free: usize,
    /// 处于 [`SlotState::Preparing`]（锁外填充发送帧）状态的槽位数。
    preparing: usize,
    /// 处于 [`SlotState::DeviceOwned`]（已提交给硬件队列）状态的槽位数。
    owned: usize,
    /// 处于 [`SlotState::Processing`]（锁外解析接收帧）状态的槽位数。
    processing: usize,
    /// 当前保存在 `Slot::page` 中的物理页数量（`Free` 与 `DeviceOwned` 槽位持有物理页）。
    pages: usize,
    /// `rx_heads` 或 `tx_heads` 映射表中当前非空的描述符链头部索引数量。
    heads: usize,
    /// 物理页 DMA 状态机标记为设备拥有（`is_device_owned() == true`）的页数。
    device_pages: usize,
    /// 每个槽位绑定的物理页在当前 DMA 域下的设备总线地址（无物理页的槽位记为 0）。
    addresses: [u64; super::POOL_MAX_SLOTS],
}

/// 网卡驱动在某一时刻的完整队列状态、槽位所有权与观测计数快照。
#[derive(Debug)]
struct Snapshot {
    /// 接收缓冲池（RX）的槽位与物理页统计。
    rx: Pool,
    /// 发送缓冲池（TX）的槽位与物理页统计。
    tx: Pool,
    /// RX Used 环中硬件已写出但驱动尚未消费的完成项数量。
    pending_rx: usize,
    /// TX Used 环中硬件已写出但驱动尚未回收的完成项数量。
    pending_tx: usize,
    /// 当前硬件 `status` 寄存器状态（若 `Inner` 已在停机后释放则为 `None`）。
    status: Option<DeviceStatus>,
    /// RX 与 TX 两个 Split VirtQueue 控制环的 `(设备总线基址, 字节长度)`（若 `Inner` 已释放则为 `None`）。
    rings: Option<[(u64, usize); 2]>,
    /// 驱动当前是否处于 [`Phase::Stopping`] 阶段。
    stopping: bool,
    /// 驱动当前是否处于 [`Phase::Stopped`] 阶段。
    stopped: bool,
    /// 累计已回收的 TX 槽位完成次数。
    tx_completed: usize,
    /// 累计已取出的 RX 完成项次数。
    rx_completed: usize,
    /// 累计因释放 TX 槽位而唤醒阻塞发送者的次数。
    kicks: usize,
    /// 累计因单轮收发预算耗尽而触发 `Work` 自续跑的次数。
    budgets: usize,
    /// 累计在重装队列中断后二次复查发现待处理项并重排 `Work` 的次数。
    rechecks: usize,
    /// 后台 `Work` 回调已进入执行的累计次数。
    entered: usize,
    /// 后台 `Work` 回调已退出执行的累计次数。
    exited: usize,
    /// 累计确认属于本网卡的中断总次数（用于 Debug 诊断输出）。
    #[expect(
        dead_code,
        reason = "NIC 快照的 Debug 诊断记录收发及停止窗口中的累计 IRQ 数"
    )]
    interrupts: usize,
    /// 驱动非 `Live` 期间收到的本网卡中断累计次数（用于 Debug 诊断输出）。
    #[expect(dead_code, reason = "NIC 停止窗口的 Debug 诊断区分停止期间到达的 IRQ")]
    stopping_interrupts: usize,
    /// 硬件复位状态读回与 DMA 内存屏障是否已完成。
    barrier: bool,
    /// 外部中断处理函数是否已同步注销。
    unregistered: bool,
    /// 缓冲池与队列控制环的物理页是否已全部收回并释放。
    returned: bool,
}

/// 在持有 `VirtioNet::state` 锁期间扫描单个方向的槽位数组与 `head` 映射表，统计 [`Pool`] 快照并校验双向所有权不变量。
fn pool(slots: &[super::Slot], heads: &[Option<usize>], domain: DmaDomain) -> Result<Pool, Error> {
    let mut result = Pool {
        count: slots.len(),
        free: 0,
        preparing: 0,
        owned: 0,
        processing: 0,
        pages: 0,
        heads: 0,
        device_pages: 0,
        addresses: [0; super::POOL_MAX_SLOTS],
    };
    // Step 1: 遍历所有固定槽位，统计各状态计数，并校验物理页存在性及 DMA 访问权与 SlotState 是否严格一致。
    for (index, slot) in slots.iter().enumerate() {
        match slot.state {
            SlotState::Free => result.free += 1,
            SlotState::Preparing => result.preparing += 1,
            SlotState::DeviceOwned => result.owned += 1,
            SlotState::Processing => result.processing += 1,
        }
        result.pages += usize::from(slot.page.is_some());
        if let Some(page) = &slot.page {
            result.addresses[index] = page
                .segment(domain)
                .map_err(super::dma_protocol)?
                .address
                .value();
            result.device_pages += usize::from(page.is_device_owned());
            if page.is_device_owned() != (slot.state == SlotState::DeviceOwned) {
                return Err(Error::IoError);
            }
        } else if !matches!(slot.state, SlotState::Preparing | SlotState::Processing) {
            return Err(Error::IoError);
        }
        if let Some(head) = slot.head {
            if heads.get(usize::from(head)) != Some(&Some(index))
                || slot.state != SlotState::DeviceOwned
            {
                return Err(Error::WrongToken);
            }
        }
    }

    // Step 2: 反向遍历描述符链 head 映射表，确认每个已登记的 head 都唯一对应一个 DeviceOwned 状态的槽位。
    for (head, index) in heads.iter().enumerate() {
        if let Some(index) = index {
            if slots.get(*index).is_none_or(|slot| {
                slot.head != Some(head as u16) || slot.state != SlotState::DeviceOwned
            }) {
                return Err(Error::WrongToken);
            }
            result.heads += 1;
        }
    }
    Ok(result)
}

impl VirtioNet {
    /// 获取 `state` 自旋锁并采样当前收发缓冲池、硬件队列待处理项及观测计数器的完整快照。
    fn snapshot(&self) -> Result<Snapshot, TestError> {
        let state = self.state.lock();
        let empty = Pool {
            count: 0,
            free: 0,
            preparing: 0,
            owned: 0,
            processing: 0,
            pages: 0,
            heads: 0,
            device_pages: 0,
            addresses: [0; super::POOL_MAX_SLOTS],
        };
        let (rx, tx, pending_rx, pending_tx, status) = if let Some(inner) = state.inner.as_ref() {
            (
                pool(&inner.rx, &inner.rx_heads, inner.domain),
                pool(&inner.tx, &inner.tx_heads, inner.domain),
                inner.driver.pending_rx_used(),
                inner.driver.pending_tx_used(),
                Some(inner.driver.status()),
            )
        } else {
            (Ok(empty), Ok(empty), Ok(0), Ok(0), None)
        };
        let rings = state.inner.as_ref().map(|inner| {
            let rx = inner.rx_ring.region();
            let tx = inner.tx_ring.region();
            [(rx.device.value(), rx.len), (tx.device.value(), tx.len)]
        });
        let observation = &self.observation;
        Ok(Snapshot {
            rx: rx.map_err(|_| TestError("NIC RX ownership mismatch"))?,
            tx: tx.map_err(|_| TestError("NIC TX ownership mismatch"))?,
            pending_rx: pending_rx.map_err(|_| TestError("NIC RX pending span invalid"))?,
            pending_tx: pending_tx.map_err(|_| TestError("NIC TX pending span invalid"))?,
            status,
            rings,
            stopping: state.phase == Phase::Stopping,
            stopped: state.phase == Phase::Stopped,
            tx_completed: observation.tx_completed.load(Ordering::Acquire),
            rx_completed: observation.rx_completed.load(Ordering::Acquire),
            kicks: observation.space_kicks.load(Ordering::Acquire),
            budgets: observation.budget_exhausted.load(Ordering::Acquire),
            rechecks: observation.rearm_pending.load(Ordering::Acquire),
            entered: observation.work_entered.load(Ordering::Acquire),
            exited: observation.work_exited.load(Ordering::Acquire),
            interrupts: observation.interrupts.load(Ordering::Acquire),
            stopping_interrupts: observation.stopping_interrupts.load(Ordering::Acquire),
            barrier: observation.reset_barrier.load(Ordering::Acquire),
            unregistered: observation.unregistered.load(Ordering::Acquire),
            returned: observation.pages_returned.load(Ordering::Acquire),
        })
    }
}

/// 测试套件级共享资源夹具（Suite Fixture），负责在套件启动时探测一次硬件库存、解析宿主 UDP 端口并记录套件资源基线。
struct Inventory {
    /// 探测到的平台 VirtIO 设备库存（供各测试用例复用）。
    devices: Arc<DeviceInventory>,
    /// 整个测试套件运行前后的物理页、堆与任务资源基线守卫。
    baseline: SuiteBaseline,
    /// 从内核启动参数 `ktest-net-udp=<port>` 解析出的宿主 UDP 对端端口号。
    udp_port: u16,
}

impl Fixture for Inventory {
    /// 探测平台硬件设备库存、解析内核命令行中的宿主 UDP 端口，并采样测试套件初始资源基线。
    fn set_up(context: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        let devices = Arc::try_new(<DeviceInventory as Fixture>::set_up(context, &())?)
            .map_err(|_| TestError("native inventory owner OOM"))?;
        let udp_port = peer_port()?;
        let baseline = <SuiteBaseline as Fixture>::set_up(context, &())?;
        Ok(Self {
            devices,
            baseline,
            udp_port,
        })
    }

    /// 在套件全部测试用例执行完毕后，校验全局内存、DMA 页与任务计数是否回归套件初始基线。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        self.baseline.tear_down(context)
    }
}

/// 从内核启动参数（`bootargs`）中解析唯一的 `ktest-net-udp=<port>` 宿主 UDP 对端端口号（要求为非零 `u16`）。
fn peer_port() -> Result<u16, TestError> {
    let mut port = None;
    for token in crate::boot::info()
        .bootargs()
        .split(|byte| byte.is_ascii_whitespace())
    {
        if let Some(value) = token.strip_prefix(b"ktest-net-udp=") {
            if port.is_some() {
                return Err(TestError("native UDP port duplicated"));
            }
            let text =
                core::str::from_utf8(value).map_err(|_| TestError("native UDP port invalid"))?;
            if value.is_empty() || !value.iter().all(u8::is_ascii_digit) {
                return Err(TestError("native UDP port invalid"));
            }
            port = Some(
                text.parse::<u16>()
                    .map_err(|_| TestError("native UDP port invalid"))?,
            );
        }
    }
    port.filter(|port| *port != 0)
        .ok_or(TestError("native UDP port missing or zero"))
}

/// 单个测试用例级夹具（Case Fixture），负责独占初始化网络栈、VirtIO 网卡、绑定 UDP 端点以及按需准备三块 VirtIO 磁盘。
struct Owner {
    /// 当前测试用例启动前的内存、DMA 页与任务资源基线。
    before: Baseline,
    /// 指向套件级硬件设备库存的共享引用。
    inventory: Arc<DeviceInventory>,
    /// 保存当前网络栈独占拥有者 [`NetStackOwner`] 的自旋锁槽位（支持移交给子任务执行并发 `drop` 停机）。
    owner: Arc<SpinLock<Option<NetStackOwner>>>,
    /// 当前已启动的内核网络协议栈引用。
    net: Option<NetStackRef>,
    /// 当前已启动的 [`VirtioNet`] 网卡驱动实例引用。
    nic: Option<Arc<VirtioNet>>,
    /// 已绑定到 `10.0.2.15:0` 的测试用 IPv4 UDP 端点。
    udp: Option<Arc<UdpEndpoint>>,
    /// 宿主侧 UDP 响应服务监听的端口号（与 `10.0.2.2:<port>` 通信）。
    port: u16,
    /// 多设备共存测试中按需准备并启动的 VirtIO 块设备列表。
    disks: Vec<DiskRef>,
    /// 多设备共存测试中为各块设备分配的独立完成工作队列列表。
    queues: Vec<WorkQueue>,
    /// 多设备共存测试中三块磁盘各自对应的硬件中断号记录。
    disk_sources: [Option<irq::IrqNumber>; 3],
}

impl Fixture<Inventory> for Owner {
    /// 采样用例执行前资源基线，并基于套件库存初始化独立的 [`NetStack`]、[`VirtioNet`] 与 UDP 端点。
    fn set_up(_: &mut TestContext<'_>, resources: &Inventory) -> Result<Self, TestError> {
        let before = baseline();
        let holder =
            Arc::try_new(SpinLock::new(None)).map_err(|_| TestError("native owner storage OOM"))?;
        let mut fixture = Self {
            before,
            inventory: Arc::clone(&resources.devices),
            owner: holder,
            net: None,
            nic: None,
            udp: None,
            port: resources.udp_port,
            disks: Vec::new(),
            queues: Vec::new(),
            disk_sources: [None; 3],
        };
        fixture.prepare(&resources.devices)?;
        Ok(fixture)
    }

    /// 在测试框架等待子任务退出（Join）前放行全部观测停点，防止后台 `Work` 或停机子任务因停点未释放而死锁。
    fn quiesce(&mut self, _: &mut TestContext<'_>) -> TestResult {
        if let Some(nic) = &self.nic {
            nic.observation.release_all();
        }
        Ok(())
    }

    /// 在子任务全部汇合后同步停止网卡与所有块设备、关闭工作队列，并校验全局内存与元数据计数是否完全恢复至用例前基线。
    fn tear_down(mut self, context: &mut TestContext<'_>) -> TestResult {
        let before = self.before;
        self.stop_network();
        for disk in &self.disks {
            if let Err(error) = disk.stop() {
                context.diagnostic(format_args!("network-nic Disk stop-error={:?}", error));
                // 若块设备未能确认硬件复位，则直接调用 fatal 终止测试虚拟机，防止在 DMA 仍活跃时释放底层物理页。
                crate::ktest::fatal("native Disk stop did not converge");
            }
        }
        self.disks.clear();
        for queue in &self.queues {
            queue.close_sync();
        }
        self.queues.clear();

        drop(self);
        let after = baseline();
        context.diagnostic(format_args!(
            "network-nic resources-before={:?} resources-after={:?}",
            before, after
        ));
        context.expect_eq(after.0, before.0);
        context.expect_eq(after.1, before.1);
        if after != before {
            return Err(TestError("native NIC resources did not converge"));
        }
        Ok(())
    }
}

impl Owner {
    /// 基于硬件库存创建带 VirtIO 网卡的独立 [`NetStackOwner`]，配置静态 IPv4 地址 `10.0.2.15/24`（网关 `10.0.2.2`）并绑定测试 UDP 端点。
    fn prepare(&mut self, inventory: &DeviceInventory) -> TestResult {
        // SAFETY:
        // 1. 测试套件内的用例串行执行，且前序用例或本用例已通过 `stop_network` 彻底停止旧网卡并释放其全部强引用。
        // 2. `inventory` 来自启动期一次性探测结果，其登记的 MMIO/PCI 硬件窗口映射在整个内核生命周期内有效且当前无其他驱动并发访问。
        let owner = unsafe { NetStack::try_new_nic(inventory) }
            .map_err(|_| TestError("native NIC preparation failed"))?;
        owner
            .stack()
            .configure_ethernet(EthernetConfig {
                address: [10, 0, 2, 15],
                prefix_len: 24,
                gateway: Some([10, 0, 2, 2]),
            })
            .map_err(|_| TestError("native Ethernet configuration failed"))?;
        let udp = owner
            .stack()
            .try_new_udp(AddressFamily::Ipv4)
            .map_err(|_| TestError("native UDP preparation failed"))?;
        udp.bind(InetEndpoint {
            address: IpAddress::Ipv4(Ipv4Address::new(10, 0, 2, 15)),
            port: 0,
        })
        .map_err(|_| TestError("native UDP bind failed"))?;
        self.nic = Some(
            owner
                .stack()
                .ktest_virtio_nic()
                .cloned()
                .ok_or(TestError("native fixture requires a VirtIO NIC"))?,
        );
        self.net = Some(Arc::clone(owner.stack()));
        self.udp = Some(udp);
        *self.owner.lock() = Some(owner);
        Ok(())
    }

    /// 放行所有观测停点并同步销毁 [`NetStackOwner`] 及关联的 UDP、协议栈与网卡强引用，使硬件回到可重新初始化的静止状态。
    fn stop_network(&mut self) {
        if let Some(nic) = &self.nic {
            nic.observation.release_all();
        }
        let owner = self.owner.lock().take();
        drop(owner);
        self.udp.take();
        self.net.take();
        self.nic.take();
    }

    /// 获取当前用例已初始化的 [`VirtioNet`] 驱动实例引用。
    fn nic(&self) -> &Arc<VirtioNet> {
        self.nic.as_ref().expect("native NIC absent")
    }

    /// 通过已绑定的 UDP 端点向宿主对端 `10.0.2.2:<self.port>` 投递完整报文，并断言软件发送队列一次性接纳全部字节。
    fn send(&self, bytes: &[u8]) -> TestResult {
        let progress = self
            .udp
            .as_ref()
            .unwrap()
            .send(
                bytes,
                Some(InetEndpoint {
                    address: IpAddress::Ipv4(Ipv4Address::new(10, 0, 2, 2)),
                    port: self.port,
                }),
            )
            .map_err(|_| TestError("native UDP acceptance failed"))?;
        if progress != IoProgress::Ready(bytes.len()) {
            return Err(TestError(
                "native UDP TX record did not accept full message",
            ));
        }
        Ok(())
    }

    /// 轮询等待 UDP 端点出现 `POLLIN` 可读事件后读取一条报文，并校验源地址为 `10.0.2.2:<self.port>` 且报文未被截断。
    fn receive(&self) -> Result<([u8; 68], usize), TestError> {
        let udp = self.udp.as_ref().unwrap();
        observe(
            || {
                udp.poll(None)
                    .is_ok_and(|mask| mask.contains(PollMask::POLLIN))
            },
            "native UDP record missing",
        )?;
        let mut bytes = [0u8; 68];
        let record = udp
            .receive(&mut ReadBuffer::kernel(&mut bytes), false)
            .map_err(|_| TestError("native UDP receive failed"))?
            .ok_or(TestError("native UDP observed record disappeared"))?;
        if record.source
            != Some(SocketAddress::Inet(InetEndpoint {
                address: IpAddress::Ipv4(Ipv4Address::new(10, 0, 2, 2)),
                port: self.port,
            }))
            || record.copied != record.original_len
        {
            return Err(TestError("native UDP actual source or length differs"));
        }
        Ok((bytes, record.copied))
    }

    /// 向宿主发送 1 字节 `"P"` 探测包并等待回复 `"Q"`，完成 ARP 邻居解析预热并确认 RX/TX 队列待处理项排空。
    fn probe(&self) -> TestResult {
        self.send(b"P")?;
        let packet = self.receive()?;
        if &packet.0[..packet.1] != b"Q" {
            return Err(TestError("native probe bytes differ"));
        }
        observe(
            || {
                self.nic().snapshot().is_ok_and(|snapshot| {
                    snapshot.tx.free == snapshot.tx.count
                        && snapshot.pending_rx == 0
                        && snapshot.pending_tx == 0
                })
            },
            "native warm queue did not drain",
        )
    }
}

/// 以 1ms 休眠步长轮询条件闭包 `ready`（最长等待 5 秒），超时仍未满足则返回指定 `TestError`。
fn observe(mut ready: impl FnMut() -> bool, error: &'static str) -> TestResult {
    let end = time::now()
        .checked_add(time::Duration::from_secs(5))
        .ok_or(TestError("native observation deadline overflow"))?;
    while !ready() {
        if end <= time::now() {
            return Err(TestError(error));
        }
        timer::sleep(time::Duration::from_millis(1));
    }
    Ok(())
}

/// 直接通过 `VirtioNet::try_transmit` 构造并提交一个 42 字节的广播 ARP Reply 以太网帧，返回是否成功认领 TX 槽位。
fn arp_reply(nic: &VirtioNet) -> Result<bool, TestError> {
    let mac = nic.info().mac;
    nic.try_transmit(42, |frame| {
        let mut frame = EthernetFrame::new_unchecked(frame);
        EthernetRepr {
            src_addr: EthernetAddress(mac),
            dst_addr: EthernetAddress::BROADCAST,
            ethertype: EthernetProtocol::Arp,
        }
        .emit(&mut frame);
        ArpRepr::EthernetIpv4 {
            operation: ArpOperation::Reply,
            source_hardware_addr: EthernetAddress(mac),
            source_protocol_addr: Ipv4Address::new(10, 0, 2, 15),
            target_hardware_addr: EthernetAddress([0; 6]),
            target_protocol_addr: Ipv4Address::new(10, 0, 2, 2),
        }
        .emit(&mut ArpPacket::new_unchecked(frame.payload_mut()));
    })
    .map_err(|_| TestError("native ARP TX failed"))
}

/// 连续接收 `count` 条 68 字节的 `"NR"` 突发响应报文，校验每条报文的 16 位序号（`0..count` 不重不漏）及确定性载荷字节。
fn burst(fixture: &Owner, count: usize) -> TestResult {
    let mut seen = 0u32;
    for _ in 0..count {
        let packet = fixture.receive()?;
        let bytes = &packet.0[..packet.1];
        if bytes.len() != 68 || bytes[..2] != *b"NR" {
            return Err(TestError("native burst record length or marker differs"));
        }
        let index = usize::from(u16::from_le_bytes([bytes[2], bytes[3]]));
        if count <= index || seen & (1 << index) != 0 {
            return Err(TestError("native burst index missing or duplicated"));
        }
        if !bytes[4..]
            .iter()
            .enumerate()
            .all(|(at, byte)| *byte == (index * 17 + at * 31 + 7) as u8)
        {
            return Err(TestError("native burst full bytes differ"));
        }
        seen |= 1 << index;
    }
    let expected = if count == 32 {
        u32::MAX
    } else {
        (1 << count) - 1
    };
    if seen != expected {
        return Err(TestError("native burst index set incomplete"));
    }
    Ok(())
}

/// VirtIO 网卡硬件队列、并发停点与多设备共存集成测试套件。
#[::ktest::suite(fixture = Owner, suite_resources = Inventory, context = "Thread", tags("network-nic", "network-nic-native"), resources("net-user"), timeout_ms = 60000)]
pub(crate) mod native {
    use super::*;

    /// 测试目标：验证纯发送（TX-only）完成项在无接收流量和后续发送请求时，能由后台 `Work` 独立回收槽位并按以太网帧长（不含 `virtio_net_hdr`）更新统计。
    /// 场景步骤：
    /// 1. 布防 `entry` 停点后提交一个 42 字节 ARP 帧，等待后台 `Work` 停在入口且硬件写出 `pending_tx > 0`；断言此时槽位仍为 `DeviceOwned`、统计未更新。
    /// 2. 放行 `entry` 停点，等待后台 `Work` 消费 TX Used 环并归还槽位。
    /// 预期结果：`tx_completed` 与 `tx_packets` 各增加 1，`tx_bytes` 精确增加 42（不含 VirtIO 头部），TX 槽位全部恢复为 `Free`。
    #[test]
    #[ktest(tags("N20", "N28"))]
    fn tx_used_without_rx_or_new_send(
        context: &mut TestContext<'_>,
        fixture: &mut Owner,
    ) -> TestResult {
        let nic = fixture.nic();
        let before = nic.snapshot()?;
        let stats_before = fixture
            .net
            .as_ref()
            .unwrap()
            .statistics(InterfaceId::Ethernet)
            .map_err(|_| TestError("native TX statistics unavailable"))?;

        // Step 1: 布防 Work 入口停点并发送 42 字节 ARP 帧，在 Work 尚未消费 TX Used 环的窗口内验证槽位与统计状态。
        nic.observation.entry.arm();
        if !arp_reply(nic)? {
            return Err(TestError("native first ARP had no TX slot"));
        }
        observe(
            || nic.observation.entry.entered(),
            "native TX Work Entry not reached",
        )?;
        observe(
            || nic.snapshot().is_ok_and(|snapshot| 0 < snapshot.pending_tx),
            "native TX used not published",
        )?;
        let held = nic.snapshot()?;
        context.diagnostic(format_args!(
            "network-nic tx-before={:?} tx-held={:?}",
            before, held
        ));
        context.expect_eq(held.tx.owned, 1);
        context.expect_eq(held.tx.heads, 1);
        context.expect_eq(held.tx_completed, before.tx_completed);
        let stats_held = nic.statistics();
        context.expect_eq(stats_held.tx_bytes, stats_before.tx_bytes);
        context.expect_eq(stats_held.tx_packets, stats_before.tx_packets);

        // Step 2: 放行 Work 入口停点，验证 TX 槽位回收、唤醒计数递增以及按 42 字节以太网帧长统计。
        nic.observation.entry.release();
        observe(
            || {
                nic.snapshot().is_ok_and(|snapshot| {
                    before.tx_completed < snapshot.tx_completed
                        && before.kicks < snapshot.kicks
                        && snapshot.tx.free == snapshot.tx.count
                })
            },
            "native TX-only completion did not recover",
        )?;
        let after = nic.snapshot()?;
        let stats_after = fixture
            .net
            .as_ref()
            .unwrap()
            .statistics(InterfaceId::Ethernet)
            .map_err(|_| TestError("native completed TX statistics unavailable"))?;
        context.expect_eq(after.tx_completed.wrapping_sub(before.tx_completed), 1);
        context.expect_eq(stats_after.tx_bytes.wrapping_sub(stats_before.tx_bytes), 42);
        context.expect_eq(
            stats_after.tx_packets.wrapping_sub(stats_before.tx_packets),
            1,
        );
        context.diagnostic(format_args!(
            "network-nic tx-stats-before={:?} tx-stats-held={:?} tx-stats-after={:?} ethernet-bytes=42 virtio-header-bytes={} completions=1",
            stats_before, stats_held, stats_after, nic.info().header_len
        ));
        context.expect_eq(after.rx_completed, before.rx_completed);
        context.expect_eq(after.pending_tx, 0);
        context.expect_eq(after.tx.heads, 0);
        context.expect_eq(after.tx.pages, after.tx.count);
        context.diagnostic(format_args!("network-nic tx-after={:?}", after));
        Ok(())
    }

    /// 测试目标：验证当单波接收报文数量（32 条）超过单轮 `RX_BUDGET`（16 条）上限时，后台 `Work` 会自动续跑并完整交付全部 32 条 UDP 记录。
    /// 场景步骤：
    /// 1. 先执行 `probe` 预热 ARP，布防 `entry` 停点后向宿主发送 `"NB\x20\x00"` 请求 32 条突发报文，等待 RX Used 环积累至少 17 个待处理项。
    /// 2. 放行 `entry` 停点并调用 `burst(fixture, 32)` 接收全部 32 条报文。
    /// 预期结果：全部 32 条 68 字节记录完整无误到达，`budget_exhausted` 计数递增，且累计 `rx_completed` 增量不少于 32。
    #[test]
    #[ktest(tags("N21"))]
    fn rx_budget_continues_all_32_records(
        context: &mut TestContext<'_>,
        fixture: &mut Owner,
    ) -> TestResult {
        fixture.probe()?;
        let before = fixture.nic().snapshot()?;

        // Step 1: 布防 Work 入口停点并触发 32 条突发响应，等待硬件 RX Used 环积累超过单轮预算（16 条）的完成项。
        fixture.nic().observation.entry.arm();
        fixture.send(&[b'N', b'B', 32, 0])?;
        observe(
            || fixture.nic().observation.entry.entered(),
            "native burst Work Entry not reached",
        )?;
        observe(
            || {
                fixture
                    .nic()
                    .snapshot()
                    .is_ok_and(|snapshot| 17 <= snapshot.pending_rx)
            },
            "native burst pending never reached 17",
        )?;
        context.diagnostic(format_args!(
            "network-nic burst-held={:?}",
            fixture.nic().snapshot()?
        ));

        // Step 2: 放行入口停点并接收全部 32 条记录，验证单轮预算耗尽后触发自续跑且无丢包。
        fixture.nic().observation.entry.release();
        burst(fixture, 32)?;
        let after = fixture.nic().snapshot()?;
        context.expect(
            before.budgets < after.budgets,
            file!(),
            line!(),
            format_args!("RX16 budget was not exhausted"),
        );
        context.expect(
            32 <= after.rx_completed - before.rx_completed,
            file!(),
            line!(),
            format_args!("RX successor did not complete all records"),
        );
        context.diagnostic(format_args!(
            "network-nic burst-before={:?} burst-after={:?}",
            before, after
        ));
        Ok(())
    }

    /// 测试目标：验证在后台 `Work` 首次判空准备退出并重装队列中断的竞态窗口内，若新到达 RX Used 完成项，二次复查（Recheck）能够捕获并继续调度后续 `Work`。
    /// 场景步骤：
    /// 1. 布防 `before_rearm` 停点并发送 1 个 ARP 帧触发后台 `Work`，等待其消费完 TX 项后停在重装中断前（此时 `pending_rx == 0` 且 `pending_tx == 0`）。
    /// 2. 在该停点窗口内向宿主请求 1 条突发 UDP 报文 `"NB\x01\x00"`，等待硬件写出 `pending_rx > 0`。
    /// 3. 放行 `before_rearm` 停点并接收该条报文。
    /// 预期结果：`rearm_pending` 二次复查命中计数递增，后台 `Work` 再次进入执行（`entered` 递增），且突发报文完整送达。
    #[test]
    #[ktest(tags("N21"))]
    fn rearm_rechecks_real_pending_successor(
        context: &mut TestContext<'_>,
        fixture: &mut Owner,
    ) -> TestResult {
        fixture.probe()?;
        let before = fixture.nic().snapshot()?;

        // Step 1: 布防 before_rearm 停点并触发一轮 Work，使其在排空队列后停在重装中断前的窗口。
        fixture.nic().observation.before_rearm.arm();
        if !arp_reply(fixture.nic())? {
            return Err(TestError("native rearm trigger full"));
        }
        observe(
            || fixture.nic().observation.before_rearm.entered(),
            "native BeforeRearm not reached",
        )?;
        let empty = fixture.nic().snapshot()?;
        context.expect_eq(empty.pending_rx, 0);
        context.expect_eq(empty.pending_tx, 0);

        // Step 2: 在重装中断前的窗口内向宿主请求 1 条突发报文，等待硬件向 RX Used 环写入新完成项。
        fixture.send(&[b'N', b'B', 1, 0])?;
        observe(
            || {
                fixture
                    .nic()
                    .snapshot()
                    .is_ok_and(|snapshot| 0 < snapshot.pending_rx)
            },
            "native rearm-window RX used missing",
        )?;
        context.diagnostic(format_args!(
            "network-nic rearm-held={:?}",
            fixture.nic().snapshot()?
        ));

        // Step 3: 放行停点，验证二次复查捕获到新完成项并调度后继 Work 完成收包。
        fixture.nic().observation.before_rearm.release();
        burst(fixture, 1)?;
        let after = fixture.nic().snapshot()?;
        context.expect(
            before.rechecks < after.rechecks,
            file!(),
            line!(),
            format_args!("real rearm pending was false"),
        );
        context.expect(
            empty.entered < after.entered,
            file!(),
            line!(),
            format_args!("original Work successor missing"),
        );
        Ok(())
    }

    /// 测试目标：验证 VirtIO 网卡与三块 VirtIO 磁盘（`root`、`extra`、`scratch`）及串口同时工作且共享外部中断线（LoongArch PCI INTx）时的中断分发与独立注销行为。
    /// 场景步骤：
    /// 1. 准备并启动三块 VirtIO 磁盘，校验各设备硬件身份与中断号（LoongArch 下网卡与 `root` 盘共享同一条 PCI INTx 中断线）。
    /// 2. 并发执行网卡 UDP 探测、`root`/`extra` 盘扇区读取与 `scratch` 盘 512 字节读写往返校验。
    /// 3. 调用 `stop_network` 同步停止网卡并注销其中断处理函数，再次对三块磁盘执行读写校验。
    /// 预期结果：网卡停机后对应中断源上的注册 Action 数量精确减 1，网卡物理页全部归还，且共享中断线上的其余块设备仍能正常完成中断驱动的块读写。
    #[test]
    #[ktest(tags("N22"), resources("root", "extra", "scratch"))]
    fn nic_three_disks_uart_and_shared_irq(
        context: &mut TestContext<'_>,
        fixture: &mut Owner,
    ) -> TestResult {
        // Step 1: 初始化三块磁盘并验证网卡与磁盘在当前架构下的总线身份、中断号及并发收发/读写能力。
        let inventory = Arc::clone(&fixture.inventory);
        fixture.prepare_disks(&inventory)?;
        fixture.check_identities(context, &inventory)?;
        fixture.probe()?;
        let first = fixture.read_disk(0, 0)?;
        let second = fixture.read_disk(1, 0)?;
        fixture.scratch_roundtrip(context)?;
        context.diagnostic(format_args!(
            "network-nic UART coexistence root={:?} extra={:?} scratch={:?} nic={:?}",
            fixture.disks[0].identity(),
            fixture.disks[1].identity(),
            fixture.disks[2].identity(),
            fixture.nic().identity()
        ));

        // Step 2: 停止网卡并注销其中断处理函数，验证共享中断线上的块设备不受影响且继续正常读写。
        let nic = Arc::clone(fixture.nic());
        let source = nic.interrupt();
        let actions = irq::test_action_count(source);
        fixture.stop_network();
        context.expect_eq(irq::test_action_count(source) + 1, actions);
        let stopped = nic.snapshot()?;
        context.expect_eq(stopped.unregistered, true);
        context.expect_eq(stopped.returned, true);
        context.expect_eq(fixture.read_disk(0, 0)?, first);
        context.expect_eq(fixture.read_disk(1, 0)?, second);
        fixture.scratch_roundtrip(context)?;
        context.diagnostic(format_args!(
            "network-nic UART after NIC unregister; remaining-actions={}",
            irq::test_action_count(source)
        ));
        Ok(())
    }

    /// 测试目标：验证 `stop_sync` 停机流程严格遵循“先等待后台 `Work` 彻底退出、再轮询硬件复位确认并执行内存屏障、最后注销中断并归还 DMA 物理页”的时序契约。
    /// 场景步骤：
    /// 1. 同时布防 `entry` 与 `after_reset` 停点，发送 ARP 帧使后台 `Work` 停在入口（`entered == exited + 1`）。
    /// 2. 派生子任务销毁 `NetStackOwner` 触发 `stop_sync`，观测到网卡进入 `Stopping` 且子任务因等待 `Work` 退出而阻塞（此时硬件尚未复位、物理页未归还）。
    /// 3. 放行 `entry` 停点，等待子任务推进至硬件复位确认后的 `after_reset` 停点，校验此时 `Work` 已退出（`entered == exited`）、`status == 0`、`barrier == true`，但中断尚未注销且 RX/TX 物理页及 DMA 映射仍保持原状。
    /// 4. 放行 `after_reset` 停点并等待子任务完成退出。
    /// 预期结果：最终网卡进入 `Stopped` 阶段，中断已注销，所有 RX/TX 缓冲页与控制环物理页全部归还。
    #[test]
    #[ktest(tags("N23"))]
    fn stop_waits_work_tail_then_reset_before_page_return(
        context: &mut TestContext<'_>,
        fixture: &mut Owner,
    ) -> TestResult {
        // Step 1: 布防 entry 与 after_reset 停点，触发后台 Work 使其停在入口处。
        let nic = Arc::clone(fixture.nic());
        nic.observation.entry.arm();
        nic.observation.after_reset.arm();
        if !arp_reply(&nic)? {
            return Err(TestError("native stop trigger full"));
        }
        observe(
            || nic.observation.entry.entered(),
            "native stop Work Entry missing",
        )?;
        observe(
            || nic.snapshot().is_ok_and(|snapshot| 0 < snapshot.pending_tx),
            "native stop TX used missing",
        )?;

        // Step 2: 在子任务中触发 stop_sync，验证停机流程首先阻塞等待后台 Work 退出，期间不提前复位硬件或收回物理页。
        let task_slot = Arc::try_new(SpinLock::new(None))
            .map_err(|_| TestError("native stop Task observer OOM"))?;
        let task_writer = Arc::clone(&task_slot);
        let holder = Arc::clone(&fixture.owner);
        let child = crate::ktest::spawn_result(move |_| {
            *task_writer.lock() = Some(task::current_task());
            let owner = holder
                .lock()
                .take()
                .ok_or(TestError("native stop owner missing"))?;
            drop(owner);
            Ok(0)
        })?;
        observe(
            || {
                nic.snapshot().is_ok_and(|snapshot| snapshot.stopping)
                    && task_slot
                        .lock()
                        .as_ref()
                        .is_some_and(|task| sched::test_state(task) == task::State::Blocked)
            },
            "native stop Task did not block on Work tail",
        )?;
        let held = nic.snapshot()?;
        context.expect_eq(child.try_join(), None);
        context.expect_eq(held.barrier, false);
        context.expect_eq(held.returned, false);
        context.expect_eq(held.tx.owned, 1);
        context.expect_eq(held.rx.owned, held.rx.count);
        context.expect_eq(held.tx.device_pages, held.tx.owned);
        context.expect_eq(held.rx.device_pages, held.rx.owned);
        context.expect_eq(held.entered, held.exited + 1);

        // Step 3: 放行 Work 入口停点使其退出，验证子任务推进至 after_reset 停点时已完成硬件复位屏障，但尚未释放任何 DMA 物理页。
        nic.observation.entry.release();
        observe(
            || nic.observation.after_reset.entered(),
            "native AfterReset not reached",
        )?;
        let reset = nic.snapshot()?;
        context.expect_eq(reset.status, Some(DeviceStatus::empty()));
        context.expect_eq(reset.barrier, true);
        context.expect_eq(reset.entered, reset.exited);
        context.expect_eq(reset.returned, false);
        context.expect_eq(reset.unregistered, false);
        context.expect_eq(reset.rx.pages, held.rx.pages);
        context.expect_eq(reset.tx.pages, held.tx.pages);
        context.expect_eq(reset.tx.owned, held.tx.owned);
        context.expect_eq(reset.rx.addresses, held.rx.addresses);
        context.expect_eq(reset.tx.addresses, held.tx.addresses);
        context.expect_eq(reset.rx.device_pages, held.rx.device_pages);
        context.expect_eq(reset.tx.device_pages, held.tx.device_pages);
        context.expect_eq(reset.rings, held.rings);
        context.expect_eq(child.try_join(), None);
        context.diagnostic(format_args!(
            "network-nic stop-held={:?} reset-held={:?}",
            held, reset
        ));

        // Step 4: 放行 after_reset 停点，验证子任务完成中断注销与全部缓冲池、控制环物理页回收。
        nic.observation.after_reset.release();
        context.expect_eq(child.join()?, 0);
        let stopped = nic.snapshot()?;
        context.expect_eq(stopped.stopped, true);
        context.expect_eq(stopped.unregistered, true);
        context.expect_eq(stopped.returned, true);
        context.expect_eq(stopped.rx.pages + stopped.tx.pages, 0);
        context.expect_eq(stopped.rings, None);
        context.diagnostic(format_args!(
            "network-nic stopped={:?}; reset rejection and StopUnconfirmed were not exercised",
            stopped
        ));
        Ok(())
    }

    /// 测试目标：验证当 32 个硬件 TX 槽位全部占满时，`try_transmit` 拒绝新帧且不调用帧构造闭包；同时上层软件发送队列可缓存 32 条待发报文、对第 33 条返回 `WouldBlock` 并完整归还 `OwnedPacket`，待硬件槽位回收后自动排空软件队列并允许重发第 33 条报文。
    /// 场景步骤：
    /// 1. 预热 ARP 后布防 `entry` 停点，连续提交 32 个 ARP 帧占满全部硬件 TX 槽位；验证第 33 次 `try_transmit` 返回 `Ok(false)` 且不执行写入闭包。
    /// 2. 向 `NetStack` 软件发送队列连续提交 32 条 `"NP"` UDP 报文占满软件配额；验证第 33 条 `"NP"` 报文返回 `Err((FsError::WouldBlock, packet))` 且归还的报文内容完好、端点进入等待队列。
    /// 3. 放行 `entry` 停点，接收前 32 条 `"NQ"` 响应并等待软件发送队列恢复可写。
    /// 4. 直接将先前退回的第 33 个 `OwnedPacket` 重新提交给 `NetStack::try_emit` 并接收其 `"NQ"` 响应。
    /// 预期结果：总共完成 65 次硬件 TX 发送（32 个 ARP + 33 个 UDP），32 个硬件 TX 槽位全部恢复为 `Free`。
    #[test]
    #[ktest(tags("N24"))]
    fn tx_32_slots_full_preserves_writer_then_recovers(
        context: &mut TestContext<'_>,
        fixture: &mut Owner,
    ) -> TestResult {
        fixture.probe()?;
        let nic = fixture.nic();
        let net = fixture.net.as_ref().unwrap();
        let udp = fixture.udp.as_ref().unwrap();
        let destination = InetEndpoint {
            address: IpAddress::Ipv4(Ipv4Address::new(10, 0, 2, 2)),
            port: fixture.port,
        };
        let before = nic.snapshot()?;
        if before.tx.count != 32 {
            return Err(TestError("native TX hardware pool is not 32 slots"));
        }

        // Step 1: 暂停后台 Work 并连续提交 32 个 ARP 帧占满硬件 TX 缓冲池，验证满池时 try_transmit 不调用 writer 闭包。
        nic.observation.entry.arm();
        if !arp_reply(nic)? {
            return Err(TestError("native full first TX rejected"));
        }
        observe(
            || nic.observation.entry.entered(),
            "native full Work Entry missing",
        )?;
        for _ in 1..32 {
            if !arp_reply(nic)? {
                return Err(TestError("native TX pool filled before 32"));
            }
        }
        let called = AtomicBool::new(false);
        context.expect_eq(
            nic.try_transmit(42, |_| {
                called.store(true, Ordering::Release);
            })
            .map_err(|_| TestError("native full TX error"))?,
            false,
        );
        context.expect_eq(called.load(Ordering::Acquire), false);
        let held = nic.snapshot()?;
        context.expect_eq(held.tx.owned, 32);
        context.expect_eq(held.tx.heads, 32);
        context.expect_eq(held.tx.free, 0);

        // Step 2: 在硬件 TX 池满的情况下向协议栈软件队列投递 32 条 UDP 报文，并验证第 33 条报文触发 WouldBlock 且原样退回 OwnedPacket。
        for index in 0..32u16 {
            let [low, high] = index.to_le_bytes();
            let packet = OwnedPacket::try_from_udp(
                udp.local().unwrap(),
                destination,
                &[b'N', b'P', low, high],
                1500,
            )
            .map_err(|_| TestError("native full software UDP preparation failed"))?;
            net.try_emit(packet)
                .map_err(|_| TestError("native software queue filled before 32 records"))?;
        }
        let packet = OwnedPacket::try_from_udp(
            udp.local().unwrap(),
            destination,
            &[b'N', b'P', 32, 0],
            1500,
        )
        .map_err(|_| TestError("native unaccepted UDP preparation failed"))?;
        let original_len = packet.len();
        let returned = match net.try_emit(packet) {
            Err((FsError::WouldBlock, packet)) => packet,
            Err((error, _packet)) => {
                context.diagnostic(format_args!("network-nic software-full error={:?}", error));
                return Err(TestError("native full software queue returned wrong error"));
            }
            Ok(()) => return Err(TestError("native full software queue accepted 33rd record")),
        };
        context.expect_eq(returned.len(), original_len);
        context.expect_eq(
            returned
                .udp_payload()
                .map_err(|_| TestError("native returned UDP invalid"))?,
            &[b'N', b'P', 32, 0][..],
        );
        context.expect_eq(
            net.egress_ready(udp.egress_wait())
                .map_err(|_| TestError("native full capacity query failed"))?,
            false,
        );
        context.expect_eq(net.egress_waiting(udp.id()), true);
        context.diagnostic(format_args!(
            "network-nic software-full accepted=32 returned-index=32 retained-binding={:?} TX-held={:?}",
            udp.local(), nic.snapshot()?
        ));

        // Step 3: 放行后台 Work 排空硬件与软件发送队列，验证全部 32 条 UDP 响应到达且端点发送等待状态解除。
        nic.observation.entry.release();
        let mut seen = 0u32;
        for _ in 0..32 {
            let response = fixture.receive()?;
            let bytes = &response.0[..response.1];
            if bytes.len() != 4 || bytes[..2] != *b"NQ" {
                return Err(TestError("native full software response bytes differ"));
            }
            let index = u16::from_le_bytes([bytes[2], bytes[3]]);
            if 32 <= index || seen & (1u32 << index) != 0 {
                return Err(TestError(
                    "native full software response index missing or duplicated",
                ));
            }
            let [low, high] = index.to_le_bytes();
            if bytes != [b'N', b'Q', low, high] {
                return Err(TestError(
                    "native full software response full record differs",
                ));
            }
            seen |= 1u32 << index;
        }
        context.expect_eq(seen, u32::MAX);
        observe(
            || net.egress_ready(udp.egress_wait()).is_ok_and(|ready| ready),
            "native software capacity did not recover",
        )?;
        context.expect_eq(net.egress_waiting(udp.id()), false);

        // Step 4: 将先前退回的第 33 个 OwnedPacket 直接重新提交发送，验证其成功送达且总计 65 次 TX 槽位全部回收。
        net.try_emit(returned)
            .map_err(|_| TestError("native recovered software queue rejected original packet"))?;
        let response = fixture.receive()?;
        context.expect_eq(&response.0[..response.1], &[b'N', b'Q', 32, 0][..]);
        observe(
            || {
                nic.snapshot().is_ok_and(|snapshot| {
                    snapshot.tx_completed == before.tx_completed + 65 && snapshot.tx.free == 32
                })
            },
            "native full hardware and software completion did not recover",
        )?;
        context.diagnostic(format_args!(
            "network-nic full-held={:?} recovered={:?}",
            held,
            nic.snapshot()?
        ));
        Ok(())
    }

    /// 测试目标：验证在物理页与堆内存耗尽（OOM）条件下调用 `VirtioNet::prepare` 失败时，驱动不会设置 `DRIVER_OK` 或泄漏半初始化内存与中断注册；解除内存压力后同一硬件可重新初始化并正常收发。
    /// 场景步骤：
    /// 1. 停止当前用例的默认网卡并确认其 `Arc` 强引用计数归零。
    /// 2. 使用 `HeapPressure` 耗尽内核堆与物理页，调用 `VirtioNet::prepare` 触发 `VirtioInitError::NoMemory`。
    /// 3. 释放内存压力并回收缓存页，校验硬件状态未置位 `DRIVER_OK`、内存统计与中断注册数无泄漏。
    /// 4. 在同一硬件库存上重新调用 `fixture.prepare` 并执行 `probe` 收发验证。
    /// 预期结果：OOM 失败路径完整回滚所有已分配资源，恢复内存后网卡可正常重初始化并完成 UDP 探测。
    #[test]
    #[ktest(tags("N24"))]
    fn real_no_memory_unpublished_cleanup_and_reprepare(
        context: &mut TestContext<'_>,
        fixture: &mut Owner,
    ) -> TestResult {
        // Step 1: 停止并销毁当前夹具持有的网卡与协议栈实例，确保全部强引用已释放。
        let old_net = Arc::downgrade(fixture.net.as_ref().unwrap());
        let old_nic = Arc::downgrade(fixture.nic());
        let source = fixture.nic().interrupt();
        fixture.stop_network();
        if old_net.upgrade().is_some() || old_nic.upgrade().is_some() {
            return Err(TestError("native old NIC strong owner survived stop"));
        }
        drop((old_net, old_nic));

        // Step 2: 施加堆与物理页耗尽压力，验证 VirtioNet::prepare 返回 NoMemory 且未将硬件置为 DRIVER_OK。
        let actions = irq::test_action_count(source);
        let inventory = Arc::clone(&fixture.inventory);
        let root =
            NetStack::try_new_lo().map_err(|_| TestError("native OOM root preparation failed"))?;
        let mut pressure = crate::cache::ktest::HeapPressure::prepare()?;
        let exhausted = pressure.exhaust();
        let before = mm::stats();
        // SAFETY:
        // 1. Step 1 已通过 `Weak::upgrade` 确认旧网卡实例的强引用彻底归零，当前无其他驱动实例访问该网卡硬件。
        // 2. `root` 协议栈已在耗尽内存前创建完毕并保持存活，`inventory` 提供的第 0 号网卡永久映射有效。
        let result = unsafe { VirtioNet::prepare(root.stack(), &inventory, 0) };
        let failure = match result {
            Err(VirtioInitError::NoMemory { status }) => Ok(status),
            Err(error) => {
                context.diagnostic(format_args!("network-nic OOM wrong-error={:?}", error));
                Err(TestError("native OOM did not return NIC NoMemory"))
            }
            Ok(nic) => {
                drop(nic);
                Err(TestError("native OOM unexpectedly prepared NIC"))
            }
        };
        mm::shrink_all();
        mm::drain_all_pages();
        let after = mm::stats();
        let actions_after = irq::test_action_count(source);

        drop(pressure);
        drop(root);
        context.expect_eq(exhausted.0, true);
        context.expect_eq(exhausted.1, true);
        let status = failure?;
        context.expect_eq(status.contains(DeviceStatus::DRIVER_OK), false);
        context.expect_eq(after, before);
        context.expect_eq(actions_after, actions);
        context.diagnostic(format_args!(
            "network-nic real-OOM={:?} status={:?} unpublished-before={:?} unpublished-after={:?}",
            exhausted, status, before, after
        ));

        // Step 3: 释放内存压力后在同一硬件上重新初始化网卡，验证 UDP 收发探测正常。
        fixture.prepare(&inventory)?;
        fixture.probe()?;
        fixture.stop_network();
        Ok(())
    }
}

impl Owner {
    /// 基于硬件库存准备并启动三块 VirtIO 磁盘（`root`、`extra`、`scratch`）及其专属工作队列，将所有权保存在当前 `Owner` 夹具中。
    fn prepare_disks(&mut self, inventory: &DeviceInventory) -> TestResult {
        self.disks
            .try_reserve_exact(3)
            .map_err(|_| TestError("native Disk owner slots OOM"))?;
        self.queues
            .try_reserve_exact(3)
            .map_err(|_| TestError("native Disk queue slots OOM"))?;
        let mut devices = Vec::new();
        devices
            .try_reserve_exact(3)
            .map_err(|_| TestError("native device slots OOM"))?;

        // Step 1: 收集架构对应的 VirtIO 块设备传输层实例（RISC-V 使用 MMIO，LoongArch 使用 PCI）。
        for candidate in &inventory.mmio_blocks {
            // SAFETY:
            // 1. 测试用例通过 `resources("root", "extra", "scratch")` 声明独占这三块磁盘，无其他并发使用者。
            // 2. `candidate` 来自启动期探测库存，其 MMIO 寄存器物理窗口在内核恒等映射中完整有效。
            devices.push(
                unsafe { VirtioDevice::prepare_mmio(candidate) }
                    .map_err(|_| TestError("native MMIO disk preparation failed"))?,
            );
        }
        for device in inventory.pci.devices().iter().filter(|device| {
            device.vendor_id == VIRTIO_VENDOR_ID
                && matches!(
                    device.device_id,
                    VIRTIO_PCI_BLOCK_LEGACY | VIRTIO_PCI_BLOCK_MODERN
                )
        }) {
            // SAFETY:
            // 1. 过滤出的 VirtIO 块设备 PCI BDF 由当前测试用例独占，与已启动的网卡位于不同的 PCI 设备槽位。
            // 2. 启动期 PCI 枚举已为各块设备分配并映射了有效的 BAR 空间。
            devices.push(
                unsafe { VirtioDevice::prepare_pci(&inventory.pci, device) }
                    .map_err(|_| TestError("native PCI disk preparation failed"))?,
            );
        }
        if devices.len() != 3 {
            return Err(TestError("native coexistence requires exactly three disks"));
        }

        // Step 2: 为每块磁盘创建独立的工作队列与 Disk 实例，随后统一启动。
        for (index, device) in devices.into_iter().enumerate() {
            self.disk_sources[index] = Some(device.interrupt());
            let queue = WorkQueue::try_new(WorkQueueConfig {
                max_active: usize::from(device.info().tag_count) + 1,
                ordered: false,
                reclaim: true,
            })
            .map_err(|_| TestError("native Disk domain OOM"))?;
            self.queues.push(queue);
            let disk = Disk::prepare(device, self.queues.last().unwrap())
                .map_err(|_| TestError("native Disk preparation failed"))?;
            self.disks.push(disk);
        }

        for disk in &self.disks {
            disk.start()
                .map_err(|_| TestError("native Disk start failed"))?;
        }
        Ok(())
    }

    /// 校验三块磁盘与网卡的硬件身份标识（MMIO 物理基址或 PCI BDF）、MAC 地址及中断号（含 LoongArch 下网卡与 `root` 盘共享 PCI INTx 中断线）。
    fn check_identities(
        &self,
        context: &mut TestContext<'_>,
        inventory: &DeviceInventory,
    ) -> TestResult {
        #[cfg(target_arch = "riscv64")]
        {
            let [root, extra, scratch] = inventory.mmio_blocks.as_slice() else {
                return Err(TestError("native RV disk inventory differs"));
            };
            for (index, (disk, candidate)) in
                self.disks.iter().zip([root, extra, scratch]).enumerate()
            {
                context.expect_eq(
                    disk.identity(),
                    DiskIdentity::Virtio(VirtioIdentity::Mmio {
                        physical: candidate.registers.start,
                        version: candidate.version,
                        vendor_id: candidate.vendor_id,
                    }),
                );
                context.expect_eq(self.disk_sources[index], Some(candidate.interrupt));
            }
            let [network] = inventory.mmio_networks.as_slice() else {
                return Err(TestError("native RV NIC inventory differs"));
            };
            context.expect_eq(
                self.nic().identity(),
                VirtioIdentity::Mmio {
                    physical: network.registers.start,
                    version: network.version,
                    vendor_id: network.vendor_id,
                },
            );
            context.expect_eq(self.nic().interrupt(), network.interrupt);
            for (slot, candidate) in [root, extra, scratch].iter().enumerate() {
                context.expect_eq(candidate.registers.start, 0x1000_1000 + slot * 0x1000);
            }
            context.expect_eq(network.registers.start, 0x1000_4000);
        }
        #[cfg(target_arch = "loongarch64")]
        {
            for (index, disk) in self.disks.iter().enumerate() {
                match disk.identity() {
                    DiskIdentity::Virtio(VirtioIdentity::Pci {
                        bus,
                        device,
                        function,
                        ..
                    }) => context.expect_eq((bus, device, function), (0, 5 + index as u8 * 4, 0)),
                    _ => return Err(TestError("native LA disk is not PCI")),
                }
                let device = inventory
                    .pci
                    .devices()
                    .iter()
                    .find(|device| {
                        device.bus == 0
                            && device.device == 5 + index as u8 * 4
                            && device.function == 0
                    })
                    .ok_or(TestError("native disk PCI inventory missing"))?;
                context.expect_eq(self.disk_sources[index], device.interrupt);
            }
            match self.nic().identity() {
                VirtioIdentity::Pci {
                    bus,
                    device,
                    function,
                    ..
                } => context.expect_eq((bus, device, function), (0, 0x11, 0)),
                _ => return Err(TestError("native LA NIC is not PCI")),
            }
            let root = inventory
                .pci
                .devices()
                .iter()
                .find(|device| device.bus == 0 && device.device == 5 && device.function == 0)
                .ok_or(TestError("native root PCI inventory missing"))?;
            context.expect_eq(
                self.nic().interrupt(),
                root.interrupt
                    .ok_or(TestError("native root INTx missing"))?,
            );
            context.expect_eq(self.disk_sources[0], Some(self.nic().interrupt()));
        }
        context.expect_eq(self.nic().info().mac, [0x52, 0x54, 0, 0x12, 0x34, 0x56]);
        context.diagnostic(format_args!(
            "network-nic inventory disk-sources={:?} NIC-source={:?}",
            self.disk_sources,
            self.nic().interrupt()
        ));
        Ok(())
    }

    /// 向第 `index` 块磁盘的指定 `sector` 扇区提交单扇区（512 字节）同步读写 `Bio`，等待完成后收回 DMA 物理页并返回其中的 512 字节数据。
    fn io(
        &self,
        index: usize,
        operation: BlkOperation,
        sector: u64,
        input: Option<&[u8; 512]>,
    ) -> Result<[u8; 512], TestError> {
        let disk = &self.disks[index];
        if disk.limits().logical_block_size != 512 {
            return Err(TestError("native coexistence requires logical block 512"));
        }
        // Step 1: 分配单张清零物理页并建立 512 字节的 DMA 映射；若是写操作则预先填入待写数据。
        let pages = mm::alloc_pages(0, true).ok_or(TestError("native Bio page OOM"))?;
        let mut mapped = MappedPages::new(
            pages,
            0,
            512,
            if operation == BlkOperation::Write {
                BufferDirection::DriverToDevice
            } else {
                BufferDirection::DeviceToDriver
            },
        )
        .map_err(|_| TestError("native Bio mapping failed"))?;
        if let Some(input) = input {
            for (target, source) in mapped.bytes_mut().iter_mut().zip(input) {
                target.write(*source);
            }
        }

        // Step 2: 构造并提交单扇区 Bio 请求，阻塞等待硬件 I/O 完成。
        let mut memory = Vec::new_in(BioAllocator::Heap);
        memory
            .try_reserve_exact(1)
            .map_err(|_| TestError("native Bio memory slots OOM"))?;
        memory.push(mapped);
        // SAFETY:
        // 1. `pages` 是刚刚通过 `mm::alloc_pages(0, true)` 独占分配并清零的物理页，写请求已在上方完成全部 512 字节初始化。
        // 2. `mapped` 移入 `Bio` 后直至 `completion.take_memory()` 收回前不存在任何 CPU 别名访问。
        let bio = unsafe { Bio::try_new(disk, operation, sector, memory, 0, None) }
            .map_err(|_| TestError("native Bio preparation failed"))?;
        let completion = bio.completion();
        disk.submit_bio(bio);
        let result = completion.wait();

        if result
            != (IoResult {
                completed: 512,
                error: None,
            })
        {
            return Err(TestError("native real disk I/O failed"));
        }
        // Step 3: 确认 512 字节传输无误后从完成句柄取回 CPU 拥有的物理页映射，复制出 512 字节载荷。
        let memory = completion
            .take_memory()
            .map_err(|_| TestError("native Bio reclaim failed"))?;
        let mut output = [0; 512];
        // SAFETY:
        // 1. `completion.wait()` 与 `take_memory()` 已保证设备 DMA 访问结束且物理页访问权已交还 CPU。
        // 2. 底层物理页在分配时已清零且长度为 512 字节，读取 `memory[0].bytes()` 不会触及未初始化内存或并发写冲突。
        output.copy_from_slice(unsafe { memory[0].bytes() });
        Ok(output)
    }

    /// 读取第 `index` 块磁盘指定 `sector` 扇区的 512 字节内容。
    fn read_disk(&self, index: usize, sector: u64) -> Result<[u8; 512], TestError> {
        self.io(index, BlkOperation::Read, sector, None)
    }

    /// 向 `scratch` 磁盘（索引 2）的第 128 扇区写入 512 字节确定性校验图样，随后独立读回并断言逐字节一致。
    fn scratch_roundtrip(&self, context: &mut TestContext<'_>) -> TestResult {
        let expected = core::array::from_fn(|index| (index * 29 + (index >> 3) + 61) as u8);
        self.io(2, BlkOperation::Write, 128, Some(&expected))?;
        let actual = self.read_disk(2, 128)?;
        context.expect_eq(actual, expected);
        Ok(())
    }
}
