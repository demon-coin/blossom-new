//! 真实网络队列、原 Work 停点和同库存设备的具名消费者。

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

const GATE_UNSET: u8 = 0;
const GATE_ARMED: u8 = 1;
const GATE_ENTERED: u8 = 2;
const GATE_RELEASED: u8 = 3;
/// 一个原子保存未设、已设、已进入和已释放事实，默认不阻塞生产 Work。
pub(super) struct Gate(AtomicU8);
impl Gate {
    /// 停点初态不改变原有执行路径。
    const fn new() -> Self {
        Self(AtomicU8::new(GATE_UNSET))
    }
    /// 普通测试控制者为下一次实际执行设置一次停点。
    fn arm(&self) {
        self.0.store(GATE_ARMED, Ordering::Release);
    }
    /// 仅在实际停点执行后返回 true。
    fn entered(&self) -> bool {
        self.0.load(Ordering::Acquire) == GATE_ENTERED
    }
    /// 清理和正文共用此释放动作，不触碰生产队列票。
    fn release(&self) {
        self.0.store(GATE_RELEASED, Ordering::Release);
    }
    /// 调用点没有短锁，等待沿真实 timer 与 scheduler，设备继续生产 used。
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

pub(super) struct Observation {
    pub(super) entry: Gate,
    pub(super) before_rearm: Gate,
    pub(super) after_reset: Gate,
    pub(super) tx_completed: AtomicUsize,
    pub(super) rx_completed: AtomicUsize,
    pub(super) space_kicks: AtomicUsize,
    pub(super) budget_exhausted: AtomicUsize,
    pub(super) rearm_pending: AtomicUsize,
    work_entered: AtomicUsize,
    work_exited: AtomicUsize,
    interrupts: AtomicUsize,
    stopping_interrupts: AtomicUsize,
    reset_barrier: AtomicBool,
    pub(super) unregistered: AtomicBool,
    pub(super) pages_returned: AtomicBool,
}
impl Observation {
    /// 计数只由真实事件增加，不设置期待值或影子完成。
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
    /// 当前原 Work 已开始执行，退出凭证覆盖 Entry 等待及全部处理尾部。
    pub(super) fn enter_work(&self) -> WorkTail<'_> {
        self.work_entered.fetch_add(1, Ordering::Relaxed);
        let tail = WorkTail(self);
        self.entry.stop();
        tail
    }
    /// 只记录 ISR 已真实确认本设备原因后的执行路径。
    pub(super) fn interrupt(&self, live: bool) {
        self.interrupts.fetch_add(1, Ordering::Relaxed);
        if !live {
            self.stopping_interrupts.fetch_add(1, Ordering::Relaxed);
        }
    }
    /// 仅从 status0 读回及完整设备屏障之后调用。
    pub(super) fn reset_confirmed(&self) {
        self.reset_barrier.store(true, Ordering::Release);
    }
    /// 所有错误清理都先解除原 Work 与 reset 尾部的停点。
    fn release_all(&self) {
        self.entry.release();
        self.before_rearm.release();
        self.after_reset.release();
    }
}
pub(super) struct WorkTail<'a>(&'a Observation);
impl Drop for WorkTail<'_> {
    /// 原回调正常返回或展开后才发表本轮执行尾部。
    fn drop(&mut self) {
        self.0.work_exited.fetch_add(1, Ordering::Release);
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Pool {
    count: usize,
    free: usize,
    preparing: usize,
    owned: usize,
    processing: usize,
    pages: usize,
    heads: usize,
    device_pages: usize,
    addresses: [u64; super::POOL_MAX_SLOTS],
}
#[derive(Debug)]
struct Snapshot {
    rx: Pool,
    tx: Pool,
    pending_rx: usize,
    pending_tx: usize,
    status: Option<DeviceStatus>,
    rings: Option<[(u64, usize); 2]>,
    stopping: bool,
    stopped: bool,
    tx_completed: usize,
    rx_completed: usize,
    kicks: usize,
    budgets: usize,
    rechecks: usize,
    entered: usize,
    exited: usize,
    #[expect(
        dead_code,
        reason = "NIC 快照的 Debug 诊断记录收发及停止窗口中的累计 IRQ 数"
    )]
    interrupts: usize,
    #[expect(dead_code, reason = "NIC 停止窗口的 Debug 诊断区分停止期间到达的 IRQ")]
    stopping_interrupts: usize,
    barrier: bool,
    unregistered: bool,
    returned: bool,
}
/// 统计同锁内的真实固定槽、页与独立 head 表，并核一一归属。
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
    /// 锁内读取真实队列和槽；已停止时不再访问已析构的 driver 或原页。
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

struct Inventory {
    devices: Arc<DeviceInventory>,
    baseline: SuiteBaseline,
    udp_port: u16,
}
impl Fixture for Inventory {
    /// 套件只发现一次库存，随后保存同 VM 的原 suite 六域与三字段基线。
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
    /// 最后 case 拥有和子任务结束后核同一个 suite 的原资源账。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        self.baseline.tear_down(context)
    }
}
/// 此 fixture 只消费宿主的明确 UDP token，不构造或猜测服务端口。
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

struct Owner {
    before: Baseline,
    inventory: Arc<DeviceInventory>,
    owner: Arc<SpinLock<Option<NetStackOwner>>>,
    net: Option<NetStackRef>,
    nic: Option<Arc<VirtioNet>>,
    udp: Option<Arc<UdpEndpoint>>,
    port: u16,
    disks: Vec<DiskRef>,
    queues: Vec<WorkQueue>,
    disk_sources: [Option<irq::IrqNumber>; 3],
}
impl Fixture<Inventory> for Owner {
    /// 本例基线之后准备独立真实 Root/NIC，局部 owner 保证任意设置失败仍真正停止。
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
    /// 先释放所有真实停点，框架随后 join 原子任务；不先停止仍被子任务使用的 owner。
    fn quiesce(&mut self, _: &mut TestContext<'_>) -> TestResult {
        if let Some(nic) = &self.nic {
            nic.observation.release_all();
        }
        Ok(())
    }
    /// 框架真实 join 之后停止全部 producer，释放最后强拥有，再核完整 MM6/metadata3。
    fn tear_down(mut self, context: &mut TestContext<'_>) -> TestResult {
        let before = self.before;
        self.stop_network();
        for disk in &self.disks {
            if let Err(error) = disk.stop() {
                context.diagnostic(format_args!("network-nic Disk stop-error={:?}", error));
                // 无法确认设备停止时保留整个 fixture 的强拥有，终结 VM 前不释放 action context。
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
    /// 独占同库存硬件，生产 Root 保存停止责任；本 fixture 只接受真实 VirtIO 变体。
    fn prepare(&mut self, inventory: &DeviceInventory) -> TestResult {
        // SAFETY: suite 串行分派，前例和本例旧 NIC 已停止并释放全部强拥有；库存及完整永久映射有效。
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
    /// 正常停止顺序与失败清理相同；旧强拥有完全释放后才允许重准备同硬件。
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
    /// 只借本例真实 NIC，不生成新的 owning 设备。
    fn nic(&self) -> &Arc<VirtioNet> {
        self.nic.as_ref().expect("native NIC absent")
    }
    /// 完整消息交原 UDP TX record，返回只表示接纳，不推测宿主已经收到。
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
    /// 等正式 POLLIN 后领取同条 source/长度/字节，等待不 pump NIC 或协议。
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
    /// P/Q 使用普通 UDP 与同一真实宿主 socket，逐字节验证一个完整记录。
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
/// 只等待原始状态转换，有限时间不作已经进入、完成或收权的证明。
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
/// 编码一个合法广播 ARP reply；宿主无需应答，没有测试专用设备输入。
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
/// 全部记录必须有唯一 index、真实 68 字节和每个正文 byte，顺序不作假定。
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

#[::ktest::suite(fixture = Owner, suite_resources = Inventory, context = "Thread", tags("network-nic", "network-nic-native"), resources("net-user"), timeout_ms = 60000)]
pub(crate) mod native {
    use super::*;

    /// 唯一消费者停在 Entry；42 字节 ARP 的真实 TX used 恢复槽并只计一次 Ethernet 完成，不含 VirtIO 头。
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

    /// NB32 从同一普通 UDP socket 产生真实 pending，RX16 满预算后仍完整交付全部记录。
    #[test]
    #[ktest(tags("N21"))]
    fn rx_budget_continues_all_32_records(
        context: &mut TestContext<'_>,
        fixture: &mut Owner,
    ) -> TestResult {
        fixture.probe()?;
        let before = fixture.nic().snapshot()?;
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

    /// 判空后停在重装前；NB1 真 used 在窗口内到达，原 Work recheck 必须保后继执行。
    #[test]
    #[ktest(tags("N21"))]
    fn rearm_rechecks_real_pending_successor(
        context: &mut TestContext<'_>,
        fixture: &mut Owner,
    ) -> TestResult {
        fixture.probe()?;
        let before = fixture.nic().snapshot()?;
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

    /// NIC 与三盘使用同库存和固定槽；同步撤己 action 后原盘仍真正读写。
    #[test]
    #[ktest(tags("N22"), resources("root", "extra", "scratch"))]
    fn nic_three_disks_uart_and_shared_irq(
        context: &mut TestContext<'_>,
        fixture: &mut Owner,
    ) -> TestResult {
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

    /// 原 owner 在子 Task 停止，Work 真尾与 reset 后仍持页两个窗口分别观察。
    #[test]
    #[ktest(tags("N23"))]
    fn stop_waits_work_tail_then_reset_before_page_return(
        context: &mut TestContext<'_>,
        fixture: &mut Owner,
    ) -> TestResult {
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

    /// 满真实 TX 池不调用 writer；原软件队列保全报文，used 恢复容量后继续同一未接受拥有。
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

        // 邻居已由真实 P/Q 解析，Root 保留同一 owned packet，设备槽满不能释放软件额度。
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

        // 返回包的同一拥有与显式绑定端点覆盖等待和重发，不重新读取或编码输入。
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

    /// 真实原页及堆耗尽后 NIC 未发表失败，释放压力后同库存重新 start 并实际 P/Q。
    #[test]
    #[ktest(tags("N24"))]
    fn real_no_memory_unpublished_cleanup_and_reprepare(
        context: &mut TestContext<'_>,
        fixture: &mut Owner,
    ) -> TestResult {
        let old_net = Arc::downgrade(fixture.net.as_ref().unwrap());
        let old_nic = Arc::downgrade(fixture.nic());
        let source = fixture.nic().interrupt();
        fixture.stop_network();
        if old_net.upgrade().is_some() || old_nic.upgrade().is_some() {
            return Err(TestError("native old NIC strong owner survived stop"));
        }
        drop((old_net, old_nic));
        let actions = irq::test_action_count(source);
        let inventory = Arc::clone(&fixture.inventory);
        let root =
            NetStack::try_new_lo().map_err(|_| TestError("native OOM root preparation failed"))?;
        let mut pressure = crate::cache::ktest::HeapPressure::prepare()?;
        let exhausted = pressure.exhaust();
        let before = mm::stats();
        // SAFETY: 旧 NIC 强拥有全已结束；本次仍是同库存硬件唯一构造者，root Work 已在耗尽前准备且保活。
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
        fixture.prepare(&inventory)?;
        fixture.probe()?;
        fixture.stop_network();
        Ok(())
    }
}

impl Owner {
    /// 所有所选盘先准备 owner 再 start；失败仍保留已发表实例供原 fixture 停止。
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
        for candidate in &inventory.mmio_blocks {
            // SAFETY: 同 suite 原库存，三个不同窗口各由此 fixture 独占，永久映射覆盖所有盘尾部。
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
            // SAFETY: BDF 来自同一次库存且无其它 driver 拥有；与 NIC 不同，完整 BAR 映射有效。
            devices.push(
                unsafe { VirtioDevice::prepare_pci(&inventory.pci, device) }
                    .map_err(|_| TestError("native PCI disk preparation failed"))?,
            );
        }
        if devices.len() != 3 {
            return Err(TestError("native coexistence requires exactly three disks"));
        }
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
    /// 核原槽位及同库存身份；LA 必须与 root 共享真实 INTx，不以资源标签代身份。
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
    /// 一页真实 Bio 的读写都在原完成/收权后领取，成功必须完整512字节。
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
        let mut memory = Vec::new_in(BioAllocator::Heap);
        memory
            .try_reserve_exact(1)
            .map_err(|_| TestError("native Bio memory slots OOM"))?;
        memory.push(mapped);
        // SAFETY: 独占清零原页，全写输入已初始化；无 CPU 别名跨提交，原 Bio 保页到 used/status 或 reset 收权。
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
        let memory = completion
            .take_memory()
            .map_err(|_| TestError("native Bio reclaim failed"))?;
        let mut output = [0; 512];
        // SAFETY: 清零原页初始化完整范围，真实完成已结束所有 DMA 写者，memory 唯一持有 CPU-owned 映射。
        output.copy_from_slice(unsafe { memory[0].bytes() });
        Ok(output)
    }
    /// 只读 root/extra 实际 sector，不写它们的盘面。
    fn read_disk(&self, index: usize, sector: u64) -> Result<[u8; 512], TestError> {
        self.io(index, BlkOperation::Read, sector, None)
    }
    /// scratch 原512字节全文图样经过实际 Write 与独立 Read 核对。
    fn scratch_roundtrip(&self, context: &mut TestContext<'_>) -> TestResult {
        let expected = core::array::from_fn(|index| (index * 29 + (index >> 3) + 61) as u8);
        self.io(2, BlkOperation::Write, 128, Some(&expected))?;
        let actual = self.read_disk(2, 128)?;
        context.expect_eq(actual, expected);
        Ok(())
    }
}
