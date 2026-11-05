//! 两个 split ring 与固定原页池的网络设备拥有者。

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

const FRAME_MAX: usize = ETHERNET_FRAME_CAPACITY;
const FRAME_MIN: usize = smoltcp::wire::ETHERNET_HEADER_LEN;
const RX_BUDGET: usize = 16;
const TX_BUDGET: usize = 32;
const FRAME_DESCRIPTOR_COUNT: u16 = 2;
const POOL_MAX_SLOTS: usize = 32;

type Driver = VirtIONet<KernelHal, SomeTransport<'static>>;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Phase {
    Prepared,
    Live,
    Faulted,
    Stopping,
    Stopped,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum SlotState {
    Free,
    Preparing,
    DeviceOwned,
    Processing,
}

struct Slot {
    state: SlotState,
    page: Option<MappedPages>,
    head: Option<u16>,
    frame_len: usize,
}

/// 控制视图在原页之前析构；外层 ManuallyDrop 保住失败时的全部拥有。
struct Inner {
    driver: ManuallyDrop<Driver>,
    domain: DmaDomain,
    rx_ring: ManuallyDrop<CoherentDma>,
    tx_ring: ManuallyDrop<CoherentDma>,
    rx: Vec<Slot>,
    tx: Vec<Slot>,
    rx_heads: Vec<Option<usize>>,
    tx_heads: Vec<Option<usize>>,
}

impl Drop for Inner {
    /// 仅未发表或真实 reset 后调用，先结束安全 CPU 控制视图，再归还全部原页。
    fn drop(&mut self) {
        assert!(
            self.rx.iter().chain(self.tx.iter()).all(|slot| {
                slot.page
                    .as_ref()
                    .is_some_and(|page| !page.is_device_owned())
            }),
            "dropping active NIC payloads"
        );
        // SAFETY: 外层仅在未发表或 reset 确认、IRQ/Work/producer 全尾后消费 Inner；
        // driver 不再被任何 CPU 调用，两个控制页仍由本对象保活。
        unsafe { ManuallyDrop::drop(&mut self.driver) };
        self.rx.clear();
        self.tx.clear();
        // SAFETY: driver 及其 queue 已析构，设备已停止或从未取得地址，原页各归还一次。
        unsafe {
            ManuallyDrop::drop(&mut self.tx_ring);
            ManuallyDrop::drop(&mut self.rx_ring);
        }
    }
}

struct State {
    phase: Phase,
    inner: Option<ManuallyDrop<Inner>>,
    must_reset: bool,
    progress_closed: bool,
    notify_active: usize,
    failure: Option<Error>,
    statistics: InterfaceStats,
}

/// Root 强持同一稳定 Arc，IRQ/Work 仅借其值，反向协议指针为 Weak。
/// 发表后的最后强引用只能在 stop_sync 成功后释放。
pub struct VirtioNet {
    state: SpinLock<State>,
    control: SleepMutex<()>,
    registration: SpinLock<Option<IrqRegistration>>,
    progress: Once<Pin<Arc<Work>>>,
    net: Weak<NetStack>,
    info: NetInfo,
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "ktest 校验 NIC 与库存的 MMIO 或 PCI 身份，生产路径不再读取该快照"
        )
    )]
    identity: VirtioIdentity,
    interrupt: IrqNumber,
    #[cfg(feature = "ktest")]
    observation: ktest::Observation,
}

impl VirtioNet {
    /// 普通任务准备未发表的两队列、固定页池与 CPU0 Work。
    ///
    /// # Safety
    /// 库存必须来自唯一真实发现；完整寄存器窗口永久强序映射，调用者独占选定硬件。
    /// Root 须先强持返回 Arc 再 start，并保活至所有 producer、Work、IRQ 与 reset 全尾。
    pub unsafe fn prepare(
        net: &NetStackRef,
        inventory: &DeviceInventory,
        cpu: usize,
    ) -> Result<Arc<Self>, VirtioInitError> {
        crate::sched::assert_switchable();
        assert_eq!(cpu, 0, "NIC progress must use CPU0");
        let domain = qemu_dma_domain()?;
        // SAFETY: 调用者承担真实库存、永久映射与独占硬件义务，选择不重新枚举资源。
        let (mut transport, identity, interrupt) = unsafe { select_transport(inventory) }?;
        if transport.device_type() != DeviceType::Network {
            return Err(ProbeError::Resource("selected VirtIO device is not network").into());
        }

        transport.request_reset();
        if !transport.reset_complete() {
            return Err(Error::ResetPending.into());
        }
        KernelHal::dma_barrier();

        let info = Driver::negotiate(&mut transport)?;
        let status = transport.get_status();
        let inner = prepare_inner(transport, info, domain)?;
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

    /// Root 已强持本 Arc 时注册 disabled action，发表队列并在同锁建立全部 RX head。
    /// 发表尝试后的失败保留原拥有，由原 Root owner 结束所有生产源后调用 stop_sync。
    pub fn start(self: &Arc<Self>) -> Result<(), VirtioInitError> {
        let _control = self.control.lock();
        if self.state.lock().phase != Phase::Prepared {
            return Err(Error::AlreadyUsed.into());
        }

        // SAFETY: context 是此真实 Arc 的固定值；Root 强拥有覆盖注册到同步注销全尾，
        // handler 有界且不等待，尚未 enable，所有页与 Work 已准备完成。
        let registration =
            unsafe { irq::register(self.interrupt, irq_callback, Arc::as_ptr(self) as *mut ()) }
                .map_err(ProbeError::Irq)?;
        *self.registration.lock() = Some(registration);

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

        let pending = result?;
        self.registration.lock().as_ref().unwrap().enable();
        self.notify(true);
        if pending {
            self.queue_progress();
        }
        Ok(())
    }

    /// 返回本设备本次协商的稳定 MAC、头长与队列深度。
    pub fn info(&self) -> NetInfo {
        self.info
    }

    /// 返回库存确认的完整硬件身份，不重新访问设备配置。
    #[cfg(feature = "ktest")]
    pub fn identity(&self) -> VirtioIdentity {
        self.identity
    }

    /// 返回本设备 action 所属的真实共享 IRQ 源。
    #[cfg(feature = "ktest")]
    pub fn interrupt(&self) -> IrqNumber {
        self.interrupt
    }

    /// 在原设备短锁内复制真实收发事件累计值，不访问 DMA 页或改变计数。
    pub fn statistics(&self) -> InterfaceStats {
        self.state.lock().statistics
    }

    /// 锁内认领固定槽，锁外纯编码，再在共同队列锁内交权、提交与安装唯一 head。
    /// writer 不得分配、uaccess、睡眠或反调协议；false 仅为真实槽满且不会调用 writer。
    /// 映射、交权或提交失败均保留原槽页，设备访问只由真实 used 或确认 reset 结束。
    pub fn try_transmit(
        &self,
        frame_len: usize,
        write_frame: impl FnOnce(&mut [u8]),
    ) -> Result<bool, VirtioInitError> {
        if !(FRAME_MIN..=FRAME_MAX).contains(&frame_len) {
            return Err(Error::InvalidParam.into());
        }
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

        let mut prepared = PreparingTx {
            nic: self,
            slot,
            page: Some(page),
        };
        {
            let bytes = prepared.page.as_mut().unwrap().bytes_mut();
            // SAFETY: 固定池原页首次完整清零，此 CPU-owned 局部拥有独占全部映射；
            // 转换只借当前可变范围，writer 返回后此别名结束，之后才交设备访问。
            let bytes =
                unsafe { slice::from_raw_parts_mut(bytes.as_mut_ptr().cast::<u8>(), bytes.len()) };
            write_frame(&mut bytes[self.info.header_len..self.info.header_len + frame_len]);
        }

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
                        // SAFETY: writer 的 CPU 别名已结束，整映射首次清零，头每次全零；
                        // 页已移回固定原槽且受共同队列锁保护，reset/used 前绝不释放。
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
                    // SAFETY: 此映射方向为 ToDevice，长度是本次真实帧长，原槽保活完整访问期；
                    // 队列锁排除 used 消费，head 表在解锁前安装，两个 SG 只引用本页。
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
        result?;
        self.notify(false);
        Ok(true)
    }

    /// 原 Root 已退出全部外部 producer 后，在普通可睡任务按 Work→reset→己 IRQ 收尾。
    /// 未确认 reset 保留原注册、完整 DMA 拥有和同一 Root/NIC context。
    pub fn stop_sync(&self) -> Result<(), VirtioInitError> {
        let _control = self.control.lock();
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
        let registration = self.registration.lock().take();
        if let Some(registration) = registration {
            registration.unregister_sync();
        }
        #[cfg(feature = "ktest")]
        self.observation
            .unregistered
            .store(true, core::sync::atomic::Ordering::Release);

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
                    // SAFETY: 所有 CPU producer/通知与 Work 已退出，真实 status0 和完整屏障
                    // 证明设备结束全部范围访问，己 IRQ 已同步注销；此页只由当前原槽拥有。
                    unsafe { page.finish_device_access() }
                        .map_err(|error| VirtioInitError::dma_prepare(error, status))?;
                }
            }
            // 同步失败时 Inner 仍在原设备中，活动页不随错误返回析构或提前归还。
            state.must_reset = false;
            state.inner.take().unwrap()
        };
        // driver/queue 的安全 CPU 入口先结束，再释放 CPU-owned payload 与两 ring。
        drop(ManuallyDrop::into_inner(owner));
        #[cfg(feature = "ktest")]
        self.observation
            .pages_returned
            .store(true, core::sync::atomic::Ordering::Release);
        self.state.lock().phase = Phase::Stopped;
        Ok(())
    }

    /// 只确认本设备原因、抑制两个 ring，并安排原 Work；停止时不再提交票。
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

    /// Work 独占消费者，按实际 used 完成数记录非零批次；原预算和 RX 恰满重检保持不变。
    fn progress(&self) -> Result<(), Error> {
        if self.state.lock().phase != Phase::Live {
            return Ok(());
        }
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
            // 即便软件拒绝输入，也把同一 CPU-owned 原页归回原槽；协议错误不再交设备。
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

    /// 真实 TX used 只按 shadow head 归槽，len 不代表发送字节数。
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
        // SAFETY: shadow 和独立 head 表共同确认本轮唯一链完成，只有此链引用本页；
        // 队列锁串行收权，TX used.len 不改变真实本轮提交长度或归页证明。
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

    /// 在同锁核真实 RX head 与本轮长度，收权后把唯一原页移给当前 Work。
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
        // SAFETY: 真实 shadow/head 表确认本轮唯一 RX 链完成，used 长度在提交范围内；
        // 页仍由原槽独占，后续解析只读取此实际前缀，设备不可再写直到重投。
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

    /// 收权前缀通过头校验后累计 Ethernet 长度，Root 拒绝仍按原语义重投同页。
    fn receive_frame(&self, page: &MappedPages, len: usize) -> Result<(), Error> {
        // SAFETY: 原页首次清零完整映射，真实 used 已收权，Work 独占此页且无裸写别名；
        // 初始化尾部只满足 Rust 读取条件，协议仅取本轮已写 len，借用止于此调用。
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

    /// 当前 Work 将同一原页归原槽，CPU 借用已结束才交权；停止或协议错误保 CPU 拥有。
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
        // SAFETY: receive_ethernet 已返回，没有解析借用跨重投；同一页归回唯一原槽，
        // 队列锁保原页直到下轮真实 used 或 reset，方向固定 FromDevice。
        unsafe { page.begin_device_access(inner.domain) }.map_err(dma_protocol)?;
        slot.state = SlotState::DeviceOwned;
        // SAFETY: 原映射覆盖完整头与1514帧范围，两个SG只引用本原页；
        // 同锁安装新 head 后 IRQ/Work 才能消费，本轮 CPU 内容借用已结束。
        let head = unsafe { inner.driver.submit_rx(segment) }?;
        install_head(&mut inner.rx, &mut inner.rx_heads, index, head)
    }

    /// 锁内认领通知尾部，锁外真实 doorbell；Root producer 全尾与 progress close 阻止 reset 竞态。
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
            // SAFETY: notify_active 记录此真实尾部，Root 先结束全部外部 producer，
            // progress close 再等 Work 返回，reset 在两者均结束后进行；硬件映射永久有效。
            unsafe { notification.notify::<KernelHal>() };
            KernelHal::dma_barrier();
            self.state.lock().notify_active -= 1;
        }
    }

    /// 重用原 Work 的 pending/running 两票，关闭竞态由 Work 自己拒绝新票。
    fn queue_progress(&self) {
        let _ = self
            .progress
            .get()
            .expect("NIC progress Work missing")
            .as_ref()
            .queue();
    }

    /// 保存真实协议失败并关闭后续发布，不将非法 used 当空队列或归还设备页。
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
    /// 未发表失败可直接归还；发表实例只能由已成功 stop 的原拥有者释放。
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

/// Preparing 的页在执行 producer 局部拥有，未交权退出时按原槽恢复，不丢池页。
struct PreparingTx<'a> {
    nic: &'a VirtioNet,
    slot: usize,
    page: Option<MappedPages>,
}

impl Drop for PreparingTx<'_> {
    /// writer 或关门路径退出时归回仍 CPU-owned 原页；成功提交已把页移回 DeviceOwned 槽。
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

/// 在发表任何地址前完成所有 fallible 页池与 shadow，局部 CPU 视图先于原页退出。
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

    // SAFETY: ring 原页先于 queue 构造且保活，真实 coherent CPU/device 同一存储；
    // 未发表的错误路径先析构 queue 控制视图，再按局部逆序析构原页。
    let rx_queue = unsafe { VirtQueue::<KernelHal>::new(rx_layout, rx_ring.region()) }
        .map_err(|error| VirtioInitError::queue_prepare(error, status))?;
    // SAFETY: TX 控制页独立且同样保活到所有 CPU 控制视图结束，尚未发表给设备。
    let tx_queue = unsafe { VirtQueue::<KernelHal>::new(tx_layout, tx_ring.region()) }
        .map_err(|error| VirtioInitError::queue_prepare(error, status))?;
    // SAFETY: 两队列绑定非拥有视图，Inner 同时保存真实原页；外层冻结后任何发表失败
    // 都保全部资源到 Root producer/Work/IRQ 全尾与真实 reset，不允许移动替换 driver。
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

/// 普通任务一次申请最多32零填原页，各映射只覆盖本头与最大合法帧。
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

/// 独立 RX/TX head 名字空间在未发表时 fallible 预留，IRQ 中只访问固定表。
fn prepare_heads(depth: u16, status: DeviceStatus) -> Result<Vec<Option<usize>>, VirtioInitError> {
    let mut heads = Vec::new();
    heads
        .try_reserve_exact(usize::from(depth))
        .map_err(|_| VirtioInitError::NoMemory { status })?;
    heads.resize(usize::from(depth), None);
    Ok(heads)
}

/// 发布后仍在共同队列锁内交全部 RX 页并保存每条真实链的唯一拥有。
fn publish_rx(inner: &mut Inner) -> Result<bool, Error> {
    inner.driver.publish()?;
    inner.driver.set_interrupts(false);

    for index in 0..inner.rx.len() {
        let slot = &mut inner.rx[index];
        let page = slot.page.as_mut().ok_or(Error::IoError)?;
        let segment = page.segment(inner.domain).map_err(dma_protocol)?;
        // SAFETY: 固定池页已完整清零且无 CPU 别名，原槽保全访问期；
        // 调用者共同队列锁覆盖 submit 与 head 表安装，外层已承担 reset 责任。
        unsafe { page.begin_device_access(inner.domain) }.map_err(dma_protocol)?;
        slot.state = SlotState::DeviceOwned;
        // SAFETY: FromDevice 映射覆盖本头和1514帧，两个SG均在此原页内；
        // 原槽已拥有页，任何失败都保到确认 reset，不从错误返回猜未发表。
        let head = unsafe { inner.driver.submit_rx(segment) }?;
        install_head(&mut inner.rx, &mut inner.rx_heads, index, head)?;
    }

    let pending = inner.driver.enable_interrupts_and_recheck();
    if pending {
        inner.driver.set_interrupts(false);
    }
    Ok(pending)
}

/// 同锁安装唯一 head；不覆盖旧提交的页拥有，也不将设备 ID 直接当池索引。
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

/// 软件表与原槽共同核本轮真实 head，错误不清拥有记录或结束 DMA。
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

/// 选当前架构的唯一真实候选，多个或缺失候选为资源错误，不启发式取首个。
///
/// # Safety
/// 继承 prepare 调用者的唯一硬件、同源库存和完整永久映射义务。
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
        // SAFETY: 候选来自同一次库存，调用者保证完整永久映射与选定设备独占。
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
        // SAFETY: 只借同库存真实功能，不重扫 BAR 或猜 INTx；调用者保证永久映射与独占。
        unsafe { crate::drivers::virtio::prepare_pci_transport(&inventory.pci, candidate) }
    }
}

/// IRQ 地址来自实际 Arc 值，Root 保活直到 unregister_sync 返回。
///
/// # Safety
/// context 须为本驱动 register 保存的同一值，整个 handler 期有效且未移动。
unsafe fn irq_callback(context: *mut ()) -> bool {
    // SAFETY: 原 Root 强拥有覆盖 action 的执行引用；同步注销前值不会释放。
    unsafe { &*context.cast::<VirtioNet>() }.handle_interrupt()
}

/// 同 Work 不重入保证唯一 RX消费者，普通任务协议错误立即保原 owning context 并 fail-fast。
fn progress_callback(work: Pin<&Work>) {
    // SAFETY: context 来自 prepare 的真实 Arc 值，Root 到 close_sync 全尾之前保持强拥有。
    let nic = unsafe { &*(work.context() as *const VirtioNet) };
    #[cfg(feature = "ktest")]
    let _tail = nic.observation.enter_work();
    if let Err(error) = nic.progress() {
        nic.record_failure(error);
        panic!("VirtIO network queue protocol failed: {:?}", error);
    }
}
