//! 单队列 DWMAC4/DWMAC1000，固定原页、真实 IRQ 与原网络 Work。
//! 控制器底稿已替换为原页拥有和同步停止实现，许可与修改声明见 dwmac/LICENSE。

mod descriptors;
mod phy;

use alloc::{
    sync::{Arc, Weak},
    vec::Vec,
};
use core::{mem::ManuallyDrop, pin::Pin, ptr, slice};
use spin::Once;
use virtio_drivers::BufferDirection;

use crate::{
    arch,
    boot::{self, DeviceSetup, GmacResources, Platform, Region},
    drivers::board_setup,
    irq::{self, IrqError, IrqNumber, IrqRegistration},
    mm::{
        self,
        dma::{DmaDomain, DmaError, MappedPages},
    },
    net::{stats::InterfaceStats, NetStack, NetStackRef},
    sync::{SleepMutex, SpinLock},
    time::{self, Duration},
    timer::{self, Timer, TimerKind},
    workqueue::Work,
};

use self::{
    descriptors::{device_owned, Ring},
    phy::{Link, Phy},
};

const RING_LEN: usize = 32;
const BUFFER_LEN: usize = 2048;
const FRAME_MIN: usize = 14;
const FRAME_MAX: usize = 1514;
const RX_BUDGET: usize = 16;

/// 原生错误保留资源、DMA、IRQ、协议和未确认停止的不同责任。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GmacError {
    NoMemory,
    Resource(&'static str),
    Dma(DmaError),
    Irq(IrqError),
    Io(&'static str),
    StopUnconfirmed,
}

impl From<DmaError> for GmacError {
    /// 页不足保留独立类别，其余 DMA 地址和同步失败保留原始原因。
    fn from(error: DmaError) -> Self {
        match error {
            DmaError::NoMemory => Self::NoMemory,
            _ => Self::Dma(error),
        }
    }
}

#[derive(Clone, Copy, Eq, PartialEq)]
enum Generation {
    Dwmac4,
    Dwmac1000,
}

/// 此值只由 unsafe prepare 在规范化资源和永久映射边界建立。
#[derive(Clone, Copy)]
struct Registers {
    region: Region,
    generation: Generation,
}

impl Registers {
    /// volatile 访问完整已映射的对齐寄存器，不借出设备内存引用。
    fn read(self, offset: usize) -> u32 {
        assert!(offset % 4 == 0 && offset + 4 <= self.region.end - self.region.start);
        // SAFETY: prepare 调用者证明此完整 MMIO 永久强序映射和设备独占；
        // 私有寄存器表中的偏移逐次检查，IRQ/Work 访问不建立普通内存别名。
        unsafe {
            ptr::read_volatile((arch::mmio_address(self.region.start) + offset) as *const u32)
        }
    }

    /// 写控制字段后真实设备屏障排序 DMA 描述符和通知。
    fn write(self, offset: usize, value: u32) {
        assert!(offset % 4 == 0 && offset + 4 <= self.region.end - self.region.start);
        arch::dma_write_barrier();
        // SAFETY: 与 read 相同的完整映射、对齐及硬件独占义务覆盖整个设备生命期。
        unsafe {
            ptr::write_volatile(
                (arch::mmio_address(self.region.start) + offset) as *mut u32,
                value,
            )
        };
        arch::dma_barrier();
    }

    /// 两代 DMA 中断寄存器不同，所有 W1C 都使用各自实际原因位。
    fn interrupt_register(self) -> usize {
        match self.generation {
            Generation::Dwmac4 => 0x1134,
            Generation::Dwmac1000 => 0x101c,
        }
    }

    /// 返回设备 DMA 状态偏移，不把旧代 CSR5 当新代通道状态。
    fn status_register(self) -> usize {
        match self.generation {
            Generation::Dwmac4 => 0x1160,
            Generation::Dwmac1000 => 0x1014,
        }
    }

    /// 仅含本通道实际可应答的中断原因，不写过程状态等只读位。
    fn status_mask(self) -> u32 {
        match self.generation {
            Generation::Dwmac4 => 0xffff,
            Generation::Dwmac1000 => 0x1ffff,
        }
    }

    /// 每条 TX/RX 都请求完成 IRQ，包含真实异常摘要和 fatal bus error。
    fn interrupt_mask(self) -> u32 {
        match self.generation {
            Generation::Dwmac4 => (1 << 15) | (1 << 14) | (1 << 12) | (1 << 7) | (1 << 6) | 1,
            Generation::Dwmac1000 => (1 << 16) | (1 << 15) | (1 << 13) | (1 << 6) | (1 << 5) | 1,
        }
    }

    /// 只认领本 MAC 的已置位 DMA 原因；共享线没有本设备原因时返回零。
    fn acknowledge(self) -> u32 {
        let reason = self.read(self.status_register()) & self.status_mask();
        if reason != 0 {
            self.write(self.status_register(), reason);
        }
        reason
    }

    /// 通道门控不改变共享外部 IRQ 的其它 action。
    fn interrupts(self, enabled: bool) {
        self.write(
            self.interrupt_register(),
            if enabled { self.interrupt_mask() } else { 0 },
        );
    }

    /// fatal 原因不能靠丢包或重新投递掩盖，后续由原拥有者停止设备。
    fn fatal(self, reason: u32) -> bool {
        reason
            & match self.generation {
                Generation::Dwmac4 => 1 << 12,
                Generation::Dwmac1000 => 1 << 13,
            }
            != 0
    }

    /// 两代 MAC0 地址寄存器采用相同字节顺序但不同偏移。
    fn address_register(self) -> usize {
        match self.generation {
            Generation::Dwmac4 => 0x300,
            Generation::Dwmac1000 => 0x40,
        }
    }

    /// 仅从真实硬件取六字节地址，合法性由资源边界统一检查。
    fn mac_address(self) -> [u8; 6] {
        let high = self.read(self.address_register()).to_le_bytes();
        let low = self.read(self.address_register() + 4).to_le_bytes();
        [low[0], low[1], low[2], low[3], high[0], high[1]]
    }

    /// MAC0 使用有效的 DT 或硬件地址，不生成本地 fallback。
    fn set_mac_address(self, mac: [u8; 6]) {
        self.write(
            self.address_register() + 4,
            u32::from_le_bytes([mac[0], mac[1], mac[2], mac[3]]),
        );
        self.write(
            self.address_register(),
            (1 << 31) | u32::from(u16::from_le_bytes([mac[4], mac[5]])),
        );
    }

    /// 关闭 MAC 和 DMA 的新收发，不以 ST/SR 清零代替停止完成，也不触发依赖 RXC 的复位。
    fn stop_dma(self) {
        self.interrupts(false);
        match self.generation {
            Generation::Dwmac4 => {
                self.write(0x1104, self.read(0x1104) & !1);
                self.write(0x1108, self.read(0x1108) & !1);
                self.write(0, self.read(0) & !3);
            }
            Generation::Dwmac1000 => {
                self.write(0x1018, self.read(0x1018) & !((1 << 13) | (1 << 1)));
                self.write(0, self.read(0) & !((1 << 3) | (1 << 2)));
            }
        }
    }

    /// 已关闭新收发并保证 PHY 输入时钟后请求软件复位，只有 SWR 清零才允许归页。
    fn request_reset(self) {
        self.write(0x1000, self.read(0x1000) | 1);
    }

    /// 可睡有界等待仅用于准备和停止，不用于稳态帧完成。
    fn wait_reset(self) -> Result<(), GmacError> {
        let deadline = time::now().checked_add(Duration::from_secs(2)).unwrap();
        while self.read(0x1000) & 1 != 0 {
            if deadline <= time::now() {
                return Err(GmacError::StopUnconfirmed);
            }
            timer::sleep(Duration::from_millis(1));
        }
        arch::dma_barrier();
        Ok(())
    }

    /// 配置一条 RX/TX 队列及实际步长，仍保持 MAC 和 DMA 停止。
    fn configure(self, inner: &Inner, mac: [u8; 6]) -> Result<(), GmacError> {
        self.interrupts(false);
        self.acknowledge();
        self.set_mac_address(mac);
        match self.generation {
            Generation::Dwmac4 => {
                let feature = self.read(0x120);
                let rx_fifo = 128usize
                    .checked_shl(feature & 0x1f)
                    .ok_or(GmacError::Resource("invalid RX FIFO feature"))?;
                let tx_fifo = 128usize
                    .checked_shl((feature >> 6) & 0x1f)
                    .ok_or(GmacError::Resource("invalid TX FIFO feature"))?;
                if rx_fifo < 256 || tx_fifo < 256 {
                    return Err(GmacError::Resource("GMAC FIFO is too small"));
                }
                self.write(
                    0xd00,
                    (((tx_fifo / 256 - 1).min(0x1ff) as u32) << 16) | (2 << 2) | (1 << 1),
                );
                self.write(0xd18, 0x10);
                self.write(
                    0xd30,
                    (((rx_fifo / 256 - 1).min(0x3ff) as u32) << 20) | (1 << 5),
                );
                self.write(0xa0, 2);
                self.write(0xa4, 1 << 20);
                // 接受广播、全部组播与 MAC0 精确匹配，硬件不处理协议校验或 FCS stripping。
                self.write(0x8, (1 << 10) | (1 << 4));
                self.write(0x70, 0);
                self.write(0x90, 0);
                self.write(0xb4, 0);
                self.write(0x1000, 1 << 19);
                self.write(0x1100, inner.rx_ring.skip_length() << 18);
                self.write(0x1104, (8 << 16) | (1 << 4));
                self.write(0x1108, (8 << 16) | ((BUFFER_LEN as u32) << 1));
                self.write(0x1110, (inner.tx_ring.address(0) >> 32) as u32);
                self.write(0x1114, inner.tx_ring.address(0) as u32);
                self.write(0x1118, (inner.rx_ring.address(0) >> 32) as u32);
                self.write(0x111c, inner.rx_ring.address(0) as u32);
                self.write(0x112c, RING_LEN as u32 - 1);
                self.write(0x1130, RING_LEN as u32 - 1);
                self.write(0x1120, inner.tx_ring.address(0) as u32);
                self.write(0x1128, inner.rx_ring.address(RING_LEN - 1) as u32);
                self.write(0, 0);
            }
            Generation::Dwmac1000 => {
                if self.read(0x1058) & (1 << 24) == 0 {
                    return Err(GmacError::Resource(
                        "GMAC enhanced descriptors are unavailable",
                    ));
                }
                // 增强格式32B，DSL单位为32bit字；非一致64B槽需跳八字，ATDS必须置位。
                self.write(
                    0x1000,
                    (inner.rx_ring.skip_length() << 2)
                        | (1 << 7)
                        | (8 << 8)
                        | (8 << 17)
                        | (1 << 23),
                );
                self.write(0x100c, inner.rx_ring.address(0) as u32);
                self.write(0x1010, inner.tx_ring.address(0) as u32);
                self.write(0x1018, (1 << 25) | (1 << 21) | (1 << 2));
                self.write(0x3c, 0x60f);
                self.write(0x4, (1 << 10) | (1 << 4));
                self.write(0x18, 0);
                self.write(0x10c, u32::MAX);
                self.write(0x110, u32::MAX);
                self.write(0x200, u32::MAX);
                self.write(0, 1 << 16);
            }
        }
        Ok(())
    }

    /// DMA 引擎启动只在所有 RX 原页和描述符已经发表之后进行。
    fn start_dma(self) {
        match self.generation {
            Generation::Dwmac4 => {
                self.write(0x1104, self.read(0x1104) | 1);
                self.write(0x1108, self.read(0x1108) | 1);
            }
            Generation::Dwmac1000 => {
                self.write(0x1018, self.read(0x1018) | (1 << 13) | (1 << 1));
                self.write(0x1008, 0);
            }
        }
    }

    /// DWMAC4 用 tail 边界，DWMAC1000 用 poll-demand，不能交换两代通知格式。
    fn notify(self, receive: bool, tail: u64) {
        match self.generation {
            Generation::Dwmac4 => self.write(if receive { 0x1128 } else { 0x1120 }, tail as u32),
            Generation::Dwmac1000 => self.write(if receive { 0x1008 } else { 0x1004 }, 0),
        }
    }

    /// 仅改变真实协商出的 MAC 速率和双工；没有 carrier 时停止 MAC 接收与发送。
    fn set_link(self, link: Option<Link>) {
        let (enable, duplex) = match self.generation {
            Generation::Dwmac4 => (3, 1 << 13),
            Generation::Dwmac1000 => ((1 << 2) | (1 << 3), 1 << 11),
        };
        let mut control = self.read(0) & !(enable | duplex | (1 << 15) | (1 << 14));
        self.write(0, control);
        if let Some(link) = link {
            control |= enable;
            if link.full_duplex {
                control |= duplex;
            }
            if link.speed < 1000 {
                control |= 1 << 15;
            }
            if link.speed == 100 {
                control |= 1 << 14;
            }
            self.write(0, control);
        }
    }
}

#[derive(Clone, Copy, Eq, PartialEq)]
enum Phase {
    Prepared,
    Live,
    Faulted,
    Stopping,
    Stopped,
}

#[derive(Clone, Copy, Eq, PartialEq)]
enum SlotState {
    Free,
    Preparing,
    Ready,
    DeviceOwned,
    Processing,
}

struct Slot {
    state: SlotState,
    page: Option<MappedPages>,
    address: u64,
    frame_len: usize,
}

/// 控制页和 payload 始终同一个 Inner 拥有，错误不移出活动 DMA 页。
struct Inner {
    domain: DmaDomain,
    rx_ring: Ring,
    tx_ring: Ring,
    rx: Vec<Slot>,
    tx: Vec<Slot>,
    rx_next: usize,
    tx_claim: usize,
    tx_publish: usize,
    tx_complete: usize,
}

impl Drop for Inner {
    /// 外层仅在从未发表或确认 reset 后消费此值。
    fn drop(&mut self) {
        assert!(
            self.rx.iter().chain(self.tx.iter()).all(|slot| slot
                .page
                .as_ref()
                .is_some_and(|page| !page.is_device_owned())),
            "dropping active GMAC payloads"
        );
    }
}

struct State {
    phase: Phase,
    inner: Option<ManuallyDrop<Inner>>,
    must_reset: bool,
    closed: bool,
    pending: bool,
    reason: u32,
    phy_due: bool,
    link: Option<Link>,
    failure: Option<GmacError>,
    statistics: InterfaceStats,
}

/// 网络根强持稳定 Arc 至同步停止成功；反向 net 为 Weak，不制造循环拥有。
pub struct Dwmac {
    state: SpinLock<State>,
    control: SleepMutex<()>,
    registration: SpinLock<Option<IrqRegistration>>,
    progress: Once<Pin<Arc<Work>>>,
    link_timer: Once<Pin<Arc<Timer>>>,
    net: Weak<NetStack>,
    registers: Registers,
    resources: GmacResources,
    interrupt: IrqNumber,
    mac: [u8; 6],
    phy: Phy,
}

impl Dwmac {
    /// 普通任务完成所有 fallible 固定内存、PHY 和 CPU0 回调准备，不发表 DMA 地址。
    ///
    /// # Safety
    /// resources 必须来自唯一库存，PCI BAR 已解析；全部 MMIO/时钟/引脚窗口永久映射，
    /// 选定 MAC、PHY 和板设置没有其它拥有者。网络根须在 start 前强持返回 Arc，
    /// 并保活到 producer、Timer、Work、IRQ 全尾与 DMA 确认停止。
    pub unsafe fn prepare(
        net: &NetStackRef,
        resources: &GmacResources,
        cpu: usize,
    ) -> Result<Arc<Self>, GmacError> {
        crate::sched::assert_switchable();
        assert_eq!(cpu, 0, "GMAC progress must use CPU0");
        let generation = match boot::info().platform {
            Platform::VisionFive2 => Generation::Dwmac4,
            Platform::Ls2k1000la => Generation::Dwmac1000,
            Platform::QemuVirt => {
                return Err(GmacError::Resource("QEMU has no selected native GMAC"))
            }
        };
        let minimum = match generation {
            Generation::Dwmac4 => 0x1164,
            Generation::Dwmac1000 => 0x105c,
        };
        if resources.registers.start == 0
            || resources.registers.start % 4 != 0
            || resources
                .registers
                .end
                .checked_sub(resources.registers.start)
                .is_none_or(|length| length < minimum)
        {
            return Err(GmacError::Resource("GMAC MMIO resource is incomplete"));
        }
        let interrupt = irq::map_source(resources.interrupt_source).map_err(GmacError::Irq)?;
        // SAFETY: 唯一规范化资源与各 provider 的完整永久映射由本接口调用者保证。
        unsafe { board_setup::apply_setup(&resources.setup, Duration::ZERO) }
            .map_err(GmacError::Resource)?;
        let registers = Registers {
            region: resources.registers,
            generation,
        };
        // SAFETY: syscon 与 MAC 同属本次独占硬件，资源检查覆盖写入的完整字段。
        unsafe { configure_interface(resources, generation) }?;
        let version = registers.read(match generation {
            Generation::Dwmac4 => 0x110,
            Generation::Dwmac1000 => 0x20,
        }) & 0xff;
        if !matches!(
            (generation, version),
            (Generation::Dwmac4, 0x51 | 0x52) | (Generation::Dwmac1000, 0x37)
        ) {
            return Err(GmacError::Resource(
                "selected GMAC hardware version is unsupported",
            ));
        }
        let mac = resources
            .mac_address
            .unwrap_or_else(|| registers.mac_address());
        if !valid_mac(mac) {
            return Err(GmacError::Resource(
                "GMAC has no valid DT or hardware MAC address",
            ));
        }
        registers.stop_dma();
        let csr_divider = if generation == Generation::Dwmac4 {
            if resources.setup.clocks.count == 0 {
                return Err(GmacError::Resource("GMAC CSR clock is missing"));
            }
            // SAFETY: 同次规范化时钟 provider 已映射且 apply_setup 已完成，读取不重做发现。
            let frequency = unsafe { board_setup::clock_rate(&resources.setup.clocks.entries[0]) }
                .map_err(GmacError::Resource)?;
            phy::clock_range(frequency)?
        } else {
            // 所选 LS2K1000LA 平台要求 CSR100–150MHz 的 /62 编码，不伪造精确输入率。
            1
        };
        let phy = Phy::prepare(registers, resources, csr_divider)?;
        // CSR/MDIO 时钟已由板设置开启，管理事务不依赖 RGMII RXC 或 DMA 复位；
        // PHY 已撤销掉电/隔离并保持无载波时的 RXC，才能请求首次依赖 RXC 的 SWR。
        registers.request_reset();
        registers.wait_reset()?;
        let line = boot::info().board.cache_line_size;
        if !resources.dma_coherent && (line == 0 || 64 < line || !line.is_power_of_two()) {
            return Err(GmacError::Resource(
                "GMAC cacheline does not fit descriptor isolation",
            ));
        }
        // 两块所选控制器使用恒等总线窗口，mask 只取本 GMAC 的真实32位约束。
        let domain = DmaDomain::new(
            resources.dma_mask.min(u32::MAX as u64),
            0,
            usize::MAX,
            0,
            if resources.dma_coherent { 1 } else { line },
            resources.dma_coherent,
        )?;
        let inner = prepare_inner(domain, generation, resources.dma_coherent)?;
        let nic = Arc::try_new(Self {
            state: SpinLock::new(State {
                phase: Phase::Prepared,
                inner: Some(ManuallyDrop::new(inner)),
                must_reset: false,
                closed: false,
                pending: false,
                reason: 0,
                phy_due: false,
                link: None,
                failure: None,
                statistics: InterfaceStats::default(),
            }),
            control: SleepMutex::new(()),
            registration: SpinLock::new(None),
            progress: Once::new(),
            link_timer: Once::new(),
            net: Arc::downgrade(net),
            registers,
            resources: *resources,
            interrupt,
            mac,
            phy,
        })
        .map_err(|_| GmacError::NoMemory)?;
        let context = Arc::as_ptr(&nic) as usize;
        let work = Work::try_new(net.work_queue(), cpu, progress_callback, context)
            .map_err(|_| GmacError::NoMemory)?;
        let timer = Timer::try_new(cpu, TimerKind::Precise, link_expired, context)
            .map_err(|_| GmacError::NoMemory)?;
        nic.progress.call_once(|| work);
        nic.link_timer.call_once(|| timer);
        Ok(nic)
    }

    /// 根已强持 Arc 时登记 disabled IRQ，再发表原页和 ring；失败保留原拥有到 stop。
    pub fn start(self: &Arc<Self>) -> Result<(), GmacError> {
        let _control = self.control.lock();
        if self.state.lock().phase != Phase::Prepared {
            return Err(GmacError::Io("GMAC is not prepared"));
        }
        // SAFETY: context 来自真实稳定 Arc，根强拥有覆盖 register 到 unregister_sync 全尾；
        // action 尚未启用，全部页、Work、Timer 已在发表前准备完成。
        let registration =
            unsafe { irq::register(self.interrupt, irq_callback, Arc::as_ptr(self) as *mut ()) }
                .map_err(GmacError::Irq)?;
        *self.registration.lock() = Some(registration);
        self.state.lock().must_reset = true;
        if let Err(error) = self.publish() {
            self.record_failure(error);
            return Err(error);
        }
        {
            let mut state = self.state.lock();
            state.phase = Phase::Live;
            state.pending = true;
            state.phy_due = true;
        }
        self.registers.start_dma();
        self.registration.lock().as_ref().unwrap().enable();
        self.queue_progress();
        Ok(())
    }

    /// 返回准备时验证的稳定真实 MAC 地址。
    pub fn mac_address(&self) -> [u8; 6] {
        self.mac
    }

    /// 查询只复制实际帧计数，不读取设备拥有的页。
    pub fn statistics(&self) -> InterfaceStats {
        self.state.lock().statistics
    }

    /// 短锁认领固定槽，writer 在锁外编码；false 仅表示真实环槽满且不调用 writer。
    /// writer 不得分配、睡眠、uaccess 或反调网络；传入切片不跨调用保留。
    pub fn try_transmit(
        &self,
        frame_len: usize,
        writer: impl FnOnce(&mut [u8]),
    ) -> Result<bool, GmacError> {
        if !(FRAME_MIN..=FRAME_MAX).contains(&frame_len) {
            return Err(GmacError::Io("invalid Ethernet frame length"));
        }
        let (index, page, domain) = {
            let mut state = self.state.lock();
            if state.phase != Phase::Live {
                return Err(state.failure.unwrap_or(GmacError::Io("GMAC is stopping")));
            }
            let inner: &mut Inner = state.inner.as_mut().unwrap();
            let index = inner.tx_claim;
            let slot = &mut inner.tx[index];
            if slot.state != SlotState::Free {
                return Ok(false);
            }
            slot.state = SlotState::Preparing;
            slot.frame_len = frame_len;
            inner.tx_claim = (index + 1) % RING_LEN;
            (index, slot.page.take().unwrap(), inner.domain)
        };
        let mut lease = SlotLease {
            nic: self,
            receive: false,
            index,
            page: Some(page),
        };
        {
            let bytes = lease.page.as_mut().unwrap().bytes_mut();
            // SAFETY: 预备页完整清零，Preparing 唯一拥有本映射，writer 的可变别名
            // 在本块结束后消失，设备尚未取得该页地址。
            let bytes =
                unsafe { slice::from_raw_parts_mut(bytes.as_mut_ptr().cast::<u8>(), frame_len) };
            writer(bytes);
        }
        // SAFETY: writer 已返回且没有 CPU 别名，lease 保完整原页，停止路径等待 Preparing 尾部。
        if let Err(error) = unsafe { lease.page.as_mut().unwrap().begin_device_access(domain) } {
            let error = GmacError::from(error);
            self.record_failure(error);
            return Err(error);
        }
        let result = {
            let mut state = self.state.lock();
            let live = state.phase == Phase::Live;
            let inner: &mut Inner = state.inner.as_mut().unwrap();
            let slot = &mut inner.tx[index];
            slot.page = lease.page.take();
            slot.state = SlotState::Ready;
            if live {
                publish_ready(inner)
            } else {
                Err(GmacError::Io("GMAC stopped during TX preparation"))
            }
        };
        match result {
            Ok(Some(tail)) => self.notify(false, tail),
            Ok(None) => (),
            Err(error) => {
                self.record_failure(error);
                return Err(error);
            }
        }
        Ok(true)
    }

    /// 关闭新 producer、Timer/Work/IRQ 并串行结束通知，确认 reset 后收权归页。
    /// 未确认停止保留整个 Inner；再次调用可继续等待同一硬件，不重放帧。
    pub fn stop_sync(&self) -> Result<(), GmacError> {
        crate::sched::assert_switchable();
        let _control = self.control.lock();
        let close = {
            let mut state = self.state.lock();
            if state.phase == Phase::Stopped {
                return Ok(());
            }
            state.phase = Phase::Stopping;
            self.registers.interrupts(false);
            let close = !state.closed;
            state.closed = true;
            close
        };
        if close {
            self.link_timer.get().unwrap().as_ref().close_sync();
            let registration = self.registration.lock().take();
            if let Some(registration) = registration {
                registration.unregister_sync();
            }
            self.progress.get().unwrap().as_ref().close_sync();
        }
        // writer 在锁外；停止不持短锁等待这些真实执行尾部，通知已由同锁串行。
        loop {
            let busy = {
                let state = self.state.lock();
                state
                    .inner
                    .as_ref()
                    .unwrap()
                    .tx
                    .iter()
                    .any(|slot| slot.state == SlotState::Preparing)
            };
            if !busy {
                break;
            }
            timer::sleep(Duration::from_millis(1));
        }
        if self.state.lock().must_reset {
            self.registers.stop_dma();
            self.registers.request_reset();
            self.registers.wait_reset()?;
        }
        // Work、producer 和 IRQ 均已结束，锁外同步最多64页；错误仍保留原槽拥有。
        for receive in [true, false] {
            for index in 0..RING_LEN {
                let page = {
                    let mut state = self.state.lock();
                    let inner: &mut Inner = state.inner.as_mut().unwrap();
                    let slot = if receive {
                        &mut inner.rx[index]
                    } else {
                        &mut inner.tx[index]
                    };
                    slot.page
                        .take()
                        .expect("GMAC page missing after all CPU tails")
                };
                let mut lease = SlotLease {
                    nic: self,
                    receive,
                    index,
                    page: Some(page),
                };
                if lease.page.as_ref().unwrap().is_device_owned() {
                    // SAFETY: SWR 已真实清零或设备从未发表地址；全部 CPU/IRQ 尾已结束，
                    // 同一 lease 保活独立原页，没有未完成设备访问或并发 CPU 别名。
                    unsafe { lease.page.as_mut().unwrap().finish_device_access() }?;
                }
            }
        }
        let owner = {
            let mut state = self.state.lock();
            state.must_reset = false;
            state.phase = Phase::Stopped;
            state.inner.take().unwrap()
        };
        drop(ManuallyDrop::into_inner(owner));
        Ok(())
    }

    /// 每个初始 RX 页在独立短锁外同步，发表失败仍留同一原槽到确认 reset。
    fn publish(&self) -> Result<(), GmacError> {
        for index in 0..RING_LEN {
            let (page, domain) = {
                let mut state = self.state.lock();
                let inner: &mut Inner = state.inner.as_mut().unwrap();
                inner.rx[index].state = SlotState::Processing;
                (inner.rx[index].page.take().unwrap(), inner.domain)
            };
            let mut lease = SlotLease {
                nic: self,
                receive: true,
                index,
                page: Some(page),
            };
            // SAFETY: 预备原页完整清零，设备尚未取得环地址，lease 独占本页且没有 CPU slice。
            unsafe { lease.page.as_mut().unwrap().begin_device_access(domain) }?;
            let mut state = self.state.lock();
            let inner: &mut Inner = state.inner.as_mut().unwrap();
            let slot = &mut inner.rx[index];
            slot.page = lease.page.take();
            slot.state = SlotState::DeviceOwned;
            inner.rx_ring.arm_rx(index, slot.address)?;
        }
        let state = self.state.lock();
        self.registers
            .configure(state.inner.as_ref().unwrap(), self.mac)
    }

    /// ISR 只应答和记录原因，不读帧、不等待 MDIO、不分配。
    fn handle_interrupt(&self) -> bool {
        let schedule = {
            let mut state = self.state.lock();
            let reason = self.registers.acknowledge();
            if reason == 0 {
                return false;
            }
            self.registers.interrupts(false);
            state.reason |= reason;
            state.pending = true;
            state.phase == Phase::Live
        };
        if schedule {
            self.queue_progress();
        }
        true
    }

    /// 唯一 Work 消费数据 IRQ；PHY 定时票只采样链路，不探测帧完成补救丢失 IRQ。
    fn progress(&self) -> Result<(), GmacError> {
        let (data, reason, phy_due) = {
            let mut state = self.state.lock();
            if state.phase != Phase::Live {
                return Ok(());
            }
            let values = (state.pending, state.reason, state.phy_due);
            state.pending = false;
            state.reason = 0;
            state.phy_due = false;
            values
        };
        if self.registers.fatal(reason) {
            return Err(GmacError::Io("GMAC fatal DMA bus error"));
        }
        if phy_due {
            self.refresh_link()?;
        }
        if data {
            let mut completed = 0;
            while completed < RING_LEN && self.complete_tx()? {
                completed += 1;
            }
            if completed != 0 {
                if let Some(net) = self.net.upgrade() {
                    net.kick_egress();
                }
            }
            let mut received = 0;
            while received < RX_BUDGET && self.receive_one()? {
                received += 1;
            }
            self.rearm(received == RX_BUDGET)?;
        }
        Ok(())
    }

    /// 锁内认领真实 OWN 已清的 TX，锁外结束 payload DMA，再按唯一完成 cursor 归槽。
    fn complete_tx(&self) -> Result<bool, GmacError> {
        let (index, page, status, length) = {
            let mut state = self.state.lock();
            if state.phase != Phase::Live {
                return Ok(false);
            }
            let inner: &mut Inner = state.inner.as_mut().unwrap();
            let index = inner.tx_complete;
            if inner.tx[index].state != SlotState::DeviceOwned {
                return Ok(false);
            }
            let status = inner.tx_ring.status(index)?;
            if device_owned(status) {
                return Ok(false);
            }
            let slot = &mut inner.tx[index];
            slot.state = SlotState::Processing;
            (index, slot.page.take().unwrap(), status, slot.frame_len)
        };
        let mut lease = SlotLease {
            nic: self,
            receive: false,
            index,
            page: Some(page),
        };
        // SAFETY: 对应单一描述符真实 OWN 清零，硬件已结束本 payload；Work 是唯一消费者，
        // 槽保留 Processing 到同步结束，stop_sync 先等当前 Work 全尾。
        unsafe { lease.page.as_mut().unwrap().finish_device_access() }?;
        let mut state = self.state.lock();
        let inner: &mut Inner = state.inner.as_mut().unwrap();
        inner.tx[index].page = lease.page.take();
        inner.tx[index].state = SlotState::Free;
        inner.tx_complete = (index + 1) % RING_LEN;
        if descriptors::transmit_error(status) {
            state.statistics.tx_errors = state.statistics.tx_errors.wrapping_add(1);
        } else {
            state.statistics.tx_packets = state.statistics.tx_packets.wrapping_add(1);
            state.statistics.tx_bytes = state.statistics.tx_bytes.wrapping_add(length as u64);
        }
        Ok(true)
    }

    /// RX 真实完成后锁外同步、输入原协议栈，再把同一固定原页交回原槽。
    fn receive_one(&self) -> Result<bool, GmacError> {
        let (index, page, status, domain) = {
            let mut state = self.state.lock();
            if state.phase != Phase::Live {
                return Ok(false);
            }
            let inner: &mut Inner = state.inner.as_mut().unwrap();
            let index = inner.rx_next;
            let slot = &mut inner.rx[index];
            if slot.state != SlotState::DeviceOwned {
                return Ok(false);
            }
            let status = inner.rx_ring.status(index)?;
            if device_owned(status) {
                return Ok(false);
            }
            slot.state = SlotState::Processing;
            (index, slot.page.take().unwrap(), status, inner.domain)
        };
        let mut lease = SlotLease {
            nic: self,
            receive: true,
            index,
            page: Some(page),
        };
        // SAFETY: 本槽真实 OWN 清除且仅引用此页，DMA 已结束完整接收；唯一 Work 保页到重投。
        unsafe { lease.page.as_mut().unwrap().finish_device_access() }?;
        if let Some(length) = descriptors::received(self.registers.generation, status) {
            {
                let mut state = self.state.lock();
                state.statistics.rx_packets = state.statistics.rx_packets.wrapping_add(1);
                state.statistics.rx_bytes = state.statistics.rx_bytes.wrapping_add(length as u64);
            }
            // SAFETY: 预备页完整清零，本轮 DMA 已收权，length 经真实帧/FCS范围验证，
            // 原协议输入只在本调用借用，返回后不保留 slice。
            let bytes = unsafe { lease.page.as_ref().unwrap().bytes() };
            let accepted = self
                .net
                .upgrade()
                .is_some_and(|net| net.receive_ethernet(&bytes[..length]).is_ok());
            if !accepted {
                let mut state = self.state.lock();
                state.statistics.rx_dropped = state.statistics.rx_dropped.wrapping_add(1);
            }
        } else {
            let mut state = self.state.lock();
            state.statistics.rx_errors = state.statistics.rx_errors.wrapping_add(1);
            state.statistics.rx_dropped = state.statistics.rx_dropped.wrapping_add(1);
        }
        if self.state.lock().phase != Phase::Live {
            return Ok(true);
        }
        // SAFETY: receive_ethernet 及全部 CPU 借用已结束；同一 lease 保活原页，
        // stop_sync 等待本 Work 返回才 reset 和收权。
        unsafe { lease.page.as_mut().unwrap().begin_device_access(domain) }?;
        let tail = {
            let mut state = self.state.lock();
            let live = state.phase == Phase::Live;
            let inner: &mut Inner = state.inner.as_mut().unwrap();
            let slot = &mut inner.rx[index];
            slot.page = lease.page.take();
            slot.state = SlotState::DeviceOwned;
            if !live {
                return Ok(true);
            }
            inner.rx_ring.arm_rx(index, slot.address)?;
            inner.rx_next = (index + 1) % RING_LEN;
            inner.rx_ring.address(index)
        };
        self.notify(true, tail);
        Ok(true)
    }

    /// arm 后重查状态和当前完成槽，关闭早完成窗口；预算耗尽保留原 Work 后继票。
    fn rearm(&self, budget_full: bool) -> Result<(), GmacError> {
        let again = {
            let mut state = self.state.lock();
            if state.phase != Phase::Live {
                return Ok(());
            }
            self.registers.interrupts(true);
            let reason = self.registers.acknowledge();
            state.reason |= reason;
            let inner: &Inner = state.inner.as_ref().unwrap();
            let rx = &inner.rx[inner.rx_next];
            let tx = &inner.tx[inner.tx_complete];
            let ready = (rx.state == SlotState::DeviceOwned
                && !device_owned(inner.rx_ring.status(inner.rx_next)?))
                || (tx.state == SlotState::DeviceOwned
                    && !device_owned(inner.tx_ring.status(inner.tx_complete)?));
            if budget_full || reason != 0 || ready || state.pending {
                self.registers.interrupts(false);
                state.pending = true;
                true
            } else {
                false
            }
        };
        if again {
            self.queue_progress();
        }
        Ok(())
    }

    /// 只由 PHY 到期票采样管理总线，链路配置及原 egress 唤醒在队列锁外进行。
    fn refresh_link(&self) -> Result<(), GmacError> {
        let link = self.phy.sample()?;
        let changed = self.state.lock().link != link;
        if changed {
            self.registers.set_link(None);
            if let Some(link) = link {
                self.phy.configure_link(link)?;
                self.update_tx_clock(link)?;
            }
            self.registers.set_link(link);
            self.state.lock().link = link;
            if let Some(net) = self.net.upgrade() {
                net.kick_egress();
            }
        }
        let state = self.state.lock();
        if state.phase == Phase::Live {
            self.link_timer
                .get()
                .unwrap()
                .as_ref()
                .arm(time::now().checked_add(Duration::from_secs(1)).unwrap())
                .map_err(|_| GmacError::Io("GMAC PHY timer is closed"))?;
        }
        Ok(())
    }

    /// 内部 TX 时钟由协商速率决定，外部 RGMII RX 源则由 PHY 自适应。
    fn update_tx_clock(&self, link: Link) -> Result<(), GmacError> {
        if self.registers.generation != Generation::Dwmac4 || self.resources.tx_use_rgmii_clock {
            return Ok(());
        }
        if self.resources.setup.clocks.count < 4 {
            return Err(GmacError::Resource("GMAC TX clock is missing"));
        }
        let mut setup = DeviceSetup::default();
        setup.clocks.count = 1;
        setup.clocks.entries[0] = self.resources.setup.clocks.entries[3];
        setup.clocks.entries[0].rate = Some(match link.speed {
            1000 => 125_000_000,
            100 => 25_000_000,
            10 => 2_500_000,
            _ => return Err(GmacError::Io("invalid negotiated GMAC speed")),
        });
        // SAFETY: 同一 MAC 的真实规范化时钟引用和永久 provider 映射仍由根拥有，
        // 此动作只包含 TX clock，不重复执行复位或 pin 动作；Work 独占链路配置。
        unsafe { board_setup::apply_setup(&setup, Duration::ZERO) }.map_err(GmacError::Resource)
    }

    /// 极短 MMIO 与最新 tail 在同锁串行，避免并发 writer 将旧 tail 写回到新边界之前。
    fn notify(&self, receive: bool, tail: u64) {
        let state = self.state.lock();
        if state.phase != Phase::Live {
            return;
        }
        let tail = if receive {
            tail
        } else {
            let inner: &Inner = state.inner.as_ref().unwrap();
            let next = if inner.tx_publish == 0 {
                RING_LEN
            } else {
                inner.tx_publish
            };
            inner.tx_ring.address(next)
        };
        self.registers.notify(receive, tail);
    }

    /// 重用原 Work 的 pending/running 两票，不新建轮询线程。
    fn queue_progress(&self) {
        let _ = self.progress.get().unwrap().as_ref().queue();
    }

    /// 首次错误关闭硬件中断和新提交，原页仍在同一拥有者中等待正常停止。
    fn record_failure(&self, error: GmacError) {
        let mut state = self.state.lock();
        state.failure.get_or_insert(error);
        if matches!(state.phase, Phase::Live | Phase::Prepared) {
            state.phase = Phase::Faulted;
        }
        self.registers.interrupts(false);
    }
}

impl Drop for Dwmac {
    /// 未发表失败可归还 CPU 页；发表实例只有确认停止后才能释放稳定 context。
    fn drop(&mut self) {
        let owner = {
            let mut state = self.state.lock();
            assert!(
                !state.must_reset,
                "dropping GMAC before confirmed DMA reset"
            );
            assert!(
                matches!(state.phase, Phase::Prepared | Phase::Stopped),
                "dropping live GMAC context"
            );
            state.inner.take()
        };
        if let Some(owner) = owner {
            drop(ManuallyDrop::into_inner(owner));
        }
    }
}

/// 锁外 writer/输入/同步始终保留唯一原页，所有错误都归回原槽而不析构活动 DMA。
struct SlotLease<'a> {
    nic: &'a Dwmac,
    receive: bool,
    index: usize,
    page: Option<MappedPages>,
}

impl Drop for SlotLease<'_> {
    /// 归回真实 CPU 或设备拥有状态，队列错误不提前释放 payload。
    fn drop(&mut self) {
        if let Some(page) = self.page.take() {
            let mut state = self.nic.state.lock();
            let inner: &mut Inner = state.inner.as_mut().unwrap();
            let slot = if self.receive {
                &mut inner.rx[self.index]
            } else {
                &mut inner.tx[self.index]
            };
            assert!(slot.page.is_none(), "GMAC slot already has a page");
            slot.state = if page.is_device_owned() {
                SlotState::DeviceOwned
            } else {
                SlotState::Free
            };
            slot.page = Some(page);
        }
    }
}

/// 原页与两个 ring 在任何硬件地址发表前一次预备完成。
fn prepare_inner(
    domain: DmaDomain,
    generation: Generation,
    coherent: bool,
) -> Result<Inner, GmacError> {
    Ok(Inner {
        domain,
        rx_ring: Ring::new(domain, generation, coherent)?,
        tx_ring: Ring::new(domain, generation, coherent)?,
        rx: prepare_pool(domain, BufferDirection::DeviceToDriver)?,
        tx: prepare_pool(domain, BufferDirection::DriverToDevice)?,
        rx_next: 0,
        tx_claim: 0,
        tx_publish: 0,
        tx_complete: 0,
    })
}

/// 每个帧占一个原页，地址受限申请由原 buddy 一次选择，不靠重复申请碰运气。
fn prepare_pool(domain: DmaDomain, direction: BufferDirection) -> Result<Vec<Slot>, GmacError> {
    let mut slots = Vec::new();
    slots
        .try_reserve_exact(RING_LEN)
        .map_err(|_| GmacError::NoMemory)?;
    let (start, end) = domain.allocation_window();
    for _ in 0..RING_LEN {
        let pages = mm::alloc_pages_in(0, true, start, end).ok_or(GmacError::NoMemory)?;
        let page = MappedPages::new(pages, 0, BUFFER_LEN, direction)?;
        let address = page.segment(domain)?.address.value();
        slots.push(Slot {
            state: SlotState::Free,
            page: Some(page),
            address,
            frame_len: 0,
        });
    }
    Ok(slots)
}

/// 并发 writer 完成次序可以不同，只有连续 Ready 前缀才按硬件 ring 顺序发表。
fn publish_ready(inner: &mut Inner) -> Result<Option<u64>, GmacError> {
    let mut tail = None;
    for _ in 0..RING_LEN {
        let index = inner.tx_publish;
        let slot = &mut inner.tx[index];
        if slot.state != SlotState::Ready {
            break;
        }
        inner.tx_ring.arm_tx(index, slot.address, slot.frame_len)?;
        slot.state = SlotState::DeviceOwned;
        inner.tx_publish = (index + 1) % RING_LEN;
        tail = Some(inner.tx_ring.address(index + 1));
    }
    Ok(tail)
}

/// 只消费已规范化的 JH7110 syscon offset/shift；LA 的 PCI/平台没有此动作。
///
/// # Safety
/// 所选 syscon 完整永久映射且由此 MAC 独占其字段，provider 资源来自同次发现。
unsafe fn configure_interface(
    resources: &GmacResources,
    generation: Generation,
) -> Result<(), GmacError> {
    if generation != Generation::Dwmac4 {
        return Ok(());
    }
    let offset = resources.sysreg[0] as usize;
    let shift = resources.sysreg[1];
    if resources.syscon.start == 0
        || offset % 4 != 0
        || 29 < shift
        || resources
            .syscon
            .end
            .checked_sub(resources.syscon.start)
            .is_none_or(|length| length < offset + 4)
    {
        return Err(GmacError::Resource(
            "GMAC interface syscon field is invalid",
        ));
    }
    let address = arch::mmio_address(resources.syscon.start) + offset;
    let mode = if resources.phy_mode == boot::PhyMode::Rmii {
        4
    } else {
        1
    };
    // SAFETY: offset/shift 和完整窗口已核对，调用者独占此实际字段，volatile 不借设备内存引用。
    unsafe {
        let value = ptr::read_volatile(address as *const u32);
        ptr::write_volatile(
            address as *mut u32,
            (value & !(7 << shift)) | (mode << shift),
        );
    }
    arch::dma_barrier();
    Ok(())
}

/// 未指定 MAC 不能以全零、广播或组播地址替代。
fn valid_mac(mac: [u8; 6]) -> bool {
    mac != [0; 6] && mac[0] & 1 == 0
}

/// action context 是 prepare 返回的同一稳定 Arc 值，根强持到同步注销全尾。
///
/// # Safety
/// context 必须是本设备 register 保存的指针，完整 handler 期有效且未移动。
unsafe fn irq_callback(context: *mut ()) -> bool {
    // SAFETY: 注册契约要求根强拥有覆盖全部 action 执行引用到 unregister_sync。
    unsafe { &*context.cast::<Dwmac>() }.handle_interrupt()
}

/// 原 CPU0 Work 唯一处理 RX/TX 和 PHY，错误留下真实 owning context 待同步停止。
fn progress_callback(work: Pin<&Work>) {
    // SAFETY: context 是 prepare 的稳定 Arc 地址，根在 close_sync 全尾前保持同一强拥有。
    let nic = unsafe { &*(work.context() as *const Dwmac) };
    if let Err(error) = nic.progress() {
        nic.record_failure(error);
        crate::klog!(
            crate::klog::Level::Error,
            "GMAC progress failed: {:?}",
            error
        );
    }
}

/// PHY 秒级定时票只标记管理采样，不读取或轮询数据 descriptor。
fn link_expired(timer: Pin<&Timer>) {
    // SAFETY: 根强拥有覆盖此 Timer 回调至 close_sync 返回，context 来自相同稳定 Arc。
    let nic = unsafe { &*(timer.context() as *const Dwmac) };
    let schedule = {
        let mut state = nic.state.lock();
        if state.phase == Phase::Live {
            state.phy_due = true;
            true
        } else {
            false
        }
    };
    if schedule {
        nic.queue_progress();
    }
}
