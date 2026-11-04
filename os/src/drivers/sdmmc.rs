//! SD 单命令控制器；原 progress Work 睡眠等待 IRQ，payload 仅经 CPU 回调访问。
//! 命令/IDMAC 实现已改为原页拥有、可失败初始化及确认停止，许可与来源见 sdmmc/LICENSE。

mod protocol;
mod registers;

use alloc::sync::Arc;
use core::mem::ManuallyDrop;
use core::pin::{pin, Pin};
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};

use crate::block::{
    BlkOperation, BlockError, IoResult, QueueLimits, NATIVE_TAG_COUNT, SECTOR_BYTES,
};
use crate::boot::{self, MmcResources};
use crate::drivers::{board_setup, mmio_region_address};
use crate::irq::{self, IrqNumber};
use crate::mm::dma::{CoherentDma, DmaDirection, DmaDomain};
use crate::mm::PAGE_SIZE;
use crate::sync::{SleepMutex, SpinLock};
use crate::time::{self, Duration, MonotonicInstant};
use crate::timer::{self, Timer, TimerKind};
use crate::wait::{WaitNode, WaitQueue, Waiter, Waker};

use protocol::*;
use registers::*;
pub(crate) use registers::{FIFO_MAX_DEPTH, FIFO_MIN_DEPTH};

const COMMAND_MS: u64 = 2000;
const PROGRAM_MS: u64 = 10_000;
const RESET_MS: u64 = 500;
const RESET_HOLD: Duration = Duration::from_micros(10);
const POWER_UP_POLL_INTERVAL: Duration = Duration::from_millis(10);
const MAX_PAYLOAD_SEGMENTS: usize = 32;
const RESET_RUNNING: u32 = 0;
const RESET_INTERFACE: u32 = 1;
const RESET_FIFO: u32 = 2;
const RESET_STOPPED: u32 = 3;

/// 设备真正访问的两块原页；原 Bio 页不建立此设备的 DMA 映射。
struct Memory {
    bounce: CoherentDma,
    descriptors: CoherentDma,
}

/// 卡协议状态仅由可睡命令执行者修改。
#[derive(Clone, Copy)]
struct Card {
    rca: u32,
    block_addressed: bool,
}

/// 单硬件通道与八软件完成槽由共同 Disk 管理；本对象只拥有硬件及固定控制内存。
pub(crate) struct Sdmmc {
    regs: Registers,
    resources: MmcResources,
    irq: IrqNumber,
    input_frequency: AtomicU64,
    memory: ManuallyDrop<Memory>,
    command: SleepMutex<Option<Card>>,
    registers: SpinLock<()>,
    events: AtomicU32,
    waiters: WaitQueue,
    capacity: AtomicU64,
    readonly: AtomicBool,
    published: AtomicBool,
    stopping: AtomicBool,
    reset_stage: AtomicU32,
    dma_live: AtomicBool,
}

impl Sdmmc {
    /// 在普通可睡任务中预备稳定 Arc、bounce 和 IDMAC 原页后初始化卡，返回时 IRQ 输出关闭。
    /// 仅真实 GPIO/CDETECT 无卡返回 None；资源、识别与停止失败不解释为介质缺失。
    ///
    /// # Safety
    /// 资源来自唯一启动快照；寄存器、syscon、clock/reset/pin 及独立 CD GPIO provider
    /// 均已有完整永久设备映射；本设备与所选 CD pin 由调用者独占，共享设置遵守板设置锁。
    /// 调用者必须在 publish 前登记保活本对象的 IRQ action，之后按共同 Disk 停止协议使用。
    pub(crate) unsafe fn prepare(
        resources: &MmcResources,
    ) -> Result<Option<Arc<Self>>, BlockError> {
        let base =
            mmio_region_address(resources.registers).map_err(|_| BlockError::InvalidRange)?;
        let span = resources
            .registers
            .end
            .checked_sub(resources.registers.start)
            .ok_or(BlockError::InvalidRange)?;
        if base % 8 != 0
            || span < IDINTEN + REGISTER_BYTES
            || !(resources.bus_width == 1 || resources.bus_width == 4)
            || resources
                .fifo_depth
                .is_some_and(|depth| !(FIFO_MIN_DEPTH..=FIFO_MAX_DEPTH).contains(&depth))
            || resources.max_frequency == 0
            || resources.setup.clocks.count != 2
        {
            return Err(BlockError::Unsupported);
        }
        let irq =
            irq::map_source(resources.interrupt_source).map_err(|_| BlockError::InvalidRange)?;
        let domain = DmaDomain::new(
            resources.dma_mask.min(u32::MAX as u64),
            0,
            usize::MAX,
            0,
            boot::info().board.cache_line_size,
            resources.dma_coherent,
        )?;
        let memory = Memory {
            bounce: CoherentDma::allocate(domain, BOUNCE_BYTES)?,
            descriptors: CoherentDma::allocate(domain, PAGE_SIZE)?,
        };
        let controller = Arc::try_new(Self {
            regs: Registers(base),
            resources: *resources,
            irq,
            input_frequency: AtomicU64::new(0),
            memory: ManuallyDrop::new(memory),
            command: SleepMutex::new(None),
            registers: SpinLock::new(()),
            events: AtomicU32::new(0),
            waiters: WaitQueue::new(),
            capacity: AtomicU64::new(0),
            readonly: AtomicBool::new(false),
            published: AtomicBool::new(false),
            stopping: AtomicBool::new(false),
            reset_stage: AtomicU32::new(RESET_RUNNING),
            dma_live: AtomicBool::new(false),
        })
        .map_err(|_| BlockError::NoMemory)?;
        // SAFETY: 调用者保证规范化 provider 的独占映射，全部可失败分配已完成。
        unsafe { board_setup::apply_setup(&resources.setup, RESET_HOLD) }
            .map_err(|_| BlockError::Unsupported)?;
        // SAFETY: apply_setup 已成功，时钟 provider 映射由 prepare 的调用契约保活。
        let frequency = unsafe { board_setup::clock_rate(&resources.setup.clocks.entries[1]) }
            .map_err(|_| BlockError::Unsupported)?;
        controller
            .input_frequency
            .store(frequency, Ordering::Release);
        // 初始化只执行命令与八字节 SCR PIO，尚未向硬件发表任何本对象的页地址。
        match controller.initialize() {
            Ok(true) => Ok(Some(controller)),
            outcome => {
                controller.request_reset();
                if !controller.wait_reset(deadline(RESET_MS)) {
                    // 强引用保留同一拥有者；不能让准备错误析构可能仍被硬件使用的内存。
                    let _retained = Arc::into_raw(controller);
                    panic!("SDMMC initialization stop is unconfirmed");
                }
                outcome.map(|_| None)
            }
        }
    }

    /// 返回真实卡容量及单硬件命令、八软件回调槽的共同限额。
    pub(crate) fn limits(&self) -> QueueLimits {
        QueueLimits {
            capacity_sectors: self.capacity.load(Ordering::Acquire),
            max_bytes: BOUNCE_BYTES,
            max_segments: MAX_PAYLOAD_SEGMENTS,
            max_segment_bytes: BOUNCE_BYTES as u32,
            logical_block_size: SECTOR_BYTES as u32,
            dma_boundary_mask: u64::MAX,
            readonly: self.readonly.load(Ordering::Acquire),
            flush: true,
            queue_depth: 1,
            tag_count: NATIVE_TAG_COUNT,
        }
    }

    /// 返回已核当前 IRQ domain 的真实设备源。
    pub(crate) fn interrupt(&self) -> IrqNumber {
        self.irq
    }

    /// IRQ action 已登记并保活控制器后开放输出；不重新识别卡或分配内存。
    pub(crate) fn publish(&self) -> Result<(), BlockError> {
        if self.stopping.load(Ordering::Acquire) || self.capacity.load(Ordering::Acquire) == 0 {
            return Err(BlockError::NotReady);
        }
        if self.published.swap(true, Ordering::AcqRel) {
            return Err(BlockError::Busy);
        }
        self.set_interrupts(true);
        Ok(())
    }

    /// ISR 只认领本设备使能的原因，先记录 W1C 结果再锁外唤醒，不访问数据或可睡锁。
    pub(crate) fn acknowledge(&self) -> bool {
        let events = {
            let _guard = self.registers.lock();
            let controller = self.regs.read(MINTSTS) & IRQ_MASK;
            let dma = self.regs.read(IDSTS) & self.regs.read(IDINTEN) & IDMAC_IRQ;
            if controller == 0 && dma == 0 {
                return false;
            }
            self.regs.write(RINTSTS, controller);
            self.regs.write(IDSTS, dma);
            let events = controller
                | if dma & IDMAC_COMPLETION != 0 {
                    IDMAC_DONE
                } else {
                    0
                }
                | if dma & IDMAC_ERRORS != 0 {
                    IDMAC_ERROR
                } else {
                    0
                };
            self.events.fetch_or(events, Ordering::Release);
            events
        };
        if events != 0 {
            self.waiters.wake_all();
        }
        true
    }

    /// 队列短锁及 ISR 均可调用；停止开始后不得重新打开输出。
    pub(crate) fn set_interrupts(&self, enabled: bool) {
        let _guard = self.registers.lock();
        let enabled = enabled && !self.stopping.load(Ordering::Acquire);
        self.regs
            .update(CTRL, INT_ENABLE, if enabled { INT_ENABLE } else { 0 });
    }

    /// 武装后复查 mailbox 与硬件原因，关闭登记前完成的窗口。
    pub(crate) fn arm_and_check(&self) -> bool {
        self.set_interrupts(true);
        self.acknowledge();
        self.events.load(Ordering::Acquire) != 0
    }

    /// 原 progress Work 锁外执行一次请求；Write 交权前、Read 收权后才调用一次 CPU transfer。
    ///
    /// # Safety
    /// 回调只复制本次全部原 payload，不能睡眠、分配或重入控制器；bytes 必须匹配 payload。
    /// Core 保活本对象到所有 IRQ/Work 停止，StopUnconfirmed 时保留完整槽和后端拥有。
    pub(crate) unsafe fn execute(
        &self,
        operation: BlkOperation,
        sector: u64,
        bytes: usize,
        transfer: impl FnOnce(*mut u8) -> Result<usize, BlockError>,
    ) -> IoResult {
        let state = self.command.lock();
        if !self.published.load(Ordering::Acquire) || self.stopping.load(Ordering::Acquire) {
            return failed(BlockError::QueueStopped);
        }
        let card = state.expect("initialized SD card is missing");
        let result = if operation == BlkOperation::Flush {
            if sector != 0 || bytes != 0 {
                Err(BlockError::InvalidRange)
            } else {
                self.wait_card(card, deadline(PROGRAM_MS), true).map(|_| 0)
            }
        } else {
            self.transfer(card, operation, sector, bytes, transfer)
        };
        match result {
            Ok(completed) => IoResult {
                completed,
                error: if completed == bytes {
                    None
                } else {
                    Some(BlockError::ShortIo)
                },
            },
            Err(error) => {
                // 失败请求不重放；全控制器 reset 终止后续准入并证明没有活 DMA。
                self.request_reset();
                if self.wait_reset(deadline(RESET_MS)) {
                    failed(if error == BlockError::StopUnconfirmed {
                        BlockError::DeviceFault
                    } else {
                        error
                    })
                } else {
                    failed(BlockError::StopUnconfirmed)
                }
            }
        }
    }

    /// 拒绝后续命令并发出真实控制器/IDMAC reset，不在短队列锁内等待。
    pub(crate) fn request_reset(&self) {
        let _guard = self.registers.lock();
        if self.stopping.swap(true, Ordering::AcqRel) {
            return;
        }
        self.regs.write(IDINTEN, 0);
        self.regs.write(INTMASK, 0);
        self.regs
            .update(CTRL, INT_ENABLE | DMA_ENABLE | USE_IDMAC, RESET);
        self.regs.update(BMOD, BMOD_ENABLE, BMOD_RESET);
        self.reset_stage.store(RESET_INTERFACE, Ordering::Release);
        self.waiters.wake_all();
    }

    /// 每次只推进一次有界 MMIO 阶段，确认 reset、DMA_REQ 和第二次 FIFO reset 后才允许归页。
    pub(crate) fn reset_complete(&self) -> bool {
        let _guard = self.registers.lock();
        match self.reset_stage.load(Ordering::Acquire) {
            RESET_INTERFACE => {
                if self.regs.read(CTRL) & RESET != 0
                    || self.regs.read(BMOD) & BMOD_RESET != 0
                    || self.regs.read(STATUS) & DMA_REQ != 0
                {
                    return false;
                }
                self.regs.update(CTRL, 0, FIFO_RESET);
                self.reset_stage.store(RESET_FIFO, Ordering::Release);
                false
            }
            RESET_FIFO => {
                if self.regs.read(CTRL) & FIFO_RESET != 0 {
                    return false;
                }
                self.regs.write(DBADDR, 0);
                self.regs.write(IDSTS, IDMAC_IRQ);
                self.regs.write(RINTSTS, u32::MAX);
                self.events.store(0, Ordering::Release);
                self.dma_live.store(false, Ordering::Release);
                self.reset_stage.store(RESET_STOPPED, Ordering::Release);
                true
            }
            RESET_STOPPED => true,
            _ => false,
        }
    }

    /// 锁外等待已经发出的 reset；这只检查硬件停止，不消费完成或补救遗失 IRQ。
    fn wait_reset(&self, until: MonotonicInstant) -> bool {
        loop {
            if self.reset_complete() {
                return true;
            }
            if until <= time::now() {
                return false;
            }
            timer::sleep(Duration::from_millis(1));
        }
    }

    /// 初始化阶段使用真实单调期限轮询；运行期命令完成不调用此路径。
    fn poll(
        &self,
        until: MonotonicInstant,
        mut condition: impl FnMut() -> bool,
    ) -> Result<(), BlockError> {
        while !condition() {
            if until <= time::now() {
                return Err(BlockError::DeviceFault);
            }
            core::hint::spin_loop();
        }
        Ok(())
    }

    /// 识别期命令可轮询完成；只有 CMD8 的真实响应超时用于识别旧 SDSC。
    fn command_polled(
        &self,
        index: u32,
        argument: u32,
        flags: u32,
    ) -> Result<[u32; 4], BlockError> {
        self.poll(deadline(COMMAND_MS), || self.regs.read(CMD) & START == 0)?;
        self.regs.write(RINTSTS, u32::MAX);
        self.regs.write(CMDARG, argument);
        self.regs.write(CMD, START | HOLD | index | flags);
        self.poll(deadline(COMMAND_MS), || {
            self.regs.read(RINTSTS) & (CMD_DONE | ERRORS) != 0
        })?;
        let status = self.regs.read(RINTSTS);
        self.regs.write(RINTSTS, status);
        if status & ERRORS != 0 {
            if status & ERRORS == RESPONSE_TIMEOUT {
                return Err(BlockError::NotReady);
            }
            return Err(BlockError::Io);
        }
        Ok(self.regs.response())
    }

    /// 三个 update-clock 均确认 START 已清除，核分频与开钟读回后返回实际卡时钟。
    fn clock(&self, target: u32) -> Result<u64, BlockError> {
        if target == 0 {
            return Err(BlockError::InvalidRange);
        }
        let input = self.input_frequency.load(Ordering::Acquire);
        if input == 0 {
            return Err(BlockError::Unsupported);
        }
        let divisor = if input <= target as u64 {
            0
        } else {
            (input + CLOCK_DIVIDER_FACTOR * target as u64 - 1)
                / (CLOCK_DIVIDER_FACTOR * target as u64)
        };
        if u64::from(CLOCK_DIVIDER_MASK) < divisor {
            return Err(BlockError::Unsupported);
        }
        self.regs.write(CLKENA, 0);
        self.update_clock()?;
        self.regs.write(CLKSRC, 0);
        self.regs.write(CLKDIV, divisor as u32);
        self.update_clock()?;
        self.regs.write(CLKENA, CLOCK_ENABLE);
        self.update_clock()?;
        if self.regs.read(CLKDIV) & CLOCK_DIVIDER_MASK != divisor as u32
            || self.regs.read(CLKENA) & CLOCK_ENABLE == 0
        {
            return Err(BlockError::Io);
        }
        Ok(if divisor == 0 {
            input
        } else {
            input / (CLOCK_DIVIDER_FACTOR * divisor)
        })
    }

    /// update-clock 是无 CMD_DONE 的控制动作，仅用于尚未发表的识别期。
    fn update_clock(&self) -> Result<(), BlockError> {
        self.regs
            .write(CMD, START | HOLD | WAIT_DATA | UPDATE_CLOCK);
        self.poll(deadline(COMMAND_MS), || self.regs.read(CMD) & START == 0)?;
        if self.regs.read(RINTSTS) & ERRORS != 0 {
            return Err(BlockError::Io);
        }
        Ok(())
    }

    /// 识别 SD、解析实际容量并完成 SCR/受支持总线宽度与默认速度；false 只表示真实缺卡。
    fn initialize(&self) -> Result<bool, BlockError> {
        self.regs.write(CTRL, RESET);
        self.regs.write(BMOD, BMOD_RESET);
        self.poll(deadline(RESET_MS), || {
            self.regs.read(CTRL) & RESET == 0 && self.regs.read(BMOD) & BMOD_RESET == 0
        })?;
        let hcon = self.regs.read(HCON);
        let version = self.regs.read(VERID);
        let (fifo_offset, fifo_width) = protocol::fifo_geometry(
            version,
            hcon,
            self.resources.data_offset,
            self.resources.registers.end - self.resources.registers.start,
        )?;
        // 仅实现内部 DMA 的 32 位地址格式；64 位 IDMAC 寄存器布局不同。
        if hcon & HCON_ADDRESS_64BIT != 0
            || (hcon >> HCON_DMA_INTERFACE_SHIFT) & HCON_DMA_INTERFACE_MASK != 0
        {
            return Err(BlockError::Unsupported);
        }
        let present = if let Some(input) = self.resources.card_detect.as_ref() {
            // SAFETY: 本对象仅由 prepare 构造；独立 GPIO/SYS CRG 永久映射和 CD pin
            // 独占由其调用者保证，输入配置与其它板设置通过同一可睡锁串行。
            let raw = unsafe { board_setup::read_gpio_input(input) }
                .map_err(|_| BlockError::Unsupported)?;
            let present = (raw != input.active_low) ^ self.resources.cd_inverted;
            crate::klog!(
                crate::klog::Level::Info,
                "SDMMC CD GPIO pin={} raw={} active_low={} inverted={} present={}",
                input.pin,
                raw,
                input.active_low,
                self.resources.cd_inverted,
                present
            );
            present
        } else {
            let detect = self.regs.read(CDETECT);
            let present = (detect & CARD_DETECT_ABSENT == 0) ^ self.resources.cd_inverted;
            crate::klog!(
                crate::klog::Level::Info,
                "SDMMC CD native={:#x} inverted={} present={}",
                detect,
                self.resources.cd_inverted,
                present
            );
            present
        };
        if !present {
            return Ok(false);
        }
        self.regs.write(INTMASK, 0);
        self.regs.write(IDINTEN, 0);
        self.regs.write(PWREN, CARD_POWER_ENABLE);
        let power_delay = Duration::from_millis(self.resources.post_power_on_delay_ms as u64);
        timer::sleep(power_delay);
        self.regs.write(CTYPE, CTYPE_1BIT);
        self.regs.write(UHS, 0);
        self.regs.write(TMOUT, u32::MAX);
        let fifoth = if self.resources.fifo_depth.is_none() {
            self.regs.read(FIFOTH)
        } else {
            0
        };
        let depth = protocol::fifo_depth(self.resources.fifo_depth, fifoth)?;
        self.regs.write(
            FIFOTH,
            ((depth / 2 - 1) << FIFO_RX_WATERMARK_SHIFT) | (depth / 2),
        );
        let identification_frequency = self.clock(IDENTIFICATION_HZ)?;
        timer::sleep(power_delay);
        self.command_polled(GO_IDLE_STATE, 0, INITIALIZE)?;
        let version2 = match self.command_polled(SEND_IF_COND, IF_COND_ARGUMENT, RESPONSE | CRC) {
            Ok(response) if response[0] & IF_COND_RESPONSE_MASK == IF_COND_ARGUMENT => true,
            Err(BlockError::NotReady) => false,
            _ => return Err(BlockError::Unsupported),
        };
        let until = deadline(COMMAND_MS);
        let ocr = loop {
            let app = self.command_polled(APP_CMD, 0, RESPONSE | CRC)?[0];
            protocol::check_r1(app)?;
            if app & R1_APP_CMD == 0 {
                return Err(BlockError::Unsupported);
            }
            let ocr = self.command_polled(
                APP_OP_COND,
                OCR_VOLTAGE_27_36 | if version2 { OCR_HIGH_CAPACITY } else { 0 },
                RESPONSE,
            )?[0];
            if ocr & OCR_VOLTAGE_27_36 == 0 {
                return Err(BlockError::Unsupported);
            }
            if ocr & OCR_POWER_UP_COMPLETE != 0 {
                break ocr;
            }
            if until <= time::now() {
                return Err(BlockError::NotReady);
            }
            timer::sleep(POWER_UP_POLL_INTERVAL);
        };
        self.command_polled(ALL_SEND_CID, 0, RESPONSE | LONG | CRC)?;
        let r6 = self.command_polled(SEND_RELATIVE_ADDR, 0, RESPONSE | CRC)?[0];
        if r6 & R6_ERRORS != 0 || r6 >> RCA_SHIFT == 0 {
            return Err(BlockError::Io);
        }
        let card = Card {
            rca: r6 & RCA_MASK,
            block_addressed: ocr & OCR_HIGH_CAPACITY != 0,
        };
        let csd =
            protocol::csd_bits(self.command_polled(SEND_CSD, card.rca, RESPONSE | LONG | CRC)?);
        let sectors = protocol::capacity(csd, card.block_addressed)?;
        protocol::check_r1(self.command_polled(SELECT_CARD, card.rca, RESPONSE | CRC)?[0])?;
        self.wait_card(card, deadline(PROGRAM_MS), false)?;
        if !card.block_addressed {
            protocol::check_r1(
                self.command_polled(
                    SET_BLOCKLEN,
                    SECTOR_BYTES as u32,
                    RESPONSE | CRC | WAIT_DATA,
                )?[0],
            )?;
        }
        self.app_polled(card)?;
        let scr = self.scr(fifo_offset, fifo_width)?;
        if scr >> SCR_STRUCTURE_SHIFT != 0 || scr & SCR_BUS_WIDTH_1 == 0 {
            return Err(BlockError::Unsupported);
        }
        let mut bus_width = 1;
        if self.resources.bus_width == 4 && scr & SCR_BUS_WIDTH_4 != 0 {
            self.app_polled(card)?;
            protocol::check_r1(
                self.command_polled(
                    APP_SET_BUS_WIDTH,
                    SD_BUS_WIDTH_4,
                    RESPONSE | CRC | WAIT_DATA,
                )?[0],
            )?;
            self.regs.write(CTYPE, CTYPE_4BIT);
            bus_width = 4;
        }
        let card_frequency =
            self.clock(protocol::transfer_frequency(csd)?.min(self.resources.max_frequency))?;
        self.wait_card(card, deadline(PROGRAM_MS), false)?;
        let write_protect = self.regs.read(WRTPRT) & CARD_WRITE_PROTECTED != 0;
        let readonly =
            (!self.resources.disable_wp && write_protect) || csd & CSD_WRITE_PROTECT != 0;
        self.readonly.store(readonly, Ordering::Release);
        crate::klog!(
            crate::klog::Level::Info,
            "SDMMC OCR={:#010x} RCA={:#06x} CSD={:#034x} SCR={:#018x}",
            ocr,
            card.rca >> RCA_SHIFT,
            csd,
            scr
        );
        crate::klog!(
            crate::klog::Level::Info,
            "SDMMC sectors={} bus_width={} readonly={} WRTPRT={} disable_wp={}",
            sectors,
            bus_width,
            readonly,
            write_protect,
            self.resources.disable_wp
        );
        crate::klog!(
            crate::klog::Level::Info,
            "SDMMC input_hz={} identification_hz={} card_hz={} VERID={:#x} HCON={:#x} FIFO={:#x}/{} depth={}",
            self.input_frequency.load(Ordering::Acquire),
            identification_frequency,
            card_frequency,
            version,
            hcon,
            fifo_offset,
            fifo_width,
            depth
        );
        self.capacity.store(sectors, Ordering::Release);
        *self.command.lock() = Some(card);
        self.regs.write(RINTSTS, u32::MAX);
        self.regs.write(IDSTS, IDMAC_IRQ);
        self.regs.write(INTMASK, IRQ_MASK);
        Ok(true)
    }

    /// 检查 APP_CMD 已被卡接受，避免将 ACMD 当作普通命令继续执行。
    fn app_polled(&self, card: Card) -> Result<(), BlockError> {
        let response =
            protocol::check_r1(self.command_polled(APP_CMD, card.rca, RESPONSE | CRC)?[0])?;
        if response & R1_APP_CMD == 0 {
            return Err(BlockError::Unsupported);
        }
        Ok(())
    }

    /// 八字节 SCR 按线路大端解释；入口和 item 宽度复用复位后已核的 FIFO 几何。
    fn scr(&self, offset: usize, width: usize) -> Result<u64, BlockError> {
        self.regs.write(BLKSIZ, SCR_BYTES as u32);
        self.regs.write(BYTCNT, SCR_BYTES as u32);
        self.regs.write(RINTSTS, u32::MAX);
        self.regs.write(CMDARG, 0);
        self.regs.write(
            CMD,
            START | HOLD | RESPONSE | CRC | DATA | WAIT_DATA | APP_SEND_SCR,
        );
        let until = deadline(COMMAND_MS);
        let mut bytes = [0u8; SCR_BYTES];
        let mut copied = 0;
        let mut seen = 0;
        loop {
            let status = self.regs.read(RINTSTS);
            seen |= status;
            if status & ERRORS != 0 {
                return Err(BlockError::Io);
            }
            while width
                <= protocol::fifo_readable_bytes(
                    self.regs.read(STATUS),
                    width,
                    bytes.len() - copied,
                )
            {
                let pointer = self.regs.0 + offset;
                // SAFETY: offset 和实际宽度已核窗口与对齐；仅初始化独占控制器访问 FIFO，八字节输出有界。
                let word = unsafe {
                    match width {
                        2 => (pointer as *const u16).read_volatile() as u64,
                        4 => (pointer as *const u32).read_volatile() as u64,
                        8 => (pointer as *const u64).read_volatile(),
                        _ => unreachable!(),
                    }
                };
                bytes[copied..copied + width].copy_from_slice(&word.to_le_bytes()[..width]);
                copied += width;
            }
            self.regs.write(RINTSTS, status);
            if seen & (CMD_DONE | DATA_OVER) == CMD_DONE | DATA_OVER && copied == SCR_BYTES {
                protocol::check_r1(self.regs.read(RESP0))?;
                return Ok(u64::from_be_bytes(bytes));
            }
            if until <= time::now() {
                return Err(BlockError::DeviceFault);
            }
            core::hint::spin_loop();
        }
    }

    /// 以同一绝对期限发出 CMD13，卡 ready/TRAN 与物理 DAT busy 分别检查。
    fn wait_card(
        &self,
        card: Card,
        until: MonotonicInstant,
        interrupt: bool,
    ) -> Result<(), BlockError> {
        loop {
            if until <= time::now() {
                return Err(BlockError::Io);
            }
            let response = if interrupt {
                self.command_irq(SEND_STATUS, card.rca, RESPONSE | CRC, CMD_DONE, until)?
            } else {
                self.command_polled(SEND_STATUS, card.rca, RESPONSE | CRC)?
            };
            let response = protocol::check_r1(response[0])?;
            if protocol::ready(response) && self.regs.read(STATUS) & DATA_BUSY == 0 {
                return Ok(());
            }
            // CMD13 的每次回复来自 IRQ；间隔只限制卡状态命令密度，不探测数据完成。
            timer::sleep(Duration::from_millis(1));
        }
    }

    /// 命令代次在同一短锁内清旧原因、开放 mailbox 并敲门，ISR 不会把旧原因归到新命令。
    fn command_irq(
        &self,
        index: u32,
        argument: u32,
        flags: u32,
        required: u32,
        until: MonotonicInstant,
    ) -> Result<[u32; 4], BlockError> {
        {
            let _guard = self.registers.lock();
            if self.stopping.load(Ordering::Acquire) {
                return Err(BlockError::QueueStopped);
            }
            if self.regs.read(CMD) & START != 0 {
                return Err(BlockError::DeviceFault);
            }
            self.regs.write(RINTSTS, u32::MAX);
            self.regs.write(IDSTS, IDMAC_IRQ);
            self.events.store(0, Ordering::Release);
            self.regs.update(CTRL, 0, INT_ENABLE);
            if flags & DATA != 0 {
                // 清旧原因先于 DMA 启动；提前完成的 IDMAC IRQ 与 CMD_DONE 属于同一代。
                self.dma_live.store(true, Ordering::Release);
                self.regs.write(
                    DBADDR,
                    self.memory.descriptors.region().device.value() as u32,
                );
                self.regs.write(IDINTEN, IDMAC_IRQ);
                self.regs.update(CTRL, 0, USE_IDMAC | DMA_ENABLE);
                self.regs.write(BMOD, BMOD_ENABLE | BMOD_FIXED_BURST);
            }
            self.regs.write(CMDARG, argument);
            self.regs.write(CMD, START | HOLD | flags | index);
            if flags & DATA != 0 {
                self.regs.write(PLDMND, 1);
            }
        }
        let mut observed = 0;
        loop {
            observed |= self.events.swap(0, Ordering::AcqRel);
            if observed & (ERRORS | IDMAC_ERROR) != 0 {
                return Err(BlockError::Io);
            }
            if observed & CMD_DONE != 0 {
                protocol::check_r1(self.regs.read(RESP0))?;
            }
            if observed & required == required {
                return Ok(self.regs.response());
            }
            if until <= time::now() {
                return Err(BlockError::Io);
            }
            if self.stopping.load(Ordering::Acquire) {
                return Err(BlockError::QueueStopped);
            }
            self.wait_event(until);
        }
    }

    /// 在真实 IRQ 源与一次绝对期限同时登记；撤等待后同步取消 timer 才允许栈退出。
    fn wait_event(&self, until: MonotonicInstant) {
        let waiter = Waiter::new();
        let mut node = pin!(WaitNode::new(waiter.waker()));
        let waker = pin!(waiter.waker());
        let timer = pin!(Timer::new(
            crate::cpu::current().id,
            TimerKind::Precise,
            timeout,
            waker.as_ref().get_ref() as *const Waker as usize
        ));
        // SAFETY: self 借用与固定栈节点保活到登记撤销，任何退出都先析构 registration。
        let registration = unsafe { self.waiters.register(node.as_mut()) };
        // SAFETY: timer/Waker 均固定，结束 active wait 后 cancel_sync 确认回调尾部才退栈。
        unsafe { timer.as_ref().arm_borrowed(until) }.expect("SDMMC deadline timer rejected");
        // mailbox 复查覆盖 ISR 先于登记、及登记后先于 park 的两种到达。
        if self.events.load(Ordering::Acquire) == 0
            && time::now() < until
            && !self.stopping.load(Ordering::Acquire)
        {
            waiter.park();
        }
        drop(registration);
        drop(waiter);
        timer.as_ref().cancel_sync();
    }

    /// 数据只走固定 IDMAC 链，卡停止/持久完成与 DMA 静止分别证明后才交还 CPU。
    fn transfer(
        &self,
        card: Card,
        operation: BlkOperation,
        sector: u64,
        bytes: usize,
        transfer: impl FnOnce(*mut u8) -> Result<usize, BlockError>,
    ) -> Result<usize, BlockError> {
        if bytes == 0 || BOUNCE_BYTES < bytes || bytes % SECTOR_BYTES as usize != 0 {
            return Err(BlockError::InvalidRange);
        }
        if self.capacity.load(Ordering::Acquire)
            < sector
                .checked_add((bytes / SECTOR_BYTES as usize) as u64)
                .ok_or(BlockError::InvalidRange)?
        {
            return Err(BlockError::InvalidRange);
        }
        let write = operation == BlkOperation::Write;
        if write && self.readonly.load(Ordering::Acquire) {
            return Err(BlockError::ReadOnly);
        }
        let argument = protocol::address(sector, card.block_addressed)?;
        self.wait_card(card, deadline(PROGRAM_MS), true)?;
        let bounce = self.memory.bounce.region();
        let descriptors = self.memory.descriptors.region();
        let mut callback = Some(transfer);
        if write {
            let copied = callback.take().unwrap()(bounce.cpu.as_ptr())?;
            if copied != bytes {
                return Err(BlockError::ShortIo);
            }
        }
        // SAFETY: SleepMutex 独占唯一硬件通道，上一请求已真实停止 DMA 并收权；本次尚未发表地址。
        let table = unsafe { &mut *descriptors.cpu.as_ptr().cast::<[Descriptor; DESCRIPTORS]>() };
        let used = protocol::descriptors(
            table,
            descriptors.device.value(),
            bounce.device.value(),
            bytes,
        )?;
        // SAFETY: CPU 描述符写入和 Write 回调已结束；两 owner 独占完整页/cacheline，设备尚未获权。
        unsafe {
            self.memory.bounce.sync_for_device(
                0,
                BOUNCE_BYTES,
                if write {
                    DmaDirection::ToDevice
                } else {
                    DmaDirection::FromDevice
                },
            )?;
            self.memory
                .descriptors
                .sync_for_device(0, DESC_BYTES, DmaDirection::Bidirectional)?;
        }
        {
            let _guard = self.registers.lock();
            if self.stopping.load(Ordering::Acquire) {
                return Err(BlockError::QueueStopped);
            }
            self.regs.write(BLKSIZ, SECTOR_BYTES as u32);
            self.regs.write(BYTCNT, bytes as u32);
        }
        let multiple = (SECTOR_BYTES as usize) < bytes;
        let index = match (write, multiple) {
            (false, false) => READ_SINGLE_BLOCK,
            (false, true) => READ_MULTIPLE_BLOCK,
            (true, false) => WRITE_BLOCK,
            (true, true) => WRITE_MULTIPLE_BLOCK,
        };
        let data = self
            .command_irq(
                index,
                argument,
                RESPONSE | CRC | DATA | WAIT_DATA | if write { WRITE } else { 0 },
                CMD_DONE | DATA_OVER | IDMAC_DONE,
                deadline(PROGRAM_MS),
            )
            .and_then(|response| protocol::check_r1(response[0]).map(|_| ()));
        let transferred = (self.regs.read(TBBCNT), self.regs.read(TCBCNT));
        // 无论数据成功与否，多块都尝试 CMD12；错误不触发未知写重放。
        let card_stop = if multiple {
            self.command_irq(
                STOP_TRANSMISSION,
                0,
                RESPONSE | CRC | STOP,
                CMD_DONE,
                deadline(COMMAND_MS),
            )
            .and_then(|response| protocol::check_r1(response[0]).map(|_| ()))
        } else {
            Ok(())
        };
        let stop = self.stop_dma(deadline(RESET_MS));
        data?;
        card_stop?;
        stop?;
        self.wait_card(card, deadline(PROGRAM_MS), true)?;
        if transferred != (bytes as u32, bytes as u32) {
            return Err(BlockError::ShortIo);
        }
        // SAFETY: stop_dma 确认接口和 IDMAC 静止，无设备别名；控制页整体收权后才能读取 OWN。
        unsafe {
            self.memory
                .descriptors
                .sync_for_cpu(0, DESC_BYTES, DmaDirection::Bidirectional)?;
        }
        // SAFETY: 控制页已收权，原 owner 保活；只读取本次有界描述符。
        let table = unsafe { &*descriptors.cpu.as_ptr().cast::<[Descriptor; DESCRIPTORS]>() };
        if table[..used]
            .iter()
            .any(|descriptor| descriptor.control & protocol::OWN != 0)
        {
            return Err(BlockError::DeviceFault);
        }
        // SAFETY: stop_dma 已证明设备结束访问，bounce 全区独占，Read 在同步完成前不交给回调。
        unsafe {
            self.memory.bounce.sync_for_cpu(
                0,
                BOUNCE_BYTES,
                if write {
                    DmaDirection::ToDevice
                } else {
                    DmaDirection::FromDevice
                },
            )?;
        }
        if !write {
            let copied = callback.take().unwrap()(bounce.cpu.as_ptr())?;
            if bytes < copied {
                return Err(BlockError::InvalidRange);
            }
            return Ok(copied);
        }
        Ok(bytes)
    }

    /// 停止内部 DMA 并确认真实 reset 与 DMA_REQ 解除，保留卡控制器以执行 CMD13。
    fn stop_dma(&self, until: MonotonicInstant) -> Result<(), BlockError> {
        {
            let _guard = self.registers.lock();
            self.regs.write(IDINTEN, 0);
            self.regs.update(CTRL, USE_IDMAC | DMA_ENABLE, DMA_RESET);
            self.regs.update(BMOD, BMOD_ENABLE, BMOD_RESET);
        }
        loop {
            {
                let _guard = self.registers.lock();
                if self.regs.read(CTRL) & DMA_RESET == 0
                    && self.regs.read(BMOD) & BMOD_RESET == 0
                    && self.regs.read(STATUS) & DMA_REQ == 0
                {
                    self.regs.write(DBADDR, 0);
                    self.dma_live.store(false, Ordering::Release);
                    return Ok(());
                }
            }
            if until <= time::now() {
                return Err(BlockError::StopUnconfirmed);
            }
            timer::sleep(Duration::from_millis(1));
        }
    }
}

impl Drop for Sdmmc {
    /// 只有确认没有活 DMA 才归还原页；未确认时 ManuallyDrop 保留整个控制/传输拥有者。
    fn drop(&mut self) {
        if !self.dma_live.load(Ordering::Acquire) {
            // SAFETY: 尚未发表 DMA 地址或已完成真实停止；最后 Arc 无活动使用者，唯一析构原页一次。
            unsafe { ManuallyDrop::drop(&mut self.memory) };
        }
    }
}

/// 构造绝对期限，不能用循环次数或饱和值替代真实硬件时间。
fn deadline(milliseconds: u64) -> MonotonicInstant {
    time::now()
        .checked_add(Duration::from_millis(milliseconds))
        .expect("SDMMC deadline overflow")
}

/// 精确定时器只通知原 Waiter；栈 Waker 保活到同步取消的真实尾部。
fn timeout(timer: Pin<&Timer>) {
    // SAFETY: wait_event 固定并保活 Waker，cancel_sync 返回前不会释放回调上下文。
    let waker = unsafe { &*(timer.context() as *const Waker) };
    waker.notify();
}

/// SD 不报告无法确认的部分写前缀，首错由共同 Bio 原样保存。
fn failed(error: BlockError) -> IoResult {
    IoResult {
        completed: 0,
        error: Some(error),
    }
}
