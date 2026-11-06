//! Synopsys DesignWare Mobile Storage Host Controller (DW-MMC) 单命令 SD/MMC 块设备控制器驱动。
//!
//! # 模块职责与架构定位
//! 本模块实现基于 32 位内置链式 IDMAC 的 DW-MMC SD 卡控制器后端，向上层通用块设备层（`block::Disk`）
//! 提供单硬件通道（`queue_depth = 1`）的扇区读写与刷盘（Flush）能力，向下通过 `drivers::board_setup`
//! 配置时钟、复位、引脚复用与 GPIO 卡检测，并通过 `mm::dma::CoherentDma` 管理设备独占的 DMA 缓冲区。
//! 许可与来源说明见 `sdmmc/LICENSE`。
//!
//! # 核心设计与关键状态机
//!
//! ## 1. 跳板缓冲区（Bounce Buffer）与 DMA 物理页访问权隔离
//! 上层块请求的 `Bio` 物理页不会直接暴露给 DW-MMC 硬件，控制器固定持有两块永久 DMA 内存（[`Memory`]）：
//! - `bounce`（64 KiB）：与 IDMAC 交换扇区数据的唯一跳板缓冲区。
//! - `descriptors`（4 KiB）：存放最多 16 项 32 位链式 IDMAC 描述符（[`Descriptor`]）的控制页。
//!
//! 单次数据传输的 DMA 访问权流转：
//! ```text
//! [Write 请求]
//!   CPU 执行 transfer 回调将数据拷入 bounce
//!       │
//!       ▼
//!   CPU 填写 descriptors 表 (OWN = 1)
//!       │
//!       ▼
//!   sync_for_device(bounce + descriptors) ──► 移交物理页的 DMA 访问权给设备
//!       │
//!       ▼
//!   启动 CMD24/CMD25 + IDMAC 传输，工作任务睡眠等待 IRQ (CMD_DONE | DATA_OVER | IDMAC_DONE)
//!       │
//!       ▼
//!   (多块时发送 CMD12) ──► stop_dma() 确认 DMA_RESET / BMOD_RESET / DMA_REQ 全部清零
//!       │
//!       ▼
//!   wait_card(CMD13) 确认卡回到 TRAN 状态且 DAT0 释放
//!       │
//!       ▼
//!   sync_for_cpu(descriptors + bounce) ──► 收回物理页的 DMA 访问权归还 CPU，校验 OWN == 0
//!
//! [Read 请求]
//!   CPU 填写 descriptors 表 ──► sync_for_device 移交设备 ──► CMD17/CMD18 + IDMAC ──►
//!   (多块 CMD12) ──► stop_dma() ──► wait_card(CMD13) ──► sync_for_cpu 收回访问权 ──►
//!   CPU 执行 transfer 回调从 bounce 拷出数据
//! ```
//!
//! ## 2. 控制器复位与安全停止状态机（`reset_stage`）
//! 当初始化失败或运行期 I/O 发生错误时，必须先确认硬件与 IDMAC 彻底停止，才能安全回收或复用 DMA 内存：
//! ```text
//! RESET_RUNNING (0)
//!    │  request_reset(): 置 stopping = true，屏蔽中断，写 CTRL::RESET 与 BMOD::BMOD_RESET
//!    ▼
//! RESET_INTERFACE (1)
//!    │  reset_complete(): 观察到 CTRL::RESET == 0、BMOD::BMOD_RESET == 0 且 STATUS::DMA_REQ == 0
//!    │                    随后写 CTRL::FIFO_RESET 触发二次 FIFO 排空复位
//!    ▼
//! RESET_FIFO (2)
//!    │  reset_complete(): 观察到 CTRL::FIFO_RESET == 0，清零 DBADDR 与中断状态，置 dma_live = false
//!    ▼
//! RESET_STOPPED (3) ──► 硬件与 IDMAC 确认静止，Drop 时允许释放 bounce 与 descriptors 物理页
//! ```
//!
//! # 锁获取顺序（Lock Ordering）与并发契约
//! 本模块包含两把锁与一个等待队列，严格遵守以下层级顺序（外层先获取，内层后获取，严禁反序）：
//! 1. `Sdmmc::command`（`SleepMutex<Option<Card>>`）：可睡眠互斥锁，串行化所有运行期块请求（`execute`），
//!    保证同一时刻最多只有一个命令占用控制器硬件通道与跳板缓冲区。中断上下文（ISR）严禁获取此锁。
//! 2. `Sdmmc::registers`（`SpinLock<()>`）：自旋锁临界区（持锁时间极短、不可睡眠），保护 `CTRL`、`BMOD`、
//!    `RINTSTS`、`IDSTS`、`IDINTEN`、`BLKSIZ`、`BYTCNT` 等共享 MMIO 寄存器与 `events` 邮箱的并发更新。
//!    注意：中断处理函数 `acknowledge` 在释放 `registers` 自旋锁之**后**才调用 `waiters.wake_all()`。

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

/// 控制器独占持有的两块一致性/流式 DMA 物理内存区域。
///
/// 上层 `Bio` 请求携带的物理页不会直接建立本设备的 DMA 映射；所有扇区数据均通过 `bounce`
/// 跳板缓冲区中转，由 CPU 在 DMA 启动前（写）或 DMA 停止并同步回 CPU 后（读）通过回调复制。
struct Memory {
    /// 64 KiB 跳板数据缓冲区（Bounce Buffer），用于暂存单次读写请求的扇区负载。
    bounce: CoherentDma,
    /// 4 KiB 描述符控制页，存放最多 16 项 32 位链式 IDMAC 描述符表。
    descriptors: CoherentDma,
}

/// 已识别 SD 卡的协议寻址状态。
///
/// 在 `initialize` 结尾写入 `Sdmmc::command` 睡眠互斥锁，运行期由持有该睡眠锁的请求执行任务读取。
#[derive(Clone, Copy)]
struct Card {
    /// 卡在 `CMD3`（`SEND_RELATIVE_ADDR`）阶段分配并左移到高 16 位（bits `[31:16]`）的相对卡地址（RCA）。
    rca: u32,
    /// 寻址模式标志：`true` 表示 SDHC/SDXC 512 字节块寻址，`false` 表示 SDSC 字节寻址。
    block_addressed: bool,
}

/// DW-MMC SD/MMC 控制器实例。
///
/// 上层通用 `block::Disk` 管理请求队列与 8 个软件完成槽位；本结构体负责维护单通道硬件寄存器、
/// 固定 DMA 缓冲区、中断等待队列及控制器复位状态机。
pub(crate) struct Sdmmc {
    /// 控制器永久 MMIO 寄存器窗口访问入口。
    regs: Registers,
    /// 启动期解析的板级控制器资源描述（MMIO 区间、时钟/复位/引脚配置、总线位宽、卡检测等）。
    resources: MmcResources,
    /// 映射后的中断控制器 IRQ 编号。
    irq: IrqNumber,
    /// 时钟源配置完成后测得的控制器输入参考时钟频率（单位：Hz）。
    input_frequency: AtomicU64,
    /// 跳板缓冲区与 IDMAC 描述符表内存，使用 `ManuallyDrop` 包装以便仅在确认无活动 DMA（`!dma_live`）时释放物理页。
    memory: ManuallyDrop<Memory>,
    /// 串行化运行期块请求与保护 `Card` 协议状态的睡眠互斥锁。
    command: SleepMutex<Option<Card>>,
    /// 保护控制寄存器更新、W1C 中断清除与复位状态切换的自旋锁（临界区极短、不可睡眠）。
    registers: SpinLock<()>,
    /// 中断处理函数向等待任务投递的已发生硬件与 IDMAC 事件位掩码（包含 `RINTSTS` 位与 `IDMAC_DONE`/`IDMAC_ERROR`）。
    events: AtomicU32,
    /// 等待当前命令或 DMA 中断完成的任务睡眠等待队列。
    waiters: WaitQueue,
    /// 识别出的 SD 卡总容量（单位：512 字节扇区数）。
    capacity: AtomicU64,
    /// 只读标志：`true` 表示硬件写保护引脚有效或 CSD 永久/临时写保护位置位。
    readonly: AtomicBool,
    /// 发布标志：`true` 表示中断处理已注册且已通过 [`Sdmmc::publish`] 开放控制器中断输出。
    published: AtomicBool,
    /// 停止标志：`true` 表示已触发 [`Sdmmc::request_reset`]，拒绝新命令并保持总中断关闭。
    stopping: AtomicBool,
    /// 复位状态机当前阶段（取值：`RESET_RUNNING`、`RESET_INTERFACE`、`RESET_FIFO`、`RESET_STOPPED`）。
    reset_stage: AtomicU32,
    /// IDMAC 活动标志：向硬件写入 `DBADDR` 启动 DMA 前置为 `true`，确认 DMA 复位并清零 `DBADDR` 后置为 `false`。
    dma_live: AtomicBool,
}

impl Sdmmc {
    /// 在普通可睡眠任务上下文中分配控制器与 DMA 缓冲区，执行板级时钟/复位/引脚配置并初始化 SD 卡。
    ///
    /// 初始化成功时控制器总中断（`CTRL::INT_ENABLE`）处于关闭状态，待上层注册好 IRQ 处理函数后再调用
    /// [`Sdmmc::publish`] 开放中断。
    /// - 返回 `Ok(Some(controller))`：检测到 SD 卡并完成识别与初始化。
    /// - 返回 `Ok(None)`：仅当 GPIO 或原生 `CDETECT` 引脚明确指示卡槽无卡时返回。
    /// - 返回 `Err(BlockError)`：资源校验失败、内存分配失败、卡识别协议错误或复位未确认。
    ///
    /// # Safety
    /// 1. `resources` 必须来自唯一的启动平台快照，其引用的控制器 MMIO、syscon、clock/reset/pinctrl 以及
    ///    可选的独立卡检测 GPIO 控制器均已建立完整的永久内核 MMIO 映射。
    /// 2. 本控制器实例及所选卡检测 GPIO 引脚由调用方独占，涉及共享 CRG/syscon 的配置由 `board_setup` 内部锁串行化。
    /// 3. 调用方必须在调用 [`Sdmmc::publish`] 之前注册并保活指向本控制器的 IRQ 处理回调，且在卸载或错误停止时遵循块设备停止协议。
    pub(crate) unsafe fn prepare(
        resources: &MmcResources,
    ) -> Result<Option<Arc<Self>>, BlockError> {
        // Step 1: 校验 MMIO 映射基址对齐、窗口大小、总线宽度、FIFO 深度范围、最大频率与时钟项数。
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

        // Step 2: 映射中断号，并在 32 位设备地址空间内分配 64 KiB 跳板缓冲区与 4 KiB IDMAC 描述符页。
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

        // Step 3: 应用板级时钟、复位与引脚复用配置，并读取控制器输入参考时钟频率。
        // SAFETY: 调用方契约保证 `resources.setup` 涉及的时钟、复位与引脚复用控制器均已建立有效永久映射。
        unsafe { board_setup::apply_setup(&resources.setup, RESET_HOLD) }
            .map_err(|_| BlockError::Unsupported)?;
        // SAFETY: `apply_setup` 已成功使能时钟，且时钟控制器的永久 MMIO 映射由 `prepare` 调用方契约保证有效。
        let frequency = unsafe { board_setup::clock_rate(&resources.setup.clocks.entries[1]) }
            .map_err(|_| BlockError::Unsupported)?;
        controller
            .input_frequency
            .store(frequency, Ordering::Release);

        // Step 4: 执行卡在位检测与协议初始化；若缺卡或初始化出错，先执行控制器复位确认硬件静止后再返回。
        match controller.initialize() {
            Ok(true) => Ok(Some(controller)),
            outcome => {
                controller.request_reset();
                if !controller.wait_reset(deadline(RESET_MS)) {
                    // 若控制器在超时内未能确认复位完成，使用 `Arc::into_raw` 刻意泄漏强引用，
                    // 防止释放仍可能被异常硬件访问的资源，随后触发 panic。
                    let _retained = Arc::into_raw(controller);
                    panic!("SDMMC initialization stop is unconfirmed");
                }
                outcome.map(|_| None)
            }
        }
    }

    /// 返回块设备队列限制参数（包含识别出的卡扇区容量、64 KiB 单次传输上限、单硬件命令深度及 8 个软件标签槽）。
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

    /// 返回本控制器在中断子系统中映射的 [`IrqNumber`]。
    pub(crate) fn interrupt(&self) -> IrqNumber {
        self.irq
    }

    /// 在上层已完成 IRQ 处理函数注册后开放控制器总中断输出，使控制器进入可接收块请求的发布状态。
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

    /// 中断处理函数（ISR）入口：读取并写 1 清零（W1C）本控制器使能的中断状态，将事件合并写入 `events` 并在锁外唤醒等待队列。
    ///
    /// 若本次中断属于本设备（控制器或 IDMAC 存在未屏蔽挂起中断）则返回 `true`，否则返回 `false`。
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

    /// 开启或关闭控制器总中断使能位（`CTRL::INT_ENABLE`）。
    ///
    /// 可在自旋锁临界区或中断上下文中调用；一旦控制器进入 `stopping` 状态，将始终保持中断关闭。
    pub(crate) fn set_interrupts(&self, enabled: bool) {
        let _guard = self.registers.lock();
        let enabled = enabled && !self.stopping.load(Ordering::Acquire);
        self.regs
            .update(CTRL, INT_ENABLE, if enabled { INT_ENABLE } else { 0 });
    }

    /// 重新使能控制器中断并立即检查一次硬件挂起状态与 `events` 邮箱，消除“注册等待节点前中断已到达”的竞态窗口。
    pub(crate) fn arm_and_check(&self) -> bool {
        self.set_interrupts(true);
        self.acknowledge();
        self.events.load(Ordering::Acquire) != 0
    }

    /// 在工作队列可睡眠任务上下文中同步执行一次块设备请求（`Read`、`Write` 或 `Flush`）。
    ///
    /// - 对于 `Write` 请求，在将跳板缓冲区移交 IDMAC 之前调用一次 `transfer` 回调拷入数据。
    /// - 对于 `Read` 请求，在 IDMAC 停止且将跳板缓冲区同步回 CPU 之后调用一次 `transfer` 回调拷出数据。
    /// - 若命令或数据传输失败，立即触发全控制器复位并等待硬件停止确认，防止残留的后台 DMA 踩内存。
    ///
    /// # Safety
    /// 1. `transfer` 回调只能对传入的 `bounce` 缓冲区指针读写本次请求的 `bytes` 字节（`bytes <= BOUNCE_BYTES`），
    ///    回调内部不得睡眠、不得阻塞或重入当前控制器。
    /// 2. 调用方必须保证在所有关联的 IRQ 与工作队列任务完全退出前保活 `self`；若返回 `BlockError::StopUnconfirmed`，
    ///    调用方不得释放或复用控制器及其关联槽位。
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
                // 请求失败时不自动重试：立即发起全控制器与 IDMAC 复位，确认无活动 DMA 后再向上层报告错误。
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

    /// 标记控制器进入 `stopping` 状态以拒绝后续命令，关闭所有中断掩码并发出控制器与 IDMAC 软复位请求，最后唤醒所有等待者。
    ///
    /// 本函数仅下发非阻塞 MMIO 复位命令，可在自旋锁临界区或中断上下文中安全调用；实际复位完成由 [`Sdmmc::reset_complete`] 轮询确认。
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

    /// 推进并检查控制器复位状态机（每调用一次最多推进一个有界 MMIO 阶段）。
    ///
    /// 依次确认控制器/IDMAC 复位位清零、`DMA_REQ` 释放以及二次 `FIFO_RESET` 清零后，清零 `DBADDR` 并将
    /// `dma_live` 置为 `false`，返回 `true` 表示硬件与 DMA 已彻底停止。
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

    /// 在可睡眠上下文中轮询等待已由 [`Sdmmc::request_reset`] 发起的硬件复位在 `until` 期限前完成。
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

    /// 在初始化阶段使用单调时钟截止期限自旋轮询 `condition`，直到条件满足或超时返回 `Err(BlockError::DeviceFault)`。
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

    /// 在初始化阶段以轮询方式发送无数据阶段的 SD 命令并等待 `CMD_DONE`，返回 4 个 32 位响应寄存器值。
    ///
    /// 若硬件报告 `RESPONSE_TIMEOUT`（如旧版 SDSC 卡不响应 `CMD8`），返回 `Err(BlockError::NotReady)`；
    /// 其他硬件错误位返回 `Err(BlockError::Io)`。
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

    /// 根据目标频率 `target`（Hz）计算分频系数并遵循 DW-MMC 三步时钟更新流程（关时钟 -> 改分频 -> 开时钟）配置卡时钟，返回实际输出的卡时钟频率（Hz）。
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
            input.div_ceil(CLOCK_DIVIDER_FACTOR * u64::from(target))
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

    /// 向 `CMD` 寄存器下发仅更新时钟寄存器而不往 SD 总线发送命令帧的 `UPDATE_CLOCK` 控制动作，并轮询等待 `START` 位自动清零。
    fn update_clock(&self) -> Result<(), BlockError> {
        self.regs
            .write(CMD, START | HOLD | WAIT_DATA | UPDATE_CLOCK);
        self.poll(deadline(COMMAND_MS), || self.regs.read(CMD) & START == 0)?;
        if self.regs.read(RINTSTS) & ERRORS != 0 {
            return Err(BlockError::Io);
        }
        Ok(())
    }

    /// 执行控制器软复位、卡在位检测及完整的 SD 卡识别与传输模式初始化流程。
    ///
    /// - 返回 `Ok(true)`：成功识别 SD 卡并完成容量解析、4-bit 总线切换（若支持）及传输时钟配置。
    /// - 返回 `Ok(false)`：卡检测引脚明确指示卡槽内无卡。
    /// - 返回 `Err(BlockError)`：硬件能力不支持或卡初始化协议交互失败。
    fn initialize(&self) -> Result<bool, BlockError> {
        // Step 1: 复位控制器与 IDMAC，校验 FIFO 几何参数并确认硬件为 32 位内部 DMA 模式。
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
        // 本驱动仅支持 32 位地址格式的内部 IDMAC；64 位 IDMAC 的描述符与寄存器布局不同。
        if hcon & HCON_ADDRESS_64BIT != 0
            || (hcon >> HCON_DMA_INTERFACE_SHIFT) & HCON_DMA_INTERFACE_MASK != 0
        {
            return Err(BlockError::Unsupported);
        }

        // Step 2: 通过板级 GPIO 或控制器原生 CDETECT 寄存器检测卡是否在位。
        let present = if let Some(input) = self.resources.card_detect.as_ref() {
            // SAFETY:
            // 1. `self` 仅由 `Sdmmc::prepare` 构造，其调用方保证独立 GPIO 与系统 CRG 映射永久有效且该引脚由本驱动独占。
            // 2. `board_setup::read_gpio_input` 内部使用全局睡眠互斥锁串行化共享寄存器配置。
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

        // Step 3: 开启卡电源、初始化超时与 FIFO 水线寄存器，并将卡时钟设为 400 kHz 识别频率。
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
        // Step 4: 发送 CMD0 复位卡状态，发送 CMD8 区分 SD 2.0+ 与 1.x，并循环发送 ACMD41 等待卡上电就绪。
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

        // Step 5: 依次发送 CMD2 获取 CID、CMD3 分配相对地址 RCA、CMD9 读取 CSD 计算容量，再发送 CMD7 选中卡。
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

        // Step 6: 通过 PIO 读取 8 字节 SCR 寄存器，若主机与卡均支持 4-bit 模式则发送 ACMD6 切换至 4-bit 总线。
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

        // Step 7: 切换到数据传输时钟频率，检测写保护状态，保存卡容量与协议状态并配置中断屏蔽掩码。
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

    /// 在初始化轮询阶段发送 `CMD55`（`APP_CMD`）并校验响应中的 `R1_APP_CMD` 位，确认卡已准备好接收下一条应用特定命令（ACMD）。
    fn app_polled(&self, card: Card) -> Result<(), BlockError> {
        let response =
            protocol::check_r1(self.command_polled(APP_CMD, card.rca, RESPONSE | CRC)?[0])?;
        if response & R1_APP_CMD == 0 {
            return Err(BlockError::Unsupported);
        }
        Ok(())
    }

    /// 在初始化阶段发送 `ACMD51`（`APP_SEND_SCR`），通过 FIFO 数据窗口以 PIO 方式读取 8 字节大端序 `SCR` 寄存器。
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
                // SAFETY:
                // 1. `offset` 与 `width`（2/4/8 字节）已由 `protocol::fifo_geometry` 校验对齐且完全位于 MMIO 窗口内。
                // 2. 初始化阶段控制器尚未发布，当前任务独占访问 FIFO 数据寄存器。
                // 3. 外层 `while width <= fifo_readable_bytes(..., bytes.len() - copied)` 保证切片写入不越界。
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

    /// 在截止期限 `until` 内循环发送 `CMD13`（`SEND_STATUS`），等待卡状态机回到 `TRAN` 且 `READY_FOR_DATA` 置位，同时确认物理 `DAT` 线不再忙碌。
    ///
    /// `interrupt == false` 时使用轮询命令路径（初始化阶段），`interrupt == true` 时使用中断睡眠等待路径（运行期）。
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
            // 若卡仍在编程或忙碌，睡眠 1ms 后再次发送 CMD13 查询，避免密集占用总线。
            timer::sleep(Duration::from_millis(1));
        }
    }

    /// 在运行期通过中断驱动方式下发一条命令（可选携带 IDMAC 数据传输），并在可睡眠上下文中等待 `required` 指定的事件位全部到达。
    ///
    /// 在 `registers` 自旋锁临界区内完成“清除残留中断 -> 清零 `events` 邮箱 -> 启动 IDMAC（若有数据）-> 写 `CMD` 触发硬件”，
    /// 确保中断处理函数不会将上一条命令的残留状态误认为新命令的完成事件。
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
                // 先清除旧中断状态再使能 IDMAC，确保快速完成的 DMA 中断与 CMD_DONE 归属同一次命令执行。
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

    /// 在等待队列与高精度超时定时器上同时注册当前任务并进入睡眠，直到中断唤醒、超时到期或控制器进入停止状态。
    ///
    /// 退出函数前先移除等待队列节点，再调用 `cancel_sync` 同步等待定时器回调彻底执行完毕，确保栈上固定的 `Waker` 不会悬垂。
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
        // SAFETY: `self.waiters` 属于当前借用的控制器，`node` 已固定在当前栈帧上，且在函数返回前必定通过 `drop(registration)` 解绑。
        let registration = unsafe { self.waiters.register(node.as_mut()) };
        // SAFETY: `timer` 与 `waker` 均固定在当前栈帧上，且函数返回前调用 `timer.as_ref().cancel_sync()` 等待正在执行的超时回调退出。
        unsafe { timer.as_ref().arm_borrowed(until) }.expect("SDMMC deadline timer rejected");

        // 在真正阻塞前复查 `events`、截止时间与 `stopping`，覆盖注册前中断已到达或刚注册即触发的窗口。
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

    /// 执行一次扇区数据读写传输（通过 64 KiB 跳板缓冲区与 16 项链式 IDMAC 描述符表）。
    ///
    /// 严格按照“写前拷入 -> 填写描述符 -> 同步给设备 -> 中断等待传输完成 -> 多块发 CMD12 -> 停止 DMA -> 等待卡回到 TRAN ->
    /// 同步回 CPU 并校验 OWN 位 -> 读后拷出”的顺序执行，返回实际完成的字节数。
    fn transfer(
        &self,
        card: Card,
        operation: BlkOperation,
        sector: u64,
        bytes: usize,
        transfer: impl FnOnce(*mut u8) -> Result<usize, BlockError>,
    ) -> Result<usize, BlockError> {
        // Step 1: 校验字节长度、扇区对齐、卡容量边界以及只读状态，并等待卡处于可接收命令的 TRAN 状态。
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
        // Step 2: 若为写操作，先调用 CPU 回调将待写入数据复制到跳板缓冲区，随后构建 IDMAC 描述符链并同步缓存给设备。
        let bounce = self.memory.bounce.region();
        let descriptors = self.memory.descriptors.region();
        let mut callback = Some(transfer);
        if write {
            let copied = callback.take().unwrap()(bounce.cpu.as_ptr())?;
            if copied != bytes {
                return Err(BlockError::ShortIo);
            }
        }

        // SAFETY:
        // 1. 调用方 `execute` 已持有 `self.command` 睡眠互斥锁，独占控制器通道与 `descriptors` 控制页。
        // 2. 上一次传输已通过 `stop_dma` 确认硬件停止并收回访问权，本次传输尚未向硬件写入 `DBADDR`。
        // 3. `descriptors` 分配大小为 `PAGE_SIZE`（4 KiB），足以容纳 16 项 16 字节对齐的 `Descriptor`。
        let table = unsafe { &mut *descriptors.cpu.as_ptr().cast::<[Descriptor; DESCRIPTORS]>() };
        let used = protocol::descriptors(
            table,
            descriptors.device.value(),
            bounce.device.value(),
            bytes,
        )?;

        // SAFETY:
        // 1. CPU 对 `descriptors` 和（写请求时）`bounce` 的写入已经全部完成，且当前不存在指向这两块内存的活跃 Rust 引用。
        // 2. 两块区域均由 `CoherentDma::allocate` 独立分配并对齐到缓存行，此时硬件 IDMAC 尚未启动，无并发访问冲突。
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

        // Step 3: 配置块大小与总字节数，选择单块或多块读写命令并通过中断等待命令、数据与 IDMAC 全部完成。
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

        // Step 4: 多块传输无论成功或失败均发送 CMD12 终止传输，随后停止 IDMAC、等待卡编程结束并核对硬件字节计数。
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

        // Step 5: 收回描述符页与跳板缓冲区的 CPU 访问权，校验所有已用描述符的 OWN 位均已被硬件清零；若为读操作则调用回调拷出数据。
        // SAFETY:
        // 1. `stop_dma` 已确认 IDMAC 复位完成且 `DMA_REQ` 为 0，设备不再访问描述符页。
        // 2. 当前任务持有 `self.command` 睡眠锁且尚未创建指向描述符页的 CPU 引用。
        unsafe {
            self.memory
                .descriptors
                .sync_for_cpu(0, DESC_BYTES, DmaDirection::Bidirectional)?;
        }
        // SAFETY: `sync_for_cpu` 已收回描述符页的 CPU 访问权，且 `used <= DESCRIPTORS` 由 `protocol::descriptors` 保证。
        let table = unsafe { &*descriptors.cpu.as_ptr().cast::<[Descriptor; DESCRIPTORS]>() };
        if table[..used]
            .iter()
            .any(|descriptor| descriptor.control & protocol::OWN != 0)
        {
            return Err(BlockError::DeviceFault);
        }

        // SAFETY:
        // 1. `stop_dma` 已确认设备完全停止访问 `bounce` 跳板缓冲区。
        // 2. 读请求在完成本次 `sync_for_cpu` 缓存失效/同步之后才把指针传给 CPU `callback` 读取。
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

    /// 停止并复位内部 IDMAC，轮询等待 `DMA_RESET`、`BMOD_RESET` 与 `DMA_REQ` 全部清零后清空 `DBADDR`，保留卡控制器状态以便后续发送 `CMD13`。
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
    /// 析构控制器：仅当 `dma_live` 为 `false`（从未启动 DMA 或已确认 DMA 硬件彻底停止）时才释放 `bounce` 与 `descriptors` 物理页。
    ///
    /// 若硬件停止未获确认（`dma_live == true`），`ManuallyDrop` 将刻意保留这两块 DMA 内存不归还页分配器，防止失控硬件破坏已复用的物理内存。
    fn drop(&mut self) {
        if !self.dma_live.load(Ordering::Acquire) {
            // SAFETY:
            // 1. `dma_live == false` 证明硬件从未获取或已通过 `stop_dma`/`reset_complete` 彻底释放对 DMA 页的访问。
            // 2. `&mut self` 保证最后一个 `Arc<Sdmmc>` 强引用正在析构，无其他并发访问者，且 `ManuallyDrop::drop` 仅在此处调用一次。
            unsafe { ManuallyDrop::drop(&mut self.memory) };
        }
    }
}

/// 根据相对毫秒数 `milliseconds` 计算基于当前单调时钟的绝对截止时间 [`MonotonicInstant`]。
fn deadline(milliseconds: u64) -> MonotonicInstant {
    time::now()
        .checked_add(Duration::from_millis(milliseconds))
        .expect("SDMMC deadline overflow")
}

/// 高精度超时定时器回调：唤醒在 [`Sdmmc::wait_event`] 中睡眠等待的关联 [`Waker`]。
fn timeout(timer: Pin<&Timer>) {
    // SAFETY: `wait_event` 将 `Waker` 固定在栈帧上并将其地址作为 `timer.context()` 传入；
    // `wait_event` 在退出前必定调用 `timer.as_ref().cancel_sync()` 等待本回调彻底执行完毕，因此指针在回调期间始终有效。
    let waker = unsafe { &*(timer.context() as *const Waker) };
    waker.notify();
}

/// 构造已完成字节数为 0 且携带指定错误码 `error` 的失败 [`IoResult`]。
fn failed(error: BlockError) -> IoResult {
    IoResult {
        completed: 0,
        error: Some(error),
    }
}
