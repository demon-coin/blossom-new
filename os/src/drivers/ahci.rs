//! AHCI（Advanced Host Controller Interface）单端口、非 NCQ SATA 块设备驱动。
//!
//! # 模块职责与架构定位
//! 本模块（`drivers::ahci`）是 SATA 磁盘控制器的底层硬件驱动，负责：
//! - **上游调用者**：通用块设备层（`crate::block`）通过可睡眠的 `progress` 工作队列任务（Work）
//!   调用 [`Ahci::execute`] 提交直接 DMA 读写或刷新请求，并在中断顶半部调用 [`Ahci::acknowledge`]
//!   应答硬件中断、在错误恢复/卸载路径调用 [`Ahci::request_reset`] 与 [`Ahci::reset_complete`]。
//! - **下游依赖**：
//!   - 子模块 [`ata`]：负责 `IDENTIFY DEVICE` 参数解析与 20 字节 Register H2D FIS 命令编码；
//!   - 子模块 [`registers`]：负责 HBA 全局与端口 MMIO 强序寄存器访问及链路 COMRESET 序列；
//!   - DMA 子系统（`crate::mm::dma`）：管理控制结构的一致性/同步 DMA 内存与请求负载物理段映射；
//!   - 同步与等待基础设施（`SleepMutex`、`SpinLock`、`WaitQueue`、`Timer`）：实现中断驱动的睡眠等待与超时控制。
//!
//! # 核心设计与关键状态机 / 内存布局
//!
//! ## 1. DMA 控制页布局（[`Controls`]）
//! 为避免在非缓存一致性（Non-coherent DMA）平台上发生 CPU 与设备共享同一条 Cache Line 导致的写回覆盖（Cache Line 污染），
//! 驱动为四类控制结构分别分配独立的整页（Page-aligned）[`CoherentDma`] 缓冲区，且仅使用命令槽 0（Slot 0）：
//!
//! ```text
//! +-----------------------+--------------------------------------------------------+
//! | CoherentDma 独立页    | 内部结构与硬件对齐要求                                 |
//! +-----------------------+--------------------------------------------------------+
//! | list (1 KiB, 1K 对齐) | 32 个 Command Header（每个 32B），仅使用 Slot 0：      |
//! |                       |   DW0: [31:16] PRDTL | [6] W (Write) | [4:0] CFL (5)   |
//! |                       |   DW1: [31:0]  PRDBC (硬件更新的已传输字节计数)        |
//! |                       |   DW2: [31:0]  CTBA  (Command Table 物理地址低 32 位)  |
//! |                       |   DW3: [31:0]  CTBAU (Command Table 物理地址高 32 位)  |
//! +-----------------------+--------------------------------------------------------+
//! | fis (256B, 256B 对齐) | Received FIS 接收区（设备回传 D2H / PIO Setup FIS 等） |
//! +-----------------------+--------------------------------------------------------+
//! | table (128B 对齐)     | Command Table：                                        |
//! |                       |   +0x000..0x080 (128B): CFIS (前 20B 为 H2D FIS) 等    |
//! |                       |   +0x080..0x400 (56 * 16B): PRDT 物理段描述符表：      |
//! |                       |     DW0: DBA  (数据段物理地址低 32 位，偶地址对齐)     |
//! |                       |     DW1: DBAU (数据段物理地址高 32 位)                 |
//! |                       |     DW2: Reserved (0)                                  |
//! |                       |     DW3: [21:0] DBC (字节数减 1，偶数长度，最大 4 MiB) |
//! +-----------------------+--------------------------------------------------------+
//! | identify (512B, 2B齐) | 探测阶段接收 256 Words `IDENTIFY DEVICE` 数据的缓冲区  |
//! +-----------------------+--------------------------------------------------------+
//! ```
//!
//! ## 2. 端口 DMA 引擎启停状态机（[`StopPhase`]）
//! AHCI 硬件要求命令引擎（`ST`/`CR`）与 FIS 接收引擎（`FRE`/`FR`）严格按序停止。
//! 只有在两个引擎彻底停稳（`CR == 0` 且 `FR == 0`）后，硬件才不再通过 DMA 访问控制页或请求数据页：
//!
//! ```text
//!                     issue() 置位 FRE=1, ST=1, CI=1
//!    +--------------------------------------------------------------+
//!    |                                                              v
//! +---------+     reset_complete()      +-----+  begin_stop()   +---------+
//! | Stopped | <------------------------ | Fis | <-------------- | Running |
//! +---------+   确认 FR==0 + DMA 读屏障 +-----+    (若已清 FRE) +---------+
//!                                          ^                        |
//!                                          | reset_complete()       | begin_stop()
//!                                          | 确认 CR==0 后清 FRE=0  | 置 IE=0, 清 ST=0
//!                                       +---------+                 |
//!                                       | Command | <---------------+
//!                                       +---------+
//! ```
//! - 若停止过程超时（返回 [`BlockError::StopUnconfirmed`]），说明硬件 DMA 引擎可能仍在访问物理内存；
//!   此时驱动严禁释放 [`Controls`] 控制页或归还上层请求的数据物理页（Folio/Page），以防止 DMA Use-After-Free 踩内存。
//!
//! # 锁获取顺序（Lock Ordering）与并发契约
//! 1. **`control: SleepMutex<()>`（外层可睡眠互斥锁）** ➔ **`state: SpinLock<State>`（内层自旋锁）**
//!    - 任何同时需要这两把锁的路径（如 [`Ahci::execute`]）必须先获取 `control` 睡眠锁，再短暂获取 `state` 自旋锁。
//!    - **严禁**在持有 `state` 自旋锁期间尝试获取 `control`、调用睡眠函数（如 `Waiter::park`、`timer::sleep`）或调用 `WaitQueue::wake_all`。
//! 2. **DMA 控制页所有权契约**：
//!    - `controls` 中的 DMA 缓冲区仅允许在持有 `control` 睡眠锁（或 `prepare` 单线程初始化阶段）且确认端口处于 `StopPhase::Stopped`（`CR == 0 && FR == 0`）时由 CPU 读写。
//!    - 中断上下文（[`Ahci::acknowledge`]）仅获取 `state` 自旋锁、读写 MMIO 端口中断寄存器并原子更新 `mailbox`，绝不触碰 `controls` 或请求负载页。

mod ata;
mod registers;

use alloc::sync::Arc;
use core::mem::ManuallyDrop;
use core::pin::{pin, Pin};
use core::ptr;
use core::sync::atomic::{AtomicU32, Ordering};

use crate::arch;
use crate::block::{BlkOperation, BlockError, IoResult, QueueLimits};
use crate::boot::AhciResources;
use crate::irq::{self, IrqNumber};
use crate::mm::dma::{CoherentDma, DmaDirection, DmaDomain, PayloadRange};
use crate::sync::{SleepMutex, SpinLock};
use crate::time::{self, Duration, MonotonicInstant};
use crate::timer::{self, Timer, TimerKind};
use crate::wait::{WaitNode, WaitQueue, Waiter, Waker};
use registers::{Host, Port};
use registers::{
    CI, CLB, CLBU, CMD, COMPLETION, CR, ERROR, FB, FBU, FR, FRE, IE, INTERRUPTS, PORT_IS, SACT, ST,
    TASK_BUSY, TASK_ERROR, TFD,
};

/// 单条 ATA 命令从下发门铃到等待中断完成的最大超时时间（5 秒）。
const COMMAND_TIMEOUT: Duration = Duration::from_secs(5);
/// 请求停止端口 DMA 引擎后等待 `CR == 0` 与 `FR == 0` 的最大超时时间（2 秒）。
const STOP_TIMEOUT: Duration = Duration::from_secs(2);

/// 完整 AHCI ABAR MMIO 窗口字节大小（包含全局控制区与规范定义的全部 32 个端口窗口）。
pub(crate) const MMIO_BYTES: usize =
    registers::PORT_BASE + registers::MAX_PORTS as usize * registers::PORT_BYTES;
/// AHCI 规范定义每个端口命令列表包含的命令槽（Command Slot）总数（32 个）。
const COMMAND_SLOTS: usize = 32;
/// 单个命令头（Command Header）的字节大小（32 字节 / 8 个 DWORD）。
const COMMAND_HEADER_BYTES: usize = 32;
/// 整个端口命令列表（Command List）的字节大小（`32 * 32 = 1024` 字节）。
const COMMAND_LIST_BYTES: usize = COMMAND_SLOTS * COMMAND_HEADER_BYTES;
/// Received FIS 接收缓冲区的字节大小（256 字节）。
const RECEIVED_FIS_BYTES: usize = 256;
/// 命令表（Command Table）头部固定区域（含 CFIS、ACMD 与保留区）的字节大小（128 字节）。
const COMMAND_TABLE_HEADER_BYTES: usize = 128;
/// 单个物理区域描述符表项（PRDT Entry）的字节大小（16 字节 / 4 个 DWORD）。
const PRDT_ENTRY_BYTES: usize = 16;
/// Command Header DW0 中 `W`（Write，方向标志）位的偏移量（位 6，置 1 表示主机写往设备）。
const COMMAND_WRITE_SHIFT: u32 = 6;
/// Command Header DW0 中 `PRDTL`（PRDT 表项数量）字段的位偏移量（位 31:16）。
const PRDT_LENGTH_SHIFT: u32 = 16;
/// Command Header 中 `PRDBC`（Physical Region Descriptor Byte Count，DW1）相对于命令头起始的字节偏移（4 字节）。
const PRDBC_OFFSET: usize = core::mem::size_of::<u32>();
/// 本驱动固定使用命令槽 0（Slot 0），对应 `PxCI` 寄存器中的位掩码 `1 << 0`。
const ACTIVE_COMMAND: u32 = 1;
/// 平台非一致性 DMA 缓存同步所需的 Cache Line 对齐粒度（64 字节）。
const DMA_CACHE_LINE_BYTES: usize = 64;

/// 端口 DMA 引擎（命令列表引擎 `ST`/`CR` 与 FIS 接收引擎 `FRE`/`FR`）的运行与停止阶段状态机。
#[derive(Clone, Copy, Eq, PartialEq)]
enum StopPhase {
    /// 命令列表引擎与 FIS 接收引擎均已确认停止（`CR == 0` 且 `FR == 0`），CPU 可安全访问 DMA 控制页。
    Stopped,
    /// 端口 DMA 引擎正在运行（`FRE == 1` 且 `ST == 1`），控制页与负载页归硬件 DMA 引擎访问。
    Running,
    /// 已清除 `CMD.ST`，正在等待硬件将 `CMD.CR`（Command List Running）清零。
    Command,
    /// 已确认 `CMD.CR == 0` 并清除 `CMD.FRE`，正在等待硬件将 `CMD.FR`（FIS Receive Running）清零。
    Fis,
}

/// 受 `Ahci::state` 自旋锁保护的端口运行期控制状态。
struct State {
    /// 端口 DMA 引擎当前所处的启停阶段。
    phase: StopPhase,
    /// 控制器是否已通过 [`Ahci::publish`] 向块设备层正式发布并开启 HBA 全局中断。
    published: bool,
    /// 端口当前是否允许接收新的 I/O 命令（在未发布或触发重置后为 `false`）。
    accepting: bool,
    /// 上层块核心是否请求开启本端口的硬件中断（对应 `PxIE` 掩码状态）。
    interrupts: bool,
    /// 当前运行周期内是否已向槽 0 写入 `PxCI` 下发过命令。
    issued: bool,
}

/// 端口专用的四块独立整页 DMA 控制缓冲区。
///
/// 每个字段均由独立的整页 [`CoherentDma`] 承载，以保证在非缓存一致性架构上不会因共享 Cache Line
/// 而发生 CPU 脏缓存行写回覆盖设备 DMA 写入的数据。
struct Controls {
    /// 命令列表（Command List，1024 字节，硬件要求 1024 字节对齐）。
    list: CoherentDma,
    /// 接收 FIS 缓冲区（Received FIS，256 字节，硬件要求 256 字节对齐）。
    fis: CoherentDma,
    /// 命令表（Command Table，含 128B 头部与 56 项 PRDT，硬件要求 128 字节对齐）。
    table: CoherentDma,
    /// `IDENTIFY DEVICE` 命令专用的 512 字节数据接收缓冲区（要求 2 字节偶地址对齐）。
    identify: CoherentDma,
}

impl Controls {
    /// 在向硬件寄存器写入任何 DMA 地址之前，分配全部四块控制缓冲区并校验其硬件地址对齐要求。
    fn allocate(domain: DmaDomain) -> Result<Self, BlockError> {
        let controls = Self {
            list: CoherentDma::allocate(domain, COMMAND_LIST_BYTES)?,
            fis: CoherentDma::allocate(domain, RECEIVED_FIS_BYTES)?,
            table: CoherentDma::allocate(
                domain,
                COMMAND_TABLE_HEADER_BYTES + ata::MAX_PRDT * PRDT_ENTRY_BYTES,
            )?,
            identify: CoherentDma::allocate(domain, ata::IDENTIFY_BYTES)?,
        };
        for (owner, alignment) in [
            (&controls.list, COMMAND_LIST_BYTES as u64),
            (&controls.fis, RECEIVED_FIS_BYTES as u64),
            (&controls.table, COMMAND_TABLE_HEADER_BYTES as u64),
            (&controls.identify, core::mem::align_of::<u16>() as u64),
        ] {
            if owner.region().device.value() % alignment != 0 {
                return Err(BlockError::Misaligned);
            }
        }
        Ok(controls)
    }

    /// 向命令表（Command Table）的第 `index` 个 PRDT 表项写入一段物理内存区域的 DMA 地址与字节长度。
    ///
    /// 仅在探测阶段单线程独占或运行期持有 `control` 睡眠锁、且端口 DMA 引擎已停止时调用。
    fn prdt(&self, index: usize, address: u64, bytes: usize) -> Result<(), BlockError> {
        if ata::MAX_PRDT <= index || bytes == 0 || ata::PRDT_BYTES < bytes {
            return Err(BlockError::InvalidRange);
        }
        if address & 1 != 0 || bytes & 1 != 0 {
            return Err(BlockError::Misaligned);
        }
        let words = [
            address as u32,
            (address >> 32) as u32,
            0,
            (bytes - 1) as u32,
        ];
        // SAFETY:
        // 1. 调用方处于 `prepare` 独占阶段或持有 `self.control` 睡眠锁，且端口 `CR`/`FR` 均已停止，设备不会并发访问 `table`；
        // 2. 上方边界检查已保证 `index < ata::MAX_PRDT`，因此 `COMMAND_TABLE_HEADER_BYTES + index * PRDT_ENTRY_BYTES`
        //    加上 16 字节仍在 `self.table` 分配的容量范围内，且起始地址满足 `u32` 对齐。
        unsafe {
            let destination = self
                .table
                .region()
                .cpu
                .as_ptr()
                .add(COMMAND_TABLE_HEADER_BYTES + index * PRDT_ENTRY_BYTES)
                .cast::<u32>();
            for (offset, word) in words.iter().enumerate() {
                ptr::write_volatile(destination.add(offset), word.to_le());
            }
        }
        Ok(())
    }

    /// 在端口停止且 CPU 拥有控制页访问权时，填充 CFIS 命令帧、清空接收 FIS 区、写入槽 0 命令头并执行下发前的 DMA 缓存同步。
    fn command(
        &self,
        fis: &[u8; ata::H2D_FIS_BYTES],
        write: bool,
        entries: usize,
    ) -> Result<(), BlockError> {
        if ata::MAX_PRDT < entries {
            return Err(BlockError::InvalidRange);
        }
        let table = self.table.region().device.value();
        let header = [
            (ata::H2D_FIS_BYTES / core::mem::size_of::<u32>()) as u32
                | (u32::from(write) << COMMAND_WRITE_SHIFT)
                | ((entries as u32) << PRDT_LENGTH_SHIFT),
            0,
            table as u32,
            (table >> 32) as u32,
        ];
        // SAFETY:
        // 1. 调用方处于 `prepare` 独占阶段或 `execute` 持有 `self.control` 睡眠锁阶段，且端口 DMA 引擎已确认停止；
        // 2. `self.table`（>= 128B）、`self.fis`（256B）与 `self.list`（1024B，满足 u32 对齐）均由 `self` 保活且容量充足；
        // 3. 写入完成后立即调用 `sync_for_device` 将控制页的访问权移交设备，且不保留任何跨越移交点的 Rust 引用或切片。
        unsafe {
            ptr::copy_nonoverlapping(fis.as_ptr(), self.table.region().cpu.as_ptr(), fis.len());
            ptr::write_bytes(self.fis.region().cpu.as_ptr(), 0, RECEIVED_FIS_BYTES);
            let destination = self.list.region().cpu.as_ptr().cast::<u32>();
            for (offset, word) in header.iter().enumerate() {
                ptr::write_volatile(destination.add(offset), word.to_le());
            }
            self.table
                .sync_for_device(0, self.table.byte_len(), DmaDirection::ToDevice)?;
            self.list
                .sync_for_device(0, self.list.byte_len(), DmaDirection::Bidirectional)?;
            self.fis
                .sync_for_device(0, self.fis.byte_len(), DmaDirection::FromDevice)?;
        }
        Ok(())
    }

    /// 在确认端口 `CR == 0` 且 `FR == 0` 停止后，将命令列表与接收 FIS 缓冲区同步回 CPU，并读取槽 0 的 `PRDBC` 已传输字节计数。
    ///
    /// 注意：当命令发生硬件或任务文件错误时，`PRDBC` 仅反映 DMA 引擎尝试传输的字节数，不能作为已成功写入介质的数据量依据。
    fn transferred(&self) -> Result<usize, BlockError> {
        // SAFETY:
        // 1. 调用方在调用前已通过 `stop_bounded` 确认 `CR == 0` 且 `FR == 0`，设备已停止对 `list` 与 `fis` 的 DMA 写入；
        // 2. `self.list` 与 `self.fis` 缓冲区有效且期间无并发 CPU 访问，`PRDBC_OFFSET`（4 字节）在 `list`（1024 字节）内且满足 `u32` 对齐。
        unsafe {
            self.list
                .sync_for_cpu(0, self.list.byte_len(), DmaDirection::Bidirectional)?;
            self.fis
                .sync_for_cpu(0, self.fis.byte_len(), DmaDirection::FromDevice)?;
            Ok(u32::from_le(ptr::read_volatile(
                self.list
                    .region()
                    .cpu
                    .as_ptr()
                    .add(PRDBC_OFFSET)
                    .cast::<u32>(),
            )) as usize)
        }
    }
}

/// AHCI 单端口控制器运行时实例。
///
/// - 中断处理路径仅访问 `state` 自旋锁、MMIO 寄存器、`mailbox` 原子字与 `waiters` 等待队列；
/// - `controls` DMA 控制缓冲区的 CPU 访问由 `control` 睡眠锁独占保护；
/// - 上层块设备持有者必须保持 `Arc<Ahci>` 强引用存活，直到所有在途请求、中断回调及工作队列任务全部执行完毕，
///   且 [`Ahci::reset_complete`] 确认硬件 DMA 引擎彻底停止（`StopPhase::Stopped`）。
pub(crate) struct Ahci {
    /// 当前控制器绑定的活动 SATA 端口 MMIO 句柄。
    port: Port,
    /// 经中断子系统映射后的外部中断号。
    interrupt: IrqNumber,
    /// 结合平台 DMA 掩码与控制器 `CAP.S64A` 能力构造的 DMA 地址域。
    domain: DmaDomain,
    /// 探测阶段通过 `IDENTIFY DEVICE` 解析出的磁盘几何参数与命令集能力。
    identify: Option<ata::Identify>,
    /// 端口 DMA 控制缓冲区集合；使用 `ManuallyDrop` 包装以确保仅在 `Drop` 断言硬件已停稳后才释放底层物理页。
    controls: ManuallyDrop<Controls>,
    /// 串行化命令准备、门铃下发、等待完成与引擎停止全过程的睡眠互斥锁。
    control: SleepMutex<()>,
    /// 保护端口启停阶段（`StopPhase`）与中断/接收标志的自旋锁。
    state: SpinLock<State>,
    /// 累积中断顶半部已应答的 `PxIS` 端口中断状态位图的原子信箱。
    mailbox: AtomicU32,
    /// 供命令执行线程睡眠等待中断完成或重置唤醒的等待队列。
    waiters: WaitQueue,
}

impl Ahci {
    /// 根据平台提供的 AHCI 硬件资源初始化 HBA、扫描端口、执行 COMRESET 链路复位并完成 `IDENTIFY DEVICE` 介质识别。
    ///
    /// - 若所有已实现端口经扫描确认均无设备连接（`DET == 0`），则停止各端口并返回 `Ok(None)`；
    /// - 若存在设备但链路未就绪、签名非 ATA 磁盘或 `IDENTIFY` 失败，则返回对应的 `Err(BlockError)`（不会将硬件错误掩盖为无介质）。
    ///
    /// # Safety
    /// 1. `resources.registers` 必须是真实的 AHCI ABAR 物理区域，已在内核中建立永久强序（Device）映射且由本驱动独占，固件不再访问；
    /// 2. `resources.dma_mask` 与 `resources.dma_coherent` 必须如实反映平台总线对本控制器的 DMA 寻址限制与缓存一致性特征；
    /// 3. 调用方收到返回的 `Arc<Self>` 后，必须在注册 IRQ 和提交请求期间持续保活该 `Arc`，并在销毁前确保所有外部回调结束且 [`Self::reset_complete`] 返回 `true`。
    pub(crate) unsafe fn prepare(
        resources: &AhciResources,
    ) -> Result<Option<Arc<Self>>, BlockError> {
        // Step 1: 验证当前处于可睡眠任务上下文，映射中断号并校验平台 DMA 地址掩码。
        crate::sched::assert_switchable();
        let interrupt =
            irq::map_source(resources.interrupt_source).map_err(|_| BlockError::InvalidRange)?;
        // `None` 表示平台总线无额外收紧限制；最终 DMA 掩码上限由控制器 `CAP.S64A` 共同决定。
        let device_mask = match resources.dma_mask {
            Some(mask) => mask,
            None => u64::MAX,
        };
        if device_mask == 0 || device_mask & device_mask.wrapping_add(1) != 0 {
            return Err(BlockError::InvalidRange);
        }

        // Step 2: 构造 HBA 寄存器窗口，执行 BIOS/OS 交接与 HBA 复位，并根据 CAP.S64A 建立 DMA 地址域。
        // SAFETY: `prepare` 的调用方契约保证了 `resources.registers` 是永久强序映射且由本驱动独占的真实 ABAR 窗口。
        let host = unsafe { Host::new(resources.registers) }?;
        let (cap, ports) = host.initialize(resources.ports_implemented)?;
        let mask = device_mask.min(if cap & registers::CAP_64BIT != 0 {
            u64::MAX
        } else {
            u32::MAX as u64
        });
        let domain = DmaDomain::new(
            mask,
            0,
            usize::MAX,
            0,
            DMA_CACHE_LINE_BYTES,
            resources.dma_coherent,
        )?;

        // Step 3: 遍历所有已实现端口，为其上电并探测第一个已建立活动连接的标准 SATA 磁盘端口。
        let mut selected = None;
        for index in 0..registers::MAX_PORTS {
            if ports & (1 << index) == 0 {
                continue;
            }
            let port = host.port(index)?;
            port.power_up(cap);
            if registers::poll(Duration::from_secs(1), || port.connected()) {
                selected = Some(port);
                break;
            } else if !registers::media_absent(port.read(registers::SSTS)) {
                return Err(BlockError::NotReady);
            }
        }
        let Some(port) = selected else {
            // 所有端口均无介质连接：逐端口确认 DMA 引擎停止并清理中断状态后返回 `Ok(None)`。
            // 此时尚未分配或向硬件写入任何 DMA 控制页地址。
            for index in 0..registers::MAX_PORTS {
                if ports & (1 << index) == 0 {
                    continue;
                }
                let port = host.port(index)?;
                if !port.stop_polled() {
                    return Err(BlockError::StopUnconfirmed);
                }
                port.clear_pending();
            }
            return Ok(None);
        };

        // Step 4: 分配四块 DMA 控制页，开启端口 FIS 接收引擎，并执行 COMRESET 链路复位与设备签名验证。
        let controls = Controls::allocate(domain)?;
        port.start_fis_rx(
            controls.list.region().device.value(),
            controls.fis.region().device.value(),
        );
        if let Err(error) = port.comreset() {
            if !port.stop_polled() {
                core::mem::forget(controls);
                panic!("AHCI comreset stop unconfirmed; control pages retained");
            }
            crate::early_println!(
                "AHCI probe failed stage=comreset cmd={:#x} ssts={:#x} sig={:#x} tfd={:#x}",
                port.read(CMD),
                port.read(registers::SSTS),
                port.read(registers::SIG),
                port.read(TFD),
            );
            return Err(error);
        }
        if !registers::poll(Duration::from_secs(1), || port.taskfile_ready()) {
            if !port.stop_polled() {
                core::mem::forget(controls);
                panic!("AHCI taskfile stop unconfirmed; control pages retained");
            }
            crate::early_println!(
                "AHCI probe failed stage=taskfile cmd={:#x} ssts={:#x} sig={:#x} tfd={:#x}",
                port.read(CMD),
                port.read(registers::SSTS),
                port.read(registers::SIG),
                port.read(TFD),
            );
            return Err(BlockError::NotReady);
        }
        if port.read(registers::SIG) != registers::ATA_SIGNATURE {
            if !port.stop_polled() {
                core::mem::forget(controls);
                panic!("AHCI signature stop unconfirmed; control pages retained");
            }
            crate::early_println!(
                "AHCI probe failed stage=signature cmd={:#x} ssts={:#x} sig={:#x} tfd={:#x}",
                port.read(CMD),
                port.read(registers::SSTS),
                port.read(registers::SIG),
                port.read(TFD),
            );
            return Err(BlockError::Unsupported);
        }

        // Step 5: 在构造控制器实例并准备 IDENTIFY 命令前，先停止端口 FIS 接收引擎以收回控制页的 CPU 独占访问权。
        // 若后续 Arc 内存分配失败而提前返回，此时硬件已确认停止，释放 controls 不会引发 DMA 悬垂访问。
        if !port.stop_polled() {
            core::mem::forget(controls);
            panic!("AHCI probe stop unconfirmed; control pages retained");
        }
        let mut controller = Arc::try_new(Self {
            port,
            interrupt,
            domain,
            identify: None,
            controls: ManuallyDrop::new(controls),
            control: SleepMutex::new(()),
            state: SpinLock::new(State {
                phase: StopPhase::Stopped,
                published: false,
                accepting: false,
                interrupts: false,
                issued: false,
            }),
            mailbox: AtomicU32::new(0),
            waiters: WaitQueue::new(),
        })
        .map_err(|_| BlockError::NoMemory)?;

        // Step 6: 通过轮询方式执行 IDENTIFY DEVICE 命令，解析磁盘容量与能力后完成控制器初始化。
        match controller.identify_polled() {
            Ok(identify) => {
                Arc::get_mut(&mut controller)
                    .expect("unpublished AHCI Arc escaped")
                    .identify = Some(identify);
                Ok(Some(controller))
            }
            Err(BlockError::StopUnconfirmed) => {
                // 若 IDENTIFY 后端口未能确认停稳，硬件可能仍在对控制页或 IDENTIFY 页做 DMA，
                // 必须通过 `Arc::into_raw` 泄露强引用并立即 panic，防止展开析构释放正在被 DMA 访问的物理页。
                let retained = Arc::into_raw(controller);
                panic!(
                    "AHCI identify stop unconfirmed; owner retained at {:p}",
                    retained
                );
            }
            Err(error) => Err(error),
        }
    }

    /// 返回探测阶段由 `IDENTIFY DEVICE` 解析出的队列限额（容量、扇区大小、PRDT 段限制及 FLUSH 支持）。
    pub(crate) fn limits(&self) -> QueueLimits {
        self.identify.expect("AHCI identify missing").limits()
    }

    /// 返回本控制器核对后的 DMA 地址域，供上层块设备层映射和同步请求数据页使用。
    pub(crate) fn domain(&self) -> DmaDomain {
        self.domain
    }

    /// 返回本控制器绑定的外部中断号。
    pub(crate) fn interrupt(&self) -> IrqNumber {
        self.interrupt
    }

    /// 在上层块设备核心已保活 `Arc<Ahci>` 并准备好中断处理程序后，正式发布控制器并开启 HBA 全局中断使能。
    ///
    /// 此阶段仅清除残留中断并置位 `GHC.IE`，端口 DMA 引擎要到第一条命令下发（[`Self::issue`]）时才会启动。
    pub(crate) fn publish(&self) -> Result<(), BlockError> {
        let mut state = self.state.lock();
        if state.published || state.phase != StopPhase::Stopped || !self.port.connected() {
            return Err(BlockError::NotReady);
        }
        self.port.clear_pending();
        self.mailbox.store(0, Ordering::Release);
        state.published = true;
        state.accepting = true;
        self.port.enable_host_irq();
        Ok(())
    }

    /// 在共享中断线或等待前检查中认领本端口的中断状态：
    /// 若 `PxIS` 非零，则通过写 1 清零（W1C）应答端口与 HBA 中断、将状态位或入 `mailbox` 并唤醒等待队列。
    ///
    /// 可在关中断的 IRQ 顶半部或普通任务上下文中安全调用，不访问任何 DMA 缓冲区。
    pub(crate) fn acknowledge(&self) -> bool {
        let claimed = {
            let _state = self.state.lock();
            let status = self.port.read(PORT_IS);
            if status == 0 {
                false
            } else {
                self.port.write(PORT_IS, status);
                self.port.ack_host();
                self.mailbox.fetch_or(status, Ordering::AcqRel);
                true
            }
        };
        if claimed {
            self.waiters.wake_all();
        }
        claimed
    }

    /// 更新本端口的中断屏蔽位（`PxIE`）。
    ///
    /// 仅当 `enabled` 为 `true`、端口处于允许接收命令状态（`accepting`）且 DMA 引擎正在运行（`StopPhase::Running`）时，
    /// 才会向硬件 `PxIE` 写入 [`INTERRUPTS`] 掩码；在命令执行期间，上层块核心需保持中断使能开启。
    pub(crate) fn set_interrupts(&self, enabled: bool) {
        let mut state = self.state.lock();
        state.interrupts = enabled;
        self.port.write(
            IE,
            if enabled && state.accepting && state.phase == StopPhase::Running {
                INTERRUPTS
            } else {
                0
            },
        );
        let _ = self.port.read(IE);
    }

    /// 开启端口中断使能并立即执行一次中断状态认领，返回 `mailbox` 中是否已有待处理的硬件中断事件。
    pub(crate) fn arm_and_check(&self) -> bool {
        self.set_interrupts(true);
        self.acknowledge();
        self.mailbox.load(Ordering::Acquire) != 0
    }

    /// 在自旋锁临界区内禁止接收新命令、关闭端口中断并启动端口停止流程的第一步（清除 `CMD.ST`），随后唤醒所有等待者。
    ///
    /// 本函数不会阻塞等待硬件 `CR` 或 `FR` 清零，可在任意非睡眠上下文中调用。
    pub(crate) fn request_reset(&self) {
        {
            let mut state = self.state.lock();
            state.accepting = false;
            state.interrupts = false;
            self.begin_stop(&mut state);
        }
        self.waiters.wake_all();
    }

    /// 向前推进一次端口 DMA 引擎停止状态机（非阻塞 MMIO 探测），返回引擎是否已彻底停稳（`StopPhase::Stopped`）。
    ///
    /// 状态迁移遵循 AHCI 硬件规范：
    /// - `StopPhase::Command`：确认 `CMD.CR == 0` 后，清除 `CMD.FRE` 并转入 `StopPhase::Fis`；
    /// - `StopPhase::Fis`：确认 `CMD.FR == 0` 后，执行 DMA 读屏障确保设备写入对 CPU 可见，随后转入 `StopPhase::Stopped` 并返回 `true`。
    ///
    /// 只有当本函数返回 `true` 后，上层才可安全收回并释放之前移交给设备的物理页。
    pub(crate) fn reset_complete(&self) -> bool {
        let mut state = self.state.lock();
        match state.phase {
            StopPhase::Running => false,
            StopPhase::Command => {
                if self.port.read(CMD) & CR != 0 {
                    return false;
                }
                self.port.write(CMD, self.port.read(CMD) & !FRE);
                let _ = self.port.read(CMD);
                state.phase = StopPhase::Fis;
                false
            }
            StopPhase::Fis => {
                if self.port.read(CMD) & FR != 0 {
                    return false;
                }
                arch::dma_read_barrier();
                state.phase = StopPhase::Stopped;
                state.issued = false;
                true
            }
            StopPhase::Stopped => true,
        }
    }

    /// 在可睡眠的块设备工作队列任务（`progress` Work）中同步提交一条直接 DMA 命令，睡眠等待硬件中断完成并返回传输结果。
    ///
    /// # Safety
    /// 1. `ranges` 描述的所有物理内存段必须属于上层块核心当前独占持有的已接收请求负载（Payload），
    ///    且在调用前已通过 [`Self::domain`] 完成 DMA 映射与向设备的缓存同步（移交 DMA 访问权）；
    /// 2. 在本函数执行期间，CPU 不得并发读写 `ranges` 覆盖的物理页；
    /// 3. 若返回结果包含 [`BlockError::StopUnconfirmed`]，说明端口 DMA 引擎在超时后仍未确认停稳，
    ///    硬件可能仍在通过 DMA 访问这些物理页；此时调用方**严禁**完成该请求、释放软件槽位、归还物理页或销毁 `Arc<Ahci>`，
    ///    必须保留它们直到后续 [`Self::reset_complete`] 返回 `true`。除 `StopUnconfirmed` 外的其他返回结果均保证硬件已彻底停止访问这些范围。
    pub(crate) unsafe fn execute(
        &self,
        operation: BlkOperation,
        sector: u64,
        bytes: usize,
        ranges: &[PayloadRange],
    ) -> IoResult {
        // Step 1: 获取可睡眠控制锁，校验请求范围并填充 PRDT 与命令头。
        let _control = self.control.lock();
        if let Err(error) = self.prepare_transfer(operation, sector, bytes, ranges) {
            return failed(error);
        }

        // Step 2: 在自旋锁内检查端口状态与寄存器空闲条件，启动端口引擎并写 `CI` 门铃下发命令。
        {
            let mut state = self.state.lock();
            if !state.published || !state.accepting {
                return failed(BlockError::QueueStopped);
            }
            if !state.interrupts || state.phase != StopPhase::Stopped {
                return failed(BlockError::NotReady);
            }
            if self.port.read(TFD) & TASK_BUSY != 0
                || self.port.read(CI) != 0
                || self.port.read(SACT) != 0
            {
                return failed(BlockError::DeviceFault);
            }
            self.issue(&mut state, true);
        }

        // Step 3: 睡眠等待硬件中断完成或 5 秒超时，随后请求停止端口 DMA 引擎并等待 `CR`/`FR` 归零。
        let deadline = time::now()
            .checked_add(COMMAND_TIMEOUT)
            .expect("AHCI command deadline overflow");
        let completion = self.wait_irq(deadline);
        if completion.is_err() {
            self.request_reset();
        } else {
            let mut state = self.state.lock();
            self.begin_stop(&mut state);
        }
        if !self.stop_bounded(true) {
            self.request_reset();
            return failed(BlockError::StopUnconfirmed);
        }

        // Step 4: 引擎已确认停稳，同步控制页并读取 PRDBC 已传输字节数，核对任务文件与中断错误位。
        let transferred = match self.controls.transferred() {
            Ok(transferred) => transferred,
            Err(error) => {
                self.request_reset();
                return failed(error);
            }
        };
        if let Err(error) = completion {
            // 命令执行出错或超时时，ATA 协议不保证任何前缀扇区已落盘，统一按 0 已完成字节报告失败。
            return failed(error);
        }
        if self.port.read(TFD) & (TASK_ERROR | TASK_BUSY) != 0
            || self.port.read(PORT_IS) & ERROR != 0
            || self.mailbox.load(Ordering::Acquire) & ERROR != 0
        {
            self.request_reset();
            return failed(BlockError::Io);
        }
        if bytes < transferred {
            self.request_reset();
            return failed(BlockError::DeviceFault);
        }
        if transferred != bytes {
            self.request_reset();
        }
        IoResult {
            completed: transferred,
            error: if transferred == bytes {
                None
            } else {
                Some(BlockError::ShortIo)
            },
        }
    }

    /// 校验块请求限额，将物理内存段按 4 MiB 上限拆分为 PRDT 表项填入命令表，并编码 H2D FIS 命令头。
    fn prepare_transfer(
        &self,
        operation: BlkOperation,
        sector: u64,
        bytes: usize,
        ranges: &[PayloadRange],
    ) -> Result<(), BlockError> {
        // Step 1: 确认端口当前允许接收请求且处于已停止状态。
        {
            let state = self.state.lock();
            if !state.accepting || state.phase != StopPhase::Stopped {
                return Err(BlockError::QueueStopped);
            }
        }
        // Step 2: 对照 IDENTIFY 队列限额校验扇区范围与 SG 段约束，并编码 20 字节 ATA FIS。
        let identify = self.identify.expect("AHCI identify missing");
        identify
            .limits()
            .validate(operation, sector, bytes, ranges.iter().copied(), 0)?;
        if ata::MAX_BYTES < bytes || ata::MAX_PRDT < ranges.len() {
            return Err(BlockError::InvalidRange);
        }

        let command = identify.command(operation, sector, bytes)?;
        // Step 3: 将每个物理段映射为设备 DMA 地址，按单表项最大 4 MiB 拆分写入 PRDT。
        let mut entries = 0;
        for range in ranges {
            let mut address = self
                .domain
                .map(range.physical, range.length as usize)?
                .value();
            let mut remaining = range.length as usize;
            while remaining != 0 {
                let length = remaining.min(ata::PRDT_BYTES);
                self.controls.prdt(entries, address, length)?;
                address = address
                    .checked_add(length as u64)
                    .ok_or(BlockError::InvalidRange)?;
                remaining -= length;
                entries += 1;
            }
        }

        // Step 4: 写入命令表 CFIS 与槽 0 命令头，并同步控制页缓存。
        self.controls
            .command(&command, operation == BlkOperation::Write, entries)
    }

    /// 在持有 `state` 自旋锁的临界区内清空残留状态、配置控制页基址、启动 `FRE`/`ST` 引擎，最后写入 `PxCI` 门铃触发槽 0 命令。
    fn issue(&self, state: &mut State, interrupts: bool) {
        self.port.clear_pending();
        self.mailbox.store(0, Ordering::Release);
        let list = self.controls.list.region().device.value();
        let fis = self.controls.fis.region().device.value();
        self.port.write(CLBU, (list >> 32) as u32);
        self.port.write(CLB, list as u32);
        self.port.write(FBU, (fis >> 32) as u32);
        self.port.write(FB, fis as u32);
        self.port.write(CMD, self.port.read(CMD) | FRE);
        let _ = self.port.read(CMD);
        self.port.write(CMD, self.port.read(CMD) | ST);
        let _ = self.port.read(CMD);
        state.phase = StopPhase::Running;
        state.issued = true;
        self.port.write(IE, if interrupts { INTERRUPTS } else { 0 });
        arch::dma_write_barrier();
        self.port.write(CI, ACTIVE_COMMAND);
        let _ = self.port.read(CI);
    }

    /// 检查当前下发的命令是否已收到中断完成通知或发生错误。
    ///
    /// 仅当 `mailbox` 记录了实际硬件中断完成标志（`COMPLETION`）且 `PxCI` 槽 0 位已被硬件清零时，
    /// 才判定命令完成（单纯 `CI == 0` 但缺失中断标志不能视为正常完成）。
    fn completion(&self) -> Option<Result<(), BlockError>> {
        let state = self.state.lock();
        if !state.accepting || state.phase != StopPhase::Running {
            return Some(Err(BlockError::QueueStopped));
        }
        let status = self.mailbox.load(Ordering::Acquire);
        if status & ERROR != 0 {
            return Some(Err(BlockError::Io));
        }
        if state.issued && status & COMPLETION != 0 && self.port.read(CI) & ACTIVE_COMMAND == 0 {
            return Some(if self.port.read(TFD) & TASK_ERROR == 0 {
                Ok(())
            } else {
                Err(BlockError::Io)
            });
        }
        None
    }

    /// 在截止时间 `deadline` 前挂起当前线程，等待端口中断唤醒或高精度定时器超时。
    ///
    /// 若被伪唤醒或阶段性中断唤醒但命令尚未完成，则沿用原绝对 `deadline` 重新注册等待，直到完成、出错或超时。
    fn wait_irq(&self, deadline: MonotonicInstant) -> Result<(), BlockError> {
        loop {
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY:
            // 1. `self.waiters` 由当前正在执行的 `&self` 保活，`node` 通过 `pin!` 固定在当前栈帧上；
            // 2. 在本轮循环末尾 `node` 与 `waiter` 离开作用域前，会先显式 `drop(registration)` 将节点从等待队列链表中安全摘除。
            let registration = unsafe { self.waiters.register(node.as_mut()) };
            let waker = pin!(waiter.waker());
            let timeout = pin!(Timer::new(
                crate::cpu::current().id,
                TimerKind::Precise,
                wake_timeout,
                waker.as_ref().get_ref() as *const Waker as usize,
            ));
            // SAFETY:
            // 1. `timeout` 与 `waker` 均通过 `pin!` 固定在当前栈帧上，`wake_timeout` 回调读取的 `*const Waker` 指针在栈帧存活期间始终有效；
            // 2. 在本轮循环退出、`waker` 与 `timeout` 析构前，必先调用 `timeout.as_ref().cancel_sync()` 同步取消定时器并等待可能正在执行的回调彻底退出。
            unsafe { timeout.as_ref().arm_borrowed(deadline) }.expect("AHCI timeout rejected");

            self.acknowledge();
            let mut result = self.completion();
            if result.is_none() && time::now() < deadline {
                waiter.park();
                // 若因达到截止时间而被唤醒，直接判定为超时 I/O 错误，不再探测寄存器状态。
                result = if deadline <= time::now() {
                    Some(Err(BlockError::Io))
                } else {
                    self.completion()
                };
            } else if result.is_none() {
                result = Some(Err(BlockError::Io));
            }

            drop(registration);
            drop(waiter);
            timeout.as_ref().cancel_sync();
            if let Some(result) = result {
                return result;
            }
        }
    }

    /// 在持有 `state` 自旋锁的前提下执行端口停止的第一步：屏蔽端口中断（`IE = 0`），
    /// 若当前处于 `StopPhase::Running` 则清除 `CMD.ST` 并转入 `StopPhase::Command`。
    fn begin_stop(&self, state: &mut State) {
        self.port.write(IE, 0);
        if state.phase == StopPhase::Running {
            self.port.write(CMD, self.port.read(CMD) & !ST);
            let _ = self.port.read(CMD);
            state.phase = StopPhase::Command;
        }
    }

    /// 在自旋锁外循环调用 [`Self::reset_complete`] 推进端口停止状态机，直到停稳或达到 `STOP_TIMEOUT`（2 秒）。
    ///
    /// - `sleep == true`（运行期命令执行路径）：每次未停稳时睡眠 1 毫秒让出 CPU；
    /// - `sleep == false`（探测阶段 `identify_polled` 路径）：使用 `spin_loop` 忙等。
    fn stop_bounded(&self, sleep: bool) -> bool {
        let deadline = time::now()
            .checked_add(STOP_TIMEOUT)
            .expect("AHCI stop deadline overflow");
        loop {
            if self.reset_complete() {
                return true;
            }
            if deadline <= time::now() {
                return false;
            }
            if sleep {
                timer::sleep(Duration::from_millis(1));
            } else {
                core::hint::spin_loop();
            }
        }
    }

    /// 在探测阶段（未开启中断）通过轮询方式下发 `IDENTIFY DEVICE` 命令，读取并解析 512 字节设备识别信息。
    fn identify_polled(&self) -> Result<ata::Identify, BlockError> {
        // Step 1: 配置 PRDT 第 0 项指向 `identify` 缓冲区，同步 DMA 缓存并填写 `IDENTIFY` 命令帧。
        self.controls.prdt(
            0,
            self.controls.identify.region().device.value(),
            ata::IDENTIFY_BYTES,
        )?;
        // SAFETY:
        // 1. 此时端口处于 `StopPhase::Stopped`，设备尚未访问 `identify` 缓冲区；
        // 2. `self.controls.identify` 独占整页且由 `self` 保活，调用 `sync_for_device` 将缓冲区访问权移交设备。
        unsafe {
            self.controls.identify.sync_for_device(
                0,
                self.controls.identify.byte_len(),
                DmaDirection::FromDevice,
            )
        }?;
        self.controls.command(&ata::fis(ata::IDENTIFY), false, 1)?;

        // Step 2: 以屏蔽中断方式（interrupts = false）下发命令，并轮询等待命令完成或报错。
        {
            let mut state = self.state.lock();
            self.issue(&mut state, false);
        }
        let completed = registers::poll(COMMAND_TIMEOUT, || {
            self.port.read(PORT_IS) & ERROR != 0 || self.port.read(CI) & ACTIVE_COMMAND == 0
        });
        let failed_command = !completed
            || self.port.read(PORT_IS) & ERROR != 0
            || self.port.read(TFD) & TASK_ERROR != 0;
        // Step 3: 停止端口 DMA 引擎，确认 `CR == 0` 且 `FR == 0` 后收回 `identify` 缓冲区的 CPU 访问权。
        {
            let mut state = self.state.lock();
            self.begin_stop(&mut state);
        }
        if !self.stop_bounded(false) {
            return Err(BlockError::StopUnconfirmed);
        }

        let transferred = self.controls.transferred()?;
        // SAFETY:
        // 1. 上方 `stop_bounded(false)` 已确认端口 `CR == 0` 且 `FR == 0`，设备已彻底结束对 `identify` 缓冲区的 DMA 写入；
        // 2. `self.controls.identify` 物理页存活且期间无并发 CPU 访问，调用 `sync_for_cpu` 刷新 CPU 缓存以读取设备写入的数据。
        unsafe {
            self.controls.identify.sync_for_cpu(
                0,
                self.controls.identify.byte_len(),
                DmaDirection::FromDevice,
            )
        }?;
        if failed_command {
            return Err(BlockError::Io);
        }
        if transferred != ata::IDENTIFY_BYTES {
            return Err(BlockError::ShortIo);
        }

        // Step 4: 从 `identify` 缓冲区按小端序读取 256 个 16-bit 字并解析设备几何参数。
        let mut words = [0; ata::IDENTIFY_WORDS];
        // SAFETY:
        // 1. 已完成 `sync_for_cpu` 且端口 DMA 引擎已停稳，CPU 独占访问 `self.controls.identify`；
        // 2. `self.controls.identify` 容量为 `IDENTIFY_BYTES`（512 字节）且满足 `u16` 对齐，读取 256 个 `u16` 不会越界。
        unsafe {
            let source = self.controls.identify.region().cpu.as_ptr().cast::<u16>();
            for (index, word) in words.iter_mut().enumerate() {
                *word = u16::from_le(ptr::read_volatile(source.add(index)));
            }
        }
        ata::Identify::parse(&words)
    }
}

impl Drop for Ahci {
    /// 析构 AHCI 控制器：断言端口 DMA 引擎已彻底停稳（`StopPhase::Stopped`），随后释放 `controls` DMA 物理页。
    fn drop(&mut self) {
        assert!(
            self.state.lock().phase == StopPhase::Stopped,
            "dropping active AHCI controller"
        );
        // SAFETY:
        // 1. 当最后一个 `Arc<Ahci>` 强引用被释放并进入 `Drop::drop` 时，调用方契约保证所有 IRQ 与工作队列任务均已退出，且 `&mut self` 保证无并发 CPU 访问；
        // 2. 上方 `assert!` 已确认端口处于 `StopPhase::Stopped`（`CR == 0` 且 `FR == 0`），硬件不再对 `controls` 进行 DMA 访问，且 `self.controls` 仅在此处 `drop` 一次。
        unsafe { ManuallyDrop::drop(&mut self.controls) };
    }
}

/// 命令超时定时器的到期回调函数：从定时器上下文恢复栈上固定的 [`Waker`] 引用并唤醒等待中的命令执行线程。
fn wake_timeout(timer: Pin<&Timer>) {
    // SAFETY:
    // `wait_irq` 在挂载定时器前将 `Waker` 通过 `pin!` 固定在栈上，并在 `waker` 离开作用域前调用 `cancel_sync()`
    // 等待本回调执行完毕，因此在回调执行期间 `timer.context()` 保存的 `*const Waker` 指针始终有效且对齐。
    let waker = unsafe { &*(timer.context() as *const Waker) };
    waker.notify();
}

/// 构造已完成字节数为 0 的错误 [`IoResult`]（用于命令下发前校验失败或执行期出错无法确认成功传输字节数的场景）。
fn failed(error: BlockError) -> IoResult {
    IoResult {
        completed: 0,
        error: Some(error),
    }
}

#[cfg(feature = "ktest")]
#[::ktest::suite(context = "Boot", tags("block", "ahci", "protocol"), timeout_ms = 1000)]
mod protocol {
    /// 验证 ATA `IDENTIFY DEVICE` 参数解析与 Register H2D FIS 命令编码协议逻辑。
    ///
    /// - **测试目标**：在不依赖真实硬件的前提下，验证 4Kn 扇区换算、LBA48 寻址编码、独立 FLUSH 能力判定及越界/未对齐错误拦截。
    /// - **场景步骤（Given-When-Then）**：调用 [`super::ata::self_check`] 依次检验 4096B 逻辑扇区、LBA48 读写 FIS 字段、
    ///   未对齐扇区拒绝、`FLUSH CACHE` / `FLUSH CACHE EXT` 操作码选择以及非法扇区大小报错。
    /// - **预期结果**：所有断言均通过。
    #[test]
    fn identify_geometry_and_flush() {
        super::ata::self_check();
    }

    /// 验证 SATA 端口 `PxSSTS` 状态寄存器的缺盘（无介质）判定边界。
    ///
    /// - **测试目标**：确保只有 `DET == 0` 被识别为无设备连接，防止将链路协商未完成、PHY 离线或 MMIO 读取全 1 误判为正常缺盘。
    /// - **场景步骤（Given-When-Then）**：分别向 [`super::registers::media_absent`] 传入 `0`（无设备）、`1`（检测到设备但 PHY 未通信）、
    ///   `0x103`（Active 且通信建立）、`4`（PHY 离线）与 `u32::MAX`（MMIO 失效）。
    /// - **预期结果**：仅输入 `0` 时返回 `true`，其余状态均返回 `false`。
    #[test]
    fn media_absence_boundary() {
        assert!(super::registers::media_absent(0));
        assert!(!super::registers::media_absent(1));
        assert!(!super::registers::media_absent(0x103));
        assert!(!super::registers::media_absent(4));
        assert!(!super::registers::media_absent(u32::MAX));
    }
}
