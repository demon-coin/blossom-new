//! DWMAC4 与 DWMAC1000 分代 DMA 描述符环及缓存一致性管理模块。
//!
//! ## 模块职责与架构定位
//! 本模块位于 `drivers::dwmac` 内部，被外层 `Dwmac` 驱动调用，向下依赖 `mm::dma::CoherentDma` 与架构级缓存维护原语（`arch::dma_cache_flush`、内存屏障）。
//! 它负责分配固定长度（`RING_LEN = 32`）的收发描述符环形缓冲区，并屏蔽 Synopsys DWMAC4（JH7110）与 DWMAC1000（LS2K1000LA 增强描述符）两代控制器在描述符二进制格式、`OWN` 位位置、跳过长度（DSL）单位以及缓存同步协议上的硬件差异。
//! 许可与修改声明见本目录 `LICENSE`。
//!
//! ## 描述符槽内存布局与缓存行隔离
//! 在非一致性 DMA（Non-coherent DMA）平台上，如果相邻描述符共享同一个 CPU 缓存行，CPU 更新槽 A 的控制字并执行写回（Clean）时，可能会用缓存中的旧副本覆盖设备正在向槽 B 写回的完成状态，导致 DMA 状态丢失或内存踩踏。
//! 因此非一致性模式下每个描述符槽强制对齐并独占完整的 64 字节（`ISOLATED_DESCRIPTOR_BYTES`）缓存行，中间空隙通过 DMA 跳过长度（Descriptor Skip Length, DSL）交由硬件自动跨过：
//!
//! ```text
//! DWMAC4 描述符槽（非一致固定 64B，硬件本体 16B，DSL = 48B / 8B AXI beat = 6）：
//! +------------------+------------------+------------------+------------------+
//! | DES0 (0x00, 4B)  | DES1 (0x04, 4B)  | DES2 (0x08, 4B)  | DES3 (0x0C, 4B)  |
//! |  Buf1 Addr Low   |  Buf1 Addr High  |  IOC / TX Len    | OWN | FS/LS | Len|
//! +------------------+------------------+------------------+------------------+
//! |                       Cacheline Padding (48 Bytes)                        |
//! +---------------------------------------------------------------------------+
//!
//! DWMAC1000 增强描述符槽（一致 32B / DSL = 0；非一致 64B / DSL = 32B / 4B word = 8）：
//! +------------------+------------------+------------------+------------------+
//! | DES0 (0x00, 4B)  | DES1 (0x04, 4B)  | DES2 (0x08, 4B)  | DES3..7 (20B)    |
//! | OWN | IC/FS/LS   |  TER | Buf1 Size |  Buf1 Addr (32b) | Reserved / Ext   |
//! +------------------+------------------+------------------+------------------+
//! |               Optional Non-coherent Padding (32 Bytes)                    |
//! +---------------------------------------------------------------------------+
//! ```
//!
//! ## 并发与所有权契约
//! - `Ring` 本身不携带内部锁；所有槽位的读写顺序由外层 `Dwmac::state` 自旋锁与 `SlotState` 状态机串行化协调。
//! - `OWN` 位（最高位 `1 << 31`）是 CPU 与 DMA 引擎之间的硬件所有权分界：发布描述符时必须先写妥地址与长度字、插入写屏障，最后写入带 `OWN = 1` 的状态控制字并刷出缓存；轮询完成状态时在 `OWN == 0` 确认前绝不收回关联的数据物理页。

use core::ptr;

use crate::{
    arch,
    drivers::dwmac::{Generation, GmacError, BUFFER_LEN, FRAME_MAX, FRAME_MIN, RING_LEN},
    mm::dma::{CoherentDma, DmaDirection, DmaDomain},
};

const OWN: u32 = 1 << 31;
const ERROR_SUMMARY: u32 = 1 << 15;

const DWMAC4_RX_CONTEXT: u32 = 1 << 30;
const DWMAC4_RX_FIRST: u32 = 1 << 29;
const DWMAC4_RX_LAST: u32 = 1 << 28;
const DWMAC4_RX_LENGTH_MASK: u32 = 0x7fff;
const DWMAC1000_RX_FIRST: u32 = 1 << 9;
const DWMAC1000_RX_LAST: u32 = 1 << 8;
const DWMAC1000_RX_LENGTH_SHIFT: u32 = 16;
const DWMAC1000_RX_LENGTH_MASK: u32 = 0x3fff;
const DWMAC4_RX_INTERRUPT: u32 = 1 << 30;
const DWMAC4_RX_BUFFER1_VALID: u32 = 1 << 24;
const DWMAC1000_RX_END_RING: u32 = 1 << 15;

const DWMAC4_TX_INTERRUPT: u32 = 1 << 31;
const DWMAC4_TX_FIRST: u32 = 1 << 29;
const DWMAC4_TX_LAST: u32 = 1 << 28;
const DWMAC1000_TX_INTERRUPT: u32 = 1 << 30;
const DWMAC1000_TX_LAST: u32 = 1 << 29;
const DWMAC1000_TX_FIRST: u32 = 1 << 28;
const DWMAC1000_TX_END_RING: u32 = 1 << 21;

const DESCRIPTOR_WORD_BYTES: usize = core::mem::size_of::<u32>();
const DWMAC4_DESCRIPTOR_BYTES: usize = 4 * DESCRIPTOR_WORD_BYTES;
const DWMAC1000_DESCRIPTOR_WORDS: usize = 8;
const DWMAC1000_DESCRIPTOR_BYTES: usize = DWMAC1000_DESCRIPTOR_WORDS * DESCRIPTOR_WORD_BYTES;
const JH7110_AXI_BEAT_BYTES: usize = core::mem::size_of::<u64>();
pub(super) const ISOLATED_DESCRIPTOR_BYTES: usize = 64;
const ETHERNET_FCS_BYTES: usize = 4;

/// 分代 DMA 描述符环形缓冲区。
///
/// 底层控制内存由 `CoherentDma` 分配并驻留，在整个 `Dwmac` 生命周期内保持物理页存活，直至硬件 DMA 软复位确认完成后才随外层 `Inner` 一并释放。
pub(super) struct Ring {
    /// 承载整个描述符环的连续 DMA 控制内存区域（总字节数为 `RING_LEN * stride`）。
    memory: CoherentDma,
    /// 所属 MAC 硬件代数，决定 4 字（DWMAC4）或 8 字（DWMAC1000 增强格式）的字段编码与状态字偏移。
    generation: Generation,
    /// 单个描述符槽占用的字节步长（一致性 DWMAC1000 为 32 字节，其余配置均为缓存行隔离的 64 字节）。
    stride: usize,
    /// 设备与 CPU 之间是否具备硬件缓存一致性；为 `false` 时需在读写边界执行显式缓存维护。
    coherent: bool,
}

impl Ring {
    /// 在普通可睡眠准备上下文中分配并清零整个描述符环控制内存，随后将其同步到设备可见状态。
    ///
    /// 初始清零保证所有槽位的 `OWN` 位均为 `0`（归属 CPU），防止 TX DMA 引擎启动后误发未初始化槽位。
    pub(super) fn new(
        domain: DmaDomain,
        generation: Generation,
        coherent: bool,
    ) -> Result<Self, GmacError> {
        let stride = descriptor_stride(generation, coherent);
        let memory = CoherentDma::allocate(domain, RING_LEN * stride)?;
        // SAFETY:
        // 1. `memory` 刚由 `CoherentDma::allocate` 分配并清零，尚未将基址写入任何硬件寄存器，无并发访问；
        // 2. 同步范围 `[0, RING_LEN * stride)` 精确覆盖整块控制内存，确保初始 `OWN = 0` 在 DMA 引擎启动前刷入主存。
        unsafe { memory.sync_for_device(0, RING_LEN * stride, DmaDirection::Bidirectional)? };
        Ok(Self {
            memory,
            generation,
            stride,
            coherent,
        })
    }

    /// 计算第 `index` 个描述符槽（允许等于 `RING_LEN` 以表示环尾边界地址）对应的设备侧 DMA 总线地址。
    pub(super) fn address(&self, index: usize) -> u64 {
        assert!(index <= RING_LEN);
        self.memory.region().device.value() + (index * self.stride) as u64
    }

    /// 返回需写入 DMA 控制寄存器的描述符跳过长度（DSL）字段值；DWMAC4 按 8 字节 AXI beat 计数，DWMAC1000 按 4 字节字计数。
    pub(super) fn skip_length(&self) -> u32 {
        descriptor_skip(self.generation, self.coherent)
    }

    /// 读取第 `index` 个描述符槽的硬件状态控制字快照（在持锁或消费者路径中调用）。
    ///
    /// 非一致性模式下先失效/刷出对应单槽缓存行再插入读屏障，确保读取到 DMA 引擎写回的最新状态；若返回值的 `OWN` 位仍为 `1`，说明硬件尚未完成传输，调用方不得收回对应的数据物理页。
    pub(super) fn status(&self, index: usize) -> Result<u32, GmacError> {
        assert!(index < RING_LEN);
        let offset = index * self.stride;
        if !self.coherent {
            // 该独立 64B 槽在上次移交设备前已完成缓存清理（Clean），此后 CPU 仅执行只读观测而未产生脏数据；
            // 执行缓存刷新可作废陈旧缓存行以获取 DMA 最新写回值，且因单槽独占缓存行不会干扰相邻槽位。
            let physical = arch::virt_to_phys(self.memory.region().cpu.as_ptr() as usize);
            arch::dma_cache_flush(physical + offset, self.stride).map_err(|error| {
                GmacError::Dma(crate::mm::dma::DmaError::CacheMaintenance(error))
            })?;
        }
        arch::dma_read_barrier();
        Ok(self.read(index, status_word(self.generation)))
    }

    /// 将第 `index` 个归属 CPU 的接收槽绑定到数据缓冲区 DMA 地址 `address`，并置位 `OWN` 移交硬件接收引擎。
    pub(super) fn arm_rx(&self, index: usize, address: u64) -> Result<(), GmacError> {
        let words = rx_descriptor(self.generation, index + 1 == RING_LEN, address);
        self.publish(index, words)
    }

    /// 将第 `index` 个归属 CPU 的发送槽绑定到长度为 `length` 字节的帧 DMA 地址 `address`，附带完成中断标志并置位 `OWN` 移交硬件发送引擎。
    pub(super) fn arm_tx(
        &self,
        index: usize,
        address: u64,
        length: usize,
    ) -> Result<(), GmacError> {
        let words = tx_descriptor(self.generation, index + 1 == RING_LEN, address, length);
        self.publish(index, words)
    }

    /// 按硬件可见性顺序将编码好的控制字写入第 `index` 个归属 CPU 的描述符槽，并将单槽缓存同步给设备。
    fn publish(
        &self,
        index: usize,
        words: [u32; DWMAC1000_DESCRIPTOR_WORDS],
    ) -> Result<(), GmacError> {
        assert!(index < RING_LEN);
        // Step 1: 先写入除状态控制字以外的所有地址、长度与填充字，确保此时 `OWN` 位尚未置位。
        let status = status_word(self.generation);
        for word in 0..self.stride / DESCRIPTOR_WORD_BYTES {
            if word != status {
                self.write(index, word, words.get(word).copied().unwrap_or(0));
            }
        }
        // Step 2: 插入 DMA 写屏障，保证缓冲地址和帧长度先于 `OWN = 1` 控制字对硬件可见。
        arch::dma_write_barrier();
        self.write(index, status, words[status]);
        // Step 3: 将当前独立缓存行刷出到设备可见内存，正式移交描述符槽的硬件所有权。
        // SAFETY:
        // 1. 前置 `assert!(index < RING_LEN)` 保证槽偏移 `index * self.stride` 位于 `self.memory` 分配容量内；
        // 2. 调用方状态机保证该槽在调用前处于 CPU 独占状态（设备未并发访问），且非一致性下 `stride` 为独立 64B 缓存行，不会波及相邻槽位。
        unsafe {
            self.memory.sync_for_device(
                index * self.stride,
                self.stride,
                DmaDirection::Bidirectional,
            )?
        };
        Ok(())
    }

    /// 以小端序 `read_volatile` 读取第 `index` 槽内第 `word` 个 32 位控制字，避免对异步 DMA 内存构造共享引用。
    fn read(&self, index: usize, word: usize) -> u32 {
        let offset = index * self.stride + word * DESCRIPTOR_WORD_BYTES;
        assert!(offset + DESCRIPTOR_WORD_BYTES <= RING_LEN * self.stride);
        // SAFETY: 前置 `assert!` 保证字节偏移未越界，且 `stride` 与 `DESCRIPTOR_WORD_BYTES` 均为 4 字节对齐；底层 `CoherentDma` 在整个 `Ring` 生命周期内持续有效。
        u32::from_le(unsafe {
            ptr::read_volatile(self.memory.region().cpu.as_ptr().add(offset).cast::<u32>())
        })
    }

    /// 以小端序 `write_volatile` 向第 `index` 个归属 CPU 的描述符槽写入第 `word` 个 32 位控制字。
    fn write(&self, index: usize, word: usize, value: u32) {
        let offset = index * self.stride + word * DESCRIPTOR_WORD_BYTES;
        assert!(offset + DESCRIPTOR_WORD_BYTES <= RING_LEN * self.stride);
        // SAFETY: 前置 `assert!` 保证字节偏移在 `self.memory` 范围内且满足 4 字节对齐；私有调用路径仅在槽位归属 CPU 时写入，不会与设备或相邻独立槽冲突。
        unsafe {
            ptr::write_volatile(
                self.memory.region().cpu.as_ptr().add(offset).cast::<u32>(),
                value.to_le(),
            )
        };
    }
}

/// 计算单个描述符槽在内存中占用的字节步长（非一致性模式固定隔离为 64 字节，一致性 DWMAC1000 采用紧凑 32 字节）。
const fn descriptor_stride(generation: Generation, coherent: bool) -> usize {
    match generation {
        Generation::Dwmac4 => ISOLATED_DESCRIPTOR_BYTES,
        Generation::Dwmac1000 if coherent => DWMAC1000_DESCRIPTOR_BYTES,
        Generation::Dwmac1000 => ISOLATED_DESCRIPTOR_BYTES,
    }
}

/// 计算相邻描述符槽之间的硬件跳过长度（DSL 字段编码值）：DWMAC4 单位为 8 字节 AXI beat，DWMAC1000 单位为 4 字节字。
const fn descriptor_skip(generation: Generation, coherent: bool) -> u32 {
    let stride = descriptor_stride(generation, coherent);
    match generation {
        Generation::Dwmac4 => ((stride - DWMAC4_DESCRIPTOR_BYTES) / JH7110_AXI_BEAT_BYTES) as u32,
        Generation::Dwmac1000 => {
            ((stride - DWMAC1000_DESCRIPTOR_BYTES) / DESCRIPTOR_WORD_BYTES) as u32
        }
    }
}

/// 返回包含 `OWN` 位与写回完成状态的 32 位控制字下标（DWMAC4 位于 `DES3` 即下标 3，DWMAC1000 位于 `DES0` 即下标 0）。
const fn status_word(generation: Generation) -> usize {
    match generation {
        Generation::Dwmac4 => 3,
        Generation::Dwmac1000 => 0,
    }
}

/// 判断描述符状态字中的 `OWN` 位是否仍置位（`true` 表示仍归属 DMA 引擎，CPU 不可访问其载荷页）。
pub(super) const fn device_owned(status: u32) -> bool {
    status & OWN != 0
}

/// 校验接收完成描述符的状态字；若为完整无错的单缓冲区数据帧且含 FCS 长度合法，则返回剥离 4 字节以太网 FCS 后的帧字节数。
pub(super) const fn received(generation: Generation, status: u32) -> Option<usize> {
    if matches!(generation, Generation::Dwmac4) && status & DWMAC4_RX_CONTEXT != 0 {
        return None;
    }

    let (first, last, length) = match generation {
        Generation::Dwmac4 => (
            DWMAC4_RX_FIRST,
            DWMAC4_RX_LAST,
            (status & DWMAC4_RX_LENGTH_MASK) as usize,
        ),
        Generation::Dwmac1000 => (
            DWMAC1000_RX_FIRST,
            DWMAC1000_RX_LAST,
            ((status >> DWMAC1000_RX_LENGTH_SHIFT) & DWMAC1000_RX_LENGTH_MASK) as usize,
        ),
    };
    if status & (OWN | ERROR_SUMMARY) != 0 || status & (first | last) != first | last {
        return None;
    }

    // 两代控制器均未开启硬件自动剥离 FCS，DMA 写回长度包含末尾 4 字节 CRC 校验和，需验证范围后予以扣除。
    if length < FRAME_MIN + ETHERNET_FCS_BYTES || FRAME_MAX + ETHERNET_FCS_BYTES < length {
        None
    } else {
        Some(length - ETHERNET_FCS_BYTES)
    }
}

/// 检查已由硬件归还（`OWN == 0`）的发送描述符状态字中是否置位了错误汇总标志（`ERROR_SUMMARY`）。
pub(super) const fn transmit_error(status: u32) -> bool {
    status & ERROR_SUMMARY != 0
}

/// 构造单缓冲区接收描述符的初始控制字数组（置位 `OWN` 与完成中断；DWMAC1000 末槽额外置位环回卷标志 `TER`）。
const fn rx_descriptor(
    generation: Generation,
    last: bool,
    address: u64,
) -> [u32; DWMAC1000_DESCRIPTOR_WORDS] {
    match generation {
        Generation::Dwmac4 => [
            address as u32,
            (address >> u32::BITS) as u32,
            0,
            OWN | DWMAC4_RX_INTERRUPT | DWMAC4_RX_BUFFER1_VALID,
            0,
            0,
            0,
            0,
        ],
        Generation::Dwmac1000 => [
            OWN,
            BUFFER_LEN as u32 | if last { DWMAC1000_RX_END_RING } else { 0 },
            address as u32,
            0,
            0,
            0,
            0,
            0,
        ],
    }
}

/// 构造单帧单缓冲区发送描述符的控制字数组（同时置位首包 `FS`、尾包 `LS`、完成中断及 `OWN`；DWMAC1000 末槽置位环回卷标志 `TER`）。
const fn tx_descriptor(
    generation: Generation,
    last: bool,
    address: u64,
    length: usize,
) -> [u32; DWMAC1000_DESCRIPTOR_WORDS] {
    match generation {
        Generation::Dwmac4 => [
            address as u32,
            (address >> u32::BITS) as u32,
            DWMAC4_TX_INTERRUPT | length as u32,
            OWN | DWMAC4_TX_FIRST | DWMAC4_TX_LAST | length as u32,
            0,
            0,
            0,
            0,
        ],
        Generation::Dwmac1000 => [
            OWN | DWMAC1000_TX_INTERRUPT
                | DWMAC1000_TX_LAST
                | DWMAC1000_TX_FIRST
                | if last { DWMAC1000_TX_END_RING } else { 0 },
            length as u32,
            address as u32,
            0,
            0,
            0,
            0,
            0,
        ],
    }
}

// 编译期静态断言：校验两代控制器的描述符步长、DSL 编码、末槽回卷位、OWN 位位置及含 FCS 帧长边界换算。
const _: () = {
    assert!(descriptor_skip(Generation::Dwmac4, false) == 6);
    assert!(descriptor_skip(Generation::Dwmac1000, false) == 8);
    assert!(descriptor_skip(Generation::Dwmac1000, true) == 0);
    assert!(descriptor_stride(Generation::Dwmac1000, true) == 32);
    assert!(rx_descriptor(Generation::Dwmac1000, true, 0)[1] == 2048 | (1 << 15));
    assert!(tx_descriptor(Generation::Dwmac1000, true, 0, 60)[0] & (1 << 21) != 0);
    assert!(device_owned(rx_descriptor(Generation::Dwmac4, false, 0)[3]));
    assert!(matches!(
        received(Generation::Dwmac4, (1 << 29) | (1 << 28) | 1518),
        Some(1514)
    ));
    assert!(matches!(
        received(Generation::Dwmac1000, (1 << 9) | (1 << 8) | (18 << 16)),
        Some(14)
    ));
    assert!(received(Generation::Dwmac4, (1 << 29) | (1 << 28) | 1519).is_none());
};
