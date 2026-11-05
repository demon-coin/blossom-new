//! 两代描述符采用各自硬件格式；每个非一致槽独占完整 cacheline。
//! 本地重写分代 ring 与缓存同步，许可与修改声明见本目录 LICENSE。

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

/// 描述符内容没有 Rust 引用；原页由外层设备拥有到确认停止。
pub(super) struct Ring {
    memory: CoherentDma,
    generation: Generation,
    stride: usize,
    coherent: bool,
}

impl Ring {
    /// 未发表时申请固定控制页，增强格式和缓存隔离分别决定真实步长。
    pub(super) fn new(
        domain: DmaDomain,
        generation: Generation,
        coherent: bool,
    ) -> Result<Self, GmacError> {
        let stride = descriptor_stride(generation, coherent);
        let memory = CoherentDma::allocate(domain, RING_LEN * stride)?;
        // SAFETY: 完整控制页刚清零且从未发表，初始 TX OWN=0 必须在引擎首次读取前可见；
        // 此范围由本拥有者独占，长度为完整隔离槽的整数倍。
        unsafe { memory.sync_for_device(0, RING_LEN * stride, DmaDirection::Bidirectional)? };
        Ok(Self {
            memory,
            generation,
            stride,
            coherent,
        })
    }

    /// 返回 DMA 基址加实际槽步长，不把 CPU 地址当设备地址。
    pub(super) fn address(&self, index: usize) -> u64 {
        assert!(index <= RING_LEN);
        self.memory.region().device.value() + (index * self.stride) as u64
    }

    /// DWMAC4 DSL 以 JH7110 的八字节 AXI beat 计，旧代 DSL 以四字节字计。
    pub(super) fn skip_length(&self) -> u32 {
        descriptor_skip(self.generation, self.coherent)
    }

    /// 只观察设备写回状态，缓存行在交权后保持干净，绝不写回 CPU 修改。
    /// OWN 仍置位时不收权；该只读快照不借用设备并发写入的内存。
    pub(super) fn status(&self, index: usize) -> Result<u32, GmacError> {
        assert!(index < RING_LEN);
        let offset = index * self.stride;
        if !self.coherent {
            // 此行上轮交权前已 clean，之后 CPU 只有 volatile load，没有脏行；
            // flush 不会用旧 CPU 内容覆盖 DMA，隔离槽不会触及邻居的 CPU 修改。
            let physical = arch::virt_to_phys(self.memory.region().cpu.as_ptr() as usize);
            arch::dma_cache_flush(physical + offset, self.stride).map_err(|error| {
                GmacError::Dma(crate::mm::dma::DmaError::CacheMaintenance(error))
            })?;
        }
        arch::dma_read_barrier();
        Ok(self.read(index, status_word(self.generation)))
    }

    /// 仅在本槽尚未发表或真实 OWN 清除后准备一条 RX，最后一个字发表 OWN。
    pub(super) fn arm_rx(&self, index: usize, address: u64) -> Result<(), GmacError> {
        let words = rx_descriptor(self.generation, index + 1 == RING_LEN, address);
        self.publish(index, words)
    }

    /// 仅在 CPU-owned 槽编码完整帧地址和长度，TX 中断请求随同一描述符发表。
    pub(super) fn arm_tx(
        &self,
        index: usize,
        address: u64,
        length: usize,
    ) -> Result<(), GmacError> {
        let words = tx_descriptor(self.generation, index + 1 == RING_LEN, address, length);
        self.publish(index, words)
    }

    /// 在本槽设备访问已结束后写完整控制字，再同步单独的行。
    fn publish(
        &self,
        index: usize,
        words: [u32; DWMAC1000_DESCRIPTOR_WORDS],
    ) -> Result<(), GmacError> {
        assert!(index < RING_LEN);
        let status = status_word(self.generation);
        for word in 0..self.stride / DESCRIPTOR_WORD_BYTES {
            if word != status {
                self.write(index, word, words.get(word).copied().unwrap_or(0));
            }
        }
        arch::dma_write_barrier();
        self.write(index, status, words[status]);
        // SAFETY: 调用者只传 CPU-owned 槽，完整行没有设备并发访问；数据与控制字
        // 已全部写入，唯一原页仍由 Ring 保活，其他槽不共享此缓存行。
        unsafe {
            self.memory.sync_for_device(
                index * self.stride,
                self.stride,
                DmaDirection::Bidirectional,
            )?
        };
        Ok(())
    }

    /// volatile 只读一个对齐控制字，不形成覆盖异步 DMA 的共享引用。
    fn read(&self, index: usize, word: usize) -> u32 {
        let offset = index * self.stride + word * DESCRIPTOR_WORD_BYTES;
        assert!(offset + DESCRIPTOR_WORD_BYTES <= RING_LEN * self.stride);
        // SAFETY: 固定控制页覆盖完整环，偏移对齐且已检查；拥有者不在设备停止前归页。
        u32::from_le(unsafe {
            ptr::read_volatile(self.memory.region().cpu.as_ptr().add(offset).cast::<u32>())
        })
    }

    /// CPU-owned 槽的裸写由外层队列状态串行，设备仍拥有的邻居不受影响。
    fn write(&self, index: usize, word: usize, value: u32) {
        let offset = index * self.stride + word * DESCRIPTOR_WORD_BYTES;
        assert!(offset + DESCRIPTOR_WORD_BYTES <= RING_LEN * self.stride);
        // SAFETY: 原页永久直映、偏移四字节对齐；私有调用者只修改未交权的独立槽。
        unsafe {
            ptr::write_volatile(
                self.memory.region().cpu.as_ptr().add(offset).cast::<u32>(),
                value.to_le(),
            )
        };
    }
}

/// 硬件格式长度与 cacheline 隔离分别决定槽长。
const fn descriptor_stride(generation: Generation, coherent: bool) -> usize {
    match generation {
        Generation::Dwmac4 => ISOLATED_DESCRIPTOR_BYTES,
        Generation::Dwmac1000 if coherent => DWMAC1000_DESCRIPTOR_BYTES,
        Generation::Dwmac1000 => ISOLATED_DESCRIPTOR_BYTES,
    }
}

/// 两代 DSL 的单位不同，不使用公共字节到位字段的隐式换算。
const fn descriptor_skip(generation: Generation, coherent: bool) -> u32 {
    let stride = descriptor_stride(generation, coherent);
    match generation {
        Generation::Dwmac4 => ((stride - DWMAC4_DESCRIPTOR_BYTES) / JH7110_AXI_BEAT_BYTES) as u32,
        Generation::Dwmac1000 => {
            ((stride - DWMAC1000_DESCRIPTOR_BYTES) / DESCRIPTOR_WORD_BYTES) as u32
        }
    }
}

/// 新代写回在第四字，旧代增强描述符写回在第一字。
const fn status_word(generation: Generation) -> usize {
    match generation {
        Generation::Dwmac4 => 3,
        Generation::Dwmac1000 => 0,
    }
}

/// OWN 清除是本槽及其单独 payload 的真实完成边界。
pub(super) const fn device_owned(status: u32) -> bool {
    status & OWN != 0
}

/// 验证硬件包含 FCS 的帧长，返回不含 FCS 的 Ethernet 长度。
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

    // 两代均关闭自动 FCS stripping，所有长度都按同一实际 DMA 字节范围解释。
    if length < FRAME_MIN + ETHERNET_FCS_BYTES || FRAME_MAX + ETHERNET_FCS_BYTES < length {
        None
    } else {
        Some(length - ETHERNET_FCS_BYTES)
    }
}

/// TX 错误摘要只在 OWN 已清除后解释，不用写回长度计算发送字节。
pub(super) const fn transmit_error(status: u32) -> bool {
    status & ERROR_SUMMARY != 0
}

/// RX 每帧仅使用 buffer1，末槽的增强 ring-end 位保留在旧代第二字。
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

/// 单 buffer 的首尾标记和 IOC 按各代格式编码，不混用增强描述符的 TER 位。
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

// 固定 ABI 检查随编译执行，覆盖两代步长、OWN、增强末槽和 FCS 长度边界。
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
