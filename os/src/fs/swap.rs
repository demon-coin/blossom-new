//! 基于块设备连续物理块分配的页面交换区（Swap）分配器模块。
//!
//! # 模块职责与架构定位
//! 本模块管理固定容量的磁盘交换空间（默认 16 MiB），负责按 4 KiB 页面粒度分配、读写和回收交换槽位（Swap Slot）：
//! - **上游调用者**：虚拟内存换出/换入机制（当前作为未挂载至主构建树的独立交换区子模块保留），通过 [`SWAP_DEVICE`] 全局句柄写入页面数据并持有 [`SwapTracker`] 凭证。
//! - **下游依赖**：构造期间从底层文件系统块分配器预留固定数量的块编号，并在换入/换出时调用块设备接口按页读写连续扇区。
//!
//! # 核心设计与位图布局
//! 每个交换槽位（`swap_id`）对应 1 张 `PAGE_SIZE`（4 KiB）物理页，在底层映射为 `BLK_PER_PG`（8 个 512B 块）连续块编号：
//!
//! ```text
//!   swap_id = i * 64 + bit_offset
//!  ┌─────────────────────────────────────────────────────────────────────┐
//!  │ bitmap[i] (u64): 每一位表示 1 个 4 KiB 交换槽位是否已被占用 (1=占用) │
//!  └──────────────────────────────┬──────────────────────────────────────┘
//!                                 │ 映射为连续块区间
//!                                 ▼
//!  ┌─────────────────────────────────────────────────────────────────────┐
//!  │ block_ids[swap_id * BLK_PER_PG .. (swap_id + 1) * BLK_PER_PG]       │
//!  │ 共 BLK_PER_PG 个连续的底层块设备块号，供整页读写传输使用              │
//!  └─────────────────────────────────────────────────────────────────────┘
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 全局实例 [`SWAP_DEVICE`] 由 `spin::Mutex` 保护，串行化交换槽位的位图查找、块读写与槽位回收。
//! - [`SwapTracker`] 在最后一个 `Arc` 引用析构时会自动获取 `SWAP_DEVICE` 锁以归还槽位；因此持有 `SWAP_DEVICE` 锁期间不得触发最后一个 `Arc<SwapTracker>` 的析构，否则会导致同核自旋死锁。

use alloc::{sync::Arc, vec::Vec};
use spin::Mutex;

use crate::{config::PAGE_SIZE, drivers::BLOCK_DEVICE, hal::BLOCK_SZ};

use super::directory_tree::FILE_SYSTEM;
use lazy_static::*;

/// 默认预分配的交换空间容量（单位：MiB）。
const DEFAULT_SWAP_MIB: usize = 16;
/// 位图中单个 `u64` 字包含的槽位比特数（64 位）。
const BITMAP_WORD_BITS: usize = u64::BITS as usize;

lazy_static! {
    /// 全局块设备交换区实例，在首次访问时延迟初始化默认容量的交换空间并由自旋互斥锁保护。
    pub static ref SWAP_DEVICE: Mutex<Swap> = Mutex::new(Swap::new(DEFAULT_SWAP_MIB));
}

/// 已分配交换槽位的 RAII 所有权追踪器。
///
/// 通常封装在 `Arc<SwapTracker>` 中由换出页表项或匿名页元数据共享持有；
/// 当最后一个引用被释放时，其 `Drop` 实现会自动将内部保存的槽位编号（字段 `0`，单位：页槽索引）归还给 [`SWAP_DEVICE`]。
#[derive(Debug)]
pub struct SwapTracker(pub usize);

impl Drop for SwapTracker {
    /// 在最后一个 `Arc<SwapTracker>` 释放时，加锁并将关联的交换槽位标记为空闲。
    fn drop(&mut self) {
        SWAP_DEVICE.lock().discard(self.0);
    }
}

/// 块设备交换空间管理器。
///
/// 维护槽位占用位图与预分配的底层块编号映射表，按页粒度执行交换槽位的分配、数据换入换出及槽位回收。
pub struct Swap {
    /// 交换槽位占用位图，每个比特位对应 1 个 `PAGE_SIZE` 大小的交换页槽（`1` 表示已占用，`0` 表示空闲）。
    bitmap: Vec<u64>,
    /// 预分配的底层块设备块号数组，总长度为总交换页数乘以 `BLK_PER_PG`。
    block_ids: Vec<usize>,
}

/// 单个内存页（`PAGE_SIZE`）包含的底层磁盘块（`BLOCK_SZ`）数量。
const BLK_PER_PG: usize = PAGE_SIZE / BLOCK_SZ;
/// 1 MiB 对应的字节数（$1024 \times 1024$ 字节），用于将容量单位从 MiB 换算为页数与块数。
const SWAP_SIZE: usize = 1024 * 1024;

impl Swap {
    /// 创建指定容量（`size`，单位：MiB）的交换空间管理器，初始化空闲位图并向文件系统预留对应数量的磁盘块。
    pub fn new(size: usize) -> Self {
        // 按 1 MiB = 256 个 4 KiB 页换算总交换槽位数，并计算所需的 u64 位图字数。
        let bit = size * (SWAP_SIZE / PAGE_SIZE);
        let vec_len = bit / usize::MAX.count_ones() as usize;
        let mut bitmap = Vec::<u64>::with_capacity(vec_len);
        bitmap.resize(bitmap.capacity(), 0);
        // 按 1 MiB = 2048 个 512 B 块换算并预留底层磁盘块。
        let blocks = size * (SWAP_SIZE / BLOCK_SZ);
        Self {
            bitmap,
            block_ids: FILE_SYSTEM.alloc_blocks(blocks),
        }
    }

    /// 从连续磁盘块区间 `block_ids` 读取一整页数据到 `buf` 缓冲区。
    ///
    /// 调用前会断言验证这组块号在物理上严格连续，以便从起始块号发起整页读取。
    fn read_page(block_ids: &[usize], buf: &mut [u8]) {
        assert!(block_ids[0] + BLK_PER_PG - 1 == block_ids[BLK_PER_PG - 1]);
        BLOCK_DEVICE.read_block(block_ids[0], buf);
    }

    /// 将 `buf` 中的一整页数据写入连续磁盘块区间 `block_ids`。
    ///
    /// 调用前会断言验证这组块号在物理上严格连续，以便从起始块号发起整页写入。
    fn write_page(block_ids: &[usize], buf: &[u8]) {
        assert!(block_ids[0] + (BLK_PER_PG - 1) == block_ids[BLK_PER_PG - 1]);
        BLOCK_DEVICE.write_block(block_ids[0], buf);
    }

    /// 将第 `pos` 号交换槽位在位图中的对应比特位置为 `1`（标记为已占用）。
    fn set_bit(&mut self, pos: usize) {
        self.bitmap[pos / BITMAP_WORD_BITS] |= 1 << (pos % BITMAP_WORD_BITS);
    }

    /// 将第 `pos` 号交换槽位在位图中的对应比特位清为 `0`（标记为空闲）。
    fn clear_bit(&mut self, pos: usize) {
        self.bitmap[pos / BITMAP_WORD_BITS] &= !(1 << (pos % BITMAP_WORD_BITS));
    }

    /// 扫描位图查找第一个空闲的交换槽位编号（不会修改位图状态）；若空间已满则返回 `None`。
    fn alloc_page(&self) -> Option<usize> {
        for (i, bit) in self.bitmap.iter().enumerate() {
            if *bit == u64::MAX {
                // 当前 64 位字的所有槽位均已被占用，直接跳过。
                continue;
            }
            let free_bit = (!*bit).trailing_zeros() as usize;
            return Some(i * BITMAP_WORD_BITS + free_bit);
        }
        None
    }

    /// 返回第 `swap_id` 号交换槽位对应的 `BLK_PER_PG` 个底层磁盘块号切片。
    fn get_block_ids(&self, swap_id: usize) -> &[usize] {
        &self.block_ids[swap_id * BLK_PER_PG + 0..swap_id * BLK_PER_PG + BLK_PER_PG]
    }

    /// 将第 `swap_id` 号交换槽位中的数据读入 `buf` 页面缓冲区。
    pub fn read(&mut self, swap_id: usize, buf: &mut [u8]) {
        Self::read_page(self.get_block_ids(swap_id), buf);
    }

    /// 分配一个空闲交换槽位并将 `buf` 中的整页数据写入磁盘，返回管理该槽位生命周期的 `Arc<SwapTracker>`。
    ///
    /// 若交换空间已耗尽则直接触发 panic。
    pub fn write(&mut self, buf: &[u8]) -> Arc<SwapTracker> {
        if let Some(swap_id) = self.alloc_page() {
            Self::write_page(self.get_block_ids(swap_id), buf);
            self.set_bit(swap_id);
            Arc::new(SwapTracker(swap_id))
        } else {
            panic!("Swap space exhausted!");
        }
    }

    /// 释放第 `swap_id` 号交换槽位，清除其位图占用标志以供后续换出复用。
    #[inline(always)]
    pub fn discard(&mut self, swap_id: usize) {
        self.clear_bit(swap_id);
    }
}
