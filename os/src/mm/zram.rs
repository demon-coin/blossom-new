//! 基于内存 LZ4 压缩的虚拟块存储设备（ZRAM）。
//!
//! ## 模块职责与架构定位
//! 本模块在内存中维护一个固定槽位数的压缩数据池，将写入的字节切片（如换出页数据）
//! 通过 `lz4_flex` 压缩后保存在内核堆向量中，并返回一个基于 `Arc` 引用计数的槽位句柄
//! （[`ZramTracker`]）。当上层不再持有该句柄时，`ZramTracker::drop` 会自动释放对应槽位。
//!
//! ## 核心设计与槽位分配机制
//! `Zram` 使用定长槽位数组 `compressed` 配合单调水位游标 `tail` 与空闲回收栈 `recycled` 管理槽位 ID：
//!
//! ```text
//! compressed: [ Some(Vec<u8>), None (已回收), Some(Vec<u8>), None (未触及), ... ]
//!                                ^                             ^
//!                                |                             |
//!                    recycled: [ 1 ]                     tail: 3 (首次分配高水位)
//! ```
//!
//! - **分配槽位（`insert`）**：优先从 `recycled` 栈顶弹出已回收的槽位 ID；若 `recycled` 为空，
//!   则推进 `tail` 游标分配新槽位；若 `tail == capacity` 则返回 [`ZramError::NoSpace`]。
//! - **释放槽位（`remove` / `discard`）**：清空 `compressed[zram_id]`；若释放的恰好是 `tail - 1`
//!   处槽位则直接回退 `tail`，否则将该 ID 压入 `recycled` 栈供后续复用。
//!
//! ## 锁获取顺序与并发契约
//! - 全局实例 [`ZRAM_DEVICE`] 由 `spin::Mutex<Zram>` 保护；
//! - **严禁重入死锁**：`ZramTracker::drop` 内部会获取 `ZRAM_DEVICE.lock()`，因此调用方绝对不能
//!   在持有 `ZRAM_DEVICE` 锁期间丢弃最后一个 `Arc<ZramTracker>`。

use alloc::{sync::Arc, vec::Vec};
use lazy_static::lazy_static;
use lz4_flex::{compress_prepend_size, decompress_size_prepended};
use spin::Mutex;

/// 全局 ZRAM 设备默认支持的最大压缩槽位数量（2048 个槽位）。
const ZRAM_SLOTS: usize = 2048;

#[derive(Debug)]
/// ZRAM 操作可能返回的错误类型。
pub enum ZramError {
    /// 传入的槽位索引超出了 `compressed` 容器的总容量范围。
    InvalidIndex,
    /// 所有槽位均已被占用（`recycled` 为空且 `tail` 已达容量上限），无法分配新槽位。
    NoSpace,
    /// 目标槽位当前处于空闲状态，尚未写入压缩数据或已被释放。
    NotAllocated,
}

#[derive(Debug)]
/// ZRAM 槽位生命周期跟踪句柄（RAII 守卫）。
///
/// 内部元组字段 `0` 保存所占用的 ZRAM 槽位索引（`zram_id`）；当外层 `Arc<ZramTracker>`
/// 的最后一个强引用被释放（Drop）时，自动调用全局 `ZRAM_DEVICE` 释放该槽位。
pub struct ZramTracker(pub usize);

impl Drop for ZramTracker {
    /// 当最后一个引用释放时，自动获取全局 `ZRAM_DEVICE` 锁并回收 `self.0` 对应的压缩槽位。
    fn drop(&mut self) {
        ZRAM_DEVICE.lock().discard(self.0).unwrap();
    }
}

/// ZRAM 内存压缩存储核心控制器。
pub struct Zram {
    /// 按槽位索引存储压缩后字节流的定长数组；`None` 表示该槽位当前空闲。
    compressed: Vec<Option<Vec<u8>>>,
    /// 已被释放但低于 `tail` 水位的空闲槽位索引回收栈，优先用于后续分配。
    recycled: Vec<u16>,
    /// 尚未分配过的连续槽位起始游标（即历史首次分配的高水位线）。
    tail: u16,
}

impl Zram {
    /// 创建一个具有指定槽位容量 `capacity` 的 `Zram` 实例，所有槽位初始为空闲（`None`）。
    pub fn new(capacity: usize) -> Self {
        // 预分配指定容量的槽位向量并全部初始化为 None。
        let mut compressed = Vec::with_capacity(capacity);
        compressed.resize(compressed.capacity(), None);
        Self {
            compressed,
            recycled: Vec::new(),
            tail: 0,
        }
    }

    /// 将已压缩的数据块 `data` 存入一个空闲槽位，并返回管理该槽位生命周期的 `Arc<ZramTracker>`。
    fn insert(&mut self, data: Vec<u8>) -> Result<Arc<ZramTracker>, ZramError> {
        // Step 1: 优先从回收栈中获取空闲槽位 ID；若回收栈为空，则尝试推进 tail 游标分配新槽位。
        let zram_id = match self.recycled.pop() {
            Some(zram_id) => zram_id as usize,
            None => {
                if self.tail as usize == self.compressed.len() {
                    return Err(ZramError::NoSpace);
                } else {
                    self.tail += 1;
                    (self.tail - 1) as usize
                }
            }
        };

        // Step 2: 写入压缩数据并封装返回 RAII 槽位跟踪句柄。
        self.compressed[zram_id] = Some(data);
        Ok(Arc::new(ZramTracker(zram_id)))
    }

    /// 获取指定槽位 `zram_id` 中保存的压缩数据切片引用。
    fn get(&self, zram_id: usize) -> Result<&Vec<u8>, ZramError> {
        // 检查槽位索引是否越界。
        if zram_id >= self.compressed.len() {
            return Err(ZramError::InvalidIndex);
        }

        match &self.compressed[zram_id] {
            Some(compressed_data) => Ok(compressed_data),
            None => Err(ZramError::NotAllocated),
        }
    }

    /// 移除并返回指定槽位 `zram_id` 中的压缩数据，同时将该槽位索引归还至 `tail` 或 `recycled` 回收栈。
    fn remove(&mut self, zram_id: usize) -> Result<Vec<u8>, ZramError> {
        // Step 1: 检查槽位索引是否越界。
        if zram_id >= self.compressed.len() {
            return Err(ZramError::InvalidIndex);
        }

        // Step 2: 若释放的恰好是 tail 紧邻的前一个槽位，则直接回退 tail；否则压入 recycled 回收栈。
        if zram_id == (self.tail - 1) as usize {
            self.tail = zram_id as u16;
        } else {
            self.recycled.push(zram_id as u16);
        }
        // Step 3: 取出槽位中的压缩数据。
        match self.compressed[zram_id].take() {
            Some(compressed_data) => Ok(compressed_data),
            None => Err(ZramError::NotAllocated),
        }
    }

    /// 读取并解压指定槽位 `zram_id` 中的数据，将解压后的原始字节复制到调用方缓冲区 `buf` 中。
    pub fn read(&mut self, zram_id: usize, buf: &mut [u8]) -> Result<(), ZramError> {
        match self.get(zram_id) {
            Ok(compressed_data) => {
                // 使用 lz4_flex 解压带有前置原始长度头部的压缩数据，并拷贝到目标缓冲区。
                let decompressed_data =
                    decompress_size_prepended(compressed_data.as_slice()).unwrap();
                buf.copy_from_slice(decompressed_data.as_slice());
                Ok(())
            }
            Err(error) => Err(error),
        }
    }

    /// 将输入缓冲区 `buf` 中的数据通过 LZ4 压缩后存入空闲槽位，并返回槽位跟踪句柄 `Arc<ZramTracker>`。
    pub fn write(&mut self, buf: &[u8]) -> Result<Arc<ZramTracker>, ZramError> {
        // Step 1: 使用 LZ4 压缩数据（前置未压缩大小）并收缩多余的堆容量。
        let mut compressed = compress_prepend_size(buf);
        compressed.shrink_to_fit();
        log::trace!("[zram] compressed len: {}", compressed.len());
        // Step 2: 分配槽位并写入压缩结果。
        self.insert(compressed)
    }

    #[inline(always)]
    /// 丢弃并释放指定槽位 `zram_id` 中的压缩数据。
    pub fn discard(&mut self, zram_id: usize) -> Result<(), ZramError> {
        match self.remove(zram_id) {
            Ok(_) => Ok(()),
            Err(error) => Err(error),
        }
    }
}

lazy_static! {
    /// 全局默认 ZRAM 设备实例（容量为 [`ZRAM_SLOTS`] 个槽位），由自旋互斥锁保护。
    pub static ref ZRAM_DEVICE: Arc<Mutex<Zram>> = Arc::new(Mutex::new(Zram::new(ZRAM_SLOTS)));
}
