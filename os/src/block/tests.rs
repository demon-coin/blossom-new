//! 块设备层受控完成顺序与 Virtqueue `used.idx` 观测辅助模块。
//!
//! # 模块职责与架构定位
//! 本模块为块设备层内核单元测试（`ktest`）提供两类底层辅助能力：
//! 1. 直接从共享一致性 DMA 环形缓冲区只读获取 Virtqueue 的 `used.idx` 硬件完成游标，供中断合并与门控测试判断硬件是否已提交完成项。
//! 2. 绕过真实磁盘 DMA 提交流程，在内存中受控驱动父 `Bio` 与子切片（child slice）的引用计数生命周期，专门用于检验“子切片先完成”与“请求拆分子任务（generator）最后退出”两种并发交错顺序下，父 `Bio` 完成回调与物理页 DMA 访问权交还的账本一致性。
//!
//! # 核心状态流转与生命周期
//! ```text
//!   Disk::admit(bio)
//!         |
//!         v
//!   ActiveGenerator (持有父 Bio 的强引用与生成期计数)
//!         |
//!         +---> bio.register_child(offset, len)   [增加子切片未完成计数]
//!         |
//!         +---> BioInner::finish_child(...)       [递减子切片计数，汇总部分完成字节]
//!         |
//!         v
//!   active.finish(None)                           [释放生成期引用；若为最后一个引用则触发父 Bio 完成]
//! ```
//!
//! # 锁获取顺序与并发契约
//! - [`used_index`] 获取 `disk.queue` 自旋锁，在持锁且保证设备未被停机释放的前提下执行 DMA 读屏障与 `read_volatile` 读取。
//! - [`generation_pair`] 与 [`generation_last`] 通过 `Disk::admit` 及 `BioInner` 内部原子状态与自旋锁完成记账，不直接嵌套持有外部锁。

use super::bio::BioInner;
use super::{Bio, Disk, DiskRef};

/// 读取指定块设备 Virtqueue 当前的硬件 `used.idx` 完成环索引。
///
/// 在持有 `disk.queue` 自旋锁期间执行 DMA 读内存屏障，并通过 `read_volatile` 直接读取一致性 DMA 映射区中的 `used.idx` 字段，不会消费完成环元素或推进驱动程序游标。
///
/// - `disk`：目标块设备实例（必须处于活跃未停机状态）。
/// - `legacy`：是否按 Legacy VirtIO 环形队列对齐布局计算 `used` 环偏移。
pub(crate) fn used_index(disk: &Disk, legacy: bool) -> u16 {
    let queue = disk.queue.lock();
    let device = queue.device.as_ref().expect("live controlled disk");
    let layout = virtio_drivers::queue::RingLayout::new(disk.limits.queue_depth, legacy)
        .expect("controlled real ring layout");
    let ring = device.test_virtio().control_regions().0;
    crate::arch::dma_read_barrier();
    // SAFETY:
    // 1. 当前持有 `disk.queue` 自旋锁，且已确认 `queue.device.is_some()`，保证底层 VirtIO 设备及其 DMA 控制区映射在读取期间不会被停机或释放。
    // 2. `layout.used_offset() + size_of::<u16>()` 精确定位到 Virtqueue `used` 环头部的 `idx` 字段（`u16` 自然对齐）。
    // 3. `ring.cpu` 为有效的一致性 DMA 虚拟地址映射，使用 `read_volatile` 仅做 2 字节读取，不构造跨越设备并发修改区的 Rust 引用。
    u16::from_le(unsafe {
        core::ptr::read_volatile(
            ring.cpu
                .as_ptr()
                .add(layout.used_offset() + core::mem::size_of::<u16>())
                .cast::<u16>(),
        )
    })
}

/// 依次准入两个 `Bio` 请求，在各自子切片完成后按顺序结束两个生成期句柄（`ActiveGenerator`）。
///
/// 用于测试两个并发父请求在“子切片先完成、生成期句柄后释放”的受控交错下，父请求完成状态与物理页访问权归还的正确性。
pub(crate) fn generation_pair(disk: &DiskRef, first: Bio, second: Bio) {
    let first = disk
        .admit(first)
        .unwrap_or_else(|(_, error)| panic!("controlled first admission: {:?}", error));
    let second = disk
        .admit(second)
        .unwrap_or_else(|(_, error)| panic!("controlled second admission: {:?}", error));
    for active in [&first, &second] {
        let length = active.bio.len();
        active.bio.register_child(0, length);
        BioInner::finish_child(&active.bio, 0, length, length, None);
    }
    first.finish(None);
    second.finish(None);
}

/// 准入单个 `Bio` 请求，先完成其唯一下属子切片，再释放生成期句柄（`ActiveGenerator`）以触发父请求最终完成。
///
/// 用于验证当子切片在生成期尚未结束前就已提前完成时，父 `Bio` 不会提前触发完成回调，而是严格等到 `active.finish(None)` 释放最后一个生成期引用后才收尾完成。
pub(crate) fn generation_last(disk: &DiskRef, bio: Bio) {
    let active = disk
        .admit(bio)
        .unwrap_or_else(|(_, error)| panic!("controlled generation admission: {:?}", error));
    let length = active.bio.len();
    active.bio.register_child(0, length);
    BioInner::finish_child(&active.bio, 0, length, length, None);
    active.finish(None);
}
