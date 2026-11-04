//! 只验证父完成账本的受控顺序，不冒充硬件反序 used 或磁盘 I/O。

use super::bio::BioInner;
use super::{Bio, Disk, DiskRef};

/// 只读真实设备 used.idx，固定布局与同一控制区 owner 保活，不消费或合成元素。
pub(crate) fn used_index(disk: &Disk, legacy: bool) -> u16 {
    let queue = disk.queue.lock();
    let device = queue.device.as_ref().expect("live controlled disk");
    let layout = virtio_drivers::queue::RingLayout::new(disk.limits.queue_depth, legacy)
        .expect("controlled real ring layout");
    let ring = device.test_virtio().control_regions().0;
    crate::arch::dma_read_barrier();
    // SAFETY: 同一 queue 锁防止停止与释放；used.idx 是真实布局的对齐 u16，
    // coherent 设备拥有该字节，此处仅 volatile 读取，不创建 DMA 区的 Rust 引用。
    u16::from_le(unsafe {
        core::ptr::read_volatile(
            ring.cpu
                .as_ptr()
                .add(layout.used_offset() + core::mem::size_of::<u16>())
                .cast::<u16>(),
        )
    })
}

/// 准入与生成尾部使用生产 API；原零页不发表给设备，只控制两个 child 的完成顺序。
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

/// 仅控制无设备引用的 child 先结束；生产准入与生成者最后完成仍使用真实实现。
pub(crate) fn generation_last(disk: &DiskRef, bio: Bio) {
    let active = disk
        .admit(bio)
        .unwrap_or_else(|(_, error)| panic!("controlled generation admission: {:?}", error));
    let length = active.bio.len();
    active.bio.register_child(0, length);
    BioInner::finish_child(&active.bio, 0, length, length, None);
    active.finish(None);
}
