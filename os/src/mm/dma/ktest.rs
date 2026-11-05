//! 设备窗口、CPU 复制方向和原页回收的具名验收。

use crate::mm;
use crate::mm::dma::{CoherentDma, DmaDomain, DmaError, MappedPages, PayloadAccess};
use crate::mm::{MemoryStats, PAGE_SIZE};
use ktest::{Fixture, TestContext, TestError, TestResult};
use virtio_drivers::BufferDirection;

#[ktest::suite(context = "Boot", tags("mm", "dma"), timeout_ms = 10000)]
mod domain {
    use super::*;

    /// mask 检查含最后字节，非恒等总线地址与物理窗口共同限制原 buddy 选择。
    #[test]
    fn translated_mask_boundary() {
        let domain = DmaDomain::new(0xffff, 0x100000, 0x120000, 0x8000, 64, false).unwrap();
        assert_eq!(domain.map(0x100000, 64).unwrap().value(), 0x8000);
        assert_eq!(domain.map(0x107fc0, 64).unwrap().value(), 0xffc0);
        assert_eq!(domain.map(0x108000, 64), Err(DmaError::AddressLimit));
        assert_eq!(domain.allocation_window(), (0x100000, 0x108000));
        assert_eq!(domain.map(0x100001, 64), Err(DmaError::Alignment));
        assert_eq!(domain.map(0x100000, 63), Err(DmaError::Alignment));
        assert_eq!(domain.map(0x0fffc0, 64), Err(DmaError::AddressLimit));
    }

    /// coherent 不强制 cacheline 隔离，真实窗口末端和无效域仍保持错误。
    #[test]
    fn domain_rejection() {
        assert_eq!(
            DmaDomain::new(0xafff, 0, PAGE_SIZE, 0, 64, false),
            Err(DmaError::InvalidDomain)
        );
        assert_eq!(
            DmaDomain::new(0xffff, 0, PAGE_SIZE, 0, 48, false),
            Err(DmaError::InvalidDomain)
        );
        assert_eq!(
            DmaDomain::new(u64::MAX, 0, PAGE_SIZE, u64::MAX, 64, true),
            Err(DmaError::AddressLimit)
        );

        let domain = DmaDomain::new(0xffff, 0x1000, 0x2000, 0x8000, 64, true).unwrap();
        assert_eq!(domain.map(0x1001, 1).unwrap().value(), 0x8001);
        assert_eq!(domain.map(0x1fff, 2), Err(DmaError::AddressLimit));
        assert_eq!(domain.map(usize::MAX, 2), Err(DmaError::AddressLimit));
        assert_eq!(domain.map(0x1000, 0), Err(DmaError::AddressLimit));
    }
}

struct DmaFixture {
    baseline: MemoryStats,
    payload: Option<MappedPages>,
    control: Option<CoherentDma>,
    folio: Option<mm::FolioRef>,
}

impl Fixture for DmaFixture {
    /// 用同一真实页池建立回收基线，不建立设备或替代分配器。
    fn set_up(_: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        mm::drain_all_pages();
        Ok(Self {
            baseline: mm::stats(),
            payload: None,
            control: None,
            folio: None,
        })
    }

    /// 结束 CPU 访问并失效未发表的私有 scratch folio，归还后核同一实例的六域页账。
    fn tear_down(mut self, context: &mut TestContext<'_>) -> TestResult {
        if let Some(payload) = self.payload.as_mut() {
            if payload.access == PayloadAccess::CpuCopy {
                // SAFETY: 用例的同步复制已返回，fixture 不保留任何内容裸别名。
                unsafe { payload.finish_cpu_access() };
            }
        }
        drop(self.payload.take());
        drop(self.control.take());
        if let Some(folio) = self.folio.take() {
            // SAFETY: 此 scratch 从未进入 inode/缓存索引或用户映射，无持久化目标；
            // 原 payload 已析构并结束 WB，正文同步复制已返回，没有其他内容使用者。
            unsafe { folio.lock_content().discard() };
        }
        mm::drain_all_pages();
        context.expect_eq(mm::stats(), self.baseline);
        Ok(())
    }
}

#[ktest::suite(fixture = DmaFixture, tags("mm", "dma"), timeout_ms = 10000)]
mod ownership {
    use super::*;

    /// CPU 复制保原物理页，反向访问和越界不写数据；完成不创建设备映射。
    #[test]
    #[case("to_device", BufferDirection::DriverToDevice)]
    #[case("from_device", BufferDirection::DeviceToDriver)]
    fn cpu_copy(_: &mut TestContext<'_>, fixture: &mut DmaFixture, direction: BufferDirection) {
        let pages = mm::alloc_pages(0, true).unwrap();
        let physical = pages.phys_addr();
        fixture.payload = Some(MappedPages::new(pages, 64, 64, direction).unwrap());
        let payload = fixture.payload.as_mut().unwrap();
        let source = [0x5au8; 64];
        let mut target = [0u8; 64];
        // SAFETY: 原页已清零，没有其他访问者；本测试只有同步 CPU 复制且保活 owner。
        unsafe { payload.begin_cpu_access() };
        // SAFETY: 本测试同步保活原 owner 和 CPU 复制期，所有计划均在收权前消费。
        let plan = unsafe { payload.cpu_copy_range(0, 64) }.unwrap();
        // SAFETY: 两个栈数组均完整有效且不与原页重叠，整个复制期只由本测试访问。
        let (copied, reverse) = unsafe {
            match direction {
                BufferDirection::DriverToDevice => (
                    plan.copy_to(target.as_mut_ptr()),
                    plan.copy_from(source.as_ptr()),
                ),
                BufferDirection::DeviceToDriver => (
                    plan.copy_from(source.as_ptr()),
                    plan.copy_to(target.as_mut_ptr()),
                ),
            }
        };
        // SAFETY: 原 owner 和访问期仍由本测试保活；越界导出失败不产生裸计划。
        let bounds = unsafe { payload.cpu_copy_range(1, 64) }.map(|_| ());
        // SAFETY: 上述所有同步调用已返回，未导出其他访问者。
        unsafe { payload.finish_cpu_access() };

        assert_eq!(copied, Ok(()));
        assert_eq!(reverse, Err(DmaError::InvalidRange));
        assert_eq!(bounds, Err(DmaError::InvalidRange));
        assert_eq!(payload.range().physical, physical + 64);
        assert!(!payload.is_device_owned());
        if direction == BufferDirection::DeviceToDriver {
            // SAFETY: 完整复制已经初始化原范围，复制访问已结束且没有设备写入。
            assert_eq!(unsafe { payload.bytes() }, source);
        } else {
            assert_eq!(target, [0u8; 64]);
        }
    }

    /// CPU bounce 读取原 WB，不在 ToDevice 完成时 invalidate，接受成功仍保留新 Dirty。
    #[test]
    fn writeback_redirty(_: &mut TestContext<'_>, fixture: &mut DmaFixture) {
        let pages = mm::alloc_pages(0, true).unwrap();
        let folio = mm::FolioRef::from_pages(pages).ok().unwrap();
        fixture.folio = Some(folio.clone());
        let mut read = folio.begin_read(0, PAGE_SIZE).unwrap();
        // SAFETY: 整页由 alloc_pages 清零，尚未发表或借出，初始化访问已经结束。
        unsafe {
            read.complete(crate::block::IoResult {
                completed: PAGE_SIZE,
                error: None,
            })
        };

        let wb = {
            let mut content = folio.lock_content();
            content.copy_from(0, &[0x31u8; 512]).unwrap();
            // SAFETY: 本 folio 未建立用户映射，内容锁排除冲突内核修改，范围已初始化。
            unsafe { content.begin_writeback(0, 512) }.unwrap().unwrap()
        };
        fixture.payload = Some(MappedPages::from_writeback(wb).ok().unwrap());
        let payload = fixture.payload.as_mut().unwrap();
        // SAFETY: 原 WB 只供本测试 CPU 复制，凭证保活原页，未发表给硬件。
        unsafe { payload.begin_cpu_access() };
        folio.lock_content().copy_from(0, &[0x72u8; 512]).unwrap();
        let mut output = [0u8; 512];
        // SAFETY: 测试保活原页及复制期，目标完整有效且与原页不重叠；无并发复制者。
        unsafe {
            payload
                .cpu_copy_range(0, 512)
                .unwrap()
                .copy_to(output.as_mut_ptr())
        }
        .unwrap();
        let physical = payload.range().physical;
        let domain = DmaDomain::new(
            u64::MAX,
            physical,
            physical + PAGE_SIZE,
            physical as u64,
            64,
            false,
        )
        .unwrap();
        // SAFETY: 本例从未将原页交设备，全部硬件访问为空；本来源是已初始化 ToDevice
        // WB，CPU 新写已完成，测试只检查完成同步不错误调用 invalidate。
        unsafe { domain.sync_for_cpu(physical, 512, crate::mm::dma::DmaDirection::ToDevice) }
            .unwrap();
        // SAFETY: 同步复制已返回，无计划会在此后继续访问原页。
        unsafe { payload.finish_cpu_access() };
        payload.accept_range(0, 512);
        // SAFETY: 该模型的唯一 CPU 复制已经结束，接受账真实覆盖完整本轮范围。
        unsafe {
            payload.complete_folio(crate::block::IoResult {
                completed: 512,
                error: None,
            })
        };

        assert_eq!(output, [0x72u8; 512]);
        assert_eq!(folio.writeback_sectors(), 0);
        assert_eq!(folio.dirty_sectors(), 1);
    }

    /// 真实控制申请保持最小整阶，受限窗口拒绝保留区且正常析构归还原页。
    #[test]
    fn constrained_control(_: &mut TestContext<'_>, fixture: &mut DmaFixture) {
        let pages = mm::alloc_pages(1, true).unwrap();
        let physical = pages.phys_addr();
        drop(pages);
        let domain = DmaDomain::new(
            u64::MAX,
            physical,
            physical + 2 * PAGE_SIZE,
            physical as u64,
            64,
            true,
        )
        .unwrap();
        fixture.control = Some(CoherentDma::allocate(domain, PAGE_SIZE + 1).unwrap());
        let control = fixture.control.as_ref().unwrap();
        assert_eq!(control.order(), 1);
        assert_eq!(control.byte_len(), 2 * PAGE_SIZE);
        assert_eq!(control.region().device.value(), physical as u64);
        assert_eq!(
            control.region().cpu.as_ptr() as usize,
            crate::arch::phys_to_virt(physical)
        );

        let kernel = crate::boot::kernel_region();
        let reserved = DmaDomain::new(
            u64::MAX,
            kernel.start,
            kernel.end,
            kernel.start as u64,
            64,
            true,
        )
        .unwrap();
        assert!(matches!(
            CoherentDma::allocate(reserved, PAGE_SIZE),
            Err(DmaError::NoMemory)
        ));
    }
}
