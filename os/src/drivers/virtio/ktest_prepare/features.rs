//! 现代真实 transport 拒绝遗漏必需 ACCESS_PLATFORM 的组合；不模拟状态读回。

use crate::drivers::virtio::{VIRTIO_PCI_BLOCK_LEGACY, VIRTIO_PCI_BLOCK_MODERN, VIRTIO_VENDOR_ID};
use crate::drivers::DeviceInventory;
use crate::{irq, mm, time, timer};
use ::ktest::{Fixture, SuiteSlot, TestContext, TestError, TestResult};
use virtio_drivers::transport::{DeviceStatus, SomeTransport, Transport};
use virtio_drivers::Error;

struct Features {
    baseline: mm::MemoryStats,
    metadata: crate::workqueue::testing::MetadataStats,
    inventory: DeviceInventory,
    transport: SomeTransport<'static>,
}

impl Fixture for Features {
    /// 声明的原盘只有一个 transport；legacy 外部协议没有 FEATURES_OK，必须使用现代资源。
    fn set_up(_: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        let baseline = super::settled()?;
        let metadata = crate::workqueue::testing::metadata_stats();
        let inventory = super::inventory()?;
        let transport = super::transport(&inventory)?;
        if transport.requires_legacy_layout() {
            return Err(TestError("hardware feature rejection requires modern MMIO (--modern) or PCI; legacy is covered by controls_before_driver_ok"));
        }
        Ok(Self {
            baseline,
            metadata,
            inventory,
            transport,
        })
    }

    /// 即使主体失败也在真正 status 零后才销毁同一 transport。
    fn quiesce(&mut self, _: &mut TestContext<'_>) -> TestResult {
        reset_transport(&mut self.transport)
    }

    /// 原硬件停止后释放本次库存，检查原六域与真实元数据归还。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        let baseline = self.baseline;
        let metadata = self.metadata;
        drop(self);
        context.expect_eq(super::settled()?, baseline);
        context.expect_eq(crate::workqueue::testing::metadata_stats(), metadata);
        Ok(())
    }
}

/// 不发表 DMA 的 transport 仍用真实读回确认 reset；错误保留 fixture 的唯一 owner。
fn reset_transport(transport: &mut SomeTransport<'static>) -> TestResult {
    transport.request_reset();
    let deadline = time::now()
        .checked_add(time::Duration::from_secs(2))
        .ok_or(TestError("feature reset deadline overflow"))?;
    while !transport.reset_complete() {
        if deadline <= time::now() {
            return Err(TestError("feature transport reset unconfirmed"));
        }
        timer::sleep(time::Duration::from_millis(1));
    }
    Ok(())
}

#[::ktest::suite(fixture = Features, isolation = "Case", tags("block", "prepare", "B03", "modern", "features-rejected"), resources("root", "virtio-access-platform"), timeout_ms = 15000)]
mod hardware {
    use super::*;
    /// 实际 MMIO/PCI 拒绝遗漏 ACCESS_PLATFORM 的组合，再确认同候选合法特性与 DRIVER_OK。
    #[test]
    fn modern_features_rejected(
        context: &mut TestContext<'_>,
        fixture: &mut Features,
    ) -> TestResult {
        let interrupt = if let Some(candidate) = fixture.inventory.mmio_blocks.first() {
            candidate.interrupt
        } else {
            fixture
                .inventory
                .pci
                .devices()
                .iter()
                .find(|device| {
                    device.vendor_id == VIRTIO_VENDOR_ID
                        && matches!(
                            device.device_id,
                            VIRTIO_PCI_BLOCK_LEGACY | VIRTIO_PCI_BLOCK_MODERN
                        )
                })
                .and_then(|device| device.interrupt)
                .ok_or(TestError("feature candidate IRQ missing"))?
        };
        let actions = irq::test_action_count(interrupt);
        let transport = &mut fixture.transport;
        reset_transport(transport)?;
        context.expect_eq(transport.queue_used(0), false);
        let offered = transport.read_device_features();
        let version_one = 1u64 << 32;
        let access_platform = 1u64 << 33;
        if offered & (version_one | access_platform) != version_one | access_platform {
            return Err(TestError("real feature rejection requires --iommu-platform and modern MMIO (--modern) or PCI"));
        }
        transport.set_status(DeviceStatus::ACKNOWLEDGE);
        transport.set_status(DeviceStatus::ACKNOWLEDGE | DeviceStatus::DRIVER);
        // 设备真实要求 ACCESS_PLATFORM；保留 VERSION_1 才进入设备的特性验证路径。
        // 本例不创建 queue 或发表 DMA 地址，因此无需启用任何 IOMMU 数据通路。
        transport.write_driver_features(version_one);
        transport.set_status(
            DeviceStatus::ACKNOWLEDGE | DeviceStatus::DRIVER | DeviceStatus::FEATURES_OK,
        );
        let rejected = transport.get_status();
        context.expect_eq(rejected.contains(DeviceStatus::FEATURES_OK), false);
        context.expect_eq(rejected.contains(DeviceStatus::DRIVER_OK), false);
        context.expect_eq(transport.queue_used(0), false);
        context.expect_eq(transport.finish_init(), Err(Error::FeaturesRejected));
        context.expect_eq(
            transport.get_status().contains(DeviceStatus::DRIVER_OK),
            false,
        );
        context.expect_eq(irq::test_action_count(interrupt), actions);
        context.diagnostic(format_args!("feature-rejection scope=actual-modern-registers offered={:#x} driver-features={:#x} live-status={:#x} queue-used={}", offered, version_one, rejected.bits(), transport.queue_used(0)));
        reset_transport(transport)?;
        let accepted = transport
            .begin_init(access_platform)
            .map_err(|_| TestError("same candidate legal feature negotiation did not recover"))?;
        context.expect_eq(accepted, version_one | access_platform);
        context.expect_eq(
            transport.get_status().contains(DeviceStatus::FEATURES_OK),
            true,
        );
        context.expect_eq(
            transport.get_status().contains(DeviceStatus::DRIVER_OK),
            false,
        );
        context.expect_eq(transport.queue_used(0), false);
        context.expect_eq(irq::test_action_count(interrupt), actions);
        transport
            .finish_init()
            .map_err(|_| TestError("same candidate DRIVER_OK handshake did not recover"))?;
        context.expect_eq(
            transport
                .get_status()
                .contains(DeviceStatus::DRIVER_OK | DeviceStatus::FEATURES_OK),
            true,
        );
        context.expect_eq(transport.queue_used(0), false);
        Ok(())
    }
}

/// 返回本叶实际 suite 注册槽。
pub(super) fn anchor() -> &'static SuiteSlot {
    hardware::__ktest_anchor()
}
