//! VirtIO 现代传输层（Modern MMIO / PCI）特性协商拒绝与恢复回归测试。
//!
//! # 模块职责与架构定位
//! 本模块作为 `drivers::virtio::ktest_prepare` 的子模块，专门针对 VirtIO 1.0+（Modern）
//! 传输层在启用平台访问隔离（`VIRTIO_F_ACCESS_PLATFORM`）时的硬件特性协商状态机进行测试。
//! 它直接调用 `virtio_drivers::transport::Transport` 接口操作设备寄存器，而不构造上层队列或块设备对象。
//!
//! # 核心状态机：VirtIO Modern 特性协商与恢复流程
//! ```text
//!   [Reset: status = 0]
//!           │
//!           ▼
//!   ACKNOWLEDGE | DRIVER
//!           │
//!           ├──► 1. 仅写入 VERSION_1（故意遗漏 ACCESS_PLATFORM）
//!           │         │
//!           │         ▼
//!           │    尝试置位 FEATURES_OK
//!           │         │
//!           │         ▼
//!           │    设备硬件拒绝（读回 status 不含 FEATURES_OK，finish_init 返回 FeaturesRejected）
//!           │
//!           └──► 2. 重新 Reset 后写入 VERSION_1 | ACCESS_PLATFORM
//!                     │
//!                     ▼
//!                置位并读回确认 FEATURES_OK
//!                     │
//!                     ▼
//!                finish_init() 置位 DRIVER_OK（协商成功，队列仍保持未启用状态）
//! ```
//!
//! # 并发与硬件独占契约
//! - 本测试套件采用 `isolation = "Case"` 串行隔离模式运行，由测试夹具 `Features` 独占唯一的测试块设备传输层句柄。
//! - 测试全程不分配 Virtqueue 描述符环、不向设备发布任何 DMA 物理地址，也不注册中断处理函数。

use crate::drivers::virtio::{VIRTIO_PCI_BLOCK_LEGACY, VIRTIO_PCI_BLOCK_MODERN, VIRTIO_VENDOR_ID};
use crate::drivers::DeviceInventory;
use crate::{irq, mm, time, timer};
use ::ktest::{Fixture, SuiteSlot, TestContext, TestError, TestResult};
use virtio_drivers::transport::{DeviceStatus, SomeTransport, Transport};
use virtio_drivers::Error;

/// VirtIO 现代传输层特性协商测试夹具。
struct Features {
    /// 测试开始前采集的系统物理内存与分配器六维统计基线，用于拆卸阶段校验内存零泄漏。
    baseline: mm::MemoryStats,
    /// 测试开始前采集的工作队列元数据统计基线，用于校验测试未遗留后台工作项。
    metadata: crate::workqueue::testing::MetadataStats,
    /// 硬件探测得到的设备清单，提供测试目标设备的 MMIO 窗口或 PCI 配置空间及中断号。
    inventory: DeviceInventory,
    /// 被测设备对应的底层 VirtIO 传输层实例（Modern MMIO 或 PCI）。
    transport: SomeTransport<'static>,
}

impl Fixture for Features {
    /// 初始化特性协商测试夹具：采样资源基线、构造独占传输层实例，并校验其支持 Modern 协议。
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

    /// 静止化夹具：无论测试主体成功或失败，均在销毁传输层前将硬件状态寄存器重置为 0。
    fn quiesce(&mut self, _: &mut TestContext<'_>) -> TestResult {
        reset_transport(&mut self.transport)
    }

    /// 拆卸夹具：释放传输层与设备清单后，断言物理内存六维统计与工作队列元数据恢复至初始基线。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        let baseline = self.baseline;
        let metadata = self.metadata;
        drop(self);
        context.expect_eq(super::settled()?, baseline);
        context.expect_eq(crate::workqueue::testing::metadata_stats(), metadata);
        Ok(())
    }
}

/// 向 VirtIO 传输层发起硬件复位并轮询等待状态寄存器归零，超时则返回错误并保留夹具对硬件的独占权。
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
    /// 验证 Modern VirtIO 传输层在遗漏 `VIRTIO_F_ACCESS_PLATFORM` 时的硬件拒绝行为及同设备复位后的正常协商恢复。
    ///
    /// # 测试目标
    /// 确认当宿主设备要求 `VIRTIO_F_ACCESS_PLATFORM`（第 33 位）时，若驱动仅确认 `VIRTIO_F_VERSION_1`（第 32 位），
    /// 硬件会清除 `FEATURES_OK` 状态位并使 `finish_init()` 返回 `Error::FeaturesRejected`，且不会误触发中断或启用队列；
    /// 随后对同一设备执行硬件复位并补齐 `ACCESS_PLATFORM` 特性位后，能够正常完成 `FEATURES_OK` 与 `DRIVER_OK` 握手。
    ///
    /// # 场景步骤（Given-When-Then）
    /// 1. **Given**：获取独占的 Modern VirtIO 传输层句柄，执行硬件复位，确认队列 0 未启用且设备提供了 `VERSION_1 | ACCESS_PLATFORM` 特性位。
    /// 2. **When**：依次置位 `ACKNOWLEDGE` 与 `DRIVER`，仅向驱动特性寄存器写入 `VERSION_1`，随后尝试置位 `FEATURES_OK` 并调用 `finish_init()`。
    /// 3. **Then**：从硬件寄存器读回的状态字不包含 `FEATURES_OK` 与 `DRIVER_OK`，`finish_init()` 返回 `Err(Error::FeaturesRejected)`，队列 0 仍未启用，中断触发计数不变。
    /// 4. **When**：再次复位同一传输层，调用 `begin_init(access_platform)` 协商完整特性并调用 `finish_init()`。
    /// 5. **Then**：返回协商结果包含 `VERSION_1 | ACCESS_PLATFORM`，状态寄存器先后置位 `FEATURES_OK` 与 `DRIVER_OK`，队列 0 保持未启用且无中断产生。
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

        // Step 1: 记录初始中断计数并复位传输层，校验设备提供的特性位包含 VERSION_1 与 ACCESS_PLATFORM。
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

        // Step 2: 仅写入 VERSION_1 而不写入设备要求的 ACCESS_PLATFORM，验证设备拒绝置位 FEATURES_OK。
        transport.set_status(DeviceStatus::ACKNOWLEDGE);
        transport.set_status(DeviceStatus::ACKNOWLEDGE | DeviceStatus::DRIVER);
        // 保留 VERSION_1 以进入 VirtIO 1.0 特性校验路径；本测试不创建 Virtqueue 也不发布 DMA 地址，不涉及数据面传输。
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

        // Step 3: 复位同一传输层后携带 ACCESS_PLATFORM 重新协商，验证 FEATURES_OK 与 DRIVER_OK 正常恢复。
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

/// 返回本子模块的 `ktest` 测试套件注册槽引用。
pub(super) fn anchor() -> &'static SuiteSlot {
    hardware::__ktest_anchor()
}
