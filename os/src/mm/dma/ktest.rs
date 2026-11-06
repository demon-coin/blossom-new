//! DMA 地址域（`DmaDomain`）、CPU 中转复制（CPU Copy）与物理页回写（Writeback）生命周期的内核单元测试。
//!
//! ## 模块职责与架构定位
//! 本模块位于 `mm::dma` 子系统下，负责对 DMA 地址映射窗口校验、缓存一致性控制区（`CoherentDma`）分配、
//! 以及流式负载页（`MappedPages`）与页缓存 `Folio` 之间的所有权流转进行端到端测试。
//! 上游由 `ktest` 测试运行器在内核引导后调用，下游直接验证 `mm::dma`、`mm::folio` 及底层伙伴分配器（Buddy Allocator）。
//!
//! ## 核心机制与状态流转
//!
//! ### 1. DMA 物理与总线地址窗口映射
//! ```text
//! 物理地址空间:     [phys_start ....................... phys_end)
//!                        |  + bus_offset (平移)
//!                        v
//! 总线地址空间:     [bus_start ....... mask_limit] (受 dma_mask 位宽截断限制)
//!                        |
//! 有效分配窗口:     [phys_start ....... min(phys_end, mask_limit - bus_offset + 1))
//! ```
//!
//! ### 2. `MappedPages` 与 `Folio` 回写状态机
//! ```text
//! [Folio Dirty]
//!      | begin_writeback()
//!      v
//! [Folio Writeback] ---> MappedPages::from_writeback()
//!                             |
//!                             +--> begin_cpu_access() (进入 CpuCopy 状态)
//!                             |      |-- cpu_copy_range() (读取回写快照，并发修改 Folio 会重标 Dirty)
//!                             |      +-- finish_cpu_access()
//!                             |
//!                             +--> accept_range() & complete_folio()
//!                                    |-- 清除本轮 Writeback 位
//!                                    +-- 保留回写期间新增的 Dirty 位
//! ```
//!
//! ## 锁获取顺序与并发契约
//! - 测试夹具 `DmaFixture` 为单线程独占上下文，不涉及多锁嵌套。
//! - 操作 `Folio` 内容时通过 `folio.lock_content()` 获取内容锁，并在调用 `MappedPages` 完成接口前释放，避免死锁。

use crate::mm;
use crate::mm::dma::{CoherentDma, DmaDomain, DmaError, MappedPages, PayloadAccess};
use crate::mm::{MemoryStats, PAGE_SIZE};
use ktest::{Fixture, TestContext, TestError, TestResult};
use virtio_drivers::BufferDirection;

#[ktest::suite(context = "Boot", tags("mm", "dma"), timeout_ms = 10000)]
mod domain {
    use super::*;

    /// 测试目标：验证带偏移转换的 DMA 地址域（`DmaDomain`）在掩码边界、对齐要求与越界地址上的检查逻辑。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：创建一个 16 位掩码（`0xffff`）、物理窗口 `[0x100000, 0x120000)`、总线起始偏移 `0x8000`、64 字节缓存行对齐的非一致性 `DmaDomain`。
    /// 2. When：分别映射窗口起点 `0x100000`、掩码上限前的最后一块 `0x107fc0`（长 64 字节，总线末字节恰好为 `0xffff`）、超出掩码的 `0x108000`，以及未对齐的起始地址/长度和低于窗口起点的地址。
    /// 3. Then：窗口内对齐范围成功转换为总线地址 `0x8000` 与 `0xffc0`；有效分配窗口收缩为 `(0x100000, 0x108000)`；超出掩码或窗口的映射返回 `DmaError::AddressLimit`，未按缓存行对齐的映射返回 `DmaError::Alignment`。
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

    /// 测试目标：验证 `DmaDomain` 构造函数的非法参数拦截，以及硬件缓存一致性（Coherent）模式下免除缓存行对齐但仍严格执行窗口边界检查。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：尝试使用非连续掩码 `0xafff`、非 2 的幂次缓存行大小 `48`、以及会导致总线地址加法溢出的 `u64::MAX` 偏移构造 `DmaDomain`。
    /// 2. When：随后构造一个启用 `coherent = true`、物理窗口 `[0x1000, 0x2000)`、总线偏移 `0x8000` 的有效域，并映射未对齐的 1 字节范围 `(0x1001, 1)`，以及跨出窗口末端、发生整数溢出或长度为 0 的范围。
    /// 3. Then：非法掩码与缓存行参数返回 `DmaError::InvalidDomain`，总线地址溢出返回 `DmaError::AddressLimit`；一致性域允许 1 字节非对齐映射，但越界、溢出和零长度映射均返回 `DmaError::AddressLimit`。
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

/// DMA 所有权与生命周期测试夹具，负责记录内存统计基线并在测试结束时自动回收 DMA 资源。
struct DmaFixture {
    /// 测试开始前排空每 CPU 缓存（PCP）后的系统内存六域统计基线（页粒度）。
    baseline: MemoryStats,
    /// 被测流式 DMA 负载页容器，在 `tear_down` 中自动结束 CPU 访问并释放。
    payload: Option<MappedPages>,
    /// 被测一致性 DMA 控制区容器，在 `tear_down` 中自动释放并归还底层物理页。
    control: Option<CoherentDma>,
    /// 被测页缓存 `Folio` 引用，在 `tear_down` 中丢弃私有临时内容并释放物理页。
    folio: Option<mm::FolioRef>,
}

impl Fixture for DmaFixture {
    /// 初始化测试夹具：排空所有 CPU 的本地页缓存（PCP），记录初始内存统计基线。
    fn set_up(_: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        mm::drain_all_pages();
        Ok(Self {
            baseline: mm::stats(),
            payload: None,
            control: None,
            folio: None,
        })
    }

    /// 清理测试夹具：结束未关闭的 CPU 访问期，释放负载页、控制区和临时 `Folio`，并校验内存统计完全恢复至基线。
    fn tear_down(mut self, context: &mut TestContext<'_>) -> TestResult {
        if let Some(payload) = self.payload.as_mut() {
            if payload.access == PayloadAccess::CpuCopy {
                // SAFETY:
                // 1. 前置检查 `payload.access == PayloadAccess::CpuCopy` 确认当前正处于已开启的 CPU 复制访问期；
                // 2. 测试用例中的所有同步复制计划（`CpuCopyPlan`）均已作为局部变量在测试函数返回前消费完毕，夹具未保留任何指向物理页内容的裸指针或切片别名。
                unsafe { payload.finish_cpu_access() };
            }
        }
        drop(self.payload.take());
        drop(self.control.take());
        if let Some(folio) = self.folio.take() {
            // SAFETY:
            // 1. 该 `folio` 是测试在内存中单独创建的临时对象，从未插入任何 Inode 的页缓存索引，也未映射给用户态进程；
            // 2. 上方 `drop(self.payload.take())` 已先行销毁关联的 `MappedPages` 并结束回写状态，没有任何并发读者或写者访问该物理页。
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

    /// 测试目标：验证 `MappedPages` 在 `DriverToDevice` 与 `DeviceToDriver` 两个方向下的 CPU 中转复制权限、越界拦截与所有权状态。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：分配 1 页清零的物理页，在页内偏移 64 字节处创建长度为 64 字节、指定方向 `direction` 的 `MappedPages`。
    /// 2. When：开启 CPU 访问期（`begin_cpu_access`），获取 `[0, 64)` 的复制计划，分别尝试与传输方向一致的复制和反向复制，再尝试获取越界的 `(1, 64)` 复制计划，最后结束 CPU 访问期。
    /// 3. Then：正向复制成功返回 `Ok(())`，反向复制和越界范围均返回 `Err(DmaError::InvalidRange)`；整个过程不向硬件设备移交所有权（`!payload.is_device_owned()`），且 `DeviceToDriver` 方向下物理页内容与源数组一致，`DriverToDevice` 方向下目标数组保持全零。
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
        // SAFETY:
        // 1. 底层物理页刚由 `alloc_pages(0, true)` 分配并清零，尚未交由任何硬件设备或外部线程访问；
        // 2. `fixture.payload` 独占持有该 `MappedPages` 实例，整个 CPU 访问期仅在当前测试线程内同步执行。
        unsafe { payload.begin_cpu_access() };
        // SAFETY:
        // 1. 上一行已调用 `begin_cpu_access()` 进入 `CpuCopy` 访问状态，且 `[0, 64)` 位于 `MappedPages` 有效长度（64 字节）范围内；
        // 2. 返回的 `plan` 仅在下方同步复制中使用，在调用 `finish_cpu_access()` 前即消费完毕。
        let plan = unsafe { payload.cpu_copy_range(0, 64) }.unwrap();
        // SAFETY:
        // 1. `source` 与 `target` 均为当前栈上长度为 64 字节的有效数组，与 `plan` 的 64 字节复制长度严格匹配；
        // 2. 栈上数组与新分配的 `pages` 物理页内存完全隔离、互不重叠，且复制期间无其他并发访问者。
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
        // SAFETY:
        // 1. `payload` 仍处于 `begin_cpu_access()` 开启的访问期内，且由 `fixture` 保活；
        // 2. 请求范围 `(1, 64)` 超出总长度 64 字节，预期立即返回 `Err(DmaError::InvalidRange)`，不会构造出越界的裸指针计划。
        let bounds = unsafe { payload.cpu_copy_range(1, 64) }.map(|_| ());
        // SAFETY:
        // 1. 上方构造的 `plan` 已在 `match` 块中按值消费完毕，`bounds` 因越界未产生有效计划；
        // 2. 此刻不存在任何存活的 `CpuCopyPlan` 或指向底层物理页的引用。
        unsafe { payload.finish_cpu_access() };

        assert_eq!(copied, Ok(()));
        assert_eq!(reverse, Err(DmaError::InvalidRange));
        assert_eq!(bounds, Err(DmaError::InvalidRange));
        assert_eq!(payload.range().physical, physical + 64);
        assert!(!payload.is_device_owned());
        if direction == BufferDirection::DeviceToDriver {
            // SAFETY:
            // 1. 在 `DeviceToDriver` 分支中，`plan.copy_from(source.as_ptr())` 已将全部 64 字节完整写入初始化；
            // 2. 上方已调用 `finish_cpu_access()` 结束复制期，且该页从未交给硬件设备，不存在并发修改。
            assert_eq!(unsafe { payload.bytes() }, source);
        } else {
            assert_eq!(target, [0u8; 64]);
        }
    }

    /// 测试目标：验证页缓存 `Folio` 在回写（Writeback）期间通过 CPU 中转读取数据时，`ToDevice` 同步不会误使 CPU 缓存失效，且回写期间的新写入会保留 `Dirty` 标记。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：分配物理页创建 `Folio`，完成整页初始读取后，向首扇区 `[0, 512)` 写入 `0x31` 并启动回写（`begin_writeback`），将其封装为 `MappedPages`。
    /// 2. When：开启 `payload` 的 CPU 访问期，随后在回写完成前再次锁定 `Folio` 内容并将首扇区改写为 `0x72`（触发重新标脏 Redirty）；接着通过 `cpu_copy_range` 拷出 512 字节，执行 `ToDevice` 方向的 `sync_for_cpu`，结束 CPU 访问期并调用 `complete_folio` 完成回写。
    /// 3. Then：拷出的内容为最新写入的 `0x72`（未被 `ToDevice` 方向的 `sync_for_cpu` 错误失效），回写扇区计数清零（`writeback_sectors() == 0`），而并发修改产生的脏扇区计数被正确保留（`dirty_sectors() == 1`）。
    #[test]
    fn writeback_redirty(_: &mut TestContext<'_>, fixture: &mut DmaFixture) {
        // Step 1: 分配物理页构造 Folio，并将整页初始化为已读完成状态。
        let pages = mm::alloc_pages(0, true).unwrap();
        let folio = mm::FolioRef::from_pages(pages).ok().unwrap();
        fixture.folio = Some(folio.clone());
        let mut read = folio.begin_read(0, PAGE_SIZE).unwrap();
        // SAFETY:
        // 1. 底层物理页已由 `alloc_pages(0, true)` 清零初始化；
        // 2. 该 `folio` 尚未加入全局缓存索引，无其他访问者，直接标记整页读取完成是安全的。
        unsafe {
            read.complete(crate::block::IoResult {
                completed: PAGE_SIZE,
                error: None,
            })
        };

        // Step 2: 锁定 Folio 内容，向首扇区写入初始数据 0x31 并开启回写（Writeback），封装为 MappedPages。
        let wb = {
            let mut content = folio.lock_content();
            content.copy_from(0, &[0x31u8; 512]).unwrap();
            // SAFETY:
            // 1. 当前持有 `folio.lock_content()` 内容锁，且该 `folio` 从未映射至任何用户态页表，排除了并发修改；
            // 2. 前一行 `content.copy_from(0, &[0x31u8; 512])` 已将 `[0, 512)` 范围初始化并标脏。
            unsafe { content.begin_writeback(0, 512) }.unwrap().unwrap()
        };
        fixture.payload = Some(MappedPages::from_writeback(wb).ok().unwrap());
        let payload = fixture.payload.as_mut().unwrap();
        // Step 3: 开启 CPU 复制访问期，并在回写完成前重新锁定 Folio 将首扇区改写为 0x72（触发 Redirty）。
        // SAFETY:
        // 1. `payload` 刚从独占的回写凭证 `wb` 构造，尚未提交给任何硬件设备；
        // 2. 回写凭证保证底层 `Folio` 物理页在回写完成前始终有效。
        unsafe { payload.begin_cpu_access() };
        folio.lock_content().copy_from(0, &[0x72u8; 512]).unwrap();
        let mut output = [0u8; 512];
        // SAFETY:
        // 1. `payload` 处于 `begin_cpu_access()` 开启的访问期内，且对 `folio` 的重写入已在前一行释放内容锁后完成；
        // 2. `output` 是当前栈上 512 字节的独立缓冲区，与 `Folio` 物理页不重叠，复制过程无并发竞争。
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
        // Step 4: 执行 ToDevice 方向的 CPU 同步（验证不误失效缓存），随后关闭 CPU 访问期并完成 Folio 回写。
        // SAFETY:
        // 1. `[physical, physical + 512)` 属于当前测试持有的 `Folio` 物理页有效范围，且已初始化；
        // 2. 方向为 `DmaDirection::ToDevice`，按架构约定不会执行丢弃 CPU 脏缓存行的 Invalidate 操作，不会破坏前文刚写入的数据。
        unsafe { domain.sync_for_cpu(physical, 512, crate::mm::dma::DmaDirection::ToDevice) }
            .unwrap();
        // SAFETY:
        // 1. 前文的临时 `CpuCopyPlan` 已在 `copy_to` 调用后立即销毁；
        // 2. 此后不再有任何通过 `payload` 访问物理页的复制计划。
        unsafe { payload.finish_cpu_access() };
        payload.accept_range(0, 512);
        // SAFETY:
        // 1. CPU 访问期已由 `finish_cpu_access()` 正式关闭，且该 `payload` 从未映射给硬件设备；
        // 2. 前一行 `accept_range(0, 512)` 已登记全部 512 字节处理完毕，与 `completed: 512` 严格一致。
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

    /// 测试目标：验证 `CoherentDma` 控制区按最小 2 的幂次阶数（Order）向上取整分配，严格遵守 `DmaDomain` 物理窗口限制，并拒绝在内核保留区内分配。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：先分配并释放一个 Order 1（2 页）的物理块以获取一段已知可用的连续物理地址 `[physical, physical + 2 * PAGE_SIZE)`，用该范围构造一致性 `DmaDomain`。
    /// 2. When：请求分配 `PAGE_SIZE + 1` 字节的 `CoherentDma` 控制区；随后再用内核镜像所在的保留物理区间构造另一个 `DmaDomain` 并请求分配 1 页控制区。
    /// 3. Then：首次分配向上取整到 Order 1（`2 * PAGE_SIZE` 字节），设备地址与 CPU 虚拟地址均精确落在该窗口起点；在内核保留区窗口上的分配因无可用空闲页返回 `Err(DmaError::NoMemory)`。
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
