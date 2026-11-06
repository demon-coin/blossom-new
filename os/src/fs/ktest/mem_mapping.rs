//! 内存文件系统（`MemoryFs`）与用户虚拟内存映射（`Mm`）协同生命周期的内核测试套件。
//!
//! # 模块职责与架构定位
//! 本模块位于文件系统测试子系统（`fs::ktest`）中，处于虚拟文件系统（VFS）、内存文件系统（`MemoryFs`/`tmpfs`）
//! 与虚拟内存空间管理（`mm::space::Mm`）的交叉边界。上游由 `fs::ktest` 测试注册表统一挂载驱动，
//! 下游调用 `Vfs`、`MemoryFs`、`File` 以及 `Mm` 的映射、缺页异常处理与用户态拷贝接口。
//!
//! # 核心设计与共享/私有页面生命周期
//! 测试围绕同一个 `MemoryFs` 文件的页面所有权、写时复制（COW）、截断（`truncate`）与删除（`unlink`）展开：
//!
//! ```text
//!                        +-------------------------------+
//!                        |   MemoryFs Inode (AddressSpace)|
//!                        |     文件首页物理页帧 PFN_0     |
//!                        +---------------+---------------+
//!                                        |
//!                +-----------------------+-----------------------+
//!                | 共享映射 (MAP_SHARED)                         | 私有映射 (MAP_PRIVATE)
//!                v                                               v
//!   +----------------------------+                 +----------------------------+
//!   | 虚拟地址 ADDRESS           |                 | 虚拟地址 PRIVATE_ADDRESS   |
//!   | 直接映射 PFN_0 (读写同步)  |                 | 写缺页触发 COW 分配 PFN_1  |
//!   +----------------------------+                 +----------------------------+
//!
//! 生命周期与页面释放顺序：
//!   1. unlink("/plain/ram") -> 目录项移除，但 File、Mm 映射及外部 Folio 引用仍在，页面数为 1
//!   2. file.truncate(0)     -> 文件大小归零，映射区再次访问触发 BeyondEof；因外部仍持 Folio 引用，页面数为 1
//!   3. drop(file) + drop(mm) + vfs.prune_unused() -> VFS 与地址空间引用释放，仅剩外部 Folio 引用
//!   4. drop(page)           -> 最后一个强引用释放，MemoryFs 计费页面数归零
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 测试在独立内核线程上下文中顺序执行，通过 `PathOwner` 夹具管理底层块设备、VFS 实例与挂载点生命周期。
//! - 路径解析、文件读写与虚拟内存缺页均遵循各自子系统的内部锁顺序，测试层不额外持有跨子系统自旋锁。

use crate::arch::UserAccess;
use crate::block::AccessMode;
use crate::fs::file::{open_path_with_flags, FileIoContext};
use crate::fs::ktest::path::PathOwner;
use crate::fs::ktest::*;
use crate::fs::memfs::{Filesystem, MemoryFs};
use crate::mm::space::{Mm, VmError};
use crate::mm::user_table::PagePermissions;

/// 共享文件映射（`MAP_SHARED`）使用的测试用户虚拟地址起点（字节）。
const ADDRESS: usize = 0x4a00_0000;
/// 私有写时复制映射（`MAP_PRIVATE`）使用的测试用户虚拟地址起点（字节，与共享映射相隔 2 页）。
const PRIVATE_ADDRESS: usize = ADDRESS + 2 * crate::mm::PAGE_SIZE;

/// 测试映射使用的可读、可写、不可执行页面权限配置。
const RW: PagePermissions = PagePermissions {
    read: true,
    write: true,
    execute: false,
};

/// 在 `PathOwner` 夹具维护的 VFS 上下文中跟随符号链接解析路径 `name`，失败时记录诊断信息并转换为测试错误。
fn resolve(context: &TestContext<'_>, fixture: &PathOwner, name: &[u8]) -> Result<Path, TestError> {
    fixture
        .vfs()
        .resolve(fixture.context(), name, ResolveMode::Follow)
        .map_err(|error| {
            context.diagnostic(format_args!("memory-mapping resolve error={error:?}"));
            TestError("memory-mapping path resolution failed")
        })
}

#[ktest::suite(
    fixture = PathOwner,
    suite_resources = crate::drivers::DeviceInventory,
    context = "Thread",
    tags("T30", "fs-memory-mapping"),
    resources("root"),
    timeout_ms = 30000
)]
pub(crate) mod mapping {
    use super::*;

    /// 验证 `MemoryFs` 文件在共享映射、私有写时复制（COW）映射、文件截断（`truncate`）、目录项删除（`unlink`）及引用释放全流程中的物理页共享与回收行为。
    ///
    /// # 测试目标
    /// - 验证共享映射（`MAP_SHARED`）与底层文件页表缓存共享同一物理页帧（PFN），用户态写入直接反映到文件读取结果。
    /// - 验证私有映射（`MAP_PRIVATE`）在写缺页时分配独立的私有物理页帧，不会增加 `MemoryFs` 的文件页计费，也不会污染原文件内容。
    /// - 验证文件缩小与扩大的 `truncate` 操作会收回共享映射的页表写权限以同步脏状态，并在截断至 0 后拒绝越界缺页（`VmError::BeyondEof`）。
    /// - 验证 `unlink`、关闭文件描述符、销毁 `Mm` 与释放最后一个外部 `Folio` 强引用后，`MemoryFs` 的已用页数精确归零且挂载点可正常卸载。
    #[test]
    fn shared_private_unlink_truncate_and_release(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        // Step 1: 准备只读根 VFS，并在 `/plain` 目录下挂载可写的 `tmpfs` 内存文件系统。
        fixture.prepare_path(context, AccessMode::ReadOnly, 4)?;
        let covered = resolve(context, fixture, b"/plain")?;
        let memory = checked(
            context,
            MemoryFs::try_tmpfs(b"mode=0777,nr_inodes=128,nr_blocks=4096"),
        )?;
        checked(
            context,
            fixture.vfs().mount(
                &covered,
                Filesystem::Memory(memory.clone()),
                AccessMode::ReadWrite,
            ),
        )?;

        // Step 2: 在 `tmpfs` 中创建并打开 `/plain/ram` 文件，写入初始 8 字节内容 `"original"`。
        let file = checked(
            context,
            fixture.vfs().open_with(
                fixture.context(),
                fixture.context().cwd(),
                b"/plain/ram",
                (0, 0),
                Some((0o666, 0)),
                true,
                false,
                |path, _| open_path_with_flags(fixture.vfs(), path, 2, FileIoContext::Kernel),
            ),
        )?;
        checked(context, file.write(b"original"))?;

        // Step 3: 创建用户地址空间 `Mm`，在 `ADDRESS` 处建立共享读写映射，触发写缺页并通过 `copyout` 修改首字节为 `'s'`，验证文件读取同步变为 `"sriginal"`。
        let mm = Mm::try_new().map_err(|error| {
            context.diagnostic(format_args!("memory-mapping MM error={error:?}"));
            TestError("memory-mapping MM construction failed")
        })?;
        mm.map_file(
            ADDRESS,
            crate::mm::PAGE_SIZE,
            RW,
            RW,
            file.clone(),
            0,
            false,
        )
        .map_err(|error| {
            context.diagnostic(format_args!("memory-mapping shared map error={error:?}"));
            TestError("memory-mapping shared map failed")
        })?;
        mm.fault(ADDRESS, UserAccess::Write).map_err(|error| {
            context.diagnostic(format_args!("memory-mapping shared fault error={error:?}"));
            TestError("memory-mapping shared fault failed")
        })?;
        context.expect_eq(mm.copyout(ADDRESS, b"s").copied, 1);
        let mut bytes = [0; 8];
        checked(context, file.read_at(0, &mut bytes))?;
        context.expect_eq(&bytes, b"sriginal");

        // Step 4: 在 `PRIVATE_ADDRESS` 处建立私有读写映射，触发写缺页并通过 `copyout` 修改首字节为 `'p'`，验证私有映射读出 `"priginal"`，而 `MemoryFs` 计费页数仍为 1 且文件内容保持 `"sriginal"`。
        mm.map_file(
            PRIVATE_ADDRESS,
            crate::mm::PAGE_SIZE,
            RW,
            RW,
            file.clone(),
            0,
            true,
        )
        .map_err(|error| {
            context.diagnostic(format_args!("memory-mapping private map error={error:?}"));
            TestError("memory-mapping private map failed")
        })?;
        mm.fault(PRIVATE_ADDRESS, UserAccess::Write)
            .map_err(|error| {
                context.diagnostic(format_args!("memory-mapping private fault error={error:?}"));
                TestError("memory-mapping private fault failed")
            })?;
        context.expect_eq(mm.copyout(PRIVATE_ADDRESS, b"p").copied, 1);
        let mut private_bytes = [0; 8];
        context.expect_eq(mm.copyin(&mut private_bytes, PRIVATE_ADDRESS).copied, 8);
        context.expect_eq(&private_bytes, b"priginal");
        context.expect_eq(memory.used_pages(), 1);
        let mut unchanged = [0; 8];
        checked(context, file.read_at(0, &mut unchanged))?;
        context.expect_eq(&unchanged, b"sriginal");

        // Step 5: 对比物理页帧地址（PFN），验证共享映射与文件 `AddressSpace` 首页物理地址相同，而私有映射拥有不同的物理页帧。
        let shared_pfn = mm.mapping(ADDRESS).map(|mapping| mapping.physical);
        let mut attempt = mm::reclaim::ReclaimAttempt::new();
        let page = checked(
            context,
            file.mapping_inode().unwrap().file_page(0, &mut attempt),
        )?;
        context.expect_eq(shared_pfn, Some(page.phys_addr()));
        let private_pfn = mm.mapping(PRIVATE_ADDRESS).map(|mapping| mapping.physical);
        context.expect_eq(shared_pfn != private_pfn, true);

        // Step 6: 将文件截断至 3 字节再扩展至 8 字节，验证 `truncate` 会清除共享映射的页表写权限，后续写入重新触发写权限恢复，被截断区间清零且不影响私有映射。
        checked(context, file.truncate(3))?;
        context.expect_eq(mm.mapping(ADDRESS).unwrap().permissions.write, false);
        context.expect_eq(mm.copyout(ADDRESS + 5, b"x").copied, 1);
        context.expect_eq(mm.mapping(ADDRESS).unwrap().permissions.write, true);
        checked(context, file.truncate(8))?;
        context.expect_eq(mm.mapping(ADDRESS).unwrap().permissions.write, false);
        let mut grown = [0xa7; 8];
        context.expect_eq(mm.copyin(&mut grown, ADDRESS).copied, grown.len());
        context.expect_eq(&grown, b"sri\0\0\0\0\0");
        context.expect_eq(mm.copyin(&mut private_bytes, PRIVATE_ADDRESS).copied, 8);
        context.expect_eq(&private_bytes, b"priginal");

        // Step 7: 从目录中 `unlink` 删除 `/plain/ram` 并将文件截断为 0，验证此时访问映射区返回 `VmError::BeyondEof`，但在外部 `page` 引用释放前内存页不会提前回收。
        checked(
            context,
            fixture.vfs().unlink_from(
                fixture.context(),
                fixture.context().cwd(),
                b"/plain/ram",
                (0, 0),
            ),
        )?;
        context.expect_eq(memory.used_pages(), 1);
        let mut unlinked = [0; 8];
        checked(context, file.read_at(0, &mut unlinked))?;
        context.expect_eq(&unlinked, b"sri\0\0\0\0\0");
        checked(context, file.truncate(0))?;
        context.expect_eq(mm.fault(ADDRESS, UserAccess::Read), Err(VmError::BeyondEof));
        context.expect_eq(
            mm.fault(PRIVATE_ADDRESS, UserAccess::Read),
            Err(VmError::BeyondEof),
        );
        context.expect_eq(memory.used_pages(), 1);

        // Step 8: 依次释放 `File`、`Mm`、清理 VFS 缓存并最终丢弃 `page` 强引用，验证在最后一个强引用释放时 `MemoryFs` 已用页数归零，随后卸载 `tmpfs`。
        drop(file);
        drop(mm);
        fixture.vfs().prune_unused();
        context.expect_eq(memory.used_pages(), 1);
        drop(page);
        context.expect_eq(memory.used_pages(), 0);
        checked(context, fixture.vfs().unmount(&covered))?;
        drop(covered);
        context.expect_eq(memory.used_pages(), 0);
        Ok(())
    }
}

/// 返回本模块唯一的静态测试套件槽位，供上层 `fs::ktest` 聚合挂载。
pub(crate) fn anchors() -> [&'static ::ktest::SuiteSlot; 1] {
    [mapping::__ktest_anchor()]
}
