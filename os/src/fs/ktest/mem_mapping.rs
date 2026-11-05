//! RAM 文件与用户映射共同拥有协议的真实行为回归。

use crate::arch::UserAccess;
use crate::block::AccessMode;
use crate::fs::file::{open_path_with_flags, FileIoContext};
use crate::fs::ktest::path::PathOwner;
use crate::fs::ktest::*;
use crate::fs::memfs::{Filesystem, MemoryFs};
use crate::mm::space::{Mm, VmError};
use crate::mm::user_table::PagePermissions;

const ADDRESS: usize = 0x4a00_0000;
const PRIVATE_ADDRESS: usize = ADDRESS + 2 * crate::mm::PAGE_SIZE;

const RW: PagePermissions = PagePermissions {
    read: true,
    write: true,
    execute: false,
};

/// 通过本次 fixture 的真实 VFS 解析 RAM 挂载路径。
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

    /// 同一 MemoryFs 的共享页、私有 COW、unlink 和 truncate 均沿真实 inode 来源完成。
    #[test]
    fn shared_private_unlink_truncate_and_release(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
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

        let shared_pfn = mm.mapping(ADDRESS).map(|mapping| mapping.physical);
        let mut attempt = mm::reclaim::ReclaimAttempt::new();
        let page = checked(
            context,
            file.mapping_inode().unwrap().file_page(0, &mut attempt),
        )?;
        context.expect_eq(shared_pfn, Some(page.phys_addr()));
        let private_pfn = mm.mapping(PRIVATE_ADDRESS).map(|mapping| mapping.physical);
        context.expect_eq(shared_pfn != private_pfn, true);

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

/// 父 ktest 入口消费本模块唯一的静态 suite 槽。
pub(crate) fn anchors() -> [&'static ::ktest::SuiteSlot; 1] {
    [mapping::__ktest_anchor()]
}
