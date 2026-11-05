//! tmpfs 增长、额度与重配消费同源普通用户 C，不另建内核执行入口。

use super::*;
use crate::fs::memfs::{Filesystem, MemoryFs, TmpfsOptions};

const PROGRAM: &[u8] = b"/test/selftests/fs/tmpfs_growth";

#[ktest::suite(fixture = UserOwner, suite_resources = UserSuite, context = "Thread", tags("user", "user-tmpfs", "F09"), resources("root"), timeout_ms = 180000)]
pub(crate) mod tmpfs_growth {
    use super::*;

    /// C 在私有 /tmp 挂载并完成所选序列，TAP 输出收为注释，真实 wait/join 后回收资源。
    #[test]
    #[case("growth", b"growth".as_slice())]
    #[case("quota", b"quota".as_slice())]
    #[case("options", b"options".as_slice())]
    fn files_bytes_limits_and_reconfigure(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
        selection: &[u8],
    ) -> TestResult {
        fixture.prepare_tty(context, 4)?;
        let covered = checked(
            context,
            fixture
                .path
                .vfs()
                .resolve(fixture.path.context(), b"/tmp", ResolveMode::Follow),
        )?;
        let memory = checked(context, MemoryFs::try_tmpfs(b"mode=01777"))?;
        checked(
            context,
            fixture
                .path
                .vfs()
                .mount(&covered, Filesystem::Memory(memory), AccessMode::ReadWrite),
        )?;

        fixture.run_selftest_named(context, PROGRAM, &[selection], Credentials::ROOT)
    }

    /// 固定输入直接消费生产纯解析对象，覆盖全部单位与真实表示边界。
    #[test]
    fn parser_units_defaults_and_overflow(
        context: &mut TestContext<'_>,
        _fixture: &mut UserOwner,
    ) -> TestResult {
        let defaults = checked(context, TmpfsOptions::parse(b"", 1001))?;
        context.expect_eq(defaults.limits(1001), (500, 500));

        for (suffix, shift) in [
            (b'k', 10),
            (b'M', 20),
            (b'g', 30),
            (b'T', 40),
            (b'p', 50),
            (b'E', 60),
        ] {
            let mut input = *b"size=1k";
            input[6] = suffix;
            let options = checked(context, TmpfsOptions::parse(&input, 1001))?;
            context.expect_eq(
                options.blocks,
                Some(((1u64 << shift) as usize).div_ceil(mm::PAGE_SIZE)),
            );
        }

        let options = checked(
            context,
            TmpfsOptions::parse(
                b"size=1K,nr_blocks=0x10,size=25%,nr_inodes=010,mode=0700,mode=01777",
                1001,
            ),
        )?;
        context.expect_eq(options.blocks, Some(251));
        context.expect_eq(options.inodes, Some(8));
        context.expect_eq(options.mode, Some(0o1777));

        let options = checked(context, TmpfsOptions::parse(b"size=0,nr_inodes=0", 1001))?;
        context.expect_eq((options.blocks, options.inodes), (Some(0), Some(0)));

        context.expect_eq(
            TmpfsOptions::parse(b"size=16E", 1001),
            Err(FsError::InvalidOperation),
        );
        context.expect_eq(
            TmpfsOptions::parse(b"nr_blocks=0x8000000000000000", 1001),
            Err(FsError::InvalidOperation),
        );
        context.expect_eq(
            TmpfsOptions::parse(b"nr_inodes=0x40000000000000", 1001),
            Err(FsError::InvalidOperation),
        );
        Ok(())
    }
}
