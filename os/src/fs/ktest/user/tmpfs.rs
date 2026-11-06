//! `tmpfs` 内存文件系统动态增长、配额限制与挂载选项解析的内核单元测试（ktest）。
//!
//! # 模块职责与架构定位
//! 本模块位于 `os/src/fs/ktest/user/tmpfs.rs`，负责从两个层面验证 `tmpfs`（`MemoryFs`）的容量管理与挂载参数语义：
//! 1. **用户态端到端集成测试**：在虚拟文件系统（VFS）的 `/tmp` 挂载点挂载可读写的 `MemoryFs`，随后运行用户态 C 测试程序 `/test/selftests/fs/tmpfs_growth`，验证文件写入增长、容量/Inode 配额拦截以及 `remount` 重配置；
//! 2. **挂载选项解析器单元测试**：直接调用 `TmpfsOptions::parse`，验证单位后缀（`k`/`M`/`g`/`T`/`p`/`E`）、百分比、八进制/十六进制数值、重复项覆盖策略以及整数溢出边界。
//!
//! # 核心设计与选项解析规则
//! `TmpfsOptions` 支持以下挂载参数组合与单位换算关系：
//!
//! ```text
//! 挂载选项字符串 (例如: "size=25%,nr_inodes=010,mode=01777")
//!   │
//!   ├──► 容量上限 (blocks):
//!   │      - "size=<N>[k|M|g|T|p|E]" ──► 字节数向上取整为页数: div_ceil(bytes, PAGE_SIZE)
//!   │      - "size=<P>%"             ──► 按总可用页数的百分比向上取整
//!   │      - "nr_blocks=<N>"         ──► 直接指定最大物理页数（支持十进制/0x十六进制/0八进制）
//!   │      * 注：若同时出现多个 size / nr_blocks，后出现的选项覆盖先前的设置
//!   │
//!   ├──► 节点上限 (inodes):
//!   │      - "nr_inodes=<N>"         ──► 直接指定最大 Inode 数量
//!   │      - 默认值（未指定时）      ──► 总物理页数的一半 (total_pages / 2)
//!   │
//!   └──► 根目录权限 (mode):
//!          - "mode=<octal>"          ──► 根目录初始权限位（如 01777 含粘滞位）
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 本模块作为测试入口不持有自定义锁；挂载操作通过 `Vfs` 内部挂载表锁串行化，用户态测试进程生命周期由 `UserOwner` 夹具统一管理与回收。

use super::*;
use crate::fs::memfs::{Filesystem, MemoryFs, TmpfsOptions};

/// `tmpfs` 动态增长、配额与重配置测试调用的用户态 C 自测程序路径。
const PROGRAM: &[u8] = b"/test/selftests/fs/tmpfs_growth";

/// `tmpfs` 容量增长、配额限制与挂载选项解析测试套件。
#[ktest::suite(fixture = UserOwner, suite_resources = UserSuite, context = "Thread", tags("user", "user-tmpfs", "F09"), resources("root"), timeout_ms = 180000)]
pub(crate) mod tmpfs_growth {
    use super::*;

    /// 测试目标：验证挂载在 `/tmp` 上的 `tmpfs` 在用户态文件读写增长、容量与 Inode 配额耗尽以及挂载参数重配置（`selection`）场景下的行为正确性。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：初始化 TTY 测试环境，解析 `/tmp` 目录路径，并以 `mode=01777` 选项创建且挂载可读写的 `MemoryFs`（`tmpfs`）实例；
    /// 2. When：以 `ROOT` 凭据启动用户态 C 自测程序 `/test/selftests/fs/tmpfs_growth`，传入子项选择器参数（`"growth"`、`"quota"` 或 `"options"`）；
    /// 3. Then：用户态测试程序内各项断言全部通过并以状态码 `0` 退出，由夹具完成 `wait` 与子任务回收。
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

    /// 测试目标：验证 `TmpfsOptions::parse` 对默认配额、全量单位后缀（`k` 至 `E`）、同名选项后写覆盖、零配额以及算术溢出边界的解析行为。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：给定总内存页数基准 `total_pages = 1001`；
    /// 2. When & Then：
    ///    - 解析空字符串 `""` 时，默认块数与 Inode 上限均为总页数的一半 `(500, 500)`；
    ///    - 依次解析 `size=1k`、`1M`、`1g`、`1T`、`1p`、`1E`，得到的页上限 `blocks` 等于对应字节数按 `PAGE_SIZE` 向上取整；
    ///    - 解析包含多次重复设置的组合串 `"size=1K,nr_blocks=0x10,size=25%,nr_inodes=010,mode=0700,mode=01777"`，最终生效值为末尾覆盖值（`blocks = Some(251)`、`inodes = Some(8)`、`mode = Some(0o1777)`）；
    ///    - 解析 `"size=0,nr_inodes=0"` 时允许显式零配额 `(Some(0), Some(0))`；
    ///    - 解析超出 64 位整数或页索引表示范围的 `"size=16E"`、`"nr_blocks=0x8000000000000000"`、`"nr_inodes=0x40000000000000"` 时，均返回 `Err(FsError::InvalidOperation)`。
    #[test]
    fn parser_units_defaults_and_overflow(
        context: &mut TestContext<'_>,
        _fixture: &mut UserOwner,
    ) -> TestResult {
        // Step 1: 验证空选项字符串下的默认配额（总页数的一半）。
        let defaults = checked(context, TmpfsOptions::parse(b"", 1001))?;
        context.expect_eq(defaults.limits(1001), (500, 500));

        // Step 2: 遍历 k/M/g/T/p/E 六种容量单位后缀，验证字节数向物理页数（PAGE_SIZE）的向上取整换算。
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

        // Step 3: 验证八进制/十六进制/百分比混排及后出现选项覆盖先前选项的规则。
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

        // Step 4: 验证零配额合法性与超大数值溢出拦截（返回 InvalidOperation）。
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
