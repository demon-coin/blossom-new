//! 明确启动盘选择、普通系统树读取与未挂原盘写入的真实消费者。

use crate::block::partition::{self, DeviceName, DeviceSelector};
use crate::block::{Bio, BioAllocator, BioMemory, BlkOperation, IoResult};
use crate::fs::ext4::crc::crc32c;
use crate::fs::ktest::*;
use crate::fs::system::{boot_devices, mount_disks, read_summary, BootDevices, FileSummary};
use crate::mm::dma::MappedPages;
use virtio_drivers::BufferDirection;

const ROOT_FILES: [&[u8]; 5] = [
    b"/test/identity",
    b"/test/data.bin",
    b"/test/empty",
    b"/test/sparse.bin",
    b"/test/bin/probe.elf",
];
const EXTRA_FILES: [&[u8]; 5] = [
    b"/mnt/extra/test/identity",
    b"/mnt/extra/test/data.bin",
    b"/mnt/extra/test/empty",
    b"/mnt/extra/test/sparse.bin",
    b"/mnt/extra/test/bin/probe.elf",
];

struct SystemOwner {
    owner: Owner,
    vfs: Option<Arc<Vfs>>,
    context: Option<FsContext>,
    files: Option<Arc<FileTable>>,
}

/// 真实 fixture 库存只附着 VirtIO，其他设备名不能被解释为同编号的测试盘。
fn fixture_index(selector: DeviceSelector) -> usize {
    match selector.device {
        DeviceName::Virtio(index) => usize::from(index),
        _ => usize::MAX,
    }
}

/// 原整盘测试明确构造无分区选择，避免把分区编号与库存槽混用。
fn whole_selector(index: u8) -> DeviceSelector {
    DeviceSelector {
        device: DeviceName::Virtio(index),
        partition: None,
    }
}

impl SystemOwner {
    /// 仅借原拥有者已经启动的槽，缺选中槽保留 MissingRoot，不猜其它磁盘。
    fn selected(
        &self,
        selection: BootDevices<'_>,
    ) -> Result<(BlockView, Option<BlockView>), FsError> {
        let root = self.selected_view(selection.root)?;
        let extra = selection
            .extra
            .map(|selector| self.selected_view(selector))
            .transpose()?;
        Ok((root, extra))
    }

    /// 分区选择沿生产解析取得相对整盘的实际范围，缺项保留 MissingRoot。
    fn selected_view(&self, selector: DeviceSelector) -> Result<BlockView, FsError> {
        let disk = self
            .owner
            .disks
            .get(fixture_index(selector))
            .ok_or(FsError::MissingRoot)?;
        let whole = BlockView::whole(disk.clone());
        let Some(number) = selector.partition else {
            return Ok(whole);
        };
        let partition = partition::discover(&whole)?
            .into_iter()
            .find(|partition| partition.number == number)
            .ok_or(FsError::MissingRoot)?;
        Ok(BlockView::new(
            disk.clone(),
            partition.start_sector,
            partition.sectors,
        )?)
    }

    /// 正文按永久命令行的根权限与目录构造卷和视图，返回的路径字节仍借 BootInfo。
    fn prepare(
        &mut self,
        context: &mut TestContext<'_>,
    ) -> Result<BootDevices<'static>, TestError> {
        self.owner.disk()?;
        let selection = checked(context, boot_devices(crate::boot::info().bootargs()))?;
        let (root, extra) = checked(context, self.selected(selection))?;
        self.owner.cache().ktest_set_load_hook(None);
        let (vfs, fs_context) = checked(
            context,
            mount_disks(
                root,
                extra,
                self.owner.cache().clone(),
                selection.root_mode,
                selection.rootdir,
            ),
        )?;
        let vfs = Arc::try_new(vfs).map_err(|_| TestError("system VFS owner OOM"))?;
        let files = Arc::try_new(checked(context, FileTable::try_new())?)
            .map_err(|_| TestError("system file table owner OOM"))?;

        context.expect_eq(
            checked(
                context,
                checked(context, fs_context.root().inode())?.disk_volume(),
            )?
            .disk()
            .identity(),
            self.owner.disks[fixture_index(selection.root)].identity(),
        );
        context.expect_eq(fs_context.root().filesystem().mode(), selection.root_mode);
        let writable = match selection.root_mode {
            AccessMode::ReadOnly => Err(FsError::ReadOnly),
            AccessMode::ReadWrite => Ok(()),
        };
        context.expect_eq(fs_context.root().check_writable(), writable);
        if selection.extra.is_some() {
            let extra = checked(
                context,
                vfs.resolve(&fs_context, b"/mnt/extra", ResolveMode::Follow),
            )?;
            context.expect_eq(extra.filesystem().mode(), AccessMode::ReadOnly);
            context.expect_eq(extra.check_writable(), Err(FsError::ReadOnly));
        }
        context.diagnostic(format_args!(
            "fs-system-selection root={:?} extra={:?} mode={:?} identity={:?}",
            selection.root,
            selection.extra,
            selection.root_mode,
            self.owner.disks[fixture_index(selection.root)].identity()
        ));

        self.vfs = Some(vfs);
        self.context = Some(fs_context);
        self.files = Some(files);
        Ok(selection)
    }

    /// 正文借用已完整发表的路径视图，不从旧 Task 取得隐式根。
    fn vfs(&self) -> &Vfs {
        self.vfs.as_ref().expect("system VFS consumed before body")
    }

    /// 根和 cwd 的 Path 保活同一真实挂载视图。
    fn context(&self) -> &FsContext {
        self.context
            .as_ref()
            .expect("system context consumed before body")
    }

    /// 正文借用真实表，静止化之前保活所有打开文件来源。
    fn files(&self) -> &Arc<FileTable> {
        self.files
            .as_ref()
            .expect("system files consumed before body")
    }

    /// 正式摘要逐块读到 EOF；日志供宿主与本次实际 staging 源全文比较。
    fn summary(
        &self,
        context: &TestContext<'_>,
        pathname: &[u8],
    ) -> Result<FileSummary, TestError> {
        let summary = checked(
            context,
            read_summary(self.files(), self.vfs(), self.context(), pathname),
        )?;
        context.diagnostic(format_args!(
            "fs-system-summary path={} bytes={} crc32c={:08x}",
            core::str::from_utf8(pathname).expect("system fixture paths are ASCII"),
            summary.bytes,
            summary.crc32c
        ));
        Ok(summary)
    }

    /// 固定五个普通文件均经同一正式 FileTable 摘要入口，图样与 ELF 首头另作客体判定。
    fn tree(
        &self,
        context: &mut TestContext<'_>,
        paths: &[&[u8]; 5],
    ) -> Result<[FileSummary; 5], TestError> {
        let identity = self.summary(context, paths[0])?;
        let data = self.summary(context, paths[1])?;
        let empty = self.summary(context, paths[2])?;
        let sparse = self.summary(context, paths[3])?;
        let probe = self.summary(context, paths[4])?;
        context.expect_eq(data.bytes, PATTERN_SIZE);
        context.expect_eq(
            empty,
            FileSummary {
                bytes: 0,
                crc32c: 0,
            },
        );
        context.expect_eq(sparse.bytes, 12325);

        let verified_data = verify_pattern(self, context, paths[1], false)?;
        context.expect_eq(verified_data, data);
        let verified_sparse = verify_pattern(self, context, paths[3], true)?;
        context.expect_eq(verified_sparse, sparse);
        verify_probe(self, context, paths[4])?;
        context.diagnostic(format_args!("fs-system-proof pattern-all-bytes=1 sparse-all-bytes=1 probe-host-full-crc-required=1 hardware-EIO-covered=0"));
        Ok([identity, data, empty, sparse, probe])
    }
}

impl Fixture<DeviceInventory> for SystemOwner {
    /// setup 只持有完整旧设备责任，fallible 文件系统准备属于正文。
    fn set_up(
        context: &mut TestContext<'_>,
        inventory: &DeviceInventory,
    ) -> Result<Self, TestError> {
        Ok(Self {
            owner: Owner::prepare(context, inventory)?,
            vfs: None,
            context: None,
            files: None,
        })
    }

    /// 先真 join 原任务再释放其表和路径来源，clean prune 后原 Owner 核六域并唯一停盘。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        let cache = self.owner.cache().clone();
        self.owner.quiesce_jobs()?;
        drop(self.files.take());
        drop(self.context.take());
        if let Some(vfs) = self.vfs.take() {
            vfs.prune_unused();
            drop(vfs);
        }
        cache.prune_unused();
        context.expect_eq(cache.len(), 0);
        drop(cache);
        self.owner.quiesce_devices(context)
    }

    /// 完整视图已经释放，消费原设备拥有者并核第二层完整六域。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        let Self {
            owner,
            vfs,
            context: fs_context,
            files,
        } = self;
        drop(files);
        drop(fs_context);
        drop(vfs);
        owner.tear_down(context)
    }
}

/// 经真实描述符逐字节核完整图样和 sparse 零区，并保证读取错误也关闭本次描述符。
fn verify_pattern(
    fixture: &SystemOwner,
    context: &mut TestContext<'_>,
    pathname: &[u8],
    sparse: bool,
) -> Result<FileSummary, TestError> {
    let files = fixture.files();
    let fd = checked(
        context,
        files.open(
            fixture.vfs(),
            fixture.context(),
            pathname,
            OpenOptions::default(),
            1024,
        ),
    )?;
    let result = (|| {
        let mut buffer = [0xa7; mm::PAGE_SIZE];
        let mut bytes = 0u64;
        let mut crc = !0u32;
        loop {
            let count = checked(context, files.read(fd, &mut buffer))?;
            if count == 0 {
                break;
            }
            for (index, byte) in buffer[..count].iter().enumerate() {
                let offset = bytes + index as u64;
                let valid = if sparse && (4096..12288).contains(&offset) {
                    *byte == 0
                } else {
                    patterned(core::slice::from_ref(byte), offset)
                };
                if !valid {
                    return Err(TestError("system complete file byte mismatch"));
                }
            }
            crc = crc32c(crc, &buffer[..count]);
            bytes += count as u64;
        }
        context.expect_eq(bytes, if sparse { 12325 } else { PATTERN_SIZE });
        Ok(FileSummary {
            bytes,
            crc32c: !crc,
        })
    })();
    let closed = checked(context, files.close(fd));
    match result {
        Ok(summary) => closed.map(|()| summary),
        Err(error) => Err(error),
    }
}

/// ELF 首头只证明类型与架构；全文外部证明必须由宿主核本次源文件摘要。
fn verify_probe(
    fixture: &SystemOwner,
    context: &mut TestContext<'_>,
    pathname: &[u8],
) -> TestResult {
    let files = fixture.files();
    let fd = checked(
        context,
        files.open(
            fixture.vfs(),
            fixture.context(),
            pathname,
            OpenOptions::default(),
            1024,
        ),
    )?;
    let result = (|| {
        let mut header = [0u8; 64];
        context.expect_eq(checked(context, files.read(fd, &mut header))?, header.len());
        context.expect_eq(&header[..7], b"\x7fELF\x02\x01\x01".as_slice());
        let machine = if cfg!(target_arch = "riscv64") {
            243
        } else {
            258
        };
        context.expect_eq(u16::from_le_bytes([header[18], header[19]]), machine);
        Ok(())
    })();
    let closed = checked(context, files.close(fd));
    result.and(closed)
}

/// 一页独占原页复用正式 Bio，Write 在移交前按既有 scratch 图样完整初始化。
fn raw_bio(disk: &Disk, operation: BlkOperation) -> Result<Bio, BlockError> {
    let mut memory = Vec::new_in(BioAllocator::Heap);
    if operation != BlkOperation::Flush {
        let bytes = disk.limits().logical_block_size as usize;
        if mm::PAGE_SIZE < bytes {
            return Err(BlockError::Unsupported);
        }
        let pages = mm::alloc_pages(0, true).ok_or(BlockError::NoMemory)?;
        let direction = if operation == BlkOperation::Write {
            BufferDirection::DriverToDevice
        } else {
            BufferDirection::DeviceToDriver
        };
        let mut mapped = MappedPages::new(pages, 0, bytes, direction).map_err(BlockError::from)?;
        if operation == BlkOperation::Write {
            for (index, byte) in mapped.bytes_mut().iter_mut().enumerate() {
                byte.write((index as u8).wrapping_mul(13).wrapping_add(71));
            }
        }
        memory
            .try_reserve_exact(1)
            .map_err(|_| BlockError::NoMemory)?;
        memory.push(mapped);
    }

    let sector = if operation == BlkOperation::Flush {
        0
    } else {
        128
    };
    // SAFETY: 原页为局部独占 CPU-owned，Write 全范围已初始化，Read 原页已清零；
    // 无裸别名和回调，唯一 memory 转交 Bio 后只在真实完成全尾取得字节借用。
    unsafe { Bio::try_new(disk, operation, sector, memory, 0, None) }
}

/// 真实提交并等待 DMA 收权后领回原 memory，错误前缀不被当作完整读写。
fn raw_io(disk: &DiskRef, operation: BlkOperation) -> Result<(IoResult, BioMemory), BlockError> {
    let bio = raw_bio(disk, operation)?;
    let completion = bio.completion();
    disk.submit_bio(bio);
    let result = completion.wait();
    let memory = completion.take_memory()?;
    Ok((result, memory))
}

/// 父仅并入真实宏锚点；各资源和启动参数条件由具名 suite 明确区分。
pub(crate) fn anchors() -> [&'static ::ktest::SuiteSlot; 6] {
    [
        arguments::__ktest_anchor(),
        root::__ktest_anchor(),
        dual::__ktest_anchor(),
        unselected::__ktest_anchor(),
        scratch::__ktest_anchor(),
        missing::__ktest_anchor(),
    ]
}

#[ktest::suite(
    context = "Thread",
    tags("fs-system", "fs-system-arguments"),
    timeout_ms = 15000
)]
pub(crate) mod arguments {
    use super::*;
    use crate::task::exec::{self, Arguments, ExecError};

    /// 解析保留设备、权限和路径原始字节；执行参数分别拒绝空路径与内嵌 NUL，不证明实际启动。
    #[test]
    #[ktest(tags("B01"))]
    fn parser_boundaries_are_not_boot_selection(context: &mut TestContext<'_>) -> TestResult {
        for (bytes, root, extra, mode) in [
            (b"root=/dev/vda".as_slice(), 0, None, AccessMode::ReadOnly),
            (
                b" \troot=/dev/vdb\r\nextra=/dev/vda\x0b\x0c",
                1,
                Some(0),
                AccessMode::ReadOnly,
            ),
            (
                b"root=/dev/vdz root=/dev/vda extra=/dev/vda extra=/dev/vdc",
                0,
                Some(2),
                AccessMode::ReadOnly,
            ),
            (
                b"console=ttyS0 root=/dev/vdz",
                25,
                None,
                AccessMode::ReadOnly,
            ),
            (
                b"root=unsupported root=/dev/vdb extra=unsupported extra=/dev/vda",
                1,
                Some(0),
                AccessMode::ReadOnly,
            ),
            (b"rw root=/dev/vda ro rw", 0, None, AccessMode::ReadWrite),
            (b"root=/dev/vda rw ro", 0, None, AccessMode::ReadOnly),
            (
                b"root=/dev/vda rw=1 ro=1 xrw rwx xro rox",
                0,
                None,
                AccessMode::ReadOnly,
            ),
            (
                b"root=/dev/vda rw ro=1 xro rox",
                0,
                None,
                AccessMode::ReadWrite,
            ),
            (
                b"rw\troot=/dev/vdz\rro\nroot=/dev/vdb\x0brw\x0cextra=/dev/vda ",
                1,
                Some(0),
                AccessMode::ReadWrite,
            ),
            (
                b"root=/dev/vda rw\x85ro rw\0ro",
                0,
                None,
                AccessMode::ReadOnly,
            ),
        ] {
            let selected = checked(context, boot_devices(bytes))?;
            context.expect_eq(
                (selected.root, selected.extra, selected.root_mode),
                (whole_selector(root), extra.map(whole_selector), mode),
            );
        }

        for (bytes, rootdir) in [
            (b"rootdir=/".as_slice(), b"/".as_slice()),
            (b"rootdir=/bad rootdir=/test", b"/test"),
            (b"rootdir=/test rootdir=", b""),
            (b"rootdir= rootdir=/missing", b"/missing"),
            (b"rootdir=/bad\x0brootdir=/\x0c", b"/"),
            (b"rootdir=/test\0tail", b"/test\0tail"),
        ] {
            context.expect_eq(checked(context, boot_devices(bytes))?.rootdir, rootdir);
        }

        for (bytes, init) in [
            (
                b"root=/dev/vda init=/bin/init".as_slice(),
                b"/bin/init".as_slice(),
            ),
            (b"root=/dev/vda init=/bad init=/sbin/init", b"/sbin/init"),
            (b"root=/dev/vda init=/sbin/init init=", b""),
            (b"root=/dev/vda init= init=/missing", b"/missing"),
            (
                b"root=/dev/vda\tinit=/bad\x0binit=/sbin/init\x0c",
                b"/sbin/init",
            ),
            (b"root=/dev/vda init=/sbin/init\0tail", b"/sbin/init\0tail"),
        ] {
            context.expect_eq(checked(context, boot_devices(bytes))?.init, init);
        }

        for (bytes, error) in [
            (
                b"root=/dev/vda init=".as_slice(),
                ExecError::File(FsError::NotFound),
            ),
            (
                b"root=/dev/vda init=/sbin/init\0tail".as_slice(),
                ExecError::Format,
            ),
        ] {
            let selected = checked(context, boot_devices(bytes))?;
            context.expect_eq(
                Arguments::from_kernel(
                    selected.init,
                    &[selected.init],
                    &[],
                    exec::DEFAULT_STACK_BYTES,
                )
                .err(),
                Some(error),
            );
        }

        for bytes in [
            b"root=".as_slice(),
            b"root=/dev/vdaa",
            b"root=/dev/vdA",
            b"root=UUID=abc",
            b"root=/dev/vda root=",
            b"root=/dev/vda extra=",
            b"rw root=/dev/vda root= ro",
            b"ro root=/dev/vda extra= rw",
        ] {
            context.expect_eq(boot_devices(bytes), Err(FsError::MissingRoot));
        }
        context.diagnostic(format_args!(
            "fs-system-arguments model-only=1 actual-bootargs-covered=0"
        ));
        Ok(())
    }
}

#[ktest::suite(fixture = SystemOwner, suite_resources = DeviceInventory, context = "Thread", tags("fs-system", "fs-system-root"), resources("root"), timeout_ms = 90000)]
pub(crate) mod root {
    use super::*;

    /// 根目录来自同一磁盘的真实子目录；绝对路径与父组件都受返回上下文的完整 Path 约束。
    #[test]
    #[ktest(tags("fs-system-rootdir"))]
    fn initial_directory_view_is_real_and_bounded(
        context: &mut TestContext<'_>,
        fixture: &mut SystemOwner,
    ) -> TestResult {
        fixture.owner.disk()?;
        let selected = checked(context, boot_devices(crate::boot::info().bootargs()))?;
        let (root, _) = checked(context, fixture.selected(selected))?;
        fixture.owner.cache().ktest_set_load_hook(None);
        let (vfs, fs_context) = checked(
            context,
            mount_disks(
                root,
                None,
                fixture.owner.cache().clone(),
                selected.root_mode,
                b"/test",
            ),
        )?;
        context.expect_eq(fs_context.root().ptr_eq(fs_context.cwd()), true);
        let file = checked(
            context,
            vfs.resolve(&fs_context, b"/identity", ResolveMode::Follow),
        )?;
        let parent_file = checked(
            context,
            vfs.resolve(&fs_context, b"../../identity", ResolveMode::Follow),
        )?;
        context.expect_eq(file.ptr_eq(&parent_file), true);
        context.expect_eq(
            vfs.resolve(&fs_context, b"/test/identity", ResolveMode::Follow)
                .err(),
            Some(FsError::NotFound),
        );
        context.expect_eq(
            checked(context, checked(context, file.inode())?.disk_volume())?
                .disk()
                .identity(),
            fixture.owner.disks[fixture_index(selected.root)].identity(),
        );
        drop(parent_file);
        drop(file);
        fixture.vfs = Some(Arc::try_new(vfs).map_err(|_| TestError("rootdir VFS owner OOM"))?);
        fixture.context = Some(fs_context);
        Ok(())
    }

    /// 显式不存在、普通文件、空值与 NUL 路径保留真实错误，不回退物理根；原 fixture 负责错误尾回收。
    #[test]
    #[ktest(tags("fs-system-rootdir"))]
    fn invalid_directory_does_not_fall_back(
        context: &mut TestContext<'_>,
        fixture: &mut SystemOwner,
    ) -> TestResult {
        fixture.owner.disk()?;
        let selected = checked(context, boot_devices(crate::boot::info().bootargs()))?;
        let (root, _) = checked(context, fixture.selected(selected))?;
        fixture.owner.cache().ktest_set_load_hook(None);
        for (rootdir, error) in [
            (b"/test/absent".as_slice(), FsError::NotFound),
            (b"/test/identity", FsError::NotDirectory),
            (b"", FsError::NotFound),
            (b"/test\0tail", FsError::InvalidName),
        ] {
            context.expect_eq(
                mount_disks(
                    root.clone(),
                    None,
                    fixture.owner.cache().clone(),
                    selected.root_mode,
                    rootdir,
                )
                .err(),
                Some(error),
            );
        }
        Ok(())
    }

    /// 单盘正式 factory 完整读普通树，并真实消费硬链接、相对链接、绝对链接和循环错误。
    #[test]
    #[ktest(tags("B01", "B16", "B24"))]
    fn selected_root_full_tree_and_links(
        context: &mut TestContext<'_>,
        fixture: &mut SystemOwner,
    ) -> TestResult {
        let selected = fixture.prepare(context)?;
        context.expect_eq(selected.extra, None);
        let summaries = fixture.tree(context, &ROOT_FILES)?;

        let data = checked(
            context,
            fixture
                .vfs()
                .resolve(fixture.context(), ROOT_FILES[1], ResolveMode::Follow),
        )?;
        let hard = checked(
            context,
            fixture
                .vfs()
                .resolve(fixture.context(), b"/test/links/hard", ResolveMode::Follow),
        )?;
        context.expect_eq(
            checked(context, data.inode())?.ptr_eq(checked(context, hard.inode())?),
            true,
        );
        context.expect_eq(
            core::ptr::eq(
                checked(context, data.inode())?.mapping(),
                checked(context, hard.inode())?.mapping(),
            ),
            true,
        );
        for pathname in [
            b"/test/links/hard".as_slice(),
            b"/test/links/relative",
            b"/test/links/absolute",
        ] {
            context.expect_eq(fixture.summary(context, pathname)?, summaries[1]);
        }

        context.expect_eq(
            read_summary(
                fixture.files(),
                fixture.vfs(),
                fixture.context(),
                b"/test/links/loop",
            ),
            Err(FsError::TooManySymlinks),
        );
        context.expect_eq(
            read_summary(
                fixture.files(),
                fixture.vfs(),
                fixture.context(),
                b"/test/absent",
            ),
            Err(FsError::NotFound),
        );
        context.expect_eq(
            read_summary(fixture.files(), fixture.vfs(), fixture.context(), b"/test"),
            Err(FsError::IsDirectory),
        );
        context.expect_eq(fixture.summary(context, ROOT_FILES[2])?, summaries[2]);
        Ok(())
    }
}

#[ktest::suite(fixture = SystemOwner, suite_resources = DeviceInventory, context = "Thread", tags("fs-system", "fs-system-dual"), resources("root", "extra"), timeout_ms = 90000)]
pub(crate) mod dual {
    use super::*;

    /// 两盘由本次固件参数明确选择，同号 identity inode 的对象、mapping 和内容必须隔离。
    #[test]
    #[ktest(tags("B01", "B16", "B24"))]
    fn explicit_root_extra_same_inode_separate_content(
        context: &mut TestContext<'_>,
        fixture: &mut SystemOwner,
    ) -> TestResult {
        let selected = fixture.prepare(context)?;
        let extra = selected.extra.ok_or(TestError(
            "system dual requires explicit extra boot argument",
        ))?;
        context.expect_eq(selected.root == extra, false);
        fixture.tree(context, &ROOT_FILES)?;
        fixture.tree(context, &EXTRA_FILES)?;

        let first = checked(
            context,
            fixture
                .vfs()
                .resolve(fixture.context(), ROOT_FILES[0], ResolveMode::Follow),
        )?;
        let second = checked(
            context,
            fixture
                .vfs()
                .resolve(fixture.context(), EXTRA_FILES[0], ResolveMode::Follow),
        )?;
        context.expect_eq(
            checked(context, checked(context, second.inode())?.disk_volume())?
                .disk()
                .identity(),
            fixture.owner.disks[fixture_index(extra)].identity(),
        );
        let first_inode = checked(context, first.inode())?;
        let second_inode = checked(context, second.inode())?;
        context.expect_eq(first_inode.number(), second_inode.number());
        context.expect_eq(first_inode.ptr_eq(second_inode), false);
        context.expect_eq(
            core::ptr::eq(first_inode.mapping(), second_inode.mapping()),
            false,
        );
        context.expect_eq(
            checked(
                context,
                fixture.vfs().inodes().iget(
                    checked(context, first_inode.disk_volume())?,
                    first_inode.number(),
                ),
            )?
            .ptr_eq(first_inode),
            true,
        );
        context.expect_eq(
            checked(
                context,
                fixture.vfs().inodes().iget(
                    checked(context, second_inode.disk_volume())?,
                    second_inode.number(),
                ),
            )?
            .ptr_eq(second_inode),
            true,
        );

        let mut root_bytes = [0xa7; 64];
        let mut extra_bytes = [0xa7; 64];
        let mut offset = 0;
        let mut different = false;
        loop {
            let first_len = checked(context, first_inode.read_at(offset, &mut root_bytes))?;
            let second_len = checked(context, second_inode.read_at(offset, &mut extra_bytes))?;
            different |= root_bytes[..first_len] != extra_bytes[..second_len];
            if first_len == 0 && second_len == 0 {
                break;
            }
            offset += first_len.max(second_len) as u64;
        }
        context.expect_eq(different, true);
        context.diagnostic(format_args!("fs-system-dual identity-inode={} root-disk={:?} extra-disk={:?} swapped-host-source-comparison-required=1", first_inode.number(), checked(context, first_inode.disk_volume())?.disk().identity(), checked(context, second_inode.disk_volume())?.disk().identity()));
        Ok(())
    }
}

#[ktest::suite(fixture = SystemOwner, suite_resources = DeviceInventory, context = "Thread", tags("fs-system", "fs-system-unselected"), resources("root", "scratch"), timeout_ms = 90000)]
pub(crate) mod unselected {
    use super::*;

    /// 未选择的 blank scratch 保留真实硬件实例，但正式 factory 不扫描其文件系统。
    #[test]
    #[ktest(tags("B01", "B24"))]
    fn attached_blank_unselected_disk_has_no_fs_io(
        context: &mut TestContext<'_>,
        fixture: &mut SystemOwner,
    ) -> TestResult {
        let selection = checked(context, boot_devices(crate::boot::info().bootargs()))?;
        context.expect_eq(selection.root, whole_selector(0));
        context.expect_eq(selection.extra, None);
        let before = fixture.owner.disks[1].test_snapshot().submitted;
        fixture.prepare(context)?;
        fixture.tree(context, &ROOT_FILES)?;
        context.expect_eq(fixture.owner.disks[1].test_snapshot().submitted, before);
        context.expect_eq(before, 0);
        Ok(())
    }
}

#[ktest::suite(fixture = SystemOwner, suite_resources = DeviceInventory, context = "Thread", tags("fs-system", "fs-system-scratch"), resources("root", "extra", "scratch"), timeout_ms = 90000)]
pub(crate) mod scratch {
    use super::*;

    /// 挂载盘由软件只读 claim 拒绝原写，未挂 scratch 实际写、读回与硬件 FLUSH 不串盘。
    #[test]
    #[ktest(tags("B10", "B24"))]
    fn mounted_claims_and_unmounted_raw_write_read_flush(
        context: &mut TestContext<'_>,
        fixture: &mut SystemOwner,
    ) -> TestResult {
        let selection = fixture.prepare(context)?;
        let extra = selection.extra.ok_or(TestError(
            "system scratch requires explicit extra boot argument",
        ))?;
        if fixture_index(selection.root) == 2 || fixture_index(extra) == 2 {
            return Err(TestError("system scratch slot cannot be mounted"));
        }
        for index in [fixture_index(selection.root), fixture_index(extra)] {
            let disk = &fixture.owner.disks[index];
            if disk.limits().readonly {
                return Err(TestError(
                    "system claim proof requires writable host root and extra",
                ));
            }
            let before = disk.test_snapshot().submitted;
            let (result, memory) = raw_io(disk, BlkOperation::Write)
                .map_err(|_| TestError("system mounted Write preparation failed"))?;
            context.expect_eq(
                result,
                IoResult {
                    completed: 0,
                    error: Some(BlockError::ReadOnly),
                },
            );
            context.expect_eq(disk.test_snapshot().submitted, before);
            drop(memory);
        }

        let disk = &fixture.owner.disks[2];
        let unit = disk.limits().logical_block_size as usize;
        let (written, memory) = raw_io(disk, BlkOperation::Write)
            .map_err(|_| TestError("system scratch Write failed"))?;
        context.expect_eq(
            written,
            IoResult {
                completed: unit,
                error: None,
            },
        );
        drop(memory);

        let (read, memory) = raw_io(disk, BlkOperation::Read)
            .map_err(|_| TestError("system scratch Read-back failed"))?;
        if read
            != (IoResult {
                completed: unit,
                error: None,
            })
        {
            return Err(TestError("system scratch Read-back incomplete"));
        }
        for mapped in &memory {
            context.expect_eq(mapped.is_device_owned(), false);
            // SAFETY: 唯一映射原页已清零，实际完整 Read 成功且 completion 已收回全部 DMA 访问；
            // 无回调、裸别名或 CPU 写者，memory 在此借用及核验期间保活。
            let bytes = unsafe { mapped.bytes() };
            context.expect_eq(
                bytes
                    .iter()
                    .enumerate()
                    .all(|(index, byte)| *byte == (index as u8).wrapping_mul(13).wrapping_add(71)),
                true,
            );
        }
        drop(memory);

        if disk.limits().flush {
            let (flushed, memory) = raw_io(disk, BlkOperation::Flush)
                .map_err(|_| TestError("system scratch FLUSH failed"))?;
            context.expect_eq(
                flushed,
                IoResult {
                    completed: 0,
                    error: None,
                },
            );
            drop(memory);
            context.diagnostic(format_args!(
                "fs-system-scratch sector=128 bytes={} write-readback=1 hardware-flush=1",
                unit
            ));
        } else {
            context.expect_eq(
                raw_bio(disk, BlkOperation::Flush).err(),
                Some(BlockError::Unsupported),
            );
            context.diagnostic(format_args!("fs-system-scratch sector=128 bytes={} write-readback=1 hardware-flush=unsupported hardware-flush-covered=0", unit));
        }

        fixture.tree(context, &ROOT_FILES)?;
        fixture.tree(context, &EXTRA_FILES)?;
        Ok(())
    }
}

#[ktest::suite(fixture = SystemOwner, suite_resources = DeviceInventory, context = "Thread", tags("fs-system", "fs-system-missing"), resources("root"), timeout_ms = 90000)]
pub(crate) mod missing {
    use super::*;

    /// 宿主明确传不存在的根名，客体真实 BootInfo 选择失败且不探测任何盘面。
    #[test]
    #[ktest(tags("B01"))]
    fn actual_bootargs_missing_selected_slot(
        context: &mut TestContext<'_>,
        fixture: &mut SystemOwner,
    ) -> TestResult {
        let selection = checked(context, boot_devices(crate::boot::info().bootargs()))?;
        if fixture_index(selection.root) < fixture.owner.disks.len() {
            return Err(TestError(
                "system missing case requires absent selected root boot slot",
            ));
        }
        context.expect_eq(
            fixture.selected(selection).err(),
            Some(FsError::MissingRoot),
        );
        context.expect_eq(
            fixture
                .owner
                .disks
                .iter()
                .all(|disk| disk.test_snapshot().submitted == 0),
            true,
        );
        context.diagnostic(format_args!(
            "fs-system-missing actual-root={:?} error=MissingRoot disk-fs-submissions=0",
            selection.root
        ));
        Ok(())
    }
}
