//! 启动设备参数解析、系统根与副盘挂载、文件目录树读取及未挂载裸盘 I/O 内核集成测试套件。
//!
//! # 模块定位与职责
//! 本模块位于文件系统测试子系统（`fs::ktest`）中，负责验证 `crate::fs::system` 模块对内核启动参数（`bootargs`）的解析、主根盘（`root`）与副盘（`extra`）的挂载组装、初始根目录视图（`rootdir`）隔离、标准测试文件集与链接读取、以及已挂载盘的只读保护与未挂载裸盘（`scratch`）的直接块 I/O。
//! 上游由 `ktest` 框架通过 `anchors()` 收集 6 个独立测试套件（`arguments`、`root`、`dual`、`unselected`、`scratch`、`missing`）并按设备资源与启动参数条件调度执行；下游调用 `boot_devices`、`mount_disks`、`read_summary`、`partition::discover`、`Vfs`、`FileTable` 以及块设备 `Bio` 接口。
//!
//! # 核心机制与磁盘挂载拓扑
//! 系统启动阶段根据 `bootargs` 中的 `root=`、`extra=`、`ro`/`rw`、`rootdir=`、`init=` 参数选择块设备（整盘或 GPT/MBR 分区）并构建 VFS 命名空间：
//!
//! ```text
//!   固件/引导参数 (bootargs)
//!       │
//!       ▼  boot_devices(bootargs)
//!   BootDevices { root, extra, root_mode (RO/RW), rootdir, init }
//!       │
//!       ├──► root  选择器 ──► VirtIO 块设备 / 分区视图 (BlockView) ──► 挂载于 "/"
//!       │                     └── 初始 FsContext (root & cwd) 限定在 rootdir 子目录
//!       │                         ├── /test/identity
//!       │                         ├── /test/data.bin
//!       │                         ├── /test/empty
//!       │                         ├── /test/sparse.bin (4096..12288 为空洞零区)
//!       │                         └── /test/bin/probe.elf
//!       │
//!       ├──► extra 选择器 ──► (可选) 副盘视图 ──► 以 ReadOnly 挂载于 "/mnt/extra"
//!       │                     └── 同编号 Inode 与根盘拥有独立 Inode/AddressSpace 实例
//!       │
//!       └──► 未选中槽位 (scratch) ──► 不执行文件系统扫描；允许直接提交裸 Bio (Write/Read/Flush)
//! ```
//!
//! # 锁与资源回收契约
//! - `SystemOwner` 夹具包装底层 `Owner` 设备与缓存管理器。在 `quiesce` 阶段按严格逆序回收：先等待后台任务完成（`quiesce_jobs`），再依次释放 `FileTable`、`FsContext`、执行 `Vfs::prune_unused` 并释放 `Vfs`，最后清理块缓存（断言缓存项归零）并停顿块设备。
//! - 直接块设备测试（`raw_bio`/`raw_io`）使用独立分配的物理页构造 `MappedPages`，在 `submit_bio` 提交给设备驱动后等待 `completion.wait()` 完成并调用 `take_memory()` 收回物理页的 CPU 访问权，随后才读取校验字节并释放页面。

use crate::block::partition::{self, DeviceName, DeviceSelector};
use crate::block::{Bio, BioAllocator, BioMemory, BlkOperation, IoResult};
use crate::fs::ext4::crc::crc32c;
use crate::fs::ktest::*;
use crate::fs::system::{boot_devices, mount_disks, read_summary, BootDevices, FileSummary};
use crate::mm::dma::MappedPages;
use virtio_drivers::BufferDirection;

/// 根文件系统（`/`）下的 5 个标准测试文件路径列表。
const ROOT_FILES: [&[u8]; 5] = [
    b"/test/identity",
    b"/test/data.bin",
    b"/test/empty",
    b"/test/sparse.bin",
    b"/test/bin/probe.elf",
];
/// 副盘挂载点（`/mnt/extra`）下的 5 个对应标准测试文件路径列表。
const EXTRA_FILES: [&[u8]; 5] = [
    b"/mnt/extra/test/identity",
    b"/mnt/extra/test/data.bin",
    b"/mnt/extra/test/empty",
    b"/mnt/extra/test/sparse.bin",
    b"/mnt/extra/test/bin/probe.elf",
];

/// 系统级文件系统测试夹具，持有底层块设备与缓存拥有者以及挂载后的 VFS、路径上下文和文件描述符表。
struct SystemOwner {
    /// 底层块设备、块缓存及内存基线管理夹具。
    owner: Owner,
    /// 已挂载的系统虚拟文件系统实例。
    vfs: Option<Arc<Vfs>>,
    /// 绑定了初始根目录与当前工作目录的文件系统路径上下文。
    context: Option<FsContext>,
    /// 供测试用例打开和读取系统文件的描述符表。
    files: Option<Arc<FileTable>>,
}

/// 将 `DeviceSelector` 映射为测试夹具 `owner.disks` 数组中的 VirtIO 槽位下标；非 VirtIO 设备返回 `usize::MAX`。
fn fixture_index(selector: DeviceSelector) -> usize {
    match selector.device {
        DeviceName::Virtio(index) => usize::from(index),
        _ => usize::MAX,
    }
}

/// 构造指向第 `index` 号 VirtIO 整盘（无分区偏移）的设备选择器。
fn whole_selector(index: u8) -> DeviceSelector {
    DeviceSelector {
        device: DeviceName::Virtio(index),
        partition: None,
    }
}

impl SystemOwner {
    /// 根据 `BootDevices` 解析结果，从夹具已初始化的磁盘列表中提取根盘（`root`）和可选副盘（`extra`）的块视图。
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

    /// 根据设备选择器 `selector` 构造对应的 `BlockView`（整盘视图或通过分区表扫描匹配的指定分区范围），若磁盘或分区不存在则返回 `Err(FsError::MissingRoot)`。
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

    /// 解析当前内核启动参数 `bootargs`，挂载选中的根盘与副盘，创建 `FileTable`，并校验挂载点设备身份与读写权限。
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

    /// 获取当前已挂载的 `Vfs` 引用。
    fn vfs(&self) -> &Vfs {
        self.vfs.as_ref().expect("system VFS consumed before body")
    }

    /// 获取当前已初始化的 `FsContext` 路径上下文引用。
    fn context(&self) -> &FsContext {
        self.context
            .as_ref()
            .expect("system context consumed before body")
    }

    /// 获取当前测试夹具持有的 `FileTable` 引用。
    fn files(&self) -> &Arc<FileTable> {
        self.files
            .as_ref()
            .expect("system files consumed before body")
    }

    /// 调用 `read_summary` 完整读取指定路径文件并计算字节数与 CRC32C 校验和，同时输出诊断日志。
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

    /// 依次读取并校验 `paths` 中的 5 个标准测试文件（验证摘要、确定性数据图样、稀疏文件空洞零区以及 ELF 头部信息）。
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
    /// 初始化底层块设备与缓存管理夹具 `Owner`，VFS 挂载留待测试正文按需执行。
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

    /// 停止后台任务，按顺序释放文件表、路径上下文与 VFS，清理未引用缓存并断言块缓存清零，最后停顿块设备。
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

    /// 销毁系统测试夹具并委托底层 `Owner` 校验全局物理内存与块资源完整回收。
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

/// 通过 `FileTable` 打开指定路径文件，逐字节校验确定性数据图样（及稀疏文件在 `4096..12288` 区间的零字节空洞），计算完整 CRC32C 并确保关闭文件描述符。
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

/// 打开并读取探测程序 `probe.elf` 的前 64 字节 ELF 头部，校验 64 位小端 ELF 魔数以及与当前目标架构匹配的 `e_machine` 字段（RISC-V 为 243，LoongArch 为 258）。
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

/// 为单个逻辑块（扇区 128）分配独立物理页并构造指定操作类型（`Read`、`Write` 或 `Flush`）的 `Bio` 请求；若为写入操作则预先填充确定性校验图样。
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
    // SAFETY:
    // 1. `pages` 由 `mm::alloc_pages(0, true)` 新分配且已清零，当前由本函数独占持有，不存在任何其他指针或并发别名。
    // 2. 当 `operation == BlkOperation::Write` 时，上方循环已对 `mapped.bytes_mut()` 全部 `bytes` 字节完成初始化；当为 `Read` 时底层页面已由分配器清零。
    // 3. `memory` 所有权完整移交给 `Bio::try_new`，且未注册异步完成回调（`None`）；调用方仅在 `raw_io` 等待 I/O 彻底完成并调用 `take_memory()` 收回 DMA 访问权后才读取页面内容。
    unsafe { Bio::try_new(disk, operation, sector, memory, 0, None) }
}

/// 向块设备提交单块裸 `Bio` 请求，阻塞等待硬件 I/O 完成，并通过 `take_memory()` 收回物理页缓冲区的 CPU 所有权。
fn raw_io(disk: &DiskRef, operation: BlkOperation) -> Result<(IoResult, BioMemory), BlockError> {
    let bio = raw_bio(disk, operation)?;
    let completion = bio.completion();
    disk.submit_bio(bio);
    let result = completion.wait();
    let memory = completion.take_memory()?;
    Ok((result, memory))
}

/// 返回本模块下全部 6 个子测试套件的静态注册槽位供父模块汇总。
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

    /// 测试目标：验证内核启动参数解析器 `boot_devices` 对各种空白分隔符、重复参数覆盖、`ro`/`rw` 标志切换、`rootdir=`/`init=` 路径提取及非法设备名的边界处理，并验证 `exec::Arguments::from_kernel` 对空路径和内嵌 NUL 的拒绝行为。
    /// 场景步骤：
    /// 1. 构造多组包含不同空白符、重复 `root=`/`extra=`、穿插不支持设备名与 `ro`/`rw` 组合的参数串，校验解析出的主副设备选择器与读写模式。
    /// 2. 校验 `rootdir=` 与 `init=` 在多次出现、空值以及包含尾随 NUL 字节时的切片提取结果。
    /// 3. 将空 `init=` 与含内嵌 NUL 的 `init=` 传入 `Arguments::from_kernel`，验证分别返回 `ExecError::File(FsError::NotFound)` 与 `ExecError::Format`。
    /// 4. 验证缺失或格式非法的最终 `root=`/`extra=` 参数使 `boot_devices` 返回 `Err(FsError::MissingRoot)`。
    #[test]
    #[ktest(tags("B01"))]
    fn parser_boundaries_are_not_boot_selection(context: &mut TestContext<'_>) -> TestResult {
        // Step 1: 校验 root=、extra= 与 ro/rw 参数在空白分隔与重复覆盖下的解析结果。
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

        // Step 2: 校验 rootdir= 与 init= 原始字节切片的覆盖与提取规则。
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

        // Step 3: 验证 exec 参数构造器拒绝空 init 路径与包含内嵌 NUL 的路径。
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

        // Step 4: 验证非法或置空的 root=/extra= 设备名返回 MissingRoot 错误。
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

    /// 测试目标：验证 `mount_disks` 指定子目录 `rootdir`（如 `b"/test"`）作为初始根目录时，返回的 `FsContext` 将根与当前目录均绑定到该子目录，且绝对路径与 `".."` 父目录回退均受该根边界约束。
    /// 场景步骤：
    /// 1. 以 `b"/test"` 作为 `rootdir` 挂载根磁盘，验证 `fs_context.root()` 与 `fs_context.cwd()` 指向同一目录项。
    /// 2. 在该上下文中解析 `b"/identity"` 与 `b"../../identity"`，验证两者解析到同一个文件且无法越界逃逸，而原全路径 `b"/test/identity"` 返回 `NotFound`。
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

    /// 测试目标：验证当 `mount_disks` 传入不存在的目录、普通文件、空路径或含 NUL 字节的非法 `rootdir` 时，直接返回对应错误码并拒绝挂载。
    /// 场景步骤：
    /// 1. 依次使用 `b"/test/absent"`、`b"/test/identity"`（普通文件）、`b""`、`b"/test\0tail"` 调用 `mount_disks`。
    /// 2. 校验分别返回 `NotFound`、`NotDirectory`、`NotFound` 与 `InvalidName`。
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

    /// 测试目标：验证单根盘挂载下的完整标准目录树读取，以及硬链接、相对符号链接、绝对符号链接与循环符号链接的解析行为。
    /// 场景步骤：
    /// 1. 按启动参数挂载单根盘并校验 `ROOT_FILES` 全部 5 个标准文件的数据摘要、稀疏空洞与 ELF 头。
    /// 2. 解析 `/test/links/hard` 并与 `/test/data.bin` 比较，验证两者共享同一个 `Inode` 实例与 `AddressSpace` 页缓存映射，且通过硬链接、相对软链接、绝对软链接读取的摘要完全一致。
    /// 3. 验证读取循环软链接返回 `TooManySymlinks`、不存在路径返回 `NotFound`、目录路径返回 `IsDirectory`。
    #[test]
    #[ktest(tags("B01", "B16", "B24"))]
    fn selected_root_full_tree_and_links(
        context: &mut TestContext<'_>,
        fixture: &mut SystemOwner,
    ) -> TestResult {
        // Step 1: 挂载单根盘并完整校验 ROOT_FILES 标准文件树。
        let selected = fixture.prepare(context)?;
        context.expect_eq(selected.extra, None);
        let summaries = fixture.tree(context, &ROOT_FILES)?;

        // Step 2: 验证硬链接共享同一 Inode 与页缓存映射，且硬链接、相对软链接、绝对软链接均可读出相同文件内容。
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

        // Step 3: 验证循环符号链接、不存在文件及对目录执行普通文件摘要读取时的错误返回。
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

    /// 测试目标：验证同时挂载主根盘（`/`）与副盘（`/mnt/extra`）时，两块磁盘上具有相同 Inode 编号的 `/test/identity` 文件在 VFS `Inode` 缓存、页缓存映射（`mapping`）以及文件内容上完全隔离。
    /// 场景步骤：
    /// 1. 根据启动参数同时挂载根盘与副盘，分别校验 `/` 与 `/mnt/extra` 下的 5 个标准文件。
    /// 2. 解析 `/test/identity` 与 `/mnt/extra/test/identity`，验证两者的 Inode 编号相同，但 `Inode` 指针与 `mapping()` 指针互不相同，且通过各自磁盘卷调用 `iget` 均能精确命中各自的 `Inode` 实例。
    /// 3. 逐块读取两个 `identity` 文件的内容，验证两盘身份标识文本确实不同（未发生跨盘缓存串用）。
    #[test]
    #[ktest(tags("B01", "B16", "B24"))]
    fn explicit_root_extra_same_inode_separate_content(
        context: &mut TestContext<'_>,
        fixture: &mut SystemOwner,
    ) -> TestResult {
        // Step 1: 挂载主根盘与副盘，并分别校验两棵目录树下的 5 个标准测试文件。
        let selected = fixture.prepare(context)?;
        let extra = selected.extra.ok_or(TestError(
            "system dual requires explicit extra boot argument",
        ))?;
        context.expect_eq(selected.root == extra, false);
        fixture.tree(context, &ROOT_FILES)?;
        fixture.tree(context, &EXTRA_FILES)?;

        // Step 2: 验证两盘上同编号的 identity Inode 在对象实例、AddressSpace 映射与 iget 索引上完全隔离。
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

        // Step 3: 对比两盘 identity 文件的实际字节，确认内容不同、未发生跨盘页缓存混用。
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

    /// 测试目标：验证当系统挂有未被启动参数选中的空白测试盘（`scratch`）时，挂载根文件系统与读取文件树不会向该未选中磁盘发起任何块 I/O 请求。
    /// 场景步骤：
    /// 1. 确认启动参数仅选择 0 号盘作为 `root` 且 `extra` 为 `None`，记录 1 号空白盘的初始 `submitted` 计数。
    /// 2. 挂载根文件系统并完整读取校验 `ROOT_FILES`。
    /// 3. 验证 1 号空白盘的 `submitted` 计数始终保持为 `0`。
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

    /// 测试目标：验证已被挂载为只读文件系统的根盘与副盘拒绝底层裸 `Bio` 写入请求，而未挂载的第 3 块测试盘（`scratch`）可正常执行裸块写入、读回校验与硬件 `Flush`，且不影响已挂载文件系统的正常读取。
    /// 场景步骤：
    /// 1. 挂载根盘与副盘后，尝试向这两块已挂载磁盘直接提交 `BlkOperation::Write` 裸 `Bio`，验证被软件只读保护拦截并返回 `BlockError::ReadOnly`，且底层硬件提交计数不变。
    /// 2. 向未挂载的 2 号 `scratch` 盘扇区 128 提交单逻辑块写入，随后提交读取请求并逐字节比对读回的数据图样。
    /// 3. 根据设备能力测试 `BlkOperation::Flush`（支持则成功完成，不支持则返回 `BlockError::Unsupported`），最后再次完整读取根盘与副盘的 `ROOT_FILES` 和 `EXTRA_FILES` 验证无跨盘干扰。
    #[test]
    #[ktest(tags("B10", "B24"))]
    fn mounted_claims_and_unmounted_raw_write_read_flush(
        context: &mut TestContext<'_>,
        fixture: &mut SystemOwner,
    ) -> TestResult {
        // Step 1: 挂载根盘与副盘，验证向已挂载只读卷提交裸 Write Bio 被软件层拒绝且不触达硬件队列。
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

        // Step 2: 向未挂载的 2 号 scratch 盘提交单块裸写入与读回请求，校验 DMA 读回字节与写入图样完全一致。
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
            // SAFETY:
            // 1. `raw_io` 已通过 `completion.wait()` 等待设备读取完成，并调用 `completion.take_memory()` 收回了 `mapped` 的 DMA 访问权（上方断言 `is_device_owned() == false`）。
            // 2. 底层物理页在分配时已清零且刚由设备完成整块读取填充，当前由局部变量 `memory` 独占持有，在 `bytes` 切片生命周期内不存在并发修改或可变别名。
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

        // Step 3: 测试 scratch 盘的 Flush 操作，并重新校验根盘与副盘文件树完好无损。
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

    /// 测试目标：验证当启动参数 `root=` 指向宿主未挂载的不存在磁盘槽位时，`selected()` 立即返回 `Err(FsError::MissingRoot)`，且不会向任何已存在的块设备发送探测 I/O。
    /// 场景步骤：
    /// 1. 解析当前 `bootargs` 获取 `selection.root`，确认其槽位下标超出 `fixture.owner.disks` 实际磁盘数量。
    /// 2. 调用 `fixture.selected(selection)`，验证返回 `Err(FsError::MissingRoot)`，且所有已连接磁盘的 `submitted` 计数均为 `0`。
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
