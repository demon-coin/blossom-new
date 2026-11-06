//! 运行时挂载管理、目标根目录（chroot）准备与 ELF 动态链接器诊断模块。
//!
//! # 模块职责与架构定位
//! 本模块位于 `contest-run` 执行层（`execute`）与测试选择层（`selection`）的底层，负责：
//! 1. **挂载生命周期管理（[`Mounts`]）**：封装 `mount(2)` 与 `umount(2)` 系统调用，按挂载顺序记录已创建的挂载点，并在退出或切换根目录时按后进先出（LIFO）逆序卸载。
//! 2. **受限根目录路径解析（[`rooted_path`]）**：在不依赖内核 `chroot` 的宿主机或预处理阶段，将目标根目录（`root`）内的相对路径、绝对路径及符号链接（Symlink）安全地约束在 `root` 子树下，供 `selection` 与本模块读取文件。
//! 3. **ELF `PT_INTERP` 诊断（[`diagnose`]）**：当子进程 `execve` 启动失败时，解析目标可执行文件的 ELF64 小端程序头（Program Header），检查动态链接器（Interpreter）路径是否存在于目标根目录中，辅助定位 `ENOENT`（缺少动态链接器 vs 缺少可执行文件本身）等常见环境问题。
//!
//! # 核心机制与数据布局
//!
//! ## 1. 受限根路径解析状态机（[`rooted_path`]）
//! ```text
//! 输入 (root, cwd, path)
//!   │
//!   ├─ path 为绝对路径 ──► initial = path
//!   └─ path 为相对路径 ──► initial = cwd.join(path)
//!                              │
//!                              ▼
//!               拆分为待处理路径组件队列 (pending: VecDeque)
//!                              │
//!        ┌─────────────────────┴─────────────────────┐
//!        ▼                                           ▼
//!  RootDir ("/") ──► 清空 resolved           ParentDir ("..") ──► resolved.pop()
//!                                                    │            （在空路径上 pop 为空操作，
//!                                                    │              防止通过 ".." 逃逸出 root）
//!                                                    ▼
//!                                            Normal(name) ──► resolved.push(name)
//!                                                    │
//!                                                    ├─ 非符号链接 / 不存在 ──► 继续处理下一组件
//!                                                    └─ 是符号链接 (links <= 40)
//!                                                         ├─ resolved.pop() 移除当前链接名
//!                                                         └─ 将 read_link 目标组件逆序压回 pending 队首
//! ```
//!
//! ## 2. ELF64 小端头部与 `PT_INTERP` 段布局
//! ```text
//! ELF64 Header (64 字节):
//! +--------+--------+-----------------------------------------------+
//! | 偏移   | 长度   | 字段说明                                      |
//! +--------+--------+-----------------------------------------------+
//! | 0..4   | 4 B    | e_ident[EI_MAG0..3]: 魔数 "\x7fELF"           |
//! | 4      | 1 B    | e_ident[EI_CLASS]: 2 表示 64 位 (ELFCLASS64)  |
//! | 5      | 1 B    | e_ident[EI_DATA]: 1 表示小端序 (ELFDATA2LSB)  |
//! | 32..40 | 8 B    | e_phoff: 程序头表在文件中的字节偏移           |
//! | 54..56 | 2 B    | e_phentsize: 单个程序头表项字节大小 (>= 56)   |
//! | 56..58 | 2 B    | e_phnum: 程序头表项数量 (诊断上限 <= 128)     |
//! +--------+--------+-----------------------------------------------+
//!
//! ELF64 Program Header (至少前 56 字节):
//! +--------+--------+-----------------------------------------------+
//! | 偏移   | 长度   | 字段说明                                      |
//! +--------+--------+-----------------------------------------------+
//! | 0..4   | 4 B    | p_type: 3 表示 PT_INTERP 动态链接器路径段     |
//! | 8..16  | 8 B    | p_offset: 段内容在文件中的字节偏移            |
//! | 32..40 | 8 B    | p_filesz: 段内容在文件中的字节长度 (2..=4096) |
//! +--------+--------+-----------------------------------------------+
//! ```
//!
//! # 锁获取顺序与并发契约
//! 本模块在单线程控制流中顺序执行，不持有任何进程内互斥锁或自旋锁；文件系统的挂载表状态由内核 VFS 层同步维护。

use crate::execute::run_steps;
use crate::model::{CommandSpec, RootSpec};
use std::collections::{BTreeMap, VecDeque};
use std::ffi::{CString, OsString};
use std::fs::{self, File};
use std::io::{self, Read, Seek, SeekFrom};
use std::os::fd::RawFd;
use std::os::unix::ffi::OsStrExt;
use std::path::{Component, Path, PathBuf};
use std::time::Duration;

/// 比赛测试盘在目标系统中的默认挂载点路径。
pub const CONTEST_ROOT: &str = "/mnt/contest";

/// 已挂载文件系统的所有权跟踪器。
///
/// 记录当前执行器成功挂载的目标目录列表，以便在阶段结束或发生错误时按挂载的逆序逐一卸载。
#[derive(Default)]
pub struct Mounts {
    /// 当前实例已成功挂载且尚未卸载的目标挂载点路径栈（按挂载先后顺序排列）。
    owned: Vec<PathBuf>,
}

impl Mounts {
    /// 创建目标挂载点目录并调用 `mount(2)` 挂载指定文件系统。
    ///
    /// # 参数
    /// - `source`：挂载源设备路径或伪文件系统名称（如 `devfs`、`proc`、`tmpfs`）。
    /// - `target`：挂载点目标目录路径；若目录不存在会先行递归创建。
    /// - `filesystem`：传给内核的文系统类型名称字符串。
    /// - `readonly`：为 `true` 时附加 `MS_RDONLY` 标志以只读方式挂载。
    ///
    /// # 返回值
    /// 挂载成功时将 `target` 压入内部 `owned` 栈并返回 `Ok(())`；系统调用失败时返回对应的 `io::Error`。
    pub fn mount(
        &mut self,
        source: &Path,
        target: &Path,
        filesystem: &str,
        readonly: bool,
    ) -> io::Result<()> {
        // Step 1: 确保挂载目标目录在文件系统中存在，并将路径与文件系统名转换为 C 风格 NUL 结尾字符串。
        fs::create_dir_all(target)?;
        let source_c = CString::new(source.as_os_str().as_bytes())?;
        let target_c = CString::new(target.as_os_str().as_bytes())?;
        let filesystem = CString::new(filesystem)?;
        let flags = if readonly { libc::MS_RDONLY } else { 0 };
        // Step 2: 陷入内核执行挂载系统调用。
        // SAFETY:
        // 1. `source_c`、`target_c` 和 `filesystem` 均由 `CString::new` 构造，内部不含内部 NUL 字节且以 NUL 结尾；
        // 2. 三个 `CString` 变量在整个 `libc::mount` 调用期间保持存活，其裸指针有效且只读；
        // 3. 挂载不携带文件系统私有数据选项，第五个参数 `data` 传空指针符合 `mount(2)` 接口规范。
        let result = unsafe {
            libc::mount(
                source_c.as_ptr(),
                target_c.as_ptr(),
                filesystem.as_ptr(),
                flags,
                std::ptr::null(),
            )
        };
        if result == -1 {
            return Err(io::Error::last_os_error());
        }
        // Step 3: 仅在内核挂载成功后登记目标路径，确保后续 `release` 只卸载由本实例建立的挂载点。
        self.owned.push(target.to_path_buf());
        Ok(())
    }

    /// 按挂载的逆序（后进先出）逐个卸载当前实例持有的所有挂载点。
    ///
    /// # 返回值
    /// 全部卸载成功时返回 `Ok(())`；若某次 `umount(2)` 失败，立即中止并返回包含挂载点路径与系统错误信息的 `Err`，尚未卸载的挂载点仍保留在 `self.owned` 中。
    pub fn release(&mut self) -> Result<(), String> {
        while let Some(target) = self.owned.last() {
            let target_c =
                CString::new(target.as_os_str().as_bytes()).map_err(|error| error.to_string())?;
            // SAFETY:
            // 1. `target_c` 由 `CString::new` 从合法路径字节构造，保证以 NUL 结尾且无内部 NUL 字节；
            // 2. `target_c` 在 `libc::umount` 调用期间保持存活，其指针有效。
            if unsafe { libc::umount(target_c.as_ptr()) } == -1 {
                return Err(format!(
                    "unmount {}: {}",
                    target.display(),
                    io::Error::last_os_error()
                ));
            }
            self.owned.pop();
        }
        Ok(())
    }

    /// 初始化目标系统根目录：依次执行根目录准备命令，并挂载配置声明的伪文件系统。
    ///
    /// # 参数
    /// - `root`：目标根目录规格配置（包含宿主机路径 `path`、准备命令列表 `prepare` 及需挂载的子目录名 `mounts`）。
    /// - `timeout`：每条准备命令的执行超时时长。
    /// - `outputs`：准备命令的子进程标准输出与标准错误重定向文件描述符。
    ///
    /// # 返回值
    /// 准备命令与所有挂载点均成功完成时返回 `Ok(())`；任意步骤失败时返回描述性错误字符串。
    pub fn prepare_root(
        &mut self,
        root: &RootSpec,
        timeout: Duration,
        outputs: [RawFd; 2],
    ) -> Result<(), String> {
        // Step 1: 校验目标根路径是否已经存在且为目录。
        if !root.path.is_dir() {
            return Err(format!("root is not a directory: {}", root.path.display()));
        }
        // Step 2: 在宿主根环境 "/" 下按序执行该 root 配置的预处理命令（如解压或目录初始化）。
        let errors = run_steps(
            None,
            Path::new("/"),
            &root.prepare,
            &BTreeMap::new(),
            timeout,
            outputs,
            true,
        );
        if let Some(error) = errors.into_iter().next() {
            return Err(format!("root prepare: {error}"));
        }
        // Step 3: 仅按配置显式列出的挂载项（支持 dev -> devfs, proc -> proc, tmp -> tmpfs）挂载到目标根内部。
        for name in &root.mounts {
            let filesystem = match name.as_str() {
                "dev" => "devfs",
                "proc" => "proc",
                "tmp" => "tmpfs",
                _ => return Err(format!("unsupported root mount: {name}")),
            };
            let target = rooted_path(&root.path, Path::new("/"), Path::new(name))
                .map_err(|error| format!("mount target {name}: {error}"))?;
            self.mount(Path::new(filesystem), &target, filesystem, false)
                .map_err(|error| format!("mount {}: {error}", target.display()))?;
        }
        Ok(())
    }
}

/// 将目标根目录内的路径 `path`（基于工作目录 `cwd`）解析为宿主机文件系统上的实际路径。
///
/// 解析过程逐组件遍历路径：
/// - 遇到 `..` 时最多回退到 `root` 根部，防止路径穿越逃逸到 `root` 外部；
/// - 遇到符号链接时在 `root` 视图下读取链接目标并展开（绝对符号链接相对于 `root` 重新解析），最多跟随 40 层符号链接，超过上限返回 `ELOOP`。
pub(crate) fn rooted_path(root: &Path, cwd: &Path, path: &Path) -> io::Result<PathBuf> {
    // Step 1: 若输入为相对路径则拼接在 `cwd` 之后，随后拆解为待解析的路径组件队列。
    let initial = if path.is_absolute() {
        path.to_path_buf()
    } else {
        cwd.join(path)
    };
    let mut pending: VecDeque<OsString> = initial
        .components()
        .map(|part| part.as_os_str().to_os_string())
        .collect();
    let mut resolved = PathBuf::new();
    let mut links = 0;
    // Step 2: 逐个消费路径组件，维护相对于 `root` 的规范化子路径 `resolved`。
    while let Some(part) = pending.pop_front() {
        match Path::new(&part).components().next() {
            Some(Component::RootDir) => resolved.clear(),
            Some(Component::CurDir) | None => {}
            Some(Component::ParentDir) => {
                resolved.pop();
            }
            Some(Component::Normal(name)) => {
                resolved.push(name);
                let system_path = root.join(&resolved);
                match fs::symlink_metadata(&system_path) {
                    Ok(metadata) if metadata.file_type().is_symlink() => {
                        links += 1;
                        if 40 < links {
                            return Err(io::Error::from_raw_os_error(libc::ELOOP));
                        }
                        let target = fs::read_link(system_path)?;
                        resolved.pop();
                        for part in target.components().rev() {
                            pending.push_front(part.as_os_str().to_os_string());
                        }
                    }
                    Ok(_) => {}
                    Err(error) if error.kind() == io::ErrorKind::NotFound => {}
                    Err(error) => return Err(error),
                }
            }
            _ => {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    "unsupported path prefix",
                ))
            }
        }
    }
    Ok(root.join(resolved))
}

/// 解析 ELF64 小端可执行文件的程序头表，提取 `PT_INTERP` 动态链接器路径字节串。
///
/// # 参数
/// - `path`：宿主机文件系统中待检查的可执行文件路径。
///
/// # 返回值
/// - `Ok(Some(bytes))`：找到合法的 `PT_INTERP` 段，返回去除末尾 NUL 后的解释器路径字节序列；
/// - `Ok(None)`：文件不是 ELF 文件，或虽然是 ELF64 文件但不包含 `PT_INTERP` 段（如静态链接程序或脚本）；
/// - `Err(io::Error)`：文件读取失败、非 64 位小端 ELF，或程序头数量/大小超出诊断安全边界。
fn interpreter(path: &Path) -> io::Result<Option<Vec<u8>>> {
    // Step 1: 读取文件前 64 字节并校验 ELF 魔数（"\x7fELF"）及 64 位小端格式标识。
    let mut file = File::open(path)?;
    let mut header = [0u8; 64];
    let count = file.read(&mut header)?;
    if count < 4 || &header[..4] != b"\x7fELF" {
        return Ok(None);
    }
    if count < header.len() || header[4] != 2 || header[5] != 1 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "diagnostic supports ELF64 little-endian only",
        ));
    }
    // Step 2: 解析 ELF 头中的程序头表偏移（e_phoff）、单表项大小（e_phentsize）与表项数量（e_phnum）。
    let offset = u64::from_le_bytes(header[32..40].try_into().unwrap());
    let size = u16::from_le_bytes(header[54..56].try_into().unwrap()) as u64;
    let count = u16::from_le_bytes(header[56..58].try_into().unwrap()) as u64;
    if size < 56 || 128 < count {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "program-header diagnostic limit exceeded",
        ));
    }
    // Step 3: 顺序扫描程序头表，查找类型为 `PT_INTERP`（p_type == 3）的段并读取其路径内容。
    for index in 0..count {
        let entry = offset.checked_add(index * size).ok_or_else(|| {
            io::Error::new(io::ErrorKind::InvalidData, "program-header offset overflow")
        })?;
        file.seek(SeekFrom::Start(entry))?;
        let mut ph = [0u8; 56];
        file.read_exact(&mut ph)?;
        if u32::from_le_bytes(ph[..4].try_into().unwrap()) != 3 {
            continue;
        }
        let offset = u64::from_le_bytes(ph[8..16].try_into().unwrap());
        let size = u64::from_le_bytes(ph[32..40].try_into().unwrap());
        if size < 2 || 4096 < size {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "PT_INTERP diagnostic limit exceeded",
            ));
        }
        let mut bytes = vec![0; size as usize];
        file.seek(SeekFrom::Start(offset))?;
        file.read_exact(&mut bytes)?;
        if bytes.pop() != Some(0) || bytes.contains(&0) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "invalid PT_INTERP terminator",
            ));
        }
        return Ok(Some(bytes));
    }
    Ok(None)
}

/// 针对启动失败的命令，检查其目标可执行文件的 ELF `PT_INTERP` 动态链接器在目标根目录中是否存在。
///
/// # 参数
/// - `root`：命令所属的目标根目录规格。
/// - `cwd`：命令在目标根目录内的工作目录。
/// - `command`：待诊断的 `Exec` 或 `Shell` 命令规格。
///
/// # 返回值
/// 返回一行人类可读的 `PT_INTERP` 诊断信息字符串，供测试报告附加到 `SpawnError` 详情中。
pub fn diagnose(root: &RootSpec, cwd: &Path, command: &CommandSpec) -> String {
    // Step 1: 提取待执行程序路径；若不含 '/' 则说明依赖子进程 PATH 搜索，无法直接定位文件。
    let program = match command {
        CommandSpec::Exec { program, .. } => program,
        CommandSpec::Shell { shell, .. } => shell,
    };
    if !program.contains('/') {
        return "PT_INTERP unavailable: program requires child PATH lookup".into();
    }
    // Step 2: 在目标根内解析程序路径并提取其 ELF `PT_INTERP` 段，再检查该解释器路径在目标根内是否存在。
    let result =
        rooted_path(&root.path, cwd, Path::new(program)).and_then(|path| interpreter(&path));
    match result {
        Ok(None) => "PT_INTERP none (ELF static or non-ELF input)".into(),
        Ok(Some(bytes)) => {
            let interp = Path::new(std::ffi::OsStr::from_bytes(&bytes));
            let availability =
                rooted_path(&root.path, Path::new("/"), interp).and_then(fs::metadata);
            match availability {
                Ok(_) => format!("PT_INTERP {:?} present in target root", interp),
                Err(error) => format!("PT_INTERP {:?}: {error}", interp),
            }
        }
        Err(error) => format!("PT_INTERP diagnostic unavailable: {error}"),
    }
}
