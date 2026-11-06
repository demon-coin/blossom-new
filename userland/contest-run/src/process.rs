//! 子进程生命周期管理、进程组超时控制与非阻塞输出流转发模块。
//!
//! ## 模块职责与架构定位
//! 本模块负责在指定的测试根（`chroot`）、工作目录（`chdir`）与环境变量下启动测试命令或系统维护命令，
//! 并在统一的单调时钟绝对截止期限（`deadline`）内完成进程监控、输出捕获转发与超时强杀回收。
//! 上游由 `execute`、`runtime` 与 `main` 调用 `run_command`，下游将捕获到的字节流喂给 `report::ProtocolObserver` 进行协议解析。
//!
//! ## 核心机制：双层管道与独立 Relay 辅助进程
//! 对于需要解析输出正文的协议（`Basic`、`Pre2025Libctest`、`Tap`），不能让主进程直接阻塞写向外部标准输出/错误（如串口终端），
//! 否则当终端写阻塞时主进程无法及时响应 `deadline` 超时信号。同时，也不能直接修改外部 `stdout`/`stderr` 的打开文件描述（OFD）为 `O_NONBLOCK`，
//! 因为外部 `stdout`/`stderr`/`stdin` 通常共享同一个底层终端 OFD，改写 `O_NONBLOCK` 会污染被测子进程继承的 `stdin`。
//!
//! 因此本模块采用“主进程非阻塞环形缓冲 + 独立 `Relay` 子进程阻塞写外部终端”的双层架构：
//!
//! ```text
//!   ┌────────────────┐  stdout/stderr 管道 (O_NONBLOCK)  ┌──────────────────────────────────┐
//!   │  被测子进程组   │ ────────────────────────────────► │         contest-run 主进程        │
//!   │ (独立 pgid=pid) │                                   │  1. 喂给 ProtocolObserver 解析    │
//!   └────────────────┘                                   │  2. 存入 64 KiB 背压队列 (Flow)   │
//!                                                        │  3. 监控 deadline 并发 SIGTERM/KILL│
//!                                                        └────────────────┬─────────────────┘
//!                                                                         │ relay 管道 (写端 O_NONBLOCK)
//!                                                                         ▼
//!                                                        ┌──────────────────────────────────┐
//!                                                        │      Relay 辅助子进程 (fork)      │
//!                                                        │  纯异步信号安全循环 (poll/read/write)│
//!                                                        │  阻塞写向外部 outputs [fd 1, fd 2] │
//!                                                        └──────────────────────────────────┘
//! ```
//!
//! 超时状态转换（`pump` 循环）：
//! 1. **正常运行**：轮询子进程退出状态与管道 I/O，每轮 `poll` 超时上限 50ms。
//! 2. **触发超时（`now >= deadline`）**：向整个被测进程组（`-pid`）发送 `SIGTERM`，设置 `terminate_at = now + 200ms`。
//! 3. **升级强杀（`now >= terminate_at`）**：若进程组仍未退尽，向进程组发送 `SIGKILL`，设置 `kill_at = now + 200ms`。
//! 4. **强制收尾（`now >= kill_at`）**：停止等待残留管道，强杀并回收 `Relay` 辅助进程，阻塞 `wait` 回收被测主进程。
//!
//! ## 并发与锁契约
//! - 主进程内部在单线程事件循环中通过 `poll` 复用 I/O，不持有跨线程锁。
//! - `command.pre_exec` 闭包与 `relay_child` 运行在 `fork` 后的子进程上下文中，严禁执行堆内存分配、Rust 锁获取或任何非异步信号安全（Async-Signal-Safe）操作，遇到致命错误必须调用 `libc::_exit` 直接终止。

use crate::model::{CommandSpec, Protocol};
use crate::outcome::{Execution, ExitOutcome, Verdict};
use crate::report::{ProtocolObserver, StreamKind};
use std::collections::{BTreeMap, VecDeque};
use std::ffi::CString;
use std::io;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd, RawFd};
use std::os::unix::ffi::OsStrExt;
use std::os::unix::process::{CommandExt, ExitStatusExt};
use std::path::Path;
use std::process::{Child, Command, ExitStatus, Stdio};
use std::time::{Duration, Instant};

/// 将操作系统的 `ExitStatus` 转换为结构化的 `ExitOutcome`（区分正常退出码与终止信号）。
fn exit_outcome(status: ExitStatus) -> ExitOutcome {
    let raw = status.into_raw();
    let status = ExitStatus::from_raw(raw);
    match status.code() {
        Some(code) => ExitOutcome::Exited { code, raw },
        None => ExitOutcome::Signaled {
            signal: status.signal().unwrap_or(0),
            raw,
        },
    }
}

/// 构造待执行的 `std::process::Command`，并在 `fork` 前准备好全部路径 `CString` 与标准流重定向。
///
/// 若 `capture` 为 `true`，子进程的 `stdout`/`stderr` 重定向到匿名管道供主进程捕获解析；
/// 若 `capture` 为 `false`，通过 `F_DUPFD_CLOEXEC` 复制 `outputs` 描述符直接交给子进程写向终端。
/// 子进程在 `exec` 前通过 `pre_exec` 依次执行 `chroot`（若指定）、`chdir` 与 `setpgid(0, 0)` 建立独立进程组。
fn command(
    root: Option<&Path>,
    cwd: &Path,
    spec: &CommandSpec,
    env: &BTreeMap<String, String>,
    capture: bool,
    outputs: [RawFd; 2],
) -> io::Result<Command> {
    let root = root
        .map(|path| CString::new(path.as_os_str().as_bytes()))
        .transpose()?;
    let cwd = CString::new(cwd.as_os_str().as_bytes())?;
    let mut command = match spec {
        CommandSpec::Exec { program, args } => {
            let mut command = Command::new(program);
            command.args(args);
            command
        }
        CommandSpec::Shell {
            shell,
            command: text,
        } => {
            let mut command = Command::new(shell);
            command.arg("-c").arg(text);
            command
        }
    };
    command.envs(env).stdin(Stdio::inherit());
    if capture {
        command.stdout(Stdio::piped()).stderr(Stdio::piped());
    } else {
        command.stdout(Stdio::from(duplicate_output(outputs[0])?));
        command.stderr(Stdio::from(duplicate_output(outputs[1])?));
    }
    // SAFETY:
    // 1. `root` 与 `cwd` 的 `CString` 已在 `fork` 前于父进程内分配并校验完毕，闭包内只读取其有效 C 字符串指针。
    // 2. 闭包运行在 `fork` 后、`exec` 前的子进程环境中，不进行任何堆内存分配或锁操作，仅调用 POSIX 异步信号安全的 `chroot`、`chdir` 与 `setpgid`。
    unsafe {
        command.pre_exec(move || {
            if let Some(root) = &root {
                if libc::chroot(root.as_ptr()) == -1 {
                    return Err(io::Error::last_os_error());
                }
            }
            if libc::chdir(cwd.as_ptr()) == -1 {
                return Err(io::Error::last_os_error());
            }
            if libc::setpgid(0, 0) == -1 {
                return Err(io::Error::last_os_error());
            }
            Ok(())
        });
    }
    Ok(command)
}

/// 使用 `F_DUPFD_CLOEXEC` 复制输出文件描述符（描述符编号大于等于 3）。
///
/// 仅复制描述符本身并设置 `FD_CLOEXEC`，不修改底层打开文件描述（OFD）的状态标志（如 `O_NONBLOCK`），
/// 从而避免影响与之共享同一个终端 OFD 的标准输入（`stdin`）。
fn duplicate_output(fd: RawFd) -> io::Result<OwnedFd> {
    // SAFETY: `fd` 由调用方保证在当前进程内有效且打开；`F_DUPFD_CLOEXEC` 仅读取该描述符并分配一个 >= 3 的新文件描述符，不涉及内存指针解引用。
    let copied = unsafe { libc::fcntl(fd, libc::F_DUPFD_CLOEXEC, 3) };
    if copied == -1 {
        return Err(io::Error::last_os_error());
    }
    // SAFETY: 前置检查已排除 `copied == -1`，此时 `copied` 是内核刚刚分配且尚未交由任何其他包装器管理的新文件描述符，可安全移交所有权给 `OwnedFd`。
    Ok(unsafe { OwnedFd::from_raw_fd(copied) })
}

/// 将当前进程独占的匿名管道端点设置为非阻塞模式（`O_NONBLOCK`）。
///
/// 仅用于本次命令新建的私有管道端点，这些端点在命令结束关闭时即销毁，无需在退出时恢复原标志。
fn nonblocking_pipe(fd: RawFd) -> io::Result<()> {
    // SAFETY: 调用方传入的 `fd` 是当前进程独占持有的有效管道描述符；`F_GETFL` 无需额外指针参数。
    let flags = unsafe { libc::fcntl(fd, libc::F_GETFL) };
    if flags == -1 {
        return Err(io::Error::last_os_error());
    }
    // SAFETY: `fd` 为独占管道端点，`flags` 为上一行 `F_GETFL` 成功读取的标志位；设置 `O_NONBLOCK` 仅影响该私有管道，不会污染标准输入/输出共享的终端文件状态。
    if unsafe { libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK) } == -1 {
        return Err(io::Error::last_os_error());
    }
    Ok(())
}

/// 创建一对带有 `O_CLOEXEC` 标志的匿名管道，并将其读写两端封装为具有唯一所有权的 `OwnedFd`。
fn relay_pipe() -> io::Result<[OwnedFd; 2]> {
    let mut fds = [-1; 2];
    // SAFETY: `fds` 是栈上长度为 2 的可写 `c_int` 数组，满足 `pipe2` 对 `int pipefd[2]` 缓冲区有效性与对齐的要求。
    if unsafe { libc::pipe2(fds.as_mut_ptr(), libc::O_CLOEXEC) } == -1 {
        return Err(io::Error::last_os_error());
    }
    // SAFETY: `pipe2` 返回非 -1 保证了 `fds[0]`（读端）与 `fds[1]`（写端）均为内核新分配且互不相同的有效文件描述符，此前无其他所有者。
    Ok(unsafe { [OwnedFd::from_raw_fd(fds[0]), OwnedFd::from_raw_fd(fds[1])] })
}

/// 负责将主进程缓冲的输出字节异步写入外部真实输出描述符的辅助子进程控制器。
struct Relay {
    /// 辅助子进程的 PID；当辅助进程已被 `waitpid` 回收后置为 `None`。
    pid: Option<libc::pid_t>,
    /// 主进程向辅助子进程发送 `stdout` 和 `stderr` 数据的非阻塞管道写端（关闭即向辅助进程发送 EOF）。
    writers: [Option<OwnedFd>; 2],
    /// 辅助子进程退出时的原始 `waitpid` 状态字。
    status: Option<i32>,
}

impl Relay {
    /// 创建两条转发管道并 `fork` 启动输出转发辅助子进程。
    ///
    /// 所有管道、栈缓冲区与线程本地 `errno` 地址均在 `fork` 前准备就绪；
    /// 子进程仅执行纯系统调用转发循环，不产生孙进程，也不触碰 Rust 运行时状态。
    fn new(outputs: [RawFd; 2]) -> io::Result<Self> {
        let [read_stdout, write_stdout] = relay_pipe()?;
        let [read_stderr, write_stderr] = relay_pipe()?;
        nonblocking_pipe(write_stdout.as_raw_fd())?;
        nonblocking_pipe(write_stderr.as_raw_fd())?;
        let readers = [read_stdout.as_raw_fd(), read_stderr.as_raw_fd()];
        let writers = [write_stdout.as_raw_fd(), write_stderr.as_raw_fd()];
        let mut buffers = [[0u8; 4096]; 2];
        let mut fds = [
            libc::pollfd {
                fd: readers[0],
                events: libc::POLLIN,
                revents: 0,
            },
            libc::pollfd {
                fd: readers[1],
                events: libc::POLLIN,
                revents: 0,
            },
        ];
        // SAFETY: `libc::__errno_location()` 返回当前线程 TLS 中的 `errno` 整型指针；`fork` 后子进程继承相同的虚拟地址映射且单线程运行，可在子进程中直接解引用读取系统调用错误码。
        let errno = unsafe { libc::__errno_location() };
        // SAFETY:
        // 1. 当前程序为单线程运行，`fork` 前已完成全部管道描述符与栈缓冲区的构造。
        // 2. 子进程分支（`pid == 0`）立即进入永不返回的 `relay_child`，并在退出时调用 `libc::_exit`，绝不执行父进程栈上对象的 Rust `Drop` 析构或堆内存分配。
        let pid = unsafe { libc::fork() };
        if pid == -1 {
            return Err(io::Error::last_os_error());
        }
        if pid == 0 {
            // SAFETY: 当前处于 `fork` 后的子进程分支，`writers`、`outputs`、`fds`、`buffers` 与 `errno` 指针均在 `fork` 前初始化完毕且在子进程地址空间中独占有效。
            unsafe { relay_child(writers, outputs, &mut fds, &mut buffers, errno) }
        }
        drop(read_stdout);
        drop(read_stderr);
        Ok(Self {
            pid: Some(pid),
            writers: [Some(write_stdout), Some(write_stderr)],
            status: None,
        })
    }

    /// 返回主进程向辅助子进程写入 `stdout` 和 `stderr` 的非阻塞管道写端原始描述符。
    fn outputs(&self) -> [RawFd; 2] {
        [
            self.writers[0].as_ref().unwrap().as_raw_fd(),
            self.writers[1].as_ref().unwrap().as_raw_fd(),
        ]
    }

    /// 关闭主进程持有的两条转发管道写端，使辅助子进程在读尽管道残留数据后收到 EOF 并正常退出。
    fn close_inputs(&mut self) {
        self.writers[0].take();
        self.writers[1].take();
    }

    /// 以非阻塞方式（`WNOHANG`）检查并回收辅助子进程；若辅助子进程异常退出或未排空即退出，将错误信息记入 `errors`。
    fn check(&mut self, errors: &mut Vec<String>) {
        let Some(pid) = self.pid else {
            return;
        };
        let mut raw = 0;
        // SAFETY: `pid` 来自本实例 `Relay::new` 创建且尚未回收的直接子进程，`raw` 为栈上有效的可写 `c_int` 变量。
        let result = unsafe { libc::waitpid(pid, &mut raw, libc::WNOHANG) };
        if result == 0 {
            return;
        }
        if result == -1 {
            let error = io::Error::last_os_error();
            if error.kind() == io::ErrorKind::Interrupted {
                return;
            }
            errors.push(format!("output relay wait: {error}"));
            if error.raw_os_error() == Some(libc::ECHILD) {
                self.pid = None;
            }
            return;
        }
        self.pid = None;
        self.status = Some(raw);
        if raw != 0 || self.writers.iter().any(Option::is_some) {
            errors.push(format!(
                "output relay exited before successful drain: raw={raw}"
            ));
        }
    }

    /// 判断辅助子进程是否已在未正常排空的情况下异常终止（此时主进程应停止向转发队列追加数据，仅继续排空被测子进程管道）。
    fn failed(&self) -> bool {
        self.pid.is_none() && (self.status != Some(0) || self.writers.iter().any(Option::is_some))
    }

    /// 强制终止并阻塞回收辅助子进程（用于超时到期或 `Drop` 清理，防止外部终端阻塞导致辅助进程残留）。
    fn stop(&mut self) -> io::Result<()> {
        self.close_inputs();
        let Some(pid) = self.pid else {
            return Ok(());
        };
        // SAFETY: `self.pid` 为 `Some` 保证该辅助子进程尚未被 `waitpid` 回收，PID 仍有效且未发生重用；辅助进程内部不创建后代，直接向 `pid` 发送 `SIGKILL` 即可终止。
        if unsafe { libc::kill(pid, libc::SIGKILL) } == -1 {
            let error = io::Error::last_os_error();
            if error.raw_os_error() != Some(libc::ESRCH) {
                return Err(error);
            }
        }
        loop {
            let mut raw = 0;
            // SAFETY: `pid` 仍由当前 `Relay` 实例独占，直到本次阻塞 `waitpid` 成功返回后才将 `self.pid` 置为 `None`；`raw` 为栈上可写的 `c_int`。
            if unsafe { libc::waitpid(pid, &mut raw, 0) } == pid {
                self.pid = None;
                self.status = Some(raw);
                return Ok(());
            }
            let error = io::Error::last_os_error();
            if error.kind() == io::ErrorKind::Interrupted {
                continue;
            }
            if error.raw_os_error() == Some(libc::ECHILD) {
                self.pid = None;
            }
            return Err(error);
        }
    }
}

impl Drop for Relay {
    /// 析构时确保关闭转发管道并终止、回收辅助子进程，避免后续命令启动失败时遗留孤儿或僵尸进程。
    fn drop(&mut self) {
        let _ = self.stop();
    }
}

/// 输出转发辅助子进程的主循环函数（在 `fork` 子分支中运行，永不返回，最终调用 `_exit` 退出）。
///
/// # Safety
/// 1. 必须在 `fork()` 返回 `0` 的子进程上下文中调用，且调用后不得再返回 Rust 栈帧执行任何析构函数。
/// 2. `writers` 必须是继承自父进程的两条转发管道写端描述符，子进程进入本函数后会立即将其关闭。
/// 3. `outputs` 必须是有效的目标输出文件描述符（如外部 `stdout`/`stderr`），在辅助进程存活期间保持有效。
/// 4. `fds` 与 `buffers` 必须是子进程栈上独占的可写数组引用，`errno` 必须指向当前线程有效的 `errno` 存储单元。
unsafe fn relay_child(
    writers: [RawFd; 2],
    outputs: [RawFd; 2],
    fds: &mut [libc::pollfd; 2],
    buffers: &mut [[u8; 4096]; 2],
    errno: *mut libc::c_int,
) -> ! {
    for fd in writers {
        // SAFETY: `fd` 是 `fork` 复制到子进程描述符表中的管道写端副本；子进程只读不写，立即关闭副本以便父进程关闭写端时能正常触发 EOF。
        unsafe {
            libc::close(fd);
        }
    }
    let mut open = 2;
    while 0 < open {
        // SAFETY: `fds` 是栈上长度为 2 的有效 `pollfd` 数组；传入 `-1` 表示无限期等待，直到管道可读、EOF 或被父进程 `SIGKILL` 终止。
        if unsafe { libc::poll(fds.as_mut_ptr(), 2, -1) } == -1 {
            // SAFETY: `errno` 指向当前子进程有效的线程本地错误码变量，紧接在 `poll` 返回 -1 后读取。
            if unsafe { *errno } == libc::EINTR {
                continue;
            }
            // SAFETY: `_exit` 是异步信号安全的系统调用，直接终止子进程而不触碰父进程继承来的 Rust 堆或对象析构。
            unsafe {
                libc::_exit(1);
            }
        }
        for index in 0..2 {
            if fds[index].fd == -1 || fds[index].revents == 0 {
                continue;
            }
            // SAFETY: `fds[index].fd` 在前面已检查不为 -1（尚未关闭），`buffers[index]` 是栈上 4096 字节的可写数组，指针与长度完全匹配。
            let count = unsafe {
                libc::read(
                    fds[index].fd,
                    buffers[index].as_mut_ptr().cast(),
                    buffers[index].len(),
                )
            };
            if count == 0 {
                // SAFETY: `read` 返回 0 表示该条转发管道已达 EOF，关闭子进程持有的管道读端描述符并在下一行将其置为 -1，防止重复关闭。
                unsafe {
                    libc::close(fds[index].fd);
                }
                fds[index].fd = -1;
                open -= 1;
                continue;
            }
            if count < 0 {
                // SAFETY: `errno` 指向当前线程有效的错误码地址，读取刚才 `read` 失败的原因。
                if unsafe { *errno } == libc::EINTR {
                    continue;
                }
                // SAFETY: 发生不可恢复的管道读取错误，直接调用异步信号安全的 `_exit(1)` 终止辅助进程。
                unsafe {
                    libc::_exit(1);
                }
            }
            let mut offset = 0;
            while offset < count as usize {
                // SAFETY: 循环条件 `offset < count as usize` 且 `count <= buffers[index].len()` 保证了切片 `buffers[index][offset..]` 在有效范围内，剩余字节数 `count as usize - offset` 精确无越界。
                let written = unsafe {
                    libc::write(
                        outputs[index],
                        buffers[index][offset..].as_ptr().cast(),
                        count as usize - offset,
                    )
                };
                if 0 < written {
                    offset += written as usize;
                    continue;
                }
                if written < 0 {
                    // SAFETY: 紧接在 `write` 返回负数后读取当前线程有效的 `errno` 指针。
                    let error = unsafe { *errno };
                    if error == libc::EINTR {
                        continue;
                    }
                    if error == libc::EAGAIN {
                        let mut output = libc::pollfd {
                            fd: outputs[index],
                            events: libc::POLLOUT,
                            revents: 0,
                        };
                        // SAFETY: `output` 是栈上单个有效的 `pollfd` 结构，等待目标输出描述符恢复可写状态。
                        if unsafe { libc::poll(&mut output, 1, -1) } != -1 {
                            continue;
                        }
                        // SAFETY: 紧接在 `poll` 返回 -1 后解引用当前线程有效的 `errno` 指针检查是否为 `EINTR`。
                        if unsafe { *errno } == libc::EINTR {
                            continue;
                        }
                    }
                }
                // SAFETY: 遇到非 `EINTR`/`EAGAIN` 的写入失败或零字节写入，直接调用 `_exit(1)` 终止子进程，不执行任何 Rust 清理逻辑。
                unsafe {
                    libc::_exit(1);
                }
            }
        }
    }
    // SAFETY: 两条转发管道均已读到 EOF 且全部字节已完整写入目标描述符，调用 `_exit(0)` 正常结束辅助子进程。
    unsafe {
        libc::_exit(0);
    }
}

/// 主进程内部用于单个输出流（`stdout` 或 `stderr`）的非阻塞读取、协议观察与背压转发通道。
struct Flow {
    /// 来自被测子进程的管道读端原始描述符（已设为 `O_NONBLOCK`）。
    input: RawFd,
    /// 通往 `Relay` 辅助子进程的管道写端原始描述符（已设为 `O_NONBLOCK`）。
    output: RawFd,
    /// 标识当前通道属于标准输出（`Stdout`）还是标准错误（`Stderr`）。
    kind: StreamKind,
    /// 被测子进程管道读端是否已达 EOF（或因读取错误而关闭读取）。
    eof: bool,
    /// 向 `Relay` 管道写入是否已发生失败（若失败则丢弃后续待转发字节，但继续读取被测子进程输出以供协议观察并避免子进程写管道阻塞）。
    failed: bool,
    /// 暂存已从被测子进程读出、但尚未写入 `Relay` 管道的字节环形队列（上限约 64 KiB）。
    queue: VecDeque<u8>,
}

impl Flow {
    /// 为指定的输出流创建预分配 64 KiB 容量的非阻塞流控通道。
    fn new(input: RawFd, output: RawFd, kind: StreamKind) -> Self {
        Self {
            input,
            output,
            kind,
            eof: false,
            failed: false,
            queue: VecDeque::with_capacity(65536),
        }
    }

    /// 从被测子进程管道非阻塞读取至多 8 KiB 数据，立即送入 `observer` 解析，并在转发未失效时压入待写队列。
    ///
    /// 当队列积压超过 56 KiB（`57344` 字节）时暂停读取，形成背压以限制主进程内存占用；每次最多读取一块（8 KiB），防止高频输出饿死超时检查或另一条流。
    fn read(&mut self, observer: &mut ProtocolObserver, errors: &mut Vec<String>) {
        if self.eof || 57344 < self.queue.len() {
            return;
        }
        let mut bytes = [0u8; 8192];
        // SAFETY: `self.input` 由调用栈 `pump` 中的 `stdout`/`stderr` `ChildStdout`/`ChildStderr` 保活且有效；`bytes` 是栈上 8192 字节的可写数组。
        let count = unsafe { libc::read(self.input, bytes.as_mut_ptr().cast(), bytes.len()) };
        if count == 0 {
            self.eof = true;
        } else if 0 < count {
            let bytes = &bytes[..count as usize];
            observer.feed(self.kind, bytes);
            if !self.failed {
                self.queue.extend(bytes);
            }
        } else {
            let error = io::Error::last_os_error();
            if error.kind() != io::ErrorKind::WouldBlock
                && error.kind() != io::ErrorKind::Interrupted
            {
                errors.push(format!("read {:?}: {error}", self.kind));
                self.eof = true;
            }
        }
    }

    /// 将队列头部的连续字节切片非阻塞写入 `Relay` 管道；若发生短写则仅移除已写出的前缀，若写入出错则清空队列并标记 `failed`。
    fn forward(&mut self, errors: &mut Vec<String>) {
        let (bytes, _) = self.queue.as_slices();
        if bytes.is_empty() {
            return;
        }
        // SAFETY: `self.output` 由 `pump` 中的 `Relay` 实例保活且有效；`bytes` 为 `VecDeque::as_slices` 返回的非空连续可读字节切片，指针与长度匹配。
        let count = unsafe { libc::write(self.output, bytes.as_ptr().cast(), bytes.len()) };
        if 0 < count {
            self.queue.drain(..count as usize);
        } else if count == 0 {
            errors.push(format!("write {:?}: zero progress", self.kind));
            self.queue.clear();
            self.failed = true;
        } else {
            let error = io::Error::last_os_error();
            if error.kind() != io::ErrorKind::WouldBlock
                && error.kind() != io::ErrorKind::Interrupted
            {
                errors.push(format!("write {:?}: {error}", self.kind));
                self.queue.clear();
                self.failed = true;
            }
        }
    }

    /// 生成当前流用于 `poll` 等待的两个描述符项：`[读入端 pollfd, 写出端 pollfd]`。
    ///
    /// 若读端已 EOF 或缓冲队列处于高水位背压状态，则读端 `fd` 置为 `-1`（暂不监听可读）；若队列为空，则写端 `fd` 置为 `-1`（暂不监听可写）。
    fn poll_fds(&self) -> [libc::pollfd; 2] {
        [
            libc::pollfd {
                fd: if self.eof || 57344 < self.queue.len() {
                    -1
                } else {
                    self.input
                },
                events: libc::POLLIN,
                revents: 0,
            },
            libc::pollfd {
                fd: if self.queue.is_empty() {
                    -1
                } else {
                    self.output
                },
                events: libc::POLLOUT,
                revents: 0,
            },
        ]
    }
}

/// 向被测子进程所在的整个进程组（PGID 等于子进程 PID）发送指定信号。
///
/// 子进程在 `pre_exec` 中已调用 `setpgid(0, 0)` 成为进程组长，因此向 `-pid` 发送信号可一并终止其派生的全部后代进程；若返回 `ESRCH` 说明进程组已完全退出，忽略该错误。
fn signal_group(child: &Child, signal: i32, errors: &mut Vec<String>) {
    // SAFETY: `child.id()` 是当前尚未完全放弃管理的被测子进程正整数 PID；取负值 `-(child.id() as i32)` 符合 POSIX `kill` 向对应进程组广播信号的语义。
    if unsafe { libc::kill(-(child.id() as i32), signal) } == -1 {
        let error = io::Error::last_os_error();
        if error.raw_os_error() != Some(libc::ESRCH) {
            errors.push(format!("kill process group signal={signal}: {error}"));
        }
    }
}

/// 在统一截止时间 `deadline` 下驱动被测子进程与可选 `Relay` 辅助进程的事件循环，完成输出采集、转发与超时终止。
///
/// 返回被测子进程的最终 `ExitOutcome`，运行期间发生的管道、信号或辅助进程错误追加到 `errors` 列表中。
fn pump(
    child: &mut Child,
    deadline: Instant,
    observer: &mut ProtocolObserver,
    mut relay: Option<Relay>,
    errors: &mut Vec<String>,
) -> ExitOutcome {
    // Step 1: 接管被测子进程的 stdout/stderr 管道，若启用了捕获模式则将其设为非阻塞并绑定到双通道 Flow。
    let stdout = child.stdout.take();
    let stderr = child.stderr.take();
    let mut flows = if let Some(relay) = &relay {
        let stdout = stdout.as_ref().expect("captured stdout");
        let stderr = stderr.as_ref().expect("captured stderr");
        if let Err(error) =
            nonblocking_pipe(stdout.as_raw_fd()).and_then(|()| nonblocking_pipe(stderr.as_raw_fd()))
        {
            signal_group(child, libc::SIGKILL, errors);
            let raw = match child.wait() {
                Ok(status) => Some(status.into_raw()),
                Err(wait) => {
                    errors.push(format!("wait after pipe failure: {wait}"));
                    None
                }
            };
            return ExitOutcome::IoError {
                errno: error.raw_os_error(),
                message: error.to_string(),
                raw,
            };
        }
        let outputs = relay.outputs();
        Some([
            Flow::new(stdout.as_raw_fd(), outputs[0], StreamKind::Stdout),
            Flow::new(stderr.as_raw_fd(), outputs[1], StreamKind::Stderr),
        ])
    } else {
        None
    };
    // Step 2: 进入主事件循环，按单调时钟推进正常排空、超时 SIGTERM 宽限期与 SIGKILL 强制终止阶段。
    let mut status = None;
    let mut timed_out = false;
    let mut terminate_at = None;
    let mut kill_at = None;
    let mut wait_error = None;
    loop {
        let now = Instant::now();
        if !timed_out
            && deadline <= now
            && status.is_some()
            && flows
                .as_ref()
                .is_none_or(|flows| flows.iter().all(|flow| flow.eof && flow.queue.is_empty()))
        {
            if let Some(relay) = &mut relay {
                relay.check(errors);
                if relay.pid.is_some() {
                    errors.push("output relay did not drain before deadline".into());
                    if let Err(error) = relay.stop() {
                        errors.push(format!("stop output relay: {error}"));
                    }
                }
            }
            break;
        }
        if !timed_out && deadline <= now {
            timed_out = true;
            signal_group(child, libc::SIGTERM, errors);
            terminate_at = Some(now + Duration::from_millis(200));
        }
        if terminate_at.is_some_and(|at| at <= now) {
            signal_group(child, libc::SIGKILL, errors);
            terminate_at = None;
            kill_at = Some(now + Duration::from_millis(200));
        }
        // Step 3: 非阻塞轮询被测主子进程的退出状态。
        if status.is_none() && wait_error.is_none() {
            match child.try_wait() {
                Ok(found) => status = found,
                Err(error) if error.kind() == io::ErrorKind::Interrupted => {}
                Err(error) => {
                    errors.push(format!("try_wait: {error}"));
                    wait_error = Some(error);
                    signal_group(child, libc::SIGKILL, errors);
                    kill_at = Some(now);
                }
            }
        }
        // Step 4: 检查 Relay 辅助进程状态，并在两条流上执行非阻塞转发与读取。
        if let Some(relay) = &mut relay {
            relay.check(errors);
            if relay.failed() {
                if let Some(flows) = &mut flows {
                    for flow in flows {
                        flow.queue.clear();
                        flow.failed = true;
                    }
                }
                relay.close_inputs();
            }
        }
        if let Some(flows) = &mut flows {
            for flow in flows {
                flow.forward(errors);
                flow.read(observer, errors);
                flow.forward(errors);
            }
        }
        // Step 5: 若两条输出流均已 EOF 且队列排空，关闭 Relay 输入端并检查是否满足循环结束条件。
        let drained = flows
            .as_ref()
            .is_none_or(|flows| flows.iter().all(|flow| flow.eof && flow.queue.is_empty()));
        if drained {
            if let Some(relay) = &mut relay {
                relay.close_inputs();
                relay.check(errors);
            }
        }
        let relay_done = relay.as_ref().is_none_or(|relay| relay.pid.is_none());
        if status.is_some() && drained && relay_done && (!timed_out || kill_at.is_some()) {
            break;
        }
        // Step 6: 若已到达强制终止期限（kill_at），记录未排空警告、强杀 Relay 并阻塞回收被测子进程。
        if kill_at.is_some_and(|at| at <= now) {
            if let Some(flows) = &flows {
                for flow in flows {
                    if !flow.eof {
                        errors.push(format!(
                            "{:?} remains open after process-group stop",
                            flow.kind
                        ));
                    }
                    if !flow.queue.is_empty() {
                        errors.push(format!(
                            "{:?} output queue did not drain before deadline",
                            flow.kind
                        ));
                    }
                }
            }
            if let Some(relay) = &mut relay {
                if relay.pid.is_some() {
                    errors.push("output relay did not drain before deadline".into());
                    if let Err(error) = relay.stop() {
                        errors.push(format!("stop output relay: {error}"));
                    }
                }
            }
            if status.is_none() {
                match child.wait() {
                    Ok(found) => status = Some(found),
                    Err(error) => {
                        errors.push(format!("wait after stop: {error}"));
                        wait_error = Some(error);
                    }
                }
            }
            break;
        }
        // Step 7: 构造下一轮 poll 描述符集，按距离最近事件点（最多 50ms）进行有界阻塞等待。
        let mut fds = [libc::pollfd {
            fd: -1,
            events: 0,
            revents: 0,
        }; 4];
        if let Some(flows) = &flows {
            let first = flows[0].poll_fds();
            let second = flows[1].poll_fds();
            fds = [first[0], first[1], second[0], second[1]];
        }
        let next = terminate_at.or(kill_at).unwrap_or(deadline);
        let millis = next
            .saturating_duration_since(Instant::now())
            .as_millis()
            .min(50) as i32;
        // SAFETY: `fds` 是栈上长度为 4 的可写 `pollfd` 数组（未激活的槽位 `fd` 为 -1，内核会直接忽略），`millis` 为限制在 `0..=50` 范围内的超时毫秒数。
        if unsafe { libc::poll(fds.as_mut_ptr(), fds.len() as libc::nfds_t, millis) } == -1 {
            let error = io::Error::last_os_error();
            if error.kind() != io::ErrorKind::Interrupted {
                errors.push(format!("poll: {error}"));
                signal_group(child, libc::SIGKILL, errors);
                kill_at = Some(Instant::now());
            }
        }
    }
    // Step 8: 汇总最终的进程退出码、超时标志或等待错误，生成 ExitOutcome。
    let raw = status.map(ExitStatusExt::into_raw);
    if timed_out {
        return ExitOutcome::TimedOut {
            raw,
            signal: raw.and_then(|raw| ExitStatus::from_raw(raw).signal()),
        };
    }
    if let Some(error) = wait_error {
        return ExitOutcome::IoError {
            errno: error.raw_os_error(),
            message: error.to_string(),
            raw,
        };
    }
    match raw {
        Some(raw) => exit_outcome(ExitStatus::from_raw(raw)),
        None => ExitOutcome::IoError {
            errno: None,
            message: "child wait status unavailable".into(),
            raw: None,
        },
    }
}

/// 启动并监控单条命令的完整执行过程，返回包含底层退出状态、I/O 错误、耗时与协议判定结果的 `Execution`。
///
/// - 对于 `Plain` 与 `Ltp` 协议，不拦截输出正文，直接将被测进程的标准输出/错误绑定到 `outputs` 的复制描述符。
/// - 对于 `Basic`、`Pre2025Libctest` 与 `Tap` 协议，建立管道捕获与 `Relay` 辅助转发进程，一边实时解析协议标记一边异步输出到终端。
/// - 所有协议下子进程均继承当前的 `stdin`，并在 `timeout` 转换的绝对截止时间保护下运行。
pub fn run_command(
    root: Option<&Path>,
    cwd: &Path,
    spec: &CommandSpec,
    env: &BTreeMap<String, String>,
    timeout: Duration,
    protocol: &Protocol,
    outputs: [RawFd; 2],
) -> Execution {
    // Step 1: 计算命令的绝对截止时间戳，并初始化对应协议的流式观察器。
    let started = Instant::now();
    let deadline = started.checked_add(timeout);
    let mut observer = ProtocolObserver::new(protocol);
    let mut io_errors = Vec::new();
    // Step 2: 构造命令与可选的 Relay 辅助进程，派生子进程并进入 pump 循环直至结束或超时。
    let outcome = match deadline {
        None => ExitOutcome::SpawnError {
            errno: Some(libc::EINVAL),
            message: "deadline exceeds monotonic clock range".into(),
        },
        Some(deadline) => {
            let capture = matches!(
                protocol,
                Protocol::Basic { .. } | Protocol::Pre2025Libctest { .. } | Protocol::Tap
            );
            match command(root, cwd, spec, env, capture, outputs) {
                Err(error) => ExitOutcome::SpawnError {
                    errno: error.raw_os_error(),
                    message: error.to_string(),
                },
                Ok(mut command) => {
                    let relay = if capture {
                        Relay::new(outputs).map(Some)
                    } else {
                        Ok(None)
                    };
                    match relay {
                        Err(error) => ExitOutcome::IoError {
                            errno: error.raw_os_error(),
                            message: format!("create output relay: {error}"),
                            raw: None,
                        },
                        Ok(relay) => match command.spawn() {
                            Ok(mut child) => {
                                pump(&mut child, deadline, &mut observer, relay, &mut io_errors)
                            }
                            Err(error) => ExitOutcome::SpawnError {
                                errno: error.raw_os_error(),
                                message: error.to_string(),
                            },
                        },
                    }
                }
            }
        }
    };
    // Step 3: 闭合协议观察器得出最终判定（若 TAP 协议下发生管道 I/O 错误则降级为 InfrastructureError）。
    let mut result = observer.finish(&outcome);
    if matches!(protocol, Protocol::Tap) && !io_errors.is_empty() {
        result.verdict = Verdict::InfrastructureError;
    }
    Execution {
        outcome,
        io_errors,
        elapsed: started.elapsed(),
        protocol: result,
    }
}
