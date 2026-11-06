//! 内核态跨文件零拷贝/直通传输模块（`copy_file_range` 与 `splice`）。
//!
//! # 模块职责与架构定位
//! 本模块实现两个在内核态直接搬运文件数据的传输入口，避免调用方在用户态反复分配缓冲并来回拷贝：
//! - **上游调用者**：系统调用层（`sys_copy_file_range`、`sys_splice`）以及内核测试。
//! - **下游依赖**：普通文件打开实例（`InodeFile`）、匿名管道端点（`PipeEnd`）、文件回写同步与脏页节流机制。
//!
//! # 核心设计与数据流向
//! ```text
//!   copy_file_range (同一挂载文件系统内的普通文件 -> 普通文件):
//!   +---------------+   read_at (4 KiB 栈缓冲)    +---------------+
//!   | 源 InodeFile  | --------------------------> | 目标 InodeFile|
//!   +---------------+                             +---------------+
//!          |  1. 按对象地址序加锁隐式 cursor             |  2. 仅按目标实际接收字节数推进两端偏移
//!          +---------------------------------------------+  3. 释放 cursor 锁后执行 O_DSYNC/O_SYNC 与脏页节流
//!
//!   splice (至少一端为匿名管道 PipeEnd):
//!   - PipeEnd   -> PipeEnd   : 管道环形页缓冲之间直接搬运物理页槽
//!   - PipeEnd   -> InodeFile : 锁住目标文件隐式 cursor -> 管道向文件写入 -> 更新偏移并放锁 -> 同步与脏页节流
//!   - InodeFile -> PipeEnd   : 锁住源文件隐式 cursor   -> 文件读入管道页 -> 更新偏移并放锁
//! ```
//!
//! # 锁获取顺序（Lock Ordering）与并发契约
//! 1. **调用前前提**：调用方必须在进程文件描述符表自旋锁（`FileTable`）之外持有源和目标的 `FileRef`。
//! 2. **双普通文件位置锁顺序**：`copy_file_range` 若两端均使用打开文件描述（OFD）的隐式游标，必须按 `InodeFile` 实例指针地址从小到大依次获取 `cursor` 睡眠锁；若源和目标指向同一个 `InodeFile` 实例，则只加锁一次，防止自死锁。
//! 3. **文件位置锁先于管道锁与 Inode I/O 锁**：`positioned_splice` 先获取普通文件的 `cursor` 睡眠锁，再进入管道环形缓冲区的睡眠锁或底层 `Inode` 读写路径。
//! 4. **同步回写位于位置锁之外**：`O_DSYNC`/`O_SYNC` 数据落盘与脏页平衡节流必须在释放 `cursor` 睡眠锁和管道锁之后执行，避免持位置锁长时间阻塞其他并发观察者。

use crate::fs::file::{FileIoContext, FileLike, FileRef, InodeFile};
use crate::fs::{FsError, InodeKind, O_ACCMODE, O_APPEND, O_RDONLY, O_WRONLY};
use crate::wait::WaitMode;

/// `splice` 非阻塞标志（对应 `SPLICE_F_NONBLOCK`）：管道无数据或无空间时立即返回 `WouldBlock` 而非睡眠等待。
const SPLICE_NONBLOCK: u32 = 2;
/// `splice` 页面移动提示标志（对应 `SPLICE_F_MOVE`）：Linux ABI 兼容提示位，不影响数据语义。
const SPLICE_F_MOVE: u32 = 1;
/// `splice` 后续仍有更多数据提示标志（对应 `SPLICE_F_MORE`）：Linux ABI 兼容提示位。
const SPLICE_F_MORE: u32 = 4;
/// `splice` 赠予用户页提示标志（对应 `SPLICE_F_GIFT`）：Linux ABI 兼容提示位。
const SPLICE_F_GIFT: u32 = 8;
/// `splice` 允许传入的全部合法标志位掩码。
const SPLICE_FLAGS: u32 = SPLICE_F_MOVE | SPLICE_NONBLOCK | SPLICE_F_MORE | SPLICE_F_GIFT;

/// `copy_file_range` 在内核栈上单次中转读写的分块大小（字节，4 KiB）。
const TRANSFER_CHUNK: usize = 4096;

/// 校验起始偏移 `offset` 与请求长度 `length` 之和未发生算术溢出，且可表示为非负有符号 64 位偏移（`loff_t`，不超过 `i64::MAX`）。
///
/// 在普通可睡眠任务上下文中调用；若越界则返回 `FsError::InvalidOperation`（对应 `EINVAL`）。
fn check_range(offset: u64, length: usize) -> Result<(), FsError> {
    let end = offset
        .checked_add(length as u64)
        .ok_or(FsError::InvalidOperation)?;
    if (i64::MAX as u64) < end {
        return Err(FsError::InvalidOperation);
    }
    Ok(())
}

/// 处理分步传输中途遇到的错误：若此前已成功传输非零字节（Partial Progress），则优先返回已完成字节数；仅当零进度时才上报错误。
fn progress(accepted: usize, error: FsError) -> Result<usize, FsError> {
    if accepted == 0 {
        Err(error)
    } else {
        Ok(accepted)
    }
}

/// 检查当前调用者在持续传输循环间隙是否收到待响应的可中断信号。
///
/// 内核内部调用（`FileIoContext::Kernel`）不受用户态信号打断，始终返回 `false`；
/// 用户线程调用（`FileIoContext::User`）在存在待处理非屏蔽信号时返回 `true`，以便及时返回短传输计数或 `EINTR`。
pub(crate) fn interruption_pending(caller: FileIoContext<'_>) -> bool {
    match caller {
        FileIoContext::Kernel => false,
        FileIoContext::User(user) => user
            .signals()
            .is_some_and(|signals| signals.interruption_pending(WaitMode::Interruptible)),
    }
}

/// 提取 `splice` 文件端对应的普通文件实例（`InodeFile`）；若传入目录、套接字或其他非普通文件对象则返回 `FsError::InvalidOperation`。
fn regular(file: &FileRef) -> Result<&InodeFile, FsError> {
    file.inode_file().ok_or(FsError::InvalidOperation)
}

/// 在同一挂载文件系统的两个普通文件之间内核态拷贝最多 `length` 字节的数据（对应 `copy_file_range` 系统调用）。
///
/// # 调用上下文与参数
/// - 必须在普通可睡眠任务上下文中调用，且不得持有描述符表锁。
/// - `source_offset` / `target_offset` 为 `Some` 时使用并更新调用方显式给定的偏移，不触碰打开文件实例（OFD）自身的游标；为 `None` 时加锁推进对应 OFD 的共享游标。
/// - 返回实际成功写入目标文件的字节数；若两个文件位于同一 `Inode` 且读写字节区间重叠，返回 `FsError::InvalidOperation`。
pub(crate) fn copy_file_range(
    source: &FileRef,
    source_offset: Option<&mut u64>,
    target: &FileRef,
    target_offset: Option<&mut u64>,
    length: usize,
    caller: FileIoContext<'_>,
) -> Result<usize, FsError> {
    let input = source.inode_file();
    let output = target.inode_file();
    // Step 1: 校验源与目标的对象类型、打开访问模式、追加标志以及是否属于同一个底层文件系统实例。
    // 目录检查必须在常规能力提取之前完成，保证对目录描述符准确返回 EISDIR 而非 EINVAL。
    if (input.is_none() && source.stat()?.kind == InodeKind::Directory)
        || (output.is_none() && target.stat()?.kind == InodeKind::Directory)
    {
        return Err(FsError::IsDirectory);
    }
    let input = input.ok_or(FsError::InvalidOperation)?;
    let output = output.ok_or(FsError::InvalidOperation)?;
    if source.flags() & O_ACCMODE == O_WRONLY
        || target.flags() & O_ACCMODE == O_RDONLY
        || target.flags() & O_APPEND != 0
    {
        return Err(FsError::BadFd);
    }
    if !input
        .path()
        .unwrap()
        .filesystem()
        .ptr_eq(output.path().unwrap().filesystem())
    {
        return Err(FsError::CrossDevice);
    }
    output.transfer_check_write()?;

    let flags = output.flags();
    let shared = core::ptr::eq(input, output);
    let lock_input = source_offset.is_none();
    let lock_output = target_offset.is_none() && !(shared && lock_input);
    // Step 2: 按 InodeFile 内存地址升序获取隐式位置睡眠锁；若源和目标是同一个 OFD 实例，仅获取一次锁以避免自死锁。
    let (mut input_cursor, mut output_cursor) =
        if (input as *const InodeFile) < (output as *const InodeFile) {
            let input_cursor = lock_input.then(|| input.transfer_cursor());
            let output_cursor = lock_output.then(|| output.transfer_cursor());
            (input_cursor, output_cursor)
        } else {
            let output_cursor = lock_output.then(|| output.transfer_cursor());
            let input_cursor = lock_input.then(|| input.transfer_cursor());
            (input_cursor, output_cursor)
        };
    let start_in = source_offset
        .as_deref()
        .copied()
        .unwrap_or_else(|| **input_cursor.as_ref().unwrap());
    let start_out = target_offset.as_deref().copied().unwrap_or_else(|| {
        if let Some(cursor) = &output_cursor {
            **cursor
        } else {
            **input_cursor.as_ref().unwrap()
        }
    });
    // Step 3: 校验两端偏移与请求长度不超出 loff_t 上限，按源文件当前大小裁剪可拷贝长度，并拒绝同文件重叠区间拷贝。
    check_range(start_in, length)?;
    check_range(start_out, length)?;
    let size = input.mapping_inode().unwrap().metadata_snapshot()?.size;
    let length = length.min(size.saturating_sub(start_in) as usize);
    if input
        .mapping_inode()
        .unwrap()
        .ptr_eq(output.mapping_inode().unwrap())
        && start_in < start_out + length as u64
        && start_out < start_in + length as u64
    {
        return Err(FsError::InvalidOperation);
    }

    // Step 4: 执行分块读写传输，仅按目标文件实际接收的字节数更新显式偏移或隐式游标，随后立即释放位置锁。
    let result = copy_bytes(input, start_in, output, start_out, length, caller);
    if let Ok(accepted) = result {
        if let Some(offset) = source_offset {
            *offset = start_in + accepted as u64;
        }
        if let Some(offset) = target_offset {
            *offset = start_out + accepted as u64;
        }
        if let Some(cursor) = &mut input_cursor {
            **cursor = start_in + accepted as u64;
        }
        if let Some(cursor) = &mut output_cursor {
            **cursor = start_out + accepted as u64;
        }
    }
    drop(output_cursor);
    drop(input_cursor);

    // Step 5: 在位置锁释放后履行目标文件的 O_DSYNC/O_SYNC 落盘与脏页回写节流。
    finish_write(output, flags, start_out, result)
}

/// 使用 4 KiB 栈上临时缓冲区循环从 `input` 指定偏移读取并写入 `output` 指定偏移。
///
/// 每次循环均检查用户线程是否有待处理信号；若目标发生短写或中途出错，仅累计目标已实际写入的字节数，不会多推进源偏移。
fn copy_bytes(
    input: &InodeFile,
    start_in: u64,
    output: &InodeFile,
    start_out: u64,
    length: usize,
    caller: FileIoContext<'_>,
) -> Result<usize, FsError> {
    let mut scratch = [0u8; TRANSFER_CHUNK];
    let mut copied = 0;
    while copied < length {
        let request = (length - copied).min(scratch.len());
        let read = match input.read_at(start_in + copied as u64, &mut scratch[..request]) {
            Ok(read) => read,
            Err(error) => return progress(copied, error),
        };
        if read == 0 {
            break;
        }
        if interruption_pending(caller) {
            return progress(copied, FsError::Interrupted);
        }

        let accepted = match output.transfer_write_at(
            start_out + copied as u64,
            &scratch[..read],
            FsError::BadFd,
        ) {
            Ok(accepted) => accepted,
            Err(error) => return progress(copied, error),
        };
        copied += accepted;
        if accepted < read || read < request {
            break;
        }
    }
    Ok(copied)
}

/// 在位置锁与管道锁释放后，对本次实际写入的非零字节范围执行同步落盘（若设置了 `O_DSYNC`/`O_SYNC`）与脏页节流。
///
/// 若数据已写入页缓存但后续同步刷盘失败，直接向上层返回该同步错误（已更新的文件内容和偏移不会回滚）。
fn finish_write(
    file: &InodeFile,
    flags: u32,
    start: u64,
    result: Result<usize, FsError>,
) -> Result<usize, FsError> {
    let accepted = result?;
    if 0 < accepted {
        file.transfer_finish(flags, start, accepted)?;
    }
    Ok(accepted)
}

/// 为涉及普通文件一端的 `splice` 操作统一管理文件偏移计算、位置锁串行化与写后同步。
///
/// 当 `offset` 为 `None` 时在整个 `operation` 期间持有文件的 `cursor` 睡眠锁，按返回的实际传输字节数更新偏移后先释放锁，
/// 若为写入方向（`writing == true`）再在锁外触发 `finish_write` 执行同步与脏页节流。
fn positioned_splice(
    file: &InodeFile,
    offset: Option<&mut u64>,
    length: usize,
    writing: bool,
    operation: impl FnOnce(u64) -> Result<usize, FsError>,
) -> Result<usize, FsError> {
    let flags = file.flags();
    let mut cursor = offset.is_none().then(|| file.transfer_cursor());
    let start = offset
        .as_deref()
        .copied()
        .unwrap_or_else(|| **cursor.as_ref().unwrap());
    check_range(start, length)?;

    let result = operation(start);
    if let Ok(accepted) = result {
        if let Some(offset) = offset {
            *offset = start + accepted as u64;
        }
        if let Some(cursor) = &mut cursor {
            **cursor = start + accepted as u64;
        }
    }
    drop(cursor);

    if writing {
        finish_write(file, flags, start, result)
    } else {
        result
    }
}

/// 在匿名管道与文件描述符之间（或两个管道之间）传输最多 `length` 字节数据（对应 `splice` 系统调用）。
///
/// # 调用上下文与参数
/// - 在普通可睡眠任务上下文中调用，要求 `source` 与 `target` 至少有一端是匿名管道（`PipeEnd`），另一端可以是管道或普通文件（`InodeFile`）。
/// - 管道端不允许指定显式偏移（否则返回 `FsError::IllegalSeek` / `ESPIPE`）；作为写入目标的普通文件不允许带有 `O_APPEND` 状态。
/// - `SPLICE_F_MOVE`、`SPLICE_F_MORE`、`SPLICE_F_GIFT` 仅作为 ABI 兼容提示位接受，底层统一通过管道环形缓冲与页缓存安全传输。
pub(crate) fn splice(
    source: &FileRef,
    source_offset: Option<&mut u64>,
    target: &FileRef,
    target_offset: Option<&mut u64>,
    length: usize,
    flags: u32,
    caller: FileIoContext<'_>,
) -> Result<usize, FsError> {
    // Step 1: 校验标志位合法性、管道端偏移约束（管道不可携带显式 offset）以及源可读/目标可写访问模式。
    if flags & !SPLICE_FLAGS != 0 {
        return Err(FsError::InvalidOperation);
    }
    let input_pipe = source.pipe_end();
    let output_pipe = target.pipe_end();
    if (input_pipe.is_some() && source_offset.is_some())
        || (output_pipe.is_some() && target_offset.is_some())
    {
        return Err(FsError::IllegalSeek);
    }
    if source.flags() & O_ACCMODE == O_WRONLY || target.flags() & O_ACCMODE == O_RDONLY {
        return Err(FsError::BadFd);
    }

    // Step 2: 按两端具体文件类型分派至“管道->管道”、“管道->普通文件”或“普通文件->管道”传输路径。
    let nonblocking = flags & SPLICE_NONBLOCK != 0;
    match (input_pipe, output_pipe) {
        (Some(input), Some(output)) => input.splice_to_pipe(output, length, nonblocking, caller),
        (Some(input), None) => {
            if target.flags() & O_APPEND != 0 {
                return Err(FsError::InvalidOperation);
            }
            let output = regular(target)?;
            output.transfer_check_write()?;
            positioned_splice(output, target_offset, length, true, |offset| {
                input.splice_to_file(output, offset, length, nonblocking, caller)
            })
        }
        (None, Some(output)) => {
            let input = regular(source)?;
            positioned_splice(input, source_offset, length, false, |offset| {
                output.splice_from_file(input, offset, length, nonblocking, caller)
            })
        }
        (None, None) => Err(FsError::InvalidOperation),
    }
}
