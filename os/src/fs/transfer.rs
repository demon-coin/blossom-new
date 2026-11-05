//! 文件间传输只提交目标实际接受量，位置锁先于原管道页环，同步在两者之外完成。

use crate::fs::file::{FileIoContext, FileLike, FileRef, InodeFile};
use crate::fs::{FsError, InodeKind, O_ACCMODE, O_APPEND, O_RDONLY, O_WRONLY};
use crate::wait::WaitMode;

const SPLICE_NONBLOCK: u32 = 2;
const SPLICE_F_MOVE: u32 = 1;
const SPLICE_F_MORE: u32 = 4;
const SPLICE_F_GIFT: u32 = 8;
const SPLICE_FLAGS: u32 = SPLICE_F_MOVE | SPLICE_NONBLOCK | SPLICE_F_MORE | SPLICE_F_GIFT;

const TRANSFER_CHUNK: usize = 4096;

/// 核明确位置和完整请求可表示为非负 loff_t，不用截断掩盖溢出。
fn check_range(offset: u64, length: usize) -> Result<(), FsError> {
    let end = offset
        .checked_add(length as u64)
        .ok_or(FsError::InvalidOperation)?;
    if (i64::MAX as u64) < end {
        return Err(FsError::InvalidOperation);
    }
    Ok(())
}

/// 后续错误保留已经真实接受的正前缀，零进度才返回原错误。
fn progress(accepted: usize, error: FsError) -> Result<usize, FsError> {
    if accepted == 0 {
        Err(error)
    } else {
        Ok(accepted)
    }
}

/// 持续有数据时仍观察本用户线程的真实可处理中断，不为内核调用创建信号等待。
pub(crate) fn interruption_pending(caller: FileIoContext<'_>) -> bool {
    match caller {
        FileIoContext::Kernel => false,
        FileIoContext::User(user) => user
            .signals()
            .is_some_and(|signals| signals.interruption_pending(WaitMode::Interruptible)),
    }
}

/// splice 的文件端必须提供实际普通文件能力，目录或其它流按 EINVAL 拒绝。
fn regular(file: &FileRef) -> Result<&InodeFile, FsError> {
    file.inode_file().ok_or(FsError::InvalidOperation)
}

/// copy_file_range 只接受实际普通文件、合法访问模式及同一个真实文件系统后端。
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
    // 两端目录检查先于其它对象和访问模式，实际普通对象不再采样无关属性。
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
    // 地址只用作 OFD 锁序；同一实例即使两端隐式位置也只取一个 guard。
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

    finish_write(output, flags, start_out, result)
}

/// 一片栈缓冲只按位置观察源；目标拒绝的后缀从未推进源位置。
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

/// 数据和位置已经提交；同步失败仍返回原错误，不回滚已接受前缀。
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

/// 一端普通文件的位置跨原页环操作串行化，真实进度提交后放锁，再同步目标。
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

/// 在原 pipe 与普通 inode 打开实例之间传输；提示位不改变数据或原页拥有结果。
pub(crate) fn splice(
    source: &FileRef,
    source_offset: Option<&mut u64>,
    target: &FileRef,
    target_offset: Option<&mut u64>,
    length: usize,
    flags: u32,
    caller: FileIoContext<'_>,
) -> Result<usize, FsError> {
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
