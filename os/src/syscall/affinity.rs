//! 用户 affinity 与内核调度共用一个许可集合；号码只在开始时解析一次。

use alloc::sync::Arc;

use crate::cpu::{self, CpuSet};
use crate::sched;
use crate::syscall::error::{copyin, copyout, Errno, SysResult};
use crate::task::{Task, UserThread};

/// 在原身份图取得稳定执行者，之后睡眠不再按可复用整数查找目标。
fn target(user: &UserThread, pid: usize, changing: bool) -> Result<Arc<Task>, Errno> {
    let pid = pid as i32;
    if pid < 0 {
        return Err(Errno::InvalidArgument);
    }

    let target = if pid == 0 {
        user.executor().ok_or(Errno::NoProcess)?
    } else {
        let identity = user.identity().ok_or(Errno::NoProcess)?;
        let thread = identity
            .graph()
            .lookup_thread(pid as usize)
            .ok_or(Errno::NoProcess)?;
        let task = thread.executor().ok_or(Errno::NoProcess)?;
        if changing {
            let caller = user.runtime().ok_or(Errno::NoProcess)?.credentials.uid;
            let owner = thread.runtime().ok_or(Errno::NoProcess)?.credentials.uid;
            if caller != 0 && caller != owner {
                return Err(Errno::NotPermitted);
            }
        }
        task
    };
    Ok(target)
}

/// 仅复制一次内部 mask 前缀，短输入补零，长输入的额外地址不访问。
pub(super) fn set(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let mut bytes = [0u8; CpuSet::byte_len()];
    let length = raw[1].min(bytes.len());
    let mm = user.mm().ok_or(Errno::NoProcess)?;
    copyin(&mm, &mut bytes[..length], raw[2])?;
    let allowed = CpuSet::from_bytes(&bytes).intersection(cpu::actual_set());
    if allowed.is_empty() {
        return Err(Errno::InvalidArgument);
    }

    let task = target(user, raw[0], true)?;
    sched::set_affinity(&task, allowed);
    Ok(0)
}

/// 按 unsigned-long 对齐交付内部前缀，并保留用户更长缓冲的尾部。
pub(super) fn get(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let length = raw[1];
    if length % core::mem::size_of::<usize>() != 0
        || length < cpu::count().div_ceil(u8::BITS as usize)
    {
        return Err(Errno::InvalidArgument);
    }

    let task = target(user, raw[0], false)?;
    let allowed = sched::affinity(&task).intersection(cpu::actual_set());
    let length = length.min(CpuSet::byte_len());
    let mut bytes = [0u8; CpuSet::byte_len()];
    allowed.write_bytes(&mut bytes[..length]);

    let mm = user.mm().ok_or(Errno::NoProcess)?;
    copyout(&mm, raw[2], &bytes[..length])?;
    Ok(length as isize)
}

/// 同一快照输出逻辑 CPU 与 node0；两个 u32 输出均可空，cache 参数忽略。
pub(super) fn getcpu(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let id = cpu::current().id as u32;
    let mm = user.mm().ok_or(Errno::NoProcess)?;
    if raw[0] != 0 {
        copyout(&mm, raw[0], &id.to_le_bytes())?;
    }
    if raw[1] != 0 {
        copyout(&mm, raw[1], &0u32.to_le_bytes())?;
    }
    Ok(0)
}
