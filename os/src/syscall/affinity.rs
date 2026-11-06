//! CPU 亲和性（Affinity）与当前 CPU 查询系统调用模块。
//!
//! ## 模块职责与架构定位
//! 本模块位于系统调用分发层（`crate::syscall`）与调度器/CPU 拓扑层（`crate::sched`、`crate::cpu`）之间，
//! 负责处理与处理器绑定及拓扑查询相关的三个系统调用：
//! - `sched_setaffinity`（122）：设置指定线程允许运行的 CPU 掩码；
//! - `sched_getaffinity`（123）：查询指定线程当前允许运行的 CPU 掩码；
//! - `getcpu`（168）：查询调用线程当前所在的逻辑 CPU 编号与 NUMA 节点编号。
//!
//! ## 核心设计与掩码内存布局
//! 用户态传入的 `cpu_set_t` 掩码缓冲区按字节数组拷贝到内核的 `CpuSet` 位图中，
//! 并与系统实际在线的 CPU 集合（`cpu::actual_set()`）求交集：
//!
//! ```text
//! 用户态 cpu_set_t 缓冲区 (长度为 raw[1] 字节)
//! +-----------------------+-----------------------+
//! | Byte 0 (CPU 0..7)     | Byte 1 (CPU 8..15) ...|
//! +-----------------------+-----------------------+
//!            | copyin / copyout (截断至 CpuSet::byte_len())
//!            v
//! 内核 CpuSet 位图  &  cpu::actual_set() (在线物理核集合)
//!            |
//!            v
//! 调度器任务亲和性掩码 (Task affinity)
//! ```
//!
//! 在系统调用入口处，内核通过 `TaskGraph` 将用户传入的线程 ID（`pid`）一次性解析为稳定的 `Arc<Task>` 引用，
//! 后续在调度器中修改或读取亲和性掩码时直接操作该任务对象，避免因等待或并发重用导致 PID 指向变化。
//!
//! ## 锁获取顺序与并发契约
//! - 解析目标线程时短暂获取 `TaskGraph` 内部锁；在 `target()` 返回 `Arc<Task>` 后即释放。
//! - 用户态内存拷贝（`copyin` / `copyout`）均在未持有调度器锁和 `TaskGraph` 锁的状态下执行，防止缺页或锁顺序反转。

use alloc::sync::Arc;

use crate::cpu::{self, CpuSet};
use crate::sched;
use crate::syscall::error::{copyin, copyout, Errno, SysResult};
use crate::task::{Task, UserThread};

/// 根据系统调用传入的 `pid` 解析目标内核任务引用，并在需要修改属性时校验调用方权限。
///
/// - **调用上下文**：普通系统调用上下文，未持有自旋锁。
/// - **参数**：
///   - `user`：发起系统调用的当前用户线程。
///   - `pid`：目标线程 ID（`0` 表示当前调用线程自身，负数非法）。
///   - `changing`：是否为修改操作（`true` 时需检查调用者与目标线程的 UID 是否匹配或具备 root 权限）。
/// - **返回值**：成功返回目标线程绑定的 `Arc<Task>`，失败返回对应的 `Errno`（如 `EINVAL`、`ESRCH`、`EPERM`）。
fn target(user: &UserThread, pid: usize, changing: bool) -> Result<Arc<Task>, Errno> {
    let pid = pid as i32;
    if pid < 0 {
        return Err(Errno::InvalidArgument);
    }

    // Step 1: 解析目标线程。pid == 0 直接取当前线程的执行体；否则在 TaskGraph 中按 TID 查找。
    let target = if pid == 0 {
        user.executor().ok_or(Errno::NoProcess)?
    } else {
        let identity = user.identity().ok_or(Errno::NoProcess)?;
        let thread = identity
            .graph()
            .lookup_thread(pid as usize)
            .ok_or(Errno::NoProcess)?;
        let task = thread.executor().ok_or(Errno::NoProcess)?;
        // Step 2: 若为修改亲和性的操作，非 root 用户（uid != 0）只能修改同一 UID 下的线程。
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

/// 处理 `sched_setaffinity` 系统调用：设置目标线程的 CPU 亲和性掩码。
///
/// - **调用上下文**：普通系统调用上下文。
/// - **参数**：
///   - `user`：当前用户线程。
///   - `raw`：系统调用原始参数数组，`raw[0]` 为 `pid`，`raw[1]` 为 `cpusetsize`（字节数），`raw[2]` 为用户态掩码指针 `mask`。
/// - **返回值**：成功返回 `Ok(0)`；若掩码与在线 CPU 集合交集为空或地址/权限非法则返回对应 `Errno`。
pub(super) fn set(user: &UserThread, raw: [usize; 6]) -> SysResult {
    // Step 1: 从用户空间拷入掩码前缀（超出内核 CpuSet 容量的尾部字节忽略，不足部分默认补零）。
    let mut bytes = [0u8; CpuSet::byte_len()];
    let length = raw[1].min(bytes.len());
    let mm = user.mm().ok_or(Errno::NoProcess)?;
    copyin(&mm, &mut bytes[..length], raw[2])?;
    // Step 2: 与系统实际在线的 CPU 集合求交集，若交集为空则拒绝设置。
    let allowed = CpuSet::from_bytes(&bytes).intersection(cpu::actual_set());
    if allowed.is_empty() {
        return Err(Errno::InvalidArgument);
    }

    // Step 3: 解析目标任务并做权限检查，随后通知调度器更新亲和性并在必要时触发迁移。
    let task = target(user, raw[0], true)?;
    sched::set_affinity(&task, allowed);
    Ok(0)
}

/// 处理 `sched_getaffinity` 系统调用：读取目标线程的 CPU 亲和性掩码并写回用户缓冲区。
///
/// - **调用上下文**：普通系统调用上下文。
/// - **参数**：
///   - `user`：当前用户线程。
///   - `raw`：系统调用原始参数数组，`raw[0]` 为 `pid`，`raw[1]` 为用户缓冲区字节长度 `cpusetsize`，`raw[2]` 为用户态缓冲区地址 `mask`。
/// - **返回值**：成功返回实际写回用户缓冲区的字节数；若缓冲区长度未按机器字长对齐或不足以容纳在线 CPU 位图则返回 `EINVAL`。
pub(super) fn get(user: &UserThread, raw: [usize; 6]) -> SysResult {
    // Step 1: 校验用户缓冲区长度必须为 usize 字节数的整数倍，且至少能容纳当前系统的全部 CPU 位。
    let length = raw[1];
    if length % core::mem::size_of::<usize>() != 0
        || length < cpu::count().div_ceil(u8::BITS as usize)
    {
        return Err(Errno::InvalidArgument);
    }

    // Step 2: 获取目标任务的亲和性掩码，并编码为字节序列写回用户空间。
    let task = target(user, raw[0], false)?;
    let allowed = sched::affinity(&task).intersection(cpu::actual_set());
    let length = length.min(CpuSet::byte_len());
    let mut bytes = [0u8; CpuSet::byte_len()];
    allowed.write_bytes(&mut bytes[..length]);

    let mm = user.mm().ok_or(Errno::NoProcess)?;
    copyout(&mm, raw[2], &bytes[..length])?;
    Ok(length as isize)
}

/// 处理 `getcpu` 系统调用：查询当前线程正在运行的逻辑 CPU 编号与 NUMA 节点编号。
///
/// - **调用上下文**：普通系统调用上下文。
/// - **参数**：
///   - `user`：当前用户线程。
///   - `raw`：系统调用原始参数数组，`raw[0]` 为输出 `cpu` 编号的 `u32` 用户指针（可为 NULL），
///     `raw[1]` 为输出 `node` 编号的 `u32` 用户指针（可为 NULL），`raw[2]` 为已废弃的 `tcache` 参数（忽略）。
/// - **返回值**：成功返回 `Ok(0)`，用户指针非法时返回 `EFAULT`。
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
