//! 旧版进程管理、系统信息查询与资源限额兼容系统调用模块。
//!
//! ## 模块职责与架构定位
//! 本文件保留了早期内核框架中与系统及进程管理相关的系统调用入口（新架构下的主流分发路径已迁移至 `syscall::system` 与 `syscall::task`）：
//! - 系统控制与内核日志：`sys_shutdown`、`sys_syslog`；
//! - 系统标识与运行状态：`sys_uname`（`UTSName`）、`sys_sysinfo`（`Sysinfo`）；
//! - 用户与组凭据查询：`sys_getuid`、`sys_geteuid`、`sys_getgid`、`sys_getegid`；
//! - 进程资源限额与健壮互斥锁链表：`sys_prlimit`（`RLimit` / `Resource`）、`sys_set_robust_list`、`sys_get_robust_list`；
//! - 多核屏障与内存访问建议兼容桩：`sys_memorybarrier`、`sys_madvise`。
//!
//! ## 核心数据结构内存布局（C ABI）
//! `UTSName` 按 Linux `struct utsname` 固定布局排列，共 6 个定长 65 字节的以 `\0` 结尾的字符串字段（总计 390 字节）：
//!
//! ```text
//! Offset   Size   Field
//! +------+------+------------------------------------+
//! |   0  |  65  | sysname    ("NPUcore\0")           |
//! |  65  |  65  | nodename   ("blossom\0")           |
//! | 130  |  65  | release    ("5.10.0-1-rv64/la64\0")|
//! | 195  |  65  | version    ("#1 SMP ...\0")        |
//! | 260  |  65  | machine    ("rv64\0" / "la64\0")   |
//! | 325  |  65  | domainname ("\0")                  |
//! +------+------+------------------------------------+
//! ```
//!
//! ## 锁获取顺序与并发契约
//! - 在 `sys_prlimit` 中，先短暂获取任务内部锁（`task.acquire_inner_lock()`）读取用户页表 Token，立即释放后再访问文件描述符表锁（`task.files.lock()`）或执行用户态内存拷贝，避免持任务内部锁跨越页表拷贝及嵌套获取 `files` 锁。
//! - 在 `sys_get_robust_list` / `sys_set_robust_list` 中，通过 `task.acquire_inner_lock()` 保护 `robust_list` 元数据的读写。

use crate::config::{PAGE_SIZE, SYSTEM_TASK_LIMIT, USER_STACK_SIZE};
use crate::hal::shutdown;
use crate::mm::{
    copy_from_user, copy_to_user, copy_to_user_string, translated_byte_buffer, UserBuffer,
};
use crate::syscall::errno::*;
use crate::task::{
    current_task, current_user_token, find_task_by_pid, procs_count,
};
use crate::timer::get_time_sec;
use core::mem::size_of;
use log::{error, info, warn};
use num_enum::FromPrimitive;

/// 关闭目标机器或通知 QEMU 模拟器停机退出。
///
/// - **调用上下文**：普通系统调用上下文；底层调用平台 `shutdown()`，正常情况下不会返回。
pub fn sys_shutdown() -> isize {
    shutdown()
}

/// Linux `syslog`（`klogctl`）系统调用的操作命令枚举。
#[allow(non_camel_case_types)]
#[derive(Debug, Eq, PartialEq, FromPrimitive)]
#[repr(u32)]
pub enum SyslogAction {
    /// 关闭内核日志句柄（空操作，直接返回成功）。
    CLOSE = 0,
    /// 打开内核日志句柄（空操作，直接返回成功）。
    OPEN = 1,
    /// 从内核日志缓冲区读取最多 `len` 字节的数据。
    READ = 2,
    /// 读取内核环形日志缓冲区中保留的最近日志内容。
    READ_ALL = 3,
    /// 读取全部日志并清空环形缓冲区。
    READ_CLEAR = 4,
    /// 仅清空环形日志缓冲区。
    CLEAR = 5,
    /// 禁止将日志输出到控制台（Console）。
    CONSOLE_OFF = 6,
    /// 恢复将日志输出到控制台（Console）。
    CONSOLE_ON = 7,
    /// 设置输出到控制台的最低日志级别。
    CONSOLE_LEVEL = 8,
    /// 返回内核日志缓冲区中尚未被读取的字节数。
    SIZE_UNREAD = 9,
    /// 返回内核日志缓冲区的总字节容量。
    SIZE_BUFFER = 10,
    /// 未定义或超出范围的非法命令码。
    #[default]
    ILLEAGAL,
}

/// 处理 `syslog` 系统调用：读取静态引导日志字符串或查询日志缓冲区容量。
///
/// - **调用上下文**：普通系统调用上下文。
/// - **参数**：
///   - `type_`：操作码，对应 [`SyslogAction`]。
///   - `buf`：接收日志文本的用户态缓冲区指针。
///   - `len`：用户态缓冲区的字节长度上限。
/// - **返回值**：成功返回读取的字节数、缓冲区总大小或 `SUCCESS`（0）；非法操作码返回 `EINVAL`。
pub fn sys_syslog(type_: u32, buf: *mut u8, len: u32) -> isize {
    const LOG_BUF_LEN: usize = 4096;
    const LOG: &str = "<5>[    0.000000] Linux version 5.10.102.1-microsoft-standard-WSL2 (rtrt@TEAM-NPUCORE) (gcc (Ubuntu 9.4.0-1ubuntu1~20.04) 9.4.0, GNU ld (GNU Binutils for Ubuntu) 2.34) #1 SMP Thu Mar 10 13:31:47 CST 2022";

    let token = current_user_token();
    let type_ = SyslogAction::from(type_);
    let len = LOG.len().min(len as usize);
    match type_ {
        SyslogAction::CLOSE | SyslogAction::OPEN => SUCCESS,
        SyslogAction::READ => {
            copy_to_user_string(token, &LOG[..len], buf).unwrap();
            len as isize
        }
        SyslogAction::READ_ALL => {
            copy_to_user_string(token, &LOG[LOG.len() - len..], buf).unwrap();
            len as isize
        }
        SyslogAction::READ_CLEAR => todo!(),
        SyslogAction::CLEAR => todo!(),
        SyslogAction::CONSOLE_OFF => todo!(),
        SyslogAction::CONSOLE_ON => todo!(),
        SyslogAction::CONSOLE_LEVEL => todo!(),
        SyslogAction::SIZE_UNREAD => todo!(),
        SyslogAction::SIZE_BUFFER => LOG_BUF_LEN as isize,
        SyslogAction::ILLEAGAL => EINVAL,
    }
}

/// Linux `uname` 系统调用返回的系统与主机标识结构体（对应 C ABI `struct utsname`）。
#[allow(unused)]
#[repr(C)]
pub struct UTSName {
    /// 操作系统名称（以 `\0` 结尾的 ASCII 字节数组，长度 65 字节）。
    sysname: [u8; 65],
    /// 网络节点主机名（以 `\0` 结尾，长度 65 字节）。
    nodename: [u8; 65],
    /// 内核发行版本号（以 `\0` 结尾，长度 65 字节）。
    release: [u8; 65],
    /// 内核构建版本与编译时间描述（以 `\0` 结尾，长度 65 字节）。
    version: [u8; 65],
    /// 硬件架构标识，如 `"rv64"` 或 `"la64"`（以 `\0` 结尾，长度 65 字节）。
    machine: [u8; 65],
    /// NIS / YP 域名（以 `\0` 结尾，长度 65 字节）。
    domainname: [u8; 65],
}

/// 处理 `uname` 系统调用：向用户态缓冲区写入内核名称、版本及架构标识信息。
///
/// - **调用上下文**：普通系统调用上下文。
/// - **参数**：
///   - `buf`：指向用户态 `UTSName` 结构体的可变字节指针。
/// - **返回值**：成功返回 `SUCCESS`（0），用户地址翻译失败返回对应负错误码。
pub fn sys_uname(buf: *mut u8) -> isize {
    let token = current_user_token();
    let mut buffer = UserBuffer::new(
        match translated_byte_buffer(token, buf, size_of::<UTSName>()) {
            Ok(buffer) => buffer,
            Err(errno) => return errno,
        },
    );

    // 按固定 65 字节字段偏移依次写入 6 个以 '\0' 结尾的字符串字段。
    const FIELD_OFFSET: usize = 65;
    buffer.write_at(FIELD_OFFSET * 0, b"NPUcore\0");
    buffer.write_at(FIELD_OFFSET * 1, b"blossom\0");
    #[cfg(feature = "riscv")]
    buffer.write_at(FIELD_OFFSET * 2, b"5.10.0-1-rv64\0");
    #[cfg(feature = "loongarch64")]
    buffer.write_at(FIELD_OFFSET * 2, b"5.10.0-1-la64\0");
    buffer.write_at(
        FIELD_OFFSET * 3,
        b"#1 SMP blossom 5.10.0-1 (2025-01-10)\0",
    );
    #[cfg(feature = "riscv")]
    buffer.write_at(FIELD_OFFSET * 4, b"rv64\0");
    #[cfg(feature = "loongarch64")]
    buffer.write_at(FIELD_OFFSET * 4, b"la64\0");
    buffer.write_at(FIELD_OFFSET * 5, b"\0");
    SUCCESS
}

/// 处理 `getuid` 系统调用：返回当前进程的真实用户 ID（固定为 root 用户 `0`）。
pub fn sys_getuid() -> isize {
    0 // root 用户 UID
}

/// 处理 `geteuid` 系统调用：返回当前进程的有效用户 ID（固定为 root 用户 `0`）。
pub fn sys_geteuid() -> isize {
    0 // root 用户 EUID
}

/// 处理 `getgid` 系统调用：返回当前进程的真实组 ID（固定为 root 组 `0`）。
pub fn sys_getgid() -> isize {
    0 // root 组 GID
}

/// 处理 `getegid` 系统调用：返回当前进程的有效组 ID（固定为 root 组 `0`）。
pub fn sys_getegid() -> isize {
    0 // root 组 EGID
}

/// Linux `sysinfo` 系统调用返回的系统全局内存与负载统计结构体（对应 C ABI `struct sysinfo`）。
///
/// 所有内存与交换分区大小字段的实际字节数均为“字段值 × `mem_unit`”。
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct Sysinfo {
    /// 系统自启动以来经过的秒数。
    uptime: usize,
    /// 1 分钟、5 分钟和 15 分钟的系统平均负载（定点数，缩放因子为 65536）。
    loads: [usize; 3],
    /// 可用物理主存总大小（单位：`mem_unit` 字节）。
    totalram: usize,
    /// 当前尚未分配的空闲物理内存大小（单位：`mem_unit` 字节）。
    freeram: usize,
    /// 共享内存总大小（单位：`mem_unit` 字节）。
    sharedram: usize,
    /// 块缓冲区占用的内存大小（单位：`mem_unit` 字节）。
    bufferram: usize,
    /// 交换空间（Swap）总大小（单位：`mem_unit` 字节）。
    totalswap: usize,
    /// 当前剩余可用的交换空间大小（单位：`mem_unit` 字节）。
    freeswap: usize,
    /// 当前系统中的活跃进程总数。
    procs: u16,
    /// 高端内存（High Memory）总大小（64 位架构下恒为 0）。
    totalhigh: usize,
    /// 可用高端内存大小（64 位架构下恒为 0）。
    freehigh: usize,
    /// 上述各内存与交换空间字段所采用的字节单位大小（本实现固定设为 1 字节）。
    mem_unit: u32,
}

/// 处理 `sysinfo` 系统调用：收集系统运行时间、进程数量及物理内存余量并写回用户空间。
///
/// - **调用上下文**：普通系统调用上下文。
/// - **参数**：
///   - `info`：指向用户态 [`Sysinfo`] 结构体的输出指针。
/// - **返回值**：成功返回 `SUCCESS`（0），拷贝至用户空间失败返回 `EFAULT`。
pub fn sys_sysinfo(info: *mut Sysinfo) -> isize {
    const LINUX_SYSINFO_LOADS_SCALE: usize = 65536;
    const SEC_1_MIN: usize = 60;
    const SEC_5_MIN: usize = SEC_1_MIN * 5;
    const SEC_15_MIN: usize = SEC_1_MIN * 15;
    const UNIMPLEMENT: usize = 0;

    let token = current_user_token();
    let procs = procs_count();
    if copy_to_user(
        token,
        &Sysinfo {
            uptime: get_time_sec(),
            // 使用当前瞬时进程数近似折算 1 / 5 / 15 分钟定点数负载。
            loads: [
                procs as usize * LINUX_SYSINFO_LOADS_SCALE / SEC_1_MIN,
                procs as usize * LINUX_SYSINFO_LOADS_SCALE / SEC_5_MIN,
                procs as usize * LINUX_SYSINFO_LOADS_SCALE / SEC_15_MIN,
            ],
            totalram: crate::config::MEMORY_END - crate::config::MEMORY_START,
            freeram: crate::mm::unallocated_frames() * PAGE_SIZE,
            sharedram: UNIMPLEMENT,
            bufferram: UNIMPLEMENT,
            totalswap: 0,
            freeswap: 0,
            procs,
            totalhigh: 0,
            freehigh: 0,
            mem_unit: 1,
        },
        info,
    )
    .is_err()
    {
        log::error!("[sys_sysinfo] Failed to copy to {:?}", info);
        EFAULT
    } else {
        SUCCESS
    }
}

/// 进程资源软硬上限结构体（对应 C ABI `struct rlimit` / `struct rlimit64`）。
#[allow(unused)]
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct RLimit {
    /// 软限制（Soft limit）：内核实际对进程强制执行的当前资源上限。
    rlim_cur: usize,
    /// 硬限制（Hard limit）：软限制允许上调到的最高天花板。
    rlim_max: usize,
}

/// POSIX / Linux 进程资源限制类型编号枚举（对应 `RLIMIT_*` 常量）。
#[derive(Debug, Eq, PartialEq, FromPrimitive)]
#[repr(u32)]
pub enum Resource {
    /// `RLIMIT_CPU`：进程可消耗的最大 CPU 时间（秒）。
    CPU = 0,
    /// `RLIMIT_FSIZE`：进程允许创建的最大文件字节数。
    FSIZE = 1,
    /// `RLIMIT_DATA`：进程数据段（初始化数据、未初始化数据与堆）的最大字节数。
    DATA = 2,
    /// `RLIMIT_STACK`：进程主线程用户栈的最大字节数。
    STACK = 3,
    /// `RLIMIT_CORE`：核心转储（Core dump）文件的最大字节数。
    CORE = 4,
    /// `RLIMIT_RSS`：进程驻留集（Resident Set Size）的最大页/字节上限。
    RSS = 5,
    /// `RLIMIT_NPROC`：当前真实用户 ID 可创建的最大进程/线程数。
    NPROC = 6,
    /// `RLIMIT_NOFILE`：进程可打开的最大文件描述符数值加 1。
    NOFILE = 7,
    /// `RLIMIT_MEMLOCK`：允许锁定在物理内存中的最大字节数。
    MEMLOCK = 8,
    /// `RLIMIT_AS`：进程虚拟地址空间的最大字节数。
    AS = 9,
    /// `RLIMIT_LOCKS`：文件锁数量上限。
    LOCKS = 10,
    /// `RLIMIT_SIGPENDING`：允许排队挂起的最大信号数量。
    SIGPENDING = 11,
    /// `RLIMIT_MSGQUEUE`：POSIX 消息队列可分配的最大字节数。
    MSGQUEUE = 12,
    /// `RLIMIT_NICE`：可通过 `setpriority` / `nice` 调高的调度优先级上限。
    NICE = 13,
    /// `RLIMIT_RTPRIO`：实时调度策略优先级的上限。
    RTPRIO = 14,
    /// `RLIMIT_RTTIME`：实时调度下不发生阻塞睡眠的最大连续 CPU 微秒数。
    RTTIME = 15,
    /// `RLIM_NLIMITS`：资源限制类别总数哨兵值。
    NLIMITS = 16,
    /// 超出已知范围的非法资源编号。
    #[num_enum(default)]
    ILLEAGAL,
}

/// 处理 `prlimit64` 系统调用：读取和/或更新目标进程的资源软硬限制。
///
/// 当前仅支持针对调用进程自身（`pid == 0`）查询 `STACK`、`NPROC`、`NOFILE` 以及更新 `NOFILE`、`STACK`。
///
/// - **调用上下文**：普通系统调用上下文。
/// - **参数**：
///   - `pid`：目标进程 ID（`0` 表示当前进程）。
///   - `resource`：资源类型编号，对应 [`Resource`]。
///   - `new_limit`：若非空，指向用户态提供的新资源限制；为空则不修改。
///   - `old_limit`：若非空，用于写回修改前的资源限制；为空则不输出旧值。
/// - **返回值**：成功返回 `SUCCESS`（0），非法资源码返回 `EINVAL`，用户指针拷贝失败返回 `EFAULT`。
pub fn sys_prlimit(
    pid: usize,
    resource: u32,
    new_limit: *const RLimit,
    old_limit: *mut RLimit,
) -> isize {
    if pid == 0 {
        // Step 1: 提取当前任务页表 Token 与资源枚举，并立即释放任务内部锁，避免持锁执行后续用户拷贝。
        let task = current_task().unwrap();
        let inner = task.acquire_inner_lock();
        let token = task.get_user_token();
        let resource = Resource::from_primitive(resource);
        info!("[sys_prlimit] pid: {}, resource: {:?}", pid, resource);

        drop(inner);
        // Step 2: 若调用方请求读取旧限额（old_limit 非空），将当前有效限额拷贝至用户空间。
        if !old_limit.is_null() {
            match resource {
                Resource::STACK => {
                    if copy_to_user(
                        token,
                        &(RLimit {
                            rlim_cur: USER_STACK_SIZE,
                            rlim_max: USER_STACK_SIZE,
                        }),
                        old_limit,
                    )
                    .is_err()
                    {
                        log::error!("[sys_prlimit] Failed to copy to {:?}", old_limit);
                        return EFAULT;
                    }
                }
                Resource::NPROC => {
                    if copy_to_user(
                        token,
                        &(RLimit {
                            rlim_cur: SYSTEM_TASK_LIMIT,
                            rlim_max: SYSTEM_TASK_LIMIT,
                        }),
                        old_limit,
                    )
                    .is_err()
                    {
                        log::error!("[sys_prlimit] Failed to copy to {:?}", old_limit);
                        return EFAULT;
                    }
                }
                Resource::NOFILE => {
                    let lock = task.files.lock();
                    if copy_to_user(
                        token,
                        &(RLimit {
                            rlim_cur: lock.get_soft_limit(),
                            rlim_max: lock.get_hard_limit(),
                        }),
                        old_limit,
                    )
                    .is_err()
                    {
                        log::error!("[sys_prlimit] Failed to copy to {:?}", old_limit);
                        return EFAULT;
                    }
                }
                Resource::ILLEAGAL => return EINVAL,
                _ => todo!(),
            }
        }

        // Step 3: 若调用方提供了新限额（new_limit 非空），从用户空间拷入并应用更新。
        if !new_limit.is_null() {
            let rlimit = &mut RLimit {
                rlim_cur: 0,
                rlim_max: 0,
            };
            if copy_from_user(token, new_limit, rlimit).is_err() {
                log::error!("[sys_prlimit] Failed to copy from {:?}", new_limit);
                return EFAULT;
            };
            match resource {
                Resource::NOFILE => {
                    task.files.lock().set_soft_limit(rlimit.rlim_cur);
                    task.files.lock().set_hard_limit(rlimit.rlim_max);
                }
                Resource::STACK => {
                    warn!("[prlimit] Unsupported modification stack");
                    assert!(rlimit.rlim_cur <= USER_STACK_SIZE);
                }
                Resource::ILLEAGAL => return EINVAL,
                _ => todo!(),
            }
        }
    } else {
        todo!();
    }
    SUCCESS
}

/// 处理 `set_robust_list` 系统调用：登记当前线程的用户态健壮互斥锁（Robust Futex）链表头地址。
///
/// 当线程异常退出时，内核可根据该链表头遍历并唤醒等待在遗留锁上的其他线程。
///
/// - **调用上下文**：普通系统调用上下文。
/// - **参数**：
///   - `head`：用户态 `struct robust_list_head` 的首地址。
///   - `len`：结构体字节长度，必须等于 `RobustList::HEAD_SIZE`（24 字节）。
/// - **返回值**：成功返回 `SUCCESS`（0），长度不匹配返回 `EINVAL`。
pub fn sys_set_robust_list(head: usize, len: usize) -> isize {
    if len != crate::task::RobustList::HEAD_SIZE {
        return EINVAL;
    }

    let task = current_task().unwrap();
    let mut inner = task.acquire_inner_lock();
    inner.robust_list.head = head;
    // RobustList::HEAD_SIZE 在入口已校验为固定值，此处仅需更新 head 指针。
    SUCCESS
}

/// 处理 `get_robust_list` 系统调用：查询指定线程已登记的健壮互斥锁链表头地址及结构体大小。
///
/// - **调用上下文**：普通系统调用上下文。
/// - **参数**：
///   - `pid`：目标线程 ID（`0` 表示当前调用线程）。
///   - `head_ptr`：用于接收链表头地址的用户态指针。
///   - `len_ptr`：用于接收结构体长度的用户态指针。
/// - **返回值**：成功返回 `SUCCESS`（0），找不到目标线程返回 `ESRCH`，用户态拷贝失败返回 `EFAULT`。
pub fn sys_get_robust_list(pid: u32, head_ptr: *mut usize, len_ptr: *mut usize) -> isize {
    let task = if pid == 0 {
        current_task().unwrap()
    } else {
        match find_task_by_pid(pid as usize) {
            Some(task) => task,
            None => return ESRCH,
        }
    };
    let inner = task.acquire_inner_lock();
    let token = current_user_token();
    if copy_to_user(token, &inner.robust_list.head, head_ptr).is_err() {
        log::error!("[sys_get_robust_list] Failed to copy to {:?}", head_ptr);
        return EFAULT;
    };
    if copy_to_user(token, &inner.robust_list.len, len_ptr).is_err() {
        log::error!("[sys_get_robust_list] Failed to copy to {:?}", len_ptr);
        return EFAULT;
    };
    SUCCESS
}

/// 处理 `membarrier` 系统调用：多核跨线程内存屏障兼容桩实现。
///
/// 用于替代用户态高频快路径上的显式内存屏障指令；当前作为兼容桩直接记录告警日志并返回成功。
///
/// - **调用上下文**：普通系统调用上下文。
pub fn sys_memorybarrier(_cmd: usize, _flags: usize, _cpu_id: usize) -> isize {
    error!("[sys_memorybarrier]=========PSEUDOIMPLEMENTATION=========");
    error!(
        "This system call is only needed by the multicore environment for faster synchronization."
    );
    error!("In theory, it can be replaced (INefficiently) by fencing.");
    return SUCCESS;
}

/// 处理 `madvise` 系统调用：接收用户态对虚拟内存区间的访问模式提示。
///
/// 由于建议性提示不影响语义正确性，当前实现直接忽略提示参数并返回 `SUCCESS`（0）。
///
/// - **调用上下文**：普通系统调用上下文。
pub fn sys_madvise(addr: usize, length: usize, advice: u32) -> isize {
    // 预留给后续页面回收或预读优化，当前无需额外操作。
    SUCCESS
}