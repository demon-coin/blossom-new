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

pub fn sys_shutdown() -> isize {
    shutdown()
}

#[allow(non_camel_case_types)]
#[derive(Debug, Eq, PartialEq, FromPrimitive)]
#[repr(u32)]
pub enum SyslogAction {
    CLOSE = 0,
    OPEN = 1,
    READ = 2,
    READ_ALL = 3,
    READ_CLEAR = 4,
    CLEAR = 5,
    CONSOLE_OFF = 6,
    CONSOLE_ON = 7,
    CONSOLE_LEVEL = 8,
    SIZE_UNREAD = 9,
    SIZE_BUFFER = 10,
    #[default]
    ILLEAGAL,
}

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

#[allow(unused)]
#[repr(C)]
pub struct UTSName {
    sysname: [u8; 65],
    nodename: [u8; 65],
    release: [u8; 65],
    version: [u8; 65],
    machine: [u8; 65],
    domainname: [u8; 65],
}

pub fn sys_uname(buf: *mut u8) -> isize {
    let token = current_user_token();
    let mut buffer = UserBuffer::new(
        match translated_byte_buffer(token, buf, size_of::<UTSName>()) {
            Ok(buffer) => buffer,
            Err(errno) => return errno,
        },
    );

    // A little stupid but still efficient.
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

pub fn sys_getuid() -> isize {
    0 // root user
}

pub fn sys_geteuid() -> isize {
    0 // root user
}

pub fn sys_getgid() -> isize {
    0 // root group
}

pub fn sys_getegid() -> isize {
    0 // root group
}

#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct Sysinfo {
    uptime: usize,     /* Seconds since boot */
    loads: [usize; 3], /* 1, 5, and 15 minute load averages */
    totalram: usize,   /* Total usable main memory size */
    freeram: usize,    /* Available memory size */
    sharedram: usize,  /* Amount of shared memory */
    bufferram: usize,  /* Memory used by buffers */
    totalswap: usize,  /* Total swap space size */
    freeswap: usize,   /* Swap space still available */
    procs: u16,        /* Number of current processes */
    totalhigh: usize,  /* Total high memory size */
    freehigh: usize,   /* Available high memory size */
    mem_unit: u32,     /* Memory unit size in bytes */
                       //char __reserved[256];
                       // In the above structure, sizes of the memory and swap fields are given as multiples of mem_unit bytes.
}

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
            // Use only current sample (as average) to evaluate
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

#[allow(unused)]
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct RLimit {
    rlim_cur: usize, /* Soft limit */
    rlim_max: usize, /* Hard limit (ceiling for rlim_cur) */
}

#[derive(Debug, Eq, PartialEq, FromPrimitive)]
#[repr(u32)]
pub enum Resource {
    CPU = 0,
    FSIZE = 1,
    DATA = 2,
    STACK = 3,
    CORE = 4,
    RSS = 5,
    NPROC = 6,
    NOFILE = 7,
    MEMLOCK = 8,
    AS = 9,
    LOCKS = 10,
    SIGPENDING = 11,
    MSGQUEUE = 12,
    NICE = 13,
    RTPRIO = 14,
    RTTIME = 15,
    NLIMITS = 16,
    #[num_enum(default)]
    ILLEAGAL,
}

/// It can be used to both set and get the resource limits of an arbitrary process.
/// # WARNING
/// Partial implementation
pub fn sys_prlimit(
    pid: usize,
    resource: u32,
    new_limit: *const RLimit,
    old_limit: *mut RLimit,
) -> isize {
    if pid == 0 {
        let task = current_task().unwrap();
        let inner = task.acquire_inner_lock();
        let token = task.get_user_token();
        let resource = Resource::from_primitive(resource);
        info!("[sys_prlimit] pid: {}, resource: {:?}", pid, resource);

        drop(inner);
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

pub fn sys_set_robust_list(head: usize, len: usize) -> isize {
    if len != crate::task::RobustList::HEAD_SIZE {
        return EINVAL;
    }

    let task = current_task().unwrap();
    let mut inner = task.acquire_inner_lock();
    inner.robust_list.head = head;
    //inner.robust_list.len = len;
    SUCCESS
}

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

/// # Versions
/// The membarrier() system call was added in Linux 4.3.
/// Before Linux 5.10, the prototype for membarrier() was:
/// `int membarrier(int cmd, int flags);`
pub fn sys_memorybarrier(_cmd: usize, _flags: usize, _cpu_id: usize) -> isize {
    error!("[sys_memorybarrier]=========PSEUDOIMPLEMENTATION=========");
    error!(
        "This system call is only needed by the multicore environment for faster synchronization."
    );
    error!("In theory, it can be replaced (INefficiently) by fencing.");
    return SUCCESS;
}

pub fn sys_madvise(addr: usize, length: usize, advice: u32) -> isize {
    // info!("[sys_madvise] addr: {}, length: {}, advice: {}", addr, length, advice);
    SUCCESS
}