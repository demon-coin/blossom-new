//! 每次调用只读取一次用户寄存器；返回值与退出是互斥控制结果，阻塞时不借用户现场。

mod affinity;
mod error;
mod file;
mod futex;
mod memory;
mod net;
mod poll;
pub(crate) mod signal;
mod system;
mod task;
pub(crate) mod time;

use alloc::vec::Vec;

use crate::arch::UserContext;
use crate::fs::ReadBuffer;
use crate::klog;
use crate::random;
use crate::sched;
use crate::syscall::error::{copyout, SysResult};
use crate::task::process::{EXIT_STATUS_MASK, EXIT_STATUS_SHIFT};
use crate::task::UserThread;
pub(crate) use error::Errno;

#[cfg(target_arch = "riscv64")]
const ARGUMENT_REGISTER: usize = 10;
#[cfg(target_arch = "loongarch64")]
const ARGUMENT_REGISTER: usize = 4;

#[cfg(target_arch = "riscv64")]
const NUMBER_REGISTER: usize = 17;
#[cfg(target_arch = "loongarch64")]
const NUMBER_REGISTER: usize = 11;

const SYSCALL_INSTRUCTION_BYTES: usize = 4;
const RESTART_SYSCALL: usize = 128;

#[derive(Clone, Copy, Debug)]
pub(crate) struct Request {
    pub(crate) pc: usize,
    pub(crate) number: usize,
    pub(crate) arguments: [usize; 6],
}

impl Request {
    /// 在进入可阻塞分派之前保存原 PC、编号及六个 raw 参数，不再读取硬件 CSR。
    pub(crate) fn snapshot(context: &UserContext) -> Self {
        Self {
            pc: context.pc,
            number: context.gpr[NUMBER_REGISTER],
            arguments: core::array::from_fn(|index| context.gpr[ARGUMENT_REGISTER + index]),
        }
    }

    /// 只在普通返回时写 a0 并越过原 syscall；fault、中断和其他控制转移不调用此尾部。
    pub(crate) fn return_to(self, context: &mut UserContext, result: isize) {
        context.gpr[ARGUMENT_REGISTER] = result as usize;
        context.pc = self
            .pc
            .checked_add(SYSCALL_INSTRUCTION_BYTES)
            .expect("syscall PC overflow");
    }

    /// 重放仅由内部restart结果授权的原始调用，恢复参数与原PC而不推断用户指令。
    pub(crate) fn restart_to(self, context: &mut UserContext, restart_block: bool) {
        context.gpr[ARGUMENT_REGISTER..ARGUMENT_REGISTER + self.arguments.len()]
            .copy_from_slice(&self.arguments);
        context.gpr[NUMBER_REGISTER] = if restart_block {
            RESTART_SYSCALL
        } else {
            self.number
        };
        context.pc = self.pc;
    }
}

#[derive(Clone, Copy)]
pub(crate) enum Restart {
    Syscall {
        request: Request,
    },
    NoHandler {
        request: Request,
    },
    Sleep {
        request: Request,
        deadline: crate::time::MonotonicInstant,
        remaining: Option<usize>,
    },
    Futex {
        request: Request,
        deadline: crate::time::MonotonicInstant,
    },
}

pub(crate) enum Outcome {
    Return(isize),
    Exit(i32),
    GroupExit(i32),
    Restart(Restart),
    Restored,
    NewImage,
}

/// 信号选择后依据已选handler策略决定EINTR或重启，剩余时间交付先于现场借用。
pub(crate) fn resolve_restart(user: &UserThread, state: Restart, handler_flags: Option<usize>) {
    let (request, restart) = match state {
        Restart::Syscall { request } => (
            request,
            handler_flags.is_none_or(|flags| flags & crate::task::signal::SA_RESTART != 0),
        ),
        Restart::NoHandler { request }
        | Restart::Sleep { request, .. }
        | Restart::Futex { request, .. } => (request, handler_flags.is_none()),
    };

    if restart {
        if matches!(state, Restart::NoHandler { .. }) {
            user.restore_saved_mask();
        }
        let block = matches!(state, Restart::Sleep { .. } | Restart::Futex { .. });
        if block {
            user.save_restart(state);
        }
        request.restart_to(&mut user.context(), block);
    } else {
        let result = match state {
            Restart::Sleep {
                deadline,
                remaining,
                ..
            } => time::interrupt_sleep(user, deadline, remaining).unwrap_or_else(Errno::result),
            _ => Errno::Interrupted.result(),
        };
        request.return_to(&mut user.context(), result);
    }
}

/// 分派真实调用；无进度I/O和ioctl中断交由用户返回裁决重启，未知编号保持ENOSYS。
pub(crate) fn dispatch(user: &UserThread, request: Request) -> Outcome {
    let raw = request.arguments;
    if matches!(request.number, 20..=22 | 72 | 73) {
        return poll::dispatch(user, request);
    }
    if request.number == 98 {
        return futex::dispatch(user, request);
    }
    if matches!(request.number, 101..=103 | 113..=115 | 153 | 165 | 169) {
        return time::dispatch(user, request);
    }
    if request.number == RESTART_SYSCALL {
        return match user.take_restart() {
            Some(Restart::Sleep {
                request,
                deadline,
                remaining,
            }) => time::resume_sleep(user, request, deadline, remaining),
            Some(Restart::Futex { request, deadline }) => {
                futex::resume_wait(user, request, deadline)
            }
            Some(Restart::Syscall { .. } | Restart::NoHandler { .. }) | None => {
                Outcome::Return(Errno::Interrupted.result())
            }
        };
    }
    if matches!(request.number, 129..=139 | 240) {
        return signal::dispatch(user, request);
    }
    if matches!(request.number, 198..=212 | 242) {
        return net::dispatch(user, request);
    }
    if matches!(request.number, 29 | 63..=66 | 76 | 285) {
        let result = match request.number {
            29 => file::ioctl(user, raw),
            63 => file::read(user, raw),
            64 => file::write(user, raw),
            65 => file::readv(user, raw),
            66 => file::writev(user, raw),
            76 => file::splice(user, raw),
            285 => file::copy_file_range(user, raw),
            _ => unreachable!(),
        };
        return match result {
            Err(Errno::RestartSyscall) => Outcome::Restart(Restart::Syscall { request }),
            other => Outcome::Return(other.unwrap_or_else(Errno::result)),
        };
    }

    let result: SysResult = match request.number {
        93 => {
            return Outcome::Exit(
                ((raw[0] & EXIT_STATUS_MASK as usize) << EXIT_STATUS_SHIFT) as i32,
            )
        }
        94 => {
            return Outcome::GroupExit(
                ((raw[0] & EXIT_STATUS_MASK as usize) << EXIT_STATUS_SHIFT) as i32,
            )
        }
        17 => file::getcwd(user, raw),
        23 => file::dup(user, raw[0]),
        24 => file::dup3(user, raw),
        25 => file::fcntl(user, raw),
        34 => file::mkdirat(user, raw),
        35 => file::unlinkat(user, raw),
        36 => file::symlinkat(user, raw),
        37 => file::linkat(user, raw),
        39 => system::umount(user, &raw),
        40 => system::mount(user, &raw),
        43 => system::statfs(user, &raw),
        44 => system::fstatfs(user, &raw),
        45 => file::truncate(user, raw),
        46 => file::ftruncate(user, raw),
        48 => file::faccessat(user, raw, 0),
        49 => file::chdir(user, raw[0]),
        50 => file::fchdir(user, raw),
        51 => file::chroot(user, raw[0]),
        52 => file::fchmod(user, raw),
        53 => file::fchmodat(user, raw),
        56 => file::openat(user, raw),
        57 => file::close(user, raw[0]),
        59 => file::pipe2(user, raw),
        61 => file::getdents(user, raw),
        62 => file::seek(user, raw),
        67 => file::pread(user, raw),
        68 => file::pwrite(user, raw),
        78 => file::readlink(user, raw),
        79 => file::statat(user, raw),
        80 => file::fstat(user, raw),
        81 => system::sync(user),
        82 => file::fsync(user, raw[0]),
        83 => file::fdatasync(user, raw[0]),
        88 => file::utimensat(user, raw),
        116 => syslog(user, raw),
        166 => file::umask(user, raw[0]),
        95 => return task::waitid(user, request),
        96 => Ok(user.set_tid_address(raw[0]) as isize),
        154..=157 | 173..=177 => task::identity(user, request.number, raw),
        160 => system::uname(user, &raw),
        163 => task::getrlimit(user, raw),
        164 => task::setrlimit(user, raw),
        261 => task::prlimit64(user, raw),
        142 => return system::reboot(user, &raw),
        122 => affinity::set(user, raw),
        123 => affinity::get(user, raw),
        168 => affinity::getcpu(user, raw),
        124 => {
            sched::yield_now();
            Ok(0)
        }
        172 | 178 => {
            let identity = user.identity().expect("syscall has no user identity");
            Ok(if request.number == 172 {
                identity.tgid()
            } else {
                identity.tid()
            } as isize)
        }
        179 => system::sysinfo(user, &raw),
        214 => Ok(memory::brk(user, raw[0])),
        215 => memory::munmap(user, raw),
        220 => task::clone(user, request),
        221 => return task::execve(user, request),
        222 => memory::mmap(user, raw),
        226 => memory::mprotect(user, raw),
        227 => memory::msync(user, raw),
        260 => return task::wait4(user, request),
        276 => file::renameat2(user, raw),
        278 => getrandom(user, raw),
        291 => file::statx(user, raw),
        439 => file::faccessat(user, raw, raw[3] as u32),
        _ => Err(Errno::NoSyscall),
    };
    Outcome::Return(result.unwrap_or_else(Errno::result))
}

/// 交付显式允许的非安全随机字节，地址故障只返回真正写入的前缀。
fn getrandom(user: &UserThread, raw: [usize; 6]) -> SysResult {
    const GRND_NONBLOCK: u32 = 1;
    const GRND_RANDOM: u32 = 2;
    const GRND_INSECURE: u32 = 4;

    let flags = raw[2] as u32;
    if flags & !(GRND_NONBLOCK | GRND_RANDOM | GRND_INSECURE) != 0
        || flags & (GRND_RANDOM | GRND_INSECURE) == (GRND_RANDOM | GRND_INSECURE)
    {
        return Err(Errno::InvalidArgument);
    }
    // 现有字节来源没有安全熵；安全请求不得消费非安全初始化数据。
    if flags & GRND_INSECURE == 0 {
        return Err(if flags & GRND_NONBLOCK != 0 {
            Errno::Again
        } else {
            Errno::NoSyscall
        });
    }

    let mm = user.mm().expect("getrandom has no MM");
    let mut target = ReadBuffer::user(&mm, raw[0], raw[1].min(file::MAX_RW_COUNT))?;
    let mut bytes = [0; 256];
    while target.copied() < target.len() {
        let count = bytes.len().min(target.len() - target.copied());
        random::fill_insecure(&mut bytes[..count]);
        if let Err(error) = target.push(&bytes[..count]) {
            return if target.copied() == 0 {
                Err(error.into())
            } else {
                Ok(target.copied() as isize)
            };
        }
    }

    Ok(target.copied() as isize)
}

/// 从同一记录环读取有界的最后完整正文；不清环、不移动打印者或 kmsg 的游标。
fn syslog(user: &UserThread, raw: [usize; 6]) -> SysResult {
    const SYSLOG_ACTION_READ_ALL: i32 = 3;
    const SYSLOG_ACTION_SIZE_BUFFER: i32 = 10;

    let action = raw[0] as i32;
    if action == SYSLOG_ACTION_SIZE_BUFFER {
        return Ok((klog::CAPACITY * klog::TEXT_CAPACITY) as isize);
    }
    if action != SYSLOG_ACTION_READ_ALL {
        return Err(if (0..SYSLOG_ACTION_SIZE_BUFFER).contains(&action) {
            Errno::NoSyscall
        } else {
            Errno::InvalidArgument
        });
    }
    let capacity = raw[2] as i32;
    if raw[1] == 0 || capacity < 0 {
        return Err(Errno::InvalidArgument);
    }
    if capacity == 0 {
        return Ok(0);
    }

    let end = klog::next_sequence();
    let mut cursor = klog::Cursor::oldest();
    let mut text = Vec::new();
    let mut lengths = [0; klog::CAPACITY];
    let mut count = 0;
    while count < lengths.len() {
        let Some(read) = cursor.read() else {
            break;
        };
        let record = read.record;
        if end <= record.seq {
            break;
        }
        if text.capacity() == 0 {
            text.try_reserve_exact(klog::CAPACITY * (klog::TEXT_CAPACITY + 4))
                .map_err(|_| Errno::NoMemory)?;
        }
        let begin = text.len();
        let level = match record.level {
            klog::Level::Error => b'3',
            klog::Level::Warn => b'4',
            klog::Level::Info => b'6',
            klog::Level::Debug => b'7',
        };
        text.extend_from_slice(&[b'<', level, b'>']);
        text.extend_from_slice(record.text().as_bytes());
        if !record.text().ends_with('\n') {
            text.push(b'\n');
        }
        lengths[count] = text.len() - begin;
        count += 1;
    }

    let mut start = 0;
    for length in &lengths[..count] {
        if text.len() - start <= capacity as usize {
            break;
        }
        start += length;
    }

    let output = &text[start..];
    let mm = user.mm().expect("syslog has no MM");
    copyout(&mm, raw[1], output)?;
    Ok(output.len() as isize)
}
