//! I/O 多路复用系统调用实现（`epoll_create1` / `epoll_ctl` / `epoll_pwait` / `pselect6` / `ppoll`）。
//!
//! # 模块职责与架构定位
//! 本模块位于系统调用分发层与 VFS 事件轮询框架（`crate::fs::poll`、`crate::fs::epoll`）之间：
//! - **上游调用者**：`crate::syscall` 总分发器，通过 [`dispatch`] 将系统调用号 `20`、`21`、`22`、`72`、`73` 路由至此。
//! - **下游依赖**：进程打开文件表 [`FileTable`]、可轮询对象接口 `Pollable`、等待登记表 [`PollTable`] / [`PollContext`]、信号子系统（临时信号掩码替换与挂起中断检查）以及用户地址空间 [`Mm`]（逐页安全拷贝 `copyin` / `copyout`）。
//!
//! # 核心机制与等待状态机
//! ## 1. `ppoll` / `pselect6` 无锁快速探测与阻塞等待状态机
//! ```text
//! 进入 wait_fds
//!   │
//!   ▼
//! [步骤 A: 首轮无登记快速扫描 (scan_fds)]
//!   ├─ 遍历感兴趣的文件描述符，直接调用 provider.poll(None) 查询当前就绪位
//!   ├─ 若已有就绪项 (count > 0) / 信号挂起 / 零超时已到期 ──► 立即返回，免去等待队列分配与加锁开销
//!   │
//!   ▼ (无就绪项且需阻塞等待)
//! [步骤 B: 构造 RegisteredPoll 并向各事件源注册通知回调]
//!   ├─ 克隆并持有每个参与轮询的打开文件对象引用 (FileRef)，防止并发 close/dup 偷换底层对象
//!   ├─ 调用 provider.poll(Some(&mut table)) 将 PollContext 挂入各文件的唤醒链表
//!   │
//!   ▼
//! [步骤 C: 循环二次扫描与可中断睡眠]
//!   ├─ 1. context.clear_triggered() 清除唤醒标记（必须在扫描前清除，避免漏掉扫描期间到达的唤醒）
//!   ├─ 2. registered.scan() 扫描已持有的打开文件引用
//!   ├─ 3. 若就绪 / 信号中断 / 绝对超时到期 ──► 退出循环，按字段声明顺序先析构 PollTable 注销回调，再释放 FileRef
//!   └─ 4. context.park_until(true, deadline) 阻塞当前线程直至被事件源唤醒、信号打断或超时
//! ```
//!
//! ## 2. 用户态 ABI 字节布局
//! ```text
//! struct pollfd (8 字节, 小端序):
//! +-------------------+-------------------+-------------------+
//! | 0..4: fd (i32)    | 4..6: events(u16) | 6..8: revents(u16)|
//! +-------------------+-------------------+-------------------+
//!
//! struct epoll_event (16 字节, RISC-V / LoongArch 64 非紧凑对齐布局):
//! +-------------------+-------------------+---------------------------------------+
//! | 0..4: events(u32) | 4..8: pad (0u32)  | 8..16: data (u64, 原样透传用户态数据) |
//! +-------------------+-------------------+---------------------------------------+
//! ```
//!
//! # 锁获取顺序与生命周期契约
//! 1. **文件表锁与事件源锁解耦**：查找文件描述符时仅短暂获取 `FileTable` 锁并克隆 `FileRef`（`Arc`），随后在表锁外调用 `poll` 或 `epoll.control`。
//! 2. **析构顺序不变量（[`RegisteredPoll`]）**：结构体字段必须保持 `_table: PollTable` 在 `files: Vec<...>` 之前，确保 Rust 自动析构时先从各底层事件源的唤醒队列中注销回调，最后才释放打开文件引用（`Drop`）。
//! 3. **临时信号掩码恢复契约**：若设置了临时信号掩码，正常返回、超时或非中断错误均在内核返回前恢复原掩码；若因 `EINTR` 中断，则保留临时掩码直至用户态信号处理函数选取完毕，由信号返回路径负责恢复。

use alloc::sync::Arc;
use alloc::vec::Vec;
use core::convert::TryInto;

use crate::arch;
use crate::fs::epoll::{EpollEvent, EpollFile, EPOLL_CTL_ADD as ADD, EPOLL_CTL_MOD as MOD};
use crate::fs::poll::{PollContext, PollMask, PollNotify, PollTable};
use crate::fs::{DescriptorFlags, FileRef, FileTable, FsError};
use crate::mm::space::Mm;
use crate::syscall::error::{copyin, copyout, Errno, SysResult};
use crate::syscall::file::nofile;
use crate::syscall::{time as time_abi, Outcome, Request, Restart};
use crate::task::signal::SIGSET_BYTES;
use crate::task::UserThread;
use crate::time::{self, Duration, MonotonicInstant, NSEC_PER_MSEC, NSEC_PER_SEC};
use crate::wait::WaitMode;

const POLLFD_SIZE: usize = 8;
const INLINE_POLL_BYTES: usize = 128 * POLLFD_SIZE;
const EVENT_SIZE: usize = 16;
const CLOEXEC: u32 = crate::fs::O_CLOEXEC;

const DEFAULT_POLL: PollMask = PollMask(
    PollMask::POLLIN.0 | PollMask::POLLOUT.0 | PollMask::POLLRDNORM.0 | PollMask::POLLWRNORM.0,
);
const READ_SET: PollMask = PollMask(
    PollMask::POLLIN.0
        | PollMask::POLLRDNORM.0
        | PollMask::POLLRDBAND.0
        | PollMask::POLLHUP.0
        | PollMask::POLLERR.0
        | PollMask::POLLNVAL.0,
);
const WRITE_SET: PollMask = PollMask(
    PollMask::POLLOUT.0
        | PollMask::POLLWRNORM.0
        | PollMask::POLLWRBAND.0
        | PollMask::POLLERR.0
        | PollMask::POLLNVAL.0,
);
const EXCEPT_SET: PollMask = PollMask(PollMask::POLLPRI.0 | PollMask::POLLNVAL.0);
const ALWAYS_POLL: PollMask =
    PollMask(PollMask::POLLERR.0 | PollMask::POLLHUP.0 | PollMask::POLLNVAL.0);

/// `ppoll` 与 `pselect6` 解析后的超时控制状态。
#[derive(Clone, Copy)]
struct Timeout {
    /// 单调时钟下的绝对截止时间点；`None` 表示无限期等待。
    deadline: Option<MonotonicInstant>,
    /// 若用户传入了非零相对 `timespec` 指针，则记录其用户态虚拟地址，用于在返回前写回剩余时间。
    remaining: Option<usize>,
}

/// `pselect6` 系统调用在内核中维护的读、写、异常三组位图缓冲。
struct SelectSets {
    /// 用户态传入的 `[readfds, writefds, exceptfds]` 三个位图起始虚拟地址（0 表示空指针）。
    addresses: [usize; 3],
    /// 从用户态拷入的三组输入监听位图快照（长度均为 `span` 字节）。
    input: [Vec<u8>; 3],
    /// 当前轮询周期产生就绪事件的三组输出位图缓冲（长度均为 `span` 字节）。
    output: [Vec<u8>; 3],
    /// 实际参与扫描的最大文件描述符数量上限（已与进程已分配槽位数取较小值）。
    nfds: usize,
    /// 每个位图按 8 字节（`u64`）机器字向上对齐后的实际字节跨度。
    span: usize,
}

/// 统一封装 `ppoll`（数组格式）与 `pselect6`（位图格式）的轮询输入与输出集合。
enum Interests<'a> {
    /// `ppoll` 使用的连续 `struct pollfd` 原始字节切片（每项 8 字节，原地更新 `revents`）。
    Poll(&'a mut [u8]),
    /// `pselect6` 使用的三组输入/输出位图结构。
    Select(&'a mut SelectSets),
}
/// 已向底层事件源注册唤醒回调的轮询会话。
///
/// # 字段析构顺序不变量
/// Rust 按结构体字段声明顺序依次执行析构（Drop）：
/// `_table` 必须声明在 `files` 之前，以保证退出等待时先同步撤销 `_table` 在各事件源上的唤醒登记，
/// 随后再释放登记期间持有的打开文件引用 [`FileRef`]。
struct RegisteredPoll {
    /// 当前轮询线程关联的唤醒上下文（用于清除触发标记与阻塞睡眠）。
    context: Arc<PollContext>,
    /// 维护所有已注册事件源回调项的轮询登记表；析构时自动从各等待队列移除监听。
    _table: PollTable,
    /// 参与本次轮询的 `(请求项下标, 打开文件强引用)` 列表；无效 `fd` 对应 `None`。
    files: Vec<(usize, Option<FileRef>)>,
}
/// I/O 多路复用系统调用分发入口。
///
/// # 调用上下文与参数
/// - 运行于普通可睡眠的用户任务上下文。
/// - 负责路由 `epoll_create1` (20)、`epoll_ctl` (21)、`epoll_pwait` (22)、`pselect6` (72) 与 `ppoll` (73)。
pub(super) fn dispatch(user: &UserThread, request: Request) -> Outcome {
    match request.number {
        20 => Outcome::Return(create(user, request.arguments[0]).unwrap_or_else(Errno::result)),
        21 => Outcome::Return(control(user, request.arguments).unwrap_or_else(Errno::result)),
        22 => epoll_pwait(user, request),
        72 => pselect(user, request),
        73 => ppoll(user, request),
        _ => unreachable!("non-poll request entered poll dispatcher"),
    }
}

/// 处理 `epoll_create1` 系统调用：创建新的 epoll 实例并分配文件描述符。
///
/// # 调用上下文与参数
/// - 运行于普通可睡眠的用户任务上下文。
/// - `raw`: 打开标志位（仅支持 `EPOLL_CLOEXEC` / `O_CLOEXEC`，其余未知位返回 `EINVAL`）。
///
/// # 返回值契约
/// - 先在进程文件表中预留描述符槽位，待 [`EpollFile`] 实例成功构造后才正式发布该槽位并返回文件描述符编号。
fn create(user: &UserThread, raw: usize) -> SysResult {
    let flags = raw as u32;
    if flags & !CLOEXEC != 0 {
        return Err(Errno::InvalidArgument);
    }

    let runtime = user.runtime().expect("epoll_create1 has no runtime");
    let reservation = runtime.files.reserve(nofile(user))?;
    let file: FileRef = EpollFile::try_new(flags)?;

    Ok(reservation.publish(
        file,
        DescriptorFlags {
            close_on_exec: flags & CLOEXEC != 0,
        },
    ) as isize)
}

/// 处理 `epoll_ctl` 系统调用：向目标 epoll 实例添加、修改或删除监听的文件描述符。
///
/// # 调用上下文与参数
/// - 运行于普通可睡眠的用户任务上下文。
/// - `raw`: `[epfd, op, fd, event_ptr, ...]`。
///
/// # 校验顺序与契约
/// 1. 当操作为 `EPOLL_CTL_ADD` 或 `EPOLL_CTL_MOD` 时，先从用户态完整拷入 16 字节 `epoll_event` 结构；`EPOLL_CTL_DEL` 忽略 `event_ptr`。
/// 2. 校验 `epfd` 与目标 `fd` 的有效性，验证目标文件是否支持轮询（不支持则返回 `EPERM`），并禁止将 epoll 实例注册到自身或其他 epoll 实例中。
fn control(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let operation = raw[1] as u32;
    // Step 1: 针对 ADD / MOD 操作，从用户态拷贝并按小端序解码 16 字节的 epoll_event。
    let event = if matches!(operation, ADD | MOD) {
        let mm = user.mm().expect("epoll_ctl has no MM");
        let mut bytes = [0u8; EVENT_SIZE];
        copyin(&mm, &mut bytes, raw[3])?;
        Some(EpollEvent {
            events: u32::from_le_bytes(bytes[..4].try_into().expect("event mask width")),
            data: u64::from_le_bytes(bytes[8..].try_into().expect("event data width")),
        })
    } else {
        None
    };

    // Step 2: 查找 epfd 与目标 fd 对应的打开文件对象，并校验轮询能力与嵌套约束。
    let runtime = user.runtime().expect("epoll_ctl has no runtime");
    let owner = runtime.files.lookup(descriptor(raw[0])?)?;
    let fd = raw[2] as i32;
    let target = runtime.files.lookup(descriptor(raw[2])?)?;
    if target.pollable().is_none() {
        return Err(Errno::NotPermitted);
    }
    let epoll = owner.epoll().ok_or(Errno::InvalidArgument)?;
    if Arc::ptr_eq(&owner, &target) || target.epoll().is_some() {
        return Err(Errno::InvalidArgument);
    }
    if target.watch_head().is_none() {
        return Err(Errno::NotPermitted);
    }

    epoll.control(&owner, operation, &target, fd, event)?;
    Ok(0)
}

/// 处理 `ppoll` 系统调用：在可选超时与临时信号掩码下轮询一组 `pollfd` 描述符。
///
/// # 调用上下文与参数
/// - 运行于普通可睡眠的用户任务上下文。
/// - `raw`: `[fds_ptr, nfds, timeout_ptr, sigmask_ptr, sigsetsize, ...]`。
fn ppoll(user: &UserThread, request: Request) -> Outcome {
    let raw = request.arguments;
    let mm = user.mm().expect("ppoll has no MM");
    let timeout = match relative_timeout(&mm, raw[2]) {
        Ok(timeout) => timeout,
        Err(error) => return Outcome::Return(error.result()),
    };
    if let Err(error) = install_mask(user, &mm, raw[3], raw[4]) {
        return Outcome::Return(error.result());
    }
    let result = core_ppoll(user, &mm, raw[0], raw[1] as u32 as usize, timeout.deadline);
    finish_select(user, &mm, request, timeout, result)
}

/// 执行 `ppoll` 的核心拷贝、等待与 `revents` 写回流程。
///
/// # 内存策略与写回契约
/// - `nfds` 不得超过当前进程的 `RLIMIT_NOFILE` 软限制。
/// - 当数组不超过 128 项（1024 字节）时直接使用栈上缓冲区，超出时在堆上按需分配。
/// - 轮询结束后（含被信号中断的 `EINTR` 情形），仅向用户态数组的每个 `revents` 字段（偏移 `+6..+8`）写回本轮结果，不覆盖用户态的 `fd` 与 `events` 字段。
fn core_ppoll(
    user: &UserThread,
    mm: &Mm,
    address: usize,
    nfds: usize,
    deadline: Option<MonotonicInstant>,
) -> Result<usize, Errno> {
    let runtime = user.runtime().expect("ppoll has no runtime");
    if nofile(user) < nfds {
        return Err(Errno::InvalidArgument);
    }

    // Step 1: 准备内核缓冲区（小数组走栈缓冲，大数组走堆分配），并一次性从用户态拷入全部 pollfd 项。
    let length = nfds * POLLFD_SIZE;
    let mut inline = [0u8; INLINE_POLL_BYTES];
    let mut allocated = Vec::new();
    let bytes = if length <= inline.len() {
        &mut inline[..length]
    } else {
        allocated
            .try_reserve_exact(length)
            .map_err(|_| Errno::NoMemory)?;
        allocated.resize(length, 0);
        allocated.as_mut_slice()
    };
    copyin(mm, bytes, address)?;

    // Step 2: 执行轮询等待，随后将每项的 revents (2 字节) 逐项写回用户空间。
    // 即使等待过程被信号中断，也先把本轮已清零或更新的 revents 交付给用户态；若写回发生缺页错误，则优先返回 EFAULT。
    let result = wait_fds(user, &runtime.files, &mut Interests::Poll(bytes), deadline);
    for (index, entry) in bytes.chunks_exact(POLLFD_SIZE).enumerate() {
        let output = address
            .checked_add(index * POLLFD_SIZE + 6)
            .ok_or(Errno::BadAddress)?;
        copyout(mm, output, &entry[6..8])?;
    }
    result
}

/// 处理 `pselect6` 系统调用：基于三组文件描述符位图、相对超时与第六参数信号掩码包执行多路复用等待。
///
/// # 调用上下文与参数
/// - 运行于普通可睡眠的用户任务上下文。
/// - `raw`: `[nfds, readfds, writefds, exceptfds, timeout_ptr, sigmask_pack_ptr]`。
/// - 第六参数指向用户态 `{ const sigset_t *ss, size_t ss_len }` 二元组（16 字节）；当 `ss` 为空指针时忽略 `ss_len`。
fn pselect(user: &UserThread, request: Request) -> Outcome {
    let raw = request.arguments;
    let mm = user.mm().expect("pselect6 has no MM");
    let mut pack = [0u8; 16];
    if raw[5] != 0 {
        if let Err(error) = copyin(&mm, &mut pack, raw[5]) {
            return Outcome::Return(error.result());
        }
    }
    let mask = word(&pack[..8]) as usize;
    let size = word(&pack[8..]) as usize;
    let timeout = match relative_timeout(&mm, raw[4]) {
        Ok(timeout) => timeout,
        Err(error) => return Outcome::Return(error.result()),
    };
    if let Err(error) = install_mask(user, &mm, mask, size) {
        return Outcome::Return(error.result());
    }
    let result = core_pselect(user, &mm, raw, timeout.deadline);
    finish_select(user, &mm, request, timeout, result)
}

/// 执行 `pselect6` 的位图缓冲分配、用户态位图拷入、轮询等待与结果位图写回。
///
/// # 边界与裁剪契约
/// - `n` 作为有符号 32 位整数解码，若为负数则返回 [`Errno::InvalidArgument`]。
/// - 实际扫描上限 `nfds` 取用户请求 `n` 与当前进程文件表已分配槽位数（`allocated_slots`）的较小值，避免为远超实际打开文件数的巨大 `n` 扫描空位。
fn core_pselect(
    user: &UserThread,
    mm: &Mm,
    raw: [usize; 6],
    deadline: Option<MonotonicInstant>,
) -> Result<usize, Errno> {
    let n = raw[0] as i32;
    if n < 0 {
        return Err(Errno::InvalidArgument);
    }

    // Step 1: 计算有效扫描上限 nfds 与按 64 位字对齐的位图字节跨度 span，并分配输入/输出共 6 份位图。
    let runtime = user.runtime().expect("pselect6 has no runtime");
    let nfds = (n as usize).min(runtime.files.allocated_slots());
    let span = nfds.div_ceil(u64::BITS as usize) * core::mem::size_of::<u64>();
    let mut input = core::array::from_fn(|_| Vec::new());
    let mut output = core::array::from_fn(|_| Vec::new());
    for set in input.iter_mut().chain(output.iter_mut()) {
        set.try_reserve_exact(span).map_err(|_| Errno::NoMemory)?;
        set.resize(span, 0);
    }
    let mut sets = SelectSets {
        addresses: [raw[1], raw[2], raw[3]],
        input,
        output,
        nfds,
        span,
    };

    // Step 2: 从用户态拷入非空的 readfds / writefds / exceptfds 位图快照。
    for index in 0..3 {
        if sets.addresses[index] != 0 {
            copyin(
                mm,
                &mut sets.input[index][..sets.span],
                sets.addresses[index],
            )?;
        }
    }

    // Step 3: 执行多路复用轮询等待，并在成功后将输出位图写回用户态。
    let count = wait_fds(
        user,
        &runtime.files,
        &mut Interests::Select(&mut sets),
        deadline,
    )?;

    for index in 0..3 {
        if sets.addresses[index] != 0 {
            copyout(mm, sets.addresses[index], &sets.output[index][..sets.span])?;
        }
    }
    Ok(count)
}

impl Interests<'_> {
    /// 返回本次轮询需要遍历的条目总数（`ppoll` 为数组项数，`pselect6` 为裁剪后的 `nfds` 上限）。
    fn len(&self) -> usize {
        match self {
            Self::Poll(bytes) => bytes.len() / POLLFD_SIZE,
            Self::Select(sets) => sets.nfds,
        }
    }

    /// 提取第 `index` 项对应的非负文件描述符编号。
    ///
    /// - `ppoll`: 若 `pollfd.fd < 0`，按规范忽略该项并返回 `None`。
    /// - `pselect6`: 仅当 `index` 在任一输入位图中被置位时才返回 `Some(index)`。
    fn descriptor(&self, index: usize) -> Option<usize> {
        match self {
            Self::Poll(bytes) => {
                let offset = index * POLLFD_SIZE;
                let fd = i32::from_le_bytes(
                    bytes[offset..offset + 4].try_into().expect("poll fd width"),
                );
                if fd < 0 {
                    None
                } else {
                    Some(fd as usize)
                }
            }
            Self::Select(sets) => {
                if sets.input.iter().any(|set| selected(set, index)) {
                    Some(index)
                } else {
                    None
                }
            }
        }
    }

    /// 在每一轮状态扫描开始前将输出区域清零，保留输入快照不变，确保上一轮残留状态不会混入当前轮。
    fn clear(&mut self) {
        match self {
            Self::Poll(bytes) => {
                for entry in bytes.chunks_exact_mut(POLLFD_SIZE) {
                    entry[6..8].fill(0);
                }
            }
            Self::Select(sets) => sets.output.iter_mut().for_each(|set| set.fill(0)),
        }
    }

    /// 根据文件底层返回的就绪掩码 `mask` 更新第 `index` 项的输出结果，并返回本次新增的就绪计数。
    ///
    /// - `ppoll`: 将 `mask` 与用户监听事件及隐式强制报告事件（`POLLERR | POLLHUP | POLLNVAL`）取交集写入 `revents`；若非空则计为 1 个就绪项。
    /// - `pselect6`: 分别匹配读、写、异常三组事件掩码，置位对应输出位图并累加就绪位数。
    fn record(&mut self, index: usize, mask: PollMask) -> usize {
        match self {
            Self::Poll(bytes) => {
                let offset = index * POLLFD_SIZE;
                let events = u16::from_le_bytes(
                    bytes[offset + 4..offset + 6]
                        .try_into()
                        .expect("poll events width"),
                );
                let returned = mask & (PollMask(u32::from(events)) | ALWAYS_POLL);
                bytes[offset + 6..offset + 8].copy_from_slice(&(returned.0 as u16).to_le_bytes());
                usize::from(!returned.is_empty())
            }
            Self::Select(sets) => {
                let mut count = 0;
                for (which, interest) in [READ_SET, WRITE_SET, EXCEPT_SET].iter().enumerate() {
                    if selected(&sets.input[which], index) && !(mask & *interest).is_empty() {
                        sets.output[which][index / 8] |= 1 << (index % 8);
                        count += 1;
                    }
                }
                count
            }
        }
    }
}

/// 第一轮无登记快速扫描：直接从文件表查找各描述符并查询当前瞬时就绪状态，不挂载等待队列回调。
fn scan_fds(files: &FileTable, interests: &mut Interests<'_>) -> Result<usize, Errno> {
    interests.clear();
    let mut count = 0;
    for index in 0..interests.len() {
        let Some(fd) = interests.descriptor(index) else {
            continue;
        };
        let mask = match files.lookup(fd) {
            Ok(file) => query(&file)?,
            Err(FsError::BadFd) if matches!(interests, Interests::Poll(_)) => PollMask::POLLNVAL,
            Err(error) => return Err(error.into()),
        };
        count += interests.record(index, mask);
    }
    Ok(count)
}

/// 查询单个打开文件的当前就绪事件掩码。
///
/// 若底层文件未实现动态事件轮询接口（如普通磁盘文件），则直接返回固定就绪掩码 [`DEFAULT_POLL`]（可读且可写）。
fn query(file: &FileRef) -> Result<PollMask, FsError> {
    match file.pollable() {
        Some(provider) => provider.poll(None),
        None => Ok(DEFAULT_POLL),
    }
}

impl RegisteredPoll {
    /// 为进入阻塞等待准备 [`RegisteredPoll`]：分配通知上下文、固定参与轮询的打开文件引用，并向各事件源注册唤醒回调。
    ///
    /// 若中途发生错误，已构造的 `RegisteredPoll` 析构时会按字段顺序先撤销已注册的回调，再释放文件引用。
    fn prepare(files: &FileTable, interests: &Interests<'_>) -> Result<Self, Errno> {
        let participating = (0..interests.len())
            .filter(|index| interests.descriptor(*index).is_some())
            .count();
        let mut retained = Vec::new();
        retained
            .try_reserve_exact(participating)
            .map_err(|_| Errno::NoMemory)?;
        let context = PollContext::try_new()?;
        let sink: Arc<dyn PollNotify> = context.clone();
        let table = PollTable::new(sink, participating * 2)?;
        let mut registered = Self {
            context,
            _table: table,
            files: retained,
        };

        for index in 0..interests.len() {
            let Some(fd) = interests.descriptor(index) else {
                continue;
            };
            let file = match files.lookup(fd) {
                Ok(file) => Some(file),
                Err(FsError::BadFd) if matches!(interests, Interests::Poll(_)) => None,
                Err(error) => return Err(error.into()),
            };
            // 先将打开文件强引用推入 files 列表固定生命周期，再将 PollTable 注册到底层事件源。
            registered.files.push((index, file));
            if let Some(file) = &registered.files.last().unwrap().1 {
                if let Some(provider) = file.pollable() {
                    provider.poll(Some(&mut registered._table))?;
                }
            }
        }

        Ok(registered)
    }

    /// 扫描已固定在 [`RegisteredPoll`] 中的打开文件集合，避免睡眠期间并发的 `close` / `dup` 替换等待目标。
    fn scan(&self, interests: &mut Interests<'_>) -> Result<usize, Errno> {
        interests.clear();
        let mut count = 0;
        for (index, file) in &self.files {
            let mask = match file {
                Some(file) => query(file)?,
                None => PollMask::POLLNVAL,
            };
            count += interests.record(*index, mask);
        }
        Ok(count)
    }
}

/// `ppoll` 与 `pselect6` 共用的核心轮询等待循环。
///
/// # 执行流程
/// 1. 首轮调用 [`scan_fds`] 进行无锁无登记快速探测；若已有就绪项、存在待处理信号或零超时已到期，则直接返回。
/// 2. 首轮未就绪时调用 [`RegisteredPoll::prepare`] 注册事件源回调，并在后续每轮扫描前先调用 `clear_triggered()` 清除唤醒标志，再执行二次扫描与 `park_until` 可中断睡眠。
fn wait_fds(
    user: &UserThread,
    files: &FileTable,
    interests: &mut Interests<'_>,
    deadline: Option<MonotonicInstant>,
) -> Result<usize, Errno> {
    let signals = user.signals().expect("poll wait has no signals");
    let mut registered: Option<RegisteredPoll> = None;
    loop {
        if let Some(waiting) = &registered {
            waiting.context.clear_triggered();
        }
        let count = match &registered {
            Some(waiting) => waiting.scan(interests)?,
            None => scan_fds(files, interests)?,
        };
        if 0 < count {
            return Ok(count);
        }
        if signals.interruption_pending(WaitMode::Interruptible) {
            return Err(Errno::Interrupted);
        }
        if deadline.is_some_and(|limit| limit <= time::now()) {
            return Ok(0);
        }

        match &registered {
            Some(waiting) => {
                waiting.context.park_until(true, deadline);
            }
            None => registered = Some(RegisteredPoll::prepare(files, interests)?),
        }
    }
}

/// 处理 `epoll_pwait` 系统调用：等待 epoll 实例产生就绪事件，并将事件拷贝至用户态缓冲区。
///
/// # 调用上下文与参数
/// - 运行于普通可睡眠的用户任务上下文。
/// - `raw`: `[epfd, events_ptr, maxevents, timeout_ms, sigmask_ptr, sigsetsize]`。
/// - `timeout_ms`: 有符号 32 位毫秒数；负数表示无限期等待，`0` 表示非阻塞立即返回，正数转换为单调时钟绝对截止时间。
fn epoll_pwait(user: &UserThread, request: Request) -> Outcome {
    let raw = request.arguments;
    let timeout = raw[3] as i32;
    let deadline = if timeout < 0 {
        None
    } else {
        match absolute_after(timeout as u64 * NSEC_PER_MSEC) {
            Ok(deadline) => Some(deadline),
            Err(error) => return Outcome::Return(error.result()),
        }
    };
    let mm = user.mm().expect("epoll_pwait has no MM");
    if let Err(error) = install_mask(user, &mm, raw[4], raw[5]) {
        return Outcome::Return(error.result());
    }

    let result = core_epoll_wait(user, &mm, raw, deadline);
    if result != Err(Errno::Interrupted) {
        user.restore_saved_mask();
    }
    Outcome::Return(
        result
            .map(|count| count as isize)
            .unwrap_or_else(Errno::result),
    )
}

/// 执行 `epoll_pwait` 的核心参数校验、事件扫描写出与阻塞等待循环。
///
/// # 边界与错误处理契约
/// - 预先校验 `maxevents > 0` 以及用户态输出缓冲区 `[events_ptr, events_ptr + maxevents * 16)` 落在合法用户地址空间内。
/// - 调用 `epoll.scan` 将就绪事件逐个通过 `copyout` 写入用户空间；若中途遇到无效页且此前已成功写出至少一个事件，`epoll.scan` 会保留已交付的正数事件计数并把未交付事件放回就绪队列。
fn core_epoll_wait(
    user: &UserThread,
    mm: &Mm,
    raw: [usize; 6],
    deadline: Option<MonotonicInstant>,
) -> Result<usize, Errno> {
    // Step 1: 校验 maxevents 合法性以及用户态事件数组地址范围。
    let maxevents = raw[2] as i32;
    if maxevents <= 0 || (i32::MAX as usize / EVENT_SIZE) < maxevents as usize {
        return Err(Errno::InvalidArgument);
    }
    let maxevents = maxevents as usize;
    let end = raw[1]
        .checked_add(maxevents * EVENT_SIZE)
        .ok_or(Errno::BadAddress)?;
    if arch::USER_ADDRESS_END < end {
        return Err(Errno::BadAddress);
    }

    // Step 2: 从进程文件表查找 epfd 并确认其为合法的 epoll 实例。
    let runtime = user.runtime().expect("epoll_pwait has no runtime");
    let owner = runtime.files.lookup(descriptor(raw[0])?)?;
    let epoll = owner.epoll().ok_or(Errno::InvalidArgument)?;
    let signals = user.signals().expect("epoll wait has no signals");
    let mut waiting: Option<(Arc<PollContext>, PollTable)> = None;

    // Step 3: 循环扫描就绪队列并拷贝事件至用户态；若当前无就绪事件则注册唤醒回调并进入可中断等待。
    loop {
        if let Some((context, _)) = &waiting {
            context.clear_triggered();
        }
        let mut next = raw[1];
        let count = epoll.scan(maxevents, &mut |event| {
            let bytes = encode_event(event);
            let progress = mm.copyout(next, &bytes);
            if progress.error.is_some() || progress.copied != EVENT_SIZE {
                return Err(FsError::BadAddress);
            }
            next += EVENT_SIZE;
            Ok(())
        })?;
        if 0 < count {
            return Ok(count);
        }
        if signals.interruption_pending(WaitMode::Interruptible) {
            return Err(Errno::Interrupted);
        }
        if deadline.is_some_and(|limit| limit <= time::now()) {
            return Ok(0);
        }

        match &waiting {
            Some((context, _)) => {
                context.park_until(true, deadline);
            }
            None => {
                let context = PollContext::try_new()?;
                let sink: Arc<dyn PollNotify> = context.clone();
                let mut table = PollTable::new(sink, 2)?;
                owner
                    .pollable()
                    .expect("epoll OFD has no provider")
                    .poll(Some(&mut table))?;
                waiting = Some((context, table));
            }
        }
    }
}

/// 将内核 [`EpollEvent`] 编码为目标架构 ABI 规定的 16 字节小端序字节数组（偏移 `4..8` 显式填零初始化）。
fn encode_event(event: EpollEvent) -> [u8; EVENT_SIZE] {
    let mut bytes = [0u8; EVENT_SIZE];
    bytes[..4].copy_from_slice(&event.events.to_le_bytes());
    bytes[8..].copy_from_slice(&event.data.to_le_bytes());
    bytes
}

/// 从用户态读取并安装系统调用期间生效的临时信号屏蔽字，同时将原屏蔽字保存在当前线程中以便返回时恢复。
fn install_mask(user: &UserThread, mm: &Mm, address: usize, size: usize) -> Result<(), Errno> {
    if address == 0 {
        return Ok(());
    }
    if size != SIGSET_BYTES {
        return Err(Errno::InvalidArgument);
    }

    let mut bytes = [0u8; SIGSET_BYTES];
    copyin(mm, &mut bytes, address)?;
    let signals = user.signals().expect("temporary mask has no signals");
    user.save_mask(signals.replace_mask(u64::from_le_bytes(bytes)));
    Ok(())
}

/// 完成 `ppoll` 与 `pselect6` 的收尾处理：按需恢复信号掩码、向用户态写回剩余超时时间，并决定返回值或无处理函数自动重启。
///
/// # 信号掩码与剩余时间契约
/// - 若本次调用未因信号中断（非 `EINTR`），立即恢复原信号掩码；若为 `EINTR`，则保留临时掩码交由信号派发路径处理。
/// - 若用户传入了非零相对超时结构体，则无论调用正常返回还是被中断，均将剩余未耗时写回用户态（若写回失败则取消自动重启并返回原结果）。
fn finish_select(
    user: &UserThread,
    mm: &Mm,
    request: Request,
    timeout: Timeout,
    result: Result<usize, Errno>,
) -> Outcome {
    let interrupted = result == Err(Errno::Interrupted);
    if !interrupted {
        user.restore_saved_mask();
    }

    let mut restart = interrupted;
    if let (Some(address), Some(deadline)) = (timeout.remaining, timeout.deadline) {
        let ns = deadline.as_nanos().saturating_sub(time::now().as_nanos());
        let mut bytes = [0u8; 16];
        bytes[..8].copy_from_slice(&(ns / NSEC_PER_SEC).to_le_bytes());
        bytes[8..].copy_from_slice(&(ns % NSEC_PER_SEC).to_le_bytes());
        if copyout(mm, address, &bytes).is_err() {
            restart = false;
        }
    }

    if restart {
        Outcome::Restart(Restart::NoHandler { request })
    } else {
        Outcome::Return(
            result
                .map(|count| count as isize)
                .unwrap_or_else(Errno::result),
        )
    }
}

/// 从用户态读取相对 `timespec` 超时参数并转换为单调时钟绝对截止时间 [`Timeout`]。
///
/// 若用户传入空指针（`address == 0`），表示无限期等待；若传入时长为 0，则不记录 `remaining` 写回地址。
fn relative_timeout(mm: &Mm, address: usize) -> Result<Timeout, Errno> {
    if address == 0 {
        return Ok(Timeout {
            deadline: None,
            remaining: None,
        });
    }

    let mut bytes = [0u8; 16];
    copyin(mm, &mut bytes, address)?;
    let ns = time_abi::decode_time(&bytes, NSEC_PER_SEC, 1)?;
    Ok(Timeout {
        deadline: Some(absolute_after(ns)?),
        remaining: if ns == 0 { None } else { Some(address) },
    })
}

/// 将相对纳秒时长 `ns` 与当前单调时钟相加，计算绝对截止时间；若发生算术溢出则返回 [`Errno::Overflow`]。
fn absolute_after(ns: u64) -> Result<MonotonicInstant, Errno> {
    time::now()
        .checked_add(Duration::from_nanos(ns))
        .ok_or(Errno::Overflow)
}

/// 将寄存器中的原始文件描述符参数按有符号 32 位整数（`i32`）解码，拒绝负数以防转换为巨大的无符号表下标。
fn descriptor(raw: usize) -> Result<usize, Errno> {
    let fd = raw as i32;
    if fd < 0 {
        Err(Errno::BadFd)
    } else {
        Ok(fd as usize)
    }
}

/// 检查 `pselect6` 位图切片 `set` 中第 `fd` 位是否被置位。
fn selected(set: &[u8], fd: usize) -> bool {
    set[fd / 8] & (1 << (fd % 8)) != 0
}

/// 从已完整拷入内核的 8 字节切片中按小端序解码 64 位无符号整数，避免直接解引用可能未对齐的用户态指针。
fn word(bytes: &[u8]) -> u64 {
    u64::from_le_bytes(bytes.try_into().expect("poll ABI word width"))
}
