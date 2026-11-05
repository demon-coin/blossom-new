//! poll/select 和事件对象共用原文件表、持续源与逐页 uaccess，用户结构仅按目标字节布局解码。

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

#[derive(Clone, Copy)]
struct Timeout {
    deadline: Option<MonotonicInstant>,
    remaining: Option<usize>,
}

struct SelectSets {
    addresses: [usize; 3],
    input: [Vec<u8>; 3],
    output: [Vec<u8>; 3],
    nfds: usize,
    span: usize,
}

enum Interests<'a> {
    Poll(&'a mut [u8]),
    Select(&'a mut SelectSets),
}

/// 先同步析构 table 的源登记，再释放登记时取得的真实 OFD。
struct RegisteredPoll {
    context: Arc<PollContext>,
    _table: PollTable,
    files: Vec<(usize, Option<FileRef>)>,
}

/// 五个 raw 入口只保存本次参数；临时 mask 的收尾由各自 syscall 返回协议决定。
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

/// CLOEXEC 归描述符槽；完整事件 OFD 准备成功后才发表原预留槽。
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

/// ADD/MOD 先完整读取目标事件，DEL 不访问 event；对象能力检查遵守目标文件优先顺序。
fn control(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let operation = raw[1] as u32;
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

/// ppoll 输入数组只复制一次；重复 fd 逐项计数，输出只写 revents，不改原 fd/events。
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

/// nfds 受本次组 NOFILE 约束，小数组使用栈快照，大数组按实际跨度准备。
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

    let result = wait_fds(user, &runtime.files, &mut Interests::Poll(bytes), deadline);
    // 中断也交付本轮 revents；数组交付错误优先于等待结果，timeout 收尾另行处理。
    for (index, entry) in bytes.chunks_exact(POLLFD_SIZE).enumerate() {
        let output = address
            .checked_add(index * POLLFD_SIZE + 6)
            .ok_or(Errno::BadAddress)?;
        copyout(mm, output, &entry[6..8])?;
    }
    result
}

/// 第六参数包先完整读取；mask 指针为空时 size 不参与校验。
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

/// signed n 拒负数后只取一次 max_fds 快照，六份位图按该范围准备。
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

    for index in 0..3 {
        if sets.addresses[index] != 0 {
            copyin(
                mm,
                &mut sets.input[index][..sets.span],
                sets.addresses[index],
            )?;
        }
    }

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
    /// 只遍历本次快照的数组项或实际 n 内编号，不读取用户集合的其余空间。
    fn len(&self) -> usize {
        match self {
            Self::Poll(bytes) => bytes.len() / POLLFD_SIZE,
            Self::Select(sets) => sets.nfds,
        }
    }

    /// 负 poll fd 不参加扫描；select 按三个输入集合的并集选择一次 provider。
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

    /// 每轮输出清零，输入快照保留；旧就绪结果不跨下一轮状态查询。
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

    /// poll 按项保留隐含错误位；select 按三个集合分别设置结果并计数。
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

/// 无登记的首轮快速观察；后续扫描只使用登记时保留的原 OFD。
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

/// 无持续 provider 的普通文件使用固定默认位，不创建不存在的通知源。
fn query(file: &FileRef) -> Result<PollMask, FsError> {
    match file.pollable() {
        Some(provider) => provider.poll(None),
        None => Ok(DEFAULT_POLL),
    }
}

impl RegisteredPoll {
    /// 确定需等待后按实际参与数预备源登记与OFD拥有，失败析构先撤登记再释放文件。
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
            // 先持有本次实际 OFD，失败析构仍先撤源登记再释放文件。
            registered.files.push((index, file));
            if let Some(file) = &registered.files.last().unwrap().1 {
                if let Some(provider) = file.pollable() {
                    provider.poll(Some(&mut registered._table))?;
                }
            }
        }

        Ok(registered)
    }

    /// 已登记项只查询原 OFD，整数关闭复用不会替换正在等待的对象。
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

/// 首次快扫描和零 timeout 不建立登记；每次 clear 都在扫描之前，期限始终沿首次值。
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

/// 有限毫秒只建立一次绝对期限，负 signed int 表示无限；EINTR 保 mask 至真实用户返回。
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

/// 输出总范围只检查整数与目标上界，不预 fault；中途页错误由 scan 保存已交付正前缀。
fn core_epoll_wait(
    user: &UserThread,
    mm: &Mm,
    raw: [usize; 6],
    deadline: Option<MonotonicInstant>,
) -> Result<usize, Errno> {
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

    let runtime = user.runtime().expect("epoll_pwait has no runtime");
    let owner = runtime.files.lookup(descriptor(raw[0])?)?;
    let epoll = owner.epoll().ok_or(Errno::InvalidArgument)?;
    let signals = user.signals().expect("epoll wait has no signals");
    let mut waiting: Option<(Arc<PollContext>, PollTable)> = None;

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

/// 事件的 ABI padding 全初始化，data 原样保留所有位，不将其解释为用户地址。
fn encode_event(event: EpollEvent) -> [u8; EVENT_SIZE] {
    let mut bytes = [0u8; EVENT_SIZE];
    bytes[..4].copy_from_slice(&event.events.to_le_bytes());
    bytes[8..].copy_from_slice(&event.data.to_le_bytes());
    bytes
}

/// 临时 mask 只完整读取原 8 字节；安装后的旧值由用户返回边界承担恢复责任。
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

/// ready/timeout/error 先恢复 mask；中断保留至 handler 选择，剩余时间单独交付不改普通结果。
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

/// 相对 timespec 一次转为绝对期限；零时长不回写剩余时间。
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

/// 内部时间域有限，超出绝对纳秒空间保原 Overflow，不延长或饱和成伪期限。
fn absolute_after(ns: u64) -> Result<MonotonicInstant, Errno> {
    time::now()
        .checked_add(Duration::from_nanos(ns))
        .ok_or(Errno::Overflow)
}

/// 文件号码按目标 signed int 解码，负数不形成巨大的表下标。
fn descriptor(raw: usize) -> Result<usize, Errno> {
    let fd = raw as i32;
    if fd < 0 {
        Err(Errno::BadFd)
    } else {
        Ok(fd as usize)
    }
}

/// 集合只查询 n 内已经初始化的字节，最后机器字的高位不参与扫描。
fn selected(set: &[u8], fd: usize) -> bool {
    set[fd / 8] & (1 << (fd % 8)) != 0
}

/// 从已完整复制的八字节快照读取 64 位字段，不依赖宿主结构对齐。
fn word(bytes: &[u8]) -> u64 {
    u64::from_le_bytes(bytes.try_into().expect("poll ABI word width"))
}
