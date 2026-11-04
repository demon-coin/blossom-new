//! 描述符槽按实际使用增长，额度由每次调用者提供，具体文件独立拥有路径和共享位置。

use alloc::boxed::Box;
use alloc::vec::Vec;
use core::mem;
use core::sync::atomic::{AtomicUsize, Ordering};

use crate::fs::file::{O_DIRECTORY, O_NOFOLLOW};
use crate::fs::{
    open_path_with_flags, DirectoryEntry, FileIoContext, FileRef, FileStat, FsContext, FsError,
    ResolveMode, SeekFrom, Vfs, WriteBuffer,
};
use crate::sync::SpinLock;

/// 描述符自己的属性，不随共享文件位置传播。
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct DescriptorFlags {
    pub close_on_exec: bool,
}

/// 只读打开的路径约束与新描述符属性。
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct OpenOptions {
    pub directory: bool,
    pub no_follow: bool,
    pub close_on_exec: bool,
}

/// 槽尚未发表时没有文件引用，只有取得预留责任的对象能完成或取消它。
enum Slot {
    Vacant,
    Reserved,
    Open {
        file: FileRef,
        flags: DescriptorFlags,
    },
}

const PAGE_SLOTS: usize = u64::BITS as usize;

/// 每页空闲位与槽在同一短锁内修改，页拥有只在目录增长时移动。
struct SlotPage {
    slots: [Slot; PAGE_SLOTS],
    free: u64,
}

struct Slots {
    pages: Vec<Box<SlotPage>>,
}

impl Slots {
    /// 已分配空间是 max_fds 快照，不代表任何线程组的限额。
    fn len(&self) -> usize {
        self.pages.len() * PAGE_SLOTS
    }

    /// fork 只为已经发表的最高号准备空间，旧高号关闭和 Reserved 不扩大子表。
    fn open_span(&self) -> usize {
        (0..self.len())
            .rev()
            .find(|fd| matches!(self.get(*fd), Some(Slot::Open { .. })))
            .map_or(0, |fd| fd + 1)
    }

    /// 借用已存在的槽，调用者不得使借用跨越表锁。
    fn get(&self, fd: usize) -> Option<&Slot> {
        self.pages
            .get(fd / PAGE_SLOTS)
            .map(|page| &page.slots[fd % PAGE_SLOTS])
    }

    /// 在同一锁内修改槽及空闲位；移出的 OFD 由调用者锁外释放。
    fn replace(&mut self, fd: usize, slot: Slot) -> Slot {
        let page = &mut self.pages[fd / PAGE_SLOTS];
        let bit = 1u64 << (fd % PAGE_SLOTS);
        if matches!(slot, Slot::Vacant) {
            page.free |= bit;
        } else {
            page.free &= !bit;
        }
        mem::replace(&mut page.slots[fd % PAGE_SLOTS], slot)
    }

    /// 位图选择不小于最低号且严格小于本次额度的最小空号。
    fn vacant(&self, minimum: usize, limit: usize) -> Option<usize> {
        if limit <= minimum {
            return None;
        }
        for index in minimum / PAGE_SLOTS..self.pages.len().min(limit.div_ceil(PAGE_SLOTS)) {
            let base = index * PAGE_SLOTS;
            let mut free = self.pages[index].free;
            if base < minimum {
                free &= u64::MAX << (minimum - base);
            }
            if limit - base < PAGE_SLOTS {
                free &= (1u64 << (limit - base)) - 1;
            }
            if free != 0 {
                return Some(base + free.trailing_zeros() as usize);
            }
        }
        None
    }
}

/// 槽页、空闲位图和目录一起发表；额度不保存在可跨线程组共享的表内。
pub struct FileTable {
    slots: SpinLock<Slots>,
    users: AtomicUsize,
}

/// 借用原表的单次预留责任；移动后只能发表一次，未发表的析构释放原槽。
pub struct FileReservation<'a> {
    table: &'a FileTable,
    fd: usize,
    active: bool,
}

impl FileTable {
    /// 锁外建立一个小槽页，不预先分配用户额度对应的空间。
    pub fn try_new() -> Result<Self, FsError> {
        let table = Self {
            slots: SpinLock::new(Slots { pages: Vec::new() }),
            users: AtomicUsize::new(0),
        };
        table.grow(PAGE_SLOTS)?;
        Ok(table)
    }

    /// 锁外准备全部新页及替代目录，重验后只移动已有页的拥有。
    fn grow(&self, required: usize) -> Result<(), FsError> {
        let needed = required.div_ceil(PAGE_SLOTS);
        loop {
            let (current, capacity) = {
                let slots = self.slots.lock();
                (slots.pages.len(), slots.pages.capacity())
            };
            if needed <= current {
                return Ok(());
            }
            let mut directory = Vec::new();
            if capacity < needed {
                directory
                    .try_reserve_exact(needed.max(capacity.saturating_mul(2)))
                    .map_err(|_| FsError::NoMemory)?;
            }
            let mut added = Vec::new();
            added
                .try_reserve_exact(needed - current)
                .map_err(|_| FsError::NoMemory)?;
            for _ in current..needed {
                added.push(
                    Box::try_new(SlotPage {
                        slots: [const { Slot::Vacant }; PAGE_SLOTS],
                        free: u64::MAX,
                    })
                    .map_err(|_| FsError::NoMemory)?,
                );
            }
            {
                let mut slots = self.slots.lock();
                if slots.pages.len() != current {
                    continue;
                }
                if capacity < needed {
                    directory.extend(slots.pages.drain(..));
                    mem::swap(&mut slots.pages, &mut directory);
                }
                slots.pages.append(&mut added);
            }
            return Ok(());
        }
    }

    /// 复制最终锁内的 Open 集合；并发增长时重新准备，Reserved 在子表保持空闲。
    pub fn try_clone(&self) -> Result<Self, FsError> {
        let child = Self::try_new()?;
        loop {
            let required = self.slots.lock().open_span();
            child.grow(required)?;
            let mut target = child.slots.lock();
            let source = self.slots.lock();
            let highest = source.open_span();
            if target.len() < highest {
                continue;
            }
            for fd in 0..highest {
                if let Some(Slot::Open { file, flags }) = source.get(fd) {
                    target.replace(
                        fd,
                        Slot::Open {
                            file: file.clone(),
                            flags: *flags,
                        },
                    );
                }
            }
            break;
        }
        Ok(child)
    }

    /// 增加真实运行绑定责任，普通表引用与诊断拥有不参与计数。
    pub(crate) fn add_user(&self) {
        self.users
            .fetch_update(Ordering::AcqRel, Ordering::Acquire, |count| {
                count.checked_add(1)
            })
            .expect("file table user count overflow");
    }

    /// 归还真实运行绑定，表与文件的最后引用由调用者在短锁外释放。
    pub(crate) fn remove_user(&self) {
        let previous = self.users.fetch_sub(1, Ordering::AcqRel);
        assert!(previous != 0, "file table user count underflow");
    }

    /// 返回真实运行绑定数，不将诊断引用误判为共享描述符表的执行者。
    pub(crate) fn user_count(&self) -> usize {
        self.users.load(Ordering::Acquire)
    }

    /// 返回本次已分配槽空间，供 select 按一次 max_fds 快照裁剪输入。
    pub fn allocated_slots(&self) -> usize {
        self.slots.lock().len()
    }

    /// 以本次调用者额度选择最小空槽，必要元数据均在短锁外准备。
    pub fn reserve(&self, limit: usize) -> Result<FileReservation<'_>, FsError> {
        loop {
            let required = {
                let mut slots = self.slots.lock();
                if let Some(fd) = slots.vacant(0, limit) {
                    slots.replace(fd, Slot::Reserved);
                    return Ok(FileReservation {
                        table: self,
                        fd,
                        active: true,
                    });
                }
                if limit <= slots.len() {
                    return Err(FsError::TooManyFiles);
                }
                (slots.len() + PAGE_SLOTS).min(limit)
            };
            self.grow(required)?;
        }
    }

    /// 短锁内取得已发表文件的强引用；调用者随后不重查可复用的整数编号。
    pub fn lookup(&self, fd: usize) -> Result<FileRef, FsError> {
        let slots = self.slots.lock();
        match slots.get(fd) {
            Some(Slot::Open { file, .. }) => Ok(file.clone()),
            _ => Err(FsError::BadFd),
        }
    }

    /// 读取当前描述符的属性，未发表的预留与空槽均不作为打开文件。
    pub fn flags(&self, fd: usize) -> Result<DescriptorFlags, FsError> {
        let slots = self.slots.lock();
        match slots.get(fd) {
            Some(Slot::Open { flags, .. }) => Ok(*flags),
            _ => Err(FsError::BadFd),
        }
    }

    /// 同一次槽锁中替换 descriptor 属性，不改变其共享 OFD。
    pub fn set_flags(&self, fd: usize, replacement: DescriptorFlags) -> Result<(), FsError> {
        let mut slots = self.slots.lock();
        match slots
            .pages
            .get_mut(fd / PAGE_SLOTS)
            .map(|page| &mut page.slots[fd % PAGE_SLOTS])
        {
            Some(Slot::Open { flags, .. }) => {
                *flags = replacement;
                Ok(())
            }
            _ => Err(FsError::BadFd),
        }
    }

    /// 普通可睡眠 Task 先预留，再解析真实路径并构造完整文件；任何失败由预留析构取消。
    pub fn open(
        &self,
        vfs: &Vfs,
        context: &FsContext,
        pathname: &[u8],
        options: OpenOptions,
        limit: usize,
    ) -> Result<usize, FsError> {
        let reservation = self.reserve(limit)?;
        let mode = if options.no_follow {
            ResolveMode::NoFollow
        } else {
            ResolveMode::Follow
        };
        let path = vfs.resolve(context, pathname, mode)?;
        let flags = if options.directory { O_DIRECTORY } else { 0 }
            | if options.no_follow { O_NOFOLLOW } else { 0 };
        let file = open_path_with_flags(vfs, path, flags, FileIoContext::Kernel)?;
        Ok(reservation.publish(
            file,
            DescriptorFlags {
                close_on_exec: options.close_on_exec,
            },
        ))
    }

    /// 在表锁外读取原文件并推进其共享位置，关闭或复用 fd 不改变本次文件身份。
    pub fn read(&self, fd: usize, buffer: &mut [u8]) -> Result<usize, FsError> {
        let file = self.lookup(fd)?;
        file.read(buffer)
    }

    /// 在表锁外按 byte offset 读取原文件，不修改其共享位置。
    pub fn read_at(&self, fd: usize, offset: u64, buffer: &mut [u8]) -> Result<usize, FsError> {
        let file = self.lookup(fd)?;
        file.read_at(offset, buffer)
    }

    /// 在表锁外修改具体文件的共享位置，保留它的边界和类型错误。
    pub fn seek(&self, fd: usize, from: SeekFrom) -> Result<u64, FsError> {
        let file = self.lookup(fd)?;
        file.seek(from)
    }

    /// 在表锁外读取真实 inode 元数据，不占用文件位置锁。
    pub fn stat(&self, fd: usize) -> Result<FileStat, FsError> {
        let file = self.lookup(fd)?;
        file.stat()
    }

    /// 在表锁外消费目录的下一项，共享枚举位置由具体目录文件保护。
    pub fn readdir(&self, fd: usize) -> Result<Option<DirectoryEntry>, FsError> {
        let file = self.lookup(fd)?;
        file.readdir()
    }

    /// 内核顺序写只绑定输入，不在描述符表内复制 inode 写循环。
    pub fn write(&self, fd: usize, buffer: &[u8]) -> Result<usize, FsError> {
        self.write_from(fd, &mut WriteBuffer::kernel(buffer))
    }

    /// 内核按位置写经过同一核心，不取得 OFD 共享位置锁。
    pub fn write_at(&self, fd: usize, offset: u64, buffer: &[u8]) -> Result<usize, FsError> {
        self.write_at_from(fd, offset, &mut WriteBuffer::kernel(buffer))
    }

    /// 表短锁外持原 OFD 执行顺序写，关闭复用 fd 不改变本次文件身份。
    pub fn write_from(&self, fd: usize, source: &mut WriteBuffer<'_>) -> Result<usize, FsError> {
        self.lookup(fd)?.write_from(source, FileIoContext::Kernel)
    }

    /// 表短锁外按明确位置写入，实际追加和错误均由原 OFD 提供。
    pub fn write_at_from(
        &self,
        fd: usize,
        offset: u64,
        source: &mut WriteBuffer<'_>,
    ) -> Result<usize, FsError> {
        self.lookup(fd)?
            .write_at_from(offset, source, FileIoContext::Kernel)
    }

    /// 先移出打开槽并使编号立即可复用，解锁后才释放表拥有的文件引用。
    pub fn close(&self, fd: usize) -> Result<(), FsError> {
        let old = {
            let mut slots = self.slots.lock();
            if !matches!(slots.get(fd), Some(Slot::Open { .. })) {
                return Err(FsError::BadFd);
            }
            slots.replace(fd, Slot::Vacant)
        };
        drop(old);
        Ok(())
    }

    /// exec 已独占或私有化此表，逐槽撤 CLOEXEC 并锁外释放最后拥有。
    pub fn close_on_exec(&self) {
        for fd in 0..self.allocated_slots() {
            let old = {
                let mut slots = self.slots.lock();
                if matches!(slots.get(fd), Some(Slot::Open { flags, .. }) if flags.close_on_exec) {
                    Some(slots.replace(fd, Slot::Vacant))
                } else {
                    None
                }
            };
            drop(old);
        }
    }

    /// 最小空号复制清除新描述符 CLOEXEC，仍共享原 OFD。
    pub fn dup(&self, fd: usize, limit: usize) -> Result<usize, FsError> {
        self.dup_min(fd, 0, DescriptorFlags::default(), limit)
    }

    /// 每次重取锁均重验源 Open 与当前空号，扩容失败不留下预留。
    pub fn dup_min(
        &self,
        fd: usize,
        min: usize,
        flags: DescriptorFlags,
        limit: usize,
    ) -> Result<usize, FsError> {
        loop {
            let required = {
                let mut slots = self.slots.lock();
                let Some(Slot::Open { file, .. }) = slots.get(fd) else {
                    return Err(FsError::BadFd);
                };
                if limit <= min {
                    return Err(FsError::InvalidOperation);
                }
                if let Some(target) = slots.vacant(min, limit) {
                    let file = file.clone();
                    slots.replace(target, Slot::Open { file, flags });
                    return Ok(target);
                }
                if limit <= slots.len() {
                    return Err(FsError::TooManyFiles);
                }
                (slots.len() + PAGE_SLOTS).max(min + 1).min(limit)
            };
            self.grow(required)?;
        }
    }

    /// 精确复制允许同号保留原属性，其余复制清除 CLOEXEC。
    pub fn dup_exact(&self, source: usize, target: usize, limit: usize) -> Result<usize, FsError> {
        self.replace_duplicate(source, target, DescriptorFlags::default(), true, limit)
    }

    /// dup3 同号无效，目标 Reserved 不可由其他责任替换。
    pub fn dup_exact_flags(
        &self,
        source: usize,
        target: usize,
        flags: DescriptorFlags,
        limit: usize,
    ) -> Result<usize, FsError> {
        self.replace_duplicate(source, target, flags, false, limit)
    }

    /// 扩容之后重验源及目标，原子替换并在表锁外释放旧 OFD。
    fn replace_duplicate(
        &self,
        source: usize,
        target: usize,
        flags: DescriptorFlags,
        allow_same: bool,
        limit: usize,
    ) -> Result<usize, FsError> {
        if !allow_same && source == target {
            return Err(FsError::InvalidOperation);
        }
        loop {
            let old = {
                let mut slots = self.slots.lock();
                let Some(Slot::Open { file, .. }) = slots.get(source) else {
                    return Err(FsError::BadFd);
                };
                if source == target && allow_same {
                    return Ok(target);
                }
                if limit <= target {
                    return Err(FsError::BadFd);
                }
                if slots.len() <= target {
                    drop(slots);
                    self.grow(target + 1)?;
                    continue;
                }
                if matches!(slots.get(target), Some(Slot::Reserved)) {
                    return Err(FsError::Busy);
                }
                let replacement = Slot::Open {
                    file: file.clone(),
                    flags,
                };
                slots.replace(target, replacement)
            };
            drop(old);
            return Ok(target);
        }
    }
}

impl FileReservation<'_> {
    /// 返回自己保留的整数编号，该槽在发表前仍不能查找或关闭。
    pub fn fd(&self) -> usize {
        self.fd
    }

    /// 消费唯一预留责任并移动完整文件入槽；原槽和位图均已准备。
    pub fn publish(mut self, file: FileRef, flags: DescriptorFlags) -> usize {
        {
            let mut slots = self.table.slots.lock();
            assert!(matches!(slots.get(self.fd), Some(Slot::Reserved)));
            slots.replace(self.fd, Slot::Open { file, flags });
            self.active = false;
        }
        self.fd
    }
}

impl Drop for FileReservation<'_> {
    /// 只取消自己尚未发表的原槽；已发表文件的引用由描述符表独立拥有。
    fn drop(&mut self) {
        if self.active {
            let mut slots = self.table.slots.lock();
            assert!(matches!(slots.get(self.fd), Some(Slot::Reserved)));
            slots.replace(self.fd, Slot::Vacant);
        }
    }
}
