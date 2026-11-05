//! 用户映射请求按目标ABI解释一次，再交同一Mm的实际范围与页拥有提交。

use crate::fs::{O_ACCMODE, O_RDONLY, O_RDWR, O_WRONLY};
use crate::mm::space::{MmapAddress, MmapRequest, MmapSource};
use crate::mm::user_table::PagePermissions;
use crate::mm::PAGE_SIZE;
use crate::syscall::error::{Errno, SysResult};
use crate::task::UserThread;

const SHARED: usize = 1;
const PRIVATE: usize = 2;
const MAP_SHARING_MASK: usize = SHARED | PRIVATE;
const FIXED: usize = 0x10;
const ANONYMOUS: usize = 0x20;
const DENYWRITE: usize = 0x800;
const EXECUTABLE: usize = 0x1000;
const STACK: usize = 0x2_0000;
const FIXED_NOREPLACE: usize = 0x10_0000;
const SUPPORTED: usize =
    SHARED | PRIVATE | FIXED | ANONYMOUS | DENYWRITE | EXECUTABLE | STACK | FIXED_NOREPLACE;

const MS_ASYNC: usize = 1;
const MS_INVALIDATE: usize = 2;
const MS_SYNC: usize = 4;

const PROT_READ: usize = 1;
const PROT_WRITE: usize = 2;
const PROT_EXEC: usize = 4;
const ALL: PagePermissions = PagePermissions {
    read: true,
    write: true,
    execute: true,
};

/// 共享最大 W 来自真实 RW OFD 与挂载写能力；私有文件写仍只授权匿名 COW。
pub(super) fn mmap(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let [address, length, prot, flags, fd, offset] = raw;
    // 栈提示不请求自动增长；历史 DENYWRITE/EXECUTABLE 不改变权限或原执行写占用。
    if flags & !SUPPORTED != 0
        || !matches!(flags & MAP_SHARING_MASK, SHARED | PRIVATE)
        || offset & (PAGE_SIZE - 1) != 0
    {
        return Err(Errno::InvalidArgument);
    }

    let permissions = permissions(prot)?;
    let private = flags & MAP_SHARING_MASK == PRIVATE;
    let mut maximum = ALL;
    let source = if flags & ANONYMOUS != 0 {
        if private {
            MmapSource::PrivateAnonymous
        } else {
            MmapSource::SharedAnonymous
        }
    } else {
        let runtime = user.runtime().expect("mmap has no runtime");
        let fd = fd as i32;
        if fd < 0 {
            return Err(Errno::BadFd);
        }
        let file = runtime.files.lookup(fd as usize)?;
        let access = file.flags() & O_ACCMODE;
        if access == O_WRONLY || (!private && permissions.write && access == O_RDONLY) {
            return Err(Errno::Permission);
        }
        let inode = file.mapping_inode().ok_or(Errno::NoDevice)?.clone();
        if !private {
            let path = file.path().ok_or(Errno::NoDevice)?;
            let writable = path.check_writable();
            maximum.write = access == O_RDWR && writable.is_ok();
            if permissions.write {
                writable?;
            }
        }
        MmapSource::File {
            file,
            inode,
            offset: offset as u64,
            private,
        }
    };

    let address = if flags & FIXED_NOREPLACE != 0 {
        MmapAddress::FixedNoReplace(address)
    } else if flags & FIXED != 0 {
        MmapAddress::Fixed(address)
    } else {
        MmapAddress::Hint(address)
    };

    let mm = user.mm().expect("mmap has no MM");
    mm.mmap(MmapRequest {
        address,
        length,
        permissions,
        maximum,
        source,
    })
    .map(|address| address as isize)
    .map_err(Errno::from)
}

/// 用户长度向上取基本页，起址和有效范围仍由同一Mm检查，空洞解除正常成功。
pub(super) fn munmap(user: &UserThread, raw: [usize; 6]) -> SysResult {
    if raw[1] == 0 {
        return Err(Errno::InvalidArgument);
    }
    let length = rounded_length(raw[1])?;
    user.mm().expect("munmap has no MM").unmap(raw[0], length)?;
    Ok(0)
}

/// 零长度仍先检查起址页齐；非空请求保当前COW与VMA最大权限。
pub(super) fn mprotect(user: &UserThread, raw: [usize; 6]) -> SysResult {
    if raw[0] & (PAGE_SIZE - 1) != 0 {
        return Err(Errno::InvalidArgument);
    }
    if raw[1] == 0 {
        return Ok(0);
    }

    let permissions = permissions(raw[2])?;
    let length = rounded_length(raw[1])?;
    user.mm()
        .expect("mprotect has no MM")
        .protect(raw[0], length, permissions)?;
    Ok(0)
}

/// 按 unsigned Linux ABI 解释页范围；零 flags 合法，异步和失效本身不发起写回或清除 PTE。
pub(super) fn msync(user: &UserThread, raw: [usize; 6]) -> SysResult {
    // flags 是目标 ABI 的 int，地址与长度为 unsigned long/size_t。
    let flags = raw[2] as u32 as usize;
    if flags & !(MS_ASYNC | MS_INVALIDATE | MS_SYNC) != 0
        || raw[0] & (PAGE_SIZE - 1) != 0
        || flags & (MS_ASYNC | MS_SYNC) == (MS_ASYNC | MS_SYNC)
    {
        return Err(Errno::InvalidArgument);
    }
    // size_t 的页进位按 unsigned 算术回绕，只有最终地址相加溢出返回 ENOMEM。
    let length = raw[1].wrapping_add(PAGE_SIZE - 1) & !(PAGE_SIZE - 1);
    let end = raw[0].checked_add(length).ok_or(Errno::NoMemory)?;
    if end == raw[0] {
        return Ok(0);
    }

    user.mm().expect("msync has no MM").sync_mapping_range(
        raw[0]..end,
        flags & MS_SYNC != 0,
        flags & MS_INVALIDATE != 0,
    )?;
    Ok(0)
}

/// raw brk的失败仍是原正break，不经过普通负errno转换。
pub(super) fn brk(user: &UserThread, address: usize) -> isize {
    user.mm().expect("brk has no MM").brk(address) as isize
}

/// 写权限在RV硬件上必然包含读，LA保持实际NR/NX能力；未知PROT位明确失败。
fn permissions(raw: usize) -> Result<PagePermissions, Errno> {
    if raw & !(PROT_READ | PROT_WRITE | PROT_EXEC) != 0 {
        return Err(Errno::InvalidArgument);
    }

    let write = raw & PROT_WRITE != 0;
    #[cfg(target_arch = "riscv64")]
    let read = raw & PROT_READ != 0 || write;
    #[cfg(target_arch = "loongarch64")]
    let read = raw & PROT_READ != 0;
    Ok(PagePermissions {
        read,
        write,
        execute: raw & PROT_EXEC != 0,
    })
}

/// 长度进位只在用户粒度转换时计算，随后严格MM接口不再解释raw长度。
fn rounded_length(length: usize) -> Result<usize, Errno> {
    length
        .checked_add(PAGE_SIZE - 1)
        .map(|length| length & !(PAGE_SIZE - 1))
        .ok_or(Errno::NoMemory)
}
