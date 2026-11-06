//! 用户态虚拟内存管理系统调用实现（`mmap` / `munmap` / `mprotect` / `msync` / `brk`）。
//!
//! # 模块职责与架构定位
//! 本模块位于系统调用分发层与虚拟内存管理子系统（`crate::mm::space::Mm`）之间：
//! - **上游调用者**：`crate::syscall` 总分发入口，传入当前用户线程 [`UserThread`] 与原始寄存器参数。
//! - **下游依赖**：进程文件描述符表（用于文件映射的 `fd` 查找与打开模式校验）、虚拟文件系统路径与 Inode（用于检查挂载写权限及获取可映射 Inode）、以及进程地址空间 [`Mm`]（执行实际的虚拟内存区域 VMA 分配、拆分、页表项更新与脏页写回）。
//!
//! # 核心机制与 ABI 转换契约
//! ```text
//! 用户态寄存器参数 (addr, len, prot, flags, fd, offset)
//!       │
//!       ├─ 1. 标志位与页对齐校验 (MAP_SHARED vs MAP_PRIVATE, offset 页对齐)
//!       ├─ 2. 保护位转换 (PROT_READ / PROT_WRITE / PROT_EXEC -> PagePermissions)
//!       │     └─ RISC-V 64 硬件要求可写页必须可读，故 PROT_WRITE 自动开启读权限；
//!       │        LoongArch 64 支持独立的不可读/不可执行位，保持原始请求权限。
//!       ├─ 3. 映射源与最大权限 (maximum) 推导
//!       │     ├─ 匿名映射：最大权限为 R/W/X 全开。
//!       │     ├─ 私有文件映射 (COW)：要求文件不可为只写 (O_WRONLY)；写操作触发写时复制，
//!       │     │                      不会修改底层文件，故最大权限允许写入。
//!       │     └─ 共享文件映射：要求底层文件与挂载点具备相应写能力；若文件以 O_RDONLY 打开，
//!       │                      则禁止映射为 PROT_WRITE，且 VMA 最大权限关闭写权限，
//!       │                      防止后续通过 mprotect 提权写入只读文件描述符。
//!       └─ 4. 提交至 Mm 执行 VMA 树与硬件页表更新
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 在 `mmap` 处理文件映射时，先短暂获取进程文件表锁（`runtime.files.lookup`）克隆打开文件对象（`Arc`），释放文件表锁后再检查挂载点写权限，最后进入 `Mm` 内部获取地址空间锁。
//! - 严禁在持有 `Mm` 地址空间锁期间回调文件表锁，以避免死锁。

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

/// 处理 `mmap` 系统调用：在当前进程地址空间中创建匿名映射或文件映射。
///
/// # 调用上下文与参数
/// - 运行于普通可睡眠的用户任务上下文。
/// - `raw`: `[address, length, prot, flags, fd, offset]` 原始系统调用参数数组。
///
/// # 权限与返回值契约
/// - 共享文件映射（`MAP_SHARED`）的最大可写权限取决于打开文件描述符是否具备读写模式（`O_RDWR`）以及所在挂载点是否可写。
/// - 私有文件映射（`MAP_PRIVATE`）的写入仅在匿名写时复制（COW）页上生效，因此只需文件可读即可授权私有写权限。
/// - 成功时返回映射区域的起始用户虚拟地址，失败返回对应的负向 [`Errno`]。
pub(super) fn mmap(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let [address, length, prot, flags, fd, offset] = raw;
    // Step 1: 校验映射标志位合法性（必须且只能指定 MAP_SHARED 或 MAP_PRIVATE 之一）以及文件偏移量的页对齐。
    // MAP_STACK 仅作为用户栈提示，不触发自动向下扩展；MAP_DENYWRITE 与 MAP_EXECUTABLE 按 Linux ABI 兼容忽略。
    if flags & !SUPPORTED != 0
        || !matches!(flags & MAP_SHARING_MASK, SHARED | PRIVATE)
        || offset & (PAGE_SIZE - 1) != 0
    {
        return Err(Errno::InvalidArgument);
    }

    // Step 2: 解析初始页保护权限，并根据是否包含 MAP_ANONYMOUS 构造映射源与最大可升级权限（maximum）。
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

    // Step 3: 根据标志位区分固定地址不覆盖（MAP_FIXED_NOREPLACE）、强制覆盖（MAP_FIXED）与提示地址（Hint）。
    let address = if flags & FIXED_NOREPLACE != 0 {
        MmapAddress::FixedNoReplace(address)
    } else if flags & FIXED != 0 {
        MmapAddress::Fixed(address)
    } else {
        MmapAddress::Hint(address)
    };

    // Step 4: 将校验完备的映射请求提交至当前进程的地址空间管理器 Mm。
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

/// 处理 `munmap` 系统调用：解除指定用户虚拟地址范围内的内存映射。
///
/// # 调用上下文与参数
/// - 运行于普通可睡眠的用户任务上下文。
/// - `raw[0]`: 待解除映射的起始虚拟地址（由底层 `Mm::unmap` 检查页对齐与用户地址范围）。
/// - `raw[1]`: 解除映射的字节长度（非零，且向上按页大小对齐）。
///
/// # 返回值契约
/// - 若指定范围内包含未映射的虚拟地址空洞，仍正常解除已有映射并返回 `Ok(0)`。
pub(super) fn munmap(user: &UserThread, raw: [usize; 6]) -> SysResult {
    if raw[1] == 0 {
        return Err(Errno::InvalidArgument);
    }
    let length = rounded_length(raw[1])?;
    user.mm().expect("munmap has no MM").unmap(raw[0], length)?;
    Ok(0)
}

/// 处理 `mprotect` 系统调用：修改指定用户虚拟地址范围内已映射页面的访问保护权限。
///
/// # 调用上下文与参数
/// - 运行于普通可睡眠的用户任务上下文。
/// - `raw[0]`: 起始虚拟地址（必须按 `PAGE_SIZE` 对齐，即使长度为 0 也会先行校验）。
/// - `raw[1]`: 字节长度（为 0 时直接成功返回；非零时向上按页对齐）。
/// - `raw[2]`: 新的保护标志位（`PROT_READ | PROT_WRITE | PROT_EXEC`）。
///
/// # 权限与返回值契约
/// - 目标权限不得超过对应虚拟内存区域（VMA）创建时记录的最大允许权限（`maximum`），否则返回 `EACCES`。
/// - 成功返回 `Ok(0)`。
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

/// 处理 `msync` 系统调用：将文件映射区域中的脏页同步写回底层文件系统或块设备。
///
/// # 调用上下文与参数
/// - 运行于普通可睡眠的用户任务上下文。
/// - `raw[0]`: 起始虚拟地址（必须按 `PAGE_SIZE` 对齐）。
/// - `raw[1]`: 同步字节长度（按无符号算术向上页对齐，若与起始地址相加溢出则返回 `ENOMEM`）。
/// - `raw[2]`: 同步标志位（`MS_ASYNC`、`MS_SYNC`、`MS_INVALIDATE`；`MS_ASYNC` 与 `MS_SYNC` 不可同时指定）。
///
/// # 返回值契约
/// - 仅当指定 `MS_SYNC` 时才阻塞等待共享文件映射的脏页写回完成；成功返回 `Ok(0)`。
pub(super) fn msync(user: &UserThread, raw: [usize; 6]) -> SysResult {
    // Step 1: 校验 flags 合法性、起始地址页对齐，以及 MS_ASYNC 与 MS_SYNC 的互斥关系。
    let flags = raw[2] as u32 as usize;
    if flags & !(MS_ASYNC | MS_INVALIDATE | MS_SYNC) != 0
        || raw[0] & (PAGE_SIZE - 1) != 0
        || flags & (MS_ASYNC | MS_SYNC) == (MS_ASYNC | MS_SYNC)
    {
        return Err(Errno::InvalidArgument);
    }
    // Step 2: 按无符号整数将长度向上取整到页边界，并检查 [addr, addr + length) 是否算术溢出。
    let length = raw[1].wrapping_add(PAGE_SIZE - 1) & !(PAGE_SIZE - 1);
    let end = raw[0].checked_add(length).ok_or(Errno::NoMemory)?;
    if end == raw[0] {
        return Ok(0);
    }

    // Step 3: 委托地址空间管理器校验 VMA 覆盖情况，并在 MS_SYNC 置位时触发文件脏页写回。
    user.mm().expect("msync has no MM").sync_mapping_range(
        raw[0]..end,
        flags & MS_SYNC != 0,
        flags & MS_INVALIDATE != 0,
    )?;
    Ok(0)
}

/// 处理 `brk` 系统调用：查询或调整进程数据段（堆区）的结束地址（Program Break）。
///
/// # 调用上下文与参数
/// - 运行于普通可睡眠的用户任务上下文。
/// - `address`: 请求设置的新堆顶虚拟地址（若为 0 或非法/越界地址，则保持原堆顶不变）。
///
/// # 返回值契约
/// - 遵循 Linux 原始 `brk` 系统调用约定：无论扩容/缩容成功与否，均直接返回当前实际的堆顶虚拟地址（正整数），不转换为负向错误码。
pub(super) fn brk(user: &UserThread, address: usize) -> isize {
    user.mm().expect("brk has no MM").brk(address) as isize
}

/// 将用户态传入的 `PROT_*` 标志位转换为内核统一的页表权限结构 [`PagePermissions`]。
///
/// # 架构差异说明
/// - 在 RISC-V 64 架构下，硬件页表不支持“仅可写不可读”（`W=1, R=0` 为保留编码），因此指定 `PROT_WRITE` 时会自动附带读权限。
/// - 在 LoongArch 64 架构下，硬件页表支持独立的不可读（NR）与不可执行（NX）控制位，因此严格保留用户请求的读写执行位组合。
/// - 若包含除 `PROT_READ | PROT_WRITE | PROT_EXEC` 以外的未知位，返回 [`Errno::InvalidArgument`]。
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

/// 将用户态传入的字节长度向上对齐至 [`PAGE_SIZE`] 的整数倍。
///
/// # 错误处理
/// - 若向上取整过程中发生 `usize` 加法溢出，返回 [`Errno::NoMemory`]（`ENOMEM`）。
fn rounded_length(length: usize) -> Result<usize, Errno> {
    length
        .checked_add(PAGE_SIZE - 1)
        .map(|length| length & !(PAGE_SIZE - 1))
        .ok_or(Errno::NoMemory)
}
