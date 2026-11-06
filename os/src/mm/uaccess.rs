//! 用户空间内存安全访问（`uaccess`）与跨页数据拷贝。
//!
//! # 模块职责与架构定位
//! 本模块作为虚拟内存管理器（[`Mm`]）的扩展实现，提供内核态读写用户态虚拟地址空间的统一安全通道：
//! - **上游调用者**：系统调用分发层、信号投递、ELF 加载器、Futex 同步原语及文件系统读写路径。
//! - **下游依赖**：`Mm::with_page`（驻留页查询与短期锁保护）、`Mm::fault`（缺页异常与写时复制处理）以及体系结构原语 `arch::copy_user_bytes` / `arch::load_user_u32` / `arch::store_user_u32`。
//!
//! # 核心设计与两阶段逐页访问状态机
//! 为避免在持有页表锁或禁止调度的状态下触发可睡眠的缺页 I/O，所有用户内存访问均按 4KiB 页边界切分，并遵循以下两阶段循环（参见 [`Mm::access_page`]）：
//! ```text
//!   用户缓冲区 [address .. address + len)
//!        |
//!        v  按页边界切分为单页片段 (page_chunk)
//!   +-------------------------------------------------------+
//!   | Phase 1: Mm::with_page(va, access)                    |
//!   | - 在短期映射保护内检查目标页是否已驻留且权限满足要求  |
//!   | - 若满足（且写操作已完成 COW 拆分）：                 |
//!   |   调用 arch::copy_user_bytes / 原子指令完成页内访问   |
//!   +-------------------------------------------------------+
//!        | 返回 None（缺页 / 权限不足 / 待拆分 COW）
//!        v
//!   +-------------------------------------------------------+
//!   | Phase 2: Mm::fault(va, access)                        |
//!   | - 已释放全部短期访问锁，允许分配物理页、触发回收或睡眠|
//!   | - 成功后回到 Phase 1 重新验证并完成拷贝               |
//!   +-------------------------------------------------------+
//! ```
//!
//! # 锁获取顺序与并发契约
//! - **禁止跨缺页持锁**：`with_page` 闭包仅在单页映射稳定期内执行原始内存拷贝或单次原子读写，闭包内严禁睡眠、动态内存分配（如 `Vec` 扩容）或将用户页指针泄露到闭包外部。
//! - **禁止构造用户内存 Rust 引用**：用户线程可能并发修改同一物理页，因此内核访问用户页只能通过 `arch::copy_user_bytes` 或原子指令按原始指针操作，不得将用户页转换为 `&[u8]` 或 `&mut [u8]`，从而避免违反 Rust 别名规则（Aliasing UB）。

use alloc::vec::Vec;
use core::mem::size_of;

use crate::arch::{self, UserAccess};
use crate::mm::space::{Mm, VmError};
use crate::mm::PAGE_SIZE;

/// 用户态与内核态之间字节拷贝的执行进度与首个错误记录。
#[must_use = "user copies may stop after a partial prefix"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CopyProgress {
    /// 遇到错误前已成功拷贝的连续字节数（单位：字节）。
    pub copied: usize,
    /// 导致拷贝提前中止的虚拟内存错误；全部字节拷贝成功时为 `None`。
    pub error: Option<VmError>,
}

/// 从用户空间读取以 NUL（`\0`）结尾的 C 字符串时可能发生的错误。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CStringError {
    /// 用户虚拟地址越界、未映射或缺页处理失败。
    Vm(VmError),
    /// 内核堆内存不足，无法扩容结果缓冲区。
    NoMemory,
    /// 在指定的最大字节数（`max_bytes`）范围内未找到 `\0` 终止符。
    Unterminated,
}

impl Mm {
    /// 从用户虚拟地址 `address` 逐页读取字节到内核缓冲区 `dst`（可在缺页处理时睡眠）。
    ///
    /// - 当 `dst` 为空切片时直接返回成功，不检查或解引用 `address`。
    /// - 若中途某页触发 `VmError`，函数立即停止并返回已成功写入 `dst` 前缀的字节数，`dst` 剩余尾部保持原值。
    pub fn copyin(&self, dst: &mut [u8], address: usize) -> CopyProgress {
        if let Err(error) = check_range(address, dst.len()) {
            return CopyProgress {
                copied: 0,
                error: Some(error),
            };
        }

        // 按页边界切分拷贝区间，每次只在单页映射保护内拷贝 `count` 字节。
        let mut copied = 0;
        while copied < dst.len() {
            let current = address + copied;
            let count = page_chunk(current, dst.len() - copied);
            let target = dst[copied..copied + count].as_mut_ptr();
            let result = self.access_page(current, UserAccess::Read, |source| {
                // SAFETY:
                // 1. `self.access_page` 保证当前物理页存活、已完成初始化且具备可读权限，`count` 不超过当前页剩余字节数；
                // 2. `target` 指向内核独占切片 `dst[copied..copied + count]`，与用户态物理页不重叠；
                // 3. `arch::copy_user_bytes` 使用原始指针指令拷贝，不构造指向用户内存的 Rust 引用，且 `source` 指针不会逃逸出闭包。
                unsafe { arch::copy_user_bytes(target, source as *const u8, count) };
            });
            if let Err(error) = result {
                return CopyProgress {
                    copied,
                    error: Some(error),
                };
            }
            copied += count;
        }
        CopyProgress {
            copied,
            error: None,
        }
    }

    /// 将内核缓冲区 `src` 的字节逐页写入用户虚拟地址 `address`（缺页分配与私有页 COW 拆分在短期访问锁外完成）。
    ///
    /// - 当 `src` 为空切片时直接返回成功，不检查或解引用 `address`。
    /// - 若中途某页发生错误，仅停止后续写入，已写入用户空间的前缀字节不会回滚。
    pub fn copyout(&self, address: usize, src: &[u8]) -> CopyProgress {
        if let Err(error) = check_range(address, src.len()) {
            return CopyProgress {
                copied: 0,
                error: Some(error),
            };
        }

        // 按页边界切分写入区间，逐页验证可写非 COW 映射并写出数据。
        let mut copied = 0;
        while copied < src.len() {
            let current = address + copied;
            let count = page_chunk(current, src.len() - copied);
            let source = src[copied..copied + count].as_ptr();
            let result = self.access_page(current, UserAccess::Write, |target| {
                // SAFETY:
                // 1. `self.access_page` 保证当前物理页存活且已完成写时复制（COW）拆分，具备独占或共享可写权限，`count` 不越出当前页；
                // 2. `source` 指向有效初始化的内核只读切片 `src[copied..copied + count]`，与用户态物理页不重叠；
                // 3. `arch::copy_user_bytes` 仅通过原始指令写入用户页，不构造用户内存的 Rust 引用，且 `target` 指针不会逃逸出闭包。
                unsafe { arch::copy_user_bytes(target as *mut u8, source, count) };
            });
            if let Err(error) = result {
                return CopyProgress {
                    copied,
                    error: Some(error),
                };
            }
            copied += count;
        }
        CopyProgress {
            copied,
            error: None,
        }
    }

    /// 从用户虚拟地址 `address` 完整读取一个小端序 `u32` 值（支持跨页边界与非自然对齐地址）。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(dead_code, reason = "用户访问测试核对跨页小端 u32 读取及真实缺页错误")
    )]
    pub fn read_u32(&self, address: usize) -> Result<u32, VmError> {
        let mut bytes = [0u8; size_of::<u32>()];
        self.read_bytes_exact(&mut bytes, address)?;
        Ok(u32::from_le_bytes(bytes))
    }

    /// 从用户虚拟地址 `address` 完整读取一个小端序 `u64` 值（支持跨页边界与非自然对齐地址）。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "用户映射测试读取原 Mm 内容，核对跨页、COW 与截断后的页数据"
        )
    )]
    pub fn read_u64(&self, address: usize) -> Result<u64, VmError> {
        let mut bytes = [0u8; size_of::<u64>()];
        self.read_bytes_exact(&mut bytes, address)?;
        Ok(u64::from_le_bytes(bytes))
    }

    /// 从用户虚拟地址 `address` 完整读取一个目标架构字宽的小端序 `usize` 值。
    pub fn read_usize(&self, address: usize) -> Result<usize, VmError> {
        let mut bytes = [0u8; size_of::<usize>()];
        self.read_bytes_exact(&mut bytes, address)?;
        Ok(usize::from_le_bytes(bytes))
    }

    /// 从用户虚拟地址 `address` 读取以 `\0` 结尾的字节串（不含 `\0` 本身），最多扫描 `max_bytes` 字节。
    ///
    /// - 不要求内容为合法 UTF-8；`max_bytes` 计入终止符 `\0`（因此 `max_bytes == 0` 直接返回 `Unterminated`）。
    /// - 按页复制到栈上 4KiB 临时缓冲区后再扫描 `\0`，且 `Vec` 堆扩容在页面访问锁外执行，避免持锁分配内存。
    pub fn read_c_string(&self, address: usize, max_bytes: usize) -> Result<Vec<u8>, CStringError> {
        address
            .checked_add(max_bytes)
            .ok_or(CStringError::Vm(VmError::InvalidRange))?;
        if max_bytes == 0 {
            return Err(CStringError::Unterminated);
        }

        // 逐页读入内核栈临时缓冲区并查找 NUL 终止符，在页面访问锁外扩容结果 Vec。
        let mut result = Vec::new();
        let mut buffer = [0u8; PAGE_SIZE];
        let mut scanned = 0;
        while scanned < max_bytes {
            let current = address + scanned;
            let count = page_chunk(current, max_bytes - scanned);
            self.read_bytes_exact(&mut buffer[..count], current)
                .map_err(CStringError::Vm)?;
            let terminator = buffer[..count].iter().position(|byte| *byte == 0);
            let length = terminator.unwrap_or(count);
            result
                .try_reserve(length)
                .map_err(|_| CStringError::NoMemory)?;
            result.extend_from_slice(&buffer[..length]);
            if terminator.is_some() {
                return Ok(result);
            }
            scanned += count;
        }
        Err(CStringError::Unterminated)
    }

    /// 对 4 字节自然对齐的用户虚拟地址 `address` 执行一次 32 位 Acquire 原子加载。
    ///
    /// 若目标页尚未驻留，会先在锁外处理缺页（可睡眠），随后在页面映射保护期内执行单条硬件原子读指令。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "futex 和事件测试读取用户共享控制字的真实 Acquire 状态"
        )
    )]
    pub fn load_user_u32(&self, address: usize) -> Result<u32, VmError> {
        check_atomic_u32(address)?;
        self.access_page(address, UserAccess::Read, |source| {
            // SAFETY:
            // 1. 前置 `check_atomic_u32(address)` 已验证 `address` 4 字节自然对齐且完全位于单个 4KiB 页内；
            // 2. `self.access_page` 保证目标物理页存活、已初始化且具备可读映射，`arch::load_user_u32` 不构造用户内存引用。
            unsafe { arch::load_user_u32(source as *const u32) }
        })
    }

    /// 对 4 字节自然对齐的用户虚拟地址 `address` 执行一次 32 位 Release 原子存储（必要时先完成缺页或 COW 拆分）。
    pub fn store_user_u32(&self, address: usize, value: u32) -> Result<(), VmError> {
        check_atomic_u32(address)?;
        self.access_page(address, UserAccess::Write, |target| {
            // SAFETY:
            // 1. 前置 `check_atomic_u32(address)` 已验证 `address` 4 字节自然对齐且完全位于单个 4KiB 页内；
            // 2. `self.access_page` 保证目标物理页存活且具备可写（非 COW）映射，`arch::store_user_u32` 仅在回调内执行单次原子写。
            unsafe { arch::store_user_u32(target as *mut u32, value) };
        })
    }

    /// 从用户虚拟地址 `address` 精确读取 `dst.len()` 字节；若发生任何部分读取失败则返回对应的 `VmError`。
    fn read_bytes_exact(&self, dst: &mut [u8], address: usize) -> Result<(), VmError> {
        let progress = self.copyin(dst, address);
        if let Some(error) = progress.error {
            return Err(error);
        }
        assert_eq!(
            progress.copied,
            dst.len(),
            "successful user copy is incomplete"
        );
        Ok(())
    }

    /// 在目标用户页驻留且权限满足 `access` 的短期映射保护期内执行回调 `visit`。
    ///
    /// 若目标页未映射、权限不足或需要写时复制拆分，则先释放访问保护并调用 `self.fault(address, access)`，
    /// 待缺页处理完成后重新循环尝试。回调 `visit` 接收该地址对应的内核直接映射虚拟地址，且不得睡眠、分配内存或泄露指针。
    fn access_page<R>(
        &self,
        address: usize,
        access: UserAccess,
        mut visit: impl FnMut(usize) -> R,
    ) -> Result<R, VmError> {
        loop {
            if let Some(result) = self.with_page(address, access, &mut visit)? {
                return Ok(result);
            }
            self.fault(address, access)?;
        }
    }
}

/// 校验非空用户地址区间 `[address, address + length)` 是否未发生算术溢出且未超出 `USER_ADDRESS_END`。
fn check_range(address: usize, length: usize) -> Result<(), VmError> {
    if length == 0 {
        return Ok(());
    }
    let end = address.checked_add(length).ok_or(VmError::InvalidRange)?;
    if arch::USER_ADDRESS_END < end {
        return Err(VmError::InvalidRange);
    }
    Ok(())
}

/// 计算从 `address` 开始、在不跨越当前 4KiB 页边界的前提下最多可连续访问的字节数（上限为 `remaining`）。
fn page_chunk(address: usize, remaining: usize) -> usize {
    remaining.min(PAGE_SIZE - (address & (PAGE_SIZE - 1)))
}

/// 校验 32 位用户态原子操作地址是否在合法用户空间内、满足 4 字节自然对齐且不跨越页边界。
fn check_atomic_u32(address: usize) -> Result<(), VmError> {
    let length = size_of::<u32>();
    check_range(address, length)?;
    if address & (length - 1) != 0 || PAGE_SIZE - length < (address & (PAGE_SIZE - 1)) {
        return Err(VmError::InvalidRange);
    }
    Ok(())
}
