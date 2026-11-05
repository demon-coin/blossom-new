//! 逐页用户访问只在短映射读侧内使用原始指令；缺页处理前释放全部访问保护。

use alloc::vec::Vec;
use core::mem::size_of;

use crate::arch::{self, UserAccess};
use crate::mm::space::{Mm, VmError};
use crate::mm::PAGE_SIZE;

/// 已复制的真实前缀；错误之后的内核目标或用户目标保持原值。
#[must_use = "user copies may stop after a partial prefix"]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CopyProgress {
    pub copied: usize,
    pub error: Option<VmError>,
}

/// 有界 NUL 字节串读取保留访问、分配与缺少终止符的独立原因。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CStringError {
    Vm(VmError),
    NoMemory,
    Unterminated,
}

impl Mm {
    /// 在普通任务中逐页读取用户字节，可能处理缺页并睡眠；失败时保留已复制前缀。
    /// 零长不检查映射或解引用地址；成功不表示并发用户写入的整体快照。
    pub fn copyin(&self, dst: &mut [u8], address: usize) -> CopyProgress {
        if let Err(error) = check_range(address, dst.len()) {
            return CopyProgress {
                copied: 0,
                error: Some(error),
            };
        }

        let mut copied = 0;
        while copied < dst.len() {
            let current = address + copied;
            let count = page_chunk(current, dst.len() - copied);
            let target = dst[copied..copied + count].as_mut_ptr();
            let result = self.access_page(current, UserAccess::Read, |source| {
                // SAFETY: with_page 保持当前页及可读映射稳定，count 不超过本页剩余字节；
                // target 来自独占内核切片且容量足够，两侧正常 RAM 不重叠，用户页已初始化。
                // 原始指令不构造用户内容引用，指针不离开本次短回调。
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

    /// 在普通任务中逐页写入用户字节，缺页或私有 COW 在访问保护之外处理。
    /// 零长不解引用；错误只终止后续复制，不撤回已经写入的前缀。
    pub fn copyout(&self, address: usize, src: &[u8]) -> CopyProgress {
        if let Err(error) = check_range(address, src.len()) {
            return CopyProgress {
                copied: 0,
                error: Some(error),
            };
        }

        let mut copied = 0;
        while copied < src.len() {
            let current = address + copied;
            let count = page_chunk(current, src.len() - copied);
            let source = src[copied..copied + count].as_ptr();
            let result = self.access_page(current, UserAccess::Write, |target| {
                // SAFETY: with_page 保持当前页及可写非 COW 映射稳定，count 位于本页内；
                // source 来自有效初始化内核切片，其借用约束保证复制期间不被修改。
                // 两侧正常 RAM 不重叠，用户侧仅经原始指令访问且指针不逃逸。
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

    /// 完整读取小端 32 位 ABI 字段；允许跨页和非自然对齐，不提供原子语义。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(dead_code, reason = "用户访问测试核对跨页小端 u32 读取及真实缺页错误")
    )]
    pub fn read_u32(&self, address: usize) -> Result<u32, VmError> {
        let mut bytes = [0u8; size_of::<u32>()];
        self.read_bytes_exact(&mut bytes, address)?;
        Ok(u32::from_le_bytes(bytes))
    }

    /// 完整读取小端 64 位 ABI 字段；任何部分读取错误都不产生字段值。
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

    /// 完整读取目标机器字宽的小端 ABI 字段，不把用户字节解释为任意 Rust 对象。
    pub fn read_usize(&self, address: usize) -> Result<usize, VmError> {
        let mut bytes = [0u8; size_of::<usize>()];
        self.read_bytes_exact(&mut bytes, address)?;
        Ok(usize::from_le_bytes(bytes))
    }

    /// 最多检查 max_bytes 个用户字节，返回首个 NUL 之前的原始字节，不要求 UTF-8。
    /// 跨页时逐页重新定位，复制和扫描用初始化内核缓冲；扩容发生在访问保护之外。
    /// 上限包含终止符，零上限直接报告缺少终止符。
    pub fn read_c_string(&self, address: usize, max_bytes: usize) -> Result<Vec<u8>, CStringError> {
        address
            .checked_add(max_bytes)
            .ok_or(CStringError::Vm(VmError::InvalidRange))?;
        if max_bytes == 0 {
            return Err(CStringError::Unterminated);
        }

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

    /// 对四字节自然对齐且位于同一页的用户字执行一次独立 Acquire 读取。
    /// 缺页可睡眠，原子读取仅在当前映射读侧保护内执行。
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
            // SAFETY: 入口验证四字节对齐与页内范围，with_page 保活已初始化正常 RAM；
            // 当前映射允许读取，原始单次原子指令不构造用户引用或泄漏指针。
            unsafe { arch::load_user_u32(source as *const u32) }
        })
    }

    /// 完成写侧缺页或 COW 后，对页内自然对齐用户字执行一次独立 Release 写入。
    pub fn store_user_u32(&self, address: usize, value: u32) -> Result<(), VmError> {
        check_atomic_u32(address)?;
        self.access_page(address, UserAccess::Write, |target| {
            // SAFETY: 入口验证四字节对齐与页内范围，with_page 保活正常 RAM 和可写非 COW 叶；
            // 原始单次原子指令不构造用户引用，写访问只存在于本次短回调。
            unsafe { arch::store_user_u32(target as *mut u32, value) };
        })
    }

    /// 只在完整复制成功后允许上层解码初始化字节；部分错误保持原始 VM 原因。
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

    /// 在实际驻留页的短读侧执行回调；缺 VMA、缺页或 COW 均先放锁，经同一 fault 后再定位。
    /// 回调必须限于本页内的原始访问，不分配、睡眠或向外保留可解引用指针。
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

/// 在复制前一次检查整个非零地址区间，排除整数回绕和用户地址上界之外的访问。
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

/// 计算当前基本页内可连续访问的最长片段，不跨页保留映射保护。
fn page_chunk(address: usize, remaining: usize) -> usize {
    remaining.min(PAGE_SIZE - (address & (PAGE_SIZE - 1)))
}

/// 拒绝不自然对齐或跨基本页的原子字，错误在任何用户访问之前产生。
fn check_atomic_u32(address: usize) -> Result<(), VmError> {
    let length = size_of::<u32>();
    check_range(address, length)?;
    if address & (length - 1) != 0 || PAGE_SIZE - length < (address & (PAGE_SIZE - 1)) {
        return Err(VmError::InvalidRange);
    }
    Ok(())
}
