//! tmpfs 挂载选项只计算额度和初始根权限，不修改文件系统状态。

use crate::fs::types::S_IALLUGO;
use crate::fs::FsError;
use crate::mm::PAGE_SIZE;

/// inode 与名字的额度估算单位，仅用于限制可表示的 inode 数。
const BOGO_INODE_SIZE: usize = 1024;

/// 未出现的额度保留为 None，供初始默认与重配分别消费。
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct TmpfsOptions {
    pub(crate) blocks: Option<usize>,
    pub(crate) inodes: Option<usize>,
    pub(crate) mode: Option<u16>,
}

impl TmpfsOptions {
    /// 按输入顺序解析；size 与 nr_blocks 共享后值覆盖语义，零额度表示无限。
    pub fn parse(bytes: &[u8], ram_pages: usize) -> Result<Self, FsError> {
        let mut options = Self::default();
        for field in bytes
            .split(|byte| *byte == b',')
            .filter(|field| !field.is_empty())
        {
            let equal = field
                .iter()
                .position(|byte| *byte == b'=')
                .ok_or(FsError::InvalidOperation)?;
            let (name, value) = (&field[..equal], &field[equal + 1..]);
            match name {
                b"size" => {
                    let (amount, rest) = memory_value(value)?;
                    let bytes = match rest {
                        b"" => u128::from(amount),
                        b"%" => {
                            u128::from(amount)
                                .checked_mul(ram_pages as u128)
                                .and_then(|value| value.checked_mul(PAGE_SIZE as u128))
                                .ok_or(FsError::InvalidOperation)?
                                / 100
                        }
                        _ => return Err(FsError::InvalidOperation),
                    };
                    let pages = bytes.div_ceil(PAGE_SIZE as u128);
                    if (isize::MAX as u128) < pages {
                        return Err(FsError::InvalidOperation);
                    }
                    options.blocks = Some(pages as usize);
                }
                b"nr_blocks" | b"nr_inodes" => {
                    let (amount, rest) = memory_value(value)?;
                    let maximum = if name == b"nr_blocks" {
                        isize::MAX as u64
                    } else {
                        (usize::MAX / BOGO_INODE_SIZE) as u64
                    };
                    if !rest.is_empty() || maximum < amount {
                        return Err(FsError::InvalidOperation);
                    }
                    if name == b"nr_blocks" {
                        options.blocks = Some(amount as usize);
                    } else {
                        options.inodes = Some(amount as usize);
                    }
                }
                b"mode" => {
                    let (mode, rest) = number(value.strip_prefix(b"+").unwrap_or(value), 8)?;
                    if (!rest.is_empty() && rest != b"\n") || (u32::MAX as u64) < mode {
                        return Err(FsError::InvalidOperation);
                    }
                    options.mode = Some((mode & u64::from(S_IALLUGO)) as u16);
                }
                _ => return Err(FsError::InvalidOperation),
            }
        }
        Ok(options)
    }

    /// 两个目标均无 highmem，默认 inode 数与块数取实际受管页一半。
    pub(crate) fn limits(self, ram_pages: usize) -> (usize, usize) {
        (
            self.blocks.unwrap_or(ram_pages / 2),
            self.inodes
                .unwrap_or((ram_pages / 2).min(usize::MAX / BOGO_INODE_SIZE)),
        )
    }
}

/// 消费 base0 数字和一个二进制单位；剩余字节由具体选项判断。
fn memory_value(bytes: &[u8]) -> Result<(u64, &[u8]), FsError> {
    let (value, rest) = number(bytes, 0)?;
    let shift = match rest.first().map(u8::to_ascii_lowercase) {
        Some(b'k') => 10,
        Some(b'm') => 20,
        Some(b'g') => 30,
        Some(b't') => 40,
        Some(b'p') => 50,
        Some(b'e') => 60,
        _ => return Ok((value, rest)),
    };
    let value = value
        .checked_mul(1u64 << shift)
        .ok_or(FsError::InvalidOperation)?;
    Ok((value, &rest[1..]))
}

/// 在给定进制或 base0 下消费至少一位，溢出保留为真实解析错误。
fn number(mut bytes: &[u8], mut base: u64) -> Result<(u64, &[u8]), FsError> {
    if base == 0 {
        base = if bytes.starts_with(b"0x") || bytes.starts_with(b"0X") {
            bytes = &bytes[2..];
            16
        } else if bytes.starts_with(b"0") {
            8
        } else {
            10
        };
    }
    let mut value = 0u64;
    let mut digits = 0;
    for &byte in bytes {
        let digit = match byte {
            b'0'..=b'9' => u64::from(byte - b'0'),
            b'a'..=b'f' => u64::from(byte - b'a' + 10),
            b'A'..=b'F' => u64::from(byte - b'A' + 10),
            _ => break,
        };
        if base <= digit {
            break;
        }
        value = value
            .checked_mul(base)
            .and_then(|value| value.checked_add(digit))
            .ok_or(FsError::InvalidOperation)?;
        digits += 1;
    }
    if digits == 0 {
        return Err(FsError::InvalidOperation);
    }
    Ok((value, &bytes[digits..]))
}
