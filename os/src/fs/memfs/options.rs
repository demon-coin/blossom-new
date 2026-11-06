//! `tmpfs` 挂载选项解析与配额计算模块。
//!
//! ## 模块职责与架构定位
//! 本模块位于内存文件系统（[`crate::fs::memfs`]）的下层，负责将挂载或重挂载（remount）
//! 时传入的原始字节选项解析为结构化的 [`TmpfsOptions`]。上游调用者为
//! `MemoryFs::try_tmpfs` 与 `MemoryFs::reconfigure`，下游依赖虚拟文件系统的权限掩码常量与
//! 物理内存页大小常量（[`PAGE_SIZE`]）。本模块仅执行无副作用的语法解析与额度换算，
//! 不直接修改任何文件系统实例状态。
//!
//! ## 选项格式与覆盖规则
//! 挂载选项以英文逗号 `,` 分隔，每项为 `key=value` 形式，按从左到右顺序扫描：
//!
//! ```text
//! 输入字节流: "size=64M,nr_inodes=4k,mode=01777"
//!               │         │            └─► 八进制权限位（仅保留 S_IALLUGO 范围内的位）
//!               │         └─► 最大 Inode 数量（支持 k/m/g/t/p/e 二进制单位后缀）
//!               └─► 最大数据容量（支持字节数+二进制单位后缀，或百分比 "%"）
//!                     └─► 向上取整换算为物理页数，写入 options.blocks
//! ```
//!
//! - `size` 与 `nr_blocks` 均控制最大物理页额度（`blocks`），若同时出现，后出现的选项覆盖前者。
//! - 额度值为 `0` 表示不限制（无限额度）；未显式指定的项保持 `None`，由初始挂载提供默认值（受管物理内存页数的一半），或在重配置（`reconfigure`）时保持原有额度不变。
//!
//! ## 锁获取顺序与并发契约
//! 本模块为纯计算逻辑，不持有任何锁，可在任意上下文中无锁调用。

use crate::fs::types::S_IALLUGO;
use crate::fs::FsError;
use crate::mm::PAGE_SIZE;

/// 估算单个 Inode 及其目录项名称占用的假想字节数（1024 字节），仅用于推导 `nr_inodes` 的防溢出上限。
const BOGO_INODE_SIZE: usize = 1024;

/// 解析后的 `tmpfs` 挂载与重配置选项。
///
/// 未在挂载参数中出现的选项字段保持为 `None`：
/// - 初次挂载时，通过 [`TmpfsOptions::limits`] 将 `None` 填充为默认配额（受管 RAM 页数的一半）；
/// - 重配置（remount）时，`None` 表示保留文件系统当前的配额不变。
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct TmpfsOptions {
    /// 最大可分配的数据物理页数（单位：页，`PAGE_SIZE` 字节）；`Some(0)` 表示不设上限。
    pub(crate) blocks: Option<usize>,
    /// 最大可分配的 Inode 数量（单位：个）；`Some(0)` 表示不设上限。
    pub(crate) inodes: Option<usize>,
    /// 根目录的初始权限位（仅包含 `S_IALLUGO` 范围内的 12 位权限与特殊位，不含文件类型位）。
    pub(crate) mode: Option<u16>,
}

impl TmpfsOptions {
    /// 按从左到右的顺序解析逗号分隔的 `tmpfs` 挂载选项字节串。
    ///
    /// - 调用上下文：任意上下文（无锁纯计算）。
    /// - 参数：
    ///   - `bytes`：原始挂载选项字节切片（如 `b"size=50%,mode=0755"`）。
    ///   - `ram_pages`：系统当前受管物理内存总页数，用于计算 `size=N%` 百分比额度。
    /// - 返回值：解析成功返回 [`TmpfsOptions`]；遇到未知键名、语法错误或数值溢出时返回 [`FsError::InvalidOperation`]。
    pub fn parse(bytes: &[u8], ram_pages: usize) -> Result<Self, FsError> {
        let mut options = Self::default();
        // Step 1: 按逗号切分选项并忽略空段，逐项拆分 `name=value` 键值对。
        for field in bytes
            .split(|byte| *byte == b',')
            .filter(|field| !field.is_empty())
        {
            let equal = field
                .iter()
                .position(|byte| *byte == b'=')
                .ok_or(FsError::InvalidOperation)?;
            let (name, value) = (&field[..equal], &field[equal + 1..]);
            // Step 2: 根据选项名称分别解析容量、块数、Inode 数或根目录权限模式。
            match name {
                b"size" => {
                    // 支持纯字节数（可带 k/m/g 等二进制单位）或占受管 RAM 的百分比（`%`），并向上取整为页数。
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
                    // 解析块数或 Inode 数（支持二进制单位后缀，但不允许尾随其他字符）。
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
                    // 解析八进制权限值（允许可选前导 `+` 号及尾随单个换行符），并掩码保留有效权限位。
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

    /// 将未显式指定的容量与 Inode 额度填充为系统默认值。
    ///
    /// 默认策略取受管物理内存页数（`ram_pages`）的一半作为最大数据页数和最大 Inode 数。
    /// 返回元组为 `(max_pages, max_inodes)`。
    pub(crate) fn limits(self, ram_pages: usize) -> (usize, usize) {
        (
            self.blocks.unwrap_or(ram_pages / 2),
            self.inodes
                .unwrap_or((ram_pages / 2).min(usize::MAX / BOGO_INODE_SIZE)),
        )
    }
}

/// 解析支持自动进制识别的非负整数及可选的二进制容量单位后缀（`k`/`m`/`g`/`t`/`p`/`e`，不区分大小写）。
///
/// 返回换算后的字节/数量值与尚未消费的剩余字节切片；若数值或左移乘法溢出则返回 [`FsError::InvalidOperation`]。
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

/// 按指定进制 `base`（若 `base == 0` 则按 `0x`/`0` 前缀自动识别 16/8/10 进制）解析无符号整数。
///
/// 要求至少消费一位有效数字；若无有效数字或计算过程中发生 `u64` 溢出，则返回 [`FsError::InvalidOperation`]。
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
