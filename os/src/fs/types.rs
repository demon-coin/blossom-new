//! 文件系统后端共享的 inode 属性、目录项与枚举位置类型。

use alloc::vec::Vec;

/// inode 模式中的对象类型掩码。
pub(crate) const S_IFMT: u16 = 0o170000;
/// 套接字 inode 类型。
pub(crate) const S_IFSOCK: u16 = 0o140000;
/// 符号链接 inode 类型。
pub(crate) const S_IFLNK: u16 = 0o120000;
/// 普通文件 inode 类型。
pub(crate) const S_IFREG: u16 = 0o100000;
/// 块设备 inode 类型。
pub(crate) const S_IFBLK: u16 = 0o060000;
/// 目录 inode 类型。
pub(crate) const S_IFDIR: u16 = 0o040000;
/// 字符设备 inode 类型。
pub(crate) const S_IFCHR: u16 = 0o020000;
/// 管道 inode 类型。
pub(crate) const S_IFIFO: u16 = 0o010000;

/// 执行时设置属主身份。
pub(crate) const S_ISUID: u16 = 0o4000;
/// 执行时设置属组身份。
pub(crate) const S_ISGID: u16 = 0o2000;
/// 目录粘滞删除位。
pub(crate) const S_ISVTX: u16 = 0o1000;

/// 属主读写执行权限。
pub(crate) const S_IRWXU: u16 = 0o700;
/// 属组读写执行权限。
pub(crate) const S_IRWXG: u16 = 0o070;
/// 其他用户读写执行权限。
pub(crate) const S_IRWXO: u16 = 0o007;
/// 全部普通权限位。
pub(crate) const S_IRWXUGO: u16 = S_IRWXU | S_IRWXG | S_IRWXO;
/// 全部权限及特殊模式位，不含 inode 类型。
pub(crate) const S_IALLUGO: u16 = S_ISUID | S_ISGID | S_ISVTX | S_IRWXUGO;
/// 任一身份的执行权限位。
pub(crate) const S_IXUGO: u16 = 0o111;
/// 属组执行权限位。
pub(crate) const S_IXGRP: u16 = 0o010;

/// 访问检查的读权限。
pub(crate) const MAY_READ: u16 = 4;
/// 访问检查的写权限。
pub(crate) const MAY_WRITE: u16 = 2;
/// 访问检查的执行权限。
pub(crate) const MAY_EXEC: u16 = 1;

/// 单个路径组件的最大字节数。
pub(crate) const NAME_MAX: usize = 255;
/// 完整路径的最大字节数，包含结尾 NUL。
pub(crate) const PATH_MAX: usize = 4096;

/// inode 类型同时保留普通文件、目录、链接和可识别的设备节点语义。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum InodeKind {
    Regular,
    Directory,
    Symlink,
    Character,
    Block,
    Fifo,
    Socket,
    Other,
}

/// inode 时间按盘面有符号秒与扩展纳秒保存，不在只读读取时修改时钟。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct InodeTime {
    pub seconds: i64,
    pub nanoseconds: u32,
}

/// 目录枚举返回的 inode 号、原始名字及下一次枚举所需的后端位置。
#[derive(Debug, Eq, PartialEq)]
pub struct DirectoryEntry {
    pub number: u64,
    pub name: Vec<u8>,
    pub kind: Option<InodeKind>,
    pub next_cookie: u64,
    pub(crate) next_cursor: DirectoryCursor,
}

/// 目录游标保留后端可见 cookie；内存目录不应被磁盘后端解释为字节位置。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum DirectoryCursor {
    /// 普通目录的盘面字节偏移。
    Byte(u64),
    /// indexed 目录的盘面哈希和碰撞续读位置。
    Indexed {
        major: u32,
        minor: u32,
        collision: Option<DirectoryPosition>,
    },
    /// 内存目录拥有的真实游标值。
    Memory(u64),
}

/// 精确定位已交付记录，区分 major/minor 完全相同的多个目录项。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct DirectoryPosition {
    /// inode mapping 中的目录逻辑块号。
    pub(crate) logical: u64,
    /// 目录块内最后一次交付的记录字节偏移。
    pub(crate) offset: u32,
}

impl DirectoryCursor {
    /// 返回给用户的稳定目录 cookie；碰撞位置只保存在 OFD 游标内。
    pub(crate) fn cookie(&self) -> u64 {
        match self {
            Self::Byte(offset) | Self::Memory(offset) => *offset,
            Self::Indexed { major, minor, .. } => {
                (u64::from(*major >> 1) << u32::BITS) | u64::from(*minor)
            }
        }
    }
}
