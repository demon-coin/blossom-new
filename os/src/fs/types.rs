//! 文件系统公共基础类型、POSIX 模式位掩码与目录遍历游标定义。
//!
//! # 模块职责与架构定位
//! 本模块定义虚拟文件系统（VFS）与各后端文件系统（如 `ext4`、`memfs`、`devfs`、`procfs`）共享的值类型与常量：
//! - **上游调用者**：系统调用层（`stat`/`getdents64`/`chmod`/`open` 等）、VFS 路径解析（`path`）与打开文件抽象（`file`）。
//! - **下游实现者**：`inode` 缓存层与各具体文件系统后端。
//!
//! # 核心设计与位域布局
//! ## 1. POSIX `st_mode` 16 位模式字布局
//! ```text
//!  15  12 11   9 8    6 5    3 2    0
//! +------+------+------+------+------+
//! | 文件 | 特殊 | 属主 | 属组 | 其他 |
//! | 类型 | 标志 | 权限 | 权限 | 权限 |
//! +------+------+------+------+------+
//!  S_IFMT  suid   rwx    rwx    rwx
//!          sgid
//!          svtx
//! ```
//! - **位 `[15:12]`（`S_IFMT = 0o170000`）**：标识 Inode 的对象类型（普通文件、目录、符号链接、块/字符设备、FIFO、Socket）。
//! - **位 `[11:9]`（`0o007000`）**：特殊执行与目录删除控制位（`S_ISUID`、`S_ISGID`、`S_ISVTX`）。
//! - **位 `[8:0]`（`S_IRWXUGO = 0o000777`）**：属主（User）、属组（Group）与其他用户（Other）的读、写、执行权限位。
//!
//! ## 2. 目录遍历游标（`DirectoryCursor`）与用户态 Cookie
//! - 磁盘线性目录与内存目录分别使用字节偏移（`Byte`）与槽位序号（`Memory`）作为游标。
//! - ext4 HTree 哈希索引目录使用 `Indexed { major, minor, collision }` 按哈希顺序枚举：对外暴露给 `getdents64`（`d_off`）及 `lseek` 的 64 位稳定 Cookie 编码为 `((major >> 1) << 32) | minor`；而同一哈希值下的碰撞块内精确位置 `DirectoryPosition` 保存在打开文件描述（OFD）内部游标中，防止哈希碰撞导致重复或漏读目录项。
//!
//! # 锁与并发契约
//! 本模块仅包含纯数据类型与不可变常量，不持有任何锁。

use alloc::vec::Vec;

/// Inode 模式字（`st_mode`）中的文件类型位域掩码（`0o170000`）。
pub(crate) const S_IFMT: u16 = 0o170000;
/// 套接字（Socket）Inode 类型值（`0o140000`）。
pub(crate) const S_IFSOCK: u16 = 0o140000;
/// 符号链接（Symbolic Link）Inode 类型值（`0o120000`）。
pub(crate) const S_IFLNK: u16 = 0o120000;
/// 普通文件（Regular File）Inode 类型值（`0o100000`）。
pub(crate) const S_IFREG: u16 = 0o100000;
/// 块设备节点（Block Device）Inode 类型值（`0o060000`）。
pub(crate) const S_IFBLK: u16 = 0o060000;
/// 目录（Directory）Inode 类型值（`0o040000`）。
pub(crate) const S_IFDIR: u16 = 0o040000;
/// 字符设备节点（Character Device）Inode 类型值（`0o020000`）。
pub(crate) const S_IFCHR: u16 = 0o020000;
/// 命名管道（FIFO）Inode 类型值（`0o010000`）。
pub(crate) const S_IFIFO: u16 = 0o010000;

/// 执行程序时将进程有效用户 ID（EUID）设为文件属主（Set-user-ID，`0o4000`）。
pub(crate) const S_ISUID: u16 = 0o4000;
/// 执行程序时将进程有效组 ID（EGID）设为文件属组，或在目录上使新建子项继承父目录属组（Set-group-ID，`0o2000`）。
pub(crate) const S_ISGID: u16 = 0o2000;
/// 目录粘滞位（Sticky Bit，`0o1000`），限制仅目录属主、文件属主或特权用户可删除或重命名目录内文件。
pub(crate) const S_ISVTX: u16 = 0o1000;

/// 文件属主（User）的读、写、执行权限掩码（`0o700`）。
pub(crate) const S_IRWXU: u16 = 0o700;
/// 文件属组（Group）的读、写、执行权限掩码（`0o070`）。
pub(crate) const S_IRWXG: u16 = 0o070;
/// 其他用户（Other）的读、写、执行权限掩码（`0o007`）。
pub(crate) const S_IRWXO: u16 = 0o007;
/// 属主、属组与其他用户的全部基本读写执行权限位掩码（`0o777`）。
pub(crate) const S_IRWXUGO: u16 = S_IRWXU | S_IRWXG | S_IRWXO;
/// 全部基本权限位及特殊模式位（`S_ISUID | S_ISGID | S_ISVTX | S_IRWXUGO`，`0o7777`），不含文件类型位。
pub(crate) const S_IALLUGO: u16 = S_ISUID | S_ISGID | S_ISVTX | S_IRWXUGO;
/// 属主、属组或其他用户任一身份的执行权限位掩码（`0o111`）。
pub(crate) const S_IXUGO: u16 = 0o111;
/// 文件属组（Group）的单独执行权限位掩码（`0o010`）。
pub(crate) const S_IXGRP: u16 = 0o010;

/// 权限检查（Permission Check）请求掩码：检查读权限（`4`）。
pub(crate) const MAY_READ: u16 = 4;
/// 权限检查（Permission Check）请求掩码：检查写权限（`2`）。
pub(crate) const MAY_WRITE: u16 = 2;
/// 权限检查（Permission Check）请求掩码：检查执行或目录搜索权限（`1`）。
pub(crate) const MAY_EXEC: u16 = 1;

/// 单个目录项名字（不含路径分隔符 `/`）的最大长度（单位：字节）。
pub(crate) const NAME_MAX: usize = 255;
/// 完整路径字符串（含末尾 `\0` 终止符）的最大长度（单位：字节）。
pub(crate) const PATH_MAX: usize = 4096;

/// 文件系统对象的逻辑类型枚举。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum InodeKind {
    /// 普通数据文件。
    Regular,
    /// 目录。
    Directory,
    /// 符号链接。
    Symlink,
    /// 字符设备节点。
    Character,
    /// 块设备节点。
    Block,
    /// 命名管道（FIFO）。
    Fifo,
    /// 本地或网络套接字节点。
    Socket,
    /// 未知或未分类的特殊 Inode 类型。
    Other,
}

/// Inode 时间戳，记录距 Unix 纪元（1970-01-01 00:00:00 UTC）的秒数与纳秒偏移。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct InodeTime {
    /// 距 Unix 纪元的有符号秒数（支持 1970 年以前的时间戳及 64 位扩展秒数，单位：秒）。
    pub seconds: i64,
    /// 秒内的亚秒级时间偏移（取值范围 `0..1_000_000_000`，单位：纳秒）。
    pub nanoseconds: u32,
}

/// 单次目录读取返回的一条目录项记录。
#[derive(Debug, Eq, PartialEq)]
pub struct DirectoryEntry {
    /// 目录项指向的目标 Inode 编号（对应 `d_ino`）。
    pub number: u64,
    /// 目录项的原始字节名字（不含末尾 `\0`）。
    pub name: Vec<u8>,
    /// 目录项记录中携带的目标对象类型提示（如 ext4 目录项的 `file_type`，若格式未记录则为 `None`）。
    pub kind: Option<InodeKind>,
    /// 指向下一条目录项的 64 位稳定偏移标识（导出至 `linux_dirent64::d_off`，单位：字节偏移或哈希值）。
    pub next_cookie: u64,
    /// 供打开文件描述（OFD）保存的下一次 `readdir` 内部续读游标。
    pub(crate) next_cursor: DirectoryCursor,
}

/// 目录遍历的内部位置游标，区分磁盘线性目录、ext4 哈希索引目录与内存目录。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum DirectoryCursor {
    /// 磁盘线性目录中的字节偏移量（单位：字节）。
    Byte(u64),
    /// ext4 HTree 哈希索引目录的遍历位置。
    Indexed {
        /// 32 位主哈希值（Major Hash）。
        major: u32,
        /// 32 位次哈希值（Minor Hash）。
        minor: u32,
        /// 发生哈希值完全碰撞时，上一次已交付目录项在叶子块中的物理位置；无碰撞时为 `None`。
        collision: Option<DirectoryPosition>,
    },
    /// 内存文件系统（`memfs`/`devfs`/`procfs`）目录中的逻辑序号游标。
    Memory(u64),
}

/// 磁盘目录块内的精确记录坐标，用于在 HTree 哈希值（Major/Minor）完全相同的多条目录项中定位续读起点。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct DirectoryPosition {
    /// 目录文件数据映射（Inode Mapping）中的逻辑块号（单位：文件系统块）。
    pub(crate) logical: u64,
    /// 上一次已交付目录项在该目录块内的起始字节偏移（单位：字节）。
    pub(crate) offset: u32,
}

impl DirectoryCursor {
    /// 将内部目录游标编码为导出给用户态（如 `getdents64` 的 `d_off` 与 `lseek`）的 64 位稳定 Cookie。
    pub(crate) fn cookie(&self) -> u64 {
        match self {
            Self::Byte(offset) | Self::Memory(offset) => *offset,
            Self::Indexed { major, minor, .. } => {
                (u64::from(*major >> 1) << u32::BITS) | u64::from(*minor)
            }
        }
    }
}
