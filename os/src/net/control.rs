//! 网络接口与默认路由的 `ioctl` 控制入口。
//!
//! # 模块职责与架构定位
//! 本模块负责处理绑定在网络套接字（Socket）上的接口与路由控制命令（`SIOC*`）。
//! 上游调用者为套接字文件描述符的 `ioctl` 分发层（`crate::net::socket`），
//! 下游委托网络协议栈根实例 [`NetStackRef`]（`crate::net`）查询或更新接口状态与路由表。
//!
//! # 核心设计与 ABI 内存布局
//! 为避免在持有网络栈内部锁期间直接访问用户态页表，本模块采用两阶段处理：
//! 1. **边界拷贝与校验**：先将用户态传入的固定长度 ABI 结构体（`ifreq` 或 `rtentry`）一次性完整拷贝到内核栈缓冲区，并在内核态解析和校验所有参数；
//! 2. **提交或回写**：调用 [`NetStackRef`] 完成状态变更或快照读取；若是查询类命令（Getter），仅修改缓冲区中的输出联合体字段，最后一次性写回用户空间。
//!
//! 在小端 LP64 架构（RISC-V 64 / LoongArch 64）下，两个核心 ABI 结构的字节布局如下：
//!
//! ```text
//! struct ifreq (40 bytes):
//! +-------------------+-----------------------------------------------+
//! | 0..16             | 16..40                                        |
//! | ifr_name (16B)    | ifr_ifru union (24B):                         |
//! | NUL-terminated    | - ifr_flags: u16 at [16..18]                  |
//! | ("lo" / "eth0")   | - ifr_addr / ifr_netmask: sockaddr at [16..32]|
//! +-------------------+-----------------------------------------------+
//!
//! struct rtentry (120 bytes, LP64):
//! +--------+------------+---------------+---------------+--------+----+--------+--------+--------+
//! | 0..8   | 8..24      | 24..40        | 40..56        | 56..58 | .. | 80..82 | 88..96 | 96..120|
//! | rt_pad | rt_dst     | rt_gateway    | rt_genmask    | flags  |    | metric | rt_dev | mtu/win|
//! | (u64)  | (sockaddr) | (sockaddr)    | (sockaddr)    | (u16)  |    | (i16)  | (*char)| /irtt  |
//! +--------+------------+---------------+---------------+--------+----+--------+--------+--------+
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 本模块自身不持有任何锁。
//! - 所有用户态内存拷贝（[`Mm::copyin`] / [`Mm::copyout`]）必须在调用 [`NetStackRef`] 的方法**之前或之后**执行，严禁在持有网络栈接口锁或自旋锁时触发用户页表访问。

use core::convert::TryInto;

use crate::fs::{FileIoContext, FsError};
use crate::mm::space::Mm;
use crate::net::address::{AF_INET, AF_UNSPEC, INET_ADDRESS_LEN};
use crate::net::route::{DefaultRouteRequest, InterfaceId, IPV4_ADDRESS_BITS, RTF_GATEWAY, RTF_UP};
use crate::net::NetStackRef;

const SIOCADDRT: u32 = 0x890b;
const SIOCDELRT: u32 = 0x890c;

const SIOCGIFFLAGS: u32 = 0x8913;
const SIOCSIFFLAGS: u32 = 0x8914;
const SIOCGIFADDR: u32 = 0x8915;
const SIOCSIFADDR: u32 = 0x8916;
const SIOCGIFNETMASK: u32 = 0x891b;
const SIOCSIFNETMASK: u32 = 0x891c;

const IFF_UP: u16 = 0x1;
const IFF_BROADCAST: u16 = 0x2;
const IFF_LOOPBACK: u16 = 0x8;
const IFF_RUNNING: u16 = 0x40;

const IFREQ_LEN: usize = 40;
const RTENTRY_LEN: usize = 120;
const IFNAME_LEN: usize = 16;

// RISC-V 64 与 LoongArch 64 均为小端 LP64 架构，`rtentry` 的三个 `sockaddr` 字段紧随首个 8 字节机器字之后，尾部指针与长整型字段按 8 字节对齐。
const RT_DST_OFFSET: usize = core::mem::size_of::<u64>();
const RT_GATEWAY_OFFSET: usize = RT_DST_OFFSET + INET_ADDRESS_LEN;
const RT_GENMASK_OFFSET: usize = RT_GATEWAY_OFFSET + INET_ADDRESS_LEN;
const RT_FLAGS_OFFSET: usize = RT_GENMASK_OFFSET + INET_ADDRESS_LEN;
const RT_METRIC_OFFSET: usize = 80;
const RT_DEVICE_OFFSET: usize = 88;
const RT_MTU_OFFSET: usize = RT_DEVICE_OFFSET + core::mem::size_of::<u64>();
const RT_WINDOW_OFFSET: usize = RT_MTU_OFFSET + core::mem::size_of::<u64>();
const RT_IRTT_OFFSET: usize = RT_WINDOW_OFFSET + core::mem::size_of::<u64>();

/// 处理网络套接字上的接口与路由控制 `ioctl` 请求。
///
/// 在普通可睡眠的用户任务上下文中调用。所有修改类命令（Setter）要求调用者具备 `uid == 0`（root 权限）。
/// 未知的请求码返回 [`FsError::NotTty`]（`ENOTTY`）；内核态调用上下文（`FileIoContext::Kernel`）不提供用户地址空间，直接拒绝并返回 [`FsError::InvalidOperation`]。
///
/// 全部用户态内存读写均在进入 [`NetStackRef`] 内部锁之前或退出之后完成。
pub(crate) fn ioctl(
    net: &NetStackRef,
    request: u32,
    argument: usize,
    caller: FileIoContext<'_>,
) -> Result<usize, FsError> {
    let setter = match request {
        SIOCGIFFLAGS | SIOCGIFADDR | SIOCGIFNETMASK => false,
        SIOCSIFFLAGS | SIOCSIFADDR | SIOCSIFNETMASK | SIOCADDRT | SIOCDELRT => true,
        _ => return Err(FsError::NotTty),
    };
    let FileIoContext::User(user) = caller else {
        return Err(FsError::InvalidOperation);
    };
    if setter
        && user
            .runtime()
            .expect("network ioctl has no runtime")
            .credentials
            .uid
            != 0
    {
        return Err(FsError::NotPermitted);
    }

    let mm = user.mm().ok_or(FsError::BadAddress)?;
    match request {
        SIOCADDRT | SIOCDELRT => route_ioctl(net, &mm, request, argument),
        _ => interface_ioctl(net, &mm, request, argument),
    }
}

/// 处理网络接口相关的 `ioctl` 命令（`SIOCGIF*` / `SIOCSIF*`）。
///
/// 一次性从用户空间拷贝完整的 40 字节 `ifreq` 结构并解析接口名：
/// - 查询命令（Getter）仅覆写 `ifreq` 偏移 16 字节处的输出字段，随后将完整结构写回用户空间；
/// - 配置命令（Setter）在完成地址族、标志位或掩码前缀校验后，将解析出的内核值提交给 [`NetStackRef`]。
fn interface_ioctl(
    net: &NetStackRef,
    mm: &Mm,
    request: u32,
    argument: usize,
) -> Result<usize, FsError> {
    let mut bytes = copyin::<IFREQ_LEN>(mm, argument)?;
    // Step 1: LP64 下 `ifreq.ifr_name` 占前 16 字节（最多 15 个有效字符），强制将第 16 字节置零以保证 NUL 结尾；偏移 16 字节处为联合体载荷。
    bytes[IFNAME_LEN - 1] = 0;
    let id = interface_name(&bytes[..IFNAME_LEN])?;

    // Step 2: 按具体命令分发读取接口快照或提交配置变更。
    match request {
        SIOCGIFFLAGS => {
            let interface = net.interface_snapshot(id)?;
            let medium = match id {
                InterfaceId::Loopback => IFF_LOOPBACK,
                InterfaceId::Ethernet => IFF_BROADCAST,
            };
            let flags = medium
                | if interface.up {
                    IFF_UP | IFF_RUNNING
                } else {
                    0
                };
            bytes[16..18].copy_from_slice(&flags.to_le_bytes());
            copyout(mm, argument, &bytes)
        }
        SIOCGIFADDR | SIOCGIFNETMASK => {
            let interface = net.interface_snapshot(id)?;
            let address = interface.address.ok_or(FsError::AddressNotAvailable)?;
            let value = if request == SIOCGIFADDR {
                address
            } else {
                prefix_mask(interface.prefix_len)
            };

            // 显式清零整个 16 字节 `sockaddr_in` 区域（含端口号与尾随填充字节），仅填入地址族与网络字节序的 IPv4 地址。
            bytes[16..32].fill(0);
            bytes[16..18].copy_from_slice(&AF_INET.to_le_bytes());
            bytes[20..24].copy_from_slice(&value);
            copyout(mm, argument, &bytes)
        }
        SIOCSIFFLAGS => {
            let flags = word(&bytes, IFNAME_LEN);
            if flags & !(IFF_UP | IFF_LOOPBACK | IFF_BROADCAST | IFF_RUNNING) != 0 {
                return Err(FsError::Unsupported);
            }

            net.set_interface_up(id, flags & IFF_UP != 0)?;
            Ok(0)
        }
        SIOCSIFADDR => {
            let address = ipv4_sockaddr(&bytes, IFNAME_LEN)?;
            net.set_interface_address(id, address)?;
            Ok(0)
        }
        SIOCSIFNETMASK => {
            let mask = ipv4_sockaddr(&bytes, IFNAME_LEN)?;
            net.set_interface_netmask(id, mask_prefix(mask)?)?;
            Ok(0)
        }
        _ => unreachable!("interface ioctl received route or unknown command"),
    }
}

/// 处理路由表相关的 `ioctl` 命令（`SIOCADDRT` / `SIOCDELRT`）。
///
/// 一次性从用户空间拷贝 120 字节的 `rtentry` 结构及可选的 `rt_dev` 设备名字符串，
/// 校验其表示一条指向网关的默认路由（`0.0.0.0/0`），随后请求 [`NetStackRef`] 添加或删除默认路由。
fn route_ioctl(
    net: &NetStackRef,
    mm: &Mm,
    request: u32,
    argument: usize,
) -> Result<usize, FsError> {
    // Step 1: 拷贝完整的 `rtentry` 并按 LP64 固定字节偏移解析目的地址、网关地址与子网掩码。
    let bytes = copyin::<RTENTRY_LEN>(mm, argument)?;
    let destination = ipv4_sockaddr(&bytes, RT_DST_OFFSET)?;
    let gateway = ipv4_sockaddr(&bytes, RT_GATEWAY_OFFSET)?;
    let mask = address_bytes(&bytes, RT_GENMASK_OFFSET);
    match word(&bytes, RT_GENMASK_OFFSET) {
        AF_INET => {}
        AF_UNSPEC if mask == [0; 4] => {}
        _ => return Err(FsError::InvalidOperation),
    }
    let prefix = mask_prefix(mask)?;
    if destination != [0; 4] || prefix != 0 {
        return Err(FsError::Unsupported);
    }

    // Step 2: 校验路由标志位（必须且只能包含 `RTF_UP | RTF_GATEWAY`）以及 metric、MTU、window、irtt 等扩展参数。
    let flags = word(&bytes, RT_FLAGS_OFFSET);
    if flags & !(RTF_UP | RTF_GATEWAY) != 0 {
        return Err(FsError::Unsupported);
    }
    if flags != RTF_UP | RTF_GATEWAY {
        return Err(FsError::InvalidOperation);
    }
    let metric = word(&bytes, RT_METRIC_OFFSET) as i16;
    if (metric != 0 && metric != 1)
        || long(&bytes, RT_MTU_OFFSET) != 0
        || long(&bytes, RT_WINDOW_OFFSET) != 0
        || word(&bytes, RT_IRTT_OFFSET) != 0
    {
        return Err(FsError::Unsupported);
    }

    // Step 3: 若 `rt_dev` 指针非空，从用户空间读取以 NUL 结尾的接口名称字符串并解析为 `InterfaceId`。
    let device = long(&bytes, RT_DEVICE_OFFSET) as usize;
    let interface = if device == 0 {
        None
    } else {
        let name = copy_device_name(mm, device)?;
        Some(interface_name(&name)?)
    };

    net.change_default_route(
        DefaultRouteRequest { interface, gateway },
        request == SIOCADDRT,
    )?;
    Ok(0)
}

/// 从 `ifreq.ifr_name` 或 `rt_dev` 字节切片中截取首个 NUL 之前的有效字节，并解析为内核接口标识 [`InterfaceId`]。
///
/// 仅识别 `"lo"` 与 `"eth0"`，不支持地址别名（如 `"eth0:0"`）或其他未知设备名。
fn interface_name(name: &[u8]) -> Result<InterfaceId, FsError> {
    let length = name
        .iter()
        .position(|byte| *byte == 0)
        .unwrap_or(name.len());
    InterfaceId::from_name(&name[..length])
}

/// 从用户空间指针 `argument` 处逐字节读取可选的 `rt_dev` 设备名字符串。
///
/// 遇到首个 NUL 字节即停止，最多读取 15 个字节（`IFNAME_LEN - 1`），第 16 个字节始终保持为 0。
fn copy_device_name(mm: &Mm, argument: usize) -> Result<[u8; IFNAME_LEN], FsError> {
    let mut name = [0u8; IFNAME_LEN];
    for (offset, byte) in name.iter_mut().enumerate().take(IFNAME_LEN - 1) {
        let address = argument.checked_add(offset).ok_or(FsError::BadAddress)?;
        *byte = copyin::<1>(mm, address)?[0];
        if *byte == 0 {
            break;
        }
    }
    Ok(name)
}

/// 校验 `bytes[offset..]` 处的 `sockaddr_in` 地址族是否为 `AF_INET`，并提取其中的 4 字节 IPv4 地址。
///
/// 忽略在接口与路由配置中没有语义的端口号（`sin_port`）和填充字节（`sin_zero`）。
fn ipv4_sockaddr(bytes: &[u8], offset: usize) -> Result<[u8; 4], FsError> {
    if word(bytes, offset) != AF_INET {
        return Err(FsError::InvalidOperation);
    }
    Ok(address_bytes(bytes, offset))
}

/// 提取 `sockaddr_in` 结构体内（偏移 `offset + 4..offset + 8`）的 4 字节网络字节序 IPv4 地址。
fn address_bytes(bytes: &[u8], offset: usize) -> [u8; 4] {
    bytes[offset + 4..offset + 8]
        .try_into()
        .expect("fixed IPv4 address field")
}

/// 将连续的 IPv4 子网掩码字节转换为 CIDR 前缀长度（`0..=32`）。
///
/// 全零掩码对应前缀长度 `0`，全一掩码对应 `32`；若掩码二进制位包含非连续的 `1`（如 `255.0.255.0`），返回 [`FsError::InvalidOperation`]。
fn mask_prefix(mask: [u8; 4]) -> Result<u8, FsError> {
    let prefix = u32::from_be_bytes(mask).leading_ones() as u8;
    if prefix_mask(prefix) != mask {
        return Err(FsError::InvalidOperation);
    }
    Ok(prefix)
}

/// 将范围在 `0..=32` 内的 CIDR 前缀长度转换为 4 字节网络字节序的 IPv4 子网掩码。
fn prefix_mask(prefix: u8) -> [u8; 4] {
    let mask = if prefix == 0 {
        0
    } else {
        u32::MAX << (IPV4_ADDRESS_BITS - prefix)
    };
    mask.to_be_bytes()
}

/// 从已拷贝到内核的 ABI 字节缓冲区 `bytes[offset..offset + 2]` 中读取小端序 `u16` 字段。
fn word(bytes: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes(
        bytes[offset..offset + 2]
            .try_into()
            .expect("fixed ioctl word field"),
    )
}

/// 从已拷贝到内核的 LP64 ABI 字节缓冲区 `bytes[offset..offset + 8]` 中读取小端序 `u64` 字段。
fn long(bytes: &[u8], offset: usize) -> u64 {
    u64::from_le_bytes(
        bytes[offset..offset + 8]
            .try_into()
            .expect("fixed ioctl long field"),
    )
}

/// 从用户地址空间 `argument` 一次性完整拷贝 `N` 字节到内核栈数组。
///
/// 若发生缺页错误或实际拷贝字节数不足 `N`，统一返回 [`FsError::BadAddress`]（`EFAULT`），避免使用不完整的输入修改网络配置。
fn copyin<const N: usize>(mm: &Mm, argument: usize) -> Result<[u8; N], FsError> {
    let mut bytes = [0u8; N];
    let progress = mm.copyin(&mut bytes, argument);
    if progress.error.is_some() || progress.copied != N {
        return Err(FsError::BadAddress);
    }
    Ok(bytes)
}

/// 将已初始化完毕的 `ifreq` 字节切片一次性完整写回用户地址空间 `argument`。
///
/// 若写入中途失败或未能写满全部字节，返回 [`FsError::BadAddress`]（`EFAULT`）。
fn copyout(mm: &Mm, argument: usize, bytes: &[u8]) -> Result<usize, FsError> {
    let progress = mm.copyout(argument, bytes);
    if progress.error.is_some() || progress.copied != bytes.len() {
        return Err(FsError::BadAddress);
    }
    Ok(0)
}
