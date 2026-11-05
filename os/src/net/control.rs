//! 接口与默认路由控制先复制目标 ABI 字节，再以内核小值修改同一网络域。

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

// 两目标均为小端 LP64，rtentry 的地址字段紧随首个机器字，尾部机器字按八字节对齐。
const RT_DST_OFFSET: usize = core::mem::size_of::<u64>();
const RT_GATEWAY_OFFSET: usize = RT_DST_OFFSET + INET_ADDRESS_LEN;
const RT_GENMASK_OFFSET: usize = RT_GATEWAY_OFFSET + INET_ADDRESS_LEN;
const RT_FLAGS_OFFSET: usize = RT_GENMASK_OFFSET + INET_ADDRESS_LEN;
const RT_METRIC_OFFSET: usize = 80;
const RT_DEVICE_OFFSET: usize = 88;
const RT_MTU_OFFSET: usize = RT_DEVICE_OFFSET + core::mem::size_of::<u64>();
const RT_WINDOW_OFFSET: usize = RT_MTU_OFFSET + core::mem::size_of::<u64>();
const RT_IRTT_OFFSET: usize = RT_WINDOW_OFFSET + core::mem::size_of::<u64>();

/// 普通任务以本次调用者权限配置 socket 所属网络域；全部用户访问发生在 Root 调用之前或之后。
/// 未知命令返回 ENOTTY，Kernel 的已知指针命令不解引用裸地址。
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

/// 一次复制完整 ifreq；getter 只改所选输出字段，setter 只提交完整解析后的内核值。
fn interface_ioctl(
    net: &NetStackRef,
    mm: &Mm,
    request: u32,
    argument: usize,
) -> Result<usize, FsError> {
    let mut bytes = copyin::<IFREQ_LEN>(mm, argument)?;
    // LP64 ifreq 名称最多十五字节，最后一字节由内核终止；union 在偏移十六处。
    bytes[IFNAME_LEN - 1] = 0;
    let id = interface_name(&bytes[..IFNAME_LEN])?;

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

            // sockaddr 的端口及 padding 明确初始化，地址保持网络字节序。
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

/// 完整 rtentry 与可选设备名复制完成后，才管理库 Routes 中唯一默认项。
fn route_ioctl(
    net: &NetStackRef,
    mm: &Mm,
    request: u32,
    argument: usize,
) -> Result<usize, FsError> {
    // 两目标均为小端 LP64：三个 sockaddr、flags、metric 与机器字字段按固定偏移读取。
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

/// 名称只认 lo 和 eth0 的原字节，不增加地址别名或隐式设备。
fn interface_name(name: &[u8]) -> Result<InterfaceId, FsError> {
    let length = name
        .iter()
        .position(|byte| *byte == 0)
        .unwrap_or(name.len());
    InterfaceId::from_name(&name[..length])
}

/// 可选 rt_dev 在首个 NUL 停止，最多读取十五字节并保留零填内核终止位。
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

/// 只检查所选 IPv4 family 和地址，不验证没有配置含义的端口及 padding。
fn ipv4_sockaddr(bytes: &[u8], offset: usize) -> Result<[u8; 4], FsError> {
    if word(bytes, offset) != AF_INET {
        return Err(FsError::InvalidOperation);
    }
    Ok(address_bytes(bytes, offset))
}

/// 提取 sockaddr 内的四个网络序地址字节，不将其按目标主机整数解释。
fn address_bytes(bytes: &[u8], offset: usize) -> [u8; 4] {
    bytes[offset + 4..offset + 8]
        .try_into()
        .expect("fixed IPv4 address field")
}

/// 连续 IPv4 掩码转换为库 CIDR 前缀，零掩码和全一掩码分别为零及三十二。
fn mask_prefix(mask: [u8; 4]) -> Result<u8, FsError> {
    let prefix = u32::from_be_bytes(mask).leading_ones() as u8;
    if prefix_mask(prefix) != mask {
        return Err(FsError::InvalidOperation);
    }
    Ok(prefix)
}

/// 将库保证范围为零至三十二的 CIDR 前缀转换为网络序掩码。
fn prefix_mask(prefix: u8) -> [u8; 4] {
    let mask = if prefix == 0 {
        0
    } else {
        u32::MAX << (IPV4_ADDRESS_BITS - prefix)
    };
    mask.to_be_bytes()
}

/// 从已经完整复制的目标 ABI 结构中读取小端十六位字段。
fn word(bytes: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes(
        bytes[offset..offset + 2]
            .try_into()
            .expect("fixed ioctl word field"),
    )
}

/// 从已经完整复制的 LP64 ABI 结构中读取小端六十四位字段。
fn long(bytes: &[u8], offset: usize) -> u64 {
    u64::from_le_bytes(
        bytes[offset..offset + 8]
            .try_into()
            .expect("fixed ioctl long field"),
    )
}

/// 一次复制完整固定结构，普通 copy 错误统一为 EFAULT，部分输入不允许提交控制。
fn copyin<const N: usize>(mm: &Mm, argument: usize) -> Result<[u8; N], FsError> {
    let mut bytes = [0u8; N];
    let progress = mm.copyin(&mut bytes, argument);
    if progress.error.is_some() || progress.copied != N {
        return Err(FsError::BadAddress);
    }
    Ok(bytes)
}

/// 仅回写已初始化的完整 ifreq；部分输出按原 uaccess 错误返回。
fn copyout(mm: &Mm, argument: usize, bytes: &[u8]) -> Result<usize, FsError> {
    let progress = mm.copyout(argument, bytes);
    if progress.error.is_some() || progress.copied != bytes.len() {
        return Err(FsError::BadAddress);
    }
    Ok(0)
}
