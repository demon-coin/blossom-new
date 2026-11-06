//! Linux/POSIX 系统调用负数错误码常量与完整枚举定义（历史参考模块）。
//!
//! # 模块职责与架构定位
//! 本文件按 Linux 标准 UAPI（`asm-generic/errno-base.h` 与 `asm-generic/errno.h`）提供完整的带符号负数错误码常量（`EPERM = -1` 至 `EHWPOISON = -133`）及 `#[repr(isize)]` 枚举 [`Errno`]：
//! - **模块状态**：保留作早期原型及标准 POSIX/Linux 错误码编号速查参考；当前活跃内核已由同级模块 `syscall::error` 按实际用到的错误子集提供强类型转换。
//! - **别名约定**：由于 Rust `#[repr(isize)]` 枚举不允许两个变体拥有相同判别值，数值完全相同的 POSIX 同义错误码（如 `EWOULDBLOCK = EAGAIN = -11`、`EDEADLOCK = EDEADLK = -35`、`ENOTSUP = EOPNOTSUPP = -95`）仅在顶层 `pub const` 常量区提供别名，[`Errno`] 枚举内保留主名称变体。
//!
//! # 核心设计：系统调用返回值符号编码布局
//! 在 Linux RISC-V 64 与 LoongArch 64 系统调用 ABI 中，内核直接通过返回值寄存器 `a0` 区分成功结果与错误码，无需单独的进位标志位：
//!
//! ```text
//!   a0 寄存器返回值 (isize):
//!   +----------------------------------+----------------------------------+
//!   |  [-4095, -1]                     |  [0, isize::MAX]                 |
//!   |  系统调用失败：值为 -errno        |  系统调用成功：非负结果或 0      |
//!   |  (用户态 libc 将 -a0 存入 errno  |  (如 fd、字节数、PID、brk 地址)  |
//!   |   并向调用者返回 -1)              |                                  |
//!   +----------------------------------+----------------------------------+
//! ```
//!
//! # 锁获取顺序与并发契约
//! 本文件仅包含编译期常量与纯数据枚举定义，不持有任何锁或可变全局状态。

#![allow(unused)]

use num_enum::TryFromPrimitive;

/// 操作成功完成（`0`）。
pub const SUCCESS: isize = 0;
/// 操作不被允许，通常因缺少特权或所有权不匹配（`EPERM = -1`）。
pub const EPERM: isize = -1;
/// 文件或目录不存在（`ENOENT = -2`）。
pub const ENOENT: isize = -2;
/// 指定的进程或线程不存在（`ESRCH = -3`）。
pub const ESRCH: isize = -3;
/// 阻塞的系统调用被信号中断（`EINTR = -4`）。
pub const EINTR: isize = -4;
/// 底层物理或逻辑输入/输出错误（`EIO = -5`）。
pub const EIO: isize = -5;
/// 设备不存在或请求的设备地址越界（`ENXIO = -6`）。
pub const ENXIO: isize = -6;
/// 参数列表与环境变量总长度超出上限（`E2BIG = -7`）。
pub const E2BIG: isize = -7;
/// 可执行文件格式不合法，如无效的 ELF 头部（`ENOEXEC = -8`）。
pub const ENOEXEC: isize = -8;
/// 文件描述符无效或访问模式不匹配（`EBADF = -9`）。
pub const EBADF: isize = -9;
/// 当前进程没有可等待的子进程（`ECHILD = -10`）。
pub const ECHILD: isize = -10;
/// 资源暂时不可用，请稍后重试（`EAGAIN = -11`）。
pub const EAGAIN: isize = -11;
/// 内存不足，内核分配或地址空间扩展失败（`ENOMEM = -12`）。
pub const ENOMEM: isize = -12;
/// 访问权限被拒绝（`EACCES = -13`）。
pub const EACCES: isize = -13;
/// 用户空间缓冲区地址非法或不可访问（`EFAULT = -14`）。
pub const EFAULT: isize = -14;
/// 操作要求块设备，但目标不是块设备（`ENOTBLK = -15`）。
pub const ENOTBLK: isize = -15;
/// 设备或资源正忙，无法立即独占或释放（`EBUSY = -16`）。
pub const EBUSY: isize = -16;
/// 目标文件、目录或映射已存在（`EEXIST = -17`）。
pub const EEXIST: isize = -17;
/// 尝试跨不同文件系统/设备建立硬链接或重命名（`EXDEV = -18`）。
pub const EXDEV: isize = -18;
/// 指定的设备不存在或不支持该操作（`ENODEV = -19`）。
pub const ENODEV: isize = -19;
/// 路径组件不是目录（`ENOTDIR = -20`）。
pub const ENOTDIR: isize = -20;
/// 目标是一个目录，无法执行仅限非目录的操作（`EISDIR = -21`）。
pub const EISDIR: isize = -21;
/// 传入的参数值、标志位或对齐方式无效（`EINVAL = -22`）。
pub const EINVAL: isize = -22;
/// 系统全局打开文件表已满（`ENFILE = -23`）。
pub const ENFILE: isize = -23;
/// 当前进程打开的文件描述符数量已达上限（`EMFILE = -24`）。
pub const EMFILE: isize = -24;
/// 对非终端设备发起了终端专属控制操作（`ENOTTY = -25`）。
pub const ENOTTY: isize = -25;
/// 可执行文件正在运行中无法修改，或正在被写入无法执行（`ETXTBSY = -26`）。
pub const ETXTBSY: isize = -26;
/// 文件大小超出文件系统或进程资源限制上限（`EFBIG = -27`）。
pub const EFBIG: isize = -27;
/// 存储设备上已无剩余空间或空闲 Inode（`ENOSPC = -28`）。
pub const ENOSPC: isize = -28;
/// 在管道、FIFO 或套接字上执行了非法的随机定位 `seek`（`ESPIPE = -29`）。
pub const ESPIPE: isize = -29;
/// 文件系统以只读方式挂载，禁止修改操作（`EROFS = -30`）。
pub const EROFS: isize = -30;
/// 单个文件的硬链接数量过多（`EMLINK = -31`）。
pub const EMLINK: isize = -31;
/// 管道或流式套接字读端已关闭，写入触发管道破裂（`EPIPE = -32`）。
pub const EPIPE: isize = -32;
/// 数学函数自变量超出定义域（`EDOM = -33`）。
pub const EDOM: isize = -33;
/// 计算结果或输出超出可表示范围或缓冲区容量（`ERANGE = -34`）。
pub const ERANGE: isize = -34;
/// 请求的锁操作会导致资源死锁（`EDEADLK = -35`）。
pub const EDEADLK: isize = -35;
/// 文件名或路径名过长（`ENAMETOOLONG = -36`）。
pub const ENAMETOOLONG: isize = -36;
/// 无可用的文件记录锁槽位（`ENOLCK = -37`）。
pub const ENOLCK: isize = -37;

/// 系统调用编号无效或未实现（`ENOSYS = -38`）。
///
/// 约定上仅当用户请求了不存在或当前内核未实现的系统调用入口时返回 `-ENOSYS`；
/// 已实现的系统调用若仅不支持某个特定标志或参数组合，通常应返回 `-EINVAL` 或 `-EOPNOTSUPP`，以便用户态区分“系统调用不存在”与“调用参数不支持”。
pub const ENOSYS: isize = -38;
/// 目标目录非空，无法删除或覆盖（`ENOTEMPTY = -39`）。
pub const ENOTEMPTY: isize = -39;
/// 解析路径时遇到的符号链接嵌套层数过多（`ELOOP = -40`）。
pub const ELOOP: isize = -40;
/// 非阻塞操作在当前状态下将会阻塞，数值与 [`EAGAIN`] 相同（`EWOULDBLOCK = -11`）。
pub const EWOULDBLOCK: isize = EAGAIN;
/// 消息队列中没有所请求类型的消息（`ENOMSG = -42`）。
pub const ENOMSG: isize = -42;
/// IPC 标识符已被移除（`EIDRM = -43`）。
pub const EIDRM: isize = -43;
/// 通道编号超出范围（`ECHRNG = -44`）。
pub const ECHRNG: isize = -44;
/// 第 2 层协议未同步（`EL2NSYNC = -45`）。
pub const EL2NSYNC: isize = -45;
/// 第 3 层协议已停止（`EL3HLT = -46`）。
pub const EL3HLT: isize = -46;
/// 第 3 层协议已复位（`EL3RST = -47`）。
pub const EL3RST: isize = -47;
/// 链路编号超出范围（`ELNRNG = -48`）。
pub const ELNRNG: isize = -48;
/// 协议驱动程序未挂载（`EUNATCH = -49`）。
pub const EUNATCH: isize = -49;
/// 无可用的 CSI 结构（`ENOCSI = -50`）。
pub const ENOCSI: isize = -50;
/// 第 2 层协议已停止（`EL2HLT = -51`）。
pub const EL2HLT: isize = -51;
/// 无效的交换操作（`EBADE = -52`）。
pub const EBADE: isize = -52;
/// 无效的请求描述符（`EBADR = -53`）。
pub const EBADR: isize = -53;
/// 交换表已满（`EXFULL = -54`）。
pub const EXFULL: isize = -54;
/// 无可用的 anode（`ENOANO = -55`）。
pub const ENOANO: isize = -55;
/// 无效的请求码（`EBADRQC = -56`）。
pub const EBADRQC: isize = -56;
/// 无效的槽位编号（`EBADSLT = -57`）。
pub const EBADSLT: isize = -57;
/// 资源死锁将会发生，数值与 [`EDEADLK`] 相同（`EDEADLOCK = -35`）。
pub const EDEADLOCK: isize = EDEADLK;
/// 字体文件格式损坏（`EBFONT = -59`）。
pub const EBFONT: isize = -59;
/// 目标设备不是字符流设备（`ENOSTR = -60`）。
pub const ENOSTR: isize = -60;
/// 无可用数据或扩展属性不存在（`ENODATA = -61`）。
pub const ENODATA: isize = -61;
/// 流操作定时器已超时（`ETIME = -62`）。
pub const ETIME: isize = -62;
/// 流资源已耗尽（`ENOSR = -63`）。
pub const ENOSR: isize = -63;
/// 主机未接入网络（`ENONET = -64`）。
pub const ENONET: isize = -64;
/// 所需的软件包未安装（`ENOPKG = -65`）。
pub const ENOPKG: isize = -65;
/// 目标对象位于远程节点上（`EREMOTE = -66`）。
pub const EREMOTE: isize = -66;
/// 通信链路已被切断（`ENOLINK = -67`）。
pub const ENOLINK: isize = -67;
/// RFS 广播通告错误（`EADV = -68`）。
pub const EADV: isize = -68;
/// RFS srmount 挂载错误（`ESRMNT = -69`）。
pub const ESRMNT: isize = -69;
/// 发送过程中发生通信错误（`ECOMM = -70`）。
pub const ECOMM: isize = -70;
/// 底层通信协议错误（`EPROTO = -71`）。
pub const EPROTO: isize = -71;
/// 尝试了不允许的多跳路由访问（`EMULTIHOP = -72`）。
pub const EMULTIHOP: isize = -72;
/// RFS 特定跨点解析错误（`EDOTDOT = -73`）。
pub const EDOTDOT: isize = -73;
/// 消息数据格式无效或校验失败（`EBADMSG = -74`）。
pub const EBADMSG: isize = -74;
/// 数值过大，无法装入目标数据类型字段（`EOVERFLOW = -75`）。
pub const EOVERFLOW: isize = -75;
/// 网络上的节点名称不唯一（`ENOTUNIQ = -76`）。
pub const ENOTUNIQ: isize = -76;
/// 文件描述符处于与当前操作不兼容的内部状态（`EBADFD = -77`）。
pub const EBADFD: isize = -77;
/// 远程主机地址已变更（`EREMCHG = -78`）。
pub const EREMCHG: isize = -78;
/// 无法访问所需的共享库（`ELIBACC = -79`）。
pub const ELIBACC: isize = -79;
/// 尝试加载已损坏的共享库（`ELIBBAD = -80`）。
pub const ELIBBAD: isize = -80;
/// 可执行文件中的 `.lib` 段已损坏（`ELIBSCN = -81`）。
pub const ELIBSCN: isize = -81;
/// 尝试链接的共享库数量超出上限（`ELIBMAX = -82`）。
pub const ELIBMAX: isize = -82;
/// 不能直接执行共享库文件（`ELIBEXEC = -83`）。
pub const ELIBEXEC: isize = -83;
/// 遇到非法的字符编码字节序列（`EILSEQ = -84`）。
pub const EILSEQ: isize = -84;
/// 被中断的系统调用应当重新启动（`ERESTART = -85`）。
pub const ERESTART: isize = -85;
/// 流管道错误（`ESTRPIPE = -86`）。
pub const ESTRPIPE: isize = -86;
/// 用户数量超出配额上限（`EUSERS = -87`）。
pub const EUSERS: isize = -87;
/// 在非套接字文件描述符上执行了套接字操作（`ENOTSOCK = -88`）。
pub const ENOTSOCK: isize = -88;
/// 发送操作缺少必需的目标地址（`EDESTADDRREQ = -89`）。
pub const EDESTADDRREQ: isize = -89;
/// 待发送的数据报长度超出协议最大限制（`EMSGSIZE = -90`）。
pub const EMSGSIZE: isize = -90;
/// 协议类型与请求的套接字类型不匹配（`EPROTOTYPE = -91`）。
pub const EPROTOTYPE: isize = -91;
/// 指定的协议选项不可用或不受支持（`ENOPROTOOPT = -92`）。
pub const ENOPROTOOPT: isize = -92;
/// 请求的协议不受支持（`EPROTONOSUPPORT = -93`）。
pub const EPROTONOSUPPORT: isize = -93;
/// 请求的套接字类型不受支持（`ESOCKTNOSUPPORT = -94`）。
pub const ESOCKTNOSUPPORT: isize = -94;
/// 传输端点或目标对象不支持该操作（`EOPNOTSUPP = -95`）。
pub const EOPNOTSUPP: isize = -95;
/// 操作不受支持，POSIX 标准别名，在 Linux 下数值与 [`EOPNOTSUPP`] 相同（`ENOTSUP = -95`）。
pub const ENOTSUP: isize = EOPNOTSUPP;
/// 请求的协议族不受支持（`EPFNOSUPPORT = -96`）。
pub const EPFNOSUPPORT: isize = -96;
/// 当前协议不支持指定的地址族（`EAFNOSUPPORT = -97`）。
pub const EAFNOSUPPORT: isize = -97;
/// 本地网络地址或端口已被占用（`EADDRINUSE = -98`）。
pub const EADDRINUSE: isize = -98;
/// 无法分配所请求的本地网络地址（`EADDRNOTAVAIL = -99`）。
pub const EADDRNOTAVAIL: isize = -99;
/// 本地网络接口已关闭（`ENETDOWN = -100`）。
pub const ENETDOWN: isize = -100;
/// 网络不可达，无可用路由（`ENETUNREACH = -101`）。
pub const ENETUNREACH: isize = -101;
/// 网络复位导致连接中断（`ENETRESET = -102`）。
pub const ENETRESET: isize = -102;
/// 本机软件主动中止了连接（`ECONNABORTED = -103`）。
pub const ECONNABORTED: isize = -103;
/// 连接被远端对等方发送 RST 复位（`ECONNRESET = -104`）。
pub const ECONNRESET: isize = -104;
/// 网络协议栈或套接字缓冲区空间不足（`ENOBUFS = -105`）。
pub const ENOBUFS: isize = -105;
/// 套接字已经处于已连接状态（`EISCONN = -106`）。
pub const EISCONN: isize = -106;
/// 套接字尚未建立连接（`ENOTCONN = -107`）。
pub const ENOTCONN: isize = -107;
/// 传输端点已执行关闭（shutdown），无法继续发送数据（`ESHUTDOWN = -108`）。
pub const ESHUTDOWN: isize = -108;
/// 引用过多，无法继续拼接（`ETOOMANYREFS = -109`）。
pub const ETOOMANYREFS: isize = -109;
/// 连接或等待操作超时（`ETIMEDOUT = -110`）。
pub const ETIMEDOUT: isize = -110;
/// 远端主机拒绝建立连接（`ECONNREFUSED = -111`）。
pub const ECONNREFUSED: isize = -111;
/// 目标主机已宕机（`EHOSTDOWN = -112`）。
pub const EHOSTDOWN: isize = -112;
/// 没有到达目标主机的路由（`EHOSTUNREACH = -113`）。
pub const EHOSTUNREACH: isize = -113;
/// 非阻塞对象上已有相同操作正在进行中（`EALREADY = -114`）。
pub const EALREADY: isize = -114;
/// 非阻塞操作已启动但尚未完成（`EINPROGRESS = -115`）。
pub const EINPROGRESS: isize = -115;
/// 文件句柄已失效（常见于网络文件系统 `ESTALE = -116`）。
pub const ESTALE: isize = -116;
/// 文件系统元数据结构需要一致性清理检查（`EUCLEAN = -117`）。
pub const EUCLEAN: isize = -117;
/// 不是 XENIX 命名类型文件（`ENOTNAM = -118`）。
pub const ENOTNAM: isize = -118;
/// 没有可用的 XENIX 信号量（`ENAVAIL = -119`）。
pub const ENAVAIL: isize = -119;
/// 目标是 XENIX 命名类型文件（`EISNAM = -120`）。
pub const EISNAM: isize = -120;
/// 远端设备发生输入/输出错误（`EREMOTEIO = -121`）。
pub const EREMOTEIO: isize = -121;
/// 超出磁盘配额限制（`EDQUOT = -122`）。
pub const EDQUOT: isize = -122;
/// 可移动设备中未检测到存储介质（`ENOMEDIUM = -123`）。
pub const ENOMEDIUM: isize = -123;
/// 存储介质类型不匹配（`EMEDIUMTYPE = -124`）。
pub const EMEDIUMTYPE: isize = -124;
/// 异步或挂起的操作已被取消（`ECANCELED = -125`）。
pub const ECANCELED: isize = -125;
/// 请求的密钥不存在或不可用（`ENOKEY = -126`）。
pub const ENOKEY: isize = -126;
/// 密钥已过期（`EKEYEXPIRED = -127`）。
pub const EKEYEXPIRED: isize = -127;
/// 密钥已被吊销（`EKEYREVOKED = -128`）。
pub const EKEYREVOKED: isize = -128;
/// 密钥被目标服务拒绝（`EKEYREJECTED = -129`）。
pub const EKEYREJECTED: isize = -129;
/// 健壮互斥锁（Robust Mutex）的上一个持有线程已异常退出（`EOWNERDEAD = -130`）。
pub const EOWNERDEAD: isize = -130;
/// 互斥锁或受保护对象的状态已无法恢复（`ENOTRECOVERABLE = -131`）。
pub const ENOTRECOVERABLE: isize = -131;
/// 由于射频硬件/软件开关（RF-kill）禁用，无线操作无法执行（`ERFKILL = -132`）。
pub const ERFKILL: isize = -132;
/// 物理内存页检测到不可纠正的硬件 ECC 错误（`EHWPOISON = -133`）。
pub const EHWPOISON: isize = -133;

/// 与 Linux 系统调用负数返回值直接一一对应的强类型错误码枚举。
///
/// 支持通过 `TryFromPrimitive` 从原始 `isize` 返回值反解出对应的枚举变体。
#[derive(Debug, Eq, PartialEq, TryFromPrimitive)]
#[repr(isize)]
pub enum Errno {
    /// 操作成功完成（`0`）。
    SUCCESS = 0,
    /// 操作不被允许（`-1`）。
    EPERM = -1,
    /// 文件或目录不存在（`-2`）。
    ENOENT = -2,
    /// 指定的进程或线程不存在（`-3`）。
    ESRCH = -3,
    /// 系统调用被信号中断（`-4`）。
    EINTR = -4,
    /// 输入/输出错误（`-5`）。
    EIO = -5,
    /// 设备不存在或地址越界（`-6`）。
    ENXIO = -6,
    /// 参数列表过长（`-7`）。
    E2BIG = -7,
    /// 可执行文件格式错误（`-8`）。
    ENOEXEC = -8,
    /// 文件描述符无效（`-9`）。
    EBADF = -9,
    /// 没有可等待的子进程（`-10`）。
    ECHILD = -10,
    /// 资源暂时不可用或操作将会阻塞（`-11`，同 `EWOULDBLOCK`）。
    EAGAIN = -11,
    /// 内存不足（`-12`）。
    ENOMEM = -12,
    /// 访问权限被拒绝（`-13`）。
    EACCES = -13,
    /// 用户空间地址非法（`-14`）。
    EFAULT = -14,
    /// 需要块设备（`-15`）。
    ENOTBLK = -15,
    /// 设备或资源正忙（`-16`）。
    EBUSY = -16,
    /// 文件或映射已存在（`-17`）。
    EEXIST = -17,
    /// 跨设备硬链接或重命名（`-18`）。
    EXDEV = -18,
    /// 设备不存在（`-19`）。
    ENODEV = -19,
    /// 不是目录（`-20`）。
    ENOTDIR = -20,
    /// 目标是目录（`-21`）。
    EISDIR = -21,
    /// 无效的参数（`-22`）。
    EINVAL = -22,
    /// 系统全局文件表溢出（`-23`）。
    ENFILE = -23,
    /// 进程打开文件数过多（`-24`）。
    EMFILE = -24,
    /// 不是终端设备（`-25`）。
    ENOTTY = -25,
    /// 可执行文件正忙（`-26`）。
    ETXTBSY = -26,
    /// 文件过大（`-27`）。
    EFBIG = -27,
    /// 设备上无剩余空间（`-28`）。
    ENOSPC = -28,
    /// 非法的随机定位 `seek`（`-29`）。
    ESPIPE = -29,
    /// 只读文件系统（`-30`）。
    EROFS = -30,
    /// 硬链接数量过多（`-31`）。
    EMLINK = -31,
    /// 管道破裂（`-32`）。
    EPIPE = -32,
    /// 数学参数超出定义域（`-33`）。
    EDOM = -33,
    /// 结果超出可表示范围（`-34`）。
    ERANGE = -34,
    /// 资源死锁将会发生（`-35`，同 `EDEADLOCK`）。
    EDEADLK = -35,
    /// 文件名过长（`-36`）。
    ENAMETOOLONG = -36,
    /// 无可用文件锁（`-37`）。
    ENOLCK = -37,
    /// 系统调用未实现（`-38`）。
    ENOSYS = -38,
    /// 目录非空（`-39`）。
    ENOTEMPTY = -39,
    /// 符号链接嵌套层数过多（`-40`）。
    ELOOP = -40,
    /// 无所请求类型的消息（`-42`）。
    ENOMSG = -42,
    /// IPC 标识符已移除（`-43`）。
    EIDRM = -43,
    /// 通道号超出范围（`-44`）。
    ECHRNG = -44,
    /// 第 2 层未同步（`-45`）。
    EL2NSYNC = -45,
    /// 第 3 层已停止（`-46`）。
    EL3HLT = -46,
    /// 第 3 层已复位（`-47`）。
    EL3RST = -47,
    /// 链路号超出范围（`-48`）。
    ELNRNG = -48,
    /// 协议驱动未挂载（`-49`）。
    EUNATCH = -49,
    /// 无可用的 CSI 结构（`-50`）。
    ENOCSI = -50,
    /// 第 2 层已停止（`-51`）。
    EL2HLT = -51,
    /// 无效的交换（`-52`）。
    EBADE = -52,
    /// 无效的请求描述符（`-53`）。
    EBADR = -53,
    /// 交换表已满（`-54`）。
    EXFULL = -54,
    /// 无可用的 anode（`-55`）。
    ENOANO = -55,
    /// 无效的请求码（`-56`）。
    EBADRQC = -56,
    /// 无效的槽位（`-57`）。
    EBADSLT = -57,
    /// 字体文件格式错误（`-59`）。
    EBFONT = -59,
    /// 设备不是流（`-60`）。
    ENOSTR = -60,
    /// 无可用数据（`-61`）。
    ENODATA = -61,
    /// 流定时器超时（`-62`）。
    ETIME = -62,
    /// 流资源耗尽（`-63`）。
    ENOSR = -63,
    /// 主机不在网络上（`-64`）。
    ENONET = -64,
    /// 软件包未安装（`-65`）。
    ENOPKG = -65,
    /// 对象位于远程节点（`-66`）。
    EREMOTE = -66,
    /// 通信链路已断开（`-67`）。
    ENOLINK = -67,
    /// 广播通告错误（`-68`）。
    EADV = -68,
    /// Srmount 挂载错误（`-69`）。
    ESRMNT = -69,
    /// 发送时发生通信错误（`-70`）。
    ECOMM = -70,
    /// 协议错误（`-71`）。
    EPROTO = -71,
    /// 尝试了多跳访问（`-72`）。
    EMULTIHOP = -72,
    /// RFS 特定错误（`-73`）。
    EDOTDOT = -73,
    /// 消息数据损坏或类型错误（`-74`）。
    EBADMSG = -74,
    /// 数值溢出目标类型范围（`-75`）。
    EOVERFLOW = -75,
    /// 网络名称不唯一（`-76`）。
    ENOTUNIQ = -76,
    /// 文件描述符状态异常（`-77`）。
    EBADFD = -77,
    /// 远程地址已改变（`-78`）。
    EREMCHG = -78,
    /// 无法访问所需的共享库（`-79`）。
    ELIBACC = -79,
    /// 共享库已损坏（`-80`）。
    ELIBBAD = -80,
    /// 可执行文件中的 `.lib` 段损坏（`-81`）。
    ELIBSCN = -81,
    /// 链接的共享库过多（`-82`）。
    ELIBMAX = -82,
    /// 不能直接执行共享库（`-83`）。
    ELIBEXEC = -83,
    /// 非法的字节序列（`-84`）。
    EILSEQ = -84,
    /// 被中断的系统调用需重启（`-85`）。
    ERESTART = -85,
    /// 流管道错误（`-86`）。
    ESTRPIPE = -86,
    /// 用户数过多（`-87`）。
    EUSERS = -87,
    /// 对非套接字执行了套接字操作（`-88`）。
    ENOTSOCK = -88,
    /// 需要提供目标地址（`-89`）。
    EDESTADDRREQ = -89,
    /// 消息过长（`-90`）。
    EMSGSIZE = -90,
    /// 套接字协议类型错误（`-91`）。
    EPROTOTYPE = -91,
    /// 协议选项不可用（`-92`）。
    ENOPROTOOPT = -92,
    /// 协议不受支持（`-93`）。
    EPROTONOSUPPORT = -93,
    /// 套接字类型不受支持（`-94`）。
    ESOCKTNOSUPPORT = -94,
    /// 传输端点不支持该操作（`-95`，同 `ENOTSUP`）。
    EOPNOTSUPP = -95,
    /// 协议族不受支持（`-96`）。
    EPFNOSUPPORT = -96,
    /// 协议不支持该地址族（`-97`）。
    EAFNOSUPPORT = -97,
    /// 地址已被占用（`-98`）。
    EADDRINUSE = -98,
    /// 无法分配请求的地址（`-99`）。
    EADDRNOTAVAIL = -99,
    /// 网络已关闭（`-100`）。
    ENETDOWN = -100,
    /// 网络不可达（`-101`）。
    ENETUNREACH = -101,
    /// 网络复位导致连接断开（`-102`）。
    ENETRESET = -102,
    /// 软件主动中止连接（`-103`）。
    ECONNABORTED = -103,
    /// 连接被对端复位（`-104`）。
    ECONNRESET = -104,
    /// 缓冲区空间不足（`-105`）。
    ENOBUFS = -105,
    /// 套接字已连接（`-106`）。
    EISCONN = -106,
    /// 套接字未连接（`-107`）。
    ENOTCONN = -107,
    /// 传输端点关闭后无法发送（`-108`）。
    ESHUTDOWN = -108,
    /// 引用过多无法拼接（`-109`）。
    ETOOMANYREFS = -109,
    /// 连接或操作超时（`-110`）。
    ETIMEDOUT = -110,
    /// 连接被拒绝（`-111`）。
    ECONNREFUSED = -111,
    /// 目标主机已关闭（`-112`）。
    EHOSTDOWN = -112,
    /// 无到达目标主机的路由（`-113`）。
    EHOSTUNREACH = -113,
    /// 操作已在进行中（`-114`）。
    EALREADY = -114,
    /// 操作正在进行中（`-115`）。
    EINPROGRESS = -115,
    /// 文件句柄已失效（`-116`）。
    ESTALE = -116,
    /// 元数据结构需要清理（`-117`）。
    EUCLEAN = -117,
    /// 不是 XENIX 命名类型文件（`-118`）。
    ENOTNAM = -118,
    /// 无可用的 XENIX 信号量（`-119`）。
    ENAVAIL = -119,
    /// 是 XENIX 命名类型文件（`-120`）。
    EISNAM = -120,
    /// 远程 I/O 错误（`-121`）。
    EREMOTEIO = -121,
    /// 磁盘配额超限（`-122`）。
    EDQUOT = -122,
    /// 未发现存储介质（`-123`）。
    ENOMEDIUM = -123,
    /// 存储介质类型错误（`-124`）。
    EMEDIUMTYPE = -124,
    /// 操作已取消（`-125`）。
    ECANCELED = -125,
    /// 所需密钥不可用（`-126`）。
    ENOKEY = -126,
    /// 密钥已过期（`-127`）。
    EKEYEXPIRED = -127,
    /// 密钥已被吊销（`-128`）。
    EKEYREVOKED = -128,
    /// 密钥被服务拒绝（`-129`）。
    EKEYREJECTED = -129,
    /// 健壮互斥锁前任持有者已死亡（`-130`）。
    EOWNERDEAD = -130,
    /// 状态无法恢复（`-131`）。
    ENOTRECOVERABLE = -131,
    /// 因 RF-kill 射频开关禁用而无法执行（`-132`）。
    ERFKILL = -132,
    /// 内存页发生硬件 ECC 错误（`-133`）。
    EHWPOISON = -133,
}
