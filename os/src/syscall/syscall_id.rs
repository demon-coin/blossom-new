//! 系统调用编号常量表（历史参考模块）。
//!
//! # 模块职责与架构定位
//! 本文件按 Linux 通用系统调用表（`asm-generic/unistd.h`，RISC-V 64 与 LoongArch 64 共用）定义了早期原型阶段的系统调用编号常量：
//! - **模块状态**：当前活跃内核已直接在 `syscall::dispatch` 中按标准 `asm-generic/unistd.h` 编号进行匹配路由，本文件作为历史兼容常量表保留。
//! - **历史命名差异注意**：本表保留了早期实验阶段个别与标准 Linux UAPI 存在偏差的历史常量命名（例如 `SYSCALL_DUP3 = 20` 在标准 Linux 中实际为 `epoll_create1`，`SYSCALL_DUP2 = 24` 在标准 Linux 中实际为 `dup3`），以及末尾编号 `>= 500` 的教学自定义扩展系统调用号。
//!
//! # 核心设计：编号区间划分概览
//! ```text
//!   +-------------------+----------------------------------------------------+
//!   | 编号区间          | 功能分类                                           |
//!   +-------------------+----------------------------------------------------+
//!   |  17 ..=  88       | 文件系统、目录、描述符、管道、I/O 与多路复用        |
//!   |  93 ..= 100       | 线程/进程退出、TID 地址绑定、Futex 与健壮锁链表    |
//!   | 101 ..= 124       | 睡眠、间隔定时器、POSIX 时钟、内核日志与主动让出   |
//!   | 129 ..= 139       | 信号发送、信号处理动作、信号掩码与信号栈恢复       |
//!   | 153 ..= 179       | 进程组/会话、系统信息、资源统计与 UID/GID/PID 查询 |
//!   | 198 ..= 210       | BSD 套接字网络通信接口                             |
//!   | 213 ..= 291, 439  | 虚拟内存管理、进程克隆/执行/等待、随机数与扩展属性 |
//!   | 500 ..= 1690      | 早期教学实验自定义非标准系统调用（非 Linux ABI）   |
//!   +-------------------+----------------------------------------------------+
//! ```
//!
//! # 锁获取顺序与并发契约
//! 本文件仅包含编译期 `usize` 常量，不涉及任何锁或运行时状态。

/// 获取当前工作目录绝对路径（`getcwd`，编号 17）。
pub const SYSCALL_GETCWD: usize = 17;
/// 历史常量名：编号 20（早期实验误标为 `dup3`，标准 Linux `asm-generic` 中 20 为 `epoll_create1`）。
pub const SYSCALL_DUP3: usize = 20;
/// 复制文件描述符到最小可用编号（`dup`，编号 23）。
pub const SYSCALL_DUP: usize = 23;
/// 历史常量名：复制文件描述符到指定编号（编号 24，标准 Linux `asm-generic` 中名称为 `dup3`）。
pub const SYSCALL_DUP2: usize = 24;
/// 文件描述符控制与状态标志操作（`fcntl`，编号 25）。
pub const SYSCALL_FCNTL: usize = 25;
/// 设备专属输入/输出控制命令（`ioctl`，编号 29）。
pub const SYSCALL_IOCTL: usize = 29;
/// 相对于目录文件描述符创建子目录（`mkdirat`，编号 34）。
pub const SYSCALL_MKDIRAT: usize = 34;
/// 相对于目录文件描述符删除目录项或空目录（`unlinkat`，编号 35）。
pub const SYSCALL_UNLINKAT: usize = 35;
/// 相对于目录文件描述符创建硬链接（`linkat`，编号 37）。
pub const SYSCALL_LINKAT: usize = 37;
/// 卸载已挂载的文件系统（`umount2`，编号 39）。
pub const SYSCALL_UMOUNT2: usize = 39;
/// 挂载文件系统到目标目录（`mount`，编号 40）。
pub const SYSCALL_MOUNT: usize = 40;
/// 按路径查询挂载文件系统的容量与块统计信息（`statfs`，编号 43）。
pub const SYSCALL_STATFS: usize = 43;
/// 将已打开文件截断或扩展至指定字节长度（`ftruncate`，编号 46）。
pub const SYSCALL_FTRUNCATE: usize = 46;
/// 相对于目录文件描述符检查文件访问权限（`faccessat`，编号 48）。
pub const SYSCALL_FACCESSAT: usize = 48;
/// 切换当前进程的工作目录（`chdir`，编号 49）。
pub const SYSCALL_CHDIR: usize = 49;
/// 相对于目录文件描述符修改文件权限模式位（`fchmodat`，编号 53）。
pub const SYSCALL_FCHMODAT: usize = 53;
/// 相对于目录文件描述符打开或创建文件（`openat`，编号 56）。
pub const SYSCALL_OPENAT: usize = 56;
/// 关闭指定的文件描述符（`close`，编号 57）。
pub const SYSCALL_CLOSE: usize = 57;
/// 创建带标志位的单向匿名管道文件描述符对（`pipe2`，编号 59）。
pub const SYSCALL_PIPE2: usize = 59;
/// 读取目录流中的 64 位目录项数组（`getdents64`，编号 61）。
pub const SYSCALL_GETDENTS64: usize = 61;
/// 重定位打开文件的读写字节偏移量（`lseek`，编号 62）。
pub const SYSCALL_LSEEK: usize = 62;
/// 从文件描述符当前偏移量读取字节流（`read`，编号 63）。
pub const SYSCALL_READ: usize = 63;
/// 向文件描述符当前偏移量写入字节流（`write`，编号 64）。
pub const SYSCALL_WRITE: usize = 64;
/// 从文件描述符分散读取数据到多个缓冲区（`readv`，编号 65）。
pub const SYSCALL_READV: usize = 65;
/// 将多个缓冲区的数据聚集写入文件描述符（`writev`，编号 66）。
pub const SYSCALL_WRITEV: usize = 66;
/// 从文件指定偏移量读取数据且不修改文件当前偏移（`pread64`，编号 67）。
pub const SYSCALL_PREAD: usize = 67;
/// 向文件指定偏移量写入数据且不修改文件当前偏移（`pwrite64`，编号 68）。
pub const SYSCALL_PWRITE: usize = 68;
/// 在两个文件描述符之间内核态直传数据（`sendfile`，编号 71）。
pub const SYSCALL_SENDFILE: usize = 71;
/// 带微秒/纳秒精度超时与原子信号掩码的多路复用等待（`pselect6`，编号 72）。
pub const SYSCALL_PSELECT6: usize = 72;
/// 带纳秒精度超时与原子信号掩码的描述符数组轮询（`ppoll`，编号 73）。
pub const SYSCALL_PPOLL: usize = 73;
/// 在管道与文件描述符之间零拷贝搬运数据（`splice`，编号 76）。
pub const SYSCALL_SPLICE: usize = 76;
/// 读取符号链接本身指向的目标路径字符串（`readlinkat`，编号 78）。
pub const SYSCALL_READLINKAT: usize = 78;
/// 相对于目录文件描述符查询文件元数据（`newfstatat`，编号 79）。
pub const SYSCALL_FSTATAT: usize = 79;
/// 通过已打开文件描述符查询文件元数据（`fstat`，编号 80）。
pub const SYSCALL_FSTAT: usize = 80;
/// 将文件脏数据与元数据同步刷回持久存储设备（`fsync`，编号 82）。
pub const SYSCALL_FSYNC: usize = 82;
/// 以纳秒精度更新文件的访问时间与修改时间（`utimensat`，编号 88）。
pub const SYSCALL_UTIMENSAT: usize = 88;

/// 终止当前调用线程（`exit`，编号 93）。
pub const SYSCALL_EXIT: usize = 93;
/// 终止当前进程（线程组）中的所有线程（`exit_group`，编号 94）。
pub const SYSCALL_EXIT_GROUP: usize = 94;
/// 设置当前线程退出时清零并触发 Futex 唤醒的用户态 TID 指针（`set_tid_address`，编号 96）。
pub const SYSCALL_SET_TID_ADDRESS: usize = 96;
/// 快速用户态互斥锁等待与唤醒操作（`futex`，编号 98）。
pub const SYSCALL_FUTEX: usize = 98;
/// 注册当前线程的健壮互斥锁（Robust Futex）链表头（`set_robust_list`，编号 99）。
pub const SYSCALL_SET_ROBUST_LIST: usize = 99;
/// 查询指定线程的健壮互斥锁链表头地址（`get_robust_list`，编号 100）。
pub const SYSCALL_GET_ROBUST_LIST: usize = 100;

/// 使当前线程按相对时长高精度睡眠（`nanosleep`，编号 101）。
pub const SYSCALL_NANOSLEEP: usize = 101;
/// 获取进程间隔定时器的当前剩余时间与周期（`getitimer`，编号 102）。
pub const SYSCALL_GETITIMER: usize = 102;
/// 设置或取消进程间隔定时器（`setitimer`，编号 103）。
pub const SYSCALL_SETITIMER: usize = 103;
/// 读取指定 POSIX 时钟源的当前时间戳（`clock_gettime`，编号 113）。
pub const SYSCALL_CLOCK_GETTIME: usize = 113;
/// 指定时钟源执行相对或绝对截止时间睡眠（`clock_nanosleep`，编号 115）。
pub const SYSCALL_CLOCK_NANOSLEEP: usize = 115;
/// 读取或控制内核日志环形缓冲区（`syslog`，编号 116）。
pub const SYSCALL_SYSLOG: usize = 116;
/// 主动让出当前 CPU 时间片给其他就绪任务（`sched_yield`，编号 124）。
pub const SYSCALL_YIELD: usize = 124;

/// 向指定进程或进程组发送信号（`kill`，编号 129）。
pub const SYSCALL_KILL: usize = 129;
/// 向指定线程 ID 发送信号（`tkill`，编号 130）。
pub const SYSCALL_TKILL: usize = 130;
/// 向指定线程组内的特定线程发送信号（`tgkill`，编号 131）。
pub const SYSCALL_TGKILL: usize = 131;
/// 查询或安装指定信号的处理函数与标志（`rt_sigaction`，编号 134）。
pub const SYSCALL_SIGACTION: usize = 134;
/// 查询或修改当前线程的信号屏蔽字（`rt_sigprocmask`，编号 135）。
pub const SYSCALL_SIGPROCMASK: usize = 135;
/// 在给定时限内同步等待指定集合中的待处理信号（`rt_sigtimedwait`，编号 137）。
pub const SYSCALL_SIGTIMEDWAIT: usize = 137;
/// 从用户态信号处理函数返回并恢复原陷阱帧与信号掩码（`rt_sigreturn`，编号 139）。
pub const SYSCALL_SIGRETURN: usize = 139;

/// 查询进程及其已回收子进程的用户态/内核态 CPU 时钟滴答统计（`times`，编号 153）。
pub const SYSCALL_TIMES: usize = 153;
/// 设置指定进程的进程组 ID（`setpgid`，编号 154）。
pub const SYSCALL_SETPGID: usize = 154;
/// 查询指定进程的进程组 ID（`getpgid`，编号 155）。
pub const SYSCALL_GETPGID: usize = 155;
/// 创建新会话并使当前进程成为会话首进程与新进程组组长（`setsid`，编号 157）。
pub const SYSCALL_SETSID: usize = 157;
/// 获取内核名称、主机名、版本与硬件架构标识（`uname`，编号 160）。
pub const SYSCALL_UNAME: usize = 160;
/// 查询进程、线程或已回收子进程的资源使用统计（`getrusage`，编号 165）。
pub const SYSCALL_GETRUSAGE: usize = 165;
/// 设置并返回进程的文件模式创建屏蔽字（`umask`，编号 166）。
pub const SYSCALL_UMASK: usize = 166;
/// 获取微秒级实时日历时间（`gettimeofday`，编号 169）。
pub const SYSCALL_GET_TIME_OF_DAY: usize = 169;
/// 获取当前进程（线程组）ID（`getpid`，编号 172）。
pub const SYSCALL_GETPID: usize = 172;
/// 获取父进程 ID（`getppid`，编号 173）。
pub const SYSCALL_GETPPID: usize = 173;
/// 获取当前真实用户 ID（`getuid`，编号 174）。
pub const SYSCALL_GETUID: usize = 174;
/// 获取当前有效用户 ID（`geteuid`，编号 175）。
pub const SYSCALL_GETEUID: usize = 175;
/// 获取当前真实组 ID（`getgid`，编号 176）。
pub const SYSCALL_GETGID: usize = 176;
/// 获取当前有效组 ID（`getegid`，编号 177）。
pub const SYSCALL_GETEGID: usize = 177;
/// 获取当前调用线程自身的内核线程 ID（`gettid`，编号 178）。
pub const SYSCALL_GETTID: usize = 178;
/// 查询系统运行时间、内存总量与进程数量概览（`sysinfo`，编号 179）。
pub const SYSCALL_SYSINFO: usize = 179;

/// 创建网络通信套接字端点（`socket`，编号 198）。
pub const SYSCALL_SOCKET: usize = 198;
/// 创建一对相互连接的本地匿名套接字（`socketpair`，编号 199）。
pub const SYSCALL_SOCKETPAIR: usize = 199;
/// 将本地协议地址与端口绑定到套接字（`bind`，编号 200）。
pub const SYSCALL_BIND: usize = 200;
/// 将流式套接字切换为被动监听连接请求状态（`listen`，编号 201）。
pub const SYSCALL_LISTEN: usize = 201;
/// 从监听套接字队列中接受一个已建立的客户端连接（`accept`，编号 202）。
pub const SYSCALL_ACCEPT: usize = 202;
/// 向指定远端地址发起连接或设置默认发送对端（`connect`，编号 203）。
pub const SYSCALL_CONNECT: usize = 203;
/// 查询套接字当前绑定的本地协议地址（`getsockname`，编号 204）。
pub const SYSCALL_GETSOCKNAME: usize = 204;
/// 查询已连接套接字的远端对等方协议地址（`getpeername`，编号 205）。
pub const SYSCALL_GETPEERNAME: usize = 205;
/// 向套接字（及可选目标地址）发送数据（`sendto`，编号 206）。
pub const SYSCALL_SENDTO: usize = 206;
/// 从套接字接收数据并可选获取发送方地址（`recvfrom`，编号 207）。
pub const SYSCALL_RECVFROM: usize = 207;
/// 设置套接字或传输层协议控制选项（`setsockopt`，编号 208）。
pub const SYSCALL_SETSOCKOPT: usize = 208;
/// 读取套接字或传输层协议控制选项及待处理错误（`getsockopt`，编号 209）。
pub const SYSCALL_GETSOCKOPT: usize = 209;
/// 单向或双向关闭全双工套接字连接的数据读写通道（`shutdown`，编号 210）。
pub const SYSCALL_SOCK_SHUTDOWN: usize = 210;

/// 早期实验保留的增量堆扩展调用号（`sbrk`，编号 213；非标准 Linux 编号）。
pub const SYSCALL_SBRK: usize = 213;
/// 查询或调整进程数据段堆顶（program break）地址（`brk`，编号 214）。
pub const SYSCALL_BRK: usize = 214;
/// 解除指定虚拟地址区间的内存映射（`munmap`，编号 215）。
pub const SYSCALL_MUNMAP: usize = 215;
/// 创建子进程或共享资源的执行线程（`clone`，编号 220；用户态 `fork()` 通常封装为 `clone(SIGCHLD, 0)`）。
pub const SYSCALL_CLONE: usize = 220;
/// 加载并执行新程序映像以替换当前进程地址空间（`execve`，编号 221）。
pub const SYSCALL_EXECVE: usize = 221;
/// 在进程虚拟地址空间中建立匿名或文件后备内存映射（`mmap`，编号 222）。
pub const SYSCALL_MMAP: usize = 222;
/// 修改已映射虚拟内存页区间的读/写/执行访问保护权限（`mprotect`，编号 226）。
pub const SYSCALL_MPROTECT: usize = 226;
/// 将文件映射内存区间中的脏页同步刷回底层文件（`msync`，编号 227）。
pub const SYSCALL_MSYNC: usize = 227;
/// 向内核提供虚拟内存区间的访问模式提示建议（`madvise`，编号 233）。
pub const SYSCALL_MADVISE: usize = 233;
/// 等待子进程状态变化并回收其资源使用统计（`wait4`，编号 260；用户态 `wait()` 封装自此调用）。
pub const SYSCALL_WAIT4: usize = 260;
/// 查询并原子设置指定进程的资源软硬限制（`prlimit64`，编号 261）。
pub const SYSCALL_PRLIMIT: usize = 261;
/// 相对于目录文件描述符并带扩展标志重命名或移动文件（`renameat2`，编号 276）。
pub const SYSCALL_RENAMEAT2: usize = 276;
/// 向用户态缓冲区填充随机字节序列（`getrandom`，编号 278）。
pub const SYSCALL_GETRANDOM: usize = 278;
/// 在多线程间执行跨核内存屏障同步（`membarrier`，编号 283）。
pub const SYSCALL_MEMBARRIER: usize = 283;
/// 在两个普通文件的指定偏移区间之间直接拷贝数据（`copy_file_range`，编号 285）。
pub const SYSCALL_COPY_FILE_RANGE: usize = 285;
/// 按掩码按需查询扩展文件元数据属性（`statx`，编号 291）。
pub const SYSCALL_STATX: usize = 291;
/// 带显式标志位（如 `AT_EACCESS`、`AT_SYMLINK_NOFOLLOW`）的文件访问权限检查（`faccessat2`，编号 439）。
pub const SYSCALL_FACCESSAT2: usize = 439;

// ---- 以下为早期教学实验定义的非标准自定义系统调用编号（不属于标准 Linux POSIX ABI） ----

/// 非标准教学系统调用：列出目录内容（编号 500）。
pub const SYSCALL_LS: usize = 500;
/// 非标准教学系统调用：直接请求机器关机（编号 501）。
pub const SYSCALL_SHUTDOWN: usize = 501;
/// 非标准教学系统调用：清空终端屏幕（编号 502）。
pub const SYSCALL_CLEAR: usize = 502;
/// 非标准教学系统调用：早期无目录基准的路径打开接口（编号 506）。
pub const SYSCALL_OPEN: usize = 506;
/// 非标准教学系统调用：早期毫秒级获取时间接口（编号 1690，区别于标准 169 号 `gettimeofday`）。
pub const SYSCALL_GET_TIME: usize = 1690;
