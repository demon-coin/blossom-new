//! 终端属性（`termios`）与窗口尺寸（`winsize`）的 UAPI 编解码、位域校验及串口参数换算。
//!
//! # 模块职责与架构定位
//! 本模块位于 TTY 子系统的协议与配置层，纯函数式地完成用户态 ABI 字节数组与内核结构体的双向转换，不触碰任何硬件寄存器或锁。
//! - **上游调用者**：`crate::fs::tty`（处理 `TCGETS`、`TCSETS*`、`TIOCGWINSZ`、`TIOCSWINSZ`、`TIOCGPGRP`/`TIOCSPGRP` 等 `ioctl`）以及 `crate::fs::tty::ntty`（读取标志位与控制字符下标）。
//! - **下游依赖**：`crate::console` 提供的 8250/16550 UART 线路控制寄存器（LCR）位常量与 `SerialConfig`。
//!
//! # 核心数据结构与二进制布局
//!
//! ## 1. `termios` 用户态 36 字节小端布局（`TERMIOS_BYTES = 36`）
//! ```text
//! 字节偏移   0..4        4..8        8..12       12..16      16     17..36 (共 19 字节)
//!         ┌───────────┬───────────┬───────────┬───────────┬───────┬───────────────────┐
//!         │ c_iflag   │ c_oflag   │ c_cflag   │ c_lflag   │c_line │   c_cc[0..19]     │
//!         │ (u32 LE)  │ (u32 LE)  │ (u32 LE)  │ (u32 LE)  │ (u8)  │   (控制字符数组)   │
//!         └───────────┴───────────┴───────────┴───────────┴───────┴───────────────────┘
//! ```
//!
//! ## 2. `c_cflag` 控制模式标志位域布局
//! ```text
//!  Bit 31   Bit 30   28..16 (CIBAUD)     12      11     10      9       8       7      6     5..4    3..0
//! ┌───────┬────────┬─────────────────┬────────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┬────────┐
//! │CRTSCTS│ CMSPAR │ 输入波特率编码   │CBAUDEX │CLOCAL│HUPCL │PARODD│PARENB│CREAD │CSTOPB│CSIZE │ CBAUD  │
//! └───────┴────────┴─────────────────┴────────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┴────────┘
//! ```
//!
//! ## 3. `winsize` 终端窗口尺寸 8 字节小端布局（`WINSIZE_BYTES = 8`）
//! ```text
//! 字节偏移   0..2        2..4        4..6        6..8
//!         ┌───────────┬───────────┬───────────┬───────────┐
//!         │ ws_row    │ ws_col    │ ws_xpixel │ ws_ypixel │
//!         │ (u16 LE)  │ (u16 LE)  │ (u16 LE)  │ (u16 LE)  │
//!         └───────────┴───────────┴───────────┴───────────┘
//! ```
//!
//! # 锁与并发契约
//! 本模块全部为无状态纯函数，不持有或获取任何锁；调用方可在任意上下文中安全调用。

use core::convert::TryFrom;

use crate::console::{
    SerialConfig, UART_LCR_EPAR, UART_LCR_PARITY, UART_LCR_STOP, UART_LCR_WLEN5, UART_LCR_WLEN6,
    UART_LCR_WLEN7, UART_LCR_WLEN8,
};
use crate::fs::FsError;

/// 用户态 `struct termios` 的固定序列化字节长度（4 个 `u32` + 1 字节 `c_line` + 19 字节 `c_cc` = 36 字节）。
pub const TERMIOS_BYTES: usize = 4 * core::mem::size_of::<u32>() + 1 + NCCS;
/// `c_cc` 控制字符数组的固定槽位数（19 个控制字符）。
pub const NCCS: usize = 19;

/// `VINTR` 控制字符下标：产生 `SIGINT` 中断信号（默认 `^C` / `0x03`）。
pub const VINTR: usize = 0;
/// `VQUIT` 控制字符下标：产生 `SIGQUIT` 退出信号（默认 `^\` / `0x1c`）。
pub const VQUIT: usize = 1;
/// `VERASE` 控制字符下标：擦除前一个字符（默认 `DEL` / `0x7f`）。
pub const VERASE: usize = 2;
/// `VKILL` 控制字符下标：擦除当前整行输入（默认 `^U` / `0x15`）。
pub const VKILL: usize = 3;
/// `VEOF` 控制字符下标：提交当前输入行或产生文件结束（EOF，默认 `^D` / `0x04`）。
pub const VEOF: usize = 4;
/// `VTIME` 控制字符下标：非规范模式读取超时时间（单位：0.1 秒 / 100 毫秒）。
pub const VTIME: usize = 5;
/// `VMIN` 控制字符下标：非规范模式读取的最少字节数阈值。
pub const VMIN: usize = 6;
/// `VSWTC` 控制字符下标：流切换字符槽位（行规程不解释，仅在编解码中保留原值）。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 的第 7 控制字符槽，codec 原样往返但行规程不解释切换字符"
)]
pub const VSWTC: usize = 7;
/// `VSTART` 控制字符下标：软件流控恢复输出字符（XON，默认 `^Q` / `0x11`）。
pub const VSTART: usize = 8;
/// `VSTOP` 控制字符下标：软件流控暂停输出字符（XOFF，默认 `^S` / `0x13`）。
pub const VSTOP: usize = 9;
/// `VSUSP` 控制字符下标：产生 `SIGTSTP` 挂起信号（默认 `^Z` / `0x1a`）。
pub const VSUSP: usize = 10;
/// `VEOL` 控制字符下标：附加行结束符（默认禁用 `0`）。
pub const VEOL: usize = 11;
/// `VREPRINT` 控制字符下标：重新回显当前未提交的输入行（默认 `^R` / `0x12`）。
pub const VREPRINT: usize = 12;
/// `VDISCARD` 控制字符下标：切换 `FLUSHO` 输出丢弃状态（默认 `^O` / `0x0f`）。
pub const VDISCARD: usize = 13;
/// `VWERASE` 控制字符下标：擦除前一个单词（默认 `^W` / `0x17`）。
pub const VWERASE: usize = 14;
/// `VLNEXT` 控制字符下标：字面量转义下一个输入字符（Literal Next，默认 `^V` / `0x16`）。
pub const VLNEXT: usize = 15;
/// `VEOL2` 控制字符下标：第二附加行结束符（默认禁用 `0`）。
pub const VEOL2: usize = 16;

/// `c_iflag` 位：忽略接收到的线路 BREAK 状态。
pub const IGNBRK: u32 = 0x001;
/// `c_iflag` 位：将未忽略的线路 BREAK 转换为前台进程组的 `SIGINT` 信号。
pub const BRKINT: u32 = 0x002;
/// `c_iflag` 位：忽略奇偶校验错误与帧错误字节。
pub const IGNPAR: u32 = 0x004;
/// `c_iflag` 位：使用 `0xff 0x00 <byte>` 三字节序列标记奇偶校验错误、帧错误或 BREAK。
pub const PARMRK: u32 = 0x008;
/// `c_iflag` 位：启用输入奇偶校验检查。
pub const INPCK: u32 = 0x010;
/// `c_iflag` 位：将输入字节掩码截断为低 7 位（`byte & 0x7f`）。
pub const ISTRIP: u32 = 0x020;
/// `c_iflag` 位：将输入的换行符（NL, `\n`）转换为回车符（CR, `\r`）。
pub const INLCR: u32 = 0x040;
/// `c_iflag` 位：丢弃输入的回车符（CR, `\r`）。
pub const IGNCR: u32 = 0x080;
/// `c_iflag` 位：将输入的回车符（CR, `\r`）转换为换行符（NL, `\n`）。
pub const ICRNL: u32 = 0x100;
/// `c_iflag` 位：启用输出方向软件流控（识别输入的 `VSTOP`/`VSTART` 暂停或恢复发送）。
pub const IXON: u32 = 0x400;
/// `c_iflag` 位：允许任意输入字符解除由 `VSTOP` 触发的软件流控暂停状态。
pub const IXANY: u32 = 0x800;
/// `c_iflag` 位：输入方向软件流控位（当前不支持，位域校验时拒绝）。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 输入流控位，当前支持掩码拒绝 IXOFF"
)]
pub const IXOFF: u32 = 0x1000;
/// `c_iflag` 位：输入缓冲区满时向输出队列写入响铃字符（BEL, `0x07`）。
pub const IMAXBEL: u32 = 0x2000;
/// `c_iflag` 位：将输入按 UTF-8 编码处理（擦除字符与列宽计算时识别多字节延续字节）。
pub const IUTF8: u32 = 0x4000;
/// `c_iflag` 位：输入大写转小写（当前不支持，位域校验时拒绝）。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 输入大小写转换位，当前支持掩码拒绝 IUCLC"
)]
pub const IUCLC: u32 = 0x200;

/// `c_oflag` 位：启用输出后处理（Output Post-processing）。
pub const OPOST: u32 = 0x01;
/// `c_oflag` 位：输出小写转大写（当前不支持，位域校验时拒绝）。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 输出大小写转换位，当前支持掩码拒绝 OLCUC"
)]
pub const OLCUC: u32 = 0x02;
/// `c_oflag` 位：将输出的换行符（NL, `\n`）展开为回车加换行（`\r\n`）。
pub const ONLCR: u32 = 0x04;
/// `c_oflag` 位：将输出的回车符（CR, `\r`）转换为换行符（NL, `\n`）。
pub const OCRNL: u32 = 0x08;
/// `c_oflag` 位：当前处于第 0 列时不输出回车符（CR, `\r`）。
pub const ONOCR: u32 = 0x10;
/// `c_oflag` 位：输出换行符（NL, `\n`）时同时执行回车列归零语义。
pub const ONLRET: u32 = 0x20;
/// `c_oflag` 位：使用填充字符代替时间延迟（当前不支持，位域校验时拒绝）。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 延迟填充位，当前支持掩码拒绝 OFILL"
)]
pub const OFILL: u32 = 0x40;
/// `c_oflag` 位：填充字符使用 DEL 而非 NUL（当前不支持，位域校验时拒绝）。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 填充字符选择位，当前支持掩码拒绝 OFDEL"
)]
pub const OFDEL: u32 = 0x80;
/// `c_oflag` 位掩码：换行延迟掩码（当前不支持，位域校验时拒绝）。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 换行延迟掩码，当前支持掩码拒绝该延迟"
)]
pub const NLDLY: u32 = 0x100;
/// `c_oflag` 位掩码：回车延迟掩码（当前不支持，位域校验时拒绝）。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 回车延迟掩码，当前支持掩码拒绝该延迟"
)]
pub const CRDLY: u32 = 0x600;
/// `c_oflag` 位掩码：水平制表符（Tab）延迟与展开模式掩码。
pub const TABDLY: u32 = 0x1800;
/// `c_oflag` 值：制表符原样输出，不展开为空格。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 无制表延迟的零值，validate 直接接受该零位模式"
)]
pub const TAB0: u32 = 0x0000;
/// `c_oflag` 值：将制表符（`\t`）按 8 列对齐展开为空格。
pub const TAB3: u32 = 0x1800;
/// `c_oflag` 位掩码：退格延迟掩码（当前不支持，位域校验时拒绝）。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 退格延迟掩码，当前支持掩码拒绝该延迟"
)]
pub const BSDLY: u32 = 0x2000;
/// `c_oflag` 位掩码：垂直制表延迟掩码（当前不支持，位域校验时拒绝）。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 垂直制表延迟掩码，当前支持掩码拒绝该延迟"
)]
pub const VTDLY: u32 = 0x4000;
/// `c_oflag` 位掩码：换页延迟掩码（当前不支持，位域校验时拒绝）。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 换页延迟掩码，当前支持掩码拒绝该延迟"
)]
pub const FFDLY: u32 = 0x8000;

/// `c_cflag` 位掩码：输出波特率编码掩码（含 `CBAUDEX` 扩展位）。
pub const CBAUD: u32 = 0x0000_100f;
/// `c_cflag` 位：非标准自定义波特率标志（当前仅支持标准波特率表，拒绝 `BOTHER`）。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 自定义波特率编码，当前标准速率表不接受 BOTHER"
)]
pub const BOTHER: u32 = 0x0000_1000;

/// `c_cflag` 位掩码：每个字符的数据位长度掩码。
pub const CSIZE: u32 = 0x0000_0030;
/// `c_cflag` 值：5 位数据位。
pub const CS5: u32 = 0x0000_0000;
/// `c_cflag` 值：6 位数据位。
pub const CS6: u32 = 0x0000_0010;
/// `c_cflag` 值：7 位数据位。
pub const CS7: u32 = 0x0000_0020;
/// `c_cflag` 值：8 位数据位。
pub const CS8: u32 = 0x0000_0030;
/// `c_cflag` 位：使用 2 位停止位（清除时使用 1 位停止位）。
pub const CSTOPB: u32 = 0x0000_0040;
/// `c_cflag` 位：启用串口接收器（清除时硬件不接收新字符）。
pub const CREAD: u32 = 0x0000_0080;
/// `c_cflag` 位：启用输出产生与输入检查奇偶校验位。
pub const PARENB: u32 = 0x0000_0100;
/// `c_cflag` 位：使用奇校验（清除时使用偶校验）。
pub const PARODD: u32 = 0x0000_0200;
/// `c_cflag` 位：最后一个打开文件关闭时拉低调制解调器控制线（DTR/RTS）。
pub const HUPCL: u32 = 0x0000_0400;
/// `c_cflag` 位：忽略调制解调器状态线（本地直连模式）。
pub const CLOCAL: u32 = 0x0000_0800;

/// `c_cflag` 位掩码：输入波特率编码掩码（等于 `CBAUD << IBSHIFT`）。
pub const CIBAUD: u32 = CBAUD << IBSHIFT;
/// `c_cflag` 中输入波特率字段相对于输出波特率字段的左移位数。
pub const IBSHIFT: u32 = 16;
/// `c_cflag` 位：固定奇偶校验位（Mark/Space 校验，当前不支持，位域校验时拒绝）。
pub const CMSPAR: u32 = 0x4000_0000;
/// `c_cflag` 位：RTS/CTS 硬件流控（当前不支持，位域校验时拒绝）。
pub const CRTSCTS: u32 = 0x8000_0000;

/// `c_lflag` 位：启用信号字符解释（识别 `VINTR`、`VQUIT`、`VSUSP`）。
pub const ISIG: u32 = 0x0001;
/// `c_lflag` 位：启用规范模式（Canonical Mode，按行缓冲并支持行编辑）。
pub const ICANON: u32 = 0x0002;
/// `c_lflag` 位：规范模式大小写转义（当前不支持，位域校验时拒绝）。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 规范模式大小写位，当前支持掩码拒绝 XCASE"
)]
pub const XCASE: u32 = 0x0004;
/// `c_lflag` 位：启用输入字符回显。
pub const ECHO: u32 = 0x0008;
/// `c_lflag` 位：在规范模式下将 `VERASE` 和 `VWERASE` 回显为“退格-空格-退格”擦除序列。
pub const ECHOE: u32 = 0x0010;
/// `c_lflag` 位：在规范模式下处理 `VKILL` 删行后额外回显换行或擦除整行。
pub const ECHOK: u32 = 0x0020;
/// `c_lflag` 位：在规范模式下即使未设置 `ECHO`，也回显换行符（NL）。
pub const ECHONL: u32 = 0x0040;
/// `c_lflag` 位：因收到 `VINTR`、`VQUIT`、`VSUSP` 或 BREAK 产生信号时不清空输入/输出队列。
pub const NOFLSH: u32 = 0x0080;
/// `c_lflag` 位：后台进程组向控制终端写入时向其发送 `SIGTTOU` 信号。
pub const TOSTOP: u32 = 0x0100;
/// `c_lflag` 位：开启 `ECHO` 时将除 `\t`、`\n`、`VSTART`、`VSTOP` 外的 ASCII 控制字符回显为 `^X`。
pub const ECHOCTL: u32 = 0x0200;
/// `c_lflag` 位：硬拷贝终端擦除打印模式（当前不支持，位域校验时拒绝）。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 打印擦除位，当前支持掩码拒绝 ECHOPRT"
)]
pub const ECHOPRT: u32 = 0x0400;
/// `c_lflag` 位：开启 `ECHO` 和 `ECHOK`/`ECHOE` 时，将 `VKILL` 回显为逐列退格擦除整行。
pub const ECHOKE: u32 = 0x0800;
/// `c_lflag` 位：正在丢弃输出（由 `VDISCARD` 切换或用户态显式设置）。
pub const FLUSHO: u32 = 0x1000;
/// `c_lflag` 位：下次读取时重印待处理输入（当前不支持，位域校验时拒绝）。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 重处理输入位，当前支持掩码拒绝 PENDIN"
)]
pub const PENDIN: u32 = 0x4000;
/// `c_lflag` 位：启用实现定义的扩展输入处理（识别 `VLNEXT`、`VREPRINT`、`VWERASE`、`VDISCARD`、`VEOL2`）。
pub const IEXTEN: u32 = 0x8000;
/// `c_lflag` 位：外部行规程处理（当前不支持，位域校验时拒绝）。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 外部行规程位，当前支持掩码拒绝 EXTPROC"
)]
pub const EXTPROC: u32 = 0x1_0000;

/// `tcsetattr` 动作：立即应用新的 `termios` 配置（对应 `TCSETS`）。
pub const TCSANOW: u32 = 0;
/// `tcsetattr` 动作：等待所有已排队输出发送完毕后再应用配置（对应 `TCSETSW`）。
pub const TCSADRAIN: u32 = 1;
/// `tcsetattr` 动作：等待所有输出发送完毕，并丢弃未读输入后应用配置（对应 `TCSETSF`）。
pub const TCSAFLUSH: u32 = 2;

/// `TCFLSH` 选择器：丢弃已接收但尚未被用户读取的输入数据。
pub const TCIFLUSH: u32 = 0;
/// `TCFLSH` 选择器：丢弃已写入但尚未发送到硬件的输出数据。
pub const TCOFLUSH: u32 = 1;
/// `TCFLSH` 选择器：同时丢弃未读输入与未发送输出。
pub const TCIOFLUSH: u32 = 2;

/// `TCXONC` 动作：挂起本地输出传输。
pub const TCOOFF: u32 = 0;
/// `TCXONC` 动作：恢复被挂起的本地输出传输。
pub const TCOON: u32 = 1;
/// `TCXONC` 动作：立即向远端发送 `VSTOP` 字符，请求远端暂停发送数据。
pub const TCIOFF: u32 = 2;
/// `TCXONC` 动作：立即向远端发送 `VSTART` 字符，请求远端恢复发送数据。
pub const TCION: u32 = 3;

/// 标准波特率 `38400` 对应的 `CBAUD` 编码值。
pub const B38400: u32 = 0x0f;

/// 终端初始化默认的 `c_iflag`（启用回车转换行与软件流控 `ICRNL | IXON`）。
pub const DEFAULT_IFLAG: u32 = ICRNL | IXON;
/// 终端初始化默认的 `c_oflag`（启用输出后处理与换行展开 `OPOST | ONLCR`）。
pub const DEFAULT_OFLAG: u32 = OPOST | ONLCR;
/// 终端初始化默认的 `c_cflag`（38400 波特率、8 位数据位、启用接收、最后关闭挂断 `B38400 | CS8 | CREAD | HUPCL`）。
pub const DEFAULT_CFLAG: u32 = B38400 | CS8 | CREAD | HUPCL;
/// 终端初始化默认的 `c_lflag`（启用信号、规范模式、回显及扩展编辑）。
pub const DEFAULT_LFLAG: u32 = ISIG | ICANON | ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE | IEXTEN;
/// 终端初始化默认的 19 个控制字符数组（`c_cc`）。
pub const DEFAULT_CC: [u8; NCCS] = [
    3, 28, 127, 21, 4, 0, 1, 0, 17, 19, 26, 0, 18, 15, 23, 22, 0, 0, 0,
];

/// 内核支持的所有 `c_iflag` 位掩码并集。
pub const SUPPORTED_IFLAG: u32 = IGNBRK
    | BRKINT
    | IGNPAR
    | PARMRK
    | INPCK
    | ISTRIP
    | INLCR
    | IGNCR
    | ICRNL
    | IXON
    | IXANY
    | IMAXBEL
    | IUTF8;
/// 内核支持的所有 `c_oflag` 位掩码并集。
pub const SUPPORTED_OFLAG: u32 = OPOST | ONLCR | OCRNL | ONOCR | ONLRET | TABDLY;
/// 内核支持的所有 `c_lflag` 位掩码并集。
pub const SUPPORTED_LFLAG: u32 = ISIG
    | ICANON
    | ECHO
    | ECHOE
    | ECHOK
    | ECHONL
    | NOFLSH
    | TOSTOP
    | ECHOCTL
    | ECHOKE
    | FLUSHO
    | IEXTEN;

/// 标准 `CBAUD` 编码与标称波特率（单位：bps）的映射表；`0x1000` 以上为 `CBAUDEX` 扩展速率。
const STANDARD_BAUDS: [(u32, u32); 19] = [
    (0x0, 0),
    (0x1, 50),
    (0x2, 75),
    (0x3, 110),
    (0x4, 134),
    (0x5, 150),
    (0x6, 200),
    (0x7, 300),
    (0x8, 600),
    (0x9, 1200),
    (0xa, 1800),
    (0xb, 2400),
    (0xc, 4800),
    (0xd, 9600),
    (0xe, 19200),
    (B38400, 38400),
    (0x1001, 57600),
    (0x1002, 115200),
    (0x1003, 230400),
];

/// 内核态表示的 `termios` 终端配置结构体，与用户态 UAPI 36 字节布局一一对应。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Termios {
    /// 输入模式标志位（`c_iflag`）。
    pub iflag: u32,
    /// 输出模式标志位（`c_oflag`）。
    pub oflag: u32,
    /// 控制模式标志位（`c_cflag`，含波特率、数据位、停止位、校验位）。
    pub cflag: u32,
    /// 本地模式标志位（`c_lflag`，含规范模式、回显、信号解释）。
    pub lflag: u32,
    /// 行规程编号（`c_line`，仅支持 `N_TTY = 0`）。
    pub line: u8,
    /// 控制字符数组（`c_cc`，共 19 项，`0` 表示对应控制功能禁用）。
    pub cc: [u8; NCCS],
}

impl Default for Termios {
    /// 构造默认终端配置（`B38400 | CS8 | CREAD | HUPCL`，启用规范模式与回显）。
    fn default() -> Self {
        Self {
            iflag: DEFAULT_IFLAG,
            oflag: DEFAULT_OFLAG,
            cflag: DEFAULT_CFLAG,
            lflag: DEFAULT_LFLAG,
            line: 0,
            cc: DEFAULT_CC,
        }
    }
}

impl Termios {
    /// 从用户态传入的 36 字节小端字节数组解码 `Termios`，并校验所有标志位与波特率编码是否合法。
    pub fn from_bytes(bytes: [u8; TERMIOS_BYTES]) -> Result<Self, FsError> {
        let termios = Self {
            iflag: u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]),
            oflag: u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]),
            cflag: u32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]),
            lflag: u32::from_le_bytes([bytes[12], bytes[13], bytes[14], bytes[15]]),
            line: bytes[16],
            cc: [
                bytes[17], bytes[18], bytes[19], bytes[20], bytes[21], bytes[22], bytes[23],
                bytes[24], bytes[25], bytes[26], bytes[27], bytes[28], bytes[29], bytes[30],
                bytes[31], bytes[32], bytes[33], bytes[34], bytes[35],
            ],
        };

        termios.validate()?;
        Ok(termios)
    }

    /// 将 `Termios` 编码为 36 字节的用户态 ABI 小端字节数组。
    pub fn to_bytes(self) -> [u8; TERMIOS_BYTES] {
        let mut bytes = [0u8; TERMIOS_BYTES];
        bytes[0..4].copy_from_slice(&self.iflag.to_le_bytes());
        bytes[4..8].copy_from_slice(&self.oflag.to_le_bytes());
        bytes[8..12].copy_from_slice(&self.cflag.to_le_bytes());
        bytes[12..16].copy_from_slice(&self.lflag.to_le_bytes());
        bytes[16] = self.line;
        bytes[17..36].copy_from_slice(&self.cc);
        bytes
    }

    /// 校验 `Termios` 各项标志位是否均在内核支持范围内；若包含不支持的位或非零行规程号则返回 `FsError::InvalidOperation`（`EINVAL`）。
    pub(crate) fn validate(&self) -> Result<(), FsError> {
        if self.line != 0 {
            return Err(FsError::InvalidOperation);
        }
        if self.iflag & !SUPPORTED_IFLAG != 0 {
            return Err(FsError::InvalidOperation);
        }
        if self.oflag & !SUPPORTED_OFLAG != 0 {
            return Err(FsError::InvalidOperation);
        }
        if self.lflag & !SUPPORTED_LFLAG != 0 {
            return Err(FsError::InvalidOperation);
        }
        if self.oflag & TABDLY != 0 && self.oflag & TABDLY != TAB3 {
            return Err(FsError::InvalidOperation);
        }
        validate_cflag(self.cflag)
    }

    /// 提取 `c_cflag` 中的输出波特率 `CBAUD` 编码。
    pub fn output_baud(self) -> u32 {
        self.cflag & CBAUD
    }
}

/// 校验 `c_cflag` 中的流控位、输出波特率、输入波特率（必须为 0 或与输出波特率一致）及未知保留位。
fn validate_cflag(cflag: u32) -> Result<(), FsError> {
    if cflag & (CRTSCTS | CMSPAR) != 0 {
        return Err(FsError::InvalidOperation);
    }

    let output = cflag & CBAUD;
    if baud_rate(output).is_none() {
        return Err(FsError::InvalidOperation);
    }
    let input = (cflag & CIBAUD) >> IBSHIFT;
    if input != 0 && input != output {
        return Err(FsError::InvalidOperation);
    }
    if input != 0 && baud_rate(input).is_none() {
        return Err(FsError::InvalidOperation);
    }

    let known = CSIZE | CSTOPB | CREAD | PARENB | PARODD | HUPCL | CLOCAL | CBAUD | CIBAUD;
    if cflag & !known != 0 {
        return Err(FsError::InvalidOperation);
    }
    Ok(())
}

/// 将 `CBAUD` 编码转换为标称波特率（单位：bps），未知编码返回 `None`。
pub fn baud_rate(code: u32) -> Option<u32> {
    STANDARD_BAUDS
        .iter()
        .find(|(value, _)| *value == code)
        .map(|(_, rate)| *rate)
}

/// 将标称波特率（单位：bps）转换为 `CBAUD` 编码，不在标准速率表中的波特率返回 `None`。
pub fn speed_for_baud(baud: u32) -> Option<u32> {
    STANDARD_BAUDS
        .iter()
        .find(|(_, rate)| *rate == baud)
        .map(|(code, _)| *code)
}

/// 根据串口基准时钟频率 `baud_base` 将标称波特率 `baud` 四舍五入换算为 16 位分频系数（Divisor Latch）。
///
/// 若 `baud` 或 `baud_base` 为 0，或计算出的分频系数超出 `1..=u16::MAX` 范围，则返回 `None`。
pub fn divisor_for_baud(baud_base: u32, baud: u32) -> Option<u16> {
    if baud == 0 || baud_base == 0 {
        return None;
    }

    let numerator = u64::from(baud_base);
    let denominator = u64::from(baud);
    let quotient = (numerator + denominator / 2) / denominator;
    if quotient == 0 || u64::from(u16::MAX) < quotient {
        return None;
    }
    Some(quotient as u16)
}

/// 将 `c_cflag` 中的数据位字长（`CSIZE`）、停止位（`CSTOPB`）与奇偶校验（`PARENB`/`PARODD`）转换为 8250 UART 线路控制寄存器（LCR）字节值。
pub fn line_control(cflag: u32) -> u8 {
    let mut value = match cflag & CSIZE {
        CS5 => UART_LCR_WLEN5,
        CS6 => UART_LCR_WLEN6,
        CS7 => UART_LCR_WLEN7,
        _ => UART_LCR_WLEN8,
    };
    if cflag & CSTOPB != 0 {
        value |= UART_LCR_STOP;
    }
    if cflag & PARENB != 0 {
        value |= UART_LCR_PARITY;
        if cflag & PARODD == 0 {
            value |= UART_LCR_EPAR;
        }
    }
    value
}

/// 根据已校验的 `Termios`、当前硬件分频系数及期望的 DTR/RTS 线状态构造 8250 串口硬件配置。
///
/// 当输出波特率设为 `B0`（0 bps）时，保留原硬件分频系数不变，但拉低 DTR 与 RTS 控制线以触发调制解调器挂断。
pub fn serial_config(
    termios: Termios,
    current_divisor: u16,
    dtr: bool,
    rts: bool,
) -> Option<SerialConfig> {
    let baud = baud_rate(termios.output_baud())?;
    let divisor = if baud == 0 {
        current_divisor
    } else {
        divisor_for_baud(crate::console::baud_base(), baud)?
    };

    let hung_up = baud == 0;
    Some(SerialConfig {
        divisor,
        line_control: line_control(termios.cflag),
        receiver: termios.cflag & CREAD != 0,
        dtr: dtr && !hung_up,
        rts: rts && !hung_up,
    })
}

/// 根据硬件分频系数 `divisor`、基准频率 `baud_base` 与 `c_cflag` 帧格式计算传输单个字符所需的纳秒数（向上取整）。
///
/// 单字符总位数 = 1 位起始位 + 数据位数（5..=8）+ 停止位数（1 或 2）+ 校验位数（0 或 1）。
/// 实际波特率 `speed = baud_base / divisor`，单帧耗时 $\lceil \text{bits} \times \text{divisor} \times 10^9 / \text{baud\_base} \rceil$ 纳秒。
pub fn character_nanos(divisor: u16, baud_base: u32, cflag: u32) -> Option<u64> {
    if divisor == 0 || baud_base == 0 {
        return None;
    }

    let mut bits = 2u64 + u64::from(char_size(cflag));
    if cflag & CSTOPB != 0 {
        bits += 1;
    }
    if cflag & PARENB != 0 {
        bits += 1;
    }

    let numerator = bits
        .checked_mul(u64::from(divisor))?
        .checked_mul(crate::time::NSEC_PER_SEC)?;
    let base = u64::from(baud_base);
    Some(numerator.div_ceil(base).max(1))
}

/// 返回 `c_cflag` 中每个字符的数据位位数（5、6、7 或 8 位）。
fn char_size(cflag: u32) -> u8 {
    match cflag & CSIZE {
        CS5 => 5,
        CS6 => 6,
        CS7 => 7,
        _ => 8,
    }
}

/// 用户态 `pid_t` 在 `TIOCGPGRP` / `TIOCSPGRP` / `TIOCGSID` 中的字节长度（4 字节有符号整数）。
pub const PID_BYTES: usize = core::mem::size_of::<i32>();
/// 用户态 `struct winsize` 在 `TIOCGWINSZ` / `TIOCSWINSZ` 中的字节长度（4 个 `u16`，共 8 字节）。
pub const WINSIZE_BYTES: usize = 4 * core::mem::size_of::<u16>();

/// 将 4 字节小端 `pid_t` 解码为非负进程组号；若为负数则返回 `FsError::InvalidOperation`。
pub fn decode_pid(bytes: [u8; PID_BYTES]) -> Result<usize, FsError> {
    let number = i32::from_le_bytes(bytes);
    if number < 0 {
        return Err(FsError::InvalidOperation);
    }
    Ok(number as usize)
}

/// 将内核进程号或进程组号编码为 4 字节小端 `pid_t` 数组。
pub fn encode_pid(number: usize) -> [u8; PID_BYTES] {
    i32::try_from(number)
        .expect("terminal pid exceeds pid_t")
        .to_le_bytes()
}

/// 终端窗口尺寸结构体（对应 UAPI `struct winsize`）。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Winsize {
    /// 窗口字符行数（`ws_row`）。
    pub rows: u16,
    /// 窗口字符列数（`ws_col`）。
    pub columns: u16,
    /// 窗口水平像素宽度（`ws_xpixel`）。
    pub xpixel: u16,
    /// 窗口垂直像素高度（`ws_ypixel`）。
    pub ypixel: u16,
}

impl Winsize {
    /// 返回串口终端的初始默认窗口尺寸（24 行 × 80 列，像素宽高为 0）。
    pub const fn initial() -> Self {
        Self {
            rows: 24,
            columns: 80,
            xpixel: 0,
            ypixel: 0,
        }
    }

    /// 从用户态 8 字节小端字节数组解码 `Winsize`。
    pub fn from_bytes(bytes: [u8; WINSIZE_BYTES]) -> Self {
        Self {
            rows: u16::from_le_bytes([bytes[0], bytes[1]]),
            columns: u16::from_le_bytes([bytes[2], bytes[3]]),
            xpixel: u16::from_le_bytes([bytes[4], bytes[5]]),
            ypixel: u16::from_le_bytes([bytes[6], bytes[7]]),
        }
    }

    /// 将 `Winsize` 编码为 8 字节的用户态小端字节数组。
    pub fn to_bytes(self) -> [u8; WINSIZE_BYTES] {
        let mut bytes = [0; WINSIZE_BYTES];
        bytes[0..2].copy_from_slice(&self.rows.to_le_bytes());
        bytes[2..4].copy_from_slice(&self.columns.to_le_bytes());
        bytes[4..6].copy_from_slice(&self.xpixel.to_le_bytes());
        bytes[6..8].copy_from_slice(&self.ypixel.to_le_bytes());
        bytes
    }
}
