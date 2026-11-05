//! 目标 termios 编解码与纯配置转换；本文件不触硬件或短锁。

use core::convert::TryFrom;

use crate::console::{
    SerialConfig, UART_LCR_EPAR, UART_LCR_PARITY, UART_LCR_STOP, UART_LCR_WLEN5, UART_LCR_WLEN6,
    UART_LCR_WLEN7, UART_LCR_WLEN8,
};
use crate::fs::FsError;

/// 目标 termios 固定 36 字节。
pub const TERMIOS_BYTES: usize = 4 * core::mem::size_of::<u32>() + 1 + NCCS;
/// 控制字符固定 19 个。
pub const NCCS: usize = 19;

/// VINTR 控制字符下标。
pub const VINTR: usize = 0;
/// VQUIT 控制字符下标。
pub const VQUIT: usize = 1;
/// VERASE 控制字符下标。
pub const VERASE: usize = 2;
/// VKILL 控制字符下标。
pub const VKILL: usize = 3;
/// VEOF 控制字符下标。
pub const VEOF: usize = 4;
/// VTIME 控制字符下标。
pub const VTIME: usize = 5;
/// VMIN 控制字符下标。
pub const VMIN: usize = 6;
/// VSWTC 控制字符下标。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 的第 7 控制字符槽，codec 原样往返但行规程不解释切换字符"
)]
pub const VSWTC: usize = 7;
/// VSTART 控制字符下标。
pub const VSTART: usize = 8;
/// VSTOP 控制字符下标。
pub const VSTOP: usize = 9;
/// VSUSP 控制字符下标。
pub const VSUSP: usize = 10;
/// VEOL 控制字符下标。
pub const VEOL: usize = 11;
/// VREPRINT 控制字符下标。
pub const VREPRINT: usize = 12;
/// VDISCARD 控制字符下标。
pub const VDISCARD: usize = 13;
/// VWERASE 控制字符下标。
pub const VWERASE: usize = 14;
/// VLNEXT 控制字符下标。
pub const VLNEXT: usize = 15;
/// VEOL2 控制字符下标。
pub const VEOL2: usize = 16;

/// IGNBRK 输入位。
pub const IGNBRK: u32 = 0x001;
/// BRKINT 输入位。
pub const BRKINT: u32 = 0x002;
/// IGNPAR 输入位。
pub const IGNPAR: u32 = 0x004;
/// PARMRK 输入位。
pub const PARMRK: u32 = 0x008;
/// INPCK 输入位。
pub const INPCK: u32 = 0x010;
/// ISTRIP 输入位。
pub const ISTRIP: u32 = 0x020;
/// INLCR 输入位。
pub const INLCR: u32 = 0x040;
/// IGNCR 输入位。
pub const IGNCR: u32 = 0x080;
/// ICRNL 输入位。
pub const ICRNL: u32 = 0x100;
/// IXON 输入位。
pub const IXON: u32 = 0x400;
/// IXANY 输入位。
pub const IXANY: u32 = 0x800;
/// IXOFF 输入位。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 输入流控位，当前支持掩码拒绝 IXOFF"
)]
pub const IXOFF: u32 = 0x1000;
/// IMAXBEL 输入位。
pub const IMAXBEL: u32 = 0x2000;
/// IUTF8 输入位。
pub const IUTF8: u32 = 0x4000;
/// IUCLC 输入位，未选择。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 输入大小写转换位，当前支持掩码拒绝 IUCLC"
)]
pub const IUCLC: u32 = 0x200;

/// OPOST 输出位。
pub const OPOST: u32 = 0x01;
/// OLCUC 输出位，未选择。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 输出大小写转换位，当前支持掩码拒绝 OLCUC"
)]
pub const OLCUC: u32 = 0x02;
/// ONLCR 输出位。
pub const ONLCR: u32 = 0x04;
/// OCRNL 输出位。
pub const OCRNL: u32 = 0x08;
/// ONOCR 输出位。
pub const ONOCR: u32 = 0x10;
/// ONLRET 输出位。
pub const ONLRET: u32 = 0x20;
/// OFILL 输出位，未选择。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 延迟填充位，当前支持掩码拒绝 OFILL"
)]
pub const OFILL: u32 = 0x40;
/// OFDEL 输出位，未选择。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 填充字符选择位，当前支持掩码拒绝 OFDEL"
)]
pub const OFDEL: u32 = 0x80;
/// NLDLY 延迟掩码，未选择。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 换行延迟掩码，当前支持掩码拒绝该延迟"
)]
pub const NLDLY: u32 = 0x100;
/// CRDLY 延迟掩码，未选择。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 回车延迟掩码，当前支持掩码拒绝该延迟"
)]
pub const CRDLY: u32 = 0x600;
/// TABDLY 延迟掩码。
pub const TABDLY: u32 = 0x1800;
/// TAB0 制表位。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 无制表延迟的零值，validate 直接接受该零位模式"
)]
pub const TAB0: u32 = 0x0000;
/// TAB3 制表位。
pub const TAB3: u32 = 0x1800;
/// BSDLY 延迟掩码，未选择。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 退格延迟掩码，当前支持掩码拒绝该延迟"
)]
pub const BSDLY: u32 = 0x2000;
/// VTDLY 延迟掩码，未选择。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 垂直制表延迟掩码，当前支持掩码拒绝该延迟"
)]
pub const VTDLY: u32 = 0x4000;
/// FFDLY 延迟掩码，未选择。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 换页延迟掩码，当前支持掩码拒绝该延迟"
)]
pub const FFDLY: u32 = 0x8000;

/// CBAUD 波特率掩码。
pub const CBAUD: u32 = 0x0000_100f;
/// BOTHER 自定义波特率标记。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 自定义波特率编码，当前标准速率表不接受 BOTHER"
)]
pub const BOTHER: u32 = 0x0000_1000;

/// CSIZE 字长掩码。
pub const CSIZE: u32 = 0x0000_0030;
/// CS5 字长。
pub const CS5: u32 = 0x0000_0000;
/// CS6 字长。
pub const CS6: u32 = 0x0000_0010;
/// CS7 字长。
pub const CS7: u32 = 0x0000_0020;
/// CS8 字长。
pub const CS8: u32 = 0x0000_0030;
/// CSTOPB 双停止位。
pub const CSTOPB: u32 = 0x0000_0040;
/// CREAD 接收使能。
pub const CREAD: u32 = 0x0000_0080;
/// PARENB 校验使能。
pub const PARENB: u32 = 0x0000_0100;
/// PARODD 奇校验。
pub const PARODD: u32 = 0x0000_0200;
/// HUPCL 挂断撤线。
pub const HUPCL: u32 = 0x0000_0400;
/// CLOCAL 本地连接。
pub const CLOCAL: u32 = 0x0000_0800;

/// CIBAUD 输入波特率掩码。
pub const CIBAUD: u32 = CBAUD << IBSHIFT;
/// IBSHIFT 输入波特率移位。
pub const IBSHIFT: u32 = 16;
/// CMSPAR 粘住校验，未选择。
pub const CMSPAR: u32 = 0x4000_0000;
/// CRTSCTS 硬件流控，未选择。
pub const CRTSCTS: u32 = 0x8000_0000;

/// ISIG 本地位。
pub const ISIG: u32 = 0x0001;
/// ICANON 本地位。
pub const ICANON: u32 = 0x0002;
/// XCASE 本地位，未选择。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 规范模式大小写位，当前支持掩码拒绝 XCASE"
)]
pub const XCASE: u32 = 0x0004;
/// ECHO 本地位。
pub const ECHO: u32 = 0x0008;
/// ECHOE 本地位。
pub const ECHOE: u32 = 0x0010;
/// ECHOK 本地位。
pub const ECHOK: u32 = 0x0020;
/// ECHONL 本地位。
pub const ECHONL: u32 = 0x0040;
/// NOFLSH 本地位。
pub const NOFLSH: u32 = 0x0080;
/// TOSTOP 后台输出检查位。
pub const TOSTOP: u32 = 0x0100;
/// ECHOCTL 本地位。
pub const ECHOCTL: u32 = 0x0200;
/// ECHOPRT 本地位，未选择。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 打印擦除位，当前支持掩码拒绝 ECHOPRT"
)]
pub const ECHOPRT: u32 = 0x0400;
/// ECHOKE 本地位。
pub const ECHOKE: u32 = 0x0800;
/// FLUSHO 本地位。
pub const FLUSHO: u32 = 0x1000;
/// PENDIN 本地位，未选择。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 重处理输入位，当前支持掩码拒绝 PENDIN"
)]
pub const PENDIN: u32 = 0x4000;
/// IEXTEN 本地位。
pub const IEXTEN: u32 = 0x8000;
/// EXTPROC 本地位，未选择。
#[expect(
    dead_code,
    reason = "保留 termios UAPI 外部行规程位，当前支持掩码拒绝 EXTPROC"
)]
pub const EXTPROC: u32 = 0x1_0000;

/// 立即提交 termios。
pub const TCSANOW: u32 = 0;
/// 排空输出后提交 termios。
pub const TCSADRAIN: u32 = 1;
/// 排空输出并清输入后提交 termios。
pub const TCSAFLUSH: u32 = 2;

/// 丢弃未读输入。
pub const TCIFLUSH: u32 = 0;
/// 丢弃未发输出。
pub const TCOFLUSH: u32 = 1;
/// 同时丢弃输入和输出。
pub const TCIOFLUSH: u32 = 2;

/// 暂停本地输出。
pub const TCOOFF: u32 = 0;
/// 恢复本地输出。
pub const TCOON: u32 = 1;
/// 向远端发送停止字符。
pub const TCIOFF: u32 = 2;
/// 向远端发送开始字符。
pub const TCION: u32 = 3;

/// 默认波特率的目标 CBAUD 编码。
pub const B38400: u32 = 0x0f;

/// 默认 termios 的 iflag。
pub const DEFAULT_IFLAG: u32 = ICRNL | IXON;
/// 默认 termios 的 oflag。
pub const DEFAULT_OFLAG: u32 = OPOST | ONLCR;
/// 默认 termios 的 cflag。
pub const DEFAULT_CFLAG: u32 = B38400 | CS8 | CREAD | HUPCL;
/// 默认 termios 的 lflag。
pub const DEFAULT_LFLAG: u32 = ISIG | ICANON | ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE | IEXTEN;
/// 默认 termios 的 19 个控制字符。
pub const DEFAULT_CC: [u8; NCCS] = [
    3, 28, 127, 21, 4, 0, 1, 0, 17, 19, 26, 0, 18, 15, 23, 22, 0, 0, 0,
];

/// 本批支持的全部 iflag 位。
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
/// 本批支持的全部 oflag 位。
pub const SUPPORTED_OFLAG: u32 = OPOST | ONLCR | OCRNL | ONOCR | ONLRET | TABDLY;
/// 本批支持的全部 lflag 位。
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

/// 标准 CBAUD 编码与名义波特率对照；高位为 CBAUDEX 扩展编码。
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

/// 目标 termios 结构，字段与 UAPI 四个 u32、line 与 19 个控制字符对应。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Termios {
    /// 输入模式标志。
    pub iflag: u32,
    /// 输出模式标志。
    pub oflag: u32,
    /// 控制模式标志。
    pub cflag: u32,
    /// 本地模式标志。
    pub lflag: u32,
    /// 线路规程号，仅 0。
    pub line: u8,
    /// 19 个控制字符。
    pub cc: [u8; NCCS],
}

impl Default for Termios {
    /// 返回 Linux 默认 B38400/CS8/CREAD/HUPCL 的 termios。
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
    /// 从目标 36 字节逐字节解码并完整验证，不接受半初始化结构。
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

    /// 编码为目标 36 字节，四字小端、line 与控制字符逐字节存放。
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

    /// 验证已解码 termios 的全部位域，未选择位返回真实 EINVAL。
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

    /// 返回输出波特率的 CBAUD 编码。
    pub fn output_baud(self) -> u32 {
        self.cflag & CBAUD
    }
}

/// 验证 cflag 的速度、字长、校验与流控位。
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

/// 把 CBAUD 编码转换为名义波特率，未知编码返回 None。
pub fn baud_rate(code: u32) -> Option<u32> {
    STANDARD_BAUDS
        .iter()
        .find(|(value, _)| *value == code)
        .map(|(_, rate)| *rate)
}

/// 把名义波特率转换为 CBAUD 编码，未列出的速率返回 None。
pub fn speed_for_baud(baud: u32) -> Option<u32> {
    STANDARD_BAUDS
        .iter()
        .find(|(_, rate)| *rate == baud)
        .map(|(code, _)| *code)
}

/// 把名义波特率转换为最接近的 16 位分频，无法表示时返回 None。
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

/// 把 termios 的字长、停止位与校验位转换为 8250 行控值。
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

/// 从已验证 termios 与当前分频构造串口寄存器目标，B0 保留旧分频。
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

/// 从回读分频与 baudbase 计算一个字符的纳秒数，向上取整不提前。
/// QEMU 实际波特率 speed=baudbase/divisor，帧纳秒=bits*divisor*1e9/baudbase。
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

/// 返回 cflag 字长的实际数据位数。
fn char_size(cflag: u32) -> u8 {
    match cflag & CSIZE {
        CS5 => 5,
        CS6 => 6,
        CS7 => 7,
        _ => 8,
    }
}

/// 两目标共同的 signed pid_t 字节数。
pub const PID_BYTES: usize = core::mem::size_of::<i32>();
/// 两目标共同的窗口尺寸字节数。
pub const WINSIZE_BYTES: usize = 4 * core::mem::size_of::<u16>();

/// 解码完整 pid_t，负进程组号码不能参与图查询。
pub fn decode_pid(bytes: [u8; PID_BYTES]) -> Result<usize, FsError> {
    let number = i32::from_le_bytes(bytes);
    if number < 0 {
        return Err(FsError::InvalidOperation);
    }
    Ok(number as usize)
}

/// 编码原号码域内的稳定 pid_t 数字。
pub fn encode_pid(number: usize) -> [u8; PID_BYTES] {
    i32::try_from(number)
        .expect("terminal pid exceeds pid_t")
        .to_le_bytes()
}

/// 窗口四字段共同提交，像素字段也属于变化判断。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Winsize {
    pub rows: u16,
    pub columns: u16,
    pub xpixel: u16,
    pub ypixel: u16,
}

impl Winsize {
    /// 固定串口默认窗口，不宣称宿主尺寸探测。
    pub const fn initial() -> Self {
        Self {
            rows: 24,
            columns: 80,
            xpixel: 0,
            ypixel: 0,
        }
    }

    /// 从完整八字节目标布局解码四个小端整数。
    pub fn from_bytes(bytes: [u8; WINSIZE_BYTES]) -> Self {
        Self {
            rows: u16::from_le_bytes([bytes[0], bytes[1]]),
            columns: u16::from_le_bytes([bytes[2], bytes[3]]),
            xpixel: u16::from_le_bytes([bytes[4], bytes[5]]),
            ypixel: u16::from_le_bytes([bytes[6], bytes[7]]),
        }
    }

    /// 初始化完整八字节输出，不裸复制 Rust 结构体。
    pub fn to_bytes(self) -> [u8; WINSIZE_BYTES] {
        let mut bytes = [0; WINSIZE_BYTES];
        bytes[0..2].copy_from_slice(&self.rows.to_le_bytes());
        bytes[2..4].copy_from_slice(&self.columns.to_le_bytes());
        bytes[4..6].copy_from_slice(&self.xpixel.to_le_bytes());
        bytes[6..8].copy_from_slice(&self.ypixel.to_le_bytes());
        bytes
    }
}
