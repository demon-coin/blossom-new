//! 固定 N_TTY 输入、回显与输出算法；本文件只操作调用者传入的短锁状态。

use super::termios::{
    Termios, BRKINT, ECHO, ECHOCTL, ECHOE, ECHOK, ECHOKE, ECHONL, ICANON, ICRNL, IEXTEN, IGNBRK,
    IGNCR, IGNPAR, IMAXBEL, INLCR, INPCK, ISIG, ISTRIP, IUTF8, IXANY, IXON, OCRNL, ONLCR, ONLRET,
    ONOCR, OPOST, PARMRK, TAB3, TABDLY, VDISCARD, VEOF, VEOL, VEOL2, VERASE, VINTR, VKILL, VLNEXT,
    VMIN, VQUIT, VREPRINT, VSTART, VSTOP, VSUSP, VTIME, VWERASE,
};
use crate::console::{
    RxSample, DATA_READY, UART_LSR_BREAK, UART_LSR_FRAME, UART_LSR_OVERRUN, UART_LSR_PARITY,
};
use crate::task::signal::{SIGINT, SIGQUIT, SIGTSTP};

/// 输入队列固定 4096 字节。
pub const INPUT_CAPACITY: usize = 4096;
/// 输出队列固定 4096 字节。
pub const OUTPUT_CAPACITY: usize = 4096;
/// 回显操作队列固定 4104 项。
pub const ECHO_CAPACITY: usize = 4104;
/// 读写 uaccess 片段固定 512 字节。
pub const SEGMENT_BYTES: usize = 512;

/// 回车字节。
const CARRIAGE_RETURN: u8 = b'\r';
/// 换行字节。
const LINE_FEED: u8 = b'\n';
/// 退格字节。
const BACKSPACE: u8 = 0x08;
/// 响铃字节。
const BELL: u8 = 0x07;
/// 禁用控制字符。
const DISABLED_CHAR: u8 = 0;
const TAB_WIDTH: usize = 8;
const TAB_COLUMN_MASK: usize = TAB_WIDTH - 1;
const ECHO_TAB_AFTER_TAB: u8 = 0x80;
const ECHO_EXPANSION_BUDGET: usize = 64;

/// 回显操作：输出一个原始字节。
const ECHO_BYTE: u8 = 0;
/// 回显操作：输出控制字符的 ^X 形式。
const ECHO_CONTROL: u8 = 1;
/// 回显操作：记录当前规范列。
const ECHO_SET_CANON: u8 = 2;
/// 回显操作：列回退一格。
const ECHO_MOVE_BACK: u8 = 3;
/// 回显操作：按记录擦除一个制表符。
const ECHO_ERASE_TAB: u8 = 4;
/// 回显操作：擦到规范起列，按当前列差增量生成退格。
const ECHO_ERASE_LINE: u8 = 5;
/// 回显操作：展开挂起重显快照，未完留头。
const ECHO_REPRINT: u8 = 6;
/// 回显项保存显式操作码与数据，不借 0 或 FF 表达空项。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct EchoItem {
    /// 操作码。
    op: u8,
    /// 数据字节。
    value: u8,
}

/// 固定容量的输入字节环，canonical 记录与 EOF 标记独立于字节值。
/// 环表达：tail 读出、head 写入、len 已存字节、committed 已提交字节。
/// 满队列 head==tail 时仍由 len 区分，不靠指针相等推断空满。
pub struct InputQueue {
    /// 环形字节存储。
    pub(crate) bytes: [u8; INPUT_CAPACITY],
    /// 记录分隔标记，与字节下标一一对应。
    eol: [bool; INPUT_CAPACITY],
    /// EOF 提交标记，与字节下标一一对应。
    eof: [bool; INPUT_CAPACITY],
    /// 读出下标。
    pub(crate) tail: usize,
    /// 写入下标。
    pub(crate) head: usize,
    /// 已保存字节数。
    pub(crate) len: usize,
    /// 已提交记录字节数，不超过 len。
    pub(crate) committed: usize,
    /// LNEXT 转义状态。
    pub(crate) lnext: bool,
    /// 挂起 EOF：旧记录读完后产生一次零返回。
    pub(crate) pending_eof: bool,
}

impl InputQueue {
    /// 建立空输入队列。
    pub const fn new() -> Self {
        Self {
            bytes: [0; INPUT_CAPACITY],
            eol: [false; INPUT_CAPACITY],
            eof: [false; INPUT_CAPACITY],
            tail: 0,
            head: 0,
            len: 0,
            committed: 0,
            lnext: false,
            pending_eof: false,
        }
    }

    /// 返回已保存字节数。
    pub fn len(&self) -> usize {
        self.len
    }

    /// 返回队列是否为空。
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// 返回已提交记录字节数。
    pub fn committed_len(&self) -> usize {
        self.committed
    }

    /// 返回未提交编辑尾长度。
    pub fn edit_len(&self) -> usize {
        self.len - self.committed
    }

    /// 返回已提交记录边界的环下标。
    pub(crate) fn committed_index(&self) -> usize {
        (self.tail + self.committed) % INPUT_CAPACITY
    }

    /// 清空全部输入与编辑状态。
    pub fn clear(&mut self) {
        self.tail = 0;
        self.head = 0;
        self.len = 0;
        self.committed = 0;
        self.lnext = false;
        self.pending_eof = false;
        self.eol = [false; INPUT_CAPACITY];
        self.eof = [false; INPUT_CAPACITY];
    }

    /// 在队尾追加一个字节，满或挂起 EOF 未发表时返回假。
    fn push_byte(&mut self, byte: u8) -> bool {
        if self.pending_eof || self.len == INPUT_CAPACITY {
            return false;
        }
        self.bytes[self.head] = byte;
        self.eol[self.head] = false;
        self.eof[self.head] = false;
        self.head = (self.head + 1) % INPUT_CAPACITY;
        self.len += 1;
        true
    }

    /// canonical 普通数据入队，总环留一格结束符，满则拒存。
    fn push_canon_data(&mut self, byte: u8) -> bool {
        if self.pending_eof || INPUT_CAPACITY - 1 <= self.len {
            return false;
        }
        self.push_byte(byte)
    }

    /// 返回 canonical 普通数据能否容下完整扩展字节数。
    fn canon_data_fits(&self, count: usize) -> bool {
        !self.pending_eof && self.len + count < INPUT_CAPACITY
    }

    /// 查看距读出端指定偏移的字节。
    fn at(&self, offset: usize) -> u8 {
        self.bytes[(self.tail + offset) % INPUT_CAPACITY]
    }

    /// 返回距读出端指定偏移的字节是否为记录分隔。
    fn is_eol(&self, offset: usize) -> bool {
        self.eol[(self.tail + offset) % INPUT_CAPACITY]
    }

    /// 返回距读出端指定偏移的字节是否为 EOF 占位。
    fn is_eof(&self, offset: usize) -> bool {
        self.eof[(self.tail + offset) % INPUT_CAPACITY]
    }

    /// 消费读出端指定数量的字节；腾出首格立即发表挂起 EOF。
    fn consume(&mut self, count: usize) {
        for _ in 0..count {
            self.eol[self.tail] = false;
            self.eof[self.tail] = false;
            self.tail = (self.tail + 1) % INPUT_CAPACITY;
            self.len -= 1;
            if 0 < self.committed {
                self.committed -= 1;
            }
        }
        if self.pending_eof && self.len < INPUT_CAPACITY {
            self.pending_eof = false;
            self.bytes[self.head] = DISABLED_CHAR;
            self.eol[self.head] = true;
            self.eof[self.head] = true;
            self.head = (self.head + 1) % INPUT_CAPACITY;
            self.len += 1;
            self.committed += 1;
        }
    }

    /// 提交当前编辑尾为一条记录；调用者保证编辑尾非空。
    fn commit_record(&mut self) {
        let index = (self.head + INPUT_CAPACITY - 1) % INPUT_CAPACITY;
        self.eol[index] = true;
        self.committed = self.len;
    }

    /// 提交一个空 EOF 占位；满时挂起一次边界，后继数据阻塞计 overrun。
    /// 第二个额外 EOF 超额度返回假，调用者计真实 overrun。
    fn commit_eof(&mut self) -> bool {
        if self.pending_eof {
            return false;
        }
        if self.push_byte(DISABLED_CHAR) {
            let index = (self.head + INPUT_CAPACITY - 1) % INPUT_CAPACITY;
            self.eol[index] = true;
            self.eof[index] = true;
            self.committed = self.len;
            return true;
        }
        // 环满且编辑尾为空时挂起；编辑尾非空不应到此（应 commit_record）。
        self.pending_eof = true;
        true
    }

    /// 丢弃未提交编辑尾的最后一个字符，返回被删字节与列宽。
    fn erase_char(&mut self, utf8: bool) -> Option<(u8, usize)> {
        if self.edit_len() == 0 {
            return None;
        }
        let boundary = self.committed_index();
        let mut start = (self.head + INPUT_CAPACITY - 1) % INPUT_CAPACITY;
        let mut count = 1;
        if utf8 {
            while start != boundary {
                let byte = self.bytes[start];
                if byte & 0xc0 != 0x80 {
                    break;
                }
                start = (start + INPUT_CAPACITY - 1) % INPUT_CAPACITY;
                count += 1;
                if self.len < count {
                    return None;
                }
            }
            if self.bytes[start] & 0xc0 == 0x80 {
                return None;
            }
        }
        let byte = self.bytes[start];
        self.head = start;
        self.len -= count;
        self.eol[start] = false;
        self.eof[start] = false;
        Some((byte, count))
    }

    /// 观察未提交编辑尾的最后一个字节。
    fn edit_last(&self) -> Option<u8> {
        if self.edit_len() == 0 {
            return None;
        }
        Some(self.bytes[(self.head + INPUT_CAPACITY - 1) % INPUT_CAPACITY])
    }

    /// 丢弃整个未提交编辑尾。
    fn erase_line(&mut self) {
        let edit = self.edit_len();
        for _ in 0..edit {
            self.head = (self.head + INPUT_CAPACITY - 1) % INPUT_CAPACITY;
            self.eol[self.head] = false;
            self.eof[self.head] = false;
            self.len -= 1;
        }
    }
}

/// 固定容量的回显操作环。
pub struct EchoQueue {
    /// 环形操作存储。
    items: [EchoItem; ECHO_CAPACITY],
    /// 读出下标。
    tail: usize,
    /// 写入下标。
    head: usize,
    /// 已保存项数。
    len: usize,
}

impl EchoQueue {
    /// 建立空回显队列。
    pub const fn new() -> Self {
        Self {
            items: [EchoItem {
                op: ECHO_BYTE,
                value: 0,
            }; ECHO_CAPACITY],
            tail: 0,
            head: 0,
            len: 0,
        }
    }

    /// 返回已保存项数。
    pub fn len(&self) -> usize {
        self.len
    }

    /// 返回队列是否为空。
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// 清空全部回显操作。
    pub fn clear(&mut self) {
        self.tail = 0;
        self.head = 0;
        self.len = 0;
    }

    /// 追加一个回显项，满时返回假。
    /// 调用者先按本次编辑最大展开量 reserve，失败回压输入不丢尾。
    fn push(&mut self, op: u8, value: u8) -> bool {
        if self.len == ECHO_CAPACITY {
            return false;
        }
        self.items[self.head] = EchoItem { op, value };
        self.head = (self.head + 1) % ECHO_CAPACITY;
        self.len += 1;
        true
    }

    /// 取出读出端一项。
    fn pop(&mut self) -> Option<EchoItem> {
        if self.len == 0 {
            return None;
        }
        let item = self.items[self.tail];
        self.tail = (self.tail + 1) % ECHO_CAPACITY;
        self.len -= 1;
        Some(item)
    }

    /// 查看读出端一项。
    fn peek(&self) -> Option<EchoItem> {
        if self.len == 0 {
            return None;
        }
        Some(self.items[self.tail])
    }
}

/// 固定容量的输出字节环。
pub struct OutputQueue {
    /// 环形字节存储。
    bytes: [u8; OUTPUT_CAPACITY],
    /// 读出下标。
    tail: usize,
    /// 写入下标。
    head: usize,
    /// 已保存字节数。
    len: usize,
}

impl OutputQueue {
    /// 建立空输出队列。
    pub const fn new() -> Self {
        Self {
            bytes: [0; OUTPUT_CAPACITY],
            tail: 0,
            head: 0,
            len: 0,
        }
    }

    /// 返回已保存字节数。
    pub fn len(&self) -> usize {
        self.len
    }

    /// 返回队列是否为空。
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// 返回剩余可写入字节数。
    pub fn space(&self) -> usize {
        OUTPUT_CAPACITY - self.len
    }

    /// 清空全部输出字节。
    pub fn clear(&mut self) {
        self.tail = 0;
        self.head = 0;
        self.len = 0;
    }

    /// 追加一个字节，满时返回假。
    pub fn push(&mut self, byte: u8) -> bool {
        if self.len == OUTPUT_CAPACITY {
            return false;
        }
        self.bytes[self.head] = byte;
        self.head = (self.head + 1) % OUTPUT_CAPACITY;
        self.len += 1;
        true
    }

    /// 取出读出端一个字节。
    pub fn pop(&mut self) -> Option<u8> {
        if self.len == 0 {
            return None;
        }
        let byte = self.bytes[self.tail];
        self.tail = (self.tail + 1) % OUTPUT_CAPACITY;
        self.len -= 1;
        Some(byte)
    }

    /// 查看读出端一个字节。
    pub fn peek(&self) -> Option<u8> {
        if self.len == 0 {
            return None;
        }
        Some(self.bytes[self.tail])
    }
}

/// 一次采样产生的真实就绪变化与待锁外递送的前台信号。
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct ReceiveOutcome {
    pub read_changed: bool,
    pub write_changed: bool,
    pub signal: Option<u8>,
}

impl ReceiveOutcome {
    /// 将纯队列变化转换为没有信号的输入效果。
    fn changes((read_changed, write_changed): (bool, bool)) -> Self {
        Self {
            read_changed,
            write_changed,
            signal: None,
        }
    }
}

/// 行规程短锁保护的全部状态。
pub struct NttyState {
    /// 当前 termios 配置。
    pub termios: Termios,
    /// 输入字节与记录队列。
    pub input: InputQueue,
    /// 回显操作队列。
    pub echo: EchoQueue,
    /// 输出字节队列。
    pub output: OutputQueue,
    /// 当前输出列。
    pub column: usize,
    /// 规范行首列。
    pub canon_column: usize,
    /// 软件流控停止状态。
    pub stopped: bool,
    /// TCXONC 停止状态。
    pub tco_stopped: bool,
    /// 优先输出字符。
    pub xchar: Option<u8>,
    /// 真实输入溢出计数。
    pub overrun: usize,
    /// 独立打开计数。
    pub opens: usize,
    /// DTR 线状态。
    pub dtr: bool,
    /// RTS 线状态。
    pub rts: bool,
    /// 当前硬件分频。
    pub divisor: u16,
    /// 挂起重显快照：echo 满时保存旧行拥有字节。
    pub pending_reprint: Option<PendingReprint>,
}

/// 挂起重显的拥有快照与进度，不保存可变环指针。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PendingReprint {
    /// 旧行字节快照。
    pub bytes: [u8; INPUT_CAPACITY],
    /// 快照有效长度。
    pub len: usize,
    /// 已生成字节数。
    pub done: usize,
}

impl NttyState {
    /// 用默认 termios 建立行规程状态。
    pub const fn new() -> Self {
        Self {
            termios: Termios {
                iflag: super::termios::DEFAULT_IFLAG,
                oflag: super::termios::DEFAULT_OFLAG,
                cflag: super::termios::DEFAULT_CFLAG,
                lflag: super::termios::DEFAULT_LFLAG,
                line: 0,
                cc: super::termios::DEFAULT_CC,
            },
            input: InputQueue::new(),
            echo: EchoQueue::new(),
            output: OutputQueue::new(),
            column: 0,
            canon_column: 0,
            stopped: false,
            tco_stopped: false,
            xchar: None,
            overrun: 0,
            opens: 0,
            dtr: true,
            rts: true,
            divisor: 1,
            pending_reprint: None,
        }
    }

    /// 返回输出丢弃状态，与 termios 快照同一位。
    pub fn flusho(&self) -> bool {
        self.termios.lflag & super::termios::FLUSHO != 0
    }

    /// 处理一个采样，只记录输入效果；实际组信号必须在全部业务锁外发送。
    pub fn receive_sample(&mut self, sample: RxSample) -> ReceiveOutcome {
        let termios = self.termios;
        if sample.status & UART_LSR_BREAK != 0 {
            return self.receive_break(termios);
        }
        if sample.status & (UART_LSR_PARITY | UART_LSR_FRAME) != 0 {
            return ReceiveOutcome::changes(self.receive_parity_error(termios, sample.byte));
        }
        if sample.status & UART_LSR_OVERRUN != 0 {
            self.overrun += 1;
        }
        if sample.status & DATA_READY == 0 {
            return ReceiveOutcome::default();
        }
        self.receive_byte(termios, sample.byte)
    }

    /// BRKINT 产生 INT；忽略与错误标记保持原输入顺序和完整容量边界。
    fn receive_break(&mut self, termios: Termios) -> ReceiveOutcome {
        if termios.iflag & IGNBRK != 0 {
            return ReceiveOutcome::default();
        }
        if termios.iflag & BRKINT != 0 {
            if termios.lflag & super::termios::NOFLSH == 0 {
                self.local_flush();
            }
            return ReceiveOutcome {
                read_changed: true,
                write_changed: true,
                signal: Some(SIGINT),
            };
        }
        let changed = if termios.iflag & PARMRK != 0 {
            self.store_marked_bytes(termios, &[0xff, 0, 0])
        } else {
            self.queue_input_byte(termios, 0)
        };
        ReceiveOutcome::changes(changed)
    }

    /// 处理真实校验或帧错误，返回读写变化；PARMRK 按 FF 00 c 原序提交。
    fn receive_parity_error(&mut self, termios: Termios, byte: u8) -> (bool, bool) {
        if termios.iflag & INPCK == 0 {
            return self.queue_input_byte(termios, byte);
        }
        if termios.iflag & IGNPAR != 0 {
            return (false, false);
        }
        if termios.iflag & PARMRK != 0 {
            return self.store_marked_bytes(termios, &[0xff, 0, byte]);
        }
        self.queue_input_byte(termios, 0)
    }

    /// 原子提交错误标记字节组，容量/回显先完整 reserve，失败整组丢弃。
    fn store_marked_bytes(&mut self, termios: Termios, marked: &[u8]) -> (bool, bool) {
        let canonical = termios.lflag & ICANON != 0;
        let need = if termios.lflag & ECHO != 0 {
            if self.input.edit_len() == 0 {
                2
            } else {
                1
            }
        } else {
            0
        };
        if !self.echo_reserve(need) {
            self.overrun += 1;
            return (false, true);
        }
        let fits = if canonical {
            self.input.canon_data_fits(marked.len())
        } else {
            !self.input.pending_eof && self.input.len + marked.len() <= INPUT_CAPACITY
        };
        if !fits {
            self.overrun += 1;
            return (false, true);
        }
        if termios.lflag & ECHO != 0 {
            if self.input.edit_len() == 0 {
                let _ = self.echo_set_canon();
            }
            let _ = self.echo_char(termios, marked[marked.len() - 1]);
        }
        for &byte in marked {
            let ok = if canonical {
                self.input.push_canon_data(byte)
            } else {
                self.input.push_byte(byte)
            };
            debug_assert!(ok, "tty marked bytes reserve lost");
        }
        if !canonical {
            self.input.committed = self.input.len;
        }
        (true, true)
    }

    /// 按 strip、literal-next、流控和信号顺序解释输入，不触任务图。
    fn receive_byte(&mut self, termios: Termios, mut byte: u8) -> ReceiveOutcome {
        if self.input.lnext {
            self.input.lnext = false;
            if termios.iflag & ISTRIP != 0 {
                byte &= 0x7f;
            }
            return ReceiveOutcome::changes(self.store_data_byte(termios, byte));
        }
        if termios.iflag & ISTRIP != 0 {
            byte &= 0x7f;
        }
        if termios.iflag & IXON != 0 && self.receive_flow_control(termios, byte) {
            return ReceiveOutcome::changes((false, true));
        }
        if termios.lflag & ISIG != 0 && byte != DISABLED_CHAR {
            let signal = if byte == termios.cc[VINTR] {
                Some(SIGINT)
            } else if byte == termios.cc[VQUIT] {
                Some(SIGQUIT)
            } else if byte == termios.cc[VSUSP] {
                Some(SIGTSTP)
            } else {
                None
            };
            if signal.is_some() {
                let read_changed = self.receive_signal(termios, byte);
                return ReceiveOutcome {
                    read_changed,
                    write_changed: true,
                    signal,
                };
            }
        }
        if self.stopped
            && !self.tco_stopped
            && termios.iflag & IXON != 0
            && termios.iflag & IXANY != 0
        {
            self.stopped = false;
        }
        if termios.lflag & IEXTEN != 0 && byte == termios.cc[VDISCARD] && byte != DISABLED_CHAR {
            self.termios.lflag ^= super::termios::FLUSHO;
            if self.flusho() {
                self.flush_output();
            }
            if termios.lflag & ECHO != 0 {
                if !self.echo_reserve(1) {
                    self.overrun += 1;
                } else {
                    let _ = self.echo_char(termios, byte);
                }
            }
            return ReceiveOutcome::changes((false, true));
        }
        if byte == CARRIAGE_RETURN {
            if termios.iflag & IGNCR != 0 {
                return ReceiveOutcome::default();
            }
            if termios.iflag & ICRNL != 0 {
                byte = LINE_FEED;
            }
        } else if byte == LINE_FEED && termios.iflag & INLCR != 0 {
            byte = CARRIAGE_RETURN;
        }
        if termios.lflag & ICANON != 0 && self.receive_canonical(termios, byte) {
            return ReceiveOutcome::changes((true, true));
        }
        ReceiveOutcome::changes(self.store_data_byte(termios, byte))
    }
    /// 处理软件流控字符，返回是否消费。
    fn receive_flow_control(&mut self, termios: Termios, byte: u8) -> bool {
        if byte == termios.cc[VSTART] && byte != DISABLED_CHAR {
            self.stopped = false;
            return true;
        }
        if byte == termios.cc[VSTOP] && byte != DISABLED_CHAR {
            self.stopped = true;
            return true;
        }
        false
    }

    /// 处理信号控制字符的本地解释与回显，不递送前台组。
    fn receive_signal(&mut self, termios: Termios, byte: u8) -> bool {
        if termios.lflag & super::termios::NOFLSH == 0 {
            self.local_flush();
        }
        if termios.iflag & IXON != 0 {
            self.stopped = false;
        }
        if termios.lflag & ECHO != 0 {
            if !self.echo_reserve(1) {
                self.overrun += 1;
            } else {
                let _ = self.echo_char(termios, byte);
            }
        }
        true
    }

    /// 信号 flush 只退休已解释输入和未发送输出，不丢接收 batch 后继采样。
    fn local_flush(&mut self) {
        self.input.clear();
        self.flush_output();
    }

    /// 处理规范模式的编辑与记录字符，返回是否已消费；回显满回压输入。
    fn receive_canonical(&mut self, termios: Termios, byte: u8) -> bool {
        if byte == termios.cc[VERASE] && byte != DISABLED_CHAR
            || byte == termios.cc[VKILL] && byte != DISABLED_CHAR
            || (termios.lflag & IEXTEN != 0 && byte == termios.cc[VWERASE] && byte != DISABLED_CHAR)
        {
            self.erase_input(termios, byte);
            return true;
        }
        if termios.lflag & IEXTEN != 0 && byte == termios.cc[VLNEXT] && byte != DISABLED_CHAR {
            if termios.lflag & ECHO != 0 && termios.lflag & ECHOCTL != 0 && !self.echo_reserve(2) {
                self.overrun += 1;
                return true;
            }
            self.input.lnext = true;
            if termios.lflag & ECHO != 0 && termios.lflag & ECHOCTL != 0 {
                let _ = self.echo_raw(b'^');
                let _ = self.echo_raw(b'\x08');
            }
            return true;
        }
        if termios.lflag & IEXTEN != 0
            && termios.lflag & ECHO != 0
            && byte == termios.cc[VREPRINT]
            && byte != DISABLED_CHAR
        {
            // 已有挂起重显时第二动作明确 overrun，不覆盖快照。
            if self.pending_reprint.is_some() {
                self.overrun += 1;
                return true;
            }
            // 前缀 2 项 + 重显操作 1 项。
            if !self.echo_reserve(3) {
                self.overrun += 1;
                return true;
            }
            self.reprint_line(termios);
            return true;
        }
        if byte == LINE_FEED {
            if termios.lflag & (ECHO | ECHONL) != 0 && !self.echo_reserve(1) {
                self.overrun += 1;
                return true;
            }
            if termios.lflag & (ECHO | ECHONL) != 0 {
                let _ = self.echo_raw(LINE_FEED);
            }
            if self.input.push_byte(byte) {
                self.input.commit_record();
            } else {
                self.overrun += 1;
            }
            return true;
        }
        if byte == termios.cc[VEOF] && byte != DISABLED_CHAR {
            if 0 < self.input.edit_len() {
                self.input.commit_record();
            } else if !self.input.commit_eof() {
                self.overrun += 1;
            }
            return true;
        }
        if (byte == termios.cc[VEOL] && byte != DISABLED_CHAR)
            || (termios.lflag & IEXTEN != 0 && byte == termios.cc[VEOL2] && byte != DISABLED_CHAR)
        {
            let need = if termios.lflag & ECHO != 0 {
                if self.input.edit_len() == 0 {
                    2
                } else {
                    1
                }
            } else {
                0
            };
            if !self.echo_reserve(need) {
                self.overrun += 1;
                return true;
            }
            if termios.lflag & ECHO != 0 {
                if self.input.edit_len() == 0 {
                    let _ = self.echo_set_canon();
                }
                let _ = self.echo_char(termios, byte);
            }
            let count = if byte == 0xff && termios.iflag & PARMRK != 0 {
                2
            } else {
                1
            };
            // 结束符扩展自身占用终止槽，完整预留后无需再给下一字符留格。
            if !(!self.input.pending_eof && self.input.len + count <= INPUT_CAPACITY) {
                self.overrun += 1;
                return true;
            }
            for _ in 0..count {
                let ok = self.input.push_byte(byte);
                debug_assert!(ok, "tty record ending reserve lost");
            }
            self.input.commit_record();
            return true;
        }
        false
    }

    /// 把数据字节存入输入队列并回显，返回读写变化；回显满回压输入。
    fn store_data_byte(&mut self, termios: Termios, byte: u8) -> (bool, bool) {
        let need = if termios.lflag & ECHO != 0 {
            if self.input.edit_len() == 0 {
                2
            } else {
                1
            }
        } else if termios.lflag & ICANON != 0 && termios.lflag & ECHONL != 0 && byte == LINE_FEED {
            1
        } else {
            0
        };
        if !self.echo_reserve(need) {
            self.overrun += 1;
            return (false, true);
        }
        // 先落回显后存输入，保证输入满时回显已可见且不丢尾。
        if termios.lflag & ECHO != 0 {
            if self.input.edit_len() == 0 {
                let _ = self.echo_set_canon();
            }
            if byte == LINE_FEED {
                let _ = self.echo_raw(LINE_FEED);
            } else {
                let _ = self.echo_char(termios, byte);
            }
        } else if termios.lflag & ICANON != 0 && termios.lflag & ECHONL != 0 && byte == LINE_FEED {
            let _ = self.echo_raw(LINE_FEED);
        }
        let canonical = termios.lflag & ICANON != 0;
        // 0xFF 加倍需完整两格，先 reserve，失败不交残片。
        if byte == 0xff && termios.iflag & PARMRK != 0 {
            let fits = if canonical {
                self.input.canon_data_fits(2)
            } else {
                !self.input.pending_eof && self.input.len + 2 <= INPUT_CAPACITY
            };
            if !fits {
                self.bell_or_drop(termios);
                return (false, true);
            }
            for _ in 0..2 {
                let ok = if canonical {
                    self.input.push_canon_data(byte)
                } else {
                    self.input.push_byte(byte)
                };
                debug_assert!(ok, "tty PARMRK reserve lost");
            }
        } else {
            let ok = if canonical {
                self.input.push_canon_data(byte)
            } else {
                self.input.push_byte(byte)
            };
            if !ok {
                self.bell_or_drop(termios);
                return (false, true);
            }
        }
        if !canonical {
            self.input.committed = self.input.len;
        }
        (true, true)
    }

    /// 把数据字节直接存入队列，不执行规范解释。
    fn queue_input_byte(&mut self, termios: Termios, byte: u8) -> (bool, bool) {
        self.store_data_byte(termios, byte)
    }

    /// 输入满时按 IMAXBEL 响铃或丢弃；输出满不伪成功。
    fn bell_or_drop(&mut self, termios: Termios) {
        self.overrun += 1;
        if termios.iflag & IMAXBEL != 0 && !self.output.push(BELL) {
            self.overrun += 1;
        }
    }

    /// 处理擦除、删词与删行；ECHOKE 用擦行操作保序增量生成。
    fn erase_input(&mut self, termios: Termios, byte: u8) {
        if self.input.edit_len() == 0 {
            return;
        }
        if byte == termios.cc[VKILL] && byte != DISABLED_CHAR {
            if termios.lflag & ECHO == 0 {
                self.input.erase_line();
                return;
            }
            if termios.lflag & ECHOK == 0
                || termios.lflag & ECHOKE == 0
                || termios.lflag & ECHOE == 0
            {
                let need = if termios.lflag & ECHOK != 0 { 2 } else { 1 };
                if !self.echo_reserve(need) {
                    self.overrun += 1;
                    return;
                }
                self.input.erase_line();
                let _ = self.echo_char(termios, byte);
                if termios.lflag & ECHOK != 0 {
                    let _ = self.echo_raw(LINE_FEED);
                }
                return;
            }
            // ECHOKE：删整行后入一个擦行操作，按列差增量生成，不预铺退格。
            if !self.echo_reserve(1) {
                self.overrun += 1;
                return;
            }
            self.input.erase_line();
            let _ = self.echo.push(ECHO_ERASE_LINE, 0);
            return;
        }
        if !self.echo_reserve(6) {
            self.overrun += 1;
            return;
        }
        let werase = termios.lflag & IEXTEN != 0 && byte == termios.cc[VWERASE];
        let mut seen_word = false;
        loop {
            let Some(last) = self.input.edit_last() else {
                break;
            };
            if werase {
                if last.is_ascii_alphanumeric() || last == b'_' {
                    seen_word = true;
                } else if seen_word {
                    break;
                }
            }
            if !self.echo_reserve(6) {
                self.overrun += 1;
                break;
            }
            if !self.erase_one(termios) {
                break;
            }
            if !werase {
                break;
            }
        }
    }
    /// 擦除一个字符并回显；调用者已 reserve，失败不删输入。
    fn erase_one(&mut self, termios: Termios) -> bool {
        let utf8 = termios.iflag & IUTF8 != 0;
        // 先按最大 6 项退格 reserve，再删输入，保证不丢回显尾。
        if termios.lflag & ECHO != 0 && !self.echo_reserve(6) {
            self.overrun += 1;
            return false;
        }
        let Some((byte, _)) = self.input.erase_char(utf8) else {
            return false;
        };
        if termios.lflag & ECHO == 0 {
            return true;
        }
        if termios.lflag & ECHOE == 0 {
            let _ = self.echo_char(termios, termios.cc[VERASE]);
            return true;
        }
        if byte == b'\t' {
            let boundary = self.input.committed_index();
            let mut count = 0usize;
            let mut index = self.input.head;
            let mut after_tab = false;
            let mut scanned = 0;
            while index != boundary && scanned < self.input.edit_len() {
                index = (index + INPUT_CAPACITY - 1) % INPUT_CAPACITY;
                let value = self.input.bytes[index];
                scanned += 1;
                if value == b'\t' {
                    after_tab = true;
                    break;
                } else if value.is_ascii_control() {
                    if termios.lflag & ECHOCTL != 0 {
                        count += 2;
                    }
                } else if !(utf8 && value & 0xc0 == 0x80) {
                    count += 1;
                }
            }
            let packed =
                (count & TAB_COLUMN_MASK) as u8 | if after_tab { ECHO_TAB_AFTER_TAB } else { 0 };
            let _ = self.echo.push(ECHO_ERASE_TAB, packed);
            return true;
        }
        if byte.is_ascii_control() && termios.lflag & ECHOCTL != 0 {
            let _ = self.echo_raw(BACKSPACE);
            let _ = self.echo_raw(b' ');
            let _ = self.echo_raw(BACKSPACE);
        }
        if !byte.is_ascii_control() || termios.lflag & ECHOCTL != 0 {
            let _ = self.echo_raw(BACKSPACE);
            let _ = self.echo_raw(b' ');
            let _ = self.echo_raw(BACKSPACE);
        }
        true
    }

    /// 重显当前未提交行；前缀与重显操作入 FIFO，快照保序展开。
    fn reprint_line(&mut self, termios: Termios) {
        let _ = self.echo_char(termios, termios.cc[VREPRINT]);
        let _ = self.echo_raw(LINE_FEED);
        let edit = self.input.edit_len();
        if edit == 0 {
            return;
        }
        let boundary = self.input.committed_index();
        let mut bytes = [0u8; INPUT_CAPACITY];
        let mut index = boundary;
        for slot in bytes.iter_mut().take(edit) {
            *slot = self.input.bytes[index];
            index = (index + 1) % INPUT_CAPACITY;
        }
        self.pending_reprint = Some(PendingReprint {
            bytes,
            len: edit,
            done: 0,
        });
        let _ = self.echo.push(ECHO_REPRINT, 0);
    }

    /// 回显一个原始字节，满时返回假，调用者回压输入。
    fn echo_raw(&mut self, byte: u8) -> bool {
        self.echo.push(ECHO_BYTE, byte)
    }

    /// 回显一个字符，控制字符按 ECHOCTL 加标记，满时返回假。
    fn echo_char(&mut self, termios: Termios, byte: u8) -> bool {
        if termios.lflag & ECHOCTL != 0 && byte.is_ascii_control() && byte != b'\t' {
            self.echo.push(ECHO_CONTROL, byte)
        } else {
            self.echo.push(ECHO_BYTE, byte)
        }
    }

    /// 记录规范行首列，满时返回假。
    fn echo_set_canon(&mut self) -> bool {
        self.echo.push(ECHO_SET_CANON, 0)
    }

    /// 按本次编辑最大展开量预留回显空间，不足返回假。
    fn echo_reserve(&self, count: usize) -> bool {
        self.echo.len() + count <= ECHO_CAPACITY
    }

    /// 把回显操作展开为输出字节，返回是否仍有未处理尾。
    pub fn process_echo(&mut self) -> bool {
        let termios = self.termios;
        while let Some(item) = self.echo.peek() {
            match item.op {
                ECHO_BYTE => {
                    if !self.output_byte(termios, item.value) {
                        return true;
                    }
                    self.echo.pop();
                }
                ECHO_CONTROL => {
                    if self.output.space() < 2 {
                        return true;
                    }
                    let _ = self.output.push(b'^');
                    self.column += 1;
                    let _ = self.output.push(item.value ^ 0x40);
                    self.column += 1;
                    self.echo.pop();
                }
                ECHO_SET_CANON => {
                    self.canon_column = self.column;
                    self.echo.pop();
                }
                ECHO_MOVE_BACK => {
                    if 0 < self.column {
                        self.column -= 1;
                    }
                    self.echo.pop();
                }
                ECHO_ERASE_TAB => {
                    let mut count = usize::from(item.value & TAB_COLUMN_MASK as u8);
                    if item.value & ECHO_TAB_AFTER_TAB == 0 {
                        count += self.canon_column;
                    }
                    let backs = TAB_WIDTH - (count & TAB_COLUMN_MASK);
                    if self.output.space() < backs {
                        return true;
                    }
                    for _ in 0..backs {
                        let _ = self.output.push(BACKSPACE);
                        if 0 < self.column {
                            self.column -= 1;
                        }
                    }
                    self.echo.pop();
                }
                ECHO_ERASE_LINE => {
                    let mut budget = ECHO_EXPANSION_BUDGET;
                    while self.canon_column < self.column && 0 < budget {
                        if self.output.space() < 3 {
                            return true;
                        }
                        let _ = self.output.push(BACKSPACE);
                        let _ = self.output.push(b' ');
                        let _ = self.output.push(BACKSPACE);
                        self.column -= 1;
                        budget -= 1;
                    }
                    if self.canon_column < self.column {
                        return true;
                    }
                    self.echo.pop();
                }
                ECHO_REPRINT => {
                    let mut budget = ECHO_EXPANSION_BUDGET;
                    loop {
                        let (byte, done) = match self.pending_reprint.as_ref() {
                            Some(p) if p.done < p.len => (p.bytes[p.done], p.done),
                            Some(_) => break,
                            None => break,
                        };
                        if budget == 0 {
                            return true;
                        }
                        let control = termios.lflag & ECHOCTL != 0
                            && byte.is_ascii_control()
                            && byte != b'\t';
                        let expanded = if control {
                            2
                        } else {
                            self.output_required_space(termios, byte)
                        };
                        if self.output.space() < expanded {
                            return true;
                        }
                        if control {
                            let _ = self.output.push(b'^');
                            self.column += 1;
                            let _ = self.output.push(byte ^ 0x40);
                            self.column += 1;
                        } else if !self.output_byte(termios, byte) {
                            return true;
                        }
                        if let Some(p) = self.pending_reprint.as_mut() {
                            p.done = done + 1;
                            if p.done == p.len {
                                self.pending_reprint = None;
                            }
                        }
                        budget -= 1;
                    }
                    if self.pending_reprint.is_some() {
                        return true;
                    }
                    self.echo.pop();
                }
                _ => {
                    self.echo.pop();
                }
            }
        }
        false
    }

    /// 返回按当前 termios/列加工一个字节所需的完整输出槽数。
    pub fn output_required_space(&self, termios: Termios, byte: u8) -> usize {
        if termios.oflag & OPOST == 0 {
            return 1;
        }
        match byte {
            LINE_FEED if termios.oflag & ONLCR != 0 => 2,
            CARRIAGE_RETURN if termios.oflag & ONOCR != 0 && self.column == 0 => 0,
            b'\t' if termios.oflag & TABDLY == TAB3 => TAB_WIDTH - (self.column & TAB_COLUMN_MASK),
            _ => 1,
        }
    }

    /// 返回输出队列能否接受一个字节的完整展开；丢弃态直接可接受。
    pub fn output_can_accept(&self, termios: Termios, byte: u8) -> bool {
        if self.flusho() {
            return true;
        }
        self.output_required_space(termios, byte) <= self.output.space()
    }

    /// 按输出模式加工一个字节，空间不足返回假且不消费输入。
    pub fn output_byte(&mut self, termios: Termios, byte: u8) -> bool {
        if self.output.space() < self.output_required_space(termios, byte) {
            return false;
        }
        if termios.oflag & OPOST == 0 {
            if !byte.is_ascii_control() && termios.iflag & IUTF8 != 0 && byte & 0xc0 == 0x80 {
                // UTF-8 后续字节不推进列。
            } else if !byte.is_ascii_control() {
                self.column += 1;
            } else if byte == BACKSPACE && 0 < self.column {
                self.column -= 1;
            }
            return self.output.push(byte);
        }
        match byte {
            LINE_FEED => {
                if termios.oflag & ONLRET != 0 {
                    self.column = 0;
                }
                if termios.oflag & ONLCR != 0 {
                    self.canon_column = 0;
                    self.column = 0;
                    let _ = self.output.push(CARRIAGE_RETURN);
                    let _ = self.output.push(LINE_FEED);
                    return true;
                }
                self.canon_column = self.column;
                self.output.push(LINE_FEED)
            }
            CARRIAGE_RETURN => {
                if termios.oflag & ONOCR != 0 && self.column == 0 {
                    return true;
                }
                if termios.oflag & OCRNL != 0 {
                    if termios.oflag & ONLRET != 0 {
                        self.canon_column = 0;
                        self.column = 0;
                    }
                    return self.output.push(LINE_FEED);
                }
                self.canon_column = 0;
                self.column = 0;
                self.output.push(CARRIAGE_RETURN)
            }
            b'\t' if termios.oflag & TABDLY == TAB3 => {
                let spaces = TAB_WIDTH - (self.column & TAB_COLUMN_MASK);
                for _ in 0..spaces {
                    let _ = self.output.push(b' ');
                }
                self.column += spaces;
                true
            }
            b'\t' => {
                self.column += TAB_WIDTH - (self.column & TAB_COLUMN_MASK);
                self.output.push(b'\t')
            }
            BACKSPACE => {
                if 0 < self.column {
                    self.column -= 1;
                }
                self.output.push(BACKSPACE)
            }
            _ => {
                if !byte.is_ascii_control() && !(termios.iflag & IUTF8 != 0 && byte & 0xc0 == 0x80)
                {
                    self.column += 1;
                }
                self.output.push(byte)
            }
        }
    }

    /// 切换 ICANON 时建立明确边界，不重解释已有字节。
    pub fn set_canonical(&mut self, canonical: bool) {
        let was = self.termios.lflag & ICANON != 0;
        if was == canonical {
            return;
        }
        if canonical {
            self.termios.lflag |= ICANON;
            if 0 < self.input.len {
                let index = (self.input.head + INPUT_CAPACITY - 1) % INPUT_CAPACITY;
                self.input.eol[index] = true;
                self.input.eof[index] = false;
                self.input.committed = self.input.len;
            } else {
                self.input.committed = 0;
            }
        } else {
            self.termios.lflag &= !ICANON;
            self.retire_eof_tokens();
            self.input.committed = self.input.len;
        }
        self.input.lnext = false;
    }

    /// 退休环内全部 EOF 占位 token，原环向前 compact 保数据与 eol 顺序。
    fn retire_eof_tokens(&mut self) {
        if self.input.is_empty() {
            self.input.pending_eof = false;
            return;
        }
        let len = self.input.len;
        let mut read = 0;
        let mut write = 0;
        while read < len {
            let at = (self.input.tail + read) % INPUT_CAPACITY;
            if self.input.eof[at] {
                read += 1;
                continue;
            }
            let to = (self.input.tail + write) % INPUT_CAPACITY;
            if to != at {
                self.input.bytes[to] = self.input.bytes[at];
                self.input.eol[to] = self.input.eol[at];
                self.input.eof[to] = false;
            } else {
                self.input.eof[to] = false;
            }
            read += 1;
            write += 1;
        }
        // 清尾部残留标记，读指针仍在写指针前。
        let mut i = write;
        while i < len {
            let at = (self.input.tail + i) % INPUT_CAPACITY;
            self.input.eol[at] = false;
            self.input.eof[at] = false;
            i += 1;
        }
        self.input.head = (self.input.tail + write) % INPUT_CAPACITY;
        self.input.len = write;
        self.input.committed = 0;
        self.input.pending_eof = false;
    }

    /// 规范读专用：返回已提交记录在本次请求内的可交付长度与是否含 EOF。
    pub fn canonical_available(&self, max: usize) -> (usize, bool) {
        let committed = self.input.committed_len();
        if max == 0 {
            return (0, false);
        }
        let mut length = committed.min(max);
        let mut index = 0;
        while index < length {
            if self.input.is_eol(index) {
                if self.input.is_eof(index) {
                    return (index, true);
                }
                return (index + 1, false);
            }
            index += 1;
        }
        if committed <= max && 0 < committed {
            length = committed;
        }
        (length, false)
    }

    /// 规范读专用：查看已提交记录，不消费。
    pub fn peek_canonical(&self, out: &mut [u8]) -> (usize, bool) {
        let (length, eof) = self.canonical_available(out.len());
        for (slot, offset) in out.iter_mut().zip(0..length) {
            *slot = self.input.at(offset);
        }
        (length, eof)
    }

    /// 规范读专用：消费已交付前缀，EOF 占位随零长度前缀一起消费。
    pub fn consume_canonical(&mut self, delivered: usize, eof: bool) {
        if eof && delivered == 0 {
            if 0 < self.input.committed_len() && self.input.is_eof(0) {
                self.input.consume(1);
            }
            return;
        }
        self.input.consume(delivered);
    }

    /// 非规范读专用：查看已提交字节，不消费。
    pub fn peek_raw(&self, out: &mut [u8]) -> usize {
        let available = self.input.committed_len().min(out.len());
        for (slot, offset) in out.iter_mut().zip(0..available) {
            *slot = self.input.at(offset);
        }
        available
    }

    /// 非规范读专用：消费已交付前缀。
    pub fn consume_raw(&mut self, delivered: usize) {
        self.input.consume(delivered);
    }

    /// 返回 poll 读就绪谓词。
    pub fn poll_readable(&self) -> bool {
        if self.termios.lflag & ICANON != 0 {
            0 < self.input.committed_len()
        } else {
            let min = usize::from(self.termios.cc[VMIN]);
            let time = self.termios.cc[VTIME];
            if time == 0 && 0 < min {
                min <= self.input.committed_len()
            } else {
                0 < self.input.committed_len()
            }
        }
    }

    /// 返回 poll 写就绪谓词。
    pub fn poll_writable(&self) -> bool {
        0 < self.output.space()
    }

    /// 丢弃未读输入。
    pub fn flush_input(&mut self) {
        self.input.clear();
    }

    /// 丢弃未发送输出与回显，取消挂起重显。
    pub fn flush_output(&mut self) {
        self.output.clear();
        self.echo.clear();
        self.xchar = None;
        self.pending_reprint = None;
    }

    /// 返回软件队列、优先字符与挂起重显是否全部为空。
    pub fn output_empty(&self) -> bool {
        self.output.is_empty()
            && self.echo.is_empty()
            && self.xchar.is_none()
            && self.pending_reprint.is_none()
    }
}
