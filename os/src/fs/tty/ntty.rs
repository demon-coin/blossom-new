//! `N_TTY` 行规程（Line Discipline）核心状态机与输入、回显、输出缓冲算法。
//!
//! # 模块职责与架构定位
//! 本模块实现标准 `N_TTY` 行规程的数据结构与纯内存处理算法：
//! - **上游调用者**：`crate::fs::tty` 在持有端点自旋锁（`Endpoint::state`）的临界区内调用本模块方法。
//! - **核心职责**：
//!   1. **输入处理**：处理 UART 采样状态（BREAK、奇偶校验错误、帧错误、硬件溢出）、`ISTRIP`、`INLCR`/`IGNCR`/`ICRNL`、软件流控（`IXON`/`IXANY`）、信号字符（`VINTR`/`VQUIT`/`VSUSP`）、规范模式行编辑（`VERASE`/`VKILL`/`VWERASE`/`VLNEXT`/`VREPRINT`）及行结束符（`\n`/`VEOF`/`VEOL`/`VEOL2`）。
//!   2. **回显生成与展开**：在编辑输入时将回显指令推入 `EchoQueue`，并在工作队列或读取路径中将其展开为实际输出字节（含 `^X` 控制字符展示、制表符退格对齐、整行擦除与重印快照分批展开）。
//!   3. **输出后处理（`OPOST`）**：维护终端当前列号（`column`）与规范行起始列号（`canon_column`），执行 `\n` 转 `\r\n`（`ONLCR`）、`\r` 转 `\n`（`OCRNL`）、第 0 列抑制 `\r`（`ONOCR`）以及制表符转空格（`TAB3`）。
//!
//! # 核心环形队列与状态布局
//!
//! ## 1. 输入环形队列（`InputQueue`，容量 4096 字节）
//! ```text
//!   0                 tail                committed_index          head            4095
//! ┌───────────────────┬──────────────────────────┬──────────────────┬─────────────────┐
//! │     空闲区域       │   已提交可读区域          │  未提交行编辑尾   │    空闲区域      │
//! │                   │ (committed 字节，含 eol)  │ (edit_len 字节)  │                 │
//! └───────────────────┴──────────────────────────┴──────────────────┴─────────────────┘
//!                      ◄────────────────────── len ────────────────►
//! ```
//! - **非规范模式（`!ICANON`）**：所有入队字节立即提交，`committed == len`，`edit_len() == 0`。
//! - **规范模式（`ICANON`）**：普通字符先进入未提交编辑尾（`committed < len`），支持退格、删词、删行；收到 `\n`、`VEOL`、`VEOL2` 或 `VEOF` 时将 `committed` 推进到 `len`，并在对应下标设置 `eol[idx] = true`（若为空 `VEOF` 则同时设置 `eof[idx] = true`）。
//!
//! ## 2. 三队列协同数据流
//! ```text
//! UART RX 采样 ──► receive_sample() ──► InputQueue (4096 B) ──► peek/consume (用户 read)
//!                       │
//!                       └──► EchoQueue (4104 项)
//!                                  │
//!                                  ▼ process_echo()
//! 用户 write   ───────────► output_byte() ──► OutputQueue (4096 B) ──► UART TX 发送
//! ```
//!
//! # 锁与并发契约
//! - 本模块所有类型（`NttyState`、`InputQueue`、`EchoQueue`、`OutputQueue`）均不包含内部锁，完全由外部调用方持有的 `SpinLock<NttyState>` 保护。
//! - 本模块方法严禁睡眠、严禁直接访问 UART 硬件寄存器或向进程组发送信号（待发送信号通过 [`ReceiveOutcome::signal`] 返回给调用方在锁外投递）。

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

/// 输入字节环形缓冲区的固定容量（单位：字节）。
pub const INPUT_CAPACITY: usize = 4096;
/// 输出字节环形缓冲区的固定容量（单位：字节）。
pub const OUTPUT_CAPACITY: usize = 4096;
/// 回显操作环形队列的固定容量（单位：操作项数）。
pub const ECHO_CAPACITY: usize = 4104;
/// 用户态读写缓冲区单次分段拷贝的字节上限（单位：字节）。
pub const SEGMENT_BYTES: usize = 512;

/// 回车符 ASCII 码（`\r`，`0x0d`）。
const CARRIAGE_RETURN: u8 = b'\r';
/// 换行符 ASCII 码（`\n`，`0x0a`）。
const LINE_FEED: u8 = b'\n';
/// 退格符 ASCII 码（`BS`，`0x08`）。
const BACKSPACE: u8 = 0x08;
/// 响铃符 ASCII 码（`BEL`，`0x07`）。
const BELL: u8 = 0x07;
/// 禁用控制字符的特殊标记值（`_POSIX_VDISABLE`，`0x00`）。
const DISABLED_CHAR: u8 = 0;

/// 标准水平制表符（Tab）的列对齐宽度（8 列）。
const TAB_WIDTH: usize = 8;
/// 计算制表符列内偏移的位掩码（`TAB_WIDTH - 1 = 0b111`）。
const TAB_COLUMN_MASK: usize = TAB_WIDTH - 1;

/// `ECHO_ERASE_TAB` 操作数高位标志：指示被擦除的制表符前方仍存在另一个制表符（无需叠加 `canon_column`）。
const ECHO_TAB_AFTER_TAB: u8 = 0x80;
/// 单次 `process_echo` 展开长耗时操作（整行擦除或重印快照）的最大步数预算，防止单次持锁时间过长。
const ECHO_EXPANSION_BUDGET: usize = 64;

/// 回显操作码：按输出模式输出单个普通字节。
const ECHO_BYTE: u8 = 0;
/// 回显操作码：将控制字符展开为 `^X` 两字节形式输出。
const ECHO_CONTROL: u8 = 1;
/// 回显操作码：将当前输出列号 `column` 记录为规范行起始列 `canon_column`。
const ECHO_SET_CANON: u8 = 2;
/// 回显操作码：将输出列号回退一列（不额外输出字节）。
const ECHO_MOVE_BACK: u8 = 3;
/// 回显操作码：根据编码的列偏移输出相应数量的退格符以擦除一个制表符。
const ECHO_ERASE_TAB: u8 = 4;
/// 回显操作码：输出“退格-空格-退格”序列直至当前列退回到 `canon_column`（对应 `ECHOKE` 整行擦除）。
const ECHO_ERASE_LINE: u8 = 5;
/// 回显操作码：逐步展开 `pending_reprint` 中保存的未提交行快照（对应 `VREPRINT`）。
const ECHO_REPRINT: u8 = 6;

/// 回显操作队列中的单个指令项。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct EchoItem {
    /// 回显操作码（`ECHO_BYTE`、`ECHO_CONTROL` 等常量之一）。
    op: u8,
    /// 与操作码关联的数据字节或打包列宽参数。
    value: u8,
}

/// 固定容量的终端输入字节环形队列。
///
/// 同时支持规范模式（按行提交与行内编辑）与非规范模式（按字节即时提交）：
/// - `tail` 指向下一个待用户读取的字节下标，`head` 指向下一个新字节写入下标；
/// - `len` 记录队列中的总字节数，用于区分空队列与满队列（`head == tail` 时既可能空也可能满）；
/// - `eol` 与 `eof` 数组按环下标记录每个位置是否为行结束符或零长度 `VEOF` 占位符，从而不占用特殊字节值。
pub struct InputQueue {
    /// 存放输入字节的环形数组（容量 `INPUT_CAPACITY` 字节）。
    pub(crate) bytes: [u8; INPUT_CAPACITY],
    /// 与 `bytes` 下标一一对应的行结束标记数组；为 `true` 表示规范模式的一条记录在此处结束。
    eol: [bool; INPUT_CAPACITY],
    /// 与 `bytes` 下标一一对应的 `VEOF` 零长度占位标记数组；读取到该位置时不向用户拷贝该占位字节，直接结束当前行。
    eof: [bool; INPUT_CAPACITY],
    /// 队首读出下标（`0..INPUT_CAPACITY`）。
    pub(crate) tail: usize,
    /// 队尾写入下标（`0..INPUT_CAPACITY`）。
    pub(crate) head: usize,
    /// 当前队列中已保存的总字节数（含已提交部分与未提交编辑尾，`0..=INPUT_CAPACITY`）。
    pub(crate) len: usize,
    /// 当前已提交可供用户 `read` 消费的字节数（恒满足 `committed <= len`）。
    pub(crate) committed: usize,
    /// 是否处于 `VLNEXT`（`^V`）字面量转义状态（为 `true` 时下一个输入字符跳过特殊控制符解释）。
    pub(crate) lnext: bool,
    /// 当输入队列已满且编辑尾为空时收到 `VEOF` 所设置的挂起标志；待消费腾出空间后自动补入一个 `EOF` 占位项。
    pub(crate) pending_eof: bool,
}

impl InputQueue {
    /// 创建空的输入环形队列。
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

    /// 返回队列中当前保存的总字节数（含未提交编辑尾）。
    #[cfg(feature = "ktest")]
    pub fn len(&self) -> usize {
        self.len
    }

    /// 判断输入队列是否完全为空。
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// 返回已提交可供用户读取的字节数。
    pub fn committed_len(&self) -> usize {
        self.committed
    }

    /// 返回规范模式下尚未提交的当前编辑行字节数（`len - committed`）。
    pub fn edit_len(&self) -> usize {
        self.len - self.committed
    }

    /// 返回已提交区域末尾（即未提交编辑行起点）在环形数组中的下标。
    pub(crate) fn committed_index(&self) -> usize {
        (self.tail + self.committed) % INPUT_CAPACITY
    }

    /// 清空所有输入字节、行标记、转义状态与挂起 `EOF` 状态。
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

    /// 在队尾追加一个字节；若队列已满或存在尚未落地的 `pending_eof` 则拒绝入队并返回 `false`。
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

    /// 在规范模式下追加一个普通数据字节，固定预留最后一个槽位给行结束符（`\n` / `VEOL` / `VEOF`）。
    fn push_canon_data(&mut self, byte: u8) -> bool {
        if self.pending_eof || INPUT_CAPACITY - 1 <= self.len {
            return false;
        }
        self.push_byte(byte)
    }

    /// 检查规范模式下是否还能容纳 `count` 个普通数据字节（同时保留 1 个行结束符槽位）。
    fn canon_data_fits(&self, count: usize) -> bool {
        !self.pending_eof && self.len + count < INPUT_CAPACITY
    }

    /// 读取距离队首 `tail` 偏移 `offset` 处的字节值（不消费）。
    fn at(&self, offset: usize) -> u8 {
        self.bytes[(self.tail + offset) % INPUT_CAPACITY]
    }

    /// 判断距离队首 `tail` 偏移 `offset` 处的字节是否带有行结束标记（`eol`）。
    fn is_eol(&self, offset: usize) -> bool {
        self.eol[(self.tail + offset) % INPUT_CAPACITY]
    }

    /// 判断距离队首 `tail` 偏移 `offset` 处的字节是否为 `VEOF` 零长度占位项。
    fn is_eof(&self, offset: usize) -> bool {
        self.eof[(self.tail + offset) % INPUT_CAPACITY]
    }

    /// 从队首消费 `count` 个字节；若此前因队列满而挂起了 `pending_eof`，则在腾出空间后立即将其补入队尾。
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

    /// 将当前非空的编辑尾提交为一条完整记录（在最后一个入队字节上标记 `eol = true` 并更新 `committed`）。
    fn commit_record(&mut self) {
        let index = (self.head + INPUT_CAPACITY - 1) % INPUT_CAPACITY;
        self.eol[index] = true;
        self.committed = self.len;
    }

    /// 提交一个空行的 `VEOF` 占位符，使下一次读完前序记录后的 `read` 返回 0（EOF）。
    ///
    /// - 若队列未满，直接推入一个同时标记 `eol` 与 `eof` 的占位槽位；
    /// - 若队列已满且尚未挂起过 `pending_eof`，则设置 `pending_eof = true`，待后续消费腾出空间时补入；
    /// - 若 `pending_eof` 已经为 `true`，说明已有挂起的 `EOF` 尚未被消费，返回 `false` 由调用方计入输入溢出（Overrun）。
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
        // 环形缓冲区已满且当前编辑尾为空，挂起一次 EOF 边界。
        self.pending_eof = true;
        true
    }

    /// 从未提交编辑尾擦除最后一个字符（若开启 `utf8` 则回退完整的 UTF-8 多字节序列），返回被删首字节与字节长度。
    fn erase_char(&mut self, utf8: bool) -> Option<(u8, usize)> {
        if self.edit_len() == 0 {
            return None;
        }
        let boundary = self.committed_index();
        let mut start = (self.head + INPUT_CAPACITY - 1) % INPUT_CAPACITY;
        let mut count = 1;
        // 若开启 IUTF8，跳过所有形如 10xxxxxx 的 UTF-8 延续字节，定位到该字符的首字节。
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

    /// 查看未提交编辑尾的最后一个字节（不移除）。
    fn edit_last(&self) -> Option<u8> {
        if self.edit_len() == 0 {
            return None;
        }
        Some(self.bytes[(self.head + INPUT_CAPACITY - 1) % INPUT_CAPACITY])
    }

    /// 清空整个未提交编辑尾（保留已提交的记录不变）。
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

/// 固定容量的回显操作环形队列。
///
/// 输入路径在接收或编辑字符时先将高层回显指令（如普通字节、控制字符、擦除制表符、擦除整行）推入此队列，
/// 随后由 `NttyState::process_echo` 按当前输出列号将其展开为具体的输出字节并送入 `OutputQueue`。
pub struct EchoQueue {
    /// 环形操作项存储数组（容量 `ECHO_CAPACITY` 项）。
    items: [EchoItem; ECHO_CAPACITY],
    /// 队首读出下标（`0..ECHO_CAPACITY`）。
    tail: usize,
    /// 队尾写入下标（`0..ECHO_CAPACITY`）。
    head: usize,
    /// 当前队列中待展开的回显操作项数量（`0..=ECHO_CAPACITY`）。
    len: usize,
}

impl EchoQueue {
    /// 创建空的回显操作队列。
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

    /// 返回队列中待处理的回显操作项数量。
    pub fn len(&self) -> usize {
        self.len
    }

    /// 判断回显操作队列是否为空。
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// 清空所有待处理的回显操作项。
    pub fn clear(&mut self) {
        self.tail = 0;
        self.head = 0;
        self.len = 0;
    }

    /// 向队尾推入一个回显操作项；队列已满时返回 `false`。
    ///
    /// 调用方在修改输入队列前会先通过 `echo_reserve` 预留本次操作所需的最大槽位数，保证回显与输入状态严格同步。
    fn push(&mut self, op: u8, value: u8) -> bool {
        if self.len == ECHO_CAPACITY {
            return false;
        }
        self.items[self.head] = EchoItem { op, value };
        self.head = (self.head + 1) % ECHO_CAPACITY;
        self.len += 1;
        true
    }

    /// 弹出并返回队首的回显操作项。
    fn pop(&mut self) -> Option<EchoItem> {
        if self.len == 0 {
            return None;
        }
        let item = self.items[self.tail];
        self.tail = (self.tail + 1) % ECHO_CAPACITY;
        self.len -= 1;
        Some(item)
    }

    /// 查看队首的回显操作项（不弹出）。
    fn peek(&self) -> Option<EchoItem> {
        if self.len == 0 {
            return None;
        }
        Some(self.items[self.tail])
    }
}

/// 固定容量的终端输出字节环形队列。
///
/// 保存已完成 `OPOST` 输出后处理及回显展开、等待写入底层 UART 发送寄存器的字节流。
pub struct OutputQueue {
    /// 存放待发送字节的环形数组（容量 `OUTPUT_CAPACITY` 字节）。
    bytes: [u8; OUTPUT_CAPACITY],
    /// 队首读出下标（`0..OUTPUT_CAPACITY`）。
    tail: usize,
    /// 队尾写入下标（`0..OUTPUT_CAPACITY`）。
    head: usize,
    /// 当前队列中待发送的字节数（`0..=OUTPUT_CAPACITY`）。
    len: usize,
}

impl OutputQueue {
    /// 创建空的输出字节环形队列。
    pub const fn new() -> Self {
        Self {
            bytes: [0; OUTPUT_CAPACITY],
            tail: 0,
            head: 0,
            len: 0,
        }
    }

    /// 返回队列中当前待发送的字节数。
    pub fn len(&self) -> usize {
        self.len
    }

    /// 判断输出队列是否为空。
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// 返回输出队列剩余可写入的空闲字节数（`OUTPUT_CAPACITY - len`）。
    pub fn space(&self) -> usize {
        OUTPUT_CAPACITY - self.len
    }

    /// 清空全部待发送的输出字节。
    pub fn clear(&mut self) {
        self.tail = 0;
        self.head = 0;
        self.len = 0;
    }

    /// 在队尾追加一个待发送字节；队列已满时返回 `false`。
    pub fn push(&mut self, byte: u8) -> bool {
        if self.len == OUTPUT_CAPACITY {
            return false;
        }
        self.bytes[self.head] = byte;
        self.head = (self.head + 1) % OUTPUT_CAPACITY;
        self.len += 1;
        true
    }

    /// 弹出并返回队首的一个待发送字节。
    pub fn pop(&mut self) -> Option<u8> {
        if self.len == 0 {
            return None;
        }
        let byte = self.bytes[self.tail];
        self.tail = (self.tail + 1) % OUTPUT_CAPACITY;
        self.len -= 1;
        Some(byte)
    }

    /// 查看队首的一个待发送字节（不弹出）。
    pub fn peek(&self) -> Option<u8> {
        if self.len == 0 {
            return None;
        }
        Some(self.bytes[self.tail])
    }
}

/// 处理单个输入采样后产生的就绪状态变化与待投递信号汇总。
///
/// 由于 `NttyState` 运行在自旋锁临界区内，不能直接获取任务图锁发送信号或通知唤醒队列，
/// 因此通过本结构体将副作用返回给外层调用者在释放自旋锁后执行。
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct ReceiveOutcome {
    /// 输入队列的已提交可读状态是否发生变化（需唤醒读等待者 / `POLLIN`）。
    pub read_changed: bool,
    /// 回显、流控或输出状态是否发生变化（需唤醒写等待者 / `POLLOUT` 或触发发送泵）。
    pub write_changed: bool,
    /// 识别到信号控制字符或 `BRKINT` 时需向控制终端前台进程组发送的信号编号（如 `SIGINT`、`SIGQUIT`、`SIGTSTP`）。
    pub signal: Option<u8>,
}

impl ReceiveOutcome {
    /// 根据 `(read_changed, write_changed)` 元组构造不含前台信号的 `ReceiveOutcome`。
    fn changes((read_changed, write_changed): (bool, bool)) -> Self {
        Self {
            read_changed,
            write_changed,
            signal: None,
        }
    }
}

/// `N_TTY` 行规程的完整运行时状态（由端点自旋锁 `Endpoint::state` 保护）。
pub struct NttyState {
    /// 当前生效的 `termios` 终端配置。
    pub termios: Termios,
    /// 终端输入字节与规范行记录环形队列。
    pub input: InputQueue,
    /// 待展开为输出字节的回显操作环形队列。
    pub echo: EchoQueue,
    /// 已完成输出后处理、等待写入串口硬件的输出字节队列。
    pub output: OutputQueue,
    /// 当前终端光标所在的逻辑列号（从 0 开始，用于制表符展开与退格计算）。
    pub column: usize,
    /// 当前规范模式输入行开始时的光标列号（用于退格边界保护与 `ECHOKE` 整行擦除）。
    pub canon_column: usize,
    /// 输出是否因软件流控（收到 `VSTOP`）或 `TCXONC(TCOOFF)` 而处于暂停状态。
    pub stopped: bool,
    /// 输出是否由 `ioctl(TCXONC, TCOOFF)` 显式暂停（为 `true` 时收到 `IXANY` 任意字符不会自动恢复输出）。
    pub tco_stopped: bool,
    /// 高优先级待发送单字节（由 `TCIOFF`/`TCION` 设置的 `VSTOP`/`VSTART`，优先于普通输出队列发送且不受流控暂停阻塞）。
    pub xchar: Option<u8>,
    /// 硬件 FIFO 溢出或软件输入/回显队列满导致的累计丢包计数。
    pub overrun: usize,
    /// 当前处于活跃未挂断状态的打开文件描述（OFD）引用计数。
    pub opens: usize,
    /// 串口调制解调器控制线 DTR（Data Terminal Ready）当前逻辑状态。
    pub dtr: bool,
    /// 串口调制解调器控制线 RTS（Request To Send）当前逻辑状态。
    pub rts: bool,
    /// 最近一次配置到 8250 硬件的 16 位波特率分频系数（单位：时钟周期数）。
    pub divisor: u16,
    /// `VREPRINT`（`^R`）触发的未提交行重印快照状态（当输出队列空间不足时分多轮增量展开）。
    pub pending_reprint: Option<PendingReprint>,
}

/// `VREPRINT` 触发的当前编辑行重印快照。
///
/// 将触发重印时刻的未提交行字节拷贝为独立快照，避免在分批展开回显期间受后续输入编辑指针移动的影响。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PendingReprint {
    /// 未提交编辑行的字节副本数组。
    pub bytes: [u8; INPUT_CAPACITY],
    /// 快照中的有效字节总数（`0..=INPUT_CAPACITY`）。
    pub len: usize,
    /// 已经成功展开并推入输出队列的字节数（`0..=len`）。
    pub done: usize,
}

impl NttyState {
    /// 使用默认 `termios` 配置初始化空的 `N_TTY` 行规程状态。
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

    /// 判断当前是否处于 `FLUSHO` 输出丢弃状态（开启时用户写入和回显直接丢弃而不进入硬件发送）。
    pub fn flusho(&self) -> bool {
        self.termios.lflag & super::termios::FLUSHO != 0
    }

    /// 处理来自 UART 接收环的单个硬件采样（含状态位与数据字节）。
    ///
    /// 在端点自旋锁 `state` 临界区内调用；仅更新行规程内部队列与状态，若触发了前台信号则通过返回值交由调用方在锁外发送。
    pub fn receive_sample(&mut self, sample: RxSample) -> ReceiveOutcome {
        let termios = self.termios;
        // Step 1: 优先处理线路 BREAK 状态。
        if sample.status & UART_LSR_BREAK != 0 {
            return self.receive_break(termios);
        }
        // Step 2: 处理奇偶校验错误或帧错误。
        if sample.status & (UART_LSR_PARITY | UART_LSR_FRAME) != 0 {
            return ReceiveOutcome::changes(self.receive_parity_error(termios, sample.byte));
        }
        // Step 3: 累计硬件接收 FIFO 溢出（Overrun）计数。
        if sample.status & UART_LSR_OVERRUN != 0 {
            self.overrun += 1;
        }
        // Step 4: 若该采样包含有效数据字节，则进入常规字节解释流程。
        if sample.status & DATA_READY == 0 {
            return ReceiveOutcome::default();
        }
        self.receive_byte(termios, sample.byte)
    }

    /// 按 `IGNBRK`、`BRKINT`、`PARMRK` 标志处理接收到的线路 BREAK 事件。
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

    /// 按 `INPCK`、`IGNPAR`、`PARMRK` 标志处理带有奇偶校验错误或帧错误的输入字节。
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

    /// 原子地向输入队列提交 `PARMRK` 错误标记字节序列（如 `[0xff, 0x00, byte]`）。
    ///
    /// 入队前先完整检查回显队列与输入队列是否能容纳整组字节；若空间不足则整组丢弃并计入 `overrun`，绝不写入半截标记。
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

    /// 按标准 `N_TTY` 优先级依次解释单个正常输入字节：
    /// `lnext` 转义 ➔ `ISTRIP` 7 位截断 ➔ `IXON` 流控 ➔ `ISIG` 信号 ➔ `IXANY` 恢复 ➔ `VDISCARD` ➔ 回车换行转换 ➔ `ICANON` 规范编辑 ➔ 普通数据入队。
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

    /// 识别并处理 `IXON` 软件流控字符（`VSTART` 恢复输出、`VSTOP` 暂停输出），返回该字节是否已被流控消费。
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

    /// 处理信号控制字符（`VINTR`、`VQUIT`、`VSUSP`）的本地队列清空（未设置 `NOFLSH` 时）与字符回显。
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

    /// 清空行规程内已接收的输入队列与待发送的输出/回显队列（用于信号触发的本地刷新，不丢弃底层串口尚未读取的后续硬件采样）。
    fn local_flush(&mut self) {
        self.input.clear();
        self.flush_output();
    }

    /// 处理规范模式（`ICANON`）下的行编辑控制字符（`VERASE`/`VKILL`/`VWERASE`/`VLNEXT`/`VREPRINT`）与行提交字符（`\n`/`VEOF`/`VEOL`/`VEOL2`）。
    ///
    /// 返回 `true` 表示该字符已作为规范模式控制或分界字符处理完毕；返回 `false` 表示应作为普通数据字符入队。
    fn receive_canonical(&mut self, termios: Termios, byte: u8) -> bool {
        // Step 1: 识别单字符擦除（VERASE）、整行擦除（VKILL）与单词擦除（VWERASE）。
        if (byte == termios.cc[VERASE] || byte == termios.cc[VKILL]) && byte != DISABLED_CHAR
            || (termios.lflag & IEXTEN != 0 && byte == termios.cc[VWERASE] && byte != DISABLED_CHAR)
        {
            self.erase_input(termios, byte);
            return true;
        }
        // Step 2: 识别字面量转义（VLNEXT, ^V），开启 lnext 状态并回显 "^\b" 提示符。
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
        // Step 3: 识别重印当前编辑行（VREPRINT, ^R）。
        if termios.lflag & IEXTEN != 0
            && termios.lflag & ECHO != 0
            && byte == termios.cc[VREPRINT]
            && byte != DISABLED_CHAR
        {
            // 若上一次重印快照尚未展开完毕，不再覆盖旧快照，直接计入溢出。
            if self.pending_reprint.is_some() {
                self.overrun += 1;
                return true;
            }
            // 预留 3 个回显槽位：VREPRINT 字符本身 + 换行符 + ECHO_REPRINT 指令。
            if !self.echo_reserve(3) {
                self.overrun += 1;
                return true;
            }
            self.reprint_line(termios);
            return true;
        }

        // Step 4: 识别换行符（\n），回显换行并将当前编辑尾提交为可读记录。
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
        // Step 5: 识别文件结束符（VEOF, ^D）；编辑尾非空时提交当前行，为空时提交零长度 EOF 占位。
        if byte == termios.cc[VEOF] && byte != DISABLED_CHAR {
            if 0 < self.input.edit_len() {
                self.input.commit_record();
            } else if !self.input.commit_eof() {
                self.overrun += 1;
            }
            return true;
        }
        // Step 6: 识别附加行结束符（VEOL / VEOL2），入队并提交当前行。
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
            // 行结束符本身即占用预留的终止槽位，只需确认总长度不超过 INPUT_CAPACITY。
            if self.input.pending_eof || INPUT_CAPACITY < self.input.len + count {
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

    /// 将普通数据字节存入输入队列并生成对应回显，返回 `(read_changed, write_changed)`。
    ///
    /// 若回显队列空间不足，则拒绝接收该字节并计入 `overrun`（以回显背压输入，防止吃掉回显）。
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

        // 先写入回显指令再尝试推入输入队列，确保当输入队列满触发 IMAXBEL 时字符回显顺序一致。
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
        // 开启 PARMRK 时，字面量 0xff 需双写为 [0xff, 0xff]；必须预先确认两格空间完整可用。
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

    /// 将错误替换字节直接作为数据字节存入输入队列（跳过控制字符与规范编辑解释）。
    fn queue_input_byte(&mut self, termios: Termios, byte: u8) -> (bool, bool) {
        self.store_data_byte(termios, byte)
    }

    /// 当输入队列已满无法接纳新字节时累加 `overrun`；若设置了 `IMAXBEL` 则尝试向输出队列推入响铃符（`BEL`）。
    fn bell_or_drop(&mut self, termios: Termios) {
        self.overrun += 1;
        if termios.iflag & IMAXBEL != 0 && !self.output.push(BELL) {
            self.overrun += 1;
        }
    }

    /// 执行规范模式下的单字符擦除（`VERASE`）、单词擦除（`VWERASE`）或整行擦除（`VKILL`）。
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
            // 开启 ECHOKE 时：清空当前编辑尾，并推入一条 ECHO_ERASE_LINE 指令由 process_echo 按列差逐步生成退格擦除。
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

    /// 从未提交编辑尾擦除单个字符并推入对应的退格擦除回显指令。
    ///
    /// 在真正从 `input` 移除字符前先预留最多 6 个回显槽位（擦除 `^X` 控制字符需 6 个字节：2 次 `BS SPACE BS`），确保回显不会因队列满而丢失。
    fn erase_one(&mut self, termios: Termios) -> bool {
        let utf8 = termios.iflag & IUTF8 != 0;
        // 预留单字符擦除所需的最大 6 个回显操作槽位。
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

    /// 处理 `VREPRINT`（`^R`）：先回显 `^R\n`，再将当前未提交编辑行保存为 `pending_reprint` 快照并推入 `ECHO_REPRINT` 指令。
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

    /// 向回显队列推入单个原始字节输出指令（`ECHO_BYTE`）。
    fn echo_raw(&mut self, byte: u8) -> bool {
        self.echo.push(ECHO_BYTE, byte)
    }

    /// 向回显队列推入单个字符回显指令；若开启 `ECHOCTL` 且为非制表符的 ASCII 控制字符，则推入 `ECHO_CONTROL`（展开为 `^X`）。
    fn echo_char(&mut self, termios: Termios, byte: u8) -> bool {
        if termios.lflag & ECHOCTL != 0 && byte.is_ascii_control() && byte != b'\t' {
            self.echo.push(ECHO_CONTROL, byte)
        } else {
            self.echo.push(ECHO_BYTE, byte)
        }
    }

    /// 向回显队列推入记录规范行起始列的指令（`ECHO_SET_CANON`）。
    fn echo_set_canon(&mut self) -> bool {
        self.echo.push(ECHO_SET_CANON, 0)
    }

    /// 检查回显队列剩余空间是否足以容纳 `count` 个回显操作项。
    fn echo_reserve(&self, count: usize) -> bool {
        self.echo.len() + count <= ECHO_CAPACITY
    }

    /// 将回显队列中的指令逐项展开为输出字节并推入 `OutputQueue`。
    ///
    /// 返回 `true` 表示仍有未展开完毕的回显指令（因输出队列空间不足或达到单轮步数预算 `ECHO_EXPANSION_BUDGET` 而暂停）。
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

    /// 计算在当前 `termios` 与光标列号 `column` 下，经过 `OPOST` 输出一个字节最多需要占用的输出队列字节数。
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

    /// 判断输出队列当前剩余空间是否足以完整接纳单个字节 `byte` 展开后的输出（若处于 `FLUSHO` 丢弃态则恒返回 `true`）。
    pub fn output_can_accept(&self, termios: Termios, byte: u8) -> bool {
        if self.flusho() {
            return true;
        }
        self.output_required_space(termios, byte) <= self.output.space()
    }

    /// 按当前 `termios` 输出模式（`OPOST`、`ONLCR`、`OCRNL`、`ONOCR`、`ONLRET`、`TAB3`）处理单个字节并推入输出队列，同时更新光标列号。
    ///
    /// 若输出队列剩余空间不足以容纳该字节展开后的全部字节，则不修改任何状态并返回 `false`。
    pub fn output_byte(&mut self, termios: Termios, byte: u8) -> bool {
        if self.output.space() < self.output_required_space(termios, byte) {
            return false;
        }
        if termios.oflag & OPOST == 0 {
            if !byte.is_ascii_control() && termios.iflag & IUTF8 != 0 && byte & 0xc0 == 0x80 {
                // UTF-8 多字节字符的后续延续字节不增加列宽。
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
                if !(byte.is_ascii_control() || termios.iflag & IUTF8 != 0 && byte & 0xc0 == 0x80) {
                    self.column += 1;
                }
                self.output.push(byte)
            }
        }
    }

    /// 切换规范模式（`ICANON`）与非规范模式时重整输入队列的提交边界。
    ///
    /// - 从非规范模式切换到规范模式时，将队列中已有的字节整体标记为一条已提交记录；
    /// - 从规范模式切换到非规范模式时，移除队列中所有的零长度 `VEOF` 占位符，并将全部剩余数据标记为已提交可读。
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

    /// 移除输入环形队列中所有的零长度 `VEOF` 占位槽位，并将有效数据字节与 `eol` 标记向前紧凑压缩。
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

        // 清除压缩后尾部空闲槽位上残留的 eol 与 eof 标记。
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

    /// 计算规范模式下单次读取请求（上限 `max` 字节）可从队首获取的字节数，以及是否在末尾遇到了 `VEOF` 占位符。
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

    /// 规范模式读取预览：将当前首条已提交记录的前缀拷贝到 `out`（不从队列移除），返回 `(拷贝字节数, 是否遇到 EOF)`。
    pub fn peek_canonical(&self, out: &mut [u8]) -> (usize, bool) {
        let (length, eof) = self.canonical_available(out.len());
        for (slot, offset) in out.iter_mut().zip(0..length) {
            *slot = self.input.at(offset);
        }
        (length, eof)
    }

    /// 规范模式读取消费：从输入队列移除已成功拷贝到用户空间的 `delivered` 个字节（若因队首 `VEOF` 返回 0 字节，则消费该 1 字节的 `VEOF` 占位符）。
    pub fn consume_canonical(&mut self, delivered: usize, eof: bool) {
        if eof && delivered == 0 {
            if 0 < self.input.committed_len() && self.input.is_eof(0) {
                self.input.consume(1);
            }
            return;
        }
        self.input.consume(delivered);
    }

    /// 非规范模式读取预览：将队首最多 `out.len()` 个已提交字节拷贝到 `out`（不从队列移除），返回拷贝的字节数。
    pub fn peek_raw(&self, out: &mut [u8]) -> usize {
        let available = self.input.committed_len().min(out.len());
        for (slot, offset) in out.iter_mut().zip(0..available) {
            *slot = self.input.at(offset);
        }
        available
    }

    /// 非规范模式读取消费：从输入队列移除已成功交付给用户的 `delivered` 个字节。
    pub fn consume_raw(&mut self, delivered: usize) {
        self.input.consume(delivered);
    }

    /// 判断当前终端对于 `poll` / `epoll` 是否处于可读就绪状态（`POLLIN`）。
    ///
    /// - 规范模式下：存在已提交记录（`committed_len() > 0`）即就绪；
    /// - 非规范模式下：当 `VTIME == 0 && VMIN > 0` 时需已提交字节数达到 `VMIN`，否则只要有任意字节（`> 0`）即就绪。
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

    /// 判断当前终端对于 `poll` / `epoll` 是否处于可写就绪状态（`POLLOUT`：输出队列有剩余空间）。
    pub fn poll_writable(&self) -> bool {
        0 < self.output.space()
    }

    /// 丢弃输入队列中所有未读字节与未提交编辑状态（对应 `TCIFLUSH`）。
    pub fn flush_input(&mut self) {
        self.input.clear();
    }

    /// 丢弃输出队列、回显队列、高优先级流控字符 `xchar` 以及未完成的重印快照（对应 `TCOFLUSH`）。
    pub fn flush_output(&mut self) {
        self.output.clear();
        self.echo.clear();
        self.xchar = None;
        self.pending_reprint = None;
    }

    /// 判断软件层面的所有待发送数据（输出队列、回显队列、高优先级字符 `xchar`、挂起重印快照）是否已全部排空。
    pub fn output_empty(&self) -> bool {
        self.output.is_empty()
            && self.echo.is_empty()
            && self.xchar.is_none()
            && self.pending_reprint.is_none()
    }
}
