//! `execve` 参数与环境变量快照及栈空间预算管理模块。
//!
//! ## 模块职责与架构定位
//! 本模块在 `execve` 装载初期将用户空间或内核传入的文件路径、命令行参数向量（`argv`）和环境变量向量（`envp`）一次性拷贝到内核堆中，并对总字节数实施严格的预算控制：
//! - **上游调用者**：`sys_execve` 系统调用入口（调用 [`Arguments::from_user`]）与内核初始进程启动路径（调用 [`Arguments::from_kernel`]）。
//! - **下游消费者**：[`super::prepare`] 在展开 `#!` 脚本时调用 [`Arguments::prepend_interpreter`] 改写参数前缀；[`super::stack::make_stack`] 在构建新地址空间初始用户栈时消费此快照。
//!
//! ## 核心机制：参数空间预算与 `#!` 解释器前缀改写
//!
//! ### 1. 参数与环境变量的计费模型
//! 预算上限 `budget()` 取当前主栈软限制（`RLIMIT_STACK`）的 $1/4$，并限制在 `[ARGUMENT_FLOOR (128 KiB), ARGUMENT_CEILING (6 MiB)]` 范围内，单个字符串最大不超过 `STRING_LIMIT`（128 KiB）：
//!
//! ```text
//!   Arguments.bytes 计费构成：
//!   +--------------------------------+-----------------------------------+-----------------------------------+
//!   | filename.len() + 1 (NUL)       | argv 各字符串 (len + 1)           | envp 各字符串 (len + 1)           |
//!   |                                | + argv.len() * sizeof(usize) 指针 | + envp.len() * sizeof(usize) 指针 |
//!   +--------------------------------+-----------------------------------+-----------------------------------+
//!   └── string_bytes() = bytes - (argv.len() + envp.len()) * sizeof(usize)
//!       （精确对应将在用户栈顶高地址区域写入的全部 C 字符串与终止符总字节数）
//! ```
//!
//! ### 2. Shebang（`#!`）脚本解释器前缀改写规则
//! 当可执行文件以 `#!interpreter [optional-arg]` 开头时，[`Arguments::prepend_interpreter`] 沿用同一栈预算改写 `argv`：
//! - **第 0 层（首次遇到脚本）**：将原 `argv[0]` 替换为脚本文件自身的执行路径 `filename`，再在最前面插入 `interpreter`（及可选的 `optional` 参数），同时保持 `filename`（即 `AT_EXECFN`）不变；
//! - **后续嵌套层（`depth > 0`）**：上一层的解释器路径已位于 `argv[0]`，直接将其后移并在头部插入新一层解释器路径及可选参数。
//!
//! ## 并发与内存安全契约
//! - **单次读取快照**：[`Arguments::from_user`] 对用户空间中的每个指针槽位和字符串仅顺序读取一次；一旦拷贝入内核堆，后续的 ELF 解析与新用户栈构建均不再访问旧地址空间，防止多线程并发修改用户态参数引发检查时与使用时不一致（TOCTOU）。
//! - **可失败内存分配（Fallible Allocation）**：所有内核 `Vec` 扩容均先通过 `try_reserve` / `try_reserve_exact` 预留容量，超预算返回 [`ExecError::ArgumentsTooBig`]（`E2BIG`），内核堆分配失败返回 [`ExecError::NoMemory`]（`ENOMEM`），绝不因用户传入超大参数触发内核 OOM panic。

use alloc::vec::Vec;
use core::mem::size_of;

use crate::fs::FsError;
use crate::mm::space::{Mm, VmError};
use crate::mm::uaccess::CStringError;
use crate::mm::PAGE_SIZE;

use super::{ExecError, MAX_PATH_BYTES};

/// 参数与环境变量预算保底下限（128 KiB，对应 Linux `ARG_MAX` 传统基线）。
const ARGUMENT_FLOOR: usize = 128 * 1024;
/// 参数与环境变量预算绝对上限（默认 8 MiB 主栈的 $3/4$，即 6 MiB）。
const ARGUMENT_CEILING: u64 =
    super::DEFAULT_STACK_BYTES / STACK_LIMIT_FRACTION * (STACK_LIMIT_FRACTION - 1);
/// 单个参数或环境变量字符串的最大字节长度（32 页，即 128 KiB，对应 Linux `MAX_ARG_STRLEN`）。
const STRING_LIMIT: usize = 32 * PAGE_SIZE;
/// 参数预算占主栈软限制大小的分母比例（$1/4$）。
const STACK_LIMIT_FRACTION: u64 = 4;

/// 已拷贝到内核空间的可执行文件路径、参数列表、环境变量列表及栈预算记账快照。
///
/// 内部保存的各个 `Vec<u8>` 字符串均不含末尾 NUL 字节；
/// `bytes` 字段则将每个字符串的末尾 NUL 终止符以及 `argv`/`envp` 有效指针槽位一并计入预算。
pub(crate) struct Arguments {
    /// 原始请求执行的文件路径字节（不含末尾 NUL），用于写入初始栈 `AT_EXECFN` 辅助向量。
    pub(super) filename: Vec<u8>,
    /// 命令行参数字符串列表（每项不含末尾 NUL，保证至少包含 `argv[0]`）。
    pub(super) argv: Vec<Vec<u8>>,
    /// 环境变量字符串列表（每项不含末尾 NUL）。
    pub(super) envp: Vec<Vec<u8>>,
    /// 采集参数时的任务主栈软限制快照（单位：字节）。
    pub(super) stack_limit: u64,
    /// 当前已累计消耗的参数预算字节数（含 `filename`、`argv`/`envp` 字符串及 NUL 终止符、有效指针槽位）。
    bytes: usize,
}

impl Arguments {
    /// 从内核态切片构造执行参数快照，并记录主栈软限制。
    ///
    /// 供内核直接启动初始用户进程或测试代码调用。若传入的 `argv` 为空，会自动补充一个空字符串作为 `argv[0]`。
    pub(crate) fn from_kernel(
        filename: &[u8],
        argv: &[&[u8]],
        envp: &[&[u8]],
        stack_limit: u64,
    ) -> Result<Self, ExecError> {
        validate_filename(filename)?;
        let mut args = Self::new(copy_bytes(filename)?, stack_limit);
        for string in argv {
            args.push_kernel(string, true)?;
        }
        args.ensure_argv0()?;
        for string in envp {
            args.push_kernel(string, false)?;
        }
        Ok(args)
    }

    /// 从当前用户地址空间读取并拷贝命令行参数向量（`argv`）与环境变量向量（`envp`）。
    ///
    /// 在系统调用上下文中调用。每个用户态指针和 C 字符串仅读取一次；
    /// 若 `argv` 指针为空（`0`）或指向的首个元素为 `NULL`，会自动合成一个空字符串作为 `argv[0]`。
    pub(crate) fn from_user(
        mm: &Mm,
        filename: Vec<u8>,
        argv: usize,
        envp: usize,
        stack_limit: u64,
    ) -> Result<Self, ExecError> {
        validate_filename(&filename)?;
        let mut args = Self::new(filename, stack_limit);
        args.read_vector(mm, argv, true)?;
        args.ensure_argv0()?;
        args.read_vector(mm, envp, false)?;
        Ok(args)
    }

    /// 返回最初请求执行的文件路径字节切片（不含末尾 NUL）。
    ///
    /// 即使后续经过 `#!` 脚本解释器展开改写了 `argv[0]`，该路径仍保持不变并用于填充 `AT_EXECFN`。
    pub(crate) fn filename(&self) -> &[u8] {
        &self.filename
    }

    /// 返回所有需要在初始用户栈高地址字符串区写入的字节总数（含 `filename`、`argv`、`envp` 本体及其各自的 NUL 终止符）。
    pub(super) fn string_bytes(&self) -> usize {
        self.bytes - (self.argv.len() + self.envp.len()) * size_of::<usize>()
    }

    /// 在 `#!` 脚本展开时，将解释器路径、可选参数及脚本路径改写到 `argv` 头部。
    ///
    /// 先完成参数数量上限、单串长度、总预算以及内核堆容量的预检查与分配，最后再原地更新 `argv` 与计费字节数；
    /// 若中途失败，原有的参数快照保持不变。
    pub(super) fn prepend_interpreter(
        &mut self,
        interpreter: &[u8],
        optional: Option<&[u8]>,
        original: bool,
    ) -> Result<(), ExecError> {
        let script = if original {
            &self.filename
        } else {
            &self.argv[0]
        };
        // Step 1: 校验新增参数后的总个数、单串长度上限以及更新后的总预算字节数。
        let additional = 1 + usize::from(optional.is_some());
        let argc = self
            .argv
            .len()
            .checked_add(additional)
            .filter(|count| *count <= i32::MAX as usize)
            .ok_or(ExecError::ArgumentsTooBig)?;
        if [Some(interpreter), optional, Some(script.as_slice())]
            .iter()
            .flatten()
            .any(|string| STRING_LIMIT <= string.len())
        {
            return Err(ExecError::ArgumentsTooBig);
        }
        let inserted = interpreter.len()
            + script.len()
            + 2
            + optional.map_or(0, |string| string.len() + 1)
            + additional * size_of::<usize>();
        let bytes = (self.bytes - self.argv[0].len() - 1)
            .checked_add(inserted)
            .filter(|bytes| *bytes <= self.budget())
            .ok_or(ExecError::ArgumentsTooBig)?;

        // Step 2: 预先分配解释器路径、可选参数及脚本路径的内核堆内存并预留 argv 槽位。
        let interpreter = copy_bytes(interpreter)?;
        let optional = optional.map(copy_bytes).transpose()?;
        let script = if original && self.argv[0] != self.filename {
            Some(copy_bytes(script)?)
        } else {
            None
        };
        self.argv
            .try_reserve_exact(argc - self.argv.len())
            .map_err(|_| ExecError::NoMemory)?;

        // Step 3: 将原 argv[0] 替换为脚本路径并右移，在头部写入解释器路径与可选参数。
        let script = script.unwrap_or_else(|| core::mem::take(&mut self.argv[0]));
        self.argv[0] = interpreter;
        self.argv.push(script);
        self.argv[1..].rotate_right(1);
        if let Some(optional) = optional {
            self.argv.push(optional);
            self.argv[1..].rotate_right(1);
        }
        self.bytes = bytes;
        Ok(())
    }

    /// 使用已校验的文件路径和主栈软限制创建空的 [`Arguments`]，并预先计入 `filename`（含 NUL）及 `argv[0]` 所需的首个指针槽位。
    fn new(filename: Vec<u8>, stack_limit: u64) -> Self {
        let bytes = filename.len() + 1 + size_of::<usize>();
        Self {
            filename,
            argv: Vec::new(),
            envp: Vec::new(),
            stack_limit,
            bytes,
        }
    }

    /// 计算当前允许使用的最大参数与环境变量预算字节数。
    ///
    /// 取 `stack_limit / 4`，向上钳位至 [`ARGUMENT_CEILING`]，向下保底至 [`ARGUMENT_FLOOR`]。
    fn budget(&self) -> usize {
        ((self.stack_limit / STACK_LIMIT_FRACTION).min(ARGUMENT_CEILING) as usize)
            .max(ARGUMENT_FLOOR)
    }

    /// 在压入下一个参数或环境变量前，校验向量元素个数上限并计算该字符串允许的最大字节长度（含末尾 NUL）。
    fn string_limit(&self, argument: bool) -> Result<usize, ExecError> {
        let count = if argument {
            self.argv.len()
        } else {
            self.envp.len()
        };
        if i32::MAX as usize <= count {
            return Err(ExecError::ArgumentsTooBig);
        }
        let pointer = if argument && count == 0 {
            0
        } else {
            size_of::<usize>()
        };
        self.budget()
            .checked_sub(self.bytes + pointer)
            .filter(|remaining| 0 < *remaining)
            .map(|remaining| remaining.min(STRING_LIMIT))
            .ok_or(ExecError::ArgumentsTooBig)
    }

    /// 校验内核态字符串切片的长度预算与内嵌 NUL 字节，深拷贝后压入 `argv` 或 `envp`。
    fn push_kernel(&mut self, string: &[u8], argument: bool) -> Result<(), ExecError> {
        if self.string_limit(argument)? <= string.len() {
            return Err(ExecError::ArgumentsTooBig);
        }
        if string.contains(&0) {
            return Err(ExecError::Format);
        }
        self.push_owned(copy_bytes(string)?, argument)
    }

    /// 将已通过长度和预算校验的独立字符串压入目标向量，并累加对应的字符串、终止符与指针计费字节数。
    fn push_owned(&mut self, string: Vec<u8>, argument: bool) -> Result<(), ExecError> {
        let pointer = if argument && self.argv.is_empty() {
            0
        } else {
            size_of::<usize>()
        };
        let vector = if argument {
            &mut self.argv
        } else {
            &mut self.envp
        };
        vector.try_reserve(1).map_err(|_| ExecError::NoMemory)?;

        self.bytes += string.len() + 1 + pointer;
        vector.push(string);
        Ok(())
    }

    /// 确保 `argv` 至少包含一个元素；若调用方传入空 `argv`，则遵循 Linux ABI 压入一个空字符串作为 `argv[0]`。
    fn ensure_argv0(&mut self) -> Result<(), ExecError> {
        if self.argv.is_empty() {
            self.push_kernel(&[], true)?;
        }
        Ok(())
    }

    /// 从用户空间地址 `address` 处顺序读取以 `NULL`（`0`）结尾的字符串指针数组，并将每个 C 字符串拷贝入内核。
    ///
    /// 若数组首地址 `address` 为 `0`（即用户传入 `NULL` 指针），直接视为空列表返回成功。
    fn read_vector(&mut self, mm: &Mm, address: usize, argument: bool) -> Result<(), ExecError> {
        if address == 0 {
            return Ok(());
        }

        let mut slot = address;
        loop {
            let pointer = mm.read_usize(slot)?;
            if pointer == 0 {
                return Ok(());
            }
            let limit = self.string_limit(argument)?;
            let string = mm
                .read_c_string(pointer, limit)
                .map_err(|error| match error {
                    CStringError::Vm(error) => ExecError::Vm(error),
                    CStringError::NoMemory => ExecError::NoMemory,
                    CStringError::Unterminated => ExecError::ArgumentsTooBig,
                })?;
            self.push_owned(string, argument)?;
            slot = slot
                .checked_add(size_of::<usize>())
                .ok_or(ExecError::Vm(VmError::InvalidRange))?;
        }
    }
}

/// 校验可执行文件路径长度是否超限、是否为空路径或包含非法的内嵌 NUL 字节。
fn validate_filename(filename: &[u8]) -> Result<(), ExecError> {
    if MAX_PATH_BYTES <= filename.len() {
        return Err(ExecError::ArgumentsTooBig);
    }
    if filename.is_empty() {
        return Err(ExecError::File(FsError::NotFound));
    }
    if filename.contains(&0) {
        return Err(ExecError::Format);
    }
    Ok(())
}

/// 使用可失败分配（`try_reserve_exact`）将字节切片深拷贝为独立的 `Vec<u8>`。
fn copy_bytes(bytes: &[u8]) -> Result<Vec<u8>, ExecError> {
    let mut owned = Vec::new();
    owned
        .try_reserve_exact(bytes.len())
        .map_err(|_| ExecError::NoMemory)?;
    owned.extend_from_slice(bytes);
    Ok(owned)
}
