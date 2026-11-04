//! 执行参数拥有一次字节快照和同次主栈软限额，解释器改写沿用同一预算。

use alloc::vec::Vec;
use core::mem::size_of;

use crate::fs::FsError;
use crate::mm::space::{Mm, VmError};
use crate::mm::uaccess::CStringError;
use crate::mm::PAGE_SIZE;

use super::{ExecError, MAX_PATH_BYTES};

const ARGUMENT_FLOOR: usize = 128 * 1024;
const ARGUMENT_CEILING: u64 =
    super::DEFAULT_STACK_BYTES / STACK_LIMIT_FRACTION * (STACK_LIMIT_FRACTION - 1);
const STRING_LIMIT: usize = 32 * PAGE_SIZE;
const STACK_LIMIT_FRACTION: u64 = 4;

/// 原始字符串不含 NUL；预算只计文件名、字符串终止符和有效参数指针。
pub(crate) struct Arguments {
    pub(super) filename: Vec<u8>,
    pub(super) argv: Vec<Vec<u8>>,
    pub(super) envp: Vec<Vec<u8>>,
    pub(super) stack_limit: u64,
    bytes: usize,
}

impl Arguments {
    /// 拷贝内核参数并保存本次软栈限额，全部动态拥有均通过可失败预留取得。
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

    /// 每个用户指针和字符串只读取一次，后续构栈不再访问旧用户参数。
    /// 并发写入不保证跨字符串原子快照；访问、预算和分配错误分别返回。
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

    /// 原执行路径保持不变，解释器替换 argv[0] 不改变 AT_EXECFN。
    pub(crate) fn filename(&self) -> &[u8] {
        &self.filename
    }

    /// 已有计费减去有效指针后就是构栈所需的全部字符串和 NUL 字节。
    pub(super) fn string_bytes(&self) -> usize {
        self.bytes - (self.argv.len() + self.envp.len()) * size_of::<usize>()
    }

    /// 新前缀先取得完整预算和容量，再移动原参数；失败保持已有快照内容。
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

    /// 空 argv 先预留一个有效指针，后续合成空 argv[0] 时只扣字符串终止符。
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

    /// 总量按软栈四分之一计算，保留固定下界并截到参数上界。
    fn budget(&self) -> usize {
        ((self.stack_limit / STACK_LIMIT_FRACTION).min(ARGUMENT_CEILING) as usize)
            .max(ARGUMENT_FLOOR)
    }

    /// 在读取或分配字符串前扣除本项新增指针，同时保留每向量的正 i32 计数域。
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

    /// 内核字符串先核字节边界与内嵌 NUL，再取得唯一字符串拥有。
    fn push_kernel(&mut self, string: &[u8], argument: bool) -> Result<(), ExecError> {
        if self.string_limit(argument)? <= string.len() {
            return Err(ExecError::ArgumentsTooBig);
        }
        if string.contains(&0) {
            return Err(ExecError::Format);
        }
        self.push_owned(copy_bytes(string)?, argument)
    }

    /// 字符串已符合单串和剩余预算，预留成功后才提交向量与计费。
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

    /// 空或 NULL argv 按初始栈 ABI 产生一个空 argv[0]。
    fn ensure_argv0(&mut self) -> Result<(), ExecError> {
        if self.argv.is_empty() {
            self.push_kernel(&[], true)?;
        }
        Ok(())
    }

    /// NULL 数组无需解引用，末项 NULL 和各有效指针均只读取一次。
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

/// 空路径保留查找错误，内嵌 NUL 和路径长度在读取 argv 前裁决。
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

/// 复制已核界的原始字节，分配失败保留为可返回错误。
fn copy_bytes(bytes: &[u8]) -> Result<Vec<u8>, ExecError> {
    let mut owned = Vec::new();
    owned
        .try_reserve_exact(bytes.len())
        .map_err(|_| ExecError::NoMemory)?;
    owned.extend_from_slice(bytes);
    Ok(owned)
}
