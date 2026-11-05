//! RV64 信号帧以固定小端字节布局保存用户现场，扩展描述位于完整浮点 union 内。

use crate::arch::{riscv64::user::UserContext, STACK_ALIGNMENT};
use crate::mm::space::{Mm, VmError};
use crate::task::signal::{SignalFrameArgs, SignalFrameRestore, SA_ONSTACK};

/// 接受备用信号栈的最小容量；固定帧之外保留处理函数的栈空间。
pub(crate) const SIGNAL_MIN_STACK: usize = 2048;

// RV64 stack_t 为 24 字节，sigset 保留区为 128 字节，mcontext 按 16 字节对齐。
const INFO_SIZE: usize = 128;
const ABI_WORD_BYTES: usize = core::mem::size_of::<u64>();
const ALTSTACK_SIZE: usize = 3 * ABI_WORD_BYTES;
const SIGNAL_MASK_RESERVED: usize = 128;
const UCONTEXT: usize = INFO_SIZE;
const ALTSTACK: usize = UCONTEXT + 2 * ABI_WORD_BYTES;
const ALTSTACK_FLAGS: usize = ALTSTACK + ABI_WORD_BYTES;
const ALTSTACK_BYTES: usize = ALTSTACK + 2 * ABI_WORD_BYTES;
const MASK: usize = ALTSTACK + ALTSTACK_SIZE;
const MCONTEXT: usize =
    (MASK + SIGNAL_MASK_RESERVED + STACK_ALIGNMENT - 1) & !(STACK_ALIGNMENT - 1);
const FP: usize = MCONTEXT + 32 * ABI_WORD_BYTES;
// Q 现场包含 64 个 64 位字、FCSR 和三个保留字；union 按 16 字节对齐。
const FP_UNION_SIZE: usize = 64 * ABI_WORD_BYTES + 4 * core::mem::size_of::<u32>();
const FCSR: usize = FP + 32 * ABI_WORD_BYTES;
const EXT_RESERVED: usize = FP + 129 * core::mem::size_of::<u32>();
const EXT_HEADER: usize = EXT_RESERVED + core::mem::size_of::<u32>();
const FRAME_SIZE: usize = FP + FP_UNION_SIZE;

impl UserContext {
    /// 在普通任务中写入完整帧，缺页或 COW 可睡眠；成功后才改动处理函数入口现场。
    pub(crate) fn setup_signal(&mut self, mm: &Mm, args: &SignalFrameArgs) -> Result<(), VmError> {
        let address = frame_address(self.gpr[2], args)?;
        let mut frame = [0u8; FRAME_SIZE];

        args.info.encode(&mut frame[..INFO_SIZE]);
        put_u64(&mut frame, ALTSTACK, args.altstack.base as u64);
        put_u32(&mut frame, ALTSTACK_FLAGS, args.altstack.flags);
        put_u64(&mut frame, ALTSTACK_BYTES, args.altstack.size as u64);
        put_u64(&mut frame, MASK, args.restore_mask);
        put_u64(&mut frame, MCONTEXT, self.pc as u64);
        for register in 1..32 {
            put_u64(
                &mut frame,
                MCONTEXT + register * 8,
                self.gpr[register] as u64,
            );
        }
        for register in 0..32 {
            put_u64(&mut frame, FP + register * 8, self.fpr[register]);
        }
        put_u32(&mut frame, FCSR, self.fcsr as u32);

        // 清零缓冲同时给出 uc_flags/link、全部填充和无 V 的 END_MAGIC/END_HDR_SIZE。
        let progress = mm.copyout(address, &frame);
        if let Some(error) = progress.error {
            return Err(error);
        }
        debug_assert_eq!(progress.copied, FRAME_SIZE);

        self.pc = args.action.handler;
        self.gpr[1] = args.trampoline;
        self.gpr[2] = address;
        self.gpr[10] = args.info.signum as usize;
        // 两个指针始终传递，单参数处理函数自然忽略 a1/a2。
        self.gpr[11] = address;
        self.gpr[12] = address + UCONTEXT;
        Ok(())
    }

    /// 完整读取并解码当前 SP 上的帧；用户可修改返回现场，失败不改动当前现场。
    pub(crate) fn restore_signal(&self, mm: &Mm) -> Result<SignalFrameRestore, VmError> {
        if self.gpr[2] & (STACK_ALIGNMENT - 1) != 0 {
            return Err(VmError::InvalidRange);
        }
        let mut frame = [0u8; FRAME_SIZE];
        let progress = mm.copyin(&mut frame, self.gpr[2]);
        if let Some(error) = progress.error {
            return Err(error);
        }
        debug_assert_eq!(progress.copied, FRAME_SIZE);

        // 此后端没有向量现场；未知扩展、非零保留字和错误终止大小均是坏帧。
        if get_u32(&frame, EXT_RESERVED) != 0
            || get_u32(&frame, EXT_HEADER) != 0
            || get_u32(&frame, EXT_HEADER + 4) != 0
        {
            return Err(VmError::InvalidRange);
        }

        // 新对象只包含用户寄存器，受信续接、异常 CSR 和 CpuLocal 不从用户字节恢复。
        let mut context = UserContext::new(get_u64(&frame, MCONTEXT) as usize, 0);
        for register in 1..32 {
            context.gpr[register] = get_u64(&frame, MCONTEXT + register * 8) as usize;
        }
        for register in 0..32 {
            context.fpr[register] = get_u64(&frame, FP + register * 8);
        }
        context.fcsr = get_u32(&frame, FCSR) as usize;
        Ok(SignalFrameRestore {
            context,
            mask: get_u64(&frame, MASK),
            // 备用栈的更新规则由持有原栈状态的提交者按恢复后的 SP 应用。
            altstack: crate::task::signal::AltStack {
                base: get_u64(&frame, ALTSTACK) as usize,
                size: get_u64(&frame, ALTSTACK_BYTES) as usize,
                flags: get_u32(&frame, ALTSTACK_FLAGS),
            },
            pending_signal: None,
        })
    }
}

/// 按真实 SP 判断嵌套或备用栈切换，减去固定帧后向下保持 16 字节对齐。
fn frame_address(sp: usize, args: &SignalFrameArgs) -> Result<usize, VmError> {
    let stack = args.altstack;
    let nested = stack.on_stack(sp);
    let switch_stack = args.action.flags & SA_ONSTACK != 0 && stack.enabled() && !nested;
    let top = if switch_stack {
        stack
            .base
            .checked_add(stack.size)
            .ok_or(VmError::InvalidRange)?
    } else {
        sp
    };
    let address =
        top.checked_sub(FRAME_SIZE).ok_or(VmError::InvalidRange)? & !(STACK_ALIGNMENT - 1);
    if (nested || switch_stack) && !stack.on_stack(address) {
        return Err(VmError::InvalidRange);
    }
    Ok(address)
}

/// 写入固定帧偏移上的小端机器字，不借用 Rust 结构体填充。
fn put_u64(frame: &mut [u8; FRAME_SIZE], offset: usize, value: u64) {
    frame[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
}

/// 写入固定帧偏移上的小端控制字段。
fn put_u32(frame: &mut [u8; FRAME_SIZE], offset: usize, value: u32) {
    frame[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

/// 从已经完整复制的帧解码固定偏移机器字。
fn get_u64(frame: &[u8; FRAME_SIZE], offset: usize) -> u64 {
    let mut bytes = [0; 8];
    bytes.copy_from_slice(&frame[offset..offset + 8]);
    u64::from_le_bytes(bytes)
}

/// 从已经完整复制的帧解码固定偏移控制字段。
fn get_u32(frame: &[u8; FRAME_SIZE], offset: usize) -> u32 {
    let mut bytes = [0; 4];
    bytes.copy_from_slice(&frame[offset..offset + 4]);
    u32::from_le_bytes(bytes)
}
