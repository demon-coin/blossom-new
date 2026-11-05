//! 用户栈保存 CPU 实际支持的最宽浮点扩展；内核续接和特权 CSR 不进入用户帧。

use core::convert::TryInto;

use crate::arch::loongarch64::register as reg;
use crate::arch::loongarch64::user::UserContext;
use crate::arch::loongarch64::user_fp_lanes;
use crate::arch::{UserAccess, STACK_ALIGNMENT, USER_ADDRESS_END};
use crate::mm::space::{Mm, VmError};
use crate::task::signal::{
    AltStack, SignalFrameArgs, SignalFrameRestore, SignalInfo, SignalPayload, SA_ONSTACK, SIGFPE,
    SS_DISABLE, UNBLOCKABLE,
};

/// 保持用户 ABI 的最小备用栈；最大 LASX 帧连同对齐消耗仍小于此容量。
pub(crate) const SIGNAL_MIN_STACK: usize = 4096;

const WORD_BYTES: usize = core::mem::size_of::<u64>();
const CONTEXT_INFO_ALIGNMENT: usize = 16;
const SIGMASK_RESERVED_BYTES: usize = 128;
const STACK_FLAGS: usize = STACK + WORD_BYTES;
const STACK_SIZE: usize = STACK + 2 * WORD_BYTES;
const FP_TRAILER_BYTES: usize = 2 * WORD_BYTES;
const PADDING_COPY_CHUNK: usize = 128;

const INFO_SIZE: usize = 128;
const UCONTEXT: usize = INFO_SIZE;
const STACK: usize = UCONTEXT + 2 * WORD_BYTES;
const MASK: usize = STACK + 3 * WORD_BYTES;
const MCONTEXT: usize =
    (MASK + SIGMASK_RESERVED_BYTES + CONTEXT_INFO_ALIGNMENT - 1) & !(CONTEXT_INFO_ALIGNMENT - 1);
const REGISTERS: usize = MCONTEXT + WORD_BYTES;
const FLAGS: usize = REGISTERS + 32 * WORD_BYTES;
const EXTCONTEXT: usize = (FLAGS + core::mem::size_of::<u32>() + CONTEXT_INFO_ALIGNMENT - 1)
    & !(CONTEXT_INFO_ALIGNMENT - 1);
const SCTX_INFO_SIZE: usize = CONTEXT_INFO_ALIGNMENT;
const MAX_BODY_SIZE: usize =
    core::mem::size_of::<[crate::arch::loongarch64::user::FpRegister; 32]>() + FP_TRAILER_BYTES;
const MAX_FRAME_SIZE: usize =
    EXTCONTEXT + SCTX_INFO_SIZE + MAX_BODY_SIZE + CONTEXT_INFO_ALIGNMENT + SCTX_INFO_SIZE;
const _: () = assert!(MAX_FRAME_SIZE < SIGNAL_MIN_STACK);

const FPU_MAGIC: u32 = 0x4650_5501;
const LSX_MAGIC: u32 = 0x5358_0001;
const LASX_MAGIC: u32 = 0x4153_5801;
const SC_USED_FP: u32 = 1;
const SC_ADDRERR_RD: u32 = 1 << 30;
const SC_ADDRERR_WR: u32 = 1 << 31;

/// 描述一个互斥的浮点扩展 body；寄存器连续存放，FCC/FCSR 紧随有效 lanes。
struct FpLayout {
    lanes: usize,
}

impl FpLayout {
    /// 按真实 CPU 能力选择完整保存宽度，无 FP 时仅生成 END。
    fn current() -> Self {
        Self {
            lanes: user_fp_lanes(),
        }
    }

    /// 从用户 magic 选择受支持的 ABI，不接受未知或本 CPU 无法恢复的扩展。
    fn decode(magic: u32) -> Result<Self, VmError> {
        let lanes = match magic {
            FPU_MAGIC => 1,
            LSX_MAGIC => 2,
            LASX_MAGIC => 4,
            _ => return Err(VmError::InvalidRange),
        };
        if user_fp_lanes() < lanes {
            return Err(VmError::InvalidRange);
        }
        Ok(Self { lanes })
    }

    /// 返回该互斥扩展的用户 ABI magic。
    fn magic(&self) -> u32 {
        match self.lanes {
            1 => FPU_MAGIC,
            2 => LSX_MAGIC,
            4 => LASX_MAGIC,
            _ => 0,
        }
    }

    /// 返回寄存器之后 FCC 的字节偏移。
    fn registers_size(&self) -> usize {
        32 * self.lanes * WORD_BYTES
    }

    /// 返回含尾部 ABI padding 的具体 body 大小。
    fn body_size(&self) -> usize {
        self.registers_size() + FP_TRAILER_BYTES
    }

    /// LASX body 需 32 字节对齐，其余扩展沿记录头的 16 字节边界。
    fn alignment(&self) -> usize {
        if self.lanes == 4 {
            core::mem::align_of::<crate::arch::loongarch64::user::FpRegister>()
        } else {
            CONTEXT_INFO_ALIGNMENT
        }
    }
}

impl UserContext {
    /// 在普通任务中完整写入用户帧后改变 handler 现场；逐页 uaccess 可缺页或处理 COW。
    /// 调用者不得持有 siglock、IRQ guard 或用户现场借用外的短锁。
    pub(crate) fn setup_signal(&mut self, mm: &Mm, args: &SignalFrameArgs) -> Result<(), VmError> {
        let layout = FpLayout::current();
        let (sp, frame_size, record_size) = frame_address(self.gpr[3], args, &layout)?;
        let mut bytes = [0u8; MAX_FRAME_SIZE];

        args.info.encode(&mut bytes[..INFO_SIZE]);
        put_u64(&mut bytes, STACK, args.altstack.base as u64);
        put_u32(&mut bytes, STACK_FLAGS, args.altstack.flags);
        put_u64(&mut bytes, STACK_SIZE, args.altstack.size as u64);
        put_u64(&mut bytes, MASK, args.restore_mask);
        put_u64(&mut bytes, MCONTEXT, self.pc as u64);
        for (index, register) in self.gpr.iter().enumerate().skip(1) {
            put_u64(&mut bytes, REGISTERS + index * 8, *register as u64);
        }
        put_u32(&mut bytes, FLAGS, context_flags(args, layout.lanes != 0));
        if layout.lanes != 0 {
            put_u32(&mut bytes, EXTCONTEXT, layout.magic());
            put_u32(&mut bytes, EXTCONTEXT + 4, record_size as u32);
            let body = EXTCONTEXT + SCTX_INFO_SIZE;
            for (index, register) in self.fpr.iter().enumerate() {
                for lane in 0..layout.lanes {
                    put_u64(
                        &mut bytes,
                        body + (index * layout.lanes + lane) * 8,
                        register.lanes[lane],
                    );
                }
            }
            put_u64(&mut bytes, body + layout.registers_size(), self.fcc as u64);
            put_u32(
                &mut bytes,
                body + layout.registers_size() + 8,
                self.fcsr as u32,
            );
        }

        // 清零缓冲同时初始化全部 ABI padding、uc_flags/link 和最终 magic=0/size=0。
        let progress = mm.copyout(sp, &bytes[..frame_size]);
        if let Some(error) = progress.error {
            return Err(error);
        }
        assert_eq!(
            progress.copied, frame_size,
            "successful signal frame copy is incomplete"
        );

        self.gpr[1] = args.trampoline;
        self.gpr[3] = sp;
        self.gpr[4] = args.info.signum as usize;
        self.gpr[5] = sp;
        self.gpr[6] = sp + UCONTEXT;
        self.pc = args.action.handler;
        Ok(())
    }

    /// 从当前用户 SP 解码完整候选；任何复制或记录错误都不修改当前现场及信号状态。
    /// 完整验证成功后若 FCSR 有已启用异常，先回写清除 cause 的原槽，失败仍不提交候选。
    pub(crate) fn restore_signal(&self, mm: &Mm) -> Result<SignalFrameRestore, VmError> {
        let sp = self.gpr[3];
        if sp & (STACK_ALIGNMENT - 1) != 0 {
            return Err(VmError::InvalidRange);
        }
        let mut bytes = [0u8; EXTCONTEXT + SCTX_INFO_SIZE];
        copyin_exact(mm, sp, &mut bytes)?;
        let flags = get_u32(&bytes, FLAGS);
        if flags & !(SC_USED_FP | SC_ADDRERR_RD | SC_ADDRERR_WR) != 0 {
            return Err(VmError::InvalidRange);
        }

        let altstack = decode_altstack(&bytes);
        // new 清除异常 CSR 与 kernel_sp；用户帧只提供整数、PC 和扩展寄存器。
        let mut context = Self::new(get_u64(&bytes, MCONTEXT) as usize, 0);
        for (index, register) in context.gpr.iter_mut().enumerate().skip(1) {
            *register = get_u64(&bytes, REGISTERS + index * 8) as usize;
        }

        let magic = get_u32(&bytes, EXTCONTEXT);
        let size = get_u32(&bytes, EXTCONTEXT + 4) as usize;
        let mut fcsr_address = None;
        if magic == 0 {
            if size != 0 || flags & SC_USED_FP != 0 {
                return Err(VmError::InvalidRange);
            }
        } else {
            let layout = FpLayout::decode(magic)?;
            let record = sp.checked_add(EXTCONTEXT).ok_or(VmError::InvalidRange)?;
            let body = record
                .checked_add(SCTX_INFO_SIZE)
                .ok_or(VmError::InvalidRange)?;
            if size < SCTX_INFO_SIZE + layout.body_size()
                || size & (CONTEXT_INFO_ALIGNMENT - 1) != 0
                || body & (layout.alignment() - 1) != 0
            {
                return Err(VmError::InvalidRange);
            }
            let end = record.checked_add(size).ok_or(VmError::InvalidRange)?;
            let frame_end = end
                .checked_add(SCTX_INFO_SIZE)
                .ok_or(VmError::InvalidRange)?;
            if USER_ADDRESS_END < frame_end {
                return Err(VmError::InvalidRange);
            }
            let mut fp = [0u8; MAX_BODY_SIZE];
            copyin_exact(mm, body, &mut fp[..layout.body_size()])?;
            // 合法放大记录的 padding 也完整访问，不能跳过用户声明但未映射的帧范围。
            copyin_padding(mm, body + layout.body_size(), end)?;
            let mut terminator = [0u8; SCTX_INFO_SIZE];
            copyin_exact(mm, end, &mut terminator)?;
            // 当前 ABI 只持一个互斥 FP/LSX/LASX 记录，重复和其它扩展不能覆盖候选。
            if get_u32(&terminator, 0) != 0 || get_u32(&terminator, 4) != 0 {
                return Err(VmError::InvalidRange);
            }
            for (index, register) in context.fpr.iter_mut().enumerate() {
                for lane in 0..layout.lanes {
                    register.lanes[lane] = get_u64(&fp, (index * layout.lanes + lane) * 8);
                }
            }
            context.fcc = get_u64(&fp, layout.registers_size()) as usize;
            context.fcsr = get_u32(&fp, layout.registers_size() + 8) as usize;
            fcsr_address = Some(body + layout.registers_size() + 8);
        }

        let pending_signal = restore_fcsr_pending(mm, fcsr_address, &mut context)?;
        Ok(SignalFrameRestore {
            context,
            mask: get_u64(&bytes, MASK) & !UNBLOCKABLE,
            altstack,
            pending_signal,
        })
    }
}

/// 从栈顶先分配 END，再对齐具体 body 并分配记录头；精确检查实际完整帧的备用栈范围。
fn frame_address(
    sp: usize,
    args: &SignalFrameArgs,
    layout: &FpLayout,
) -> Result<(usize, usize, usize), VmError> {
    let stack = args.altstack;
    let nested = stack.on_stack(sp);
    let use_altstack = args.action.flags & SA_ONSTACK != 0
        && stack.flags & SS_DISABLE == 0
        && stack.size != 0
        && !nested;
    let top = if use_altstack {
        stack
            .base
            .checked_add(stack.size)
            .ok_or(VmError::InvalidRange)?
    } else {
        sp
    };
    let top_aligned = top & !(STACK_ALIGNMENT - 1);
    let end = top_aligned
        .checked_sub(SCTX_INFO_SIZE)
        .ok_or(VmError::InvalidRange)?;
    let record = if layout.lanes == 0 {
        end
    } else {
        let body = end
            .checked_sub(layout.body_size())
            .ok_or(VmError::InvalidRange)?
            & !(layout.alignment() - 1);
        body.checked_sub(SCTX_INFO_SIZE)
            .ok_or(VmError::InvalidRange)?
    };
    let address = record
        .checked_sub(EXTCONTEXT)
        .ok_or(VmError::InvalidRange)?;
    if USER_ADDRESS_END < top || ((nested || use_altstack) && address < stack.base) {
        return Err(VmError::InvalidRange);
    }
    Ok((address, top_aligned - address, end - record))
}

/// 标记实际保存的浮点扩展及故障方向，公共访问判别不当作硬件异常号。
fn context_flags(args: &SignalFrameArgs, used_fp: bool) -> u32 {
    let direction = match args.info.payload {
        SignalPayload::Fault { trap, .. } if trap == UserAccess::Write as usize => SC_ADDRERR_WR,
        SignalPayload::Fault { trap, .. }
            if trap == UserAccess::Read as usize || trap == UserAccess::Execute as usize =>
        {
            SC_ADDRERR_RD
        }
        _ => 0,
    };
    u32::from(used_fp) | direction
}

/// 完整解码备用栈原始候选；是否可替换登记由信号状态所有者按恢复后的 SP 判断。
fn decode_altstack(bytes: &[u8]) -> AltStack {
    AltStack {
        base: get_u64(bytes, STACK) as usize,
        size: get_u64(bytes, STACK_SIZE) as usize,
        flags: get_u32(bytes, STACK_FLAGS),
    }
}

/// 将帧中已启用的浮点异常 cause 清除并回写原槽，成功后请求内核生成的同步 SIGFPE。
fn restore_fcsr_pending(
    mm: &Mm,
    address: Option<usize>,
    context: &mut UserContext,
) -> Result<Option<SignalInfo>, VmError> {
    let fcsr = context.fcsr as u32;
    let enabled = (fcsr & reg::FCSR_ENABLE_MASK) << reg::FCSR_CAUSE_SHIFT;
    if fcsr & enabled == 0 {
        return Ok(None);
    }

    let cleared = fcsr & !enabled;
    let progress = mm.copyout(
        address.expect("pending FCSR requires an FP record"),
        &cleared.to_le_bytes(),
    );
    if let Some(error) = progress.error {
        return Err(error);
    }
    assert_eq!(
        progress.copied, 4,
        "successful signal FCSR copy is incomplete"
    );

    context.fcsr = cleared as usize;
    Ok(Some(SignalInfo::kernel(SIGFPE)))
}

/// 完整访问扩展记录尾部 padding，复用固定小缓冲且不为用户 size 分配内核存储。
fn copyin_padding(mm: &Mm, mut address: usize, end: usize) -> Result<(), VmError> {
    let mut bytes = [0u8; PADDING_COPY_CHUNK];
    while address < end {
        let count = (end - address).min(bytes.len());
        copyin_exact(mm, address, &mut bytes[..count])?;
        address += count;
    }
    Ok(())
}

/// 完整复制后才允许解码；跨页、缺页和 COW 的访问责任由正式 Mm 入口承担。
fn copyin_exact(mm: &Mm, address: usize, bytes: &mut [u8]) -> Result<(), VmError> {
    let progress = mm.copyin(bytes, address);
    if let Some(error) = progress.error {
        return Err(error);
    }
    assert_eq!(
        progress.copied,
        bytes.len(),
        "successful signal frame copy is incomplete"
    );
    Ok(())
}

/// 在初始化缓冲中写入小端机器字，不借用用户对象或结构体 padding。
fn put_u64(bytes: &mut [u8], offset: usize, value: u64) {
    bytes[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
}

/// 写入小端固定宽度标志或记录头字段。
fn put_u32(bytes: &mut [u8], offset: usize, value: u32) {
    bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

/// 仅从已完整复制且由调用者保证范围的字节缓冲解码机器字。
fn get_u64(bytes: &[u8], offset: usize) -> u64 {
    u64::from_le_bytes(
        bytes[offset..offset + 8]
            .try_into()
            .expect("signal frame field range"),
    )
}

/// 仅从已完整复制且由调用者保证范围的字节缓冲解码固定宽度字段。
fn get_u32(bytes: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes(
        bytes[offset..offset + 4]
            .try_into()
            .expect("signal frame field range"),
    )
}
