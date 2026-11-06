//! LoongArch64 用户态信号栈帧（`rt_sigframe`）构建与恢复模块。
//!
//! # 模块职责与架构定位
//! 本模块位于 `arch::loongarch64` 用户态支撑层，向上为 `task::signal` 提供架构相关的信号帧压栈
//! （[`UserContext::setup_signal`]）与 `rt_sigreturn` 现场恢复（[`UserContext::restore_signal`]），
//! 向下依赖虚拟内存管理（[`Mm::copyin`] / [`Mm::copyout`]）完成跨页、缺页与写时复制（COW）安全的用户栈访问。
//!
//! # 核心设计与用户栈信号帧布局
//! LoongArch64 Linux ABI 的 `rt_sigframe` 采用固定头部加可变长扩展上下文（`sctx_info` 链表）的布局。
//! 内核根据当前 CPU 实际支持的最高浮点/向量扩展（标量 FPU / 128 位 LSX / 256 位 LASX）保存互斥的扩展记录；
//! 内核态续接栈指针与特权 CSR 绝不写入用户栈，防止用户态篡改特权状态。
//!
//! ```text
//! 高地址（原用户 SP 或 sigaltstack 栈顶，向下 16 字节对齐）
//! +---------------------------------------------------------------+
//! | 终止 sctx_info (16 B): magic = 0, size = 0, padding = 0       | <- end
//! +---------------------------------------------------------------+
//! | 可选对齐填充（当 LASX 要求 32 字节对齐 body 时可能存在）      |
//! +---------------------------------------------------------------+
//! | 浮点/向量扩展 Body（当 CPU 支持 FP/LSX/LASX 时存在）:         | <- body (16B 或 32B 对齐)
//! |   - 32 个浮点/向量寄存器 (32 * lanes * 8 B)                   |
//! |   - fcc  (8 B, 8 个条件码标志位打包)                          |
//! |   - fcsr (4 B, 浮点控制与状态寄存器) + 尾部对齐填充 (4 B)     |
//! +---------------------------------------------------------------+
//! | 扩展记录头 sctx_info (16 B):                                  | <- sp + EXTCONTEXT (record)
//! |   - magic (4 B): FPU_MAGIC / LSX_MAGIC / LASX_MAGIC           |
//! |   - size  (4 B): 从本记录头到终止记录头之间的总字节数         |
//! |   - padding (8 B)                                             |
//! +---------------------------------------------------------------+
//! | mcontext_t 固定区 (偏移 MCONTEXT = 304):                      |
//! |   - sc_pc    (8 B, 偏移 304): 信号触发时的用户态 PC           |
//! |   - sc_regs  (32 * 8 B, 偏移 312): 通用寄存器 r0..r31         |
//! |   - sc_flags (4 B, 偏移 568) + 12 B 对齐填充                  |
//! +---------------------------------------------------------------+
//! | ucontext_t 头部 (偏移 UCONTEXT = 128):                        |
//! |   - uc_flags (8 B, 0) / uc_link (8 B, 0)                      |
//! |   - uc_stack (24 B, 偏移 STACK = 144): ss_sp, ss_flags, size  |
//! |   - uc_sigmask (128 B, 偏移 MASK = 168): 信号掩码及保留空间   |
//! +---------------------------------------------------------------+
//! | siginfo_t (128 B, 偏移 0)                                     | <- 新用户 SP (16 字节对齐)
//! +---------------------------------------------------------------+
//! 低地址
//! ```
//!
//! # 锁获取顺序与并发契约
//! - `setup_signal` 与 `restore_signal` 均在普通任务上下文中执行，内部通过 [`Mm::copyin`] / [`Mm::copyout`]
//!   访问用户虚拟内存，期间可能触发缺页分配或写时复制并获取 `Mm` 内部锁。
//! - 调用方在进入本模块前**不得持有**任务信号自旋锁（`siglock`）、关中断守卫（IRQ guard）或其他自旋锁，
//!   以免在页表操作或物理页分配中违反锁层级。

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

/// 用户态备用信号栈（`sigaltstack`）的最小字节容量；最宽的 LASX 信号帧加上对齐开销仍小于此值。
pub(crate) const SIGNAL_MIN_STACK: usize = 4096;

/// 64 位机器字字节数（8 字节）。
const WORD_BYTES: usize = core::mem::size_of::<u64>();
/// `mcontext_t` 与 `sctx_info` 扩展记录头的 ABI 对齐字节数（16 字节）。
const CONTEXT_INFO_ALIGNMENT: usize = 16;
/// `ucontext_t::uc_sigmask` 在 ABI 中预留的总字节数（128 字节）。
const SIGMASK_RESERVED_BYTES: usize = 128;
/// `uc_stack.ss_flags` 字段在信号帧中的字节偏移。
const STACK_FLAGS: usize = STACK + WORD_BYTES;
/// `uc_stack.ss_size` 字段在信号帧中的字节偏移。
const STACK_SIZE: usize = STACK + 2 * WORD_BYTES;
/// 浮点扩展记录尾部 `fcc`（8 字节）与 `fcsr` 及其填充（8 字节）共占用的字节数。
const FP_TRAILER_BYTES: usize = 2 * WORD_BYTES;
/// 校验用户自定义扩展记录尾部填充区时，单次栈上缓冲分块读取的字节数。
const PADDING_COPY_CHUNK: usize = 128;

/// `siginfo_t` 结构体在信号帧起始处的固定字节大小。
const INFO_SIZE: usize = 128;
/// `ucontext_t` 结构体在信号帧中的起始字节偏移。
const UCONTEXT: usize = INFO_SIZE;
/// `ucontext_t::uc_stack`（备用栈信息）在信号帧中的字节偏移。
const STACK: usize = UCONTEXT + 2 * WORD_BYTES;
/// `ucontext_t::uc_sigmask`（信号屏蔽字）在信号帧中的字节偏移。
const MASK: usize = STACK + 3 * WORD_BYTES;
/// `ucontext_t::uc_mcontext`（机器上下文）经 16 字节对齐后的起始字节偏移。
const MCONTEXT: usize =
    (MASK + SIGMASK_RESERVED_BYTES + CONTEXT_INFO_ALIGNMENT - 1) & !(CONTEXT_INFO_ALIGNMENT - 1);
/// `mcontext_t::sc_regs`（32 个通用寄存器数组）在信号帧中的字节偏移。
const REGISTERS: usize = MCONTEXT + WORD_BYTES;
/// `mcontext_t::sc_flags`（上下文标志字）在信号帧中的字节偏移。
const FLAGS: usize = REGISTERS + 32 * WORD_BYTES;
/// `mcontext_t::sc_extcontext`（首个 `sctx_info` 扩展记录头）经 16 字节对齐后的起始字节偏移。
const EXTCONTEXT: usize = (FLAGS + core::mem::size_of::<u32>() + CONTEXT_INFO_ALIGNMENT - 1)
    & !(CONTEXT_INFO_ALIGNMENT - 1);
/// 单个 `sctx_info` 扩展记录头（含 `magic`、`size` 及对齐填充）的字节大小（16 字节）。
const SCTX_INFO_SIZE: usize = CONTEXT_INFO_ALIGNMENT;
/// 最宽浮点/向量扩展（LASX 32 个 256 位寄存器 + 尾部 `fcc`/`fcsr`）的最大 Body 字节大小。
const MAX_BODY_SIZE: usize =
    core::mem::size_of::<[crate::arch::loongarch64::user::FpRegister; 32]>() + FP_TRAILER_BYTES;
/// 内核生成的完整信号帧在最坏对齐情况下的最大字节大小。
const MAX_FRAME_SIZE: usize =
    EXTCONTEXT + SCTX_INFO_SIZE + MAX_BODY_SIZE + CONTEXT_INFO_ALIGNMENT + SCTX_INFO_SIZE;
/// 编译期断言：最大信号帧尺寸必须严格小于最小备用信号栈容量 `SIGNAL_MIN_STACK`。
const _: () = assert!(MAX_FRAME_SIZE < SIGNAL_MIN_STACK);

/// LoongArch64 基础 64 位标量浮点扩展上下文（`fpu_context`）的 ABI 魔数。
const FPU_MAGIC: u32 = 0x4650_5501;
/// LoongArch64 128 位向量扩展上下文（`lsx_context`）的 ABI 魔数。
const LSX_MAGIC: u32 = 0x5358_0001;
/// LoongArch64 256 位高级向量扩展上下文（`lasx_context`）的 ABI 魔数。
const LASX_MAGIC: u32 = 0x4153_5801;
/// `mcontext_t::sc_flags` 标志位：指示当前信号帧保存了浮点/向量扩展上下文。
const SC_USED_FP: u32 = 1;
/// `mcontext_t::sc_flags` 标志位：指示触发信号的内存异常为读或取指访问。
const SC_ADDRERR_RD: u32 = 1 << 30;
/// `mcontext_t::sc_flags` 标志位：指示触发信号的内存异常为写访问。
const SC_ADDRERR_WR: u32 = 1 << 31;

/// 浮点/向量扩展上下文（`fpu_context` / `lsx_context` / `lasx_context`）的数据区布局描述。
///
/// LoongArch64 的标量浮点（FP）、128 位向量（LSX）与 256 位向量（LASX）复用同一组物理寄存器，
/// 因此信号帧中只保存与当前 CPU 能力匹配的一种互斥扩展记录：32 个寄存器按有效 64 位通道（lane）
/// 连续排列，`fcc` 与 `fcsr` 紧随寄存器数组末尾。
struct FpLayout {
    /// 单个浮点/向量寄存器包含的 64 位通道（lane）数：无浮点为 0，FP 为 1，LSX 为 2，LASX 为 4。
    lanes: usize,
}

impl FpLayout {
    /// 根据当前 CPU 硬件支持的最高浮点/向量扩展能力创建对应的帧布局（不支持浮点时 `lanes == 0`）。
    fn current() -> Self {
        Self {
            lanes: user_fp_lanes(),
        }
    }

    /// 根据用户态信号帧中的 `sctx_info::magic` 解码扩展布局，拒绝未知魔数或超出当前 CPU 硬件能力的扩展。
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

    /// 返回当前浮点/向量扩展对应的 Linux ABI `sctx_info::magic` 魔数（无浮点时返回 0）。
    fn magic(&self) -> u32 {
        match self.lanes {
            1 => FPU_MAGIC,
            2 => LSX_MAGIC,
            4 => LASX_MAGIC,
            _ => 0,
        }
    }

    /// 返回 32 个浮点/向量寄存器占用的总字节数（即 `fcc` 字段相对于扩展 Body 起点的字节偏移）。
    fn registers_size(&self) -> usize {
        32 * self.lanes * WORD_BYTES
    }

    /// 返回包含 32 个寄存器及尾部 `fcc`/`fcsr`（含 ABI 填充）在内的扩展 Body 总字节数。
    fn body_size(&self) -> usize {
        self.registers_size() + FP_TRAILER_BYTES
    }

    /// 返回扩展 Body 所需的字节对齐要求：LASX 要求 32 字节对齐，标量 FP 与 LSX 要求 16 字节对齐。
    fn alignment(&self) -> usize {
        if self.lanes == 4 {
            core::mem::align_of::<crate::arch::loongarch64::user::FpRegister>()
        } else {
            CONTEXT_INFO_ALIGNMENT
        }
    }
}

impl UserContext {
    /// 在普通任务上下文中构建信号栈帧并写入用户栈，写入成功后将 `self` 重定向到信号处理函数入口。
    ///
    /// # 调用上下文与锁契约
    /// - 必须在普通可睡眠任务上下文中调用；内部调用 [`Mm::copyout`] 时可能触发缺页异常或写时复制（COW）。
    /// - 调用方不得持有任务的信号锁（`siglock`）、关中断守卫或任何自旋锁。
    /// - 若栈地址越界、对齐非法或写入用户内存失败，直接返回 `Err(VmError)` 且不修改当前 `UserContext`。
    pub(crate) fn setup_signal(&mut self, mm: &Mm, args: &SignalFrameArgs) -> Result<(), VmError> {
        // Step 1: 计算信号帧在用户栈（或备用信号栈）上的起始地址、总长度与扩展记录跨度。
        let layout = FpLayout::current();
        let (sp, frame_size, record_size) = frame_address(self.gpr[3], args, &layout)?;
        let mut bytes = [0u8; MAX_FRAME_SIZE];

        // Step 2: 在内核栈临时缓冲区中编码 siginfo_t、ucontext_t 头部、被中断的 PC 与通用寄存器。
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
        // Step 3: 若 CPU 支持浮点/向量扩展，编码 sctx_info 记录头以及全部浮点寄存器、FCC 与 FCSR。
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

        // Step 4: 将组装好的信号帧一次性复制到用户栈；栈上缓冲区预先清零，自动满足所有对齐填充与末尾 magic=0 终止记录。
        let progress = mm.copyout(sp, &bytes[..frame_size]);
        if let Some(error) = progress.error {
            return Err(error);
        }
        assert_eq!(
            progress.copied, frame_size,
            "successful signal frame copy is incomplete"
        );

        // Step 5: 信号帧写入成功后，设置返回跳板（r1/ra）、新栈顶（r3/sp）、参数寄存器（a0..a2）及信号处理函数入口 PC。
        self.gpr[1] = args.trampoline;
        self.gpr[3] = sp;
        self.gpr[4] = args.info.signum as usize;
        self.gpr[5] = sp;
        self.gpr[6] = sp + UCONTEXT;
        self.pc = args.action.handler;
        Ok(())
    }

    /// 从当前用户栈顶（`r3/sp`）读取并校验完整的信号栈帧，生成供 `rt_sigreturn` 恢复的候选上下文。
    ///
    /// # 原子性与 FCSR 异常处理
    /// - 在完成全部内存读取、魔数与边界校验前，绝不修改当前任务的执行现场或信号状态。
    /// - 若信号帧中的 `fcsr` 含有已使能的浮点异常原因位（cause），内核会先清除这些原因位并写回用户栈原槽位，
    ///   再随返回结果附带一个同步 `SIGFPE` 信号；若写回用户栈失败，整个恢复操作中止且不提交任何变更。
    pub(crate) fn restore_signal(&self, mm: &Mm) -> Result<SignalFrameRestore, VmError> {
        // Step 1: 校验用户栈指针 16 字节对齐，并读取固定大小的信号帧头部及首个扩展记录头。
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

        // Step 2: 解码备用信号栈候选与通用寄存器现场；UserContext::new 会将异常 CSR 与 kernel_sp 清零，只从用户栈恢复通用寄存器与 PC。
        let altstack = decode_altstack(&bytes);
        let mut context = Self::new(get_u64(&bytes, MCONTEXT) as usize, 0);
        for (index, register) in context.gpr.iter_mut().enumerate().skip(1) {
            *register = get_u64(&bytes, REGISTERS + index * 8) as usize;
        }

        // Step 3: 解析并校验 sctx_info 扩展上下文记录。
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
            // Step 4: 读取浮点/向量扩展 Body，逐块触碰用户声明的尾部填充区以验证映射有效性，并确认紧随其后的是 magic=0 终止记录。
            let mut fp = [0u8; MAX_BODY_SIZE];
            copyin_exact(mm, body, &mut fp[..layout.body_size()])?;
            copyin_padding(mm, body + layout.body_size(), end)?;
            let mut terminator = [0u8; SCTX_INFO_SIZE];
            copyin_exact(mm, end, &mut terminator)?;
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

        // Step 5: 检查并处理 FCSR 中待触发的浮点异常，组装最终恢复结果（强制剔除不可屏蔽信号）。
        let pending_signal = restore_fcsr_pending(mm, fcsr_address, &mut context)?;
        Ok(SignalFrameRestore {
            context,
            mask: get_u64(&bytes, MASK) & !UNBLOCKABLE,
            altstack,
            pending_signal,
        })
    }
}

/// 从目标栈顶向下依次分配终止记录头、对齐后的浮点扩展 Body、扩展记录头及固定帧头，并校验地址空间与备用信号栈边界。
///
/// 返回 `(frame_sp, frame_size, ext_record_size)`。
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

/// 根据当前是否保存浮点扩展以及触发信号的内存故障访问方向，计算写入 `mcontext_t::sc_flags` 的标志位。
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

/// 从信号帧字节流中解码 `uc_stack` 备用信号栈配置（是否允许应用该配置由上层信号模块结合恢复后的 SP 裁决）。
fn decode_altstack(bytes: &[u8]) -> AltStack {
    AltStack {
        base: get_u64(bytes, STACK) as usize,
        size: get_u64(bytes, STACK_SIZE) as usize,
        flags: get_u32(bytes, STACK_FLAGS),
    }
}

/// 检查恢复后的 `fcsr` 是否包含已使能的浮点异常原因位：若存在，则清除原因位并写回用户栈原槽位，同时生成同步 `SIGFPE` 信号。
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

/// 分块读取扩展上下文记录尾部的对齐填充区，使用固定大小的栈缓冲区验证用户声明的整个记录范围均已合法映射。
fn copyin_padding(mm: &Mm, mut address: usize, end: usize) -> Result<(), VmError> {
    let mut bytes = [0u8; PADDING_COPY_CHUNK];
    while address < end {
        let count = (end - address).min(bytes.len());
        copyin_exact(mm, address, &mut bytes[..count])?;
        address += count;
    }
    Ok(())
}

/// 从用户虚拟地址 `address` 完整读取 `bytes.len()` 字节到内核缓冲区，跨页、缺页与写时复制由 [`Mm::copyin`] 处理。
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

/// 将 64 位无符号整数按小端序写入信号帧缓冲区的指定字节偏移处。
fn put_u64(bytes: &mut [u8], offset: usize, value: u64) {
    bytes[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
}

/// 将 32 位无符号整数按小端序写入信号帧缓冲区的指定字节偏移处。
fn put_u32(bytes: &mut [u8], offset: usize, value: u32) {
    bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

/// 从信号帧缓冲区的指定字节偏移处按小端序读取 64 位无符号整数（调用方保证偏移与长度在切片范围内）。
fn get_u64(bytes: &[u8], offset: usize) -> u64 {
    u64::from_le_bytes(
        bytes[offset..offset + 8]
            .try_into()
            .expect("signal frame field range"),
    )
}

/// 从信号帧缓冲区的指定字节偏移处按小端序读取 32 位无符号整数（调用方保证偏移与长度在切片范围内）。
fn get_u32(bytes: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes(
        bytes[offset..offset + 4]
            .try_into()
            .expect("signal frame field range"),
    )
}
