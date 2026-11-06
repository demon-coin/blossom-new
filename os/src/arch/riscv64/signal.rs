//! RISC-V 64 用户态信号栈帧（`rt_sigframe`）的编码与恢复模块。
//!
//! # 模块职责与架构定位
//! 本模块位于 `arch::riscv64` 架构层，向顶层信号子系统（`crate::task::signal`）提供与
//! Linux RISC-V 64 UABI 兼容的用户态信号栈帧构建（[`UserContext::setup_signal`]）与恢复
//! （[`UserContext::restore_signal`]）能力，下游依赖地址空间模块（`crate::mm::space::Mm`）
//! 完成用户态内存的安全拷贝（`copyout` / `copyin`）。
//!
//! # 核心设计与信号帧布局
//! 当内核向用户线程递送自定义处理函数的信号时，会在当前用户栈（或配置的备用信号栈 `sigaltstack`）
//! 上向下分配一个按 16 字节对齐的固定大小帧（`FRAME_SIZE = 1104` 字节），并将中断时刻的用户寄存器、
//! 信号掩码、备用栈状态及 `siginfo_t` 写入其中。信号处理函数执行完毕返回至信号跳板（Trampoline）后，
//! 触发 `rt_sigreturn` 系统调用，由 [`UserContext::restore_signal`] 从当前用户 `sp` 读回并还原现场。
//!
//! 信号栈帧（`rt_sigframe`，总计 `FRAME_SIZE = 1104` 字节）的小端字节布局如下：
//!
//! ```text
//! 高地址 (原 sp 或 altstack 栈顶)
//! +-----------------------------------------------------------------------+ 1104 (FRAME_SIZE)
//! | __riscv_mc_fp_state 尾部 16 字节对齐填充 (4B)                         | 1100
//! | __riscv_ctx_hdr 终止头: magic=0 (4B) + size=0 (4B)                    | 1092 (EXT_HEADER)
//! | 扩展上下文前保留字 (ext_reserved = 0, 4B)                             | 1088 (EXT_RESERVED)
//! | __riscv_mc_q_ext_state 联合体剩余空间 (268B)                          | 820
//! | __riscv_mc_d_ext_state.fcsr: 浮点控制状态寄存器 (4B)                  | 816  (FCSR)
//! | __riscv_mc_d_ext_state.f[0..32]: 32 个 64 位浮点寄存器 (256B)         | 560  (FP)
//! | uc_mcontext.sc_regs: [0] 存 pc，[1..31] 存 x1..x31 (256B)             | 304  (MCONTEXT)
//! | 16 字节对齐填充 (8B)                                                  | 296
//! | uc_sigmask: 64 位信号掩码 (8B) + 扩展掩码保留区 (120B)                | 168  (MASK)
//! | uc_stack (stack_t): ss_sp (8B) + ss_flags (4B+4B pad) + ss_size (8B)  | 144  (ALTSTACK)
//! | ucontext_t 头: uc_flags (8B, 0) + uc_link (8B, 0)                     | 128  (UCONTEXT)
//! | siginfo_t: 信号元数据与附加负载 (128B)                                | 0    (INFO_SIZE)
//! +-----------------------------------------------------------------------+ 低地址 (新 sp = a1)
//! ```
//!
//! # 锁获取顺序与并发契约
//! - `setup_signal` 与 `restore_signal` 均在普通可睡眠任务上下文中调用。
//! - 调用期间**不得持有任何自旋锁**，因为底层的 `mm.copyout` / `mm.copyin` 在遇到未映射页或
//!   写时复制（COW）页时会获取地址空间锁并可能触发缺页睡眠。
//! - 两个函数均遵循“先在内核栈组装/校验完整缓冲，成功后再一次性提交到 `UserContext`”的原子化更新契约：
//!   一旦用户内存读写或格式校验失败，当前线程的 `UserContext` 保持原样不变。

use crate::arch::{riscv64::user::UserContext, STACK_ALIGNMENT};
use crate::mm::space::{Mm, VmError};
use crate::task::signal::{SignalFrameArgs, SignalFrameRestore, SA_ONSTACK};

/// 启用备用信号栈（`sigaltstack`）所需的最小栈容量（字节）：除容纳固定信号帧外，还需为信号处理函数预留栈空间。
pub(crate) const SIGNAL_MIN_STACK: usize = 2048;

// Linux RV64 UABI 信号帧各部分的字节偏移与尺寸常量：stack_t 为 24 字节，sigset 预留 128 字节，mcontext 按 16 字节对齐。
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
// 浮点上下文 union 按最大 Q 扩展尺寸定义（64 个 64 位字 + fcsr + 3 个 32 位保留字），整体占 544 字节。
const FP_UNION_SIZE: usize = 64 * ABI_WORD_BYTES + 4 * core::mem::size_of::<u32>();
const FCSR: usize = FP + 32 * ABI_WORD_BYTES;
const EXT_RESERVED: usize = FP + 129 * core::mem::size_of::<u32>();
const EXT_HEADER: usize = EXT_RESERVED + core::mem::size_of::<u32>();
const FRAME_SIZE: usize = FP + FP_UNION_SIZE;

impl UserContext {
    /// 在当前线程的用户栈（或备用信号栈）上构建并写入信号帧，随后将用户执行流重定向至信号处理函数。
    ///
    /// # 调用上下文
    /// 在普通可睡眠任务返回用户态前的信号递送阶段调用。底层 `mm.copyout` 可能会触发缺页处理或写时复制（COW），
    /// 调用时不得持有自旋锁。只有在整帧数据成功写入用户栈后，才会修改 `self` 中的 `pc`、`sp`、`ra` 与参数寄存器。
    pub(crate) fn setup_signal(&mut self, mm: &Mm, args: &SignalFrameArgs) -> Result<(), VmError> {
        // Step 1: 计算信号栈帧在用户空间的目标起始地址（处理 SA_ONSTACK 备用栈切换与 16 字节对齐）。
        let address = frame_address(self.gpr[2], args)?;
        let mut frame = [0u8; FRAME_SIZE];

        // Step 2: 在内核栈临时缓冲区中编码 siginfo、备用栈状态、信号掩码以及被中断的整数与浮点寄存器。
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

        // Step 3: 将组装好的信号帧一次性拷贝至用户栈；未显式赋值的字节保持为 0，
        // 自然充当 uc_flags、uc_link、对齐填充以及表示无向量扩展的终止头（END_MAGIC = 0, END_HDR_SIZE = 0）。
        let progress = mm.copyout(address, &frame);
        if let Some(error) = progress.error {
            return Err(error);
        }
        debug_assert_eq!(progress.copied, FRAME_SIZE);

        // Step 4: 写入用户栈成功后，更新用户态寄存器使返回用户态时直接进入信号处理函数。
        self.pc = args.action.handler;
        self.gpr[1] = args.trampoline;
        self.gpr[2] = address;
        self.gpr[10] = args.info.signum as usize;
        // 始终同时传递 a1 (siginfo_t*) 与 a2 (ucontext_t*)，未开启 SA_SIGINFO 的单参数处理函数会直接忽略 a1/a2。
        self.gpr[11] = address;
        self.gpr[12] = address + UCONTEXT;
        Ok(())
    }

    /// 从当前用户栈指针（`sp`）处读取并校验信号帧，解码出信号处理前保存的用户上下文、掩码与备用栈配置。
    ///
    /// # 调用上下文
    /// 在 `rt_sigreturn` 系统调用路径（普通可睡眠任务上下文）中调用。
    /// 允许用户态在信号处理期间主动修改栈帧中的通用/浮点寄存器与 `pc`；若地址未对齐、用户内存不可读或
    /// 扩展头字段非法，则直接返回 `Err` 且不改动当前 `UserContext`。
    pub(crate) fn restore_signal(&self, mm: &Mm) -> Result<SignalFrameRestore, VmError> {
        // Step 1: 校验当前用户栈指针的 16 字节对齐，并从用户空间完整读入信号帧字节。
        if self.gpr[2] & (STACK_ALIGNMENT - 1) != 0 {
            return Err(VmError::InvalidRange);
        }
        let mut frame = [0u8; FRAME_SIZE];
        let progress = mm.copyin(&mut frame, self.gpr[2]);
        if let Some(error) = progress.error {
            return Err(error);
        }
        debug_assert_eq!(progress.copied, FRAME_SIZE);

        // Step 2: 校验浮点状态尾部的扩展上下文描述头；当前内核不支持向量（V）等扩展现场，
        // 若保留字或扩展头（magic / size）非零，说明栈帧被破坏或包含无法识别的扩展数据，拒绝恢复。
        if get_u32(&frame, EXT_RESERVED) != 0
            || get_u32(&frame, EXT_HEADER) != 0
            || get_u32(&frame, EXT_HEADER + 4) != 0
        {
            return Err(VmError::InvalidRange);
        }

        // Step 3: 构建新的 UserContext 并填入从用户信号帧解码出的 pc、通用寄存器与浮点寄存器。
        // 内核续接栈指针（kernel_sp）、异常 CSR（scause/stval）以及内核线程指针（CpuLocal）属于内核受保护状态，
        // 由 UserContext::new 初始化为 0，绝不从用户态字节中读取。
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
            // Step 4: 提取帧中记录的备用栈描述符，交由上层信号子系统根据恢复后的 sp 更新线程备用栈状态。
            altstack: crate::task::signal::AltStack {
                base: get_u64(&frame, ALTSTACK) as usize,
                size: get_u64(&frame, ALTSTACK_BYTES) as usize,
                flags: get_u32(&frame, ALTSTACK_FLAGS),
            },
            pending_signal: None,
        })
    }
}

/// 根据当前用户栈指针 `sp` 与信号动作配置计算信号帧的压栈地址，处理备用信号栈（`SA_ONSTACK`）切换并向下按 16 字节对齐。
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

/// 向信号帧缓冲区的指定字节偏移 `offset` 处以小端序写入一个 64 位整数（避免依赖 Rust 结构体的隐式对齐填充）。
fn put_u64(frame: &mut [u8; FRAME_SIZE], offset: usize, value: u64) {
    frame[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
}

/// 向信号帧缓冲区的指定字节偏移 `offset` 处以小端序写入一个 32 位整数。
fn put_u32(frame: &mut [u8; FRAME_SIZE], offset: usize, value: u32) {
    frame[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

/// 从已拷贝到内核的信号帧缓冲区指定偏移 `offset` 处按小端序解码一个 64 位整数。
fn get_u64(frame: &[u8; FRAME_SIZE], offset: usize) -> u64 {
    let mut bytes = [0; 8];
    bytes.copy_from_slice(&frame[offset..offset + 8]);
    u64::from_le_bytes(bytes)
}

/// 从已拷贝到内核的信号帧缓冲区指定偏移 `offset` 处按小端序解码一个 32 位整数。
fn get_u32(frame: &[u8; FRAME_SIZE], offset: usize) -> u32 {
    let mut bytes = [0; 4];
    bytes.copy_from_slice(&frame[offset..offset + 4]);
    u32::from_le_bytes(bytes)
}
