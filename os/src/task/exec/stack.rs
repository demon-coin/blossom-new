//! `execve` 初始用户栈布局计算与 System V ABI 数据填充模块。
//!
//! ## 模块职责与架构定位
//! 本模块是 `execve` 装载器的子模块（`crate::task::exec::stack`），负责在新建的私有地址空间 [`Mm`] 中规划初始用户栈范围、检查与 ELF 装载段的冲突、建立栈 VMA 映射，并按 System V ABI 规范向用户栈写入参数字符串、环境变量、随机数种子及辅助向量（Auxiliary Vector, `auxv`）。
//!
//! ## 核心机制：System V ABI 初始用户栈内存布局
//!
//! 初始用户栈从 [`arch::USER_STACK_TOP`] 向低地址生长，分为高地址的**字符串与随机数字节区**（由 `strings` 游标从高向低填充）和低地址的**机器字指针与辅助向量表区**（由 `pointers` 游标从 `sp` 向高地址顺序填充）：
//!
//! ```text
//!   高地址  arch::USER_STACK_TOP
//!           +----------------------------------------------------+
//!           | filename ("/bin/sh\0")                             | ◄── layout.execfn (AT_EXECFN 指向此处)
//!           +----------------------------------------------------+
//!           | argv[0] 字符串 ... argv[argc-1] 字符串 (均含 '\0')   |
//!           +----------------------------------------------------+
//!           | envp[0] 字符串 ... envp[envc-1] 字符串 (均含 '\0')   |
//!           +----------------------------------------------------+
//!           | 16 字节伪随机数 (AT_RANDOM_BYTES)                    | ◄── layout.random (AT_RANDOM 指向此处)
//!           +----------------------------------------------------+
//!           | 栈指针对齐填充 (0..15 字节，对齐到 STACK_ALIGNMENT)  |
//!           +----------------------------------------------------+
//!           | auxv[16]: (AT_NULL, 0)                             | ▲
//!           | auxv[0..15]: AT_PHDR / AT_ENTRY / AT_RANDOM ...    | │ pointers 游标
//!           +----------------------------------------------------+ │ 从 layout.sp
//!           | NULL (0usize, envp 结尾标记)                        | │ 向高地址推进
//!           | envp[0..envc] 指针数组 (指向高地址各 envp 字符串)    | │
//!           +----------------------------------------------------+ │
//!           | NULL (0usize, argv 结尾标记)                        | │
//!           | argv[0..argc] 指针数组 (指向高地址各 argv 字符串)    | │
//!           +----------------------------------------------------+ │
//!           | argc (命令行参数个数，usize)                         | ◄── layout.sp (初始用户栈指针 sp，16 字节对齐)
//!           +----------------------------------------------------+
//!           | 初始预留栈增长空间 (headroom，最多 128 KiB)          |
//!   低地址  +----------------------------------------------------+ ◄── layout.bottom (栈 VMA 下界，页对齐)
//! ```
//!
//! ## 并发与安全契约
//! - **双游标无额外堆分配**：[`Layout::calculate`] 预先精确计算出字符串区与指针区的边界地址，[`make_stack`] 在栈页映射完成后同时推进 `strings`（向下）和 `pointers`（向上）两个游标，无需在内核堆中额外分配临时数组保存用户态字符串地址。
//! - **离线拷贝与完整校验**：所有栈数据均通过 [`copyout_exact`] 写入尚未发布的候选 `Mm`，不构造指向用户空间的 Rust 引用；任何越界或缺页失败均直接中止装载并自动回收候选 `Mm`。

use core::mem::size_of;

use crate::arch;
use crate::mm::space::{Mm, VmError};
use crate::mm::user_table::PagePermissions;
use crate::mm::PAGE_SIZE;
use crate::random;
use crate::task::time::USER_HZ;

use super::elf::Elf;
use super::{Arguments, ExecError, RWX};

/// 除参数与环境变量占用的栈页外，额外预映射的初始启动栈余量上限（128 KiB）。
const STARTUP_HEADROOM: usize = 128 * 1024;
/// 写入初始用户栈的辅助向量（`auxv`）键值对总数（含末尾 `AT_NULL`）。
const AUXV_PAIRS: usize = 17;
/// `AT_RANDOM` 指向的栈上随机字节长度（16 字节）。
const AT_RANDOM_BYTES: usize = 16;

/// `AT_NULL`：辅助向量表结束标记。
const AT_NULL: usize = 0;
/// `AT_PHDR`：主程序头表在用户空间的虚拟地址。
const AT_PHDR: usize = 3;
/// `AT_PHENT`：单个程序头表项的字节大小。
const AT_PHENT: usize = 4;
/// `AT_PHNUM`：程序头表项的数量。
const AT_PHNUM: usize = 5;
/// `AT_PAGESZ`：系统硬件页大小（字节）。
const AT_PAGESZ: usize = 6;
/// `AT_BASE`：动态链接器（解释器）装载基地址。
const AT_BASE: usize = 7;
/// `AT_FLAGS`：可执行文件标志位（固定为 `0`）。
const AT_FLAGS: usize = 8;
/// `AT_ENTRY`：主程序入口虚拟地址（`e_entry` + `bias`）。
const AT_ENTRY: usize = 9;
/// `AT_UID`：进程的真实用户 ID。
const AT_UID: usize = 11;
/// `AT_EUID`：进程的有效用户 ID。
const AT_EUID: usize = 12;
/// `AT_GID`：进程的真实组 ID。
const AT_GID: usize = 13;
/// `AT_EGID`：进程的有效组 ID。
const AT_EGID: usize = 14;
/// `AT_HWCAP`：CPU 硬件指令集扩展能力掩码。
const AT_HWCAP: usize = 16;
/// `AT_CLKTCK`：`times()` 系统调用的每秒时钟滴答数（`USER_HZ`）。
const AT_CLKTCK: usize = 17;
/// `AT_SECURE`：安全执行模式标志（非 setuid 启动时为 `0`）。
const AT_SECURE: usize = 23;
/// `AT_RANDOM`：指向栈上 16 字节随机数的虚拟地址。
const AT_RANDOM: usize = 25;
/// `AT_EXECFN`：指向栈上原始可执行文件路径字符串的虚拟地址。
const AT_EXECFN: usize = 31;

/// 预先计算出的初始用户栈关键虚拟地址边界。
struct Layout {
    /// 初始栈 VMA 映射的页对齐下界虚拟地址。
    bottom: usize,
    /// 满足平台 ABI 对齐要求的初始用户栈指针（`sp`）虚拟地址。
    sp: usize,
    /// `AT_EXECFN` 可执行文件路径字符串在栈上的起始虚拟地址。
    execfn: usize,
    /// `AT_RANDOM` 16 字节随机数在栈上的起始虚拟地址。
    random: usize,
}

impl Layout {
    /// 根据参数快照计算初始栈的各区域起始地址与 VMA 映射下界。
    ///
    /// 即使当前任务的 `stack_limit` 软限制较小，也会保证完整容纳所有已通过预算检查的参数页；
    /// 若软限制尚有富余，则向下最多扩展 [`STARTUP_HEADROOM`]（128 KiB）作为初始启动栈空间。
    fn calculate(args: &Arguments) -> Result<Self, ExecError> {
        let execfn = arch::USER_STACK_TOP
            .checked_sub(args.filename.len() + 1)
            .ok_or(ExecError::ArgumentsTooBig)?;
        let random = arch::USER_STACK_TOP
            .checked_sub(args.string_bytes())
            .and_then(|address| address.checked_sub(AT_RANDOM_BYTES))
            .ok_or(ExecError::ArgumentsTooBig)?;
        let words = args
            .argv
            .len()
            .checked_add(args.envp.len())
            .and_then(|count| count.checked_add(3 + 2 * AUXV_PAIRS))
            .and_then(|count| count.checked_mul(size_of::<usize>()))
            .ok_or(ExecError::ArgumentsTooBig)?;
        let sp = random
            .checked_sub(words)
            .ok_or(ExecError::ArgumentsTooBig)?
            & !(arch::STACK_ALIGNMENT - 1);

        let actual_bottom = sp & !(PAGE_SIZE - 1);
        let actual = arch::USER_STACK_TOP - actual_bottom;
        let headroom = args
            .stack_limit
            .saturating_sub(actual as u64)
            .min(STARTUP_HEADROOM as u64) as usize
            & !(PAGE_SIZE - 1);
        let bottom = actual_bottom
            .checked_sub(headroom)
            .ok_or(ExecError::Vm(VmError::InvalidRange))?;

        Ok(Self {
            bottom,
            sp,
            execfn,
            random,
        })
    }
}

/// 在候选地址空间 `mm` 中建立初始用户栈映射，并写入 `argc`、`argv`、`envp` 与 `auxv` 辅助向量。
///
/// 成功时返回对齐后的初始用户栈顶指针 `sp`。
#[expect(
    clippy::too_many_arguments,
    reason = "离线栈构造分别消费主映像、解释器、参数、凭据与权限，保持 auxv 和映射输入的原 ABI 关系"
)]
pub(super) fn make_stack(
    mm: &Mm,
    elf: &Elf,
    interpreter: Option<&Elf>,
    interpreter_base: usize,
    args: &Arguments,
    uid: u32,
    gid: u32,
    permissions: PagePermissions,
) -> Result<usize, ExecError> {
    // Step 1: 计算栈布局，检查是否与主程序或动态链接器的 PT_LOAD 段重叠，并在 mm 中建立栈 VMA。
    let layout = Layout::calculate(args)?;
    if elf.overlaps(layout.bottom, arch::USER_STACK_TOP)
        || interpreter.is_some_and(|image| image.overlaps(layout.bottom, arch::USER_STACK_TOP))
    {
        return Err(ExecError::Vm(VmError::Occupied));
    }

    mm.map_stack(
        layout.bottom,
        arch::USER_STACK_TOP - layout.bottom,
        permissions,
        RWX,
    )?;

    // Step 2: 同时推进高地址字符串游标 strings 与低地址指针游标 pointers，依次写入 filename、argc、argv[] 与 envp[]。
    let mut strings = arch::USER_STACK_TOP;
    let execfn = stack_string(mm, &mut strings, &args.filename)?;
    debug_assert_eq!(execfn, layout.execfn);

    let mut pointers = layout.sp;
    stack_word(mm, &mut pointers, args.argv.len())?;
    for string in &args.argv {
        let address = stack_string(mm, &mut strings, string)?;
        stack_word(mm, &mut pointers, address)?;
    }
    stack_word(mm, &mut pointers, 0)?;

    for string in &args.envp {
        let address = stack_string(mm, &mut strings, string)?;
        stack_word(mm, &mut pointers, address)?;
    }
    stack_word(mm, &mut pointers, 0)?;

    // Step 3: 在字符串区底部写入 16 字节随机数，并在指针区顺序写入 17 对辅助向量（auxv）。
    debug_assert_eq!(strings - AT_RANDOM_BYTES, layout.random);
    let mut random_bytes = [0; AT_RANDOM_BYTES];
    random::fill_insecure(&mut random_bytes);
    copyout_exact(mm, layout.random, &random_bytes)?;

    let auxv = [
        (AT_PHDR, elf.phdr.unwrap_or(0)),
        (AT_PHENT, elf.phent),
        (AT_PHNUM, elf.phnum),
        (AT_PAGESZ, PAGE_SIZE),
        (AT_BASE, interpreter_base),
        (AT_FLAGS, 0),
        (AT_ENTRY, elf.entry),
        (AT_UID, uid as usize),
        (AT_EUID, uid as usize),
        (AT_GID, gid as usize),
        (AT_EGID, gid as usize),
        (AT_HWCAP, hwcap()),
        (AT_CLKTCK, USER_HZ as usize),
        (AT_SECURE, 0),
        (AT_RANDOM, layout.random),
        (AT_EXECFN, layout.execfn),
        (AT_NULL, 0),
    ];
    debug_assert_eq!(auxv.len(), AUXV_PAIRS);
    for (kind, value) in auxv {
        stack_word(mm, &mut pointers, kind)?;
        stack_word(mm, &mut pointers, value)?;
    }

    debug_assert!(pointers <= layout.random);
    Ok(layout.sp)
}

/// 将字节切片 `bytes` 及末尾 NUL 终止符写入栈字符串游标 `*cursor` 下方，并将游标更新为新字符串的起始虚拟地址。
fn stack_string(mm: &Mm, cursor: &mut usize, bytes: &[u8]) -> Result<usize, ExecError> {
    let address = *cursor - bytes.len() - 1;
    copyout_exact(mm, address, bytes)?;
    copyout_exact(mm, address + bytes.len(), &[0])?;
    *cursor = address;
    Ok(address)
}

/// 将单个 `usize` 机器字按小端序写入栈指针游标 `*cursor` 处，并将游标向高地址推进一个机器字宽度。
fn stack_word(mm: &Mm, cursor: &mut usize, word: usize) -> Result<(), ExecError> {
    copyout_exact(mm, *cursor, &word.to_le_bytes())?;
    *cursor += size_of::<usize>();
    Ok(())
}

/// 向候选地址空间 `mm` 的指定虚拟地址 `address` 完整拷贝 `bytes` 切片。
fn copyout_exact(mm: &Mm, address: usize, bytes: &[u8]) -> Result<(), ExecError> {
    let progress = mm.copyout(address, bytes);
    if let Some(error) = progress.error {
        return Err(ExecError::Vm(error));
    }
    assert_eq!(
        progress.copied,
        bytes.len(),
        "successful exec copy is incomplete"
    );
    Ok(())
}

/// 返回 RISC-V 64 架构的 `AT_HWCAP` 硬件能力位图（声明当前内核与上下文切换完整支持的 `IMAFDC` 扩展）。
#[cfg(target_arch = "riscv64")]
fn hwcap() -> usize {
    const HWCAP_ISA_A: usize = 1;
    (1 << (b'I' - b'A'))
        | (1 << (b'M' - b'A'))
        | HWCAP_ISA_A
        | (1 << (b'F' - b'A'))
        | (1 << (b'D' - b'A'))
        | (1 << (b'C' - b'A'))
}

/// 返回 LoongArch 64 架构的 `AT_HWCAP` 硬件能力位图（根据硬件探测及内核保存能力声明 `CPUCFG`、`FP`、`LSX` 与 `LASX` 扩展）。
#[cfg(target_arch = "loongarch64")]
fn hwcap() -> usize {
    const HWCAP_CPUCFG: usize = 1;
    const EUEN_TO_HWCAP_SHIFT: u32 = 3;
    let euen = crate::arch::user_euen();
    HWCAP_CPUCFG
        | ((euen & arch::EUEN_FP) << EUEN_TO_HWCAP_SHIFT)
        | ((euen & arch::EUEN_LSX) << EUEN_TO_HWCAP_SHIFT)
        | ((euen & arch::EUEN_LASX) << EUEN_TO_HWCAP_SHIFT)
}
