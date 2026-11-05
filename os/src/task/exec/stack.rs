//! 初始栈先计算实际布局，再登记有限启动范围，字符串与指针分别顺序交付。

use core::mem::size_of;

use crate::arch;
use crate::mm::space::{Mm, VmError};
use crate::mm::user_table::PagePermissions;
use crate::mm::PAGE_SIZE;
use crate::random;
use crate::task::time::USER_HZ;

use super::elf::Elf;
use super::{Arguments, ExecError, RWX};

const STARTUP_HEADROOM: usize = 128 * 1024;
const AUXV_PAIRS: usize = 17;
const AT_RANDOM_BYTES: usize = 16;

const AT_NULL: usize = 0;
const AT_PHDR: usize = 3;
const AT_PHENT: usize = 4;
const AT_PHNUM: usize = 5;
const AT_PAGESZ: usize = 6;
const AT_BASE: usize = 7;
const AT_FLAGS: usize = 8;
const AT_ENTRY: usize = 9;
const AT_UID: usize = 11;
const AT_EUID: usize = 12;
const AT_GID: usize = 13;
const AT_EGID: usize = 14;
const AT_HWCAP: usize = 16;
const AT_CLKTCK: usize = 17;
const AT_SECURE: usize = 23;
const AT_RANDOM: usize = 25;
const AT_EXECFN: usize = 31;

/// 保存纯计算的初始范围，不将运行中增长限额缓存进 Mm。
struct Layout {
    bottom: usize,
    sp: usize,
    execfn: usize,
    random: usize,
}

impl Layout {
    /// 参数预算与真实栈空间分别计算，软限额较小时仍保留全部实际参数页。
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

/// 离线候选先登记全部写入页，再按已知长度推进两个游标，不另存参数指针数组。
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

/// 布局已核对完整长度，复制原字节及唯一 NUL 后向下推进字符串游标。
fn stack_string(mm: &Mm, cursor: &mut usize, bytes: &[u8]) -> Result<usize, ExecError> {
    let address = *cursor - bytes.len() - 1;
    copyout_exact(mm, address, bytes)?;
    copyout_exact(mm, address + bytes.len(), &[0])?;
    *cursor = address;
    Ok(address)
}

/// 初始栈整数以目标小端机器字交付，不创建指向用户页的 Rust 引用。
fn stack_word(mm: &Mm, cursor: &mut usize, word: usize) -> Result<(), ExecError> {
    copyout_exact(mm, *cursor, &word.to_le_bytes())?;
    *cursor += size_of::<usize>();
    Ok(())
}

/// 映像准备只接受完整用户拷贝，部分成功不会发表为完整栈。
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

/// 当前 RV64GC 平台与用户现场共同支持 IMAFDC；不保存的 V 不对用户声明。
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

/// 仅声明实际 CPU 存在且用户入口、复制与信号路径完整保存的浮点和向量扩展。
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
