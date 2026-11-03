//! 真实 timer trap 的寄存器往返探针，只在受控 ktest 环境启用。

use core::{arch::naked_asm, sync::atomic::AtomicUsize};

/// 临时开放 IRQ 等待本核计数推进，检查整数及可选 FP 图样并恢复调用者的整数 ABI 与状态。
///
/// # Safety
/// counter 必须指向永久、对齐的本核 AtomicUsize，expected 必须由已编程 timer 推进；
/// 本核 trap、timer、CpuLocal 与有效栈必须已安装，调用期间固定 CPU 且禁止调度迁移。
/// CPU 必须支持 D，无活跃 FP 现场，fp 只能为 0 或 1；fp=1 会覆盖 f0/f31。
/// 不得持有中断处理器会获取的锁；调用者须在同核 IRQ-off 下交还生产 timer deadline。
#[naked]
#[no_mangle]
pub(super) unsafe extern "C" fn rv_register_roundtrip(
    counter: *const AtomicUsize,
    expected: usize,
    fp: usize,
) -> usize {
    // SAFETY: 调用者保证本核 trap/timer、永久计数器及当前栈有效且期间不调度；
    // 手动帧保存原整数 ABI 和控制状态，只有显式 FP 模式触及约定的浮点寄存器。
    unsafe {
        naked_asm!(
            ".option push",
            ".option arch, +d",
            // 固定 112 字节 ABI 栈帧保存调用者及原 sstatus，记录本核 SP/TP。
            r#"
    addi sp, sp, -112
    sd ra, 0(sp)
    sd s0, 8(sp)
    sd s1, 16(sp)
    sd s2, 24(sp)
    sd s3, 32(sp)
    sd s4, 40(sp)
    sd s5, 48(sp)
    csrr t0, sstatus
    sd t0, 56(sp)
    mv s0, sp
    mv s1, tp
    mv s3, a0
    mv s4, a1
    mv s5, a2
"#,
            // 仅受控 FP 模式启用 D 并设置图样；整数模式保持 FS Off。
            r#"
    beqz s5, 1f
    li t0, 0x6000
    csrs sstatus, t0
    frcsr t0
    sd t0, 64(sp)
    li t0, 0x61
    fscsr t0
    li t0, 0x3ff123456789abcd
    fmv.d.x f0, t0
    li t0, 0x400fedcba9876543
    fmv.d.x f31, t0
    j 2f
1:  li t0, 0x6000
    csrc sstatus, t0
2:  li t0, 0x123456789abcdef
    li t1, 0x76543210fedcba98
    li a3, 0x13579bdf2468ace0
    li s2, 0x1122334455667788
"#,
            // 真正开放 IRQ 并等待 timer 计数推进，再在 IRQ-off 下检查完整图样。
            r#"
    csrsi sstatus, 2
3:  ld a0, 0(s3)
    fence r, r
    bltu a0, s4, 3b
    csrci sstatus, 2
    li a0, 0
    bne sp, s0, 9f
    bne tp, s1, 9f
    li a1, 0x123456789abcdef
    bne t0, a1, 9f
    li a1, 0x76543210fedcba98
    bne t1, a1, 9f
    li a1, 0x13579bdf2468ace0
    bne a3, a1, 9f
    li a1, 0x1122334455667788
    bne s2, a1, 9f
    beqz s5, 8f
    fmv.x.d a1, f0
    li a2, 0x3ff123456789abcd
    bne a1, a2, 9f
    fmv.x.d a1, f31
    li a2, 0x400fedcba9876543
    bne a1, a2, 9f
    frcsr a1
    li a2, 0x61
    bne a1, a2, 9f
8:  li a0, 1
"#,
            // 恢复原 FCSR、sstatus 和整数 callee-saved；不承诺保存被测 f0/f31。
            r#"
9:  beqz s5, 10f
    ld t0, 64(sp)
    fscsr t0
10: ld t0, 56(sp)
    csrw sstatus, t0
    ld ra, 0(sp)
    ld s0, 8(sp)
    ld s1, 16(sp)
    ld s2, 24(sp)
    ld s3, 32(sp)
    ld s4, 40(sp)
    ld s5, 48(sp)
    addi sp, sp, 112
    ret
"#,
            ".option pop",
        );
    }
}
