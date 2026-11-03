//! 共享硬件向量先按 sscratch 分流，再在真实内核栈保存与恢复完整 trap 现场。

use crate::arch::riscv64::{rv_handle_trap, KernelTrapFrame, UserContext};
use core::{
    arch::global_asm,
    mem::{offset_of, size_of},
};

// 用户入口跨模块长跳转，内核入口保留原栈；两条路径均不能接受编译器序言。
global_asm!(
    // sscratch 非零仅指向受信用户现场；内核分支在覆盖临时寄存器前保存全部 GPR 和 CSR。
    r#"
.text

.option arch, +d
.balign 4
.global rv_trap_entry
rv_trap_entry:
    // sscratch 为零时保留原内核栈；用户执行时只接受内核预置的私有现场指针。
    csrrw sp, sscratch, sp
    beqz sp, .Lrv_kernel_trap
    // 不依赖外部入口位于 JAL 的 ±1MiB 内；先保存用户 t0，再用 PC 相对长跳转。
    sd t0, ({user_gpr} + 5 * {word})(sp)
    lla t0, rv_user_trap_entry
    jr t0
.Lrv_kernel_trap:
    csrrw sp, sscratch, sp
    addi sp, sp, -{frame_size}
    sd x1, ({gpr} + 1 * {word})(sp)
    sd x3, ({gpr} + 3 * {word})(sp)
    sd x4, ({gpr} + 4 * {word})(sp)
    sd x5, ({gpr} + 5 * {word})(sp)
    sd x6, ({gpr} + 6 * {word})(sp)
    sd x7, ({gpr} + 7 * {word})(sp)
    sd x8, ({gpr} + 8 * {word})(sp)
    sd x9, ({gpr} + 9 * {word})(sp)
    sd x10, ({gpr} + 10 * {word})(sp)
    sd x11, ({gpr} + 11 * {word})(sp)
    sd x12, ({gpr} + 12 * {word})(sp)
    sd x13, ({gpr} + 13 * {word})(sp)
    sd x14, ({gpr} + 14 * {word})(sp)
    sd x15, ({gpr} + 15 * {word})(sp)
    sd x16, ({gpr} + 16 * {word})(sp)
    sd x17, ({gpr} + 17 * {word})(sp)
    sd x18, ({gpr} + 18 * {word})(sp)
    sd x19, ({gpr} + 19 * {word})(sp)
    sd x20, ({gpr} + 20 * {word})(sp)
    sd x21, ({gpr} + 21 * {word})(sp)
    sd x22, ({gpr} + 22 * {word})(sp)
    sd x23, ({gpr} + 23 * {word})(sp)
    sd x24, ({gpr} + 24 * {word})(sp)
    sd x25, ({gpr} + 25 * {word})(sp)
    sd x26, ({gpr} + 26 * {word})(sp)
    sd x27, ({gpr} + 27 * {word})(sp)
    sd x28, ({gpr} + 28 * {word})(sp)
    sd x29, ({gpr} + 29 * {word})(sp)
    sd x30, ({gpr} + 30 * {word})(sp)
    sd x31, ({gpr} + 31 * {word})(sp)
    sd zero, {gpr}(sp)
    addi t0, sp, {frame_size}
    sd t0, ({gpr} + 2 * {word})(sp)
    csrr t0, sepc
    sd t0, {sepc}(sp)
    csrr t0, sstatus
    sd t0, {sstatus}(sp)
    csrr t0, scause
    sd t0, {scause}(sp)
    csrr t0, stval
    sd t0, {stval}(sp)
"#,
    // 只在原 FS 非 Off 时保存全部 FPR/FCSR，未保存字段仍由 MaybeUninit 表达。
    r#"
    ld t0, {sstatus}(sp)
    li t1, 0x6000
    and t0, t0, t1
    beqz t0, 1f
    fsd f0, ({fpr} + 0 * {float_word})(sp)
    fsd f1, ({fpr} + 1 * {float_word})(sp)
    fsd f2, ({fpr} + 2 * {float_word})(sp)
    fsd f3, ({fpr} + 3 * {float_word})(sp)
    fsd f4, ({fpr} + 4 * {float_word})(sp)
    fsd f5, ({fpr} + 5 * {float_word})(sp)
    fsd f6, ({fpr} + 6 * {float_word})(sp)
    fsd f7, ({fpr} + 7 * {float_word})(sp)
    fsd f8, ({fpr} + 8 * {float_word})(sp)
    fsd f9, ({fpr} + 9 * {float_word})(sp)
    fsd f10, ({fpr} + 10 * {float_word})(sp)
    fsd f11, ({fpr} + 11 * {float_word})(sp)
    fsd f12, ({fpr} + 12 * {float_word})(sp)
    fsd f13, ({fpr} + 13 * {float_word})(sp)
    fsd f14, ({fpr} + 14 * {float_word})(sp)
    fsd f15, ({fpr} + 15 * {float_word})(sp)
    fsd f16, ({fpr} + 16 * {float_word})(sp)
    fsd f17, ({fpr} + 17 * {float_word})(sp)
    fsd f18, ({fpr} + 18 * {float_word})(sp)
    fsd f19, ({fpr} + 19 * {float_word})(sp)
    fsd f20, ({fpr} + 20 * {float_word})(sp)
    fsd f21, ({fpr} + 21 * {float_word})(sp)
    fsd f22, ({fpr} + 22 * {float_word})(sp)
    fsd f23, ({fpr} + 23 * {float_word})(sp)
    fsd f24, ({fpr} + 24 * {float_word})(sp)
    fsd f25, ({fpr} + 25 * {float_word})(sp)
    fsd f26, ({fpr} + 26 * {float_word})(sp)
    fsd f27, ({fpr} + 27 * {float_word})(sp)
    fsd f28, ({fpr} + 28 * {float_word})(sp)
    fsd f29, ({fpr} + 29 * {float_word})(sp)
    fsd f30, ({fpr} + 30 * {float_word})(sp)
    fsd f31, ({fpr} + 31 * {float_word})(sp)
    frcsr t0
    sd t0, {fcsr}(sp)
1:
"#,
    // 关闭 FS 后以完整、独占的 frame 调用普通 Rust 分派。
    r#"
    li t0, 0x6000
    csrc sstatus, t0
    mv a0, sp
    call {handle_trap}
"#,
    // 按保存的 FS 状态恢复浮点现场，普通内核路径不执行浮点访问。
    r#"
    ld t0, {sstatus}(sp)
    csrw sstatus, t0
    li t1, 0x6000
    and t0, t0, t1
    beqz t0, 2f
    fld f0, ({fpr} + 0 * {float_word})(sp)
    fld f1, ({fpr} + 1 * {float_word})(sp)
    fld f2, ({fpr} + 2 * {float_word})(sp)
    fld f3, ({fpr} + 3 * {float_word})(sp)
    fld f4, ({fpr} + 4 * {float_word})(sp)
    fld f5, ({fpr} + 5 * {float_word})(sp)
    fld f6, ({fpr} + 6 * {float_word})(sp)
    fld f7, ({fpr} + 7 * {float_word})(sp)
    fld f8, ({fpr} + 8 * {float_word})(sp)
    fld f9, ({fpr} + 9 * {float_word})(sp)
    fld f10, ({fpr} + 10 * {float_word})(sp)
    fld f11, ({fpr} + 11 * {float_word})(sp)
    fld f12, ({fpr} + 12 * {float_word})(sp)
    fld f13, ({fpr} + 13 * {float_word})(sp)
    fld f14, ({fpr} + 14 * {float_word})(sp)
    fld f15, ({fpr} + 15 * {float_word})(sp)
    fld f16, ({fpr} + 16 * {float_word})(sp)
    fld f17, ({fpr} + 17 * {float_word})(sp)
    fld f18, ({fpr} + 18 * {float_word})(sp)
    fld f19, ({fpr} + 19 * {float_word})(sp)
    fld f20, ({fpr} + 20 * {float_word})(sp)
    fld f21, ({fpr} + 21 * {float_word})(sp)
    fld f22, ({fpr} + 22 * {float_word})(sp)
    fld f23, ({fpr} + 23 * {float_word})(sp)
    fld f24, ({fpr} + 24 * {float_word})(sp)
    fld f25, ({fpr} + 25 * {float_word})(sp)
    fld f26, ({fpr} + 26 * {float_word})(sp)
    fld f27, ({fpr} + 27 * {float_word})(sp)
    fld f28, ({fpr} + 28 * {float_word})(sp)
    fld f29, ({fpr} + 29 * {float_word})(sp)
    fld f30, ({fpr} + 30 * {float_word})(sp)
    fld f31, ({fpr} + 31 * {float_word})(sp)
    ld t0, {fcsr}(sp)
    fscsr t0
2:
"#,
    // 恢复原 PC、状态、全部整数寄存器与 SP，最后执行 sret。
    r#"
    ld t0, {sepc}(sp)
    csrw sepc, t0
    ld t0, {sstatus}(sp)
    csrw sstatus, t0
    ld x1, ({gpr} + 1 * {word})(sp)
    ld x3, ({gpr} + 3 * {word})(sp)
    ld x4, ({gpr} + 4 * {word})(sp)
    ld x5, ({gpr} + 5 * {word})(sp)
    ld x6, ({gpr} + 6 * {word})(sp)
    ld x7, ({gpr} + 7 * {word})(sp)
    ld x8, ({gpr} + 8 * {word})(sp)
    ld x9, ({gpr} + 9 * {word})(sp)
    ld x10, ({gpr} + 10 * {word})(sp)
    ld x11, ({gpr} + 11 * {word})(sp)
    ld x12, ({gpr} + 12 * {word})(sp)
    ld x13, ({gpr} + 13 * {word})(sp)
    ld x14, ({gpr} + 14 * {word})(sp)
    ld x15, ({gpr} + 15 * {word})(sp)
    ld x16, ({gpr} + 16 * {word})(sp)
    ld x17, ({gpr} + 17 * {word})(sp)
    ld x18, ({gpr} + 18 * {word})(sp)
    ld x19, ({gpr} + 19 * {word})(sp)
    ld x20, ({gpr} + 20 * {word})(sp)
    ld x21, ({gpr} + 21 * {word})(sp)
    ld x22, ({gpr} + 22 * {word})(sp)
    ld x23, ({gpr} + 23 * {word})(sp)
    ld x24, ({gpr} + 24 * {word})(sp)
    ld x25, ({gpr} + 25 * {word})(sp)
    ld x26, ({gpr} + 26 * {word})(sp)
    ld x27, ({gpr} + 27 * {word})(sp)
    ld x28, ({gpr} + 28 * {word})(sp)
    ld x29, ({gpr} + 29 * {word})(sp)
    ld x30, ({gpr} + 30 * {word})(sp)
    ld x31, ({gpr} + 31 * {word})(sp)
    ld sp, ({gpr} + 2 * {word})(sp)
    sret
"#,
    frame_size = const size_of::<KernelTrapFrame>(),
    gpr = const offset_of!(KernelTrapFrame, gpr),
    user_gpr = const offset_of!(UserContext, gpr),
    word = const size_of::<usize>(),
    sepc = const offset_of!(KernelTrapFrame, sepc),
    sstatus = const offset_of!(KernelTrapFrame, sstatus),
    scause = const offset_of!(KernelTrapFrame, scause),
    stval = const offset_of!(KernelTrapFrame, stval),
    fpr = const offset_of!(KernelTrapFrame, fpr),
    float_word = const size_of::<u64>(),
    fcsr = const offset_of!(KernelTrapFrame, fcsr),
    handle_trap = sym rv_handle_trap,
);
