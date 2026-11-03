//! 内核异常与物理 refill 的硬件入口；两者的 CSR 和 frame 不混用。

use crate::arch::loongarch64::{KernelTrapFrame, RefillFrame};
use core::arch::global_asm;
use core::mem::{offset_of, size_of};

global_asm!(
    // 普通异常先保存全部整数寄存器和原 SP，再记录异常 CSR 与 KS0。
    r#"
.text

# EENTRY 低 12 位只读零，异常入口必须按 4KiB 对齐，不能只按 64 字节对齐。
.balign 4096
.global la_kernel_trap
la_kernel_trap:
    addi.d $sp, $sp, -{frame_size}
    st.d $r0, $sp, {gpr}+0*8
    st.d $r1, $sp, {gpr}+1*8
    st.d $r2, $sp, {gpr}+2*8
    st.d $r4, $sp, {gpr}+4*8
    st.d $r5, $sp, {gpr}+5*8
    st.d $r6, $sp, {gpr}+6*8
    st.d $r7, $sp, {gpr}+7*8
    st.d $r8, $sp, {gpr}+8*8
    st.d $r9, $sp, {gpr}+9*8
    st.d $r10, $sp, {gpr}+10*8
    st.d $r11, $sp, {gpr}+11*8
    st.d $r12, $sp, {gpr}+12*8
    st.d $r13, $sp, {gpr}+13*8
    st.d $r14, $sp, {gpr}+14*8
    st.d $r15, $sp, {gpr}+15*8
    st.d $r16, $sp, {gpr}+16*8
    st.d $r17, $sp, {gpr}+17*8
    st.d $r18, $sp, {gpr}+18*8
    st.d $r19, $sp, {gpr}+19*8
    st.d $r20, $sp, {gpr}+20*8
    st.d $r21, $sp, {gpr}+21*8
    st.d $r22, $sp, {gpr}+22*8
    st.d $r23, $sp, {gpr}+23*8
    st.d $r24, $sp, {gpr}+24*8
    st.d $r25, $sp, {gpr}+25*8
    st.d $r26, $sp, {gpr}+26*8
    st.d $r27, $sp, {gpr}+27*8
    st.d $r28, $sp, {gpr}+28*8
    st.d $r29, $sp, {gpr}+29*8
    st.d $r30, $sp, {gpr}+30*8
    st.d $r31, $sp, {gpr}+31*8
    addi.d $t0, $sp, {frame_size}
    st.d $t0, $sp, {gpr}+3*8
    csrrd $t0, 6
    st.d $t0, $sp, {era}
    csrrd $t0, 1
    st.d $t0, $sp, {prmd}
    csrrd $t0, 5
    st.d $t0, $sp, {estat}
    csrrd $t0, 7
    st.d $t0, $sp, {badv}
    csrrd $t0, 2
    st.d $t0, $sp, {euen}
    csrrd $t0, 48
    st.d $t0, $sp, {cpu_base}
"#,
    // 只有原 EUEN.FPE 开启时才保存全部标量 FP、FCSR 与八个 FCC。
    r#"
    ld.d $t0, $sp, {euen}
    andi $t0, $t0, 1
    beqz $t0, .Lla_fp_saved
    fst.d $f0, $sp, {fpr}+0*8
    fst.d $f1, $sp, {fpr}+1*8
    fst.d $f2, $sp, {fpr}+2*8
    fst.d $f3, $sp, {fpr}+3*8
    fst.d $f4, $sp, {fpr}+4*8
    fst.d $f5, $sp, {fpr}+5*8
    fst.d $f6, $sp, {fpr}+6*8
    fst.d $f7, $sp, {fpr}+7*8
    fst.d $f8, $sp, {fpr}+8*8
    fst.d $f9, $sp, {fpr}+9*8
    fst.d $f10, $sp, {fpr}+10*8
    fst.d $f11, $sp, {fpr}+11*8
    fst.d $f12, $sp, {fpr}+12*8
    fst.d $f13, $sp, {fpr}+13*8
    fst.d $f14, $sp, {fpr}+14*8
    fst.d $f15, $sp, {fpr}+15*8
    fst.d $f16, $sp, {fpr}+16*8
    fst.d $f17, $sp, {fpr}+17*8
    fst.d $f18, $sp, {fpr}+18*8
    fst.d $f19, $sp, {fpr}+19*8
    fst.d $f20, $sp, {fpr}+20*8
    fst.d $f21, $sp, {fpr}+21*8
    fst.d $f22, $sp, {fpr}+22*8
    fst.d $f23, $sp, {fpr}+23*8
    fst.d $f24, $sp, {fpr}+24*8
    fst.d $f25, $sp, {fpr}+25*8
    fst.d $f26, $sp, {fpr}+26*8
    fst.d $f27, $sp, {fpr}+27*8
    fst.d $f28, $sp, {fpr}+28*8
    fst.d $f29, $sp, {fpr}+29*8
    fst.d $f30, $sp, {fpr}+30*8
    fst.d $f31, $sp, {fpr}+31*8
    movfcsr2gr $t0, $fcsr0
    st.d $t0, $sp, {fcsr}
    movcf2gr $t0, $fcc0
    st.d $t0, $sp, {fcc}+0*8
    movcf2gr $t0, $fcc1
    st.d $t0, $sp, {fcc}+1*8
    movcf2gr $t0, $fcc2
    st.d $t0, $sp, {fcc}+2*8
    movcf2gr $t0, $fcc3
    st.d $t0, $sp, {fcc}+3*8
    movcf2gr $t0, $fcc4
    st.d $t0, $sp, {fcc}+4*8
    movcf2gr $t0, $fcc5
    st.d $t0, $sp, {fcc}+5*8
    movcf2gr $t0, $fcc6
    st.d $t0, $sp, {fcc}+6*8
    movcf2gr $t0, $fcc7
    st.d $t0, $sp, {fcc}+7*8
"#,
    // 完整现场建立后调用 Rust，返回时先恢复 EUEN 再恢复条件保存的 FP。
    r#"
.Lla_fp_saved:
    move $a0, $sp
    bl loongarch_trap
    ld.d $t0, $sp, {euen}
    csrwr $t0, 2
    ld.d $t0, $sp, {euen}
    andi $t0, $t0, 1
    beqz $t0, .Lla_fp_restored
    fld.d $f0, $sp, {fpr}+0*8
    fld.d $f1, $sp, {fpr}+1*8
    fld.d $f2, $sp, {fpr}+2*8
    fld.d $f3, $sp, {fpr}+3*8
    fld.d $f4, $sp, {fpr}+4*8
    fld.d $f5, $sp, {fpr}+5*8
    fld.d $f6, $sp, {fpr}+6*8
    fld.d $f7, $sp, {fpr}+7*8
    fld.d $f8, $sp, {fpr}+8*8
    fld.d $f9, $sp, {fpr}+9*8
    fld.d $f10, $sp, {fpr}+10*8
    fld.d $f11, $sp, {fpr}+11*8
    fld.d $f12, $sp, {fpr}+12*8
    fld.d $f13, $sp, {fpr}+13*8
    fld.d $f14, $sp, {fpr}+14*8
    fld.d $f15, $sp, {fpr}+15*8
    fld.d $f16, $sp, {fpr}+16*8
    fld.d $f17, $sp, {fpr}+17*8
    fld.d $f18, $sp, {fpr}+18*8
    fld.d $f19, $sp, {fpr}+19*8
    fld.d $f20, $sp, {fpr}+20*8
    fld.d $f21, $sp, {fpr}+21*8
    fld.d $f22, $sp, {fpr}+22*8
    fld.d $f23, $sp, {fpr}+23*8
    fld.d $f24, $sp, {fpr}+24*8
    fld.d $f25, $sp, {fpr}+25*8
    fld.d $f26, $sp, {fpr}+26*8
    fld.d $f27, $sp, {fpr}+27*8
    fld.d $f28, $sp, {fpr}+28*8
    fld.d $f29, $sp, {fpr}+29*8
    fld.d $f30, $sp, {fpr}+30*8
    fld.d $f31, $sp, {fpr}+31*8
    ld.d $t0, $sp, {fcsr}
    movgr2fcsr $fcsr0, $t0
    ld.d $t0, $sp, {fcc}+0*8
    movgr2cf $fcc0, $t0
    ld.d $t0, $sp, {fcc}+1*8
    movgr2cf $fcc1, $t0
    ld.d $t0, $sp, {fcc}+2*8
    movgr2cf $fcc2, $t0
    ld.d $t0, $sp, {fcc}+3*8
    movgr2cf $fcc3, $t0
    ld.d $t0, $sp, {fcc}+4*8
    movgr2cf $fcc4, $t0
    ld.d $t0, $sp, {fcc}+5*8
    movgr2cf $fcc5, $t0
    ld.d $t0, $sp, {fcc}+6*8
    movgr2cf $fcc6, $t0
    ld.d $t0, $sp, {fcc}+7*8
    movgr2cf $fcc7, $t0
"#,
    // 恢复原异常返回状态、KS0 和整数现场，最后以原 SP 执行 ERTN。
    r#"
.Lla_fp_restored:
    ld.d $t0, $sp, {era}
    csrwr $t0, 6
    ld.d $t0, $sp, {prmd}
    csrwr $t0, 1
    ld.d $t0, $sp, {cpu_base}
    csrwr $t0, 48
    ld.d $r1, $sp, {gpr}+1*8
    ld.d $r2, $sp, {gpr}+2*8
    ld.d $r4, $sp, {gpr}+4*8
    ld.d $r5, $sp, {gpr}+5*8
    ld.d $r6, $sp, {gpr}+6*8
    ld.d $r7, $sp, {gpr}+7*8
    ld.d $r8, $sp, {gpr}+8*8
    ld.d $r9, $sp, {gpr}+9*8
    ld.d $r10, $sp, {gpr}+10*8
    ld.d $r11, $sp, {gpr}+11*8
    ld.d $r12, $sp, {gpr}+12*8
    ld.d $r13, $sp, {gpr}+13*8
    ld.d $r14, $sp, {gpr}+14*8
    ld.d $r15, $sp, {gpr}+15*8
    ld.d $r16, $sp, {gpr}+16*8
    ld.d $r17, $sp, {gpr}+17*8
    ld.d $r18, $sp, {gpr}+18*8
    ld.d $r19, $sp, {gpr}+19*8
    ld.d $r20, $sp, {gpr}+20*8
    ld.d $r21, $sp, {gpr}+21*8
    ld.d $r22, $sp, {gpr}+22*8
    ld.d $r23, $sp, {gpr}+23*8
    ld.d $r24, $sp, {gpr}+24*8
    ld.d $r25, $sp, {gpr}+25*8
    ld.d $r26, $sp, {gpr}+26*8
    ld.d $r27, $sp, {gpr}+27*8
    ld.d $r28, $sp, {gpr}+28*8
    ld.d $r29, $sp, {gpr}+29*8
    ld.d $r30, $sp, {gpr}+30*8
    ld.d $r31, $sp, {gpr}+31*8
    ld.d $sp, $sp, {gpr}+3*8
    ertn

"#,
    // 物理 refill 独立按页对齐；用户路径保留 TLBRSAVE/KS2 并装入完整偶奇叶。
    r#"
# TLBRENTRY 低 12 位只读零，重填入口必须按 4KiB 对齐，不能只按 64 字节对齐。
.balign 4096
.global la_kernel_refill
la_kernel_refill:
    # 用户 refill 把 t0 存入 TLBRSAVE、t1 存入临时 KS2，不触碰用户 SP 或 KS0。
    csrwr $t0, 139
    csrrd $t0, 143
    andi $t0, $t0, 3
    xori $t0, $t0, 3
    beqz $t0, .Luser_refill
    # PLV0 的用户入口窗口仍可能持有 context/用户 SP；只在该窗口借 KS3 续接栈。
    # KS2 是非返回的内核 refill 临时槽，原 GPR 在压致命 frame 前恢复。
    csrwr $t1, 50
    csrrd $t0, 12
    la.abs $t1, la_user_trap
    bne $t0, $t1, .Lordinary_kernel_refill
    b la_user_kernel_refill_fault
.Lordinary_kernel_refill:
    csrrd $t1, 50
    csrrd $t0, 139
    b .Lkernel_refill_fatal
.Luser_refill:
    # 三层只索引低 39 位；超出政策范围的地址只能走永久 invalid 链。
    csrrd $t0, 137
    srli.d $t0, $t0, 39
    bnez $t0, .Linvalid_user_root
    csrrd $t0, 25
    b .Luser_refill_walk
.Linvalid_user_root:
    la.abs $t0, LA_INVALID_ROOT
    bstrpick.d $t0, $t0, 47, 0
.Luser_refill_walk:
    # PWCH DIR2 / PWCL DIR0 / PT 构成固定三级；空目录总指向永久 invalid 页。
    lddir $t0, $t0, 3
    lddir $t0, $t0, 1
    # QEMU 9.2.1 的 LDPTE 会截断高权限位；固定 4KiB 叶以完整 64 位装入偶奇 CSR。
    # KS2 只暂存 t1，真正用户 trap 会重新保存 t0；TLBREHI 保留本次 VPPN 和既有 PS12。
    csrwr $t1, 50
    csrrd $t1, 137
    bstrpick.d $t1, $t1, 20, 13
    slli.d $t1, $t1, 4
    add.d $t0, $t0, $t1
    ld.d $t1, $t0, 8
    csrwr $t1, 141
    ld.d $t0, $t0, 0
    csrwr $t0, 140
    csrrd $t1, 50
    tlbfill
    csrrd $t0, 139
    ertn
"#,
    // 内核 refill 在 DA 下保存独立 frame，切回高别名和 DMW 后报告致命异常。
    r#"
.Lkernel_refill_fatal:
    # refill 自动处于 DA；高地址栈仍以物理掩码可访问。
    addi.d $sp, $sp, -{refill_size}
    st.d $r0, $sp, {refill_gpr}+0*8
    st.d $r1, $sp, {refill_gpr}+1*8
    st.d $r2, $sp, {refill_gpr}+2*8
    st.d $r4, $sp, {refill_gpr}+4*8
    st.d $r5, $sp, {refill_gpr}+5*8
    st.d $r6, $sp, {refill_gpr}+6*8
    st.d $r7, $sp, {refill_gpr}+7*8
    st.d $r8, $sp, {refill_gpr}+8*8
    st.d $r9, $sp, {refill_gpr}+9*8
    st.d $r10, $sp, {refill_gpr}+10*8
    st.d $r11, $sp, {refill_gpr}+11*8
    st.d $r12, $sp, {refill_gpr}+12*8
    st.d $r13, $sp, {refill_gpr}+13*8
    st.d $r14, $sp, {refill_gpr}+14*8
    st.d $r15, $sp, {refill_gpr}+15*8
    st.d $r16, $sp, {refill_gpr}+16*8
    st.d $r17, $sp, {refill_gpr}+17*8
    st.d $r18, $sp, {refill_gpr}+18*8
    st.d $r19, $sp, {refill_gpr}+19*8
    st.d $r20, $sp, {refill_gpr}+20*8
    st.d $r21, $sp, {refill_gpr}+21*8
    st.d $r22, $sp, {refill_gpr}+22*8
    st.d $r23, $sp, {refill_gpr}+23*8
    st.d $r24, $sp, {refill_gpr}+24*8
    st.d $r25, $sp, {refill_gpr}+25*8
    st.d $r26, $sp, {refill_gpr}+26*8
    st.d $r27, $sp, {refill_gpr}+27*8
    st.d $r28, $sp, {refill_gpr}+28*8
    st.d $r29, $sp, {refill_gpr}+29*8
    st.d $r30, $sp, {refill_gpr}+30*8
    st.d $r31, $sp, {refill_gpr}+31*8
    addi.d $t0, $sp, {refill_size}
    st.d $t0, $sp, {refill_gpr}+3*8
    csrrd $t0, 138
    st.d $t0, $sp, {refill_era}
    csrrd $t0, 143
    st.d $t0, $sp, {refill_prmd}
    csrrd $t0, 137
    st.d $t0, $sp, {refill_badv}
    # TLBRENTRY 是物理入口；DA 下先转高别名再恢复 DMW。
    la.abs $t0, .Lrefill_high
    jirl $zero, $t0, 0
.Lrefill_high:
    li.w $t0, 0x10
    li.w $t1, 0x18
    csrxchg $t0, $t1, 0
    move $a0, $sp
    bl loongarch_refill
"#,
    frame_size = const size_of::<KernelTrapFrame>(),
    gpr = const offset_of!(KernelTrapFrame, gpr),
    era = const offset_of!(KernelTrapFrame, era),
    prmd = const offset_of!(KernelTrapFrame, prmd),
    estat = const offset_of!(KernelTrapFrame, estat),
    badv = const offset_of!(KernelTrapFrame, badv),
    euen = const offset_of!(KernelTrapFrame, euen),
    cpu_base = const offset_of!(KernelTrapFrame, cpu_base),
    fpr = const offset_of!(KernelTrapFrame, fpr),
    fcsr = const offset_of!(KernelTrapFrame, fcsr),
    fcc = const offset_of!(KernelTrapFrame, fcc),
    refill_size = const size_of::<RefillFrame>(),
    refill_gpr = const offset_of!(RefillFrame, gpr),
    refill_era = const offset_of!(RefillFrame, era),
    refill_prmd = const offset_of!(RefillFrame, prmd),
    refill_badv = const offset_of!(RefillFrame, badv),
);
