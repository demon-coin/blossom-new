//! 调度栈等待边界；启 IRQ 至 IDLE 的精确窗口供 trap 修改返回 PC。

use core::arch::global_asm;

global_asm!(
    // 窗口起点单独按 32 字节对齐，入口准备不属于可改写返回 PC 的窗口。
    r#"
.text

.balign 32
.global la_idle_once_asm
.type la_idle_once_asm, @function
la_idle_once_asm:
    ori $t0, $zero, 4
    ori $t1, $zero, 4
    b .Lidle_enable
    .balign 32
.global la_idle_window_start
la_idle_window_start:
.Lidle_enable:
"#,
    // 八条指令覆盖启 IE、屏障、提示重查和 IDLE，三个全局标签保持机器协议。
    r#"
    # 精确 32 字节窗口覆盖启 IE、提示重查和 IDLE；窗口中断后返回调度器重查队列。
    csrxchg $t0, $t1, 0
    dbar 0
    ld.d $t0, $a0, 0
    bnez $t0, .Lidle_skip
    ld.bu $t0, $a1, 0
    bnez $t0, .Lidle_skip
    nop
    idle 0
.global la_idle_window_end
la_idle_window_end:
"#,
    // 仅真正完成 IDLE 才累加测试计数，全部退出路径只清 IE 并保留 KS0。
    r#"
.if {execution_tests}
    # 只有 IDLE 的完成路径到达这里；原子旧值、增量和地址使用三个不同寄存器。
    ori $t0, $zero, 1
    amadd_db.d $t1, $t0, $a2
.endif
.global la_idle_window_exit
la_idle_window_exit:
.Lidle_skip:
    # 返回调度栈之前只关闭 IE，不覆盖其他 CRMD 位，也不修改 FP 或 KS0。
    ori $t1, $zero, 4
    csrxchg $zero, $t1, 0
    jirl $zero, $ra, 0
.size la_idle_once_asm, .-la_idle_once_asm
"#,
    execution_tests = const cfg!(feature = "ktest") as usize,
);
