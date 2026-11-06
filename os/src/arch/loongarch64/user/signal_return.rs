//! LoongArch64 用户态信号返回跳板（Signal Return Trampoline）机器码定义。
//!
//! # 模块职责与架构定位
//! 本模块位于 `arch::loongarch64::user` 底层，通过 `global_asm!` 在内核只读数据段
//! （`.rodata.user_signal_return`）中定义一段位置无关的 `rt_sigreturn` 系统调用指令序列。
//! 上游由 [`super::signal_return_code`] 将起止符号 [`la_user_signal_return_start`] 与
//! [`la_user_signal_return_end`] 封装为 `&'static [u8]` 切片，并在进程地址空间初始化时
//! 由 [`Mm`](crate::mm::space::Mm) 复制到用户态专用的只读可执行（RX）信号跳板页中。
//!
//! # 核心设计与指令布局
//! 当用户态信号处理函数执行完毕并执行 `jr $ra` 返回时，CPU 跳转到用户跳板页中的以下两行指令：
//!
//! ```text
//! la_user_signal_return_start: (4 字节对齐)
//! +---------------------------------------------------------------+
//! | addi.d  $a7, $zero, 139   (4 B)  设置系统调用号 SYS_rt_sigreturn (139) |
//! | syscall 0                 (4 B)  陷入内核执行 rt_sigreturn 恢复原现场  |
//! +---------------------------------------------------------------+
//! la_user_signal_return_end:   (总长度 8 字节)
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 本模块仅包含编译期静态生成的只读机器码字节，运行期无可变状态且不涉及任何锁。

use core::arch::global_asm;

global_asm!(
    r#"
# 用户信号返回只发真实 rt_sigreturn，指令字节作为不可写内核 ABI 片段。
.section .rodata.user_signal_return, "a", @progbits

.balign 4
.global la_user_signal_return_start
.type la_user_signal_return_start, @object
la_user_signal_return_start:
    addi.d $a7, $zero, 139
    syscall 0
.global la_user_signal_return_end
la_user_signal_return_end:
.size la_user_signal_return_start, la_user_signal_return_end-la_user_signal_return_start
"#
);
