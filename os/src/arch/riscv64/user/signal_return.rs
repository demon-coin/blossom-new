//! RISC-V 64 用户态信号返回跳板（`rt_sigreturn` Trampoline）只读机器码定义。
//!
//! # 模块职责与架构定位
//! 本模块定义了用户态信号处理函数返回时执行的跳板指令序列，通过全局链接符号
//! `rv_user_signal_return_start` 与 `rv_user_signal_return_end` 暴露给 [`super::signal_return_code`]。
//! 地址空间管理模块（`crate::mm::space::Mm`）在创建用户页表时，会将这段原始机器码拷贝至固定虚拟页
//! `SIGNAL_RETURN_PAGE` 并映射为用户态只读可执行（U+R+X）。
//!
//! # 核心设计与指令布局
//! 当内核递送信号（[`super::UserContext::setup_signal`]）时，会将用户态返回地址寄存器 `ra`（`x1`）
//! 设为跳板页首地址。用户在信号处理函数末尾执行 `ret` 后即跳转至此，将系统调用号 `139`
//! （Linux RISC-V 64 `__NR_rt_sigreturn`）装入 `a7` 并执行 `ecall` 陷入内核恢复信号前现场。
//!
//! 跳板节（`.rodata.user_signal_return`，禁用 RVC 压缩指令与链接器松弛）的 8 字节布局如下：
//!
//! ```text
//! Offset  Bytes  Instruction     Description
//! ------  -----  --------------  -----------------------------------------------------
//! 0       4      li a7, 139      加载 __NR_rt_sigreturn (139) 到系统调用号寄存器 a7 (x17)
//! 4       4      ecall           触发环境调用异常陷入 S-mode，执行 rt_sigreturn
//! ```
//!
//! # 锁获取顺序与并发契约
//! 本模块仅包含编译期静态生成的只读数据节（`.rodata`），无运行期可变状态，不涉及任何锁。

use core::arch::global_asm;

// 显式开启 norvc 与 norelax 并按 4 字节对齐，确保生成的两条 32 位指令共 8 字节，不受编译单元压缩指令集或链接器松弛影响。
global_asm!(
    r#"
# 用户信号返回只发真实 rt_sigreturn，指令字节作为不可写内核 ABI 片段。
.section .rodata.user_signal_return, "a", @progbits

.option push
.option norvc
.option norelax
.balign 4
.global rv_user_signal_return_start
.type rv_user_signal_return_start, @object
rv_user_signal_return_start:
    li a7, 139
    ecall
.global rv_user_signal_return_end
rv_user_signal_return_end:
.size rv_user_signal_return_start, rv_user_signal_return_end-rv_user_signal_return_start
.option pop
"#,
);
