//! LoongArch64 调度器空闲等待（`IDLE`）与丢失唤醒防护窗口汇编实现。
//!
//! ## 模块职责与架构定位
//! 本模块位于 `arch::loongarch64` 架构层，提供全局汇编函数 `la_idle_once_asm`，供本核调度器在无就绪任务可运行时安全地开启中断并执行 `idle 0` 指令进入低功耗休眠。
//! - **上游调用者**：`arch::loongarch64::idle_once`（由调度器主循环在专属调度栈、本地中断关闭的状态下调用）。
//! - **协同组件**：`arch::loongarch64::loongarch_trap`（内核硬件中断入口）。当中断发生在 `[la_idle_window_start, la_idle_window_end)` 地址区间内时，中断处理函数会将异常返回地址（`ERA`）重定向到 `la_idle_window_exit`。
//!
//! ## 核心机制：丢失唤醒（Lost Wakeup）竞态与 PC 重写协议
//! 在操作系统内核中，“检查唤醒条件为空”与“开启中断并执行休眠指令（`idle 0`）”之间若不是原子的，就会出现经典的丢失唤醒（Lost Wakeup）死锁：
//! 如果开启中断（`CRMD.IE = 1`）之后、`idle 0` 执行之前恰好到来一次 IPI 或设备中断，且该中断将某个任务唤醒并放入运行队列，中断返回后 CPU 仍继续执行 `idle 0`，就会在已有就绪任务的情况下陷入无限休眠。
//!
//! 本模块通过**精确指令窗口 + Trap 返回 PC 重定向**解决该问题，布局如下：
//!
//! ```text
//! la_idle_once_asm:
//!   [入口准备] 装载 CRMD_IE 掩码到 $t0 / $t1 (此时中断仍关闭，不在重写窗口内)
//!       |
//!       v
//! la_idle_window_start:  <--- 32 字节对齐，共 8 条指令（32 字节）的可中断窗口起点
//!   1. csrxchg $t0, $t1, CSR_CRMD   (开启本核全局中断 CRMD.IE = 1)
//!   2. dbar 0                       (完整数据内存屏障，确保开启中断对后续重查可见)
//!   3. ld.d    $t0, $a0, 0          (重查本核 kick_pending 原子标志)
//!   4. bnez    $t0, .Lidle_skip     (若非零则跳过休眠，直接关中断返回)
//!   5. ld.bu   $t0, $a1, 0          (重查本核 wake_pending 原子标志)
//!   6. bnez    $t0, .Lidle_skip     (若非零则跳过休眠，直接关中断返回)
//!   7. nop                          (凑齐 8 条指令的 32 字节窗口边界)
//!   8. idle 0                       (使 CPU 核心休眠并等待中断到来)
//! la_idle_window_end:    <--- 可中断窗口终点
//!   [仅在 ktest 模式下] 原子递增 $a2 指向的 wait_completions 计数器
//!       |
//!       v
//! la_idle_window_exit / .Lidle_skip:
//!   [退出清理] csrxchg 清除 CRMD.IE 恢复关中断状态，jirl 返回调用方
//! ```
//!
//! ### 状态转移与中断交互契约
//! 1. **在 `csrxchg` 开启中断之后、`idle 0` 执行之前触发中断**：
//!    硬件将断点 `PC` 保存在 `ERA` 中，此时 `la_idle_window_start <= ERA < la_idle_window_end`。
//!    `loongarch_trap` 检测到 `ERA` 落在该窗口内，立即将栈帧中的 `frame.era` 修改为 `la_idle_window_exit`，随后处理中断。
//!    中断处理完毕执行 `ertn` 返回时，CPU 直接跳转到 `la_idle_window_exit` 关闭中断并返回调度器重新检查运行队列，绝不会再次进入 `idle 0`。
//! 2. **在 `idle 0` 休眠期间触发中断**：
//!    根据 LoongArch 架构规范，当 CPU 因中断从 `idle 0` 唤醒时，硬件根据具体实现可能将 `ERA` 指向 `idle 0` 本身或其下一条指令 `la_idle_window_end`：
//!    - 若 `ERA` 指向 `idle 0`（仍在窗口内），`loongarch_trap` 将其重定向到 `la_idle_window_exit`；
//!    - 若 `ERA` 指向 `la_idle_window_end`（已越过窗口终点），`ertn` 返回后顺序执行（在 `ktest` 下记录一次完整 `idle` 完成计数），随后到达 `la_idle_window_exit` 关闭中断返回。
//!
//! ## 调用 ABI 与锁契约
//! - **禁止持锁调用**：进入 `la_idle_once_asm` 前必须处于关中断（`CRMD.IE == 0`）状态，且不得持有任何自旋锁或抢占守卫。
//! - **参数约定**：
//!   - `$a0`：`*const AtomicUsize`（指向本核 `kick_pending` 标志）
//!   - `$a1`：`*const AtomicBool`（指向当前等待的 `wake_pending` 标志）
//!   - `$a2`（仅 `ktest` 启用时）：`*const AtomicUsize`（指向 `wait_completions` 统计计数器）
//! - **返回保证**：无论从哪条路径返回，函数退出时保证 `CRMD.IE == 0`（恢复关中断状态），且不改动 `CSR_KS0`（Per-CPU 基址）或浮点状态。

use crate::arch::loongarch64::register as reg;
use core::arch::global_asm;

global_asm!(
    // 第一段：函数入口准备与窗口 32 字节对齐。在开启中断前先把 CRMD_IE 掩码装入临时寄存器，使这段准备指令位于 PC 重定向窗口之外。
    r#"
.text

.balign 32
.global la_idle_once_asm
.type la_idle_once_asm, @function
la_idle_once_asm:
    ori $t0, $zero, {crmd_ie}
    ori $t1, $zero, {crmd_ie}
    b .Lidle_enable
    .balign 32
.global la_idle_window_start
la_idle_window_start:
.Lidle_enable:
"#,
    // 第二段：精确 8 条指令（32 字节）的开启中断与唤醒条件重查窗口。若在此窗口内被中断打断，trap 处理函数会将返回 PC 改写到 la_idle_window_exit。
    r#"
    # 精确 32 字节窗口覆盖启 IE、提示重查和 IDLE；窗口中断后返回调度器重查队列。
    csrxchg $t0, $t1, {csr_crmd}
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
    // 第三段：休眠完成统计（仅 ktest）与统一关中断退出路径。只清除 CRMD.IE 位，保留其他 CRMD 状态位及 KS0 不变。
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
    ori $t1, $zero, {crmd_ie}
    csrxchg $zero, $t1, {csr_crmd}
    jirl $zero, $ra, 0
.size la_idle_once_asm, .-la_idle_once_asm
"#,
    crmd_ie = const reg::CRMD_IE,
    csr_crmd = const reg::CSR_CRMD,
    execution_tests = const cfg!(feature = "ktest") as usize,
);
