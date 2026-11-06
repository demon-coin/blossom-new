//! 任务上下文（TaskContext）与寄存器现场定义。
//!
//! ## 模块职责与架构定位
//! 本模块定义内核任务在主动或受控上下文切换（`switch_context`）时保存的被调用者保存寄存器（Callee-saved Registers）布局。
//! - **上游调用者**：`crate::sched`（在每个 CPU 的静态调度器上下文与具体 `Task` 上下文之间切换）以及 `crate::task`（构造新任务的初始上下文）。
//! - **下游依赖**：`crate::arch::switch_context` 汇编例程（直接按本模块的 `#[repr(C)]` 偏移读写寄存器）与 `crate::sched::task_trampoline`（新任务首次切入后的入口跳板）。
//!
//! ## 核心设计与内存布局
//! 上下文切换发生在普通函数调用边界上，因此无需保存调用者保存寄存器（Caller-saved Registers）或浮点/向量状态；当前 CPU 指针寄存器（RISC-V 的 `tp`、LoongArch 的 `$r2`/`tp`）始终由各物理 CPU 固定保留，不在任务切换时覆盖。
//!
//! ```text
//! +---------------------------------------------------------------+
//! | TaskContext (#[repr(C)])                                      |
//! +---------------------------------------------------------------+
//! | offset 0 * usize : ra    (返回地址 / 首次切入跳板地址)          |
//! | offset 1 * usize : sp    (内核栈顶指针)                        |
//! | offset 2 * usize : saved[0..SAVED_REGISTERS]                  |
//! |   - riscv64     (12 个): s0 - s11                             |
//! |   - loongarch64 (10 个): s0 - s8, fp ($r22 - $r31)            |
//! +---------------------------------------------------------------+
//! ```

/// RISC-V 64 架构下需要保存的被调用者保存通用寄存器数量（`s0`..=`s11` 共 12 个）。
#[cfg(target_arch = "riscv64")]
const SAVED_REGISTERS: usize = 12;
/// LoongArch 64 架构下需要保存的被调用者保存通用寄存器数量（`s0`..=`s8` 与 `fp` 共 10 个）。
#[cfg(target_arch = "loongarch64")]
const SAVED_REGISTERS: usize = 10;

/// 内核上下文切换时保存的整数寄存器现场。
///
/// 采用 `#[repr(C)]` 固定字段顺序与内存偏移，供架构相关的 `switch_context` 汇编代码按字节偏移直接寻址。
#[repr(C)]
pub struct TaskContext {
    /// 函数返回地址寄存器（`ra`）。新任务首次被调度时指向 `task_trampoline`，后续切换指向 `switch_context` 的返回点。
    pub(crate) ra: usize,
    /// 当前执行栈的栈顶指针（`sp`，字节虚拟地址）。
    pub(crate) sp: usize,
    /// 架构 ABI 规定的被调用者保存寄存器数组（Callee-saved Registers）。
    pub(crate) saved: [usize; SAVED_REGISTERS],
}

impl TaskContext {
    /// 创建一个全零的空上下文，供每个 CPU 的静态调度器上下文（`scheduler_ctx`）在编译期或启动期占位初始化。
    ///
    /// 当该 CPU 首次从 `sched::run` 切换到具体任务时，`switch_context` 汇编会将当前调度栈的真实寄存器现场写入该结构。
    pub const fn empty() -> Self {
        Self {
            ra: 0,
            sp: 0,
            saved: [0; SAVED_REGISTERS],
        }
    }

    /// 为新建任务构造初始内核执行上下文。
    ///
    /// 将栈指针 `sp` 指向新分配的独占内核栈顶 `stack_top`（必须满足架构栈对齐要求 `STACK_ALIGNMENT`），
    /// 并将返回地址 `ra` 设为 `task_trampoline`。当调度器首次通过 `switch_context` 切入该任务时，
    /// CPU 将跳转到 `task_trampoline` 开始执行任务入口闭包。
    pub(crate) fn initial(stack_top: usize) -> Self {
        assert_eq!(
            stack_top & (crate::arch::STACK_ALIGNMENT - 1),
            0,
            "task stack is not aligned"
        );
        Self {
            ra: crate::sched::task_trampoline as usize,
            sp: stack_top,
            saved: [0; SAVED_REGISTERS],
        }
    }
}
