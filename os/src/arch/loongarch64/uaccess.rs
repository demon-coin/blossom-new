//! LoongArch64 用户态/共享物理页的原始内存复制与 32 位原子访存原语。
//!
//! # 模块职责与架构定位
//! 本模块位于 `arch::loongarch64` 硬件抽象层，提供对已驻留物理内存页（通过内核 `PLV0` 可缓存直接映射窗口 `DMW1` 访问）的底层汇编访存函数。
//! - **上游调用者**：
//!   - `mm::uaccess`（`copyin`、`copyout`、`load_user_u32`、`store_user_u32`）：在页表锁/页面引用保护下逐页读写用户空间内存；
//!   - `mm::folio` 与 `mm::space`：对页缓存（Page Cache / Folio）和写时复制（COW）物理页执行原始字节复制或清零；
//!   - `task::user`（`AleMemory`）：在非对齐访存模拟（ALE）期间逐字节读写用户内存。
//! - **下游依赖**：依赖 `register` 模块定义的 `dbar` 内存屏障提示字（`DBAR_ACQUIRE`、`DBAR_RELEASE`）。
//!
//! # 核心设计与内存模型契约
//! 1. **为什么使用内联汇编而非 `core::ptr::copy_nonoverlapping` 或 Rust 引用**：
//!    - 用户态线程或共享文件映射（`MAP_SHARED`）可能在其他 CPU 上并发读写同一物理页。
//!      如果将共享物理页构造为 Rust 的 `&[u8]` 或 `&mut [u8]`，一旦发生并发修改即构成 Rust 抽象机层面的数据竞争未定义行为（UB）。
//!    - 通过内联汇编直接发射机器指令（`ld.bu`/`st.b`/`ld.d`/`st.d`），访问被严格建模为寄存器与外部内存之间的挥发副作用，不会在 Rust 侧产生冲突引用。
//! 2. **兼顾性能与严格对齐的 `copy_user_bytes` 状态机**：
//!    LoongArch64 的非对齐宽访存能力（UAL）属于可选硬件特性。为保证在未开启或不支持硬件非对齐访问的处理器上也不触发地址非对齐异常（ALE），
//!    `copy_user_bytes` 采用如下分段复制策略：
//!    ```text
//!    调用入口 (dst, src, len)
//!       |
//!       +-- len == 0 --------------------------------------------------> [7: 直接返回]
//!       |
//!       +-- ((dst ^ src) & 7) != 0 (源与目标模 8 不同余，无法同时 8 字节对齐)
//!       |      |
//!       |      +-------------------------------------------------------> [5/6: 全程 ld.bu / st.b 逐字节复制]
//!       |
//!       +-- ((dst ^ src) & 7) == 0 (源与目标模 8 同余)
//!              |
//!              v
//!        [2: 首部对齐循环] 当 (dst & 7) != 0 时，用 ld.bu/st.b 逐字节复制 1..7 字节，直至 dst 与 src 同时按 8 字节对齐
//!              |
//!              v
//!        [4: 主体 8 字节快路径] 当 len >= 8 时，使用自然对齐的 ld.d / st.d 每次复制 8 字节
//!              |
//!              v
//!        [5/6: 尾部剩余字节] 对不足 8 字节的尾部（0..7 字节），用 ld.bu/st.b 逐字节复制完毕 -> [7: 返回]
//!    ```
//! 3. **32 位同步原语（Acquire Load / Release Store）**：
//!    - `load_user_u32`：使用自然对齐的 `ld.wu` 读取 32 位字，后接 `dbar 0x14`（`DBAR_ACQUIRE`，排序后续读写），供 `futex` 状态检查使用。
//!    - `store_user_u32`：前置 `dbar 0x12`（`DBAR_RELEASE`，排序此前读写），后接自然对齐的 `st.w` 写入 32 位字，供线程退出 `clear_child_tid` 等场景使用。
//!
//! # 锁获取顺序与并发契约
//! - 本模块所有函数均为无锁、不分配内存、不可睡眠的纯指令序列，可在持有自旋锁（如页面访问回调）的上下文中调用。
//! - 调用方（如 `Mm::with_page`）必须在调用期间保活底层物理页（`Folio` / `OwnedPages`）及权限映射，确保目标物理页不会被并发释放回伙伴分配器。

use crate::arch::loongarch64::register as reg;
use core::arch::asm;

/// 在内核可缓存直接映射窗口（`DMW1`）内将 `src` 指向的 `len` 个字节复制到 `dst`。
///
/// 本函数不分配内存、不睡眠，且不在源或目标内存上构造 Rust 引用，允许与用户态对同一共享物理页的并发访问共存。
///
/// # Safety
/// - 当 `len == 0` 时，函数直接返回，不解引用 `dst` 或 `src`。
/// - 当 `len > 0` 时：
///   1. `[src, src + len)` 与 `[dst, dst + len)` 必须完全位于内核可访问的普通可缓存物理内存（`DMW1` 窗口）内，严禁指向强序设备寄存器（MMIO），且两段物理内存范围不得重叠。
///   2. 源范围 `[src, src + len)` 的所有字节必须已完成初始化。
///   3. 调用方必须在整个调用期间保活两端对应的底层物理页（防止物理页被并发释放或交由 DMA 设备改写）。
///   4. 若任一端为普通内核私有内存（如内核栈或堆缓冲区），则内核目标缓冲区必须由当前执行流独占可写，内核源缓冲区在复制期间不得被并发修改；若任一端为用户态映射页或共享文件缓存页，则不得在该物理页上同时持有冲突的 Rust 引用（`&[u8]` / `&mut [u8]`）。
///   5. 若操作涉及用户地址空间，调用方必须已按访问方向完成页表权限检查（目标为用户页时还须已完成写时复制 COW 拆分），且单次调用跨越的用户侧区间必须位于同一个基本物理页内。
pub unsafe fn copy_user_bytes(dst: *mut u8, src: *const u8, len: usize) {
    // Step 1: 检查源与目标地址的模 8 同余性，同余时先对齐首部再执行 8 字节对齐读写（ld.d/st.d），异余或尾部退化为单字节读写（ld.bu/st.b）。
    // SAFETY:
    // 1. 调用方契约保证了当 `len > 0` 时，`[src, src + len)` 与 `[dst, dst + len)` 均为已保活、不重叠的 `DMW1` 普通内存，且源字节已初始化、无冲突 Rust 引用。
    // 2. 汇编入口通过 `beqz {len}, 7f` 排除了零长度解引用；`4:` 标签的 8 字节宽指令（`ld.d`/`st.d`）仅在 `(dst ^ src) & 7 == 0` 且 `dst & 7 == 0` 以及 `len >= 8` 时执行，严格满足 8 字节自然对齐与边界不越界。
    // 3. `asm!` 未声明 `nomem` 或 `readonly`，编译器将其视为具有完整读写内存副作用的屏障。
    unsafe {
        asm!(
            "beqz {len}, 7f",
            "xor {tmp}, {dst}, {src}",
            "andi {tmp}, {tmp}, 7",
            "bnez {tmp}, 6f",
            "2:",
            "andi {tmp}, {dst}, 7",
            "beqz {tmp}, 4f",
            "ld.bu {tmp}, {src}, 0",
            "st.b {tmp}, {dst}, 0",
            "addi.d {src}, {src}, 1",
            "addi.d {dst}, {dst}, 1",
            "addi.d {len}, {len}, -1",
            "beqz {len}, 7f",
            "b 2b",
            "4:",
            "sltui {tmp}, {len}, 8",
            "bnez {tmp}, 6f",
            "ld.d {tmp}, {src}, 0",
            "st.d {tmp}, {dst}, 0",
            "addi.d {src}, {src}, 8",
            "addi.d {dst}, {dst}, 8",
            "addi.d {len}, {len}, -8",
            "b 4b",
            "6:",
            "beqz {len}, 7f",
            "5:",
            "ld.bu {tmp}, {src}, 0",
            "st.b {tmp}, {dst}, 0",
            "addi.d {src}, {src}, 1",
            "addi.d {dst}, {dst}, 1",
            "addi.d {len}, {len}, -1",
            "bnez {len}, 5b",
            "7:",
            dst = inout(reg) dst => _,
            src = inout(reg) src => _,
            len = inout(reg) len => _,
            tmp = out(reg) _,
            options(nostack),
        );
    }
}

/// 对已保活的用户态 32 位对齐字执行一次带 Acquire 语义的原子读取（供 `futex` 等待条件检查等场景使用）。
///
/// # Safety
/// - `src` 必须是 4 字节自然对齐的内核 `DMW1` 虚拟地址，指向已完成初始化且位于同一基本物理页内的普通可缓存 RAM。
/// - 调用方必须已验证用户态对该地址具备可读权限，并在整个函数调用期间保活底层物理页。
/// - 调用期间不得在该内存位置上持有冲突的 Rust 可变引用。
pub unsafe fn load_user_u32(src: *const u32) -> u32 {
    let value: usize;
    // SAFETY:
    // 1. 调用方契约保证 `src` 按 4 字节自然对齐、指向已保活且已初始化的 `DMW1` 普通物理内存。
    // 2. 在 LoongArch64 下，对 4 字节自然对齐地址执行的 `ld.wu` 是单次 32 位原子读取；紧随其后的 `dbar 0x14`（`DBAR_ACQUIRE`）在硬件层面阻止本次读取与后续读写重排，且 `asm!` 默认内存副作用在编译器层面构成 Acquire 屏障。
    unsafe {
        asm!(
            "ld.wu {value}, {src}, 0",
            "dbar {hint}",
            hint = const reg::DBAR_ACQUIRE,
            src = in(reg) src,
            value = out(reg) value,
            options(nostack),
        );
    }
    value as u32
}

/// 对已保活的用户态 32 位对齐字执行一次带 Release 语义的原子写入（供线程退出时清零 `clear_child_tid` 等场景使用）。
///
/// # Safety
/// - `dst` 必须是 4 字节自然对齐的内核 `DMW1` 虚拟地址，指向位于同一基本物理页内的普通可缓存 RAM。
/// - 调用方必须已验证用户态对该地址具备可写权限、完成了必要的私有页写时复制（COW），并在整个函数调用期间保活底层物理页。
/// - 调用期间不得在该内存位置上持有任何冲突的 Rust 引用。
pub unsafe fn store_user_u32(dst: *mut u32, value: u32) {
    // SAFETY:
    // 1. 调用方契约保证 `dst` 按 4 字节自然对齐、指向已保活且可写的 `DMW1` 普通物理内存。
    // 2. 前置 `dbar 0x12`（`DBAR_RELEASE`）在硬件层面确保此前的读写操作先于本次写入提交，自然对齐的 `st.w` 执行单次 32 位原子写入，且 `asm!` 默认内存副作用在编译器层面构成 Release 屏障。
    unsafe {
        asm!(
            "dbar {hint}",
            "st.w {value}, {dst}, 0",
            hint = const reg::DBAR_RELEASE,
            dst = in(reg) dst,
            value = in(reg) value as usize,
            options(nostack),
        );
    }
}
