//! 已保活正常 RAM 的原始字节访问；用户权限由 MM 逐页检查，地址使用 PLV0 缓存 DMW。

use crate::arch::loongarch64::register as reg;
use core::arch::asm;

/// 原始复制已驻留的正常 RAM 字节，不分配、不睡眠，不构造共享内容引用或并发快照。
///
/// # Safety
/// 非零长度时，两端须在整个调用期间可访问且物理范围不重叠，源字节已初始化。
/// 指针须为内核可访问的正常缓存 RAM，不能指向 MMIO；普通内核目标须独占，
/// 普通内核源的借用约束须成立。共享文件/用户 RAM 可有并发原始字节访问，
/// 但不能存在与访问冲突的 Rust 引用，已可被共享读取的目标范围必须已初始化。
/// 页拥有者须保活原页并排除设备写入及归还；本次调用不代替 DMA 收权或持久化屏障。
/// 任一用户侧范围须处于同一基本页，MM 已检查方向权限并保活原页和映射；
/// 用户写目标还须完成必要 COW，不能放宽原 MM 权限。零长不解引用。
pub unsafe fn copy_user_bytes(dst: *mut u8, src: *const u8, len: usize) {
    // 未对齐宽访问依赖 CPU 的 UAL 能力；同余对齐先复制首部再使用对齐 ld.d/st.d，
    // 异余对齐及尾部使用字节指令，不依赖可选 UAL，所有访问严格限制在本次范围内。
    // SAFETY: 调用者保证正常 RAM 的方向、容量、初始化、保活及无冲突 Rust 引用；
    // 宽指令前同时对齐两端，剩余量检查排除越界，零长退出，汇编声明真实内存副作用。
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

/// 对已保活的用户 32 位字执行一次 Acquire 读取，供 futex 比较使用。
///
/// # Safety
/// 地址须为四字节对齐、已初始化且处于同一基本页的正常缓存 RAM 内核映射；
/// MM 已检查用户可读权限并保活原页和映射直到返回。并发访问不得借助 Rust 引用，
/// 需要同步的用户写入须使用兼容的对齐原子访问及 Release 发布。
pub unsafe fn load_user_u32(src: *const u32) -> u32 {
    let value: usize;
    // SAFETY: 调用者保证对齐、初始化及页拥有；ld.wu 是单次自然对齐原子读取，
    // 后置 dbar 0x14 排序读至后续读写，默认内存副作用同时提供编译器 Acquire 边界。
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

/// 对已保活的用户 32 位字执行一次 Release 写入，供 child_tid 清零及发布使用。
///
/// # Safety
/// 地址须为四字节对齐且处于同一基本页的正常缓存 RAM 内核映射；
/// MM 已检查用户可写权限、完成私有页 COW 并保活原页和映射直到返回。
/// 并发访问不得借助 Rust 引用；需要同步的用户读取须使用兼容的 Acquire 访问。
pub unsafe fn store_user_u32(dst: *mut u32, value: u32) {
    // SAFETY: 调用者保证对齐、可写及页拥有；前置 dbar 0x12 排序此前读写至本次写，
    // st.w 是单次自然对齐原子写入，默认内存副作用同时提供编译器 Release 边界。
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
