//! 终端 ANSI 颜色格式化宏与原始字节切片视图转换辅助（Language & Formatting Helpers）。
//!
//! # 模块职责与架构定位
//! 本模块提供两个基础工具：
//! 1. [`color_text!`]：生成带 ANSI 转义序列的 `core::fmt::Arguments`，供终端与日志输出着色；
//! 2. [`Bytes`] trait：将定长值类型 `T` 直接按内存布局映射为只读或可变字节切片（`&[u8]` / `&mut [u8]`）。
//!
//! # 核心设计与内存布局契约
//! ```text
//! ANSI 颜色包装格式:
//!   "\x1b[" + <color_code> + "m" + <text> + "\x1b[0m"
//!
//! Bytes<T> 内存视图映射:
//!   &self / &mut self  ──►  *const u8 / *mut u8 (长度 = size_of::<T>())
//! ```
//!
//! # 同步与安全契约
//! - 本模块不持有任何全局状态或锁；
//! - 实现 [`Bytes<T>`] 的类型必须保证其自身大小不少于 `size_of::<T>()`，且在转换为 `&[u8]` 或 `&mut [u8]`
//!   时满足字节级全部初始化（无未初始化填充字节被读取）与任意字节写入合法性（POD / Plain Old Data 约束）。

/// 将文本表达式 `$text` 包装为带指定 ANSI 颜色码 `$color`（并在末尾重置为 `\x1b[0m`）的格式化参数。
#[macro_export]
macro_rules! color_text {
    ($text:expr, $color:expr) => {{
        format_args!("\x1b[{}m{}\x1b[0m", $color, $text)
    }};
}

/// 将对象内存直接重新解释为长度等于 `size_of::<T>()` 的原始字节切片视图。
///
/// 调用方与实现方需确保目标类型属于纯数据（POD）结构，不含未初始化的填充字节（Padding），
/// 且在调用 [`Bytes::as_bytes_mut`] 时任意字节组合均不会破坏类型 `T` 的位模式不变量。
pub trait Bytes<T> {
    /// 返回当前对象的只读原始字节切片视图（长度固定为 `core::mem::size_of::<T>()` 字节）。
    fn as_bytes(&self) -> &[u8] {
        let size = core::mem::size_of::<T>();
        // SAFETY:
        // 1. `self` 是有效引用，其起始地址非空且对 `u8`（对齐要求为 1）天然满足对齐；
        // 2. 切片生命周期绑定在 `&self` 上，且实现方约定 `Self` 具有 `T` 的完整已初始化内存布局。
        unsafe {
            core::slice::from_raw_parts(self as *const _ as *const T as usize as *const u8, size)
        }
    }

    /// 返回当前对象的可变原始字节切片视图（长度固定为 `core::mem::size_of::<T>()` 字节）。
    fn as_bytes_mut(&mut self) -> &mut [u8] {
        let size = core::mem::size_of::<T>();
        // SAFETY:
        // 1. `self` 是独占可变引用，其起始地址非空且满足 `u8` 的 1 字节对齐要求；
        // 2. 返回的 `&mut [u8]` 生命周期受限于 `&mut self`，期间不存在其他别名引用，且实现方约定 `Self` 支持按字节覆写。
        unsafe {
            core::slice::from_raw_parts_mut(self as *mut _ as *mut T as usize as *mut u8, size)
        }
    }
}
