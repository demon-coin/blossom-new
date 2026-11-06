//! 内核测试套件只读注册表（Test Suite Registry）的加载、校验与目录导出模块。
//!
//! ## 模块职责与架构定位
//! 本模块负责在内核启动早期（尚未初始化堆分配器、内存管理子系统 `mm` 或任务调度器 `task` 之前），
//! 直接扫描并校验链接脚本收集在只读数据段（`.rodata`）中的测试套件注册槽（`SuiteSlot`），
//! 并支持将全部测试套件与测试用例元数据以 JSON Lines 格式流式输出给宿主机测试运行器。
//! - **上游调用者**：`ktest::runner` 在解析宿主机命令或执行测试套件时调用。
//! - **下游依赖**：`ktest::report`（串口同步输出）、链接器导出的段边界符号（`__ktest_suites_start` 等）及 `arch::test_shutdown`。
//!
//! ## 核心设计与只读段内存布局
//! 过程宏 `#[ktest::suite]` 会为每个测试套件生成一个 `SuiteDesc` 静态常量，并将指向它的 `SuiteSlot` 放入专用链接输入段，
//! 最终由链接脚本汇总在 `.rodata` 段内的 `[__ktest_suites_start, __ktest_suites_end)` 连续区间中：
//!
//! ```text
//! 虚拟地址空间 (内核永久映射区间)
//! +---------------------------------------------------------------+ __executable_start
//! | .text (代码段)                                                |
//! |   suite.run 函数指针必须位于 [__executable_start, __etext)    |
//! +---------------------------------------------------------------+ __etext
//! | .rodata (只读数据段)                                          | __rodata_start
//! |   +-------------------------------------------------------+   | __ktest_suites_start
//! |   | SuiteSlot[0] -> 指向 .rodata 内的 SuiteDesc           |   |
//! |   | SuiteSlot[1] -> 指向 .rodata 内的 SuiteDesc           |   |
//! |   | ...                                                   |   |
//! |   +-------------------------------------------------------+   | __ktest_suites_end
//! |   SuiteDesc / CaseDesc / 静态字符串 / 标签切片均位于 .rodata  |
//! +---------------------------------------------------------------+ __rodata_end
//! ```
//!
//! `Registry::load` 在建立任何 Rust 引用前，会先以整数地址形式校验所有注册槽、描述符、切片及字符串是否完整落在 `[__rodata_start, __rodata_end)` 内且满足对齐要求，并检查套件回调函数指针是否位于可执行代码段内。
//!
//! ## 锁获取顺序与并发契约
//! - `Registry` 本身是只读元数据的轻量级句柄（`Copy`），不包含任何锁或可变状态，可在任意上下文中无锁读取。
//! - `Registry::catalog` 调用 `report::write` 输出目录行时会短暂获取 `report::OUTPUT` 串口输出自旋锁。

use super::report::{self, Json, Nullable};
use ::ktest::{CaseDesc, Context, Defaults, Isolation, SuiteDesc, SuiteSlot, TestError};
use core::mem::{align_of, size_of};

/// ASCII 删除控制字符（`0x7f`），用于检查并拒绝包含控制字符的测试名称。
const ASCII_DEL: u8 = 0x7f;

unsafe extern "C" {
    /// 测试套件注册槽数组的起始符号（位于 `.rodata` 段内，由链接脚本定义）。
    static __ktest_suites_start: u8;
    /// 测试套件注册槽数组的结束符号（位于 `.rodata` 段内，由链接脚本定义）。
    static __ktest_suites_end: u8;
    /// 内核只读数据段（`.rodata`）的起始虚拟地址符号。
    static __rodata_start: u8;
    /// 内核只读数据段（`.rodata`）的结束虚拟地址符号。
    static __rodata_end: u8;
    /// 内核可执行代码段（`.text`）的起始虚拟地址符号。
    static __executable_start: u8;
    /// 内核可执行代码段（`.text`）的结束虚拟地址符号。
    static __etext: u8;
}

/// 已完成边界与合法性校验的内核测试套件只读注册表句柄。
///
/// 结构体中记录的所有地址均为内核虚拟地址空间中的永久只读映射地址（VMA），而非物理加载地址（LMA）。
#[derive(Clone, Copy)]
pub(crate) struct Registry {
    /// 测试套件注册槽数组（`__ktest_suites_start`）的起始虚拟地址（单位：字节地址）。
    start: usize,
    /// 注册表中包含的测试套件（`SuiteSlot`）总数量。
    count: usize,
    /// 内核只读数据段（`__rodata_start`）的起始虚拟地址（单位：字节地址）。
    ro_start: usize,
    /// 内核只读数据段（`__rodata_end`）的结束虚拟地址（单位：字节地址）。
    ro_end: usize,
}

impl Registry {
    /// 在无堆分配的前提下加载并全面校验 ELF 只读段中的测试套件注册表。
    ///
    /// 调用上下文：内核启动早期（`ktest::runner::early`）调用。
    /// 校验内容包括：注册槽数组边界与对齐、每个套件描述符与用例切片的只读段归属、套件回调函数的代码段归属、
    /// 用例属性相对于套件默认值的单调收紧关系，以及致命故障期望（`expected_fatal`）的配置合法性。
    /// 跨套件的完整测试名称查重由宿主机在收到目录后统一完成。
    pub(crate) fn load() -> Result<Self, TestError> {
        // Step 1: 读取链接脚本符号地址，计算注册槽数量并校验注册槽数组自身的段边界与对齐。
        let start = core::ptr::addr_of!(__ktest_suites_start) as usize;
        let end = core::ptr::addr_of!(__ktest_suites_end) as usize;
        let value = Self {
            start,
            count: end
                .checked_sub(start)
                .ok_or(TestError("registry reversed"))?
                / size_of::<SuiteSlot>(),
            ro_start: core::ptr::addr_of!(__rodata_start) as usize,
            ro_end: core::ptr::addr_of!(__rodata_end) as usize,
        };

        if value.ro_end <= value.ro_start
            || start < value.ro_start
            || value.ro_end < end
            || start % align_of::<SuiteSlot>() != 0
            || end % align_of::<SuiteSlot>() != 0
            || (end - start) % size_of::<SuiteSlot>() != 0
            || !value.contains(start, end - start)
            || (u32::MAX as usize) < value.count
        {
            return Err(TestError(
                "registry boundary/alignment outside permanent RO mapping",
            ));
        }

        // Step 2: 逐个遍历测试套件，校验套件描述符、默认属性、用例切片及套件入口函数地址。
        for index in 0..value.count {
            let suite = value.suite(index as u32)?;
            value.text(suite.name)?;
            if suite.name.is_empty() {
                return Err(TestError("empty suite name"));
            }
            value.defaults(&suite.defaults)?;
            value.slice(suite.cases)?;
            if suite.cases.is_empty() || (u32::MAX as usize) < suite.cases.len() {
                return Err(TestError("invalid suite case count"));
            }

            let run = suite.run as usize;
            if run < core::ptr::addr_of!(__executable_start) as usize
                || core::ptr::addr_of!(__etext) as usize <= run
            {
                return Err(TestError("suite callback outside executable mapping"));
            }

            // Step 3: 逐个校验套件内的测试用例元数据、命名规范、属性约束及 expected_fatal 契约。
            for case in suite.cases {
                value.text(case.name)?;
                value.text(case.source)?;
                value.defaults(&case.attributes)?;
                if case.name.is_empty() {
                    return Err(TestError("empty case name"));
                }
                for text in [
                    case.parameter,
                    case.ignore,
                    case.should_panic,
                    case.expected_fatal,
                    case.fatal_message,
                ]
                .iter()
                .flatten()
                {
                    value.text(text)?;
                }
                if case.attributes.context != suite.defaults.context
                    || case.attributes.isolation < suite.defaults.isolation
                    || case.attributes.min_cpus < suite.defaults.min_cpus
                {
                    return Err(TestError("case weakens suite defaults"));
                }
                if case.name.bytes().any(|b| b < b' ' || b == ASCII_DEL)
                    || suite.name.bytes().any(|b| b < b' ' || b == ASCII_DEL)
                {
                    return Err(TestError("control character in test name"));
                }

                if let Some(kind) = case.expected_fatal {
                    if !matches!(
                        kind,
                        "illegal"
                            | "refill"
                            | "irq-schedule"
                            | "preempt-schedule"
                            | "oom"
                            | "timekeeper"
                            | "panic"
                            | "double-panic"
                            | "abi"
                    ) || case.attributes.isolation != Isolation::Case
                        || case.should_panic.is_some()
                        || (kind == "panic" && case.fatal_message.is_none_or(str::is_empty))
                    {
                        return Err(TestError("invalid fatal contract"));
                    }
                } else if case.fatal_message.is_some() {
                    return Err(TestError("fatal message without fatal"));
                }
            }
        }
        Ok(value)
    }

    /// 检查起始地址为 `pointer`、长度为 `bytes` 字节的内存区间是否完整位于内核永久只读数据段（`.rodata`）内。
    ///
    /// 当 `bytes == 0` 时直接返回 `true`，以兼容 Rust 空切片使用的非空对齐悬空指针（`NonNull::dangling()`）。
    fn contains(&self, pointer: usize, bytes: usize) -> bool {
        bytes == 0
            || (pointer >= self.ro_start
                && pointer
                    .checked_add(bytes)
                    .is_some_and(|end| end <= self.ro_end))
    }

    /// 校验静态切片 `values` 的起始地址对齐以及整个字节区间是否位于 `.rodata` 只读段内。
    fn slice<T>(&self, values: &'static [T]) -> Result<(), TestError> {
        if (values.as_ptr() as usize) % align_of::<T>() != 0
            || !values
                .len()
                .checked_mul(size_of::<T>())
                .is_some_and(|n| self.contains(values.as_ptr() as usize, n))
        {
            return Err(TestError("metadata slice outside RO mapping"));
        }
        Ok(())
    }

    /// 校验静态字符串切片 `value` 的底层字节区间是否位于 `.rodata` 只读段内。
    fn text(&self, value: &'static str) -> Result<(), TestError> {
        if !self.contains(value.as_ptr() as usize, value.len()) {
            return Err(TestError("metadata text outside RO mapping"));
        }
        Ok(())
    }

    /// 校验测试套件或用例的默认属性配置（`Defaults`）：检查标签与资源切片的只读段归属，并确保最小 CPU 数与超时时间均非零。
    fn defaults(&self, value: &Defaults) -> Result<(), TestError> {
        self.slice(value.tags)?;
        self.slice(value.resources)?;
        for tag in value.tags {
            self.text(tag)?;
        }
        if value.min_cpus == 0 || value.timeout_ms == 0 {
            return Err(TestError("zero CPU or timeout metadata"));
        }
        Ok(())
    }

    /// 根据套件序号 `id` 读取并校验对应的 `SuiteDesc` 静态引用。
    ///
    /// 先从已校验的注册槽数组中按 `usize` 读取目标指针地址，验证目标地址对齐且完整位于 `.rodata` 段内后，再转换为 `'static` 引用。
    pub(crate) fn suite(&self, id: u32) -> Result<&'static SuiteDesc, TestError> {
        if id as usize >= self.count {
            return Err(TestError("unknown suite ID"));
        }

        let slot = self.start + id as usize * size_of::<SuiteSlot>();
        // SAFETY:
        // 1. `Registry::load` 已校验 `[self.start, self.start + self.count * size_of::<SuiteSlot>())` 完整位于 `.rodata` 段内且按 `SuiteSlot` 对齐。
        // 2. 前置条件 `id as usize < self.count` 保证了 `slot` 地址有效且不越界；此处仅按 `usize` 读取指针数值，尚未解引用目标描述符。
        let pointer = unsafe { (slot as *const usize).read() };
        if pointer % align_of::<SuiteDesc>() != 0 || !self.contains(pointer, size_of::<SuiteDesc>())
        {
            return Err(TestError("suite descriptor outside RO mapping"));
        }
        // SAFETY:
        // 1. 每个注册槽均由编译期 `#[repr(C)]` 的 `SuiteSlot` 静态常量生成，内容为指向静态 `SuiteDesc` 的有效指针。
        // 2. 上方条件已检查 `pointer` 满足 `SuiteDesc` 对齐，且整个 `SuiteDesc` 结构体区间完整位于永久只读的 `.rodata` 段内。
        Ok(unsafe { &*(pointer as *const SuiteDesc) })
    }

    /// 将当前内核镜像中的全部测试套件与测试用例元数据以 JSON Lines 格式流式输出到控制串口，随后直接关机退出。
    ///
    /// 调用上下文：当宿主机在握手阶段发送 `"catalog"` 命令时由 `ktest::runner::early` 调用。
    /// 全程逐条流式输出，不在内存中分配或构建集中式的用例列表。
    pub(crate) fn catalog(&self, arch: &str, cpus: u32, mode: &str, panic: &str) -> ! {
        report::write(format_args!("{{\"type\":\"catalog\",\"version\":1,\"arch\":\"{}\",\"cpus\":{},\"mode\":\"{}\",\"panic\":\"{}\"}}\n", arch, cpus, mode, panic));
        let mut cases = 0u64;
        for id in 0..self.count {
            let suite = self
                .suite(id as u32)
                .unwrap_or_else(|error| super::context::fatal(error.0));
            report::write(format_args!(
                "{{\"type\":\"suite\",\"suite\":{},\"name\":{},{} }}\n",
                id,
                Json(suite.name),
                Attributes(&suite.defaults)
            ));
            for (cid, case) in suite.cases.iter().enumerate() {
                emit_case(id, cid, case);
                cases += 1;
            }
        }
        report::write(format_args!(
            "{{\"type\":\"catalog_end\",\"suites\":{},\"cases\":{}}}\n",
            self.count, cases
        ));
        crate::arch::test_shutdown()
    }
}

/// 用于将 `Defaults` 属性（元组字段 `0`）格式化为 JSON 键值对片段的辅助包装结构体。
struct Attributes<'a>(&'a Defaults);

impl core::fmt::Display for Attributes<'_> {
    /// 将 `Defaults` 中的执行上下文、隔离级别、标签列表、最小 CPU 数、超时时间与资源列表输出为 JSON 字段序列。
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        use core::fmt::Write;
        let d = self.0;
        write!(
            f,
            "\"context\":\"{}\",\"isolation\":\"{}\",\"tags\":[",
            if d.context == Context::Boot {
                "Boot"
            } else {
                "Thread"
            },
            if d.isolation == Isolation::Suite {
                "Suite"
            } else {
                "Case"
            }
        )?;
        for (i, tag) in d.tags.iter().enumerate() {
            if i != 0 {
                f.write_char(',')?;
            }
            write!(f, "{}", Json(tag))?;
        }
        write!(
            f,
            "],\"min_cpus\":{},\"timeout_ms\":{},\"resources\":[",
            d.min_cpus, d.timeout_ms
        )?;
        for (i, r) in d.resources.iter().enumerate() {
            if i != 0 {
                f.write_char(',')?;
            }
            write!(f, "{}", Json(r.as_str()))?;
        }
        f.write_char(']')
    }
}

/// 向控制串口输出单个测试用例的完整 JSON 目录记录行（包含名称、参数、源码位置、属性及故障期望）。
fn emit_case(suite: usize, id: usize, c: &CaseDesc) {
    report::write(format_args!("{{\"type\":\"case\",\"suite\":{},\"case\":{},\"name\":{},\"parameter\":{},\"source\":{},\"line\":{}, {},\"ignore\":{},\"expected_fatal\":{},\"fatal_message\":{}}}\n", suite, id, Json(c.name), Nullable(c.parameter), Json(c.source), c.line, Attributes(&c.attributes), Nullable(c.ignore), Nullable(c.expected_fatal), Nullable(c.fatal_message)));
}
