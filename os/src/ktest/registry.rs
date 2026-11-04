//! 最终 ELF 中的只读注册槽；目录发现不依赖 MM、Task 或堆。
use super::report::{self, Json, Nullable};
use ::ktest::{CaseDesc, Context, Defaults, Isolation, SuiteDesc, SuiteSlot, TestError};
use core::mem::{align_of, size_of};

const ASCII_DEL: u8 = 0x7f;

unsafe extern "C" {
    static __ktest_suites_start: u8;
    static __ktest_suites_end: u8;
    static __rodata_start: u8;
    static __rodata_end: u8;
    static __executable_start: u8;
    static __etext: u8;
}

/// 地址区间是链接器提供的永久只读执行映射，不是物理 LMA。
#[derive(Clone, Copy)]
pub(crate) struct Registry {
    start: usize,
    count: usize,
    ro_start: usize,
    ro_end: usize,
}
impl Registry {
    /// 无分配核验注册槽、永久只读元数据和可执行回调；完整名称冲突由宿主目录集合检查。
    pub(crate) fn load() -> Result<Self, TestError> {
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
    /// 核验任意有界永久 RO 区间，包括空 slice 的 Rust 对齐悬空地址。
    fn contains(&self, pointer: usize, bytes: usize) -> bool {
        bytes == 0
            || (pointer >= self.ro_start
                && pointer
                    .checked_add(bytes)
                    .is_some_and(|end| end <= self.ro_end))
    }
    /// 核验静态 slice 的真实长度和映射。
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
    /// 字符串无目录容量上限，只要求真实静态 UTF-8 RO 存储。
    fn text(&self, value: &'static str) -> Result<(), TestError> {
        if !self.contains(value.as_ptr() as usize, value.len()) {
            return Err(TestError("metadata text outside RO mapping"));
        }
        Ok(())
    }
    /// 有效默认值必须来自正式 API 的固定资源枚举。
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
    /// 只从已校验槽区间读取一个原指针，先验证目标再建立永久借用。
    pub(crate) fn suite(&self, id: u32) -> Result<&'static SuiteDesc, TestError> {
        if id as usize >= self.count {
            return Err(TestError("unknown suite ID"));
        }
        let slot = self.start + id as usize * size_of::<SuiteSlot>();
        // SAFETY: load 核验槽区间、对齐和步长；指针位模式先按 usize 读取，尚不建立描述符引用。
        let pointer = unsafe { (slot as *const usize).read() };
        if pointer % align_of::<SuiteDesc>() != 0 || !self.contains(pointer, size_of::<SuiteDesc>())
        {
            return Err(TestError("suite descriptor outside RO mapping"));
        }
        // SAFETY: 槽由 repr(C) SuiteSlot 的真实 Rust 静态引用生成；目标已核永久 RO 映射和对齐。
        Ok(unsafe { &*(pointer as *const SuiteDesc) })
    }
    /// 流式输出最终 ELF 的完整目录，不建立中央 case 表。
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
struct Attributes<'a>(&'a Defaults);
impl core::fmt::Display for Attributes<'_> {
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
/// 输出每个实例真实有效属性与 fatal 正文合同。
fn emit_case(suite: usize, id: usize, c: &CaseDesc) {
    report::write(format_args!("{{\"type\":\"case\",\"suite\":{},\"case\":{},\"name\":{},\"parameter\":{},\"source\":{},\"line\":{}, {},\"ignore\":{},\"expected_fatal\":{},\"fatal_message\":{}}}\n", suite, id, Json(c.name), Nullable(c.parameter), Json(c.source), c.line, Attributes(&c.attributes), Nullable(c.ignore), Nullable(c.expected_fatal), Nullable(c.fatal_message)));
}
