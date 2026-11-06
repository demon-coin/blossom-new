//! `#[ktest::suite]` 过程宏实现。
//!
//! # 模块职责与架构定位
//! 本模块在编译期解析内联测试模块（`mod`）上的套件属性与内部测试函数，校验函数签名与属性合法性，
//! 并将其展开为 `ktest` 运行时所需的静态元数据表（`SuiteDesc`、`CaseDesc`）、参数化实例包装函数以及 `.ktest.suites` 链接段注册槽。
//! - **上游调用者**：内核与宿主测试代码中带有 `#[ktest::suite(...)]` 属性的内联模块。
//! - **下游依赖方**：展开后的代码直接调用 `ktest` crate 中的 `run_suite`、`run_case`、`IntoTestResult` 等运行时接口，并由链接器收集 `.ktest.suites` 段中的 `SuiteSlot`。
//!
//! # 核心设计与代码展开布局
//!
//! ## 1. 源码输入到生成代码的映射流程
//! ```text
//! #[ktest::suite(fixture = Dev, suite_resources = Base, ...)]
//! mod my_suite {
//!     #[test]
//!     #[case("a", 1)]
//!     #[case("b", 2)]
//!     fn check(ctx: &mut TestContext, dev: &mut Dev, val: u32) -> TestResult { ... }
//! }
//!                           |
//!                           v  (编译期过程宏展开)
//! mod my_suite {
//!     // 1. 保留原模块中的辅助类型与剥离了专用属性后的原始测试函数 `fn check(...)`
//!     // 2. 为每个静态参数实例生成独立包装函数：
//!     //    fn __ktest_case_0(ctx, res) -> CaseOutcome { run_case::<Dev, Base>(...) }
//!     //    fn __ktest_case_1(ctx, res) -> CaseOutcome { run_case::<Dev, Base>(...) }
//!     // 3. 生成静态用例描述符数组：
//!     //    pub static __KTEST_CASES: &[CaseDesc] = &[CaseDesc { ... }, CaseDesc { ... }];
//!     // 4. 生成索引分派器与套件运行入口：
//!     //    fn __ktest_dispatch(ctx, res, idx) -> CaseOutcome
//!     //    fn __ktest_run(ctx, runner) -> SuiteOutcome
//!     // 5. 生成套件描述符与 `.ktest.suites` 链接段槽位及归档导出锚点：
//!     //    pub static __KTEST_SUITE: SuiteDesc = ...;
//!     //    #[link_section = ".ktest.suites"] pub static __KTEST_SLOT: SuiteSlot = ...;
//!     //    pub fn __ktest_anchor() -> &'static SuiteSlot { &__KTEST_SLOT }
//! }
//! ```
//!
//! ## 2. 套件与用例属性合并规则
//! - `context`：仅允许在套件级指定（默认 `Thread`），防止同一套件内的用例混用不同执行上下文。
//! - `isolation`：默认 `Suite`；若套件或用例任一方指定为 `Case`，或者声明了 `expected_fatal`，则自动提升为 `Case` 级隔离。
//! - `min_cpus`：取套件与用例声明值的较大者（默认 `1`）。
//! - `timeout_ms`：优先采用用例级覆盖值，未指定则继承套件级配置（默认 `10_000` 毫秒）。
//! - `tags` / `resources`：取套件级列表与用例级列表的去重并集。

use std::collections::BTreeSet;

use proc_macro::TokenStream;
use proc_macro2::{Span, TokenStream as Tokens};
use quote::{format_ident, quote, ToTokens};
use syn::{
    parse::Parser, punctuated::Punctuated, spanned::Spanned, visit::Visit, Expr, FnArg, Item,
    ItemFn, ItemMod, Lit, LitStr, Meta, ReturnType, Token, Type,
};

/// 将带有 `#[ktest::suite]` 属性的内联模块展开为静态测试套件描述符、用例分派表及生命周期包装函数。
#[proc_macro_attribute]
pub fn suite(attributes: TokenStream, input: TokenStream) -> TokenStream {
    match expand(attributes.into(), input.into()) {
        Ok(tokens) => tokens.into(),
        Err(error) => error.into_compile_error().into(),
    }
}

/// 解析后的套件级或用例级 `#[ktest(...)]` 配置选项集合。
#[derive(Default, Clone)]
struct Options {
    /// 用例级独立夹具类型（仅允许在套件属性中通过 `fixture = Type` 指定）。
    fixture: Option<Type>,
    /// 套件级共享资源类型（仅允许在套件属性中通过 `suite_resources = Type` 指定）。
    base: Option<Type>,
    /// 附加在 `module_path!()` 之后的自定义套件子名称。
    name: Option<LitStr>,
    /// 套件执行上下文名称（`"Boot"` 或 `"Thread"`，仅套件级有效）。
    context: Option<LitStr>,
    /// 隔离级别名称（`"Suite"` 或 `"Case"`）。
    isolation: Option<LitStr>,
    /// 静态分类标签字符串字面量列表。
    tags: Vec<LitStr>,
    /// 依赖的平台资源标识符字符串字面量列表。
    resources: Vec<LitStr>,
    /// 要求的最少在线 CPU 数量。
    min_cpus: Option<u32>,
    /// 超时时间配额（单位：毫秒）。
    timeout_ms: Option<u64>,
    /// 跳过该套件或用例的原因字符串。
    ignore: Option<LitStr>,
    /// 预期触发的内核致命异常类型名称。
    fatal: Option<LitStr>,
    /// 预期致命异常发生时要求匹配的诊断消息子串。
    fatal_message: Option<LitStr>,
    /// 已出现过的属性键名集合，用于检测并拒绝重复属性。
    seen: BTreeSet<String>,
}

/// `Options` 的编译期属性解析实现。
impl Options {
    /// 解析 `#[ktest::suite(...)]`（当 `suite == true` 时）或 `#[ktest(...)]`（当 `suite == false` 时）中的键值对与列表属性。
    fn parse(&mut self, tokens: Tokens, suite: bool) -> syn::Result<()> {
        let entries = Punctuated::<Meta, Token![,]>::parse_terminated.parse2(tokens)?;
        for entry in entries {
            // Step 1: 提取属性标识符并检查同一作用域内是否重复声明了相同属性。
            let key = entry
                .path()
                .get_ident()
                .ok_or_else(|| syn::Error::new(entry.span(), "expected a simple attribute name"))?
                .to_string();
            if !self.seen.insert(key.clone()) {
                return Err(syn::Error::new(entry.span(), "duplicate ktest attribute"));
            }
            // Step 2: 按属性名与语法形态分发校验，分别解析标签/资源列表、类型、字符串枚举及正整数配额。
            match (key.as_str(), &entry) {
                ("tags" | "resources", Meta::List(list)) => {
                    let values =
                        list.parse_args_with(Punctuated::<LitStr, Token![,]>::parse_terminated)?;
                    let mut names = BTreeSet::new();
                    for value in values {
                        valid_label(&value)?;
                        if !names.insert(value.value()) {
                            return Err(syn::Error::new(value.span(), "duplicate tag or resource"));
                        }
                        if key == "resources"
                            && !matches!(
                                value.value().as_str(),
                                "uart"
                                    | "scratch"
                                    | "root"
                                    | "extra"
                                    | "ap-freeze"
                                    | "virtio-access-platform"
                                    | "net-user"
                            )
                        {
                            return Err(syn::Error::new(
                                value.span(),
                                "resource must be uart, scratch, root, extra, ap-freeze, virtio-access-platform or net-user",
                            ));
                        }
                        if key == "tags" {
                            self.tags.push(value);
                        } else {
                            self.resources.push(value);
                        }
                    }
                }
                ("fixture" | "suite_resources", Meta::NameValue(value)) if suite => {
                    let ty: Type = syn::parse2(value.value.to_token_stream())?;
                    if key == "fixture" {
                        self.fixture = Some(ty);
                    } else {
                        self.base = Some(ty);
                    }
                }
                ("name" | "context", Meta::NameValue(value)) if suite => {
                    let text = literal_string(&value.value)?;
                    if key == "name" {
                        valid_label(&text)?;
                        self.name = Some(text);
                    } else {
                        if !matches!(text.value().as_str(), "Boot" | "Thread" | "boot" | "thread") {
                            return Err(syn::Error::new(
                                text.span(),
                                "context must be Boot or Thread",
                            ));
                        }
                        self.context = Some(text);
                    }
                }
                ("isolation", Meta::NameValue(value)) => {
                    let text = literal_string(&value.value)?;
                    if !matches!(text.value().as_str(), "Suite" | "Case" | "suite" | "case") {
                        return Err(syn::Error::new(
                            text.span(),
                            "isolation must be suite or case",
                        ));
                    }
                    self.isolation = Some(text);
                }
                ("ignore" | "expected_fatal" | "fatal_message", Meta::NameValue(value)) => {
                    let text = literal_string(&value.value)?;
                    valid_message(&text)?;
                    match key.as_str() {
                        "ignore" => self.ignore = Some(text),
                        "fatal_message" => self.fatal_message = Some(text),
                        _ => {
                            if !matches!(
                                text.value().as_str(),
                                "illegal"
                                    | "refill"
                                    | "irq-schedule"
                                    | "preempt-schedule"
                                    | "oom"
                                    | "timekeeper"
                                    | "panic"
                                    | "double-panic"
                                    | "abi"
                            ) {
                                return Err(syn::Error::new(
                                    text.span(),
                                    "unknown expected_fatal type",
                                ));
                            }
                            self.fatal = Some(text);
                        }
                    }
                }
                ("min_cpus" | "timeout_ms" | "timeout", Meta::NameValue(value)) => {
                    let Expr::Lit(literal) = &value.value else {
                        return Err(syn::Error::new(
                            value.span(),
                            "expected a positive integer literal",
                        ));
                    };
                    let Lit::Int(integer) = &literal.lit else {
                        return Err(syn::Error::new(
                            value.span(),
                            "expected a positive integer literal",
                        ));
                    };
                    let number: u64 = integer.base10_parse()?;
                    if number == 0 {
                        return Err(syn::Error::new(
                            integer.span(),
                            "budget and CPU count must be positive",
                        ));
                    }
                    if key == "min_cpus" {
                        self.min_cpus = Some(u32::try_from(number).map_err(|_| {
                            syn::Error::new(integer.span(), "CPU count exceeds u32")
                        })?);
                    } else {
                        if self.timeout_ms.is_some() {
                            return Err(syn::Error::new(
                                integer.span(),
                                "timeout and timeout_ms are the same attribute",
                            ));
                        }
                        self.timeout_ms = Some(number);
                    }
                }
                _ => {
                    return Err(syn::Error::new(
                        entry.span(),
                        "unsupported ktest attribute or value shape",
                    ))
                }
            }
        }
        Ok(())
    }
}

/// 从 AST 表达式中提取字符串字面量；若传入非常量字符串字面量则返回编译错误，确保测试目录元数据在编译期完全确定。
fn literal_string(expr: &Expr) -> syn::Result<LitStr> {
    if let Expr::Lit(value) = expr {
        if let Lit::Str(value) = &value.lit {
            return Ok(value.clone());
        }
    }
    Err(syn::Error::new(expr.span(), "expected a string literal"))
}
/// 校验套件名、标签名或参数实例名：必须非空，且不得包含控制字符或用于拼接用例全限定名的分隔符（`:`、`[`、`]`）。
fn valid_label(value: &LitStr) -> syn::Result<()> {
    let name = value.value();
    if name.trim().is_empty()
        || name
            .chars()
            .any(|c| c.is_control() || matches!(c, '[' | ']' | ':'))
    {
        return Err(syn::Error::new(
            value.span(),
            "name must be nonempty without control characters, ':', '[' or ']'",
        ));
    }
    Ok(())
}
/// 校验 `ignore` 原因与 `fatal_message` 诊断子串：必须非空且不包含控制字符（保证单行日志协议不被破坏）。
fn valid_message(value: &LitStr) -> syn::Result<()> {
    if value.value().trim().is_empty() || value.value().chars().any(char::is_control) {
        return Err(syn::Error::new(
            value.span(),
            "message must be nonempty without control characters",
        ));
    }
    Ok(())
}

/// 单个参数化测试用例实例（对应一条 `#[case("label", arg1, arg2, ...)]` 属性或无参默认实例）。
struct Instance {
    /// 参数实例的名称标签；普通非参数化测试为 `None`。
    label: Option<LitStr>,
    /// 传递给测试函数静态参数列表的实参表达式集合。
    arguments: Vec<Expr>,
}
/// 解析并校验通过后的单个测试函数完整描述。
struct Test {
    /// 剥离了 `#[test]`、`#[case]`、`#[ktest]` 等框架专用属性后的原始测试函数 AST。
    function: ItemFn,
    /// 用例级 `#[ktest(...)]` 与 `#[ignore]` 属性解析结果。
    options: Options,
    /// `#[should_panic]` 属性声明的预期 Panic 消息子串（无参 `#[should_panic]` 对应空字符串 `""`）。
    panic: Option<LitStr>,
    /// 该测试函数展开出的全部执行实例（至少包含一个元素）。
    instances: Vec<Instance>,
    /// 测试函数首个参数是否接收 `&mut TestContext`。
    context: bool,
    /// 测试函数第二个参数是否接收用例夹具 `&mut Fixture`。
    fixture: bool,
}

/// 检查函数参数类型树中是否包含 `impl Trait` 匿名泛型语法的 AST 访问器。
struct AnonymousGeneric {
    /// 是否在参数类型中发现了 `TypeImplTrait` 节点。
    found: bool,
}

impl<'ast> Visit<'ast> for AnonymousGeneric {
    /// 遇到 `impl Trait` 类型节点时标记为已发现。
    fn visit_type_impl_trait(&mut self, _: &'ast syn::TypeImplTrait) {
        self.found = true;
    }

    /// 不递归进入类型内部的常量表达式（如数组长度 `[u8; { fn helper(x: impl Sized) {} 4 }]`），避免误判局部辅助函数。
    fn visit_expr(&mut self, _: &'ast Expr) {}
}

/// 解析并校验单个标记了 `#[test]` 的函数，提取测试属性、参数注入方式与参数化实例列表。
fn parse_test(mut function: ItemFn, suite: &Options) -> syn::Result<Test> {
    // Step 1: 校验函数签名修饰符，测试函数必须是安全、非泛型（含无 `impl Trait` 参数）、同步的普通 Rust 函数。
    let signature = &function.sig;
    if signature.asyncness.is_some()
        || signature.unsafety.is_some()
        || signature.constness.is_some()
        || signature.abi.is_some()
        || !signature.generics.params.is_empty()
        || signature.generics.where_clause.is_some()
        || signature.variadic.is_some()
        || signature.inputs.iter().any(|argument| {
            let FnArg::Typed(argument) = argument else {
                return false;
            };
            let mut anonymous = AnonymousGeneric { found: false };
            anonymous.visit_type(&argument.ty);
            anonymous.found
        })
    {
        return Err(syn::Error::new(
            signature.span(),
            "ktest requires a safe, non-generic, synchronous Rust function",
        ));
    }
    // Step 2: 校验函数返回类型，仅允许默认返回、`-> ()` 或 `-> TestResult`。
    match &signature.output {
        ReturnType::Default => (),
        ReturnType::Type(_, ty)
            if matches!(&**ty, Type::Tuple(tuple) if tuple.elems.is_empty())
                || type_name(ty) == Some("TestResult") =>
        {
            ()
        }
        other => {
            return Err(syn::Error::new(
                other.span(),
                "test must return () or TestResult",
            ))
        }
    }
    // Step 3: 扫描并提取函数上的 `#[test]`、`#[case]`、`#[ktest]`、`#[ignore]`、`#[should_panic]` 属性，同时拒绝单个函数上的 `#[cfg]`。
    let mut options = Options::default();
    let mut panic = None;
    let mut panic_seen = false;
    let mut instances = Vec::new();
    let mut retained = Vec::new();
    let mut parameter_names = BTreeSet::new();
    for attribute in function.attrs {
        if attribute.path().is_ident("test") {
            continue;
        }
        if attribute.path().is_ident("case") {
            let args =
                attribute.parse_args_with(Punctuated::<Expr, Token![,]>::parse_terminated)?;
            let mut args = args.into_iter();
            let first = args.next().ok_or_else(|| {
                syn::Error::new(attribute.span(), "case requires a parameter name")
            })?;
            let label = literal_string(&first)?;
            valid_label(&label)?;
            if !parameter_names.insert(label.value()) {
                return Err(syn::Error::new(label.span(), "duplicate parameter name"));
            }
            instances.push(Instance {
                label: Some(label),
                arguments: args.collect(),
            });
        } else if attribute.path().is_ident("ktest") {
            options.parse(attribute.meta.require_list()?.tokens.clone(), false)?;
        } else if attribute.path().is_ident("ignore") {
            if options.ignore.is_some() {
                return Err(syn::Error::new(
                    attribute.span(),
                    "duplicate ignore attribute",
                ));
            }
            let value = match &attribute.meta {
                Meta::NameValue(value) => literal_string(&value.value)?,
                _ => {
                    return Err(syn::Error::new(
                        attribute.span(),
                        "ignore requires a reason: #[ignore = \"reason\"]",
                    ))
                }
            };
            valid_message(&value)?;
            options.ignore = Some(value);
            options.seen.insert("ignore".into());
        } else if attribute.path().is_ident("should_panic") {
            if panic_seen {
                return Err(syn::Error::new(
                    attribute.span(),
                    "duplicate should_panic attribute",
                ));
            }
            panic_seen = true;
            match &attribute.meta {
                Meta::Path(_) => panic = Some(LitStr::new("", attribute.span())),
                Meta::List(list) => {
                    let args =
                        list.parse_args_with(Punctuated::<Meta, Token![,]>::parse_terminated)?;
                    if args.len() != 1 {
                        return Err(syn::Error::new(
                            attribute.span(),
                            "should_panic accepts only expected = \"message\"",
                        ));
                    }
                    match args.first().unwrap() {
                        Meta::NameValue(value) if value.path.is_ident("expected") => {
                            panic = Some(literal_string(&value.value)?)
                        }
                        _ => {
                            return Err(syn::Error::new(
                                attribute.span(),
                                "should_panic accepts only expected = \"message\"",
                            ))
                        }
                    }
                }
                _ => {
                    return Err(syn::Error::new(
                        attribute.span(),
                        "invalid should_panic attribute",
                    ))
                }
            }
        } else if attribute.path().is_ident("cfg") || attribute.path().is_ident("cfg_attr") {
            return Err(syn::Error::new(
                attribute.span(),
                "gate the suite module, not an individual registered test",
            ));
        } else {
            retained.push(attribute);
        }
    }
    function.attrs = retained;
    let inputs: Vec<_> = function.sig.inputs.iter().collect();
    // Step 4: 识别前两个参数是否为框架注入的 `&mut TestContext` 与 `&mut Fixture`，并校验其余静态参数位置未误放 `TestContext` 或 `self`。
    let context = inputs
        .first()
        .is_some_and(|arg| is_mut_reference(arg, "TestContext"));
    // 当首个参数为 `&mut TestContext` 且套件声明了 `fixture` 时，第二个可变引用参数即视为用例夹具引用（具体类型一致性由编译器后续类型检查保证）。
    let fixture = context
        && suite.fixture.is_some()
        && matches!(inputs.get(1).copied(), Some(FnArg::Typed(arg)) if matches!(&*arg.ty, Type::Reference(reference) if reference.mutability.is_some()));
    let injected = usize::from(context) + usize::from(fixture);
    for arg in &inputs[injected..] {
        if let FnArg::Typed(arg) = arg {
            if type_name(&arg.ty) == Some("TestContext")
                || matches!(&*arg.ty, Type::Reference(reference) if type_name(&reference.elem) == Some("TestContext"))
            {
                return Err(syn::Error::new(
                    arg.span(),
                    "TestContext must be the first &mut argument",
                ));
            }
        } else {
            return Err(syn::Error::new(arg.span(), "test cannot have a receiver"));
        }
    }
    // Step 5: 若未声明任何 `#[case]`，则生成单个无标签默认实例；并校验每个实例的实参个数与函数静态参数个数严格一致。
    if instances.is_empty() {
        instances.push(Instance {
            label: None,
            arguments: Vec::new(),
        });
    }
    for instance in &instances {
        if instance.arguments.len() != inputs.len() - injected {
            return Err(syn::Error::new(
                function.sig.span(),
                "case expression count does not match static parameters",
            ));
        }
    }
    // Step 6: 校验 `should_panic` 与 `expected_fatal` 互斥关系，以及 `fatal_message` 的配套约束。
    if panic.is_some() && (options.fatal.is_some() || suite.fatal.is_some()) {
        return Err(syn::Error::new(
            function.sig.span(),
            "should_panic and expected_fatal are mutually exclusive",
        ));
    }
    let fatal = options.fatal.as_ref().or(suite.fatal.as_ref());
    let fatal_message = options
        .fatal_message
        .as_ref()
        .or(suite.fatal_message.as_ref());
    if fatal.is_none() && fatal_message.is_some() {
        return Err(syn::Error::new(
            function.sig.span(),
            "fatal_message requires expected_fatal",
        ));
    }
    if fatal.is_some_and(|value| value.value() == "panic") && fatal_message.is_none() {
        return Err(syn::Error::new(
            function.sig.span(),
            "panic expected_fatal requires a nonempty fatal_message",
        ));
    }
    Ok(Test {
        function,
        options,
        panic,
        instances,
        context,
        fixture,
    })
}

/// 提取路径类型的最末段标识符名称，用于在语法层面识别 `TestContext` 与 `TestResult`。
fn type_name(ty: &Type) -> Option<&str> {
    if let Type::Path(path) = ty {
        let ident = &path.path.segments.last()?.ident;
        if ident == "TestContext" {
            Some("TestContext")
        } else if ident == "TestResult" {
            Some("TestResult")
        } else {
            None
        }
    } else {
        None
    }
}
/// 判断函数参数是否为指向指定类型名称 `name` 的可变引用（`&mut <name>`）。
fn is_mut_reference(arg: &FnArg, name: &str) -> bool {
    matches!(arg, FnArg::Typed(arg) if matches!(&*arg.ty, Type::Reference(reference) if reference.mutability.is_some() && type_name(&reference.elem) == Some(name)))
}

/// 合并套件级与用例级字符串列表（如 `tags` 或 `resources`），保持原有顺序并去除重复项。
fn merged_strings(base: &[LitStr], case: &[LitStr]) -> Vec<LitStr> {
    let mut result = base.to_vec();
    for value in case {
        if !result.iter().any(|old| old.value() == value.value()) {
            result.push(value.clone());
        }
    }
    result
}
/// 合并套件级 `base` 与可选用例级 `case` 属性，生成 `ktest::Defaults` 结构体字面量的 TokenStream。
fn defaults(base: &Options, case: Option<&Options>) -> Tokens {
    let context = if base
        .context
        .as_ref()
        .is_some_and(|value| value.value().eq_ignore_ascii_case("boot"))
    {
        quote!(ktest::Context::Boot)
    } else {
        quote!(ktest::Context::Thread)
    };
    let isolated = base
        .isolation
        .as_ref()
        .is_some_and(|value| value.value().eq_ignore_ascii_case("case"))
        || case.is_some_and(|case| {
            case.isolation
                .as_ref()
                .is_some_and(|value| value.value().eq_ignore_ascii_case("case"))
        })
        || base.fatal.is_some()
        || case.is_some_and(|case| case.fatal.is_some());
    let isolation = if isolated {
        quote!(ktest::Isolation::Case)
    } else {
        quote!(ktest::Isolation::Suite)
    };
    let min_cpus = base
        .min_cpus
        .unwrap_or(1)
        .max(case.and_then(|case| case.min_cpus).unwrap_or(1));
    let timeout = case
        .and_then(|case| case.timeout_ms)
        .or(base.timeout_ms)
        .unwrap_or(10_000);
    let tags = merged_strings(&base.tags, case.map_or(&[], |case| &case.tags));
    let resources = merged_strings(&base.resources, case.map_or(&[], |case| &case.resources));
    let resources = resources.iter().map(|value| match value.value().as_str() {
        "uart" => quote!(ktest::Resource::Uart),
        "scratch" => quote!(ktest::Resource::Scratch),
        "root" => quote!(ktest::Resource::Root),
        "extra" => quote!(ktest::Resource::Extra),
        "ap-freeze" => quote!(ktest::Resource::ApFreeze),
        "virtio-access-platform" => quote!(ktest::Resource::VirtioAccessPlatform),
        "net-user" => quote!(ktest::Resource::NetUser),
        _ => unreachable!(),
    });
    quote!(ktest::Defaults { context: #context, isolation: #isolation, tags: &[#(#tags),*], min_cpus: #min_cpus, timeout_ms: #timeout, resources: &[#(#resources),*] })
}
/// 将编译期可选字符串字面量转换为运行时的 `Some("...")` 或 `None` 表达式 TokenStream。
fn optional(value: Option<&LitStr>) -> Tokens {
    match value {
        Some(value) => quote!(Some(#value)),
        None => quote!(None),
    }
}

/// 展开 `#[ktest::suite]` 模块：解析套件选项、收集测试函数、生成用例包装函数与静态注册表。
fn expand(attributes: Tokens, input: Tokens) -> syn::Result<Tokens> {
    // Step 1: 解析套件级属性，并确保输入是一个内联模块（`mod name { ... }`）。
    let mut options = Options::default();
    options.parse(attributes, true)?;
    let mut module: ItemMod = syn::parse2(input)?;
    let module_span = module.ident.span();
    let (_, items) = module
        .content
        .as_mut()
        .ok_or_else(|| syn::Error::new(module_span, "suite requires an inline module"))?;
    // Step 2: 遍历模块内条目，分离出带有 `#[test]` 的测试函数并校验无重名测试，其余辅助条目原样保留。
    let mut tests = Vec::new();
    let mut names = BTreeSet::new();
    let mut preserved = Vec::new();
    for item in items.drain(..) {
        if let Item::Fn(function) = item {
            if function
                .attrs
                .iter()
                .any(|attr| attr.path().is_ident("test"))
            {
                if !names.insert(function.sig.ident.to_string()) {
                    return Err(syn::Error::new(
                        function.sig.ident.span(),
                        "duplicate test name",
                    ));
                }
                tests.push(parse_test(function, &options)?);
            } else {
                preserved.push(Item::Fn(function));
            }
        } else {
            preserved.push(item);
        }
    }
    if tests.is_empty() {
        return Err(syn::Error::new(
            module.ident.span(),
            "suite must contain at least one test",
        ));
    }
    // Step 3: 为每个测试函数的每个参数化实例生成 `CaseDesc` 静态描述符、生命周期包装函数 `__ktest_case_N` 及分派分支。
    let fixture = options
        .fixture
        .clone()
        .unwrap_or_else(|| syn::parse_quote!(()));
    let base = options
        .base
        .clone()
        .unwrap_or_else(|| syn::parse_quote!(()));
    let suite_name = match &options.name {
        Some(name) => quote!(concat!(module_path!(), "::", #name)),
        None => quote!(module_path!()),
    };
    let suite_defaults = defaults(&options, None);
    let mut cases = Vec::new();
    let mut wrappers = Vec::new();
    let mut arms = Vec::new();
    for test in tests {
        let function = &test.function;
        let name = &function.sig.ident;
        let source_span = name.span();
        let attrs = defaults(&options, Some(&test.options));
        let ignore = optional(test.options.ignore.as_ref().or(options.ignore.as_ref()));
        let panic = optional(test.panic.as_ref());
        let fatal = optional(test.options.fatal.as_ref().or(options.fatal.as_ref()));
        let fatal_message = optional(
            test.options
                .fatal_message
                .as_ref()
                .or(options.fatal_message.as_ref()),
        );
        for instance in &test.instances {
            let index = cases.len();
            let wrapper = format_ident!("__ktest_case_{}", index, span = Span::mixed_site());
            let full_name = match &instance.label {
                Some(label) => {
                    quote!(concat!(#suite_name, "::", stringify!(#name), "[", #label, "]"))
                }
                None => quote!(concat!(#suite_name, "::", stringify!(#name))),
            };
            let parameter = optional(instance.label.as_ref());
            let source = quote::quote_spanned!(source_span=> file!());
            let line = quote::quote_spanned!(source_span=> line!());
            cases.push(quote!(ktest::CaseDesc { name: #full_name, parameter: #parameter, source: #source, line: #line, attributes: #attrs, ignore: #ignore, should_panic: #panic, expected_fatal: #fatal, fatal_message: #fatal_message }));
            let mut arguments = Vec::new();
            if test.context {
                arguments.push(quote!(__context));
            }
            if test.fixture {
                arguments.push(quote!(__fixture));
            }
            arguments.extend(instance.arguments.iter().map(ToTokens::to_token_stream));
            wrappers.push(quote!(
                /// 执行单个测试实例的包装函数：将夹具构造与销毁交由 `run_case` 管理，仅在测试主体闭包内对参数表达式求值并调用测试函数。
                fn #wrapper(__context: &mut ktest::TestContext<'_>, __resources: &#base) -> ktest::CaseOutcome {
                    ktest::run_case::<#fixture, #base>(__context, __resources, |__context, __fixture| {
                        ktest::IntoTestResult::into_test_result(#name(#(#arguments),*))
                    })
                }
            ));
            arms.push(quote!(#index => #wrapper(__context, __resources)));
        }
        preserved.push(Item::Fn(test.function));
    }
    // Step 4: 生成静态用例表、索引分派器、套件运行入口、`.ktest.suites` 注册槽及链接归档锚点函数（使用完整模块路径作为导出符号名以避免跨模块同名冲突）。
    let generated: syn::File = syn::parse2(quote!(
        #(#wrappers)*
        pub static __KTEST_CASES: &[ktest::CaseDesc] = &[#(#cases),*];
        /// 根据静态用例索引分派执行对应的包装函数 `__ktest_case_N`。
        fn __ktest_dispatch(__context: &mut ktest::TestContext<'_>, __resources: &#base, __index: usize) -> ktest::CaseOutcome {
            match __index { #(#arms,)* _ => panic!("invalid ktest case index") }
        }
        /// 套件统一运行入口：初始化套件级共享资源 `#base` 并驱动 `CaseRunner` 执行选中的用例。
        fn __ktest_run(__context: &mut ktest::TestContext<'_>, __runner: &mut dyn ktest::CaseRunner) -> ktest::SuiteOutcome {
            ktest::run_suite::<#base>(__context, __runner, __KTEST_CASES, __ktest_dispatch)
        }
        pub static __KTEST_SUITE: ktest::SuiteDesc = ktest::SuiteDesc {
            name: #suite_name, defaults: #suite_defaults, cases: __KTEST_CASES, run: __ktest_run,
        };
        #[used]
        #[link_section = ".ktest.suites"]
        #[export_name = concat!("__ktest_register__", module_path!())]
        pub static __KTEST_SLOT: ktest::SuiteSlot = ktest::SuiteSlot(&__KTEST_SUITE);
        /// 返回当前套件注册槽的静态引用，作为编译产物中的显式符号引用锚点，确保静态库链接时保留 `.ktest.suites` 槽位。
        #[inline(never)]
        pub fn __ktest_anchor() -> &'static ktest::SuiteSlot { &__KTEST_SLOT }
    ))?;
    preserved.extend(generated.items);
    *items = preserved;
    Ok(quote!(#module))
}

#[cfg(test)]
mod tests;
