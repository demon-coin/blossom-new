//! `ktest-macros` 过程宏编译期语法与元数据校验单元测试。
//!
//! # 模块职责与架构定位
//! 本模块针对 `crate::expand` 展开函数进行负向边界测试，验证过程宏在遇到非法函数签名、
//! 匿名泛型参数、歧义用例命名、冲突或缺失的元数据属性以及用例级条件编译时，能够在编译期拒绝并返回错误。

use quote::quote;

use crate::expand;

/// 测试目标：验证不支持的函数签名修饰符、返回类型或上下文参数位置会在宏展开阶段被拒绝。
/// 场景步骤：依次向 `expand` 传入 `async`、泛型、返回 `bool`、`unsafe`、`extern "C"`、无 `#[case]` 却带参数、不可变借用 `&TestContext` 以及 `TestContext` 非首参的测试函数。
/// 预期结果：所有非法函数签名的 `expand` 调用均返回 `Err`。
#[test]
fn rejects_unsupported_signatures() {
    let functions = [
        quote!(
            async fn example() {}
        ),
        quote!(
            fn example<T>() {}
        ),
        quote!(
            fn example() -> bool {
                true
            }
        ),
        quote!(
            unsafe fn example() {}
        ),
        quote!(
            extern "C" fn example() {}
        ),
        quote!(
            fn example(value: usize) {}
        ),
        quote!(
            fn example(context: &ktest::TestContext) {}
        ),
        quote!(
            fn example(value: usize, context: &mut ktest::TestContext) {}
        ),
    ];
    for function in functions {
        assert!(expand(quote!(), quote!(mod tests { #[test] #function })).is_err());
    }
}

/// 测试目标：验证函数参数中的 `impl Trait` 匿名泛型（包括裸形式、不可变/可变引用形式及元组包裹形式）会被宏展开拒绝。
/// 场景步骤：构造参数类型为 `impl Debug`、`&impl Debug`、`&mut impl Debug` 与 `(impl Debug)` 的参数化测试函数并调用 `expand`。
/// 预期结果：所有包含 `impl Trait` 参数的测试函数均在宏展开时返回 `Err`。
#[test]
fn rejects_anonymous_generic_parameters() {
    for parameter in [
        quote!(impl core::fmt::Debug),
        quote!(&impl core::fmt::Debug),
        quote!(&mut impl core::fmt::Debug),
        quote!((impl core::fmt::Debug)),
    ] {
        assert!(expand(
            quote!(),
            quote!(
                mod tests {
                    #[test]
                    #[case("one", 1u32)]
                    fn example(value: #parameter) {}
                }
            )
        )
        .is_err());
    }
}

/// 测试目标：验证会导致测试用例全限定名冲突、解析歧义或实参个数不匹配的定义会被拒绝。
/// 场景步骤：向 `expand` 传入重复的 `#[case]` 标签名、含 `[`/`]`/`::`/换行符或空串的标签名、实参个数与形参不匹配的 `#[case]`，以及同模块内重名的两个 `#[test]` 函数。
/// 预期结果：所有存在命名歧义、重名或参数数量不匹配的输入均返回 `Err`。
#[test]
fn rejects_ambiguous_parameter_instances() {
    let inputs = [
        quote!(
            mod tests {
                #[test]
                #[case("same", 1)]
                #[case("same", 2)]
                fn example(value: usize) {}
            }
        ),
        quote!(
            mod tests {
                #[test]
                #[case("bad[name]", 1)]
                fn example(value: usize) {}
            }
        ),
        quote!(
            mod tests {
                #[test]
                #[case("bad::name", 1)]
                fn example(value: usize) {}
            }
        ),
        quote!(
            mod tests {
                #[test]
                #[case("bad\nname", 1)]
                fn example(value: usize) {}
            }
        ),
        quote!(
            mod tests {
                #[test]
                #[case("", 1)]
                fn example(value: usize) {}
            }
        ),
        quote!(
            mod tests {
                #[test]
                #[case("one", 1, 2)]
                fn example(value: usize) {}
            }
        ),
        quote!(
            mod tests {
                #[test]
                fn same() {}
                #[test]
                fn same() {}
            }
        ),
    ];
    for input in inputs {
        assert!(expand(quote!(), input).is_err());
    }
}

/// 测试目标：验证重复属性、非正数配额、未知枚举值以及不完整的致命异常配置会被宏展开拒绝。
/// 场景步骤：分别测试重复 `min_cpus`、零值配额、`timeout` 与 `timeout_ms` 混用、非法 `context`、未知或重复 `resources`、缺失或空的 `fatal_message`、`expected_fatal` 与 `should_panic` 混用，以及在用例级指定 `context`。
/// 预期结果：所有无效元数据组合均在宏展开阶段返回 `Err`。
#[test]
fn rejects_invalid_metadata() {
    let attributes = [
        quote!(min_cpus = 1, min_cpus = 2),
        quote!(min_cpus = 0),
        quote!(timeout_ms = 0),
        quote!(timeout = 3, timeout_ms = 4),
        quote!(context = "Interrupt"),
        quote!(resources("network")),
        quote!(resources("root", "root")),
        quote!(expected_fatal = "unknown"),
        quote!(expected_fatal = "panic"),
        quote!(fatal_message = "unbound"),
        quote!(expected_fatal = "panic", fatal_message = ""),
    ];
    for attribute in attributes {
        assert!(expand(
            attribute,
            quote!(
                mod tests {
                    #[test]
                    fn example() {}
                }
            )
        )
        .is_err());
    }
    assert!(expand(
        quote!(expected_fatal = "illegal"),
        quote!(
            mod tests {
                #[test]
                #[should_panic]
                fn example() {}
            }
        )
    )
    .is_err());
    assert!(expand(
        quote!(),
        quote!(
            mod tests {
                #[test]
                #[ktest(context = "Boot")]
                fn example() {}
            }
        )
    )
    .is_err());
}

/// 测试目标：验证禁止在单个注册的测试函数上使用 `#[cfg]` 或 `#[cfg_attr]` 条件编译属性。
/// 场景步骤：对套件模块内的单个 `#[test]` 函数附加 `#[cfg(feature = "other")]` 并调用 `expand`。
/// 预期结果：`expand` 返回 `Err`，强制要求在套件模块外层进行条件编译门控。
#[test]
fn rejects_per_case_conditional_compilation() {
    assert!(expand(
        quote!(),
        quote!(
            mod tests {
                #[test]
                #[cfg(feature = "other")]
                fn example() {}
            }
        )
    )
    .is_err());
}
