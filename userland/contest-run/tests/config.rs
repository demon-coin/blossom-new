//! `contest-run` 测试计划配置加载（`config`）与用例选择展开（`selection`）集成测试集。
//!
//! # 模块职责与架构定位
//! 本文件作为 `contest-run` 的外部集成测试，针对 [`contest_run::config::load`] 与 [`contest_run::selection::resolve`] 构成的完整输入与选择流水线进行端到端验证，覆盖：
//! 1. 多文件引入顺序、目标根目录（`roots`）迭代顺序、通配符（Glob）匹配与排除（`exclude`）规则；
//! 2. 命令行显式子配置文件列表覆盖与防递归引入保护；
//! 3. `Exec`、Shell 与 LTP `runtest` 清单命令的参数原样保留语义；
//! 4. 多级环境变量合并、`override` 精确覆盖、重复 ID 去重与冲突检测；
//! 5. 目标根目录下的相对/绝对符号链接（Symlink）受限解析行为。
//!
//! # 测试夹具与隔离布局（[`Tree`]）
//! ```text
//! $TMPDIR/contest-config-<pid>-<seq>/    <-- Tree(PathBuf) 独立临时目录（Drop 时自动清理）
//! ├── plan.toml                          <-- 顶层测试计划文件
//! ├── sets/*.toml / override.toml        <-- 子步骤文件与覆盖规则文件
//! └── root/                              <-- 模拟的目标系统根目录 (roots.a / roots.b)
//!     ├── catalog / run-dynamic.sh       <-- 文本测试清单
//!     └── cases/* / basic/*              <-- 具有 0o755 权限的可执行脚本或符号链接
//! ```
//!
//! # 锁获取顺序与并发契约
//! 各测试用例通过全局原子计数器 [`NEXT`] 生成互不冲突的临时子目录，支持 `cargo test` 多线程并行运行，无需共享互斥锁。

use contest_run::config;
use contest_run::model::{CommandSpec, Protocol};
use contest_run::selection;
use std::fmt::Write;
use std::fs;
use std::os::unix::fs::{symlink, PermissionsExt};
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::atomic::{AtomicUsize, Ordering};

/// 用于为每个测试实例分配唯一临时目录序号的全局原子递增计数器。
static NEXT: AtomicUsize = AtomicUsize::new(0);

/// 单个测试用例独占的临时目录夹具（RAII 守卫，在析构时自动递归删除整个目录）。
struct Tree(PathBuf);

impl Tree {
    /// 在系统临时目录下创建一个带进程号与递增序号的独立测试目录。
    fn new() -> Self {
        let path = std::env::temp_dir().join(format!(
            "contest-config-{}-{}",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        ));
        fs::create_dir_all(&path).unwrap();
        Self(path)
    }

    /// 将相对路径拼接到当前测试临时目录根路径下，返回完整路径。
    fn path(&self, relative: &str) -> PathBuf {
        self.0.join(relative)
    }

    /// 在测试临时目录内创建指定相对路径的文件（自动创建缺失的父目录）并写入文本内容。
    fn put(&self, relative: &str, contents: &str) {
        let path = self.path(relative);
        fs::create_dir_all(path.parent().unwrap()).unwrap();
        fs::write(path, contents).unwrap();
    }

    /// 在测试临时目录内创建一个最小可执行 Shell 脚本，并将其权限位设置为 `0o755`。
    fn executable(&self, relative: &str) {
        self.put(relative, "#!/bin/sh\nexit 0\n");
        fs::set_permissions(self.path(relative), fs::Permissions::from_mode(0o755)).unwrap();
    }

    /// 生成指向同一个 `root` 子目录的两个根目录（`roots.a` 与 `roots.b`）的 TOML 配置片段，用于测试多 `roots` 展开顺序。
    fn roots(&self) -> String {
        format!(
            "[roots.a]\npath = {:?}\ncwd = '/'\n[roots.b]\npath = {:?}\ncwd = '/'\n",
            self.path("root"),
            self.path("root")
        )
    }

    /// 加载指定相对路径的测试计划文件，并按 `"riscv64"` 架构解析展开为 [`contest_run::model::ResolvedCase`] 列表。
    fn resolve(&self, plan: &str) -> Result<Vec<contest_run::model::ResolvedCase>, String> {
        let config = config::load(&self.path(plan), &[])?;
        selection::resolve(&config, "riscv64")
    }
}

impl Drop for Tree {
    /// 测试结束时递归清理当前测试实例创建的临时目录。
    fn drop(&mut self) {
        fs::remove_dir_all(&self.0).unwrap();
    }
}

/// 验证多配置文件、`roots` 列表、`include` 精确名与通配符（Glob）、`exclude` 排除项的展开顺序及源文件行号溯源。
///
/// - **测试目标**：确认 `selection::resolve` 严格按照文件顺序 -> `roots` 顺序 -> `include` 顺序 -> 清单行顺序生成用例，并准确记录清单文件路径与行号。
/// - **场景步骤**：构造包含两个子文件、两个 root（`['b', 'a']`）、多条 `include` 模式与 `exclude` 过滤的测试计划并解析。
/// - **预期结果**：返回的 ID 序列与预期顺序完全一致，且 `source` 元数据准确指向 `/catalog` 第 2 行。
#[test]
fn multifile_order_globs_exclude_and_locations() {
    let tree = Tree::new();
    tree.put(
        "root/catalog",
        "# comment\ncase2 printf two\ncase1 printf one\ncase3 printf three\nbeta printf beta\n",
    );
    tree.put(
        "plan.toml",
        &format!(
            "files = ['sets/first.toml', 'sets/last.toml']\n{}",
            tree.roots()
        ),
    );
    tree.put("sets/first.toml", "# 多行 include 与注释使用 TOML 原语法\n[[step]]\nkind='ltp'\nroots=['b', 'a']\nsuite='syscalls'\ncatalog='/catalog'\ninclude=[\n 'beta', # 先精确名\n 'case[12]',\n 'case?',\n]\nexclude=['case3']\n");
    tree.put(
        "sets/last.toml",
        "[[step]]\nkind='exec'\nid='last'\nroot='a'\nprogram='/bin/last'\n",
    );
    let cases = tree.resolve("plan.toml").unwrap();
    let ids: Vec<&str> = cases.iter().map(|case| case.id.as_str()).collect();
    assert_eq!(
        ids,
        [
            "ltp:b:syscalls:beta",
            "ltp:b:syscalls:case2",
            "ltp:b:syscalls:case1",
            "ltp:a:syscalls:beta",
            "ltp:a:syscalls:case2",
            "ltp:a:syscalls:case1",
            "last"
        ]
    );
    assert_eq!(cases[1].source.file, tree.path("sets/first.toml"));
    assert_eq!(cases[1].source.entry, 1);
    assert_eq!(
        cases[1].source.catalog.as_deref(),
        Some(Path::new("/catalog"))
    );
    assert_eq!(cases[1].source.line, Some(2));
}

/// 验证命令行显式传入的子文件列表会替换顶层 `files` 字段，且子文件不允许再次嵌套声明 `files`。
///
/// - **测试目标**：确保 CLI 文件覆盖只替换步骤文件列表而不丢失顶层 `defaults` 与 `roots`，同时拒绝递归包含。
/// - **场景步骤**：先用 `chosen.toml` 替换不存在的默认 `files` 并验证解析结果；再向 `chosen.toml` 写入嵌套 `files` 字段。
/// - **预期结果**：首次解析继承了顶层 `timeout_sec = 17`；二次加载因检测到子文件中的 `files` 字段而报错。
#[test]
fn cli_files_replace_without_recursive_loading() {
    let tree = Tree::new();
    tree.put(
        "plan.toml",
        &format!(
            "files=['not-present.toml']\n[defaults]\ntimeout_sec=17\n{}",
            tree.roots()
        ),
    );
    tree.put(
        "chosen.toml",
        "[[step]]\nkind='exec'\nid='chosen'\nroot='b'\nprogram='/bin/true'\n",
    );
    let config = config::load(&tree.path("plan.toml"), &[PathBuf::from("chosen.toml")]).unwrap();
    let cases = selection::resolve(&config, "loongarch64").unwrap();
    assert_eq!(cases.len(), 1);
    assert_eq!(cases[0].id, "chosen");
    assert_eq!(cases[0].root, "b");
    assert_eq!(cases[0].timeout_sec, 17);
    tree.put("chosen.toml", "files=['nested.toml']\n");
    let error = config::load(&tree.path("plan.toml"), &[PathBuf::from("chosen.toml")]).unwrap_err();
    assert!(error.contains("chosen.toml") && error.contains("files"));
}

/// 验证被禁用（`enabled = false`）或架构不匹配的测试步骤不会读取清单文件，且解析阶段不执行任何 `prepare`、`setup` 或 `cleanup` 命令。
///
/// - **测试目标**：保证配置解析与选择阶段无任何外部命令执行副作用，且未启用的步骤不要求其清单路径存在。
/// - **场景步骤**：配置带 `touch` 副作用的 `prepare`/`setup`/`cleanup` 命令，以及指向不存在清单的禁用步骤和非当前架构步骤，调用 `resolve`。
/// - **预期结果**：仅选中当前架构下启用的 `active` 步骤，且文件系统上未产生 `must-not-exist` 标记文件。
#[test]
fn disabled_arch_and_list_have_no_side_effects() {
    let tree = Tree::new();
    let marker = tree.path("must-not-exist");
    tree.put(
        "plan.toml",
        &format!(
            "files=['steps.toml']\n[roots.a]\npath={:?}\nprepare=[['/bin/sh','-c','touch {}']]\n",
            tree.path("root"),
            marker.display()
        ),
    );
    tree.put("steps.toml", &format!("[[step]]\nkind='ltp'\nenabled=false\nroots=['a']\nsuite='off'\ncatalog='/missing'\ninclude=['*']\n[[step]]\nkind='ltp'\narch=['loongarch64']\nroots=['a']\nsuite='la'\ncatalog='/also-missing'\ninclude=['*']\n[[step]]\nkind='exec'\nid='active'\nroot='a'\nprogram='/bin/true'\nhooks=true\nsetup=[['/bin/sh','-c','touch {}']]\ncleanup=[['/bin/sh','-c','touch {}']]\n", marker.display(), marker.display()));
    let cases = tree.resolve("plan.toml").unwrap();
    assert_eq!(cases.len(), 1);
    assert!(cases[0].hooks);
    assert!(!marker.exists());
}

/// 验证 `exec`、LTP `runtest` 与 `shell` 步骤混合出现时，空参数、含空格/Unicode 参数及原始 Shell 命令行文本均原样保留。
///
/// - **测试目标**：确保 `exec` 参数不被拆词，且 LTP 清单中的命令尾部空白、引号与管道符号不被破坏。
/// - **场景步骤**：构造包含空字符串与多字节字符的 `exec` 步骤、带管道与尾部空格的 LTP 步骤及 `shell` 步骤并解析。
/// - **预期结果**：各用例的 `CommandSpec` 与输入完全吻合。
#[test]
fn mixed_commands_preserve_empty_arguments_and_shell_text() {
    let tree = Tree::new();
    let command = "MODE='value with spaces' printf '%s\\n' '' '雪 山' | cat  ";
    tree.put("root/catalog", &format!("tag {command}\n"));
    tree.put("plan.toml", &format!("{}\n[[step]]\nkind='exec'\nid='first'\nroot='a'\nprogram='./probe'\nargs=['','a b','雪 山','$literal']\n[[step]]\nkind='ltp'\nroots=['a']\nsuite='mixed'\ncatalog='/catalog'\nshell='/custom/sh'\ninclude=['tag']\n[[step]]\nkind='shell'\nid='last'\nroot='a'\nshell='/bin/sh'\ncommand='echo last | cat'\n", tree.roots()));
    let cases = tree.resolve("plan.toml").unwrap();
    assert_eq!(
        cases
            .iter()
            .map(|case| case.id.as_str())
            .collect::<Vec<_>>(),
        ["first", "ltp:a:mixed:tag", "last"]
    );
    assert_eq!(
        cases[0].command,
        CommandSpec::Exec {
            program: "./probe".into(),
            args: vec!["".into(), "a b".into(), "雪 山".into(), "$literal".into()]
        }
    );
    assert_eq!(
        cases[1].command,
        CommandSpec::Shell {
            shell: "/custom/sh".into(),
            command: command.into()
        }
    );
}

/// 验证从 LTP `runtest` 清单提取出的命令文本在交由 `/bin/sh -c` 执行时保持完整的 Shell 语法语义。
///
/// - **测试目标**：确认清单命令中的内联环境变量赋值、单双引号嵌套、空位置参数和管道可由 Shell 正常解析执行。
/// - **场景步骤**：解析包含复杂 Shell 表达式的 LTP 条目，并实际调用 `Command::new(shell).args(["-c", command])` 运行。
/// - **预期结果**：子进程成功退出，标准输出精确匹配 `"<two words><>"`。
#[test]
fn runtest_tail_runs_with_original_shell_semantics() {
    let tree = Tree::new();
    tree.put(
        "root/catalog",
        "tag VALUE='two words' /bin/sh -c 'printf \"<%s><%s>\" \"$VALUE\" \"$1\"' sh '' | cat\n",
    );
    tree.put("plan.toml", &format!("{}\n[[step]]\nkind='ltp'\nroots=['a']\nsuite='shell'\ncatalog='/catalog'\nshell='/bin/sh'\ninclude=['tag']\n", tree.roots()));
    let cases = tree.resolve("plan.toml").unwrap();
    let CommandSpec::Shell { shell, command } = &cases[0].command else {
        panic!("not shell");
    };
    let output = Command::new(shell).args(["-c", command]).output().unwrap();
    assert!(output.status.success());
    assert_eq!(output.stdout, b"<two words><>");
}

/// 验证精确 ID 覆盖（`[[override]]`）按层级合并环境变量，并对 `args`、`setup`、`cleanup` 等数组字段执行整体替换。
///
/// - **测试目标**：确认 `defaults` -> `root` -> `step` -> `override` 四层配置的合并与覆盖优先级符合设计规范。
/// - **场景步骤**：在四层配置中分别设置同名与异名环境变量及 `setup`/`cleanup` 数组，解析并检查最终生成的 `ResolvedCase`。
/// - **预期结果**：环境变量保留各层非冲突键且同名键取最高优先级值；数组字段完全被 `override` 中的新数组替换。
#[test]
fn exact_override_merges_environment_and_replaces_arrays() {
    let tree = Tree::new();
    tree.put("plan.toml", &format!("files=['steps.toml']\noverride_files=['override.toml']\n[defaults]\nenv={{KEEP='default',LEVEL='default'}}\nsetup=[['/default']]\ncleanup=[['/default-clean']]\n[roots.a]\npath={:?}\nenv={{LEVEL='root',ROOT='yes'}}\n", tree.path("root")));
    tree.put("steps.toml", "[[step]]\nkind='exec'\nid='one'\nroot='a'\nprogram='/old'\nargs=['old','args']\nenv={LEVEL='step',STEP='yes'}\nsetup=[['/step']]\ncleanup=[['/step-clean']]\n");
    tree.put("override.toml", "[[override]]\nid='one'\nprogram='/new'\nargs=['','a b']\nenv={LEVEL='override',LOCAL='yes'}\nsetup=[]\ncleanup=[['/cleanup','x y']]\ntimeout_sec=19\nrepeat=3\nhooks=true\n");
    let cases = tree.resolve("plan.toml").unwrap();
    let case = &cases[0];
    assert_eq!(cases.len(), 1);
    assert_eq!(case.id, "one");
    assert_eq!(
        case.command,
        CommandSpec::Exec {
            program: "/new".into(),
            args: vec!["".into(), "a b".into()]
        }
    );
    assert_eq!(case.env["LEVEL"], "override");
    assert_eq!(case.env["KEEP"], "default");
    assert_eq!(case.env["ROOT"], "yes");
    assert_eq!(case.env["STEP"], "yes");
    assert_eq!(case.env["LOCAL"], "yes");
    assert!(case.setup.is_empty());
    assert_eq!(case.cleanup, vec![vec!["/cleanup", "x y"]]);
    assert_eq!(case.timeout_sec, 19);
    assert_eq!(case.repeat, 3);
    assert!(case.hooks);
}

/// 验证重复命中同一用例 ID 时的幂等去重、语义冲突报错以及通过 `[[override]]` 统一冲突字段的行为。
///
/// - **测试目标**：确保完全相同的重复定义保留首次来源，存在差异的重复定义显式报错，且可通过 `override` 消解冲突。
/// - **场景步骤**：先在两个子文件中定义完全相同的 `same` 步骤；再将第二个改为不同程序路径触发冲突；最后添加 `fix.toml` 覆盖 `program`。
/// - **预期结果**：语义相同时去重保留 `one.toml` 来源；语义不同时报错指出两个来源文件；添加覆盖后两者收敛为同一语义并通过解析。
#[test]
fn duplicate_semantics_conflict_and_explicit_override() {
    let tree = Tree::new();
    tree.put(
        "plan.toml",
        &format!("files=['one.toml','two.toml']\n{}", tree.roots()),
    );
    let one = "[[step]]\nkind='exec'\nid='same'\nroot='a'\nprogram='/one'\n";
    tree.put("one.toml", one);
    tree.put("two.toml", one);
    let cases = tree.resolve("plan.toml").unwrap();
    assert_eq!(cases.len(), 1);
    assert_eq!(cases[0].source.file, tree.path("one.toml"));
    tree.put("two.toml", &one.replace("/one", "/two"));
    let error = tree.resolve("plan.toml").unwrap_err();
    assert!(
        error.contains("conflicting selection ID")
            && error.contains("one.toml")
            && error.contains("two.toml")
    );
    tree.put(
        "plan.toml",
        &format!(
            "files=['one.toml','two.toml']\noverride_files=['fix.toml']\n{}",
            tree.roots()
        ),
    );
    tree.put("fix.toml", "[[override]]\nid='same'\nprogram='/fixed'\n");
    let cases = tree.resolve("plan.toml").unwrap();
    assert_eq!(cases.len(), 1);
    assert_eq!(
        cases[0].command,
        CommandSpec::Exec {
            program: "/fixed".into(),
            args: vec![]
        }
    );
}

/// 验证 `[[override]]` 拒绝未选中的目标 ID，并拒绝同一 ID 的多条覆盖规则在同一字段或环境变量上设置不同值。
///
/// - **测试目标**：防止拼写错误的覆盖 ID 被静默忽略，并杜绝多条覆盖规则之间的隐式先后覆盖歧义。
/// - **场景步骤**：依次测试指向未选中 ID 的覆盖、同一字段赋予不同值的两条覆盖、同一环境变量赋予不同值的两条覆盖，以及补充不同字段（或相同值）的两条合法覆盖。
/// - **预期结果**：前三者均返回包含具体位置与冲突字段名的错误，最后一种合法组合成功合并。
#[test]
fn overrides_reject_unselected_ids_and_field_conflicts() {
    let tree = Tree::new();
    tree.put("plan.toml", &format!("override_files=['override.toml']\n{}\n[[step]]\nkind='exec'\nid='one'\nroot='a'\nprogram='/bin/true'\n", tree.roots()));
    tree.put("override.toml", "[[override]]\nid='missing'\nrepeat=2\n");
    let error = tree.resolve("plan.toml").unwrap_err();
    assert!(error.contains("override.toml") && error.contains("not selected"));
    tree.put(
        "override.toml",
        "[[override]]\nid='one'\nrepeat=2\n[[override]]\nid='one'\nrepeat=3\n",
    );
    let error = tree.resolve("plan.toml").unwrap_err();
    assert!(error.contains("entry 2") && error.contains("entry 1") && error.contains("repeat"));
    tree.put(
        "override.toml",
        "[[override]]\nid='one'\nenv={A='one'}\n[[override]]\nid='one'\nenv={A='two'}\n",
    );
    assert!(tree
        .resolve("plan.toml")
        .unwrap_err()
        .contains("conflicting override env"));
    tree.put(
        "override.toml",
        "[[override]]\nid='one'\nrepeat=2\n[[override]]\nid='one'\ntimeout_sec=23\nrepeat=2\n",
    );
    let cases = tree.resolve("plan.toml").unwrap();
    assert_eq!((cases[0].repeat, cases[0].timeout_sec), (2, 23));
}

/// 验证清单名称未命中、通配符未命中、排除后为空以及全局无选中用例时的错误定位信息。
///
/// - **测试目标**：确保选择阶段的所有空结果错误都准确报告对应的步骤位置、目标 `root` 名称与清单路径。
/// - **场景步骤**：分别使用不存在的精确名、不匹配的通配符、将唯一匹配项排除的配置以及全部禁用的步骤调用 `resolve`。
/// - **预期结果**：每次调用均返回包含预期错误分类关键词与源文件位置的错误信息。
#[test]
fn selection_errors_report_the_actual_source() {
    let tree = Tree::new();
    tree.put("root/catalog", "known printf yes\n");
    tree.put(
        "plan.toml",
        &format!("files=['select.toml']\n{}", tree.roots()),
    );
    for (include, exclude, expected) in [
        ("missing", "", "name"),
        ("absent*", "", "pattern"),
        ("known", "known", "empty selection after exclude"),
    ] {
        tree.put("select.toml", &format!("[[step]]\nkind='ltp'\nroots=['a']\nsuite='s'\ncatalog='/catalog'\ninclude=[{include:?}]\nexclude=[{exclude:?}]\n").replace("exclude=[\"\"]", "exclude=[]"));
        let error = tree.resolve("plan.toml").unwrap_err();
        assert!(
            error.contains("select.toml:entry 1")
                && error.contains("root \"a\"")
                && error.contains("/catalog")
                && error.contains(expected),
            "{error}"
        );
    }
    tree.put(
        "select.toml",
        "[[step]]\nkind='exec'\nenabled=false\nid='off'\nroot='a'\nprogram='/missing'\n",
    );
    assert!(tree
        .resolve("plan.toml")
        .unwrap_err()
        .contains("empty selection"));
}

/// 验证所有 TOML 配置层级均拒绝未知拼写错误字段，且被引入的子文件禁止再次声明 `files` 或 `override_files`。
///
/// - **测试目标**：确保配置反序列化启用严格字段校验（`deny_unknown_fields`），并限制文件包含关系为单层树结构。
/// - **场景步骤**：依次向顶层、`[defaults]`、`[roots]`、`[[step]]`、`[[override]]` 注入拼写错误的字段名，并在子文件中声明 `override_files`。
/// - **预期结果**：所有非法输入均在 `config::load` 阶段立即报错并指出具体文件名。
#[test]
fn unknown_fields_and_nested_files_are_rejected_at_input() {
    let tree = Tree::new();
    for text in [
        "typo=true",
        "[defaults]\ntimeuot_sec=2",
        "[roots.a]\npath='/'\nunknown=true",
        "[[step]]\nkind='exec'\nid='x'\nroot='a'\nprogram='/x'\nargz=[]",
    ] {
        tree.put("plan.toml", text);
        let error = config::load(&tree.path("plan.toml"), &[]).unwrap_err();
        assert!(
            error.contains("plan.toml") && error.contains("unknown field"),
            "{error}"
        );
    }
    tree.put(
        "plan.toml",
        &format!("files=['nested.toml']\n{}", tree.roots()),
    );
    tree.put("nested.toml", "override_files=['deeper.toml']");
    assert!(config::load(&tree.path("plan.toml"), &[])
        .unwrap_err()
        .contains("nested.toml"));
    tree.put(
        "plan.toml",
        &format!("override_files=['bad.toml']\n{}", tree.roots()),
    );
    tree.put("bad.toml", "[[override]]\nid='x'\nunknown=true");
    assert!(config::load(&tree.path("plan.toml"), &[])
        .unwrap_err()
        .contains("unknown field"));
}

/// 验证缺失必填字段、未知 `root` 引用、零超时/零重复次数、路径穿越 `cwd`、命令类型混用及非法通配符在加载阶段被拦截。
///
/// - **测试目标**：确保配置语义校验在 `config::load` 阶段完成，不将非法参数留到执行阶段。
/// - **场景步骤**：遍历 9 种典型的非法 `[[step]]` 配置片段并调用 `config::load`。
/// - **预期结果**：所有用例均返回包含 `"plan.toml:entry 1"` 的校验错误。
#[test]
fn invalid_fields_and_missing_required_values_are_rejected() {
    let tree = Tree::new();
    let base = tree.roots();
    for tail in [
        "[[step]]\nkind='exec'\nroot='a'\nprogram='/x'",
        "[[step]]\nkind='exec'\nid='x'\nroot='unknown'\nprogram='/x'",
        "[[step]]\nkind='exec'\nid='x'\nroot='a'\nprogram='/x'\nrepeat=0",
        "[[step]]\nkind='exec'\nid='x'\nroot='a'\nprogram='/x'\ntimeout_sec=0",
        "[[step]]\nkind='exec'\nid='x'\nroot='a'\nprogram='/x'\ncwd='/../escape'",
        "[[step]]\nkind='exec'\nid='x'\nroot='a'\nprogram='/x'\ncommand='wrong kind'",
        "[[step]]\nkind='ltp'\nroots=['a']\nsuite='s'\ninclude=['x']",
        "[[step]]\nkind='basic'\nroots=['a']\ninclude=['[bad']",
        "[[step]]\nkind='libctest'\nroots=['a']\nvariant='guessed'\ninclude=['x']",
    ] {
        tree.put("plan.toml", &format!("{base}\n{tail}\n"));
        let error = config::load(&tree.path("plan.toml"), &[]).unwrap_err();
        assert!(error.contains("plan.toml:entry 1"), "{error}");
    }
}

/// 验证基于目录扫描的测试步骤（`source = 'executables'` 与 `kind = 'basic'`）仅选取具有可执行权限的普通文件并按字典序排列。
///
/// - **测试目标**：确认目录扫描自动跳过不可执行的数据文件与子目录，且 `basic` 步骤默认将工作目录设为该可执行目录。
/// - **场景步骤**：在测试根目录下创建两个可执行脚本、一个普通文本文件和一个子目录，解析 `ltp`（`executables` 模式）与 `basic` 步骤。
/// - **预期结果**：仅 `alpha`、`zeta` 与 `mmap` 被选中并按字母序排列，`basic` 用例的 `cwd` 为 `/basic` 且协议为 `Protocol::Basic`。
#[test]
fn executable_source_and_basic_use_actual_root_directories() {
    let tree = Tree::new();
    tree.executable("root/cases/zeta");
    tree.executable("root/cases/alpha");
    tree.put("root/cases/data", "not executable");
    fs::create_dir_all(tree.path("root/cases/directory")).unwrap();
    tree.executable("root/basic/mmap");
    tree.put("plan.toml", &format!("{}\n[[step]]\nkind='ltp'\nsource='executables'\nroots=['a']\nsuite='elf'\ndirectory='/cases'\ninclude=['*']\n[[step]]\nkind='basic'\nroots=['a']\ndirectory='/basic'\ninclude=['mmap']\n", tree.roots()));
    let cases = tree.resolve("plan.toml").unwrap();
    assert_eq!(
        cases
            .iter()
            .map(|case| case.id.as_str())
            .collect::<Vec<_>>(),
        ["ltp:a:elf:alpha", "ltp:a:elf:zeta", "basic:a:mmap"]
    );
    assert_eq!(cases[2].cwd, Path::new("/basic"));
    assert_eq!(
        cases[2].command,
        CommandSpec::Exec {
            program: "/basic/mmap".into(),
            args: vec![]
        }
    );
    assert_eq!(
        cases[2].protocol,
        Protocol::Basic {
            name: "mmap".into()
        }
    );
    tree.put("plan.toml", &format!("{}\n[[step]]\nkind='ltp'\nroots=['a']\nsuite='missing'\ncatalog='/runtest-missing'\ninclude=['*']\n", tree.roots()));
    let error = tree.resolve("plan.toml").unwrap_err();
    assert!(error.contains("runtest-missing") && error.contains("No such file"));
}

/// 验证 `libctest` 步骤严格解析 `./runtest.exe -w entry-<variant>.exe <SELECTOR>` 格式的清单行，并拒绝变体名称不匹配的条目。
///
/// - **测试目标**：确保 `libctest` 清单解析器按行序提取测试选择器（Selector），保留原始参数并校验 `entry-<variant>.exe` 一致性。
/// - **场景步骤**：构造 `run-dynamic.sh` 与 `run-static.sh` 清单并解析；随后将 `run-dynamic.sh` 中的入口程序篡改为 `entry-static.exe` 再次解析。
/// - **预期结果**：合法清单按序生成 `Pre2025Libctest` 协议用例；篡改后的清单在报告第 1 行位置后返回解析错误。
#[test]
fn fixed_libctest_preserves_variant_selector_and_original_argv() {
    let tree = Tree::new();
    tree.put("root/run-dynamic.sh", "#!/bin/sh\n./runtest.exe -w entry-dynamic.exe env\n./runtest.exe -w entry-dynamic.exe argv\n");
    tree.put(
        "root/run-static.sh",
        "./runtest.exe -w entry-static.exe argv\n",
    );
    tree.put("plan.toml", &format!("{}\n[[step]]\nkind='libctest'\nroots=['b']\nvariant='dynamic'\ncatalog='/run-dynamic.sh'\ninclude=['*']\n[[step]]\nkind='libctest'\nroots=['a']\nvariant='static'\ncatalog='/run-static.sh'\ninclude=['argv']\n", tree.roots()));
    let cases = tree.resolve("plan.toml").unwrap();
    assert_eq!(
        cases
            .iter()
            .map(|case| case.id.as_str())
            .collect::<Vec<_>>(),
        [
            "libctest:b:dynamic:env",
            "libctest:b:dynamic:argv",
            "libctest:a:static:argv"
        ]
    );
    assert_eq!(
        cases[1].command,
        CommandSpec::Exec {
            program: "./runtest.exe".into(),
            args: vec!["-w".into(), "entry-dynamic.exe".into(), "argv".into()]
        }
    );
    assert_eq!(
        cases[1].protocol,
        Protocol::Pre2025Libctest {
            variant: "dynamic".into(),
            selector: "argv".into()
        }
    );
    assert_eq!(cases[1].source.line, Some(3));
    tree.put(
        "root/run-dynamic.sh",
        "./runtest.exe -w entry-static.exe argv\n",
    );
    assert!(tree
        .resolve("plan.toml")
        .unwrap_err()
        .contains("run-dynamic.sh:1"));
}

/// 验证包含 1000 个条目的大型测试清单能够保序展开，且单条精确选择后仅执行目标命令。
///
/// - **测试目标**：检验清单解析器在千级条目规模下的顺序稳定性与精确筛选隔离性。
/// - **场景步骤**：生成含 1000 条命令的清单，先用通配符全选验证数量与首尾 ID，再精确选择 `case0591` 并通过 Shell 实际运行该条命令。
/// - **预期结果**：全选阶段不产生任何执行副作用文件；单选并运行后仅生成 `marker-0591` 一个标记文件。
#[test]
fn thousand_entry_catalog_and_single_selected_execution() {
    let tree = Tree::new();
    let mut catalog = String::new();
    for index in 0..1000 {
        writeln!(
            catalog,
            "case{index:04} printf 'case{index:04}'; : > marker-{index:04}"
        )
        .unwrap();
    }
    tree.put("root/catalog", &catalog);
    let plan = |include: &str| {
        format!("{}\n[[step]]\nkind='ltp'\nroots=['a']\nsuite='large'\ncatalog='/catalog'\nshell='/bin/sh'\ninclude=[{include:?}]\n", tree.roots())
    };
    tree.put("plan.toml", &plan("case[0-9]*"));
    let all = tree.resolve("plan.toml").unwrap();
    assert_eq!(all.len(), 1000);
    assert_eq!(all[0].id, "ltp:a:large:case0000");
    assert_eq!(all[999].id, "ltp:a:large:case0999");
    assert!(!tree.path("root/marker-0000").exists());
    tree.put("plan.toml", &plan("case0591"));
    let cases = tree.resolve("plan.toml").unwrap();
    assert_eq!(cases.len(), 1);
    let CommandSpec::Shell { shell, command } = &cases[0].command else {
        panic!("not shell");
    };
    let output = Command::new(shell)
        .args(["-c", command])
        .current_dir(tree.path("root"))
        .output()
        .unwrap();
    assert!(output.status.success());
    assert_eq!(output.stdout, b"case0591");
    let markers: Vec<_> = fs::read_dir(tree.path("root"))
        .unwrap()
        .map(|entry| entry.unwrap().file_name())
        .filter(|name| name.to_str().unwrap().starts_with("marker-"))
        .collect();
    assert_eq!(markers, vec![std::ffi::OsString::from("marker-0591")]);
}

/// 验证通过 `[[override]]` 修改 `root` 字段可消解两个步骤在不同 `root` 下定义相同 ID 的冲突，并自动继承新 `root` 的默认工作目录与环境变量。
///
/// - **测试目标**：确认 `override` 中的 `root` 覆盖在构造 `ResolvedCase` 的环境与默认 `cwd` 之前生效。
/// - **场景步骤**：在两个子文件中定义同名步骤 `same`（分别指向 `a` 和 `b`），通过 `override` 将 `same` 的 `root` 统一指定为 `b`；随后测试指向未声明 `root` 的覆盖。
/// - **预期结果**：`same` 的最终 `root` 为 `b`、`cwd` 为 `/second`、环境变量仅包含 `roots.b` 的 `VIEW=b`（不含 `roots.a` 的 `OLD`）；未声明的 `root` 触发报错。
#[test]
fn root_override_resolves_duplicate_root_semantics_without_reselection() {
    let tree = Tree::new();
    tree.put("plan.toml", &format!("files=['one.toml','two.toml']\noverride_files=['override.toml']\n[roots.a]\npath={:?}\ncwd='/first'\nenv={{VIEW='a',OLD='only-a'}}\n[roots.b]\npath={:?}\ncwd='/second'\nenv={{VIEW='b'}}\n", tree.path("a"), tree.path("b")));
    let one = "[[step]]\nkind='exec'\nid='same'\nroot='a'\nprogram='/bin/true'\n";
    tree.put("one.toml", one);
    tree.put("two.toml", &one.replace("root='a'", "root='b'"));
    tree.put("override.toml", "[[override]]\nid='same'\nroot='b'\n");
    let cases = tree.resolve("plan.toml").unwrap();
    assert_eq!(cases.len(), 1);
    assert_eq!(cases[0].id, "same");
    assert_eq!(cases[0].root, "b");
    assert_eq!(cases[0].cwd, Path::new("/second"));
    assert_eq!(cases[0].env["VIEW"], "b");
    assert!(!cases[0].env.contains_key("OLD"));
    tree.put(
        "override.toml",
        "[[override]]\nid='same'\nroot='undeclared'\n",
    );
    assert!(tree
        .resolve("plan.toml")
        .unwrap_err()
        .contains("unknown root"));
}

/// 验证 LTP 步骤的 `source = 'executables'` 与 `source = 'runtest'` 严格区分 `directory` 与 `catalog` 字段，即便步骤被禁用也不允许混用。
///
/// - **测试目标**：确保配置 schema 校验在步骤禁用时依然生效，防止 `directory` 与 `catalog` 混填或同时填写。
/// - **场景步骤**：在 `enabled = false` 的 LTP 步骤中遍历 4 种 `source`/`directory`/`catalog` 的非法组合并调用 `config::load`；最后测试合法的禁用配置。
/// - **预期结果**：4 种矛盾组合均在加载阶段报错；合法的禁用步骤不访问缺失目录并正常返回其余启用步骤。
#[test]
fn directory_and_catalog_are_not_implicit_aliases() {
    let tree = Tree::new();
    for fields in [
        "source='executables'\ncatalog='/cases'",
        "source='executables'\ndirectory='/cases'\ncatalog='/catalog'",
        "source='runtest'\ndirectory='/cases'",
        "source='runtest'\ncatalog='/catalog'\ndirectory='/cases'",
    ] {
        tree.put("plan.toml", &format!("{}\n[[step]]\nenabled=false\nkind='ltp'\nroots=['a']\nsuite='s'\ninclude=['*']\n{fields}\n", tree.roots()));
        assert!(config::load(&tree.path("plan.toml"), &[])
            .unwrap_err()
            .contains("entry 1"));
    }
    tree.put("plan.toml", &format!("{}\n[[step]]\nenabled=false\nkind='ltp'\nsource='executables'\nroots=['a']\nsuite='s'\ndirectory='/missing'\ninclude=['*']\n[[step]]\nkind='exec'\nid='active'\nroot='a'\nprogram='/bin/true'\n", tree.roots()));
    assert_eq!(tree.resolve("plan.toml").unwrap()[0].id, "active");
}

/// 验证 `basic` 类型测试在被 `[[override]]` 覆盖 `program` 路径后，其结果判定协议（`Protocol::Basic`）仍保留原始测试名称。
///
/// - **测试目标**：确保输出协议中的期望测试名与用例初始选择名绑定，不因执行程序路径被覆盖而改变。
/// - **场景步骤**：选择 `basic:unusual:mkdir_` 并通过 `override` 将其 `program` 替换为 `/other/program`。
/// - **预期结果**：生成的用例命令变为 `/other/program`，但 `protocol` 仍为 `Protocol::Basic { name: "mkdir_" }`。
#[test]
fn basic_protocol_keeps_original_name_after_command_override() {
    let tree = Tree::new();
    tree.executable("root/basic/mkdir_");
    tree.put("plan.toml", &format!("override_files=['override.toml']\n[roots.unusual]\npath={:?}\n[[step]]\nkind='basic'\nroots=['unusual']\ndirectory='/basic'\ninclude=['mkdir_']\n", tree.path("root")));
    tree.put(
        "override.toml",
        "[[override]]\nid='basic:unusual:mkdir_'\nprogram='/other/program'\n",
    );
    let cases = tree.resolve("plan.toml").unwrap();
    assert_eq!(
        cases[0].protocol,
        Protocol::Basic {
            name: "mkdir_".into()
        }
    );
    assert_eq!(
        cases[0].command,
        CommandSpec::Exec {
            program: "/other/program".into(),
            args: vec![]
        }
    );
}

/// 验证可执行目录下的相对符号链接（Symlink）在指向有效可执行文件时可被正常选中，且用例 ID 与执行路径保留符号链接别名。
///
/// - **测试目标**：确认目录扫描跟随符号链接检查目标文件可执行权限，同时保留链接本身的名称与路径供子进程调用。
/// - **场景步骤**：在 `root/cases/alias` 创建指向 `../bin/probe` 可执行文件的相对符号链接，并在 LTP `executables` 模式下选择 `alias`。
/// - **预期结果**：生成 ID 为 `ltp:a:elf:alias`，执行程序路径为 `/cases/alias`。
#[test]
fn executable_relative_symlink_preserves_alias_program_and_id() {
    let tree = Tree::new();
    tree.executable("root/bin/probe");
    fs::create_dir_all(tree.path("root/cases")).unwrap();
    symlink("../bin/probe", tree.path("root/cases/alias")).unwrap();
    tree.put("plan.toml", &format!("{}\n[[step]]\nkind='ltp'\nsource='executables'\nroots=['a']\nsuite='elf'\ndirectory='/cases'\ninclude=['alias']\n", tree.roots()));
    let cases = tree.resolve("plan.toml").unwrap();
    assert_eq!(cases.len(), 1);
    assert_eq!(cases[0].id, "ltp:a:elf:alias");
    assert_eq!(
        cases[0].command,
        CommandSpec::Exec {
            program: "/cases/alias".into(),
            args: vec![]
        }
    );
    assert_eq!(
        cases[0].source.catalog.as_deref(),
        Some(Path::new("/cases"))
    );
}

/// 验证指向目标根目录内部路径的绝对符号链接清单（如 `/catalog -> /sources/catalog`）会相对于目标根目录解析，而非穿透到宿主机根目录。
///
/// - **测试目标**：确保 `rooted_path` 在遇到绝对符号链接时将其重定根到当前选择的 `root` 下，同时 `source.catalog` 保留配置中的原始路径。
/// - **场景步骤**：在 `root/catalog` 创建指向 `"/sources/catalog"` 的绝对符号链接，解析引用 `/catalog` 的 LTP 步骤。
/// - **预期结果**：成功从 `root/sources/catalog` 读取到命令内容，且 `source.catalog` 仍记录为 `/catalog`。
#[test]
fn absolute_catalog_symlink_reads_from_selected_root() {
    let tree = Tree::new();
    tree.put("root/sources/catalog", "probe printf 'root catalog'\n");
    symlink("/sources/catalog", tree.path("root/catalog")).unwrap();
    tree.put("plan.toml", &format!("{}\n[[step]]\nkind='ltp'\nroots=['a']\nsuite='linked'\ncatalog='/catalog'\ninclude=['probe']\n", tree.roots()));
    let cases = tree.resolve("plan.toml").unwrap();
    assert_eq!(cases.len(), 1);
    assert_eq!(cases[0].id, "ltp:a:linked:probe");
    assert_eq!(
        cases[0].command,
        CommandSpec::Shell {
            shell: "/bin/sh".into(),
            command: "printf 'root catalog'".into()
        }
    );
    assert_eq!(
        cases[0].source.catalog.as_deref(),
        Some(Path::new("/catalog"))
    );
    assert_eq!(cases[0].source.line, Some(1));
}

/// 验证包含中间目录绝对符号链接的路径（如 `/linked/cases` 与 `/linked/catalog`，其中 `/linked -> /sources`）在目标根内正确解析且不改写执行路径。
///
/// - **测试目标**：确认路径中间组件为绝对符号链接时，无论是目录扫描还是文本清单读取都能限制在目标根目录内完成。
/// - **场景步骤**：将 `root/linked` 设为指向 `"/sources"` 的绝对符号链接，分别通过 `/linked/cases` 和 `/linked/catalog` 选择测试。
/// - **预期结果**：两个步骤均成功解析，且可执行路径与 `source.catalog` 均保留未展开前的 `/linked/...` 原始路径。
#[test]
fn intermediate_directory_symlink_sources_read_from_selected_root() {
    let tree = Tree::new();
    tree.executable("root/sources/cases/probe");
    tree.put("root/sources/catalog", "probe printf 'root catalog'\n");
    symlink("/sources", tree.path("root/linked")).unwrap();
    tree.put("plan.toml", &format!("{}\n[[step]]\nkind='ltp'\nsource='executables'\nroots=['a']\nsuite='elf'\ndirectory='/linked/cases'\ninclude=['probe']\n[[step]]\nkind='ltp'\nroots=['a']\nsuite='text'\ncatalog='/linked/catalog'\ninclude=['probe']\n", tree.roots()));
    let cases = tree.resolve("plan.toml").unwrap();
    assert_eq!(
        cases
            .iter()
            .map(|case| case.id.as_str())
            .collect::<Vec<_>>(),
        ["ltp:a:elf:probe", "ltp:a:text:probe"]
    );
    assert_eq!(
        cases[0].command,
        CommandSpec::Exec {
            program: "/linked/cases/probe".into(),
            args: vec![]
        }
    );
    assert_eq!(
        cases[0].source.catalog.as_deref(),
        Some(Path::new("/linked/cases"))
    );
    assert_eq!(
        cases[1].command,
        CommandSpec::Shell {
            shell: "/bin/sh".into(),
            command: "printf 'root catalog'".into()
        }
    );
    assert_eq!(
        cases[1].source.catalog.as_deref(),
        Some(Path::new("/linked/catalog"))
    );
}

/// 验证 `libctest` 步骤未显式指定 `cwd` 时，自动继承所属 `root` 配置的默认工作目录（如 `cwd = '/libc-test'`），使 `./runtest.exe` 相对路径得以正确定位。
///
/// - **测试目标**：确保 `libctest` 条目自身不强制覆盖 `cwd`，从而允许通过 `root.cwd` 或 `step.cwd` 指定 `runtest.exe` 所在的实际目录。
/// - **场景步骤**：在 `[roots.a]` 中设置 `cwd = '/libc-test'`，并在 `root/libc-test/run-dynamic.sh` 中放置相对路径命令，解析该步骤。
/// - **预期结果**：生成的用例 `cwd` 为 `/libc-test`，命令保持 `./runtest.exe -w entry-dynamic.exe argv`。
#[test]
fn libctest_inherits_explicit_root_cwd_and_original_command() {
    let tree = Tree::new();
    tree.put(
        "root/libc-test/run-dynamic.sh",
        "./runtest.exe -w entry-dynamic.exe argv\n",
    );
    tree.put("plan.toml", &format!("[roots.a]\npath={:?}\ncwd='/libc-test'\n[[step]]\nkind='libctest'\nroots=['a']\nvariant='dynamic'\ncatalog='/libc-test/run-dynamic.sh'\ninclude=['argv']\n", tree.path("root")));
    let cases = tree.resolve("plan.toml").unwrap();
    assert_eq!(cases.len(), 1);
    assert_eq!(cases[0].id, "libctest:a:dynamic:argv");
    assert_eq!(cases[0].cwd, Path::new("/libc-test"));
    assert_eq!(
        cases[0].command,
        CommandSpec::Exec {
            program: "./runtest.exe".into(),
            args: vec!["-w".into(), "entry-dynamic.exe".into(), "argv".into()]
        }
    );
    assert_eq!(
        cases[0].protocol,
        Protocol::Pre2025Libctest {
            variant: "dynamic".into(),
            selector: "argv".into()
        }
    );
}
