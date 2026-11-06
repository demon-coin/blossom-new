//! 测试计划（Config）解析、目录/清单扫描与最终测试用例列表（`ResolvedCase`）展开模块。
//!
//! # 模块职责与架构定位
//! 本模块位于配置加载器（`config`）与运行执行器（`execute`）之间，负责将声明式的 TOML 测试计划展开为具有确定性顺序、稳定唯一 ID 和完整运行参数的 [`ResolvedCase`] 序列。
//! - **上游**：接收 `config::load` 解析出的 [`Config`] 与目标硬件架构名称（`"riscv64"` 或 `"loongarch64"`）。
//! - **下游**：向 `execute` 或 `--list` 模式输出展开后的 [`ResolvedCase`] 列表；文件系统的受限根路径读取委托给 [`crate::runtime::rooted_path`]。
//!
//! # 核心机制与选择流水线
//! ```text
//! Config (defaults + roots + steps + overrides)
//!   │
//!   ├─ 1. 预校验 (validate_overrides)
//!   │     └─ 检查相同 ID 的多条 override 是否在同一字段/环境变量上存在冲突值
//!   │
//!   ├─ 2. 按声明顺序遍历 Step
//!   │     ├─ 过滤：跳过 enabled == false 或 arch 不匹配的 step
//!   │     ├─ 直接命令 (Exec / Shell) ──► 构造单个 Entry，合并配置并应用 override
//!   │     └─ 目录/清单扫描 (Ltp / Basic / Libctest)
//!   │           │
//!   │           ├─ 按 step.roots 顺序遍历每个目标根目录
//!   │           ├─ 扫描可执行目录 (executables) 或解析文本清单 (text_catalog)
//!   │           ├─ 按 step.include 模式顺序匹配条目，再剔除匹配 step.exclude 的条目
//!   │           └─ 生成稳定 ID -> 合并 defaults/root/step -> 应用 override -> 去重/冲突检查
//!   │
//!   └─ 3. 后置完整性检查
//!         ├─ 确保每条 override 的目标 ID 都至少命中了一个已选中的测试用例
//!         └─ 确保最终选中的测试用例集合非空
//! ```
//!
//! ## 配置与环境变量合并优先级（由低到高）
//! ```text
//! [defaults]  ──►  [roots.<name>]  ──►  [[step]]  ──►  [[override]] (按 ID 精确匹配)
//! ```
//! - 环境变量（`env`）按字典键逐层覆盖合并；
//! - 命令参数（`args`）、前置/后置步骤（`setup`/`cleanup`）等数组字段采用整字段替换。
//!
//! # 锁获取顺序与并发契约
//! 本模块为纯单线程配置展开与只读文件系统元数据/清单读取逻辑，不涉及任何并发锁或子进程创建。

use crate::model::{
    CaseOverride, CatalogSource, CommandSpec, Config, Protocol, ResolvedCase, RootSpec,
    SourceLocation, Step, StepKind,
};
use crate::runtime::rooted_path;
use glob::Pattern;
use std::collections::BTreeMap;
use std::fs;
use std::os::unix::fs::PermissionsExt;
use std::path::{Path, PathBuf};

/// 从测试步骤或清单文件中提取出的单个候选测试条目。
struct Entry {
    /// 条目在清单或目录中的短名称（用于匹配 `include`/`exclude` 及拼接用例 ID）。
    name: String,
    /// 待执行的命令规格（直接执行 `Exec` 或通过 Shell 执行 `Shell`）。
    command: CommandSpec,
    /// 该条目在配置文件及清单文件中的定义位置（用于错误诊断与报告溯源）。
    source: SourceLocation,
    /// 条目自带的默认工作目录（例如 `basic` 类型默认在其所在目录执行）。
    cwd: Option<PathBuf>,
    /// 输出解析与结果判定协议（`Plain`、`Ltp`、`Basic` 或 `Pre2025Libctest`）。
    protocol: Protocol,
}

/// 根据当前架构将配置展开为按声明顺序排列的测试用例列表。
///
/// # 参数
/// - `config`：已加载并完成语法校验的测试计划配置。
/// - `arch`：当前目标架构名称（仅支持 `"riscv64"` 或 `"loongarch64"`）。
///
/// # 返回值
/// 成功时返回去重且应用完所有覆盖规则的 [`ResolvedCase`] 列表；若架构未知、模式未命中任何条目、存在未命中的 `override` 或重复 ID 语义冲突，则返回包含具体源文件位置的错误字符串。
pub fn resolve(config: &Config, arch: &str) -> Result<Vec<ResolvedCase>, String> {
    // Step 1: 校验目标架构名称与全局 override 规则自身的字段一致性。
    if !matches!(arch, "riscv64" | "loongarch64") {
        return Err(format!(
            "{}: unknown architecture {arch:?}",
            config.plan.display()
        ));
    }
    validate_overrides(&config.overrides)?;
    let mut cases = Vec::new();
    let mut selected = BTreeMap::<String, usize>::new();
    // Step 2: 按配置文件与 step 的声明顺序遍历所有启用的测试步骤。
    for step in &config.steps {
        if !step.enabled || (!step.arch.is_empty() && !step.arch.iter().any(|value| value == arch))
        {
            continue;
        }
        if matches!(step.kind, StepKind::Exec | StepKind::Shell) {
            let root_name = step.root.as_ref().unwrap();
            let entry = direct_entry(step);
            let case = resolved(config, step, root_name, step.id.clone().unwrap(), &entry);
            insert(case, config, &mut selected, &mut cases)?;
            continue;
        }
        let inclusions: Vec<Pattern> = step
            .include
            .iter()
            .map(|text| Pattern::new(text).unwrap())
            .collect();
        let exclusions: Vec<Pattern> = step
            .exclude
            .iter()
            .map(|text| Pattern::new(text).unwrap())
            .collect();
        for root_name in &step.roots {
            let entries = catalog(step, &config.roots[root_name])?;
            let mut count = 0;
            for (include, pattern) in step.include.iter().zip(&inclusions) {
                let mut matched = false;
                for entry in entries.iter().filter(|entry| pattern.matches(&entry.name)) {
                    matched = true;
                    if exclusions
                        .iter()
                        .any(|pattern| pattern.matches(&entry.name))
                    {
                        continue;
                    }
                    let id = case_id(step, root_name, &entry.name);
                    let case = resolved(config, step, root_name, id, entry);
                    insert(case, config, &mut selected, &mut cases)?;
                    count += 1;
                }
                if !matched {
                    let kind = if include.contains(['*', '?', '[']) {
                        "pattern"
                    } else {
                        "name"
                    };
                    return Err(format!(
                        "{}: root {root_name:?}: {kind} {include:?} matched no entries in {}",
                        step.location.as_ref().unwrap(),
                        catalog_path(step).display()
                    ));
                }
            }
            if count == 0 {
                return Err(format!(
                    "{}: root {root_name:?}: empty selection after exclude in {}",
                    step.location.as_ref().unwrap(),
                    catalog_path(step).display()
                ));
            }
        }
    }
    // Step 3: 确认所有声明的 override 均命中了实际被选中的用例，且最终用例列表非空。
    for item in &config.overrides {
        if !selected.contains_key(&item.id) {
            return Err(format!(
                "{}: override ID {:?} is not selected",
                item.location.as_ref().unwrap(),
                item.id
            ));
        }
    }
    if cases.is_empty() {
        return Err(format!(
            "{}: empty selection for architecture {arch}",
            config.plan.display()
        ));
    }
    Ok(cases)
}

/// 将 `Exec` 或 `Shell` 类型的直接命令步骤转换为候选 [`Entry`]。
fn direct_entry(step: &Step) -> Entry {
    let command = match step.kind {
        StepKind::Exec => CommandSpec::Exec {
            program: step.program.clone().unwrap(),
            args: step.args.clone().unwrap_or_default(),
        },
        StepKind::Shell => CommandSpec::Shell {
            shell: step.shell.clone().unwrap(),
            command: step.command.clone().unwrap(),
        },
        _ => unreachable!(),
    };
    Entry {
        name: step.id.clone().unwrap(),
        command,
        source: step.location.clone().unwrap(),
        cwd: None,
        protocol: Protocol::Plain,
    }
}

/// 返回目录/清单类测试步骤在目标根目录内的清单文件或可执行文件目录路径。
fn catalog_path(step: &Step) -> PathBuf {
    match step.kind {
        StepKind::Ltp if step.source.unwrap_or_default() == CatalogSource::Executables => {
            step.directory.clone().unwrap()
        }
        StepKind::Basic => step
            .directory
            .clone()
            .unwrap_or_else(|| PathBuf::from("/basic")),
        StepKind::Ltp => step.catalog.clone().unwrap(),
        StepKind::Libctest => step.catalog.clone().unwrap_or_else(|| {
            PathBuf::from(format!("/run-{}.sh", step.variant.as_ref().unwrap()))
        }),
        _ => unreachable!(),
    }
}

/// 根据步骤类型（`Basic`、`Ltp`、`Libctest`）和数据源模式读取目标根目录下的候选条目列表。
fn catalog(step: &Step, root: &RootSpec) -> Result<Vec<Entry>, String> {
    let path = catalog_path(step);
    match step.kind {
        StepKind::Basic => executables(step, root, &path),
        StepKind::Ltp if step.source.unwrap_or_default() == CatalogSource::Executables => {
            executables(step, root, &path)
        }
        StepKind::Ltp | StepKind::Libctest => text_catalog(step, root, &path),
        _ => unreachable!(),
    }
}

/// 扫描目标根目录下指定路径中的所有普通可执行文件，并按文件名升序排列返回候选条目。
fn executables(step: &Step, root: &RootSpec, path: &Path) -> Result<Vec<Entry>, String> {
    let at = step.location.as_ref().unwrap();
    let directory = rooted_path(&root.path, Path::new("/"), path)
        .and_then(fs::read_dir)
        .map_err(|error| {
            format!(
                "{at}: root {}: {}: {error}",
                root.path.display(),
                path.display()
            )
        })?;
    let mut entries = Vec::new();
    for item in directory {
        let item = item.map_err(|error| {
            format!(
                "{at}: root {}: {}: {error}",
                root.path.display(),
                path.display()
            )
        })?;
        let entry_path = path.join(item.file_name());
        let metadata = rooted_path(&root.path, Path::new("/"), &entry_path)
            .and_then(fs::metadata)
            .map_err(|error| {
                format!(
                    "{at}: root {}: {}: {error}",
                    root.path.display(),
                    entry_path.display()
                )
            })?;
        if !metadata.is_file() || metadata.permissions().mode() & 0o111 == 0 {
            continue;
        }
        let name = item.file_name().into_string().map_err(|_| {
            format!(
                "{at}: non-UTF-8 executable name in root {}: {}",
                root.path.display(),
                path.display()
            )
        })?;
        catalog_name(&name, at)?;
        let mut source = at.clone();
        source.catalog = Some(path.to_path_buf());
        let protocol = if step.kind == StepKind::Basic {
            Protocol::Basic { name: name.clone() }
        } else {
            Protocol::Ltp
        };
        entries.push(Entry {
            command: CommandSpec::Exec {
                program: entry_path.to_str().unwrap().to_owned(),
                args: Vec::new(),
            },
            name,
            source,
            cwd: if step.kind == StepKind::Basic {
                Some(path.to_path_buf())
            } else {
                None
            },
            protocol,
        });
    }
    entries.sort_by(|left, right| left.name.cmp(&right.name));
    Ok(entries)
}

/// 读取并解析目标根目录下的文本格式测试清单（LTP `runtest` 文件或 `libctest` 启动脚本）。
fn text_catalog(step: &Step, root: &RootSpec, path: &Path) -> Result<Vec<Entry>, String> {
    let at = step.location.as_ref().unwrap();
    let text = rooted_path(&root.path, Path::new("/"), path)
        .and_then(fs::read_to_string)
        .map_err(|error| {
            format!(
                "{at}: root {}: {}: {error}",
                root.path.display(),
                path.display()
            )
        })?;
    let mut entries = Vec::new();
    for (index, line) in text.lines().enumerate() {
        let line = line.trim_start();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let mut source = at.clone();
        source.catalog = Some(path.to_path_buf());
        source.line = Some(index + 1);
        let entry = if step.kind == StepKind::Ltp {
            let end = line
                .find(char::is_whitespace)
                .ok_or_else(|| format!("{source}: runtest tag has no command"))?;
            let name = &line[..end];
            catalog_name(name, &source)?;
            let command = line[end..].trim_start();
            if command.is_empty() || command.contains('\0') {
                return Err(format!("{source}: empty or NUL-containing runtest command"));
            }
            Entry {
                name: name.to_owned(),
                command: CommandSpec::Shell {
                    shell: step.shell.clone().unwrap_or_else(|| "/bin/sh".to_owned()),
                    command: command.to_owned(),
                },
                source,
                cwd: None,
                protocol: Protocol::Ltp,
            }
        } else {
            libctest_entry(step.variant.as_ref().unwrap(), line, source)?
        };
        entries.push(entry);
    }
    Ok(entries)
}

/// 解析单行 `libctest` 启动命令（固定格式为 `./runtest.exe -w entry-<variant>.exe <SELECTOR>`）。
fn libctest_entry(variant: &str, line: &str, source: SourceLocation) -> Result<Entry, String> {
    let words: Vec<&str> = line.split_whitespace().collect();
    let entry = format!("entry-{variant}.exe");
    if words.len() != 4 || words[0] != "./runtest.exe" || words[1] != "-w" || words[2] != entry {
        return Err(format!(
            "{source}: expected ./runtest.exe -w {entry} SELECTOR"
        ));
    }
    let selector = words[3];
    catalog_name(selector, &source)?;
    Ok(Entry {
        name: selector.to_owned(),
        source,
        cwd: None,
        command: CommandSpec::Exec {
            program: words[0].to_owned(),
            args: words[1..].iter().map(|word| (*word).to_owned()).collect(),
        },
        protocol: Protocol::Pre2025Libctest {
            variant: variant.to_owned(),
            selector: selector.to_owned(),
        },
    })
}

/// 校验目录或清单中的测试条目名称是否合法（非空、不含空白字符、不含 `:` 与 NUL 字节）。
fn catalog_name(name: &str, at: &SourceLocation) -> Result<(), String> {
    if name.is_empty() || name.contains([':', '\0']) || name.chars().any(char::is_whitespace) {
        return Err(format!("{at}: invalid catalog name {name:?}"));
    }
    Ok(())
}

/// 根据测试类型、根目录名、套件/变体名及条目名拼接全局稳定的测试用例 ID。
fn case_id(step: &Step, root: &str, name: &str) -> String {
    match step.kind {
        StepKind::Ltp => format!("ltp:{root}:{}:{name}", step.suite.as_ref().unwrap()),
        StepKind::Basic => format!("basic:{root}:{name}"),
        StepKind::Libctest => format!("libctest:{root}:{}:{name}", step.variant.as_ref().unwrap()),
        _ => unreachable!(),
    }
}

/// 合并全局默认配置（`defaults`）、目标根目录配置（`root`）与当前步骤配置（`step`），构造初始 [`ResolvedCase`]。
fn resolved(
    config: &Config,
    step: &Step,
    root_name: &str,
    id: String,
    entry: &Entry,
) -> ResolvedCase {
    let root_name = config
        .overrides
        .iter()
        .filter(|item| item.id == id)
        .find_map(|item| item.root.as_deref())
        .unwrap_or(root_name);
    let root = &config.roots[root_name];
    let defaults = &config.defaults;
    let mut env = defaults.env.clone();
    env.extend(root.env.clone());
    env.extend(step.env.clone());
    ResolvedCase {
        id,
        source: entry.source.clone(),
        root: root_name.to_owned(),
        cwd: step
            .cwd
            .clone()
            .or_else(|| entry.cwd.clone())
            .unwrap_or_else(|| root.cwd.clone()),
        command: entry.command.clone(),
        env,
        timeout_sec: step.timeout_sec.unwrap_or(defaults.timeout_sec),
        repeat: step.repeat.unwrap_or(defaults.repeat),
        hooks: step.hooks.unwrap_or(defaults.hooks),
        setup: step.setup.clone().unwrap_or_else(|| defaults.setup.clone()),
        cleanup: step
            .cleanup
            .clone()
            .unwrap_or_else(|| defaults.cleanup.clone()),
        protocol: entry.protocol.clone(),
    }
}

/// 对测试用例应用匹配的 `override` 规则并插入结果集；若 ID 已存在则校验两次命中的运行语义是否完全一致。
fn insert(
    mut case: ResolvedCase,
    config: &Config,
    selected: &mut BTreeMap<String, usize>,
    cases: &mut Vec<ResolvedCase>,
) -> Result<(), String> {
    for item in &config.overrides {
        if item.id == case.id {
            apply_override(&mut case, item)?;
        }
    }
    if let Some(index) = selected.get(&case.id) {
        let first = &cases[*index];
        if !same_semantics(first, &case) {
            return Err(format!("{}: conflicting selection ID {:?}; first selected at {} (use an exact override or another ID)", case.source, case.id, first.source));
        }
    } else {
        selected.insert(case.id.clone(), cases.len());
        cases.push(case);
    }
    Ok(())
}

/// 判断两个同 ID 的 [`ResolvedCase`] 除定义来源位置（`source`）外的所有执行语义是否完全相同。
fn same_semantics(left: &ResolvedCase, right: &ResolvedCase) -> bool {
    left.root == right.root
        && left.cwd == right.cwd
        && left.command == right.command
        && left.env == right.env
        && left.timeout_sec == right.timeout_sec
        && left.repeat == right.repeat
        && left.hooks == right.hooks
        && left.setup == right.setup
        && left.cleanup == right.cleanup
        && left.protocol == right.protocol
}

/// 校验针对同一个用例 ID 的多条 `override` 规则之间是否存在同一字段或同一环境变量键的不同赋值冲突。
fn validate_overrides(items: &[CaseOverride]) -> Result<(), String> {
    let mut seen: BTreeMap<&str, Vec<&CaseOverride>> = BTreeMap::new();
    for item in items {
        for previous in seen.entry(&item.id).or_default().iter() {
            macro_rules! conflict {
                ($($field:ident),+) => { $(
                    if let (Some(left), Some(right)) = (&previous.$field, &item.$field) {
                        if left != right { return Err(format!("{}: conflicting override field {} for {:?}; previous at {}", item.location.as_ref().unwrap(), stringify!($field), item.id, previous.location.as_ref().unwrap())); }
                    }
                )+ };
            }
            conflict!(
                root,
                cwd,
                program,
                args,
                shell,
                command,
                timeout_sec,
                repeat,
                hooks,
                setup,
                cleanup
            );
            for (key, value) in &item.env {
                if previous.env.get(key).is_some_and(|old| old != value) {
                    return Err(format!(
                        "{}: conflicting override env {key:?} for {:?}; previous at {}",
                        item.location.as_ref().unwrap(),
                        item.id,
                        previous.location.as_ref().unwrap()
                    ));
                }
            }
        }
        seen.get_mut(item.id.as_str()).unwrap().push(item);
    }
    Ok(())
}

/// 将单条 [`CaseOverride`] 规则合并到目标 [`ResolvedCase`] 上，并拒绝跨命令类型（`Exec` 与 `Shell` 互改）的非法覆盖。
fn apply_override(case: &mut ResolvedCase, item: &CaseOverride) -> Result<(), String> {
    let at = item.location.as_ref().unwrap();
    match &mut case.command {
        CommandSpec::Exec { program, args } => {
            if item.shell.is_some() || item.command.is_some() {
                return Err(format!("{at}: shell override for exec ID {:?}", item.id));
            }
            if let Some(value) = &item.program {
                program.clone_from(value);
            }
            if let Some(value) = &item.args {
                args.clone_from(value);
            }
        }
        CommandSpec::Shell { shell, command } => {
            if item.program.is_some() || item.args.is_some() {
                return Err(format!("{at}: exec override for shell ID {:?}", item.id));
            }
            if let Some(value) = &item.shell {
                shell.clone_from(value);
            }
            if let Some(value) = &item.command {
                command.clone_from(value);
            }
        }
    }
    if let Some(value) = &item.root {
        case.root.clone_from(value);
    }
    if let Some(value) = &item.cwd {
        case.cwd.clone_from(value);
    }
    if let Some(value) = item.timeout_sec {
        case.timeout_sec = value;
    }
    if let Some(value) = item.repeat {
        case.repeat = value;
    }
    if let Some(value) = item.hooks {
        case.hooks = value;
    }
    if let Some(value) = &item.setup {
        case.setup.clone_from(value);
    }
    if let Some(value) = &item.cleanup {
        case.cleanup.clone_from(value);
    }
    case.env.extend(item.env.clone());
    Ok(())
}
