//! 测试计划（TOML）的单层文件组合与静态输入边界校验。
//!
//! ## 模块职责与架构定位
//! 本模块作为 `contest-run` 的配置加载前端，由 `main` 在挂载比赛磁盘或扫描目标根目录之前调用。
//! 它负责读取主计划文件（如 `/etc/contest/plan.toml`）及其引用的外部步骤文件（`files`）和覆盖文件（`override_files`），
//! 将它们组装为统一的 [`Config`] 结构，并对所有字段执行与文件系统内容无关的静态合法性检查。
//! 目标根目录内的清单文件是否存在、通配符是否命中具体测例，则留给下游 [`crate::selection`] 模块在挂载后展开验证。
//!
//! ## 计划组合与校验流程
//! ```text
//! plan.toml (主入口文件)
//!   ├── roots / defaults / finish
//!   ├── [[step]] (主文件内嵌步骤，记录 plan.toml:entry N)
//!   ├── files (或命令行 --file 覆盖列表，相对主计划所在目录解析)
//!   │     └── step_file.toml ──> 追加 [[step]] (保留原文件位置 step_file.toml:entry M)
//!   └── override_files (相对主计划所在目录解析)
//!         └── override_file.toml ──> 追加 [[override]] (保留原文件位置 override_file.toml:entry K)
//!                     │
//!                     ▼
//!             validate(&Config)
//!   ├── 1. 校验 defaults (timeout/repeat > 0, env 键值合法, setup/cleanup 非空 argv)
//!   ├── 2. 校验 roots (名称合法, 绝对路径且无 "..", 挂载项仅限无重复的 dev/proc/tmp)
//!   ├── 3. 校验 steps (按 StepKind 检查互斥字段、架构名、引用 root 与 glob 语法)
//!   └── 4. 校验 overrides (ID 合法, 引用 root 存在, 不混用 Exec 与 Shell 字段)
//! ```
//!
//! ## 并发与锁契约
//! 配置加载在进程启动初期单线程顺序执行，仅对配置文件做只读访问，不涉及共享锁。

use crate::model::{
    CaseOverride, Config, Defaults, Finish, RootSpec, SourceLocation, Step, StepKind,
};
use glob::Pattern;
use serde::de::DeserializeOwned;
use serde::Deserialize;
use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::{Component, Path, PathBuf};

/// 主计划文件（如 `plan.toml`）的 TOML 反序列化顶层结构。
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Plan {
    /// 附加步骤文件（`StepFile`）的相对或绝对路径列表；可被命令行 `--file` 参数整体替换。
    #[serde(default)]
    files: Vec<PathBuf>,
    /// 附加单测例覆盖文件（`OverrideFile`）的路径列表，相对于主计划文件所在目录解析。
    #[serde(default)]
    override_files: Vec<PathBuf>,
    /// 命名执行根目录（如 `glibc`、`musl`）的规格映射表。
    #[serde(default)]
    roots: BTreeMap<String, RootSpec>,
    /// 全局默认运行参数（超时、重复轮次、环境变量与前后置命令等）。
    #[serde(default)]
    defaults: Defaults,
    /// 全部测例运行结束后的收尾动作（关机或返回调用方）。
    #[serde(default)]
    finish: Finish,
    /// 主计划文件内直接声明的测试步骤列表。
    #[serde(default, rename = "step")]
    steps: Vec<Step>,
}

/// 外部步骤文件（由 `Plan::files` 或命令行 `--file` 引入）的 TOML 反序列化结构。
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct StepFile {
    /// 该步骤文件中声明的测试步骤（`[[step]]`）列表。
    #[serde(default, rename = "step")]
    steps: Vec<Step>,
}

/// 外部测例覆盖文件（由 `Plan::override_files` 引入）的 TOML 反序列化结构。
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct OverrideFile {
    /// 该覆盖文件中声明的单测例参数覆盖项（`[[override]]`）列表。
    #[serde(default, rename = "override")]
    overrides: Vec<CaseOverride>,
}

/// 加载主计划文件及其引用的步骤文件与覆盖文件，合并并校验后返回完整的 [`Config`]。
///
/// 调用上下文：在测试运行器启动初期、挂载外部文件系统之前调用。
/// 参数 `plan` 为主计划文件路径；`files_override` 为命令行 `--file` 传入的步骤文件列表（非空时替换主计划中的 `files` 字段，所有相对路径均相对于 `plan` 所在父目录解析）。
/// 返回值：解析且静态校验通过的 [`Config`]；若存在文件读取、TOML 语法或字段边界错误，返回包含具体文件与条目位置的错误字符串。
pub fn load(plan: &Path, files_override: &[PathBuf]) -> Result<Config, String> {
    // Step 1: 解析主计划文件，提取基础目录并初始化 Config 骨架。
    let input: Plan = decode(plan)?;
    let base = plan.parent().unwrap_or_else(|| Path::new("."));
    let mut config = Config {
        roots: input.roots,
        defaults: input.defaults,
        steps: located_steps(input.steps, plan),
        overrides: Vec::new(),
        finish: input.finish,
        plan: plan.to_path_buf(),
    };
    // Step 2: 确定生效的外部步骤文件列表（优先使用命令行覆盖列表），依次解析并追加步骤。
    let files = if files_override.is_empty() {
        &input.files
    } else {
        files_override
    };
    for file in files {
        let path = base.join(file);
        let input: StepFile = decode(&path)?;
        config.steps.extend(located_steps(input.steps, &path));
    }
    // Step 3: 依次解析所有外部覆盖文件（override_files），记录每个覆盖条目在原文件中的位置。
    for file in input.override_files {
        let path = base.join(file);
        let input: OverrideFile = decode(&path)?;
        config
            .overrides
            .extend(
                input
                    .overrides
                    .into_iter()
                    .enumerate()
                    .map(|(index, mut item)| {
                        item.location = Some(location(&path, index));
                        item
                    }),
            );
    }
    // Step 4: 对合并后的完整配置执行静态边界与字段互斥校验。
    validate(&config)?;
    Ok(config)
}

/// 读取指定路径的 UTF-8 文本并反序列化为目标 TOML 结构。
///
/// 若文件读取失败或 TOML 语法/字段不匹配，将文件路径前缀附加到错误信息中返回。
fn decode<T: DeserializeOwned>(path: &Path) -> Result<T, String> {
    let text = fs::read_to_string(path).map_err(|error| format!("{}: {error}", path.display()))?;
    toml::from_str(&text).map_err(|error| format!("{}: {error}", path.display()))
}

/// 为步骤列表中的每个 [`Step`] 绑定其在所属 TOML 文件中的原始序号（从 1 开始），跨文件合并后仍保留各自的文件来源。
fn located_steps(steps: Vec<Step>, path: &Path) -> Vec<Step> {
    steps
        .into_iter()
        .enumerate()
        .map(|(index, mut step)| {
            step.location = Some(location(path, index));
            step
        })
        .collect()
}

/// 根据配置文件路径和零基数组下标构造从 1 开始计数的 [`SourceLocation`]。
fn location(path: &Path, index: usize) -> SourceLocation {
    SourceLocation {
        file: path.to_path_buf(),
        entry: index + 1,
        catalog: None,
        line: None,
    }
}

/// 对合并后的 [`Config`] 执行一次性静态结构与字段边界校验。
///
/// 此函数仅检查配置项自身的格式、路径合法性、引用关系与字段互斥约束；目标根目录内的实际目录或清单内容留给 [`crate::selection::resolve`] 检查。
fn validate(config: &Config) -> Result<(), String> {
    // Step 1: 校验全局默认参数（defaults）的超时、重复次数、环境变量与前后置命令。
    let plan = config.plan.display().to_string();
    timing(config.defaults.timeout_sec, config.defaults.repeat, &plan)?;
    environment(&config.defaults.env, &plan)?;
    commands(&config.defaults.setup, &plan)?;
    commands(&config.defaults.cleanup, &plan)?;
    // Step 2: 校验所有声明的命名根目录（roots），确保名称、路径、环境变量及挂载点列表（仅限无重复的 dev/proc/tmp）合法。
    for (name, root) in &config.roots {
        let at = format!("{plan}:roots.{name}");
        identifier(name, &at)?;
        absolute(&root.path, &at)?;
        absolute(&root.cwd, &at)?;
        environment(&root.env, &at)?;
        commands(&root.prepare, &at)?;
        let mut mounts = BTreeSet::new();
        for mount in &root.mounts {
            if !matches!(mount.as_str(), "dev" | "proc" | "tmp") || !mounts.insert(mount) {
                return Err(format!("{at}: invalid or repeated mount {mount:?}"));
            }
        }
    }
    // Step 3: 逐条校验测试步骤（steps）的公共字段与按类型区分的专属字段。
    for step in &config.steps {
        validate_step(step, config)?;
    }
    // Step 4: 逐条校验单测例覆盖项（overrides），确保引用的 root 已声明且未混用直接执行（Exec）与 Shell 命令字段。
    for item in &config.overrides {
        let at = item.location.as_ref().unwrap().to_string();
        identifier(&item.id, &at)?;
        if let Some(root) = &item.root {
            check_root(root, config, &at)?;
        }
        fields(
            item.cwd.as_deref(),
            item.timeout_sec,
            item.repeat,
            &item.env,
            item.setup.as_deref(),
            item.cleanup.as_deref(),
            &at,
        )?;
        for text in [&item.program, &item.shell, &item.command]
            .into_iter()
            .flatten()
        {
            nonempty(text, &at)?;
        }
        strings(item.args.as_deref().unwrap_or_default(), &at)?;
        if (item.program.is_some() || item.args.is_some())
            && (item.shell.is_some() || item.command.is_some())
        {
            return Err(format!("{at}: override mixes exec and shell fields"));
        }
    }
    Ok(())
}

/// 校验单个测试步骤（[`Step`]）的字段完整性与类型互斥规则。
///
/// 确保每种 [`StepKind`] 只包含对其有意义的字段，防止直接执行命令与清单筛选字段混用，或不同测试套件的专属字段相互污染。
fn validate_step(step: &Step, config: &Config) -> Result<(), String> {
    // Step 1: 校验步骤的公共运行参数（cwd、timeout_sec、repeat、env、setup、cleanup）与目标架构列表。
    let at = step.location.as_ref().unwrap().to_string();
    fields(
        step.cwd.as_deref(),
        step.timeout_sec,
        step.repeat,
        &step.env,
        step.setup.as_deref(),
        step.cleanup.as_deref(),
        &at,
    )?;
    for arch in &step.arch {
        if !matches!(arch.as_str(), "riscv64" | "loongarch64") {
            return Err(format!("{at}: unknown architecture {arch:?}"));
        }
    }
    // Step 2: 区分直接命令步骤（Exec/Shell）与清单筛选步骤（Ltp/Basic/Libctest），检查各自必填字段并禁止跨类别字段混用。
    let direct = matches!(step.kind, StepKind::Exec | StepKind::Shell);
    if direct {
        identifier(
            step.id
                .as_deref()
                .ok_or_else(|| format!("{at}: missing id"))?,
            &at,
        )?;
        if !step.roots.is_empty()
            || !step.include.is_empty()
            || !step.exclude.is_empty()
            || step.catalog.is_some()
            || step.directory.is_some()
            || step.source.is_some()
            || step.suite.is_some()
            || step.variant.is_some()
        {
            return Err(format!("{at}: exec/shell cannot contain selection fields"));
        }
        check_root(
            step.root
                .as_deref()
                .ok_or_else(|| format!("{at}: missing root"))?,
            config,
            &at,
        )?;
    } else {
        if step.root.is_some()
            || step.id.is_some()
            || step.program.is_some()
            || step.args.is_some()
            || step.command.is_some()
        {
            return Err(format!(
                "{at}: catalog step cannot contain direct command fields"
            ));
        }
        if step.roots.is_empty() || step.include.is_empty() {
            return Err(format!("{at}: roots and include must not be empty"));
        }
        for root in &step.roots {
            check_root(root, config, &at)?;
        }
        for pattern in step.include.iter().chain(&step.exclude) {
            nonempty(pattern, &at)?;
            Pattern::new(pattern)
                .map_err(|error| format!("{at}: invalid pattern {pattern:?}: {error}"))?;
        }
        if let Some(catalog) = &step.catalog {
            absolute(catalog, &at)?;
        }
        if let Some(directory) = &step.directory {
            absolute(directory, &at)?;
        }
    }
    // Step 3: 按具体的 StepKind 校验专属字段（如程序名、Shell 命令、LTP 数据源、Libctest 链接变体等）。
    match step.kind {
        StepKind::Exec => {
            nonempty(
                step.program
                    .as_deref()
                    .ok_or_else(|| format!("{at}: missing program"))?,
                &at,
            )?;
            if step.shell.is_some() || step.command.is_some() {
                return Err(format!("{at}: exec has shell fields"));
            }
        }
        StepKind::Shell => {
            nonempty(
                step.shell
                    .as_deref()
                    .ok_or_else(|| format!("{at}: missing shell"))?,
                &at,
            )?;
            nonempty(
                step.command
                    .as_deref()
                    .ok_or_else(|| format!("{at}: missing command"))?,
                &at,
            )?;
            if step.program.is_some() || step.args.is_some() {
                return Err(format!("{at}: shell has exec fields"));
            }
        }
        StepKind::Ltp => {
            identifier(
                step.suite
                    .as_deref()
                    .ok_or_else(|| format!("{at}: missing suite"))?,
                &at,
            )?;
            if step.variant.is_some() {
                return Err(format!("{at}: ltp has variant"));
            }
            match step.source.unwrap_or_default() {
                crate::model::CatalogSource::Runtest => {
                    if step.catalog.is_none() || step.directory.is_some() {
                        return Err(format!("{at}: runtest requires catalog and forbids directory (no directory fallback)"));
                    }
                    if let Some(shell) = &step.shell {
                        nonempty(shell, &at)?;
                    }
                }
                crate::model::CatalogSource::Executables => {
                    if step.directory.is_none() || step.catalog.is_some() || step.shell.is_some() {
                        return Err(format!(
                            "{at}: executables requires directory and forbids catalog/shell"
                        ));
                    }
                }
            }
        }
        StepKind::Basic | StepKind::Libctest => {
            if step.source.is_some() || step.suite.is_some() || step.shell.is_some() {
                return Err(format!("{at}: basic/libctest has ltp fields"));
            }
            if step.kind == StepKind::Libctest {
                if step.directory.is_some() {
                    return Err(format!("{at}: libctest uses catalog, not directory"));
                }
                if !matches!(step.variant.as_deref(), Some("dynamic" | "static")) {
                    return Err(format!("{at}: libctest variant must be dynamic or static"));
                }
            } else if step.variant.is_some() || step.catalog.is_some() {
                return Err(format!("{at}: basic uses directory and has no variant"));
            }
        }
    }
    strings(step.args.as_deref().unwrap_or_default(), &at)
}

/// 检查步骤或覆盖项引用的根目录名称是否已在 `config.roots` 中显式声明。
fn check_root(name: &str, config: &Config, at: &str) -> Result<(), String> {
    if !config.roots.contains_key(name) {
        return Err(format!("{at}: unknown root {name:?}"));
    }
    Ok(())
}

/// 校验步骤或覆盖项中的公共可选运行字段（工作目录、超时、重复次数、环境变量及前后置命令列表）。
fn fields(
    cwd: Option<&Path>,
    timeout: Option<u64>,
    repeat: Option<u32>,
    env: &BTreeMap<String, String>,
    setup: Option<&[Vec<String>]>,
    cleanup: Option<&[Vec<String>]>,
    at: &str,
) -> Result<(), String> {
    if let Some(cwd) = cwd {
        absolute(cwd, at)?;
    }
    timing(timeout.unwrap_or(1), repeat.unwrap_or(1), at)?;
    environment(env, at)?;
    commands(setup.unwrap_or_default(), at)?;
    commands(cleanup.unwrap_or_default(), at)
}

/// 校验超时秒数（`timeout`）与重复轮次（`repeat`）必须为正整数（大于 0）。
fn timing(timeout: u64, repeat: u32, at: &str) -> Result<(), String> {
    if timeout == 0 || repeat == 0 {
        return Err(format!("{at}: timeout_sec and repeat must be positive"));
    }
    Ok(())
}

/// 校验配置中的路径必须为不含 `..`（父目录组件）和 NUL 字节的绝对路径，防止路径穿越逃逸出目标根目录。
pub(crate) fn absolute(path: &Path, at: &str) -> Result<(), String> {
    if !path.is_absolute()
        || path.components().any(|part| part == Component::ParentDir)
        || path.as_os_str().as_encoded_bytes().contains(&0)
    {
        return Err(format!(
            "{at}: expected absolute path without parent components: {}",
            path.display()
        ));
    }
    Ok(())
}

/// 校验标识符（如根目录名、套件名或测例 ID）非空，且不包含任何空白字符或 NUL 字节。
fn identifier(text: &str, at: &str) -> Result<(), String> {
    nonempty(text, at)?;
    if text.chars().any(char::is_whitespace) {
        return Err(format!("{at}: invalid identifier {text:?}"));
    }
    Ok(())
}

/// 校验字符串非空且不包含底层 `execve` 系统调用无法表示的 NUL（`\0`）字符。
fn nonempty(text: &str, at: &str) -> Result<(), String> {
    if text.is_empty() || text.contains('\0') {
        return Err(format!("{at}: empty or NUL-containing value"));
    }
    Ok(())
}

/// 校验命令参数列表（`argv`）中的每个字符串均不包含 NUL（`\0`）字符（允许空字符串、空格及任意合法 UTF-8 字符）。
fn strings(values: &[String], at: &str) -> Result<(), String> {
    if values.iter().any(|text| text.contains('\0')) {
        return Err(format!("{at}: argument contains NUL"));
    }
    Ok(())
}

/// 校验环境变量映射表：键名不得为空或包含 `=` 与 `\0`，键值不得包含 `\0`。
fn environment(env: &BTreeMap<String, String>, at: &str) -> Result<(), String> {
    for (key, value) in env {
        if key.is_empty() || key.contains(['=', '\0']) || value.contains('\0') {
            return Err(format!("{at}: invalid environment key/value {key:?}"));
        }
    }
    Ok(())
}

/// 校验准备（`setup`/`prepare`）或清理（`cleanup`）命令数组列表：每条命令数组不得为空，首元素（可执行文件名）不得为空串，且所有参数均不含 NUL。
fn commands(commands: &[Vec<String>], at: &str) -> Result<(), String> {
    for command in commands {
        nonempty(
            command
                .first()
                .ok_or_else(|| format!("{at}: empty command array"))?,
            at,
        )?;
        strings(command, at)?;
    }
    Ok(())
}
