//! 测试计划配置模型、用例选择来源与已展开可执行用例的数据结构定义。
//!
//! ## 模块职责与架构定位
//! 本模块位于 `contest-run` 的数据模型层，不依赖具体的系统调用或执行逻辑。
//! 上游由 `config` 模块将 TOML 配置文件反序列化为 `Config`、`Step` 与 `CaseOverride`，
//! 中游由 `selection` 模块结合架构与套件清单将 `Config` 展开为 `Vec<ResolvedCase>`，
//! 下游由 `execute`、`process` 与 `report` 模块读取 `ResolvedCase`、`CommandSpec` 与 `Protocol` 驱动测试运行。
//!
//! ## 核心配置展开管线
//!
//! ```text
//!   TOML 计划文件 (plan.toml + --file 覆盖文件)
//!       │
//!       ▼
//!   Config { roots, defaults, steps, overrides, finish }
//!       │
//!       ├─ 结合 RootSpec (chroot 路径/默认 cwd/env/mounts/prepare)
//!       ├─ 结合 Defaults (全局默认 timeout_sec/repeat/hooks/env/setup/cleanup)
//!       ├─ 展开 Step (Ltp / Basic / Libctest / Exec / Shell)
//!       └─ 应用 CaseOverride (按用例 id 覆盖特定参数)
//!       │
//!       ▼
//!   ResolvedCase { id, source, root, cwd, command, env, timeout_sec, repeat, hooks, setup, cleanup, protocol }
//! ```
//!
//! ## 并发与锁契约
//! 本模块仅包含纯值类型（Value Types），无内部可变性或锁原语；所有结构在配置解析完成后均以不可变借用或克隆方式在单线程控制流中传递。

use serde::Deserialize;
use std::collections::BTreeMap;
use std::fmt;
use std::path::PathBuf;

/// 测试用例在计划文件与外部套件清单中的定义位置，用于诊断输出与错误溯源。
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SourceLocation {
    /// 定义该条目所属的 TOML 计划文件路径（或命令行虚拟路径 `<command-line>`）。
    pub file: PathBuf,
    /// 该条目在对应 TOML 文件的 `steps` 或 `overrides` 数组中的序号（从 1 开始计数）。
    pub entry: usize,
    /// 若用例展开自外部测试清单文件（如 LTP `runtest` 文件），记录该清单文件的路径。
    pub catalog: Option<PathBuf>,
    /// 若用例来自外部测试清单文件，记录其在清单文件中的行号（从 1 开始计数）。
    pub line: Option<usize>,
}

impl fmt::Display for SourceLocation {
    /// 将来源位置格式化为 `文件路径:entry 序号 (清单路径:行号)` 的可读文本。
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}:entry {}", self.file.display(), self.entry)?;
        if let Some(catalog) = &self.catalog {
            write!(f, " ({}", catalog.display())?;
            if let Some(line) = self.line {
                write!(f, ":{line}")?;
            }
            write!(f, ")")?;
        }
        Ok(())
    }
}

/// 合并主计划文件与附加覆盖文件后的完整测试运行配置。
#[derive(Clone, Debug)]
pub struct Config {
    /// 命名测试根环境映射表（键为根名称，如 `"musl"`、`"glibc"`，值为对应的 `RootSpec`）。
    pub roots: BTreeMap<String, RootSpec>,
    /// 全局默认执行参数，作为所有测试步骤的基础回退值。
    pub defaults: Defaults,
    /// 顺序定义的测试步骤列表，每个步骤可展开为一个或多个具体测试用例。
    pub steps: Vec<Step>,
    /// 针对特定用例 ID 的单点参数覆盖规则列表。
    pub overrides: Vec<CaseOverride>,
    /// 全部测试完成后的收尾动作（关机或直接返回退出码）。
    pub finish: Finish,
    /// 主测试计划文件路径。
    pub plan: PathBuf,
}

/// 单个测试根环境（chroot 容器目录）的配置规格。
#[derive(Clone, Debug, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct RootSpec {
    /// 测试根在宿主文件系统中的绝对路径（如 `/mnt/contest/riscv64/musl`）。
    pub path: PathBuf,
    /// 该测试根下用例的默认工作目录（相对于 chroot 后的根目录，默认为 `"/"`）。
    #[serde(default = "root_cwd")]
    pub cwd: PathBuf,
    /// 该测试根下所有用例默认注入的环境变量集合。
    #[serde(default)]
    pub env: BTreeMap<String, String>,
    /// 进入该测试根前需要挂载的子目录列表（仅支持 `"dev"`、`"proc"`、`"tmp"`）。
    #[serde(default)]
    pub mounts: Vec<String>,
    /// 准备该测试根时在宿主根（非 chroot 环境）下预先执行的命令数组列表。
    #[serde(default)]
    pub prepare: Vec<Vec<String>>,
}

/// 提供 `RootSpec::cwd` 的反序列化默认值 `"/"`，避免受执行器自身当前工作目录影响。
fn root_cwd() -> PathBuf {
    PathBuf::from("/")
}

/// 全局默认测试执行参数。
#[derive(Clone, Debug, Deserialize, PartialEq, Eq)]
#[serde(default, deny_unknown_fields)]
pub struct Defaults {
    /// 单个测试用例单次运行的超时时间（单位：秒，默认 60 秒）。
    pub timeout_sec: u64,
    /// 单个测试用例的重复执行轮数（默认 1 次）。
    pub repeat: u32,
    /// 是否启用内置的 Rust 自定义前后置钩子（`custom::hook`，默认 `false`）。
    pub hooks: bool,
    /// 全局默认环境变量映射表。
    pub env: BTreeMap<String, String>,
    /// 每轮测试启动前在测试根（chroot 环境）内执行的准备命令列表。
    pub setup: Vec<Vec<String>>,
    /// 每轮测试结束后在测试根（chroot 环境）内执行的清理命令列表。
    pub cleanup: Vec<Vec<String>>,
}

impl Default for Defaults {
    /// 构造标准默认运行参数（超时 60 秒、重复 1 次、不启用钩子、无额外环境变量与前后置命令）。
    fn default() -> Self {
        Self {
            timeout_sec: 60,
            repeat: 1,
            hooks: false,
            env: BTreeMap::new(),
            setup: Vec::new(),
            cleanup: Vec::new(),
        }
    }
}

/// 测试运行全部结束后的收尾动作模式。
#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum Finish {
    /// 执行 `sync` 后调用 `poweroff` 关闭系统（默认模式，适用于无人值守评测）。
    #[default]
    Poweroff,
    /// 执行 `sync` 后直接退出 `contest-run` 进程并返回退出码（适用于交互式调试或单命令运行）。
    Return,
}

/// 测试计划步骤（`Step`）的种类。
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum StepKind {
    /// Linux Test Project (LTP) 测试套件步骤。
    Ltp,
    /// 初赛基础系统调用测试套件（`basic`）步骤。
    Basic,
    /// 2025 年前旧版 `libctest` 静态/动态链接测试套件步骤。
    Libctest,
    /// 直接执行指定可执行文件与参数列表的独立步骤。
    Exec,
    /// 通过指定 shell 的 `-c` 参数执行命令字符串的独立步骤。
    Shell,
}

/// LTP 测试步骤发现测试用例的来源方式。
#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum CatalogSource {
    /// 从 LTP 的 `runtest/<suite>` 清单文件中解析测试命令（默认方式）。
    #[default]
    Runtest,
    /// 直接扫描指定目录下的可执行文件作为独立测试用例。
    Executables,
}

/// 测试计划中的单个配置步骤，在 `selection` 阶段展开为一个或多个 `ResolvedCase`。
#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Step {
    /// 步骤类型，决定必填字段约束与用例展开规则。
    pub kind: StepKind,
    /// 是否启用该步骤（默认为 `true`；设为 `false` 时仍需通过配置格式校验，但不产生选中用例）。
    #[serde(default = "enabled")]
    pub enabled: bool,
    /// 允许运行该步骤的目标架构名称列表（为空表示不限制架构）。
    #[serde(default)]
    pub arch: Vec<String>,
    /// 独立命令步骤（`Exec` / `Shell`）的唯一用例标识符。
    pub id: Option<String>,
    /// 单个目标测试根名称（与 `roots` 二选一，取决于步骤类型）。
    pub root: Option<String>,
    /// 多个目标测试根名称列表（用于同一套件在 `musl`、`glibc` 等多个根下分别展开）。
    #[serde(default)]
    pub roots: Vec<String>,
    /// LTP 测试套件名称（对应 `runtest/<suite>` 文件名）。
    pub suite: Option<String>,
    /// LTP 用例来源模式（`Runtest` 清单或 `Executables` 目录扫描）。
    pub source: Option<CatalogSource>,
    /// 显式指定的套件清单文件路径（相对于测试根）。
    pub catalog: Option<PathBuf>,
    /// 显式指定的可执行文件扫描目录路径（相对于测试根）。
    pub directory: Option<PathBuf>,
    /// `Libctest` 套件的链接变体名称（如 `"static"` 或 `"dynamic"`）。
    pub variant: Option<String>,
    /// 仅包含的子用例名称白名单列表（为空表示包含套件内全部用例）。
    #[serde(default)]
    pub include: Vec<String>,
    /// 需要排除的子用例名称黑名单列表。
    #[serde(default)]
    pub exclude: Vec<String>,
    /// 覆盖测试根默认值的用例工作目录（相对于测试根的绝对路径）。
    pub cwd: Option<PathBuf>,
    /// `Exec` 步骤的目标可执行文件路径。
    pub program: Option<String>,
    /// `Exec` 步骤传递给可执行文件的命令行参数列表。
    pub args: Option<Vec<String>>,
    /// `Shell` 步骤使用的 shell 解释器路径（默认为 `"/bin/sh"`）。
    pub shell: Option<String>,
    /// `Shell` 步骤交给 `shell -c` 执行的命令脚本文本。
    pub command: Option<String>,
    /// 步骤级附加或覆盖的环境变量映射表。
    #[serde(default)]
    pub env: BTreeMap<String, String>,
    /// 步骤级覆盖的单次运行超时时间（单位：秒）。
    pub timeout_sec: Option<u64>,
    /// 步骤级覆盖的重复执行次数。
    pub repeat: Option<u32>,
    /// 步骤级覆盖的内置自定义钩子开关。
    pub hooks: Option<bool>,
    /// 步骤级覆盖的每轮前置准备命令列表。
    pub setup: Option<Vec<Vec<String>>>,
    /// 步骤级覆盖的每轮后置清理命令列表。
    pub cleanup: Option<Vec<Vec<String>>>,
    /// 该步骤在计划文件中的解析来源位置（由配置加载器填充，不参与反序列化）。
    #[serde(skip)]
    pub location: Option<SourceLocation>,
}

/// 提供 `Step::enabled` 的反序列化默认值 `true`。
fn enabled() -> bool {
    true
}

/// 针对特定已展开用例 ID 的参数覆盖规则。
#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CaseOverride {
    /// 目标测试用例的唯一标识符（必须匹配某个已配置步骤产出的用例 ID）。
    pub id: String,
    /// 覆盖目标测试根名称。
    pub root: Option<String>,
    /// 覆盖用例在测试根内的工作目录。
    pub cwd: Option<PathBuf>,
    /// 将用例执行方式覆盖为直接 `Exec` 指定的可执行文件路径。
    pub program: Option<String>,
    /// 覆盖 `Exec` 方式的命令行参数列表。
    pub args: Option<Vec<String>>,
    /// 覆盖 `Shell` 方式的解释器路径。
    pub shell: Option<String>,
    /// 将用例执行方式覆盖为 `Shell` 命令字符串。
    pub command: Option<String>,
    /// 针对该用例追加或覆盖的环境变量映射表。
    #[serde(default)]
    pub env: BTreeMap<String, String>,
    /// 覆盖单次运行超时时间（单位：秒）。
    pub timeout_sec: Option<u64>,
    /// 覆盖重复执行次数。
    pub repeat: Option<u32>,
    /// 覆盖是否启用内置自定义钩子。
    pub hooks: Option<bool>,
    /// 覆盖每轮前置准备命令列表。
    pub setup: Option<Vec<Vec<String>>>,
    /// 覆盖每轮后置清理命令列表。
    pub cleanup: Option<Vec<Vec<String>>>,
    /// 该覆盖条目在计划文件中的来源位置（由配置加载器填充）。
    #[serde(skip)]
    pub location: Option<SourceLocation>,
}

/// 子进程命令的启动方式规格。
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CommandSpec {
    /// 直接通过 `exec` 执行目标程序并传递参数列表（`program` 为可执行文件路径或程序名，`args` 为不含 `argv[0]` 的参数列表），不经过 shell 解析。
    Exec { program: String, args: Vec<String> },
    /// 启动指定的 shell 解释器并通过 `-c` 执行单条命令文本（`shell` 为解释器路径如 `"/bin/sh"`，`command` 为命令字符串）。
    Shell { shell: String, command: String },
}

/// 测试用例的输出解析与结果判定协议。
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Protocol {
    /// 普通进程协议：不解析标准输出正文，仅根据进程退出码是否为 0 判定 `Pass` 或 `Failed`。
    Plain,
    /// LTP 退出码位图协议：不解析标准输出正文，按 `TFAIL`/`TBROK`/`TWARN`/`TCONF` 位掩码判定结果。
    Ltp,
    /// Test Anything Protocol (TAP) 协议：流式解析标准输出中的测试计划与子测试结果行。
    Tap,
    /// 初赛 `basic` 套件输出标记协议：检查特定测试（`name` 为子测试名如 `"clone"`、`"mkdir_"`）的起止标记行、已知错误文本与退出码。
    Basic { name: String },
    /// 2025 年前旧版 `libctest` 输出标记协议：检查 `entry-<variant>.exe <selector>` 起止标记、`Pass!` 行与退出码（`variant` 为 `"static"` 或 `"dynamic"`，`selector` 为子测试选择器名称）。
    Pre2025Libctest { variant: String, selector: String },
}

/// 经过配置合并、套件展开与覆盖规则应用后，可直接交付执行的最终测试用例定义。
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ResolvedCase {
    /// 全局唯一的测试用例标识符（如 `"ltp:musl:open01"`）。
    pub id: String,
    /// 用例定义来源（所属计划文件条目及可选的外部清单行号）。
    pub source: SourceLocation,
    /// 所属测试根环境的名称（对应 `Config::roots` 中的键）。
    pub root: String,
    /// 在测试根（chroot 环境）内执行时的当前工作目录绝对路径。
    pub cwd: PathBuf,
    /// 待执行的命令规格（直接 `Exec` 或 `Shell -c`）。
    pub command: CommandSpec,
    /// 合并测试根默认、全局默认、步骤与覆盖规则后的最终环境变量表。
    pub env: BTreeMap<String, String>,
    /// 单次运行的超时时间（单位：秒，大于 0）。
    pub timeout_sec: u64,
    /// 连续重复执行的轮数（每轮使用相同的 `id`，在执行记录中以 `iteration` 区分）。
    pub repeat: u32,
    /// 是否在每轮执行前后调用内置的 Rust 钩子函数。
    pub hooks: bool,
    /// 每轮主命令运行前在测试根内顺序执行的准备命令列表。
    pub setup: Vec<Vec<String>>,
    /// 每轮主命令运行后在测试根内顺序执行的清理命令列表。
    pub cleanup: Vec<Vec<String>>,
    /// 主命令使用的输出观察与结果判定协议。
    pub protocol: Protocol,
}
