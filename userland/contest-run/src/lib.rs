//! 用户态比赛测试与内置自测（Selftests）执行框架的核心库。
//!
//! ## 模块职责与架构定位
//! 本库为 `/bin/contest-run`（比赛主入口）和 `/bin/user-selftests`（内置自测入口）提供共享的配置解析、
//! 测例选择、根环境挂载、子进程超时监控与测试协议解析能力。
//!
//! ## 核心执行流水线
//! ```text
//! +-------------------+     +----------------------+     +------------------------+
//! |   config 模块     |     |    selection 模块    |     | runtime / execute 模块 |
//! | 加载 TOML 计划与  | ──> | 扫描目标 root 下的   | ──> | 挂载虚拟文件系统、执行 |
//! | 静态字段边界校验  |     | 清单/目录并合并覆盖  |     | setup/hook/process/    |
//! +-------------------+     +----------------------+     | cleanup 与协议判定     |
//!                                                        +------------------------+
//!                                                                    │
//!                                                                    ▼
//!                                                        +------------------------+
//!                                                        |  outcome / report 模块 |
//!                                                        | 解析 LTP/TAP/Basic 等  |
//!                                                        | 输出协议并生成统计报告 |
//!                                                        +------------------------+
//! ```
//!
//! ## 并发与锁契约
//! 库内各模块均面向单进程顺序调度设计；子进程执行阶段通过非阻塞管道与 `poll` 轮询同时收集标准输出与标准错误，不引入跨线程共享锁。

/// 测试计划（TOML）的加载、多文件组合与静态输入边界校验。
pub mod config;
/// 内核测试钩子字符设备（`/dev/testhook`）的单消息写入接口。
pub mod custom;
/// 单个测例的 `setup`、`before`/`after` 钩子配对、主命令执行与 `cleanup` 编排。
pub mod execute;
/// 配置模型、步骤种类、输出协议类型与展开后测例（[`model::ResolvedCase`]）的数据结构定义。
pub mod model;
/// 子进程退出状态、输出协议解析结果与测例最终判定（[`outcome::Verdict`]）。
pub mod outcome;
/// 基于 `fork`/`execve`、进程组信号控制与非阻塞管道轮询的子进程执行引擎。
pub mod process;
/// 测试结果汇总统计与格式化报告输出。
pub mod report;
/// 比赛根目录挂载管理（`dev`/`proc`/`tmp`）、受限符号链接路径解析与 ELF 解释器诊断。
pub mod runtime;
/// 基于目标根目录内清单或可执行文件目录的测例筛选、排序与参数覆盖合并。
pub mod selection;
/// 内置用户态自测集（`kselftest-list.txt`）的命令行解析、筛选与独立运行实现。
pub mod selftests;
/// TAP（Test Anything Protocol）文本输出流的状态机解析器。
mod tap;
