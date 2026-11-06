//! 评测用例前后置生命周期钩子分发层（Contest Case Lifecycle Hooks）。
//!
//! ## 模块职责与架构定位
//! 本模块承接用户态评测控制程序向设备文件写入的 `before <case>\n` 与 `after <case>\n` 控制命令：
//! - **上游调用者**：`fs::devfs::dispatch_hook_message` 在内核缓冲区完整解析并校验命令格式与用例标识符后，调用本模块的 [`before`] 或 [`after`]。
//! - **下游依赖**：`trace` 子系统（针对 `runner:trace` 用例启停全局每 CPU 事件追踪，观测窗口同时覆盖被测进程及同一时段的内核后台任务）。
//!
//! ## 核心调用流程
//! ```text
//!  用户态评测器写入命令
//!    │  "before <case>\n" / "after <case>\n"
//!    ▼
//!  fs::devfs::dispatch_hook_message (语法与非空无空白校验)
//!    ├──► contest_hooks::before(case)
//!    │      ├── b"runner:trace" ──► trace::start() (清空各核旧环并开启采集；已开启则返回 FsError::Busy)
//!    │      └── 其它未知用例 ID  ──► Ok(()) (无操作放行)
//!    └──► contest_hooks::after(case)
//!           ├── b"runner:trace" ──► trace::stop()  (关闭采集并等待全部在线核在途事件写入完毕；重复停止幂等成功)
//!           └── 其它未知用例 ID  ──► Ok(()) (无操作放行)
//! ```
//!
//! ## 锁获取顺序与并发契约
//! - 本模块自身不维护额外锁状态，直接委托给 `trace::start` / `trace::stop`（内部获取 `trace::CONTROL` 睡眠锁并可能发起跨核同步 IPI）。
//! - 调用者必须运行在可睡眠、已开启本地中断的普通任务上下文中，且在进入本模块前不得持有用户地址空间访问借用或任何自旋锁。

use crate::fs::FsError;
use crate::trace;

/// 执行指定评测用例运行前的内核准备钩子（必须在普通可睡眠任务且未持有子系统自旋锁的上下文中调用）。
///
/// `case` 为经上层校验的非空 ASCII 用例标识符：命中 `"runner:trace"` 时启动全局事件追踪（若追踪已在运行则返回 `Err(FsError::Busy)`）；未注册的用例标识符直接返回 `Ok(())`。
pub(crate) fn before(case: &[u8]) -> Result<(), FsError> {
    match case {
        b"runner:trace" => trace::start(),
        _ => Ok(()),
    }
}

/// 执行指定评测用例结束后的内核收尾钩子（必须在普通可睡眠任务且未持有子系统自旋锁的上下文中调用）。
///
/// `case` 为经上层校验的非空 ASCII 用例标识符：命中 `"runner:trace"` 时停止全局事件追踪并等待所有在线 CPU 完成在途事件提交（重复停止幂等返回 `Ok(())`，且保留已冻结的追踪记录供后续读取）；未注册的用例标识符直接返回 `Ok(())`。
pub(crate) fn after(case: &[u8]) -> Result<(), FsError> {
    match case {
        b"runner:trace" => trace::stop(),
        _ => Ok(()),
    }
}
