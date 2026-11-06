//! 内核网络栈（Network Stack）与套接字（Socket）集成测试入口模块。
//!
//! # 模块职责与架构定位
//! 本模块汇总内核网络子系统的全套 `ktest` 回归测试套件，负责向测试注册表导出所有网络测试锚点。
//! 上游由 `crate::ktest` 测试框架在启动或测试阶段扫描调用，下游依赖 `crate::net`（核心协议栈、
//! 后台回环网络服务、TCP/UDP/Unix 域套接字）以及 `crate::fs::epoll` 等文件抽象。
//!
//! # 核心设计与套件划分
//! 网络测试按照测试层级与待测组件划分为两个基础协议层套件和七个套接字行为套件，
//! 通过 `include!` 宏内联到本模块中，使每个套件拥有独立的测试夹具（Fixture）与资源泄漏基线检查：
//!
//! ```text
//! +-----------------------------------------------------------------------+
//! |                        ktest::network 测试矩阵                        |
//! +-----------------------------------------------------------------------+
//! | 辅助支撑层 (Submodules)                                               |
//! |   - wire           : 同步内存包队列夹具 NetworkOwner (无后台线程)     |
//! |   - owner          : 后台网络栈服务夹具 StackOwner (含异步 Work)      |
//! |   - support        : 端点常量、超时轮询与错误诊断包装                 |
//! |   - stack_helpers  : 协议层 TcpEndpoint / UDP 缓冲区构建与握手辅助    |
//! |   - socket_helpers : VFS SocketFile 读写、Epoll 收集与 OOM 门控任务   |
//! +-----------------------------------------------------------------------+
//! | 1. 同步协议核心层 (core.rs -> NetworkOwner)                           |
//! |   - 验证 TcpCore 状态机、环形缓冲区回绕、MSG_PEEK、队列满回退与缺页   |
//! +-----------------------------------------------------------------------+
//! | 2. 后台网络栈层 (stack.rs -> StackOwner)                              |
//! |   - 验证 NetStack 后台回环转发、并发连接建立與端点生命周期            |
//! +-----------------------------------------------------------------------+
//! | 3. 套接字与 VFS 集成层 (socket_*.rs -> StackOwner)                    |
//! |   - socket_unix_stream    : Unix 流式套接字读写与对端关闭语义         |
//! |   - socket_unix_seqpacket : Unix 顺序包套接字边界保持与截断语义       |
//! |   - socket_unix_owner     : Unix 套接字所有权转移与资源回收           |
//! |   - socket_tcp_listener   : TCP 监听套接字并发 accept 与队列管理      |
//! |   - socket_tcp_progress   : TCP 套接字流式收发、阻塞唤醒与 Epoll 边沿 |
//! |   - socket_udp_lease      : UDP 套接字租约分配、收发与容量边界        |
//! |   - socket_udp_retirement : UDP 套接字在内存压力与关闭时的退役回收    |
//! +-----------------------------------------------------------------------+
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 本文件仅组织子模块与编译期内联套件，自身不持有锁。
//! - 各内联套件通过各自的测试夹具（`NetworkOwner` 或 `StackOwner`）在每个测试用例前后执行
//!   `set_up` -> `quiesce` -> `tear_down` 生命周期，确保测试任务退出、后台网络服务停止后再校验内存基线。

mod owner;
mod socket_helpers;
mod stack_helpers;
mod support;
mod wire;

// `#[ktest::suite]` 宏要求直接作用于模块定义上；通过 `include!` 将各测试套件内联到本文件中，
// 使每个套件拥有独立的测试夹具生命周期与资源基线检查边界。
include!("network/core.rs");
include!("network/stack.rs");
include!("network/socket_unix_stream.rs");
include!("network/socket_tcp_listener.rs");
include!("network/socket_tcp_progress.rs");
include!("network/socket_unix_seqpacket.rs");
include!("network/socket_unix_owner.rs");
include!("network/socket_udp_lease.rs");
include!("network/socket_udp_retirement.rs");

/// 返回全部 9 个网络测试套件的静态注册槽引用数组。
///
/// 供上层 `ktest` 注册表显式引用，防止链接器在死代码消除（Dead Code Elimination）阶段
/// 丢弃由宏生成的静态测试套件描述符。
pub(super) fn anchors() -> [&'static ::ktest::SuiteSlot; 9] {
    [
        core::__ktest_anchor(),
        stack::__ktest_anchor(),
        socket_unix_stream::__ktest_anchor(),
        socket_tcp_listener::__ktest_anchor(),
        socket_tcp_progress::__ktest_anchor(),
        socket_unix_seqpacket::__ktest_anchor(),
        socket_unix_owner::__ktest_anchor(),
        socket_udp_lease::__ktest_anchor(),
        socket_udp_retirement::__ktest_anchor(),
    ]
}
