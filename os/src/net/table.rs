//! 网络协议栈的端口绑定租约表、协议端点注册表与报文分发路由索引。
//!
//! ## 模块职责与架构定位
//! 本模块位于网络域（[`NetStack`]）内部，负责集中管理三类核心表项：
//! 1. **端口绑定表（`bindings`）**：跟踪 TCP/UDP 本地地址与端口预留（[`BindLease`]）、动态临时端口分配以及 `SO_REUSEADDR` / 双栈（Dual-Stack）冲突检查。
//! 2. **协议对象持有表（`tcp`、`udp`、`unix`）**：保存活跃 [`TcpEndpoint`]、[`UdpEndpoint`] 与 [`UnixPair`] 的强引用（`Arc`），作为网络域生命周期内的所有权根。
//! 3. **收包查找索引（`tuples`、`listeners`、`udp_keys`）**：维护从四元组、监听端点或 UDP 本地/远端键到协议对象 ID 的映射，供底层收包分发与容量恢复通知快速定位目标端点。
//!
//! 上游调用者为 [`NetStack`]、[`TcpEndpoint`]、[`UdpEndpoint`] 与 [`UnixPair`]；本模块仅依赖基础同步原语与网络地址定义，不反向调用协议状态机。
//!
//! ## 核心设计与表结构关系
//! ```text
//! +-----------------------------------------------------------------------+
//! |                              NetTables                                |
//! |                                                                       |
//! |  [查找索引层]                        [对象持有层]                     |
//! |  tuples:    (id, TcpTuple)       ---> tcp:  Vec<Arc<TcpEndpoint>>     |
//! |  listeners: (id, local, dual)    --/                                  |
//! |  udp_keys:  (id, local, peer, d) ---> udp:  Vec<Arc<UdpEndpoint>>     |
//! |                                       unix: Vec<Arc<UnixPair>>        |
//! |                                                                       |
//! |  [端口租约层]                                                         |
//! |  bindings:  Binding { lease_id, owner_id, protocol, local, ... }      |
//! |       ^                                                               |
//! |       +--- BindLease::drop() 在最后一个 Arc 释放时按 lease_id 注销    |
//! +-----------------------------------------------------------------------+
//! ```
//! - **查找键与对象持有分离**：索引表只保存轻量级整数 `id` 和地址键，使得删除索引或更新 UDP 键时无需克隆或析构底层协议端点。
//! - **自旋锁外扩容与析构**：`TablesState` 由自旋锁（`SpinLock`）保护，临界区内严禁执行堆内存分配、协议状态机操作或最后一个 `Arc` 的析构（Drop）。当 `Vec` 容量不足时，插入方先释放自旋锁，在对应的 `SleepMutex` 保护下调用 `prepare_growth` 预分配双倍容量的空 `Vec`，随后重新加自旋锁完成常数时间的 `append` 与 `swap`。
//!
//! ## 锁获取顺序（Lock Ordering）与并发契约
//! 1. **扩容互斥锁优先于状态自旋锁**：`*_growth`（`SleepMutex`） ➔ `state`（`SpinLock`）。任何路径严禁在持有 `state` 自旋锁时获取睡眠锁或执行可能睡眠的操作。
//! 2. **各表的 `*_growth` 锁相互独立**：每次插入操作仅获取目标表自身的 `*_growth` 睡眠锁，严禁嵌套获取多个 `*_growth` 锁。
//! 3. **表自旋锁与协议端点锁解耦**：`lookup_*` / `take_*` 只在 `state` 自旋锁临界区内拷贝或移出 `Arc` 引用，必须在释放 `state` 锁之后再调用端点的方法或触发 `Arc` 析构。

use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::mem;
use smoltcp::wire::{IpAddress, Ipv6Address};

use crate::fs::FsError;
use crate::net::address::InetEndpoint;
use crate::net::tcp::TcpEndpoint;
use crate::net::udp::UdpEndpoint;
use crate::net::unix::UnixPair;
use crate::net::{NetStack, NetStackRef};
use crate::sync::{SleepMutex, SpinLock};

/// 动态分配临时端口（Ephemeral Port）的起始端口号（IANA 建议范围 49152..=65535）。
const FIRST_EPHEMERAL_PORT: u16 = 49152;

/// 传输层协议类型，用于隔离 TCP 与 UDP 各自独立的端口名字空间。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TransportProtocol {
    /// 面向连接的 TCP 传输协议。
    Tcp,
    /// 无连接的 UDP 数据报协议。
    Udp,
}

/// 已建立连接的 TCP 四元组查找键，由完整的本地端点与远端对端端点组成，与监听套接字索引分离存储。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TcpTuple {
    /// 本地绑定的 IP 地址与端口号（单位：主机字节序端口号）。
    pub local: InetEndpoint,
    /// 远端对端的 IP 地址与端口号（单位：主机字节序端口号）。
    pub peer: InetEndpoint,
}

/// 本地端口绑定的生命周期租约句柄。
///
/// 当持有该租约的最后一个 `Arc<BindLease>` 被释放（Drop）时，自动从所属网络域的绑定表中移除对应记录。
/// TCP 监听套接字派生出的子连接会共享同一个 `Arc<BindLease>`，以确保只要监听者或任意子连接存活，该端口预留就持续有效。
pub struct BindLease {
    /// 该绑定所属的传输层协议（TCP 或 UDP）。
    protocol: TransportProtocol,
    /// 实际绑定的本地 IP 地址与端口（通配地址保持全零值）。
    local: InetEndpoint,
    /// 最初发起端口预留的套接字端点 ID（子连接复用监听者的 `owner_id`）。
    owner_id: u64,
    /// 本次绑定租约的全局唯一 ID，用于精确匹配并注销本次预留记录。
    lease_id: u64,
    /// 所属网络域的弱引用，用于在析构时访问 `NetTables` 而避免循环引用。
    net: Weak<NetStack>,
    /// 标记该租约是否已成功写入 `TablesState::bindings`；若预留中途失败，析构时跳过表注销操作。
    published: bool,
}

impl BindLease {
    /// 返回绑定成功后的本地 IP 地址与端口号（通配绑定时地址部分保持未指定零值）。
    pub fn local(&self) -> InetEndpoint {
        self.local
    }

    /// 返回最初发起该端口绑定的套接字端点 ID（监听套接字派生的子连接沿用原监听者的 `owner_id`）。
    pub fn owner_id(&self) -> u64 {
        self.owner_id
    }

    /// 返回本次绑定预留专属的唯一租约 ID，避免旧在途发送持有的租约在延后释放时误删同一端点的后续新绑定。
    pub fn id(&self) -> u64 {
        self.lease_id
    }
}

impl Drop for BindLease {
    /// 当最后一个 `Arc<BindLease>` 强引用被释放时，若该租约已正式注册到表中，则在表锁外升级网络域弱引用并注销对应的 `lease_id`。
    fn drop(&mut self) {
        if self.published {
            if let Some(net) = self.net.upgrade() {
                net.tables().release_binding(self.lease_id);
            }
        }
    }
}

/// 绑定表内部的单条端口预留记录，受 `NetTables::state` 自旋锁保护。
#[derive(Clone, Copy)]
struct Binding {
    /// 预留该端口的传输层协议类型（TCP 或 UDP）。
    protocol: TransportProtocol,
    /// 绑定的本地 IP 地址与端口号。
    local: InetEndpoint,
    /// 持有该绑定的主套接字端点 ID。
    owner_id: u64,
    /// 唯一标识本次预留实例的租约 ID，与 [`BindLease::lease_id`] 一一对应。
    lease_id: u64,
    /// 当前是否开启了 `SO_REUSEADDR` 地址复用策略。
    reuse: bool,
    /// 当绑定在 IPv6 未指定地址（`::`）时，是否同时接收 IPv4 流量（即 `IPV6_V6ONLY == false`）。
    dual_stack: bool,
}

/// 网络域所有对象表与查找索引的受保护内部状态，由 `NetTables::state` 自旋锁统一保护。
struct TablesState {
    /// 活跃的 TCP/UDP 端口绑定记录列表。
    bindings: Vec<Binding>,
    /// 当前网络域持有的全部活跃 TCP 端点对象列表。
    tcp: Vec<Arc<TcpEndpoint>>,
    /// 已连接 TCP 端点的四元组路由索引，元素为 `(端点 ID, TcpTuple)`。
    tuples: Vec<(u64, TcpTuple)>,
    /// 处于监听状态的 TCP 端点索引，元素为 `(端点 ID, 监听本地端点, 是否双栈通配)`。
    listeners: Vec<(u64, InetEndpoint, bool)>,
    /// 当前网络域持有的全部活跃 UDP 端点对象列表。
    udp: Vec<Arc<UdpEndpoint>>,
    /// UDP 收包匹配索引，元素为 `(端点 ID, 本地绑定端点, 可选已连接对端端点, 是否双栈通配)`。
    udp_keys: Vec<(u64, InetEndpoint, Option<InetEndpoint>, bool)>,
    /// 当前网络域持有的全部活跃匿名 UNIX 套接字对列表。
    unix: Vec<Arc<UnixPair>>,
    /// 下一次分配动态临时端口时的扫描起始游标（范围 `FIRST_EPHEMERAL_PORT..=u16::MAX`）。
    next_port: u16,
    /// 标记网络域是否已进入关闭销毁阶段；置为 `true` 后拒绝一切新绑定与新对象注册。
    closed: bool,
}

/// 网络域的协议端点所有权表与收包查找索引集合。
///
/// ponytail: 各类对象使用独立增长的线性索引，查找竞争影响吞吐时再按真实键拆桶。
pub struct NetTables {
    /// 保护所有内部 `Vec` 表项、端口游标及关闭标志的自旋锁（临界区内不分配内存、不析构复杂对象）。
    state: SpinLock<TablesState>,
    /// 串行化 `bindings` 表插入与锁外容量预分配的睡眠互斥锁；删除和只读查找仅获取 `state` 自旋锁。
    bindings_growth: SleepMutex<()>,
    /// 串行化 `tcp` 对象表插入与锁外容量预分配的睡眠互斥锁。
    tcp_growth: SleepMutex<()>,
    /// 串行化 `tuples` 四元组索引表插入与锁外容量预分配的睡眠互斥锁。
    tuples_growth: SleepMutex<()>,
    /// 串行化 `listeners` 监听索引表插入与锁外容量预分配的睡眠互斥锁。
    listeners_growth: SleepMutex<()>,
    /// 串行化 `udp` 对象表插入与锁外容量预分配的睡眠互斥锁。
    udp_growth: SleepMutex<()>,
    /// 串行化 `udp_keys` 查找键表插入与锁外容量预分配的睡眠互斥锁。
    udp_keys_growth: SleepMutex<()>,
    /// 串行化 `unix` 对象表插入与锁外容量预分配的睡眠互斥锁。
    unix_growth: SleepMutex<()>,
}

impl NetTables {
    /// 创建包含七张空表的 [`NetTables`]，初始不预分配堆块，后续插入操作各自在自旋锁外按需扩容。
    pub fn try_new() -> Result<Self, FsError> {
        Ok(Self {
            state: SpinLock::new(TablesState {
                bindings: Vec::new(),
                tcp: Vec::new(),
                tuples: Vec::new(),
                listeners: Vec::new(),
                udp: Vec::new(),
                udp_keys: Vec::new(),
                unix: Vec::new(),
                next_port: FIRST_EPHEMERAL_PORT,
                closed: false,
            }),
            bindings_growth: SleepMutex::new(()),
            tcp_growth: SleepMutex::new(()),
            tuples_growth: SleepMutex::new(()),
            listeners_growth: SleepMutex::new(()),
            udp_growth: SleepMutex::new(()),
            udp_keys_growth: SleepMutex::new(()),
            unix_growth: SleepMutex::new(()),
        })
    }

    /// 读取当前网络域中 UDP 相关三张表（`udp`、`bindings`、`udp_keys`）的实际长度与容量，供内核测试定位扩容边界。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_udp_storage(&self) -> [(usize, usize); 3] {
        let state = self.state.lock();
        [
            (state.udp.len(), state.udp.capacity()),
            (state.bindings.len(), state.bindings.capacity()),
            (state.udp_keys.len(), state.udp_keys.capacity()),
        ]
    }

    /// 为指定协议端点预留本地地址与端口（若端口为 `0` 则自动选取临时端口），返回管理该绑定生命周期的 [`BindLease`]。
    ///
    /// 在可睡眠上下文中调用。全程持有 `bindings_growth` 互斥锁以串行化 `Arc<BindLease>` 分配与 `bindings` 表的锁外扩容，最后在 `state` 自旋锁临界区内完成冲突复核与原子注册。
    pub fn reserve_binding(
        &self,
        net: &NetStackRef,
        owner_id: u64,
        protocol: TransportProtocol,
        requested: InetEndpoint,
        reuse: bool,
        dual_stack: bool,
    ) -> Result<Arc<BindLease>, FsError> {
        // Step 1: 校验双栈参数合法性以及非通配地址是否属于本网络域接口。
        if dual_stack && requested.address != IpAddress::Ipv6(Ipv6Address::UNSPECIFIED) {
            return Err(FsError::InvalidOperation);
        }
        if !requested.is_unspecified() && !net.is_local(requested.address) {
            return Err(FsError::AddressNotAvailable);
        }

        // Step 2: 获取绑定表插入互斥锁，并在自旋锁外预先分配租约对象。
        let _growth = self.bindings_growth.lock();
        let mut lease = Arc::try_new(BindLease {
            protocol,
            local: requested,
            owner_id,
            lease_id: net.allocate_identity(),
            net: Arc::downgrade(net),
            published: false,
        })
        .map_err(|_| FsError::NoMemory)?;
        let mut candidate = Vec::new();
        // Step 3: 在自旋锁内检查端口冲突或分配临时端口；若表容量已满则释放自旋锁进行倍增扩容后重试。
        loop {
            let mut state = self.state.lock();
            if state.closed {
                return Err(FsError::InvalidOperation);
            }
            let (local, next_port) = if requested.port == 0 {
                choose_ephemeral(&state, protocol, requested, dual_stack)?
            } else {
                if binding_conflicts(&state.bindings, protocol, requested, reuse, dual_stack)
                    || (protocol == TransportProtocol::Tcp
                        && state
                            .listeners
                            .iter()
                            .any(|entry| overlaps(entry.1, entry.2, requested, dual_stack)))
                {
                    return Err(FsError::AddressInUse);
                }
                (requested, state.next_port)
            };

            if state.bindings.len() == state.bindings.capacity() {
                if candidate.capacity() <= state.bindings.len() {
                    let capacity = state.bindings.capacity();
                    drop(state);
                    candidate = prepare_growth(capacity)?;
                    continue;
                }
                candidate.append(&mut state.bindings);
                mem::swap(&mut candidate, &mut state.bindings);
            }

            // Step 4: 此时 `lease` 尚未对外返回，不存在其他强/弱引用，安全更新最终选定的端口并提交绑定记录。
            let prepared = Arc::get_mut(&mut lease).expect("unpublished binding lease shared");
            prepared.local = local;
            state.bindings.push(Binding {
                protocol: prepared.protocol,
                local,
                owner_id,
                lease_id: prepared.lease_id,
                reuse,
                dual_stack,
            });
            state.next_port = next_port;
            prepared.published = true;
            break;
        }
        Ok(lease)
    }

    /// 根据绑定租约 ID（`lease_id`）更新对应绑定记录的 `SO_REUSEADDR` 标志，不影响同一端点早期残留的在途发送租约。
    pub fn set_reuse(&self, lease_id: u64, reuse: bool) -> Result<(), FsError> {
        let mut state = self.state.lock();
        if state.closed {
            return Err(FsError::NotConnected);
        }
        if let Some(binding) = state
            .bindings
            .iter_mut()
            .find(|entry| entry.lease_id == lease_id)
        {
            binding.reuse = reuse;
        }
        Ok(())
    }

    /// 根据唯一租约 ID（`lease_id`）从绑定表中移除对应的端口预留记录。由 [`BindLease`] 析构函数调用。
    pub fn release_binding(&self, lease_id: u64) {
        let mut state = self.state.lock();
        if let Some(index) = state
            .bindings
            .iter()
            .position(|entry| entry.lease_id == lease_id)
        {
            state.bindings.swap_remove(index);
        }
    }

    /// 将新创建的 [`TcpEndpoint`] 注册到网络域对象持有表中。
    ///
    /// 全程持有 `tcp_growth` 睡眠锁以在自旋锁外完成必要的容量倍增；若网络域已关闭或 ID 重复则返回错误，失败时的入参及旧缓冲区均在自旋锁外释放。
    pub fn insert_tcp(&self, endpoint: Arc<TcpEndpoint>) -> Result<(), FsError> {
        let _growth = self.tcp_growth.lock();
        let id = endpoint.id();
        let mut candidate = Vec::new();
        loop {
            let mut state = self.state.lock();
            if state.closed || state.tcp.iter().any(|entry| entry.id() == id) {
                return Err(FsError::InvalidOperation);
            }

            if state.tcp.len() == state.tcp.capacity() {
                if candidate.capacity() <= state.tcp.len() {
                    let capacity = state.tcp.capacity();
                    drop(state);
                    candidate = prepare_growth(capacity)?;
                    continue;
                }
                candidate.append(&mut state.tcp);
                mem::swap(&mut candidate, &mut state.tcp);
            }

            state.tcp.push(endpoint);
            return Ok(());
        }
    }

    /// 将新创建的 [`UdpEndpoint`] 注册到网络域对象持有表中。
    ///
    /// 全程持有 `udp_growth` 睡眠锁以在自旋锁外完成容量扩容；若失败则不在表中保留引用。
    pub fn insert_udp(&self, endpoint: Arc<UdpEndpoint>) -> Result<(), FsError> {
        let _growth = self.udp_growth.lock();
        let id = endpoint.id();
        let mut candidate = Vec::new();
        loop {
            let mut state = self.state.lock();
            if state.closed || state.udp.iter().any(|entry| entry.id() == id) {
                return Err(FsError::InvalidOperation);
            }

            if state.udp.len() == state.udp.capacity() {
                if candidate.capacity() <= state.udp.len() {
                    let capacity = state.udp.capacity();
                    drop(state);
                    candidate = prepare_growth(capacity)?;
                    continue;
                }
                candidate.append(&mut state.udp);
                mem::swap(&mut candidate, &mut state.udp);
            }

            state.udp.push(endpoint);
            return Ok(());
        }
    }

    /// 将初始化完毕的匿名 [`UnixPair`] 注册到网络域对象持有表中。
    ///
    /// 全程持有 `unix_growth` 睡眠锁进行锁外扩容准备，表扩容搬移元素时仅移动指针而不触发引用计数增减。
    pub fn insert_unix(&self, pair: Arc<UnixPair>) -> Result<(), FsError> {
        let _growth = self.unix_growth.lock();
        let id = pair.id();
        let mut candidate = Vec::new();
        loop {
            let mut state = self.state.lock();
            if state.closed || state.unix.iter().any(|entry| entry.id() == id) {
                return Err(FsError::InvalidOperation);
            }

            if state.unix.len() == state.unix.capacity() {
                if candidate.capacity() <= state.unix.len() {
                    let capacity = state.unix.capacity();
                    drop(state);
                    candidate = prepare_growth(capacity)?;
                    continue;
                }
                candidate.append(&mut state.unix);
                mem::swap(&mut candidate, &mut state.unix);
            }

            state.unix.push(pair);
            return Ok(());
        }
    }

    /// 为已登记的 TCP 端点注册完整的四元组（`TcpTuple`）收包路由索引。
    ///
    /// 在 `tuples_growth` 睡眠锁保护下按需扩容，并在 `state` 自旋锁内校验端点存在性与四元组唯一性后提交。
    pub fn publish_tcp_tuple(&self, id: u64, tuple: TcpTuple) -> Result<(), FsError> {
        if tuple.local.address.version() != tuple.peer.address.version()
            || tuple.local.is_unspecified()
            || tuple.peer.is_unspecified()
            || tuple.local.port == 0
            || tuple.peer.port == 0
        {
            return Err(FsError::InvalidOperation);
        }

        let _growth = self.tuples_growth.lock();
        let mut candidate = Vec::new();
        loop {
            let mut state = self.state.lock();
            if state.closed || !state.tcp.iter().any(|entry| entry.id() == id) {
                return Err(FsError::BadFd);
            }
            if state.tuples.iter().any(|entry| entry.1 == tuple) {
                return Err(FsError::AddressInUse);
            }
            if state.tuples.iter().any(|entry| entry.0 == id) {
                return Err(FsError::InvalidOperation);
            }

            if state.tuples.len() == state.tuples.capacity() {
                if candidate.capacity() <= state.tuples.len() {
                    let capacity = state.tuples.capacity();
                    drop(state);
                    candidate = prepare_growth(capacity)?;
                    continue;
                }
                candidate.append(&mut state.tuples);
                mem::swap(&mut candidate, &mut state.tuples);
            }

            state.tuples.push((id, tuple));
            return Ok(());
        }
    }

    /// 将已绑定端口的 TCP 端点正式注册到监听索引表（`listeners`）中。
    ///
    /// 在 `listeners_growth` 睡眠锁保护下准备容量，并在 `state` 自旋锁内核对绑定租约归属、端口复用策略以及是否存在重叠的监听者。
    pub fn publish_listener(&self, id: u64, local: InetEndpoint) -> Result<(), FsError> {
        let _growth = self.listeners_growth.lock();
        let mut candidate = Vec::new();
        loop {
            let mut state = self.state.lock();
            if state.closed || !state.tcp.iter().any(|entry| entry.id() == id) {
                return Err(FsError::BadFd);
            }
            let binding = state
                .bindings
                .iter()
                .find(|entry| entry.owner_id == id)
                .ok_or(FsError::BadFd)?;
            if binding.protocol != TransportProtocol::Tcp || binding.local != local {
                return Err(FsError::InvalidOperation);
            }
            let dual_stack = binding.dual_stack;
            if state
                .listeners
                .iter()
                .any(|entry| overlaps(entry.1, entry.2, local, dual_stack))
                || state.bindings.iter().any(|entry| {
                    entry.owner_id != id
                        && entry.protocol == TransportProtocol::Tcp
                        && overlaps(entry.local, entry.dual_stack, local, dual_stack)
                        && !(binding.reuse && entry.reuse)
                })
            {
                return Err(FsError::AddressInUse);
            }
            if local.port == 0 || state.listeners.iter().any(|entry| entry.0 == id) {
                return Err(FsError::InvalidOperation);
            }

            if state.listeners.len() == state.listeners.capacity() {
                if candidate.capacity() <= state.listeners.len() {
                    let capacity = state.listeners.capacity();
                    drop(state);
                    candidate = prepare_growth(capacity)?;
                    continue;
                }
                candidate.append(&mut state.listeners);
                mem::swap(&mut candidate, &mut state.listeners);
            }

            state.listeners.push((id, local, dual_stack));
            return Ok(());
        }
    }

    /// 发布或更新指定 UDP 端点的收包匹配键（本地绑定地址与可选的远端连接地址）。
    ///
    /// 若该端点此前已发布过匹配键，则先移除旧项再追加至末尾，从而保证存在多个同优先级匹配时以最近发布的端点为准。
    pub fn publish_udp(
        &self,
        id: u64,
        local: InetEndpoint,
        peer: Option<InetEndpoint>,
    ) -> Result<(), FsError> {
        let _growth = self.udp_keys_growth.lock();
        let mut candidate = Vec::new();
        loop {
            let mut state = self.state.lock();
            if state.closed || !state.udp.iter().any(|entry| entry.id() == id) {
                return Err(FsError::BadFd);
            }
            if local.port == 0 {
                return Err(FsError::InvalidOperation);
            }
            let dual_stack = state
                .bindings
                .iter()
                .find(|entry| {
                    entry.owner_id == id
                        && entry.protocol == TransportProtocol::Udp
                        && matches_local(entry.local, entry.dual_stack, local)
                })
                .ok_or(FsError::BadFd)?
                .dual_stack;

            if let Some(index) = state.udp_keys.iter().position(|entry| entry.0 == id) {
                state.udp_keys.remove(index);
            } else if state.udp_keys.len() == state.udp_keys.capacity() {
                if candidate.capacity() <= state.udp_keys.len() {
                    let capacity = state.udp_keys.capacity();
                    drop(state);
                    candidate = prepare_growth(capacity)?;
                    continue;
                }
                candidate.append(&mut state.udp_keys);
                mem::swap(&mut candidate, &mut state.udp_keys);
            }

            state.udp_keys.push((id, local, peer, dual_stack));
            return Ok(());
        }
    }

    /// 移除指定 UDP 端点的收包匹配键（例如断开连接并回退自动绑定时），底层端点所有权与端口租约仍由各自持有者管理。
    pub fn unpublish_udp(&self, id: u64) {
        self.state.lock().udp_keys.retain(|entry| entry.0 != id);
    }

    /// 根据完整的 TCP 四元组查找对应的 [`TcpEndpoint`] 强引用；调用方在释放表自旋锁后再推进协议逻辑。
    pub fn lookup_tcp(&self, tuple: TcpTuple) -> Option<Arc<TcpEndpoint>> {
        let state = self.state.lock();
        let id = state.tuples.iter().find(|entry| entry.1 == tuple)?.0;
        state.tcp.iter().find(|entry| entry.id() == id).cloned()
    }

    /// 根据端点 ID 查找对应的 [`TcpEndpoint`] 强引用，供延时定时器或发送队列容量恢复通知定向唤醒该连接。
    pub fn tcp_by_id(&self, id: u64) -> Option<Arc<TcpEndpoint>> {
        let state = self.state.lock();
        state.tcp.iter().find(|entry| entry.id() == id).cloned()
    }

    /// 根据目标本地地址与端口查找匹配的监听 [`TcpEndpoint`]，精确 IP 地址匹配优先于通配地址监听者。
    pub fn lookup_listener(&self, local: InetEndpoint) -> Option<Arc<TcpEndpoint>> {
        let state = self.state.lock();
        let id = lookup_local(&state.listeners, local)?;
        state.tcp.iter().find(|entry| entry.id() == id).cloned()
    }

    /// 根据收包的目的端点（`local`）与源端点（`peer`）查找最匹配的 [`UdpEndpoint`]。
    ///
    /// 匹配优先级从高到低依次为：绑定具体本地 IP 地址 > 已连接指定远端对端（Connected Peer） > 最近一次发布的端点（数组下标最大）。
    pub fn lookup_udp(&self, local: InetEndpoint, peer: InetEndpoint) -> Option<Arc<UdpEndpoint>> {
        let state = self.state.lock();
        let id = state
            .udp_keys
            .iter()
            .enumerate()
            .filter(|(_, entry)| {
                matches_local(entry.1, entry.3, local)
                    && entry.2.is_none_or(|expected| expected == peer)
            })
            .max_by_key(|(index, entry)| {
                (
                    (
                        u8::from(!entry.1.is_unspecified()),
                        u8::from(entry.2.is_some()),
                    ),
                    *index,
                )
            })
            .map(|(_, entry)| entry.0)?;
        state.udp.iter().find(|entry| entry.id() == id).cloned()
    }

    /// 根据端点 ID 查找对应的 [`UdpEndpoint`] 强引用，供发送路径容量恢复通知在表锁外唤醒等待队列。
    pub fn udp_by_id(&self, id: u64) -> Option<Arc<UdpEndpoint>> {
        let state = self.state.lock();
        state.udp.iter().find(|entry| entry.id() == id).cloned()
    }

    /// 从表中移除指定 ID 的 TCP 端点及其关联的全部四元组与监听索引，并将端点 `Arc` 移交给调用方在自旋锁外处理销毁。
    pub fn take_tcp(&self, id: u64) -> Option<Arc<TcpEndpoint>> {
        let mut state = self.state.lock();
        state.tuples.retain(|entry| entry.0 != id);
        state.listeners.retain(|entry| entry.0 != id);
        let index = state.tcp.iter().position(|entry| entry.id() == id)?;
        Some(state.tcp.swap_remove(index))
    }

    /// 从表中移除指定 ID 的 UDP 端点及其收包匹配键，并将端点 `Arc` 移交给调用方在自旋锁外析构。
    pub fn take_udp(&self, id: u64) -> Option<Arc<UdpEndpoint>> {
        let mut state = self.state.lock();
        state.udp_keys.retain(|entry| entry.0 != id);
        let index = state.udp.iter().position(|entry| entry.id() == id)?;
        Some(state.udp.swap_remove(index))
    }

    /// 当匿名 UNIX 套接字对的两端均已关闭后，从表中移出该 [`UnixPair`] 的 `Arc` 强引用，交由调用方在自旋锁外释放。
    pub fn take_unix(&self, id: u64) -> Option<Arc<UnixPair>> {
        let mut state = self.state.lock();
        let index = state.unix.iter().position(|entry| entry.id() == id)?;
        Some(state.unix.swap_remove(index))
    }

    /// 将表状态标记为已关闭，清空所有收包查找索引，并一次性移出全部 TCP、UDP 与 UNIX 对象列表供网络域停机流程在锁外同步清理。
    #[expect(
        clippy::type_complexity,
        reason = "唯一停止调用者立即解构三类原拥有，在表锁外分别同步停止 TCP、UDP 与 UNIX"
    )]
    pub fn take_all(
        &self,
    ) -> (
        Vec<Arc<TcpEndpoint>>,
        Vec<Arc<UdpEndpoint>>,
        Vec<Arc<UnixPair>>,
    ) {
        let mut state = self.state.lock();
        state.closed = true;
        state.tuples.clear();
        state.listeners.clear();
        state.udp_keys.clear();

        (
            mem::take(&mut state.tcp),
            mem::take(&mut state.udp),
            mem::take(&mut state.unix),
        )
    }
}

/// 按照原容量的两倍（至少为 1）分配新的空 `Vec` 缓冲区；必须在 `state` 自旋锁外调用。
fn prepare_growth<T>(capacity: usize) -> Result<Vec<T>, FsError> {
    let capacity = capacity.checked_mul(2).ok_or(FsError::NoMemory)?.max(1);
    let mut entries = Vec::new();
    entries
        .try_reserve_exact(capacity)
        .map_err(|_| FsError::NoMemory)?;
    Ok(entries)
}

/// 检查待绑定的本地端点是否与现有同协议绑定记录冲突（仅当重叠双方均开启 `SO_REUSEADDR` 时允许共存）。
fn binding_conflicts(
    bindings: &[Binding],
    protocol: TransportProtocol,
    local: InetEndpoint,
    reuse: bool,
    dual_stack: bool,
) -> bool {
    bindings.iter().any(|entry| {
        entry.protocol == protocol
            && overlaps(entry.local, entry.dual_stack, local, dual_stack)
            && !(reuse && entry.reuse)
    })
}

/// 判断两个本地绑定端点在接收流量范围上是否存在交集（含同协议族通配、相同具体地址，或双栈 IPv6 通配对 IPv4 地址的覆盖）。
fn overlaps(left: InetEndpoint, left_dual: bool, right: InetEndpoint, right_dual: bool) -> bool {
    matches_local(left, left_dual, right) || matches_local(right, right_dual, left)
}

/// 判断单条绑定规则（`binding` 及其 `dual_stack` 标志）是否覆盖指定的目标本地端点（`local`）。
fn matches_local(binding: InetEndpoint, dual_stack: bool, local: InetEndpoint) -> bool {
    binding.port == local.port
        && (binding.address == local.address
            || (binding.is_unspecified()
                && (binding.address.version() == local.address.version()
                    || (dual_stack && matches!(local.address, IpAddress::Ipv4(_))))))
}

/// 从 `state.next_port` 开始循环扫描，为端口为 `0` 的绑定请求挑选一个不与现有绑定及 TCP 监听者冲突的动态临时端口，并返回更新后的游标。
fn choose_ephemeral(
    state: &TablesState,
    protocol: TransportProtocol,
    requested: InetEndpoint,
    dual_stack: bool,
) -> Result<(InetEndpoint, u16), FsError> {
    let start = state.next_port;
    let mut next_port = start;
    loop {
        let local = InetEndpoint {
            port: next_port,
            ..requested
        };
        next_port = if next_port == u16::MAX {
            FIRST_EPHEMERAL_PORT
        } else {
            next_port + 1
        };
        if !(binding_conflicts(&state.bindings, protocol, local, false, dual_stack)
            || protocol == TransportProtocol::Tcp
                && state
                    .listeners
                    .iter()
                    .any(|entry| overlaps(entry.1, entry.2, local, dual_stack)))
        {
            return Ok((local, next_port));
        }
        if next_port == start {
            return Err(FsError::AddressInUse);
        }
    }
}

/// 在监听索引列表中查找覆盖目标本地端点（`local`）的端点 ID，绑定了具体 IP 地址的项优先于通配地址项。
fn lookup_local(entries: &[(u64, InetEndpoint, bool)], local: InetEndpoint) -> Option<u64> {
    entries
        .iter()
        .filter(|entry| matches_local(entry.1, entry.2, local))
        .max_by_key(|entry| u8::from(!entry.1.is_unspecified()))
        .map(|entry| entry.0)
}
