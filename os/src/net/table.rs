//! 本域绑定、协议拥有与查找键分别保存；短锁不执行协议或最后对象析构。

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

const FIRST_EPHEMERAL_PORT: u16 = 49152;

/// TCP 和 UDP 使用互不冲突的端口空间。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TransportProtocol {
    Tcp,
    Udp,
}

/// 已连接 TCP 以完整本地与远端端点查找，不与监听键混用。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TcpTuple {
    pub local: InetEndpoint,
    pub peer: InetEndpoint,
}

/// 最后真实预留拥有结束时撤绑定；监听孩子共享同一个 Arc。
pub struct BindLease {
    protocol: TransportProtocol,
    local: InetEndpoint,
    owner_id: u64,
    lease_id: u64,
    net: Weak<NetStack>,
    /// 准备中的 Arc 尚未发表，失败析构不能撤其它真实记录。
    published: bool,
}

impl BindLease {
    /// 返回绑定时选定的地址和端口，通配地址保持零值。
    pub fn local(&self) -> InetEndpoint {
        self.local
    }

    /// 返回原端口预留者身份；孩子不获得新的绑定身份。
    pub fn owner_id(&self) -> u64 {
        self.owner_id
    }

    /// 每次预留使用独立不复用身份，旧发送的最后租约不能撤除后继绑定。
    pub fn id(&self) -> u64 {
        self.lease_id
    }
}

impl Drop for BindLease {
    /// 只撤已经发表的预留，弱域升级及最终域引用析构均在表锁外。
    fn drop(&mut self) {
        if self.published {
            if let Some(net) = self.net.upgrade() {
                net.tables().release_binding(self.lease_id);
            }
        }
    }
}

#[derive(Clone, Copy)]
struct Binding {
    protocol: TransportProtocol,
    local: InetEndpoint,
    owner_id: u64,
    lease_id: u64,
    reuse: bool,
    dual_stack: bool,
}

struct TablesState {
    bindings: Vec<Binding>,
    tcp: Vec<Arc<TcpEndpoint>>,
    tuples: Vec<(u64, TcpTuple)>,
    listeners: Vec<(u64, InetEndpoint, bool)>,
    udp: Vec<Arc<UdpEndpoint>>,
    udp_keys: Vec<(u64, InetEndpoint, Option<InetEndpoint>, bool)>,
    unix: Vec<Arc<UnixPair>>,
    next_port: u16,
    closed: bool,
}

/// ponytail: 各类对象使用独立增长的线性索引，查找竞争影响吞吐时再按真实键拆桶。
pub struct NetTables {
    state: SpinLock<TablesState>,
    /// 每类插入都从准备到提交持有自己的互斥；删除和查找只取 state。
    bindings_growth: SleepMutex<()>,
    tcp_growth: SleepMutex<()>,
    tuples_growth: SleepMutex<()>,
    listeners_growth: SleepMutex<()>,
    udp_growth: SleepMutex<()>,
    udp_keys_growth: SleepMutex<()>,
    unix_growth: SleepMutex<()>,
}

impl NetTables {
    /// 建立七个空表；后继插入只为自身索引在短锁外申请后备。
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
    /// 只读本域 UDP 相关表的实际长度和后备容量，消费者据此选择真正的增长边界。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_udp_storage(&self) -> [(usize, usize); 3] {
        let state = self.state.lock();
        [
            (state.udp.len(), state.udp.capacity()),
            (state.bindings.len(), state.bindings.capacity()),
            (state.udp_keys.len(), state.udp_keys.capacity()),
        ]
    }

    /// 插入互斥覆盖租约和后备准备；短锁复核绑定冲突后一次发表租约身份与端口。
    pub fn reserve_binding(
        &self,
        net: &NetStackRef,
        owner_id: u64,
        protocol: TransportProtocol,
        requested: InetEndpoint,
        reuse: bool,
        dual_stack: bool,
    ) -> Result<Arc<BindLease>, FsError> {
        if dual_stack && requested.address != IpAddress::Ipv6(Ipv6Address::UNSPECIFIED) {
            return Err(FsError::InvalidOperation);
        }
        if !requested.is_unspecified() && !net.is_local(requested.address) {
            return Err(FsError::AddressNotAvailable);
        }
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
            // 尚无外部强引用或弱引用，只有准备者能修改最终端点。
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

    /// 按当前租约身份更新已有策略，不影响同 owner 的旧在途发送租约。
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

    /// 最后租约只撤自己的绑定记录，旧租约释放不撤同 owner 的后继绑定。
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

    /// 插入互斥串行 TCP 拥有准备；短锁复核身份，失败参数及旧后备均在锁外析构。
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

    /// 插入互斥串行 UDP 拥有准备；端口索引独立增长，失败不取得节点拥有。
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

    /// 完整 pair 在专用插入互斥内取得表拥有，移动时不克隆或释放端点引用。
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

    /// tuple 插入串行准备后备，短锁重新核对端点身份及完整键唯一性后才提交。
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

    /// 监听索引独立准备后备，重取短锁后复核租约、复用策略和重叠监听者。
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

    /// UDP 插入串行准备后备；同身份更新复用原槽并移至末尾，保持最近发表优先。
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

    /// 断开自动绑定只撤本身份的查询键，节点拥有和租约由各自拥有者结束。
    pub fn unpublish_udp(&self, id: u64) {
        self.state.lock().udp_keys.retain(|entry| entry.0 != id);
    }

    /// 查完整 tuple 并取得真实节点引用；协议调用由持引用者在解锁后执行。
    pub fn lookup_tcp(&self, tuple: TcpTuple) -> Option<Arc<TcpEndpoint>> {
        let state = self.state.lock();
        let id = state.tuples.iter().find(|entry| entry.1 == tuple)?.0;
        state.tcp.iter().find(|entry| entry.id() == id).cloned()
    }

    /// 按真实身份取得拥有表引用，容量恢复通知在解锁后只安排这个连接。
    pub fn tcp_by_id(&self, id: u64) -> Option<Arc<TcpEndpoint>> {
        let state = self.state.lock();
        state.tcp.iter().find(|entry| entry.id() == id).cloned()
    }

    /// 先查具体地址再查通配 listener，查找不改变监听者状态。
    pub fn lookup_listener(&self, local: InetEndpoint) -> Option<Arc<TcpEndpoint>> {
        let state = self.state.lock();
        let id = lookup_local(&state.listeners, local)?;
        state.tcp.iter().find(|entry| entry.id() == id).cloned()
    }

    /// 具体本地地址优先，其次 connected peer，同评分取最近发表的真实节点。
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

    /// 定向容量通知只取得这个 UDP 身份，解表锁后才访问接收事实或持续源。
    pub fn udp_by_id(&self, id: u64) -> Option<Arc<UdpEndpoint>> {
        let state = self.state.lock();
        state.udp.iter().find(|entry| entry.id() == id).cloned()
    }

    /// 一次撤全部该 TCP 身份的索引并移出真实拥有，Lease 寿命保持独立。
    pub fn take_tcp(&self, id: u64) -> Option<Arc<TcpEndpoint>> {
        let mut state = self.state.lock();
        state.tuples.retain(|entry| entry.0 != id);
        state.listeners.retain(|entry| entry.0 != id);
        let index = state.tcp.iter().position(|entry| entry.id() == id)?;
        Some(state.tcp.swap_remove(index))
    }

    /// 撤 UDP 本地键并移出真实拥有，最后节点析构由调用者在锁外承担。
    pub fn take_udp(&self, id: u64) -> Option<Arc<UdpEndpoint>> {
        let mut state = self.state.lock();
        state.udp_keys.retain(|entry| entry.0 != id);
        let index = state.udp.iter().position(|entry| entry.id() == id)?;
        Some(state.udp.swap_remove(index))
    }

    /// 两端均已关闭后移出根 pair 拥有，最后释放发生在调用者的锁外作用域。
    pub fn take_unix(&self, id: u64) -> Option<Arc<UnixPair>> {
        let mut state = self.state.lock();
        let index = state.unix.iter().position(|entry| entry.id() == id)?;
        Some(state.unix.swap_remove(index))
    }

    /// 唯一整域停止关闭发表并移动三类拥有；绑定只随最后 Lease 结束。
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

/// 按单表原容量几何准备空后备；只能在 state 短锁外调用。
fn prepare_growth<T>(capacity: usize) -> Result<Vec<T>, FsError> {
    let capacity = capacity.checked_mul(2).ok_or(FsError::NoMemory)?.max(1);
    let mut entries = Vec::new();
    entries
        .try_reserve_exact(capacity)
        .map_err(|_| FsError::NoMemory)?;
    Ok(entries)
}

/// 同协议的重叠绑定仅在双方显式允许复用时不冲突。
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

/// 两接收面只有同族通配、相同具体值或双面通配的 IPv4 别名才重叠。
fn overlaps(left: InetEndpoint, left_dual: bool, right: InetEndpoint, right_dual: bool) -> bool {
    matches_local(left, left_dual, right) || matches_local(right, right_dual, left)
}

/// 绑定冲突、端口选择和接收查询共用同一纯地址覆盖规则。
fn matches_local(binding: InetEndpoint, dual_stack: bool, local: InetEndpoint) -> bool {
    binding.port == local.port
        && (binding.address == local.address
            || (binding.is_unspecified()
                && (binding.address.version() == local.address.version()
                    || (dual_stack && matches!(local.address, IpAddress::Ipv4(_))))))
}

/// 只试选动态端口与后继游标；调用者在容量和冲突复核通过后一起提交。
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

/// 地址精确键优先于同端口通配键，返回值不借用索引存储。
fn lookup_local(entries: &[(u64, InetEndpoint, bool)], local: InetEndpoint) -> Option<u64> {
    entries
        .iter()
        .filter(|entry| matches_local(entry.1, entry.2, local))
        .max_by_key(|entry| u8::from(!entry.1.is_unspecified()))
        .map(|entry| entry.0)
}
