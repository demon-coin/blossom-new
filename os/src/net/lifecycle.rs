//! 等待链只持轻量节点；退休链沿真实协议对象的独立投影保活回调全尾。

use alloc::sync::Arc;

use crate::fs::FsError;
use crate::net::tcp::TcpEndpoint;
use crate::net::udp::UdpEndpoint;
use crate::sync::SpinLock;

/// 节点在端点创建时准备，等待拥有不延长协议对象寿命。
pub(crate) struct EgressWait {
    id: u64,
    projection: SpinLock<WaitProjection>,
}

struct WaitProjection {
    linked: bool,
    next: Option<Arc<EgressWait>>,
}

impl EgressWait {
    /// 完整准备不可变身份及空投影，失败不发表端点。
    pub(crate) fn try_new(id: u64) -> Result<Arc<Self>, FsError> {
        Arc::try_new(Self {
            id,
            projection: SpinLock::new(WaitProjection {
                linked: false,
                next: None,
            }),
        })
        .map_err(|_| FsError::NoMemory)
    }

    /// 取得原端点身份，恢复者只据此定向查找真实业务对象。
    pub(crate) fn id(&self) -> u64 {
        self.id
    }

    /// 查询登记投影；根容量锁覆盖检查与登记整个边界。
    pub(crate) fn linked(&self) -> bool {
        self.projection.lock().linked
    }
}

impl Drop for EgressWait {
    /// 末拥有前必须摘链，避免递归释放剩余等待者。
    fn drop(&mut self) {
        let projection = self.projection.lock();
        assert!(
            !projection.linked && projection.next.is_none(),
            "linked egress waiter dropped"
        );
    }
}

/// 全部方法由根 egress 短锁串行，投影锁不反向取得根锁。
pub(crate) struct EgressWaiting {
    head: Option<Arc<EgressWait>>,
    tail: Option<Arc<EgressWait>>,
    len: usize,
}

impl EgressWaiting {
    /// 空链不预留任何总量后备。
    pub(crate) const fn new() -> Self {
        Self {
            head: None,
            tail: None,
            len: 0,
        }
    }

    /// 返回本轮入口长度，恢复轮不追逐通知后重新登记的节点。
    pub(crate) fn len(&self) -> usize {
        self.len
    }

    /// 重复登记保持原链位置，不分配或克隆业务对象。
    pub(crate) fn push(&mut self, node: &Arc<EgressWait>) {
        let mut projection = node.projection.lock();
        if projection.linked {
            return;
        }
        assert!(
            projection.next.is_none(),
            "unlinked waiter retains successor"
        );
        projection.linked = true;
        drop(projection);
        if let Some(tail) = self.tail.as_ref() {
            tail.projection.lock().next = Some(node.clone());
        } else {
            self.head = Some(node.clone());
        }
        self.tail = Some(node.clone());
        self.len = self
            .len
            .checked_add(1)
            .expect("egress waiter count overflow");
    }

    /// 摘首先断 next，返回的实际 Arc 由根锁外通知并释放。
    pub(crate) fn pop(&mut self) -> Option<Arc<EgressWait>> {
        let node = self.head.take()?;
        let mut projection = node.projection.lock();
        assert!(projection.linked, "egress head is not linked");
        self.head = projection.next.take();
        projection.linked = false;
        drop(projection);
        self.len -= 1;
        if self.head.is_none() {
            self.tail.take();
        }
        Some(node)
    }

    /// 身份查询只观察同一链，不把编号当作业务拥有。
    #[cfg(feature = "ktest")]
    pub(crate) fn contains(&self, id: u64) -> bool {
        let mut current = self.head.clone();
        while let Some(node) = current {
            if node.id == id {
                return true;
            }
            current = node.projection.lock().next.clone();
        }
        false
    }

    /// 取消只摘自己的节点，调用者在根锁外释放返回拥有，不等待回调。
    pub(crate) fn remove(&mut self, id: u64) -> Option<Arc<EgressWait>> {
        if self.head.as_ref().is_some_and(|node| node.id == id) {
            return self.pop();
        }
        let mut previous = self.head.clone()?;
        loop {
            let node = previous.projection.lock().next.clone()?;
            if node.id == id {
                let mut projection = node.projection.lock();
                let next = projection.next.take();
                projection.linked = false;
                drop(projection);
                previous.projection.lock().next = next;
                if self.tail.as_ref().is_some_and(|tail| tail.id == id) {
                    self.tail = Some(previous);
                }
                self.len -= 1;
                return Some(node);
            }
            previous = node;
        }
    }
}

impl Drop for EgressWaiting {
    /// 正式停止必须逐节点摘空，不能依赖链式递归析构。
    fn drop(&mut self) {
        assert!(
            self.head.is_none() && self.tail.is_none() && self.len == 0,
            "egress waiting chain not drained"
        );
    }
}

/// 只有表中成功移出的具体节点才能进入退休链。
pub(crate) enum RetiredEndpoint {
    Tcp(Arc<TcpEndpoint>),
    Udp(Arc<UdpEndpoint>),
}

struct RetirementProjection {
    linked: bool,
    next: Option<RetiredEndpoint>,
}

/// 投影与协议状态分离，不取得整个端点的可变引用。
pub(crate) struct RetirementLink {
    projection: SpinLock<RetirementProjection>,
}

impl RetirementLink {
    /// 准备空投影；入链与摘首均由根退休短锁串行。
    pub(crate) const fn new() -> Self {
        Self {
            projection: SpinLock::new(RetirementProjection {
                linked: false,
                next: None,
            }),
        }
    }
}

impl Drop for RetirementLink {
    /// 最后协议拥有释放前，回收者必须已断开退休投影。
    fn drop(&mut self) {
        let projection = self.projection.lock();
        assert!(
            !projection.linked && projection.next.is_none(),
            "linked protocol endpoint dropped"
        );
    }
}

impl RetiredEndpoint {
    /// 只借具体节点的独立投影，不借用协议核心或输入队列。
    fn link(&self) -> &RetirementLink {
        match self {
            Self::Tcp(endpoint) => &endpoint.retirement,
            Self::Udp(endpoint) => &endpoint.retirement,
        }
    }

    /// 根锁外的本地拥有覆盖 Timer 和 Work 的同步停止全尾。
    pub(crate) fn stop_sync(self) {
        match self {
            Self::Tcp(endpoint) => endpoint.stop_sync(),
            Self::Udp(endpoint) => endpoint.stop_sync(),
        }
    }
}

/// 根只保存链首，节点自身的退休投影保活后继。
pub(crate) struct RetiredEndpoints {
    head: Option<RetiredEndpoint>,
}

impl RetiredEndpoints {
    /// 空退休链不按产品端点总量分配。
    pub(crate) const fn new() -> Self {
        Self { head: None }
    }

    /// 已从表移出的同一 Arc 入链一次；当前协议回调不停止自身。
    pub(crate) fn push(&mut self, endpoint: RetiredEndpoint) {
        let mut projection = endpoint.link().projection.lock();
        assert!(
            !projection.linked && projection.next.is_none(),
            "protocol endpoint retired twice"
        );
        projection.linked = true;
        projection.next = self.head.take();
        drop(projection);
        self.head = Some(endpoint);
    }

    /// 摘取前断后继，局部返回拥有在根锁外停止并 put。
    pub(crate) fn pop(&mut self) -> Option<RetiredEndpoint> {
        let endpoint = self.head.take()?;
        let mut projection = endpoint.link().projection.lock();
        assert!(projection.linked, "retired head is not linked");
        self.head = projection.next.take();
        projection.linked = false;
        drop(projection);
        Some(endpoint)
    }

    /// 回收批结束后只查询是否还有真实拥有需要后继票。
    pub(crate) fn is_empty(&self) -> bool {
        self.head.is_none()
    }
}

impl Drop for RetiredEndpoints {
    /// 正式停止必须逐项停止全尾，不能递归释放业务链。
    fn drop(&mut self) {
        assert!(self.head.is_none(), "retired endpoint chain not drained");
    }
}
