//! 逻辑请求拥有原页；生成者与已登记 child 共同决定唯一父完成。

use alloc::sync::Arc;
use core::pin::{pin, Pin};
use core::sync::atomic::{AtomicBool, Ordering};

use core::cell::UnsafeCell;

use crate::block::{BioAllocator, BioMemory};
use crate::block::{
    BlkOperation, BlockError, Disk, DiskIdentity, DiskRef, EndIo, FsClaim, IoResult,
};
use crate::fs::background::WritebackDomain;
use crate::mm::dma::{DmaDomain, MappedPages, PayloadRange};
use crate::sched;
use crate::sync::SpinLock;
use crate::wait::{WaitNode, WaitQueue, Waiter};
use crate::workqueue::Work;

/// 唯一可提交凭证；完成观察者只克隆同一父对象，不能重新提交。
pub struct Bio {
    pub(super) inner: BioArc,
}

/// 完成观察者不拥有 Disk，也不创建新的等待存储或内存副本。
#[derive(Clone)]
pub struct BioCompletion {
    inner: BioArc,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Phase {
    New,
    Beginning,
    Active,
    Finishing,
    Complete,
}

struct BioState {
    phase: Phase,
    memory: Option<BioMemory>,
    remaining: usize,
    generating: bool,
    registered_end: usize,
    registered_children: usize,
    accepted_end: usize,
    first_error: Option<(usize, BlockError)>,
    result: Option<IoResult>,
    completion_owner: Option<DiskRef>,
    fs_claim: Option<FsClaim>,
    finalizer_ticket: Option<BioRawTicket>,
    writeback_domain: Option<Arc<WritebackDomain>>,
}

/// 不可变范围与常数完成账本分开；队列 part 只强拥有此对象。
pub(super) struct BioInner {
    target: DiskIdentity,
    domain: Option<DmaDomain>,
    pub(super) raw_writer_next: UnsafeCell<*const BioInner>,
    operation: BlkOperation,
    sector: u64,
    len: usize,
    segment_count: usize,
    end_io: Option<EndIo>,
    finalizer_work: Option<Pin<Arc<Work>>>,
    state: SpinLock<BioState>,
    ready: AtomicBool,
    waiters: WaitQueue,
}

// SAFETY: raw_writer_next 只在目标 Disk.queue 锁下读写，accepted 生成者保活链节点；
// 其余可变状态由 state 锁保护，来源外部访问期由每个 child 的原账本结束。
unsafe impl Send for BioInner {}
// SAFETY: 共享引用不访问 payload，裸链的同步和寿命遵循同一个物理队列锁。
unsafe impl Sync for BioInner {}

/// 所有父、子请求及观察者同时携带原 allocator 拥有。
pub(super) type BioArc = Arc<BioInner, BioAllocator>;

/// 一张真实 raw strong 与其转出的原 allocator 值共同移动。
struct BioRawTicket {
    pointer: *const BioInner,
    allocator: BioAllocator,
}

// SAFETY: pointer 来自真实 into_raw_with_allocator strong；票只由一次
// Work 移交，状态锁保护其存取，allocator 覆盖回调及真正 deallocate。
unsafe impl Send for BioRawTicket {}

impl BioRawTicket {
    /// 消费真 Arc 并保留原 allocator，不新建计数或猜来源。
    fn from_arc(owner: BioArc) -> Self {
        let (pointer, allocator) = Arc::into_raw_with_allocator(owner);
        Self { pointer, allocator }
    }

    /// 同一票一次恢复原 strong 与 allocator owning。
    fn into_arc(self) -> BioArc {
        // SAFETY: 本票的 pointer/allocator 从同一次真实 into_raw 转移；
        // 没有其它 from_raw 消费者，allocator 值恰好移回这一份 Arc。
        unsafe { Arc::from_raw_in(self.pointer, self.allocator) }
    }
}

impl Bio {
    /// 在普通可切换 Task、所有队列锁外校验原映射并准备全部 fallible 完成存储。
    /// 失败不发表请求，folio 以原错误结束读入责任，原页来源按原阶归还。
    ///
    /// # Safety
    /// Write 的整个范围必须已初始化；独占来源的全部 CPU 裸别名须结束至真实收权。
    /// 共享文件 WB 来源必须持有对应原页及映射撤权完成的凭证，之后原始写会记录新 Dirty，
    /// 不允许导出内容切片。每个输入映射须为 CPU-owned，移交来源完整设备访问责任；
    /// Read 的读入者尚未初始化完成，短结果不授予未写尾部的内容访问权。
    /// 回调若调用 MappedPages::bytes，须证明独占来源整个范围初始化且无并发写。
    /// Inline 回调不得睡眠、动态准备或递归停止本 Disk；Task 回调可以睡眠，
    /// 但不得等待自身父完成、等待本 Disk 停止或复用仍由回调访问的 slot。
    /// 两种回调都不得泄露 memory 借用或在返回后继续访问其裸别名。
    /// Task 回调的现存工作域须保开放至本 Bio 被拒绝或完成，沿用域 owner 停外部生产源后关闭的规则。
    /// Task Bio 构造期另备同域独占执行栈；拒绝或生成者最后完成也不能耗尽设备完成的准入。
    pub(crate) unsafe fn try_new(
        disk: &Disk,
        operation: BlkOperation,
        sector: u64,
        mut memory: BioMemory,
        attributes: u32,
        end_io: Option<EndIo>,
    ) -> Result<Self, BlockError> {
        sched::assert_switchable();
        let prepared = (|| {
            let mut len = 0_usize;
            for mapped in &memory {
                if mapped.is_device_owned() || mapped.is_folio_complete() {
                    return Err(BlockError::Busy);
                }
                len = len
                    .checked_add(mapped.len())
                    .ok_or(BlockError::InvalidRange)?;
            }
            disk.limits().validate(
                operation,
                sector,
                len,
                memory.iter().map(MappedPages::range),
                attributes,
            )?;
            if let Some(domain) = disk.domain {
                for mapped in &memory {
                    mapped.segment(domain).map_err(BlockError::from)?;
                }
            }

            let allocator = memory.allocator().control();
            let mut uninit = Arc::<BioInner, BioAllocator>::try_new_uninit_in(allocator)
                .map_err(|_| BlockError::NoMemory)?;
            Arc::get_mut(&mut uninit)
                .expect("new bio has another owner")
                .write(BioInner {
                    target: disk.identity(),
                    domain: disk.domain,
                    raw_writer_next: UnsafeCell::new(core::ptr::null()),
                    operation,
                    sector,
                    len,
                    segment_count: memory.len(),
                    end_io,
                    finalizer_work: None,
                    state: SpinLock::new(BioState {
                        phase: Phase::New,
                        memory: None,
                        remaining: 0,
                        generating: false,
                        registered_end: 0,
                        registered_children: 0,
                        accepted_end: 0,
                        first_error: None,
                        result: None,
                        completion_owner: None,
                        fs_claim: None,
                        finalizer_ticket: None,
                        writeback_domain: None,
                    }),
                    ready: AtomicBool::new(false),
                    waiters: WaitQueue::new(),
                });
            // SAFETY: 新 uninit Arc 尚未发表且没有其它引用，全部字段已写齐，
            // 原 allocator 值随同同一真实控制分配继续存在。
            let inner = unsafe { uninit.assume_init() };
            if inner.needs_task() {
                let ticket = BioRawTicket::from_arc(inner);
                let raw = ticket.pointer;
                let owner = ticket.into_arc();
                let work = disk.prepare_finalizer(finalizer_callback, raw as usize)?;
                // SAFETY: Work 尚无票，原映射未移入，没有任何回调或第二份 Bio 引用。
                unsafe { (*raw.cast_mut()).finalizer_work = Some(work) };
                return Ok(owner);
            }
            Ok(inner)
        })();

        match prepared {
            Ok(inner) => {
                inner.state.lock().memory = Some(memory);
                Ok(Self { inner })
            }
            Err(error) => {
                for mapped in &mut memory {
                    if !mapped.is_device_owned() {
                        // SAFETY: 请求未发表，构造义务保证 CPU 别名结束；
                        // 错误结果不授予初始化范围，保留真正准备错误而非析构取消错误。
                        unsafe {
                            mapped.complete_folio(IoResult {
                                completed: 0,
                                error: Some(error),
                            })
                        };
                    }
                }
                Err(error)
            }
        }
    }

    /// 克隆现存父引用供提交后的观察，不分配或复制 payload。
    pub fn completion(&self) -> BioCompletion {
        BioCompletion {
            inner: Arc::clone(&self.inner),
        }
    }
}

impl BioCompletion {
    /// 普通可切换 Task 登记栈节点、复查再阻塞；通知只用于重查 Release 完成事实。
    pub fn wait(&self) -> IoResult {
        sched::assert_switchable();
        loop {
            if let Some(result) = self.result() {
                return result;
            }
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY: self 强拥有等待源，pin! 固定栈节点；registration 不被遗忘，
            // 所有返回均先撤 registration，再析构节点与 waiter，源始终有效。
            let _registration = unsafe { self.inner.waiters.register(node.as_mut()) };
            if let Some(result) = self.result() {
                return result;
            }
            waiter.park();
        }
    }

    /// Acquire 观察最终 DMA 收权和回调尾部，完成结果不被其他观察者消费。
    pub fn result(&self) -> Option<IoResult> {
        if !self.is_complete() {
            return None;
        }
        self.inner.state.lock().result
    }

    /// 只在所有义务、收权及回调已经结束后为真。
    pub fn is_complete(&self) -> bool {
        self.inner.ready.load(Ordering::Acquire)
    }

    /// 完成后只允许一个观察者移出原 memory，未完成或重复领取返回 Busy。
    /// Read 错误和短读的尾部仍可能未初始化；返回映射只提供安全 MaybeUninit 写入口，
    /// unsafe bytes 的全范围初始化义务不因领取 memory 而解除。
    pub(crate) fn take_memory(&self) -> Result<BioMemory, BlockError> {
        if !self.is_complete() {
            return Err(BlockError::Busy);
        }
        self.inner
            .state
            .lock()
            .memory
            .take()
            .ok_or(BlockError::Busy)
    }

    /// 延长原 Arc 控制及其 owning allocator 的弱尾，不复制计数或保留父强引用。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_weak(&self) -> impl Send + Sync + 'static {
        Arc::downgrade(&self.inner)
    }

    /// 验收等待已预备父 finalizer 的真实票尾，不以 ready 推测 worker 已返还。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_flush_finalizer(&self) {
        if let Some(work) = &self.inner.finalizer_work {
            work.as_ref().flush();
        }
    }

    /// 观察原父的登记与真实设备接受前缀，登记失败的 child 不会扩大接受账。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_acceptance(&self) -> (usize, usize, usize) {
        let state = self.inner.state.lock();
        (
            state.registered_children,
            state.registered_end,
            state.accepted_end,
        )
    }

    /// 只观察已有父 finalizer 的原生任务；弱观察升级不申请新的执行资源。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_finalizer_task(&self) -> Option<Arc<crate::task::Task>> {
        self.inner
            .finalizer_work
            .as_ref()
            .and_then(|work| work.ktest_reserved_task())
    }

    /// 只在原 manager 完成该 finalizer 原生任务的真实 join 后为真。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_finalizer_joined(&self) -> bool {
        self.inner
            .finalizer_work
            .as_ref()
            .is_some_and(|work| work.ktest_reserved_joined())
    }
}

impl BioInner {
    /// 返回构造时固定的完整设备身份，不强拥有设备。
    pub(super) fn target(&self) -> DiskIdentity {
        self.target
    }

    /// 返回父逻辑操作，所有 child 必须保持此操作。
    pub(super) fn operation(&self) -> BlkOperation {
        self.operation
    }

    /// 返回固定的 512 字节协议 sector 起点。
    pub(super) fn sector(&self) -> u64 {
        self.sector
    }

    /// 返回原映射总字节数，Flush 的范围为空。
    pub(super) fn len(&self) -> usize {
        self.len
    }

    /// 返回真实原映射数量，不把拆分后的 SG 数当作原页数量。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "批量读取测试按原页数观察每个父 Bio 的准备和完成责任"
        )
    )]
    pub(super) fn segment_count(&self) -> usize {
        self.segment_count
    }

    /// 从唯一原拥有者复制一份有界 DMA 元数据，不导出页借用。
    pub(super) fn segment(&self, index: usize) -> Option<PayloadRange> {
        self.state
            .lock()
            .memory
            .as_ref()?
            .get(index)
            .map(MappedPages::range)
    }

    /// 已登记 child 在锁外执行 CPU 复制，短锁只取得原页的有界裸地址计划。
    ///
    /// # Safety
    /// 调用者持本 child 的唯一完成责任和目标 RAM 的独占复制通道，ram
    /// 覆盖 length 字节且不与原 payload 重叠；child 完成前不得结束 CPU 访问期。
    pub(super) unsafe fn copy_cpu(
        &self,
        offset: usize,
        length: usize,
        ram: *mut u8,
    ) -> Result<(), BlockError> {
        let mut index = 0;
        let mut base = 0;
        let mut copied = 0;
        while copied < length {
            let (plan, part_len, next_base) = {
                let state = self.state.lock();
                assert_eq!(state.phase, Phase::Active);
                let mapped = state
                    .memory
                    .as_ref()
                    .expect("CPU copy lost original memory")
                    .get(index)
                    .ok_or(BlockError::InvalidRange)?;
                let end = base + mapped.len();
                if end <= offset + copied {
                    index += 1;
                    base = end;
                    continue;
                }
                let within = offset + copied - base;
                let part_len = (mapped.len() - within).min(length - copied);
                // SAFETY: 已登记 child 未完成，memory 不会移出；计划只含本 child 的范围，
                // 单一复制通道排斥重叠访问，执行时不再借用 MappedPages。
                let plan =
                    unsafe { mapped.cpu_copy_range(within, part_len) }.map_err(BlockError::from)?;
                (plan, part_len, end)
            };
            // SAFETY: 完整 RAM 范围独占且长度已核实；child 保原页和 CPU 访问期到返回。
            unsafe {
                match self.operation {
                    BlkOperation::Read => plan.copy_from(ram.add(copied)),
                    BlkOperation::Write => plan.copy_to(ram.add(copied)),
                    BlkOperation::Flush => unreachable!("Flush has no CPU payload"),
                }
            }
            .map_err(BlockError::from)?;
            copied += part_len;
            base = next_base;
            index += 1;
        }
        Ok(())
    }

    /// 按父 payload 次序投影到实际原 folio，支持一个 Bio 内的多个原映射。
    #[cfg(feature = "ktest")]
    pub(super) fn ktest_folio_origin(&self, offset: usize) -> Option<(usize, usize)> {
        let state = self.state.lock();
        let mut base = 0;
        for mapped in state.memory.as_ref()? {
            let end = base + mapped.len();
            if base <= offset && offset < end {
                return Some((
                    mapped.ktest_folio_phys_addr()?,
                    mapped.offset() + offset - base,
                ));
            }
            base = end;
        }
        None
    }

    /// 只分类本 child 的实际 WB 来源，用于本盘一次性受控错误。
    #[cfg(feature = "ktest")]
    pub(super) fn ktest_writeback_metadata(&self, offset: usize) -> Option<bool> {
        let state = self.state.lock();
        let mut base = 0;
        for mapped in state.memory.as_ref()? {
            let end = base + mapped.len();
            if base <= offset && offset < end {
                return mapped.ktest_writeback_metadata();
            }
            base = end;
        }
        None
    }

    /// 按本 child 的 payload 起点识别实际文件读入，不访问 DMA 内容。
    #[cfg(feature = "ktest")]
    pub(super) fn ktest_file_read(&self, offset: usize) -> Option<bool> {
        let state = self.state.lock();
        let mut base = 0;
        for mapped in state.memory.as_ref()? {
            let end = base + mapped.len();
            if base <= offset && offset < end {
                return Some(mapped.ktest_file_read());
            }
            base = end;
        }
        None
    }

    /// slot 在 IRQ 分派前检查是否必须使用预建完成 Work。
    pub(super) fn needs_task(&self) -> bool {
        matches!(self.end_io, Some(EndIo::Task(_)))
    }

    /// 唯一 WB 准备者在准入前登记真实统计域，不分配新控制对象。
    pub(super) fn track_writeback(&self, domain: &Arc<WritebackDomain>) {
        let mut state = self.state.lock();
        assert_eq!(state.phase, Phase::New);
        assert!(state.writeback_domain.is_none());
        state.writeback_domain = Some(domain.clone());
    }

    /// 已接受 FS 来源在任何 child 发表之前保留共享占用，拒绝的 Bio 不取得该责任。
    pub(super) fn retain_fs_claim(&self, claim: &FsClaim) {
        let mut state = self.state.lock();
        assert_eq!(state.phase, Phase::Active);
        assert!(state.fs_claim.is_none());
        state.fs_claim = Some(claim.clone());
    }

    /// 为准备中的 part 克隆同一占用，延续至 slot 资源与队列尾部结束。
    pub(super) fn fs_claim(&self) -> Option<FsClaim> {
        self.state.lock().fs_claim.clone()
    }

    /// 唯一 admission 在普通 Task 中建立生成者义务，再锁外交整个原 memory 给设备。
    pub(super) fn begin(&self) -> Result<(), BlockError> {
        sched::assert_switchable();
        let mut memory = {
            let mut state = self.state.lock();
            assert_eq!(state.phase, Phase::New, "bio admitted twice");
            state.phase = Phase::Beginning;
            state.remaining = 1;
            state.generating = true;
            state
                .memory
                .take()
                .expect("bio memory missing at admission")
        };

        let mut started = 0;
        let mut failure = None;
        for mapped in &mut memory {
            // SAFETY: 唯一 admission 尚未发表 child；来源初始化和 CPU 别名义务由构造保证。
            let result = unsafe {
                match self.domain {
                    Some(domain) => mapped.begin_device_access(domain),
                    None => {
                        mapped.begin_cpu_access();
                        Ok(())
                    }
                }
            };
            if let Err(error) = result {
                failure = Some(BlockError::from(error));
                break;
            }
            started += 1;
        }
        if failure.is_some() {
            for mapped in &mut memory[..started] {
                // SAFETY: 尚无任何 child 或硬件引用，所有已开始访问可以立即归还。
                unsafe {
                    if self.domain.is_some() {
                        mapped
                            .finish_device_access()
                            .expect("unpublished DMA ownership rollback failed");
                    } else {
                        mapped.finish_cpu_access();
                    }
                }
            }
        }

        let mut state = self.state.lock();
        assert_eq!(state.phase, Phase::Beginning);
        state.memory = Some(memory);
        if let Some(error) = failure {
            state.phase = Phase::New;
            state.remaining = 0;
            state.generating = false;
            return Err(error);
        }
        state.phase = Phase::Active;
        Ok(())
    }

    /// 按父逻辑次序在 avail 发表前登记一个 child，不分配完整 child 列表。
    pub(super) fn register_child(&self, offset: usize, length: usize) {
        let end = offset
            .checked_add(length)
            .expect("bio child range overflow");
        assert!(end <= self.len);
        assert!(0 < length || self.operation == BlkOperation::Flush);

        let mut state = self.state.lock();
        assert_eq!(state.phase, Phase::Active);
        assert!(state.generating, "bio child registered after generation");
        assert_eq!(
            offset, state.registered_end,
            "bio child ranges are not contiguous"
        );
        if self.operation == BlkOperation::Flush {
            assert_eq!(state.registered_children, 0, "flush registered twice");
        }
        state.registered_end = end;
        state.registered_children = state
            .registered_children
            .checked_add(1)
            .expect("bio child count overflow");
        state.remaining = state
            .remaining
            .checked_add(1)
            .expect("bio pending count overflow");
    }

    /// driver.submit 成功后仍持同一 queue 锁登记真实接受，IRQ 尚不能认领完成。
    pub(super) fn accept_child(&self, offset: usize, length: usize) {
        let end = offset.checked_add(length).expect("bio acceptance overflow");
        let mut state = self.state.lock();
        assert_eq!(state.phase, Phase::Active);
        assert!(state.generating && end <= state.registered_end);
        assert_eq!(
            offset, state.accepted_end,
            "bio acceptance is not contiguous"
        );
        let mut base = 0;
        for mapped in state.memory.as_mut().expect("active bio memory missing") {
            let mapped_end = base + mapped.len();
            let start = offset.max(base);
            let stop = end.min(mapped_end);
            if start < stop {
                mapped.accept_range(start - base, stop - start);
            }
            base = mapped_end;
        }
        state.accepted_end = end;
        if let Some(domain) = &state.writeback_domain {
            domain.note_submitted(length);
        }
    }

    /// 唯一 slot 完成者在 used 或已确认 reset 后结束 child；Task 回调须由 slot Work 执行。
    /// 未发表的失败 child 也消费同一登记义务，生成者仍保住此前在途原页。
    pub(super) fn finish_child(
        bio: &BioArc,
        offset: usize,
        length: usize,
        completed: usize,
        error: Option<BlockError>,
    ) {
        let end = offset
            .checked_add(length)
            .expect("bio child range overflow");
        assert!(end <= bio.len && completed <= length);
        let error = error.or_else(|| (completed < length).then_some(BlockError::ShortIo));
        let completion = {
            let mut state = bio.state.lock();
            assert_eq!(state.phase, Phase::Active);
            assert!(end <= state.registered_end);
            assert!(
                usize::from(state.generating) < state.remaining,
                "bio child ended twice"
            );
            if let Some(error) = error {
                state.record_error(offset + completed, error);
            }
            let mut base = 0;
            for mapped in state.memory.as_mut().expect("active bio memory missing") {
                let mapped_end = base + mapped.len();
                let start = offset.max(base);
                let stop = end.min(mapped_end);
                if start < stop {
                    let progress = (offset + completed).saturating_sub(start).min(stop - start);
                    let mapped_error = if progress < stop - start || completed == length {
                        error
                    } else {
                        None
                    };
                    mapped.record_writeback_result(
                        start - base,
                        stop - start,
                        progress,
                        mapped_error,
                    );
                }
                base = mapped_end;
            }
            state.end_obligation(bio.len)
        };

        if let Some((memory, result)) = completion {
            Self::complete(bio, memory, result, true, None, false);
        }
    }

    /// 结束生成者账本；true 表示可睡完成及 accepted 尾部已移交构造期同域预留 Work。
    /// accepted 生成者借用实际 DiskRef；只有独立完成 Work 接责时才取得强拥有。
    pub(super) fn finish_generation(
        bio: &BioArc,
        error: Option<(usize, BlockError)>,
        owner: Option<&DiskRef>,
    ) -> bool {
        sched::assert_switchable();
        let completion = {
            let mut state = bio.state.lock();
            assert_eq!(state.phase, Phase::Active);
            assert!(state.generating, "bio generation ended twice");
            if let Some((offset, error)) = error {
                assert!(offset <= state.registered_end);
                state.record_error(offset, error);
            } else {
                assert_eq!(
                    state.registered_end, bio.len,
                    "bio generation is incomplete"
                );
                assert!(0 < state.registered_children, "bio generated no child");
            }
            state.generating = false;
            state.end_obligation(bio.len)
        };

        if let Some((memory, result)) = completion {
            return Self::complete(bio, memory, result, true, owner, true);
        }
        false
    }

    /// 普通 Task 在未 admission 时立即失败，原 memory 从未交设备，回调仍在锁外。
    pub(super) fn reject(bio: &BioArc, error: BlockError) {
        sched::assert_switchable();
        let memory = {
            let mut state = bio.state.lock();
            assert_eq!(state.phase, Phase::New, "rejecting an admitted bio");
            state.phase = Phase::Finishing;
            state.memory.take().expect("rejected bio memory missing")
        };
        Self::complete(
            bio,
            memory,
            IoResult {
                completed: 0,
                error: Some(error),
            },
            false,
            None,
            true,
        );
    }

    /// 最后义务先锁外收原映射；非 slot 的 Task 完成独立排队，不阻塞批次生成者。
    fn complete(
        bio: &BioArc,
        mut memory: BioMemory,
        result: IoResult,
        device_owned: bool,
        owner: Option<&DiskRef>,
        queue_task: bool,
    ) -> bool {
        if device_owned {
            for mapped in &mut memory {
                // SAFETY: remaining 已归零并唯一移出 memory；已发表 child 只在真实 used
                // 或确认 reset 后结束，未发表 child 没有设备引用，生成者也已结束。
                unsafe {
                    if bio.domain.is_some() {
                        mapped
                            .finish_device_access()
                            .expect("completed DMA cache synchronization failed");
                    } else {
                        mapped.finish_cpu_access();
                    }
                };
            }
        }

        if queue_task && bio.needs_task() {
            let ticket = BioRawTicket::from_arc(Arc::clone(bio));
            let owned = ticket.pointer;
            {
                let mut state = bio.state.lock();
                assert_eq!(state.phase, Phase::Finishing);
                assert!(
                    state.memory.is_none()
                        && state.result.is_none()
                        && state.completion_owner.is_none()
                );
                state.memory = Some(memory);
                state.result = Some(result);
                // 所有 child 已结束并从槽移出，临时 owner 没有 Disk→本 Bio 的返回边。
                state.completion_owner = owner.cloned();
                assert!(state.finalizer_ticket.is_none());
                state.finalizer_ticket = Some(ticket);
            }
            let work = bio
                .finalizer_work
                .as_ref()
                .expect("Task bio has no prepared finalizer");
            assert_eq!(owned as usize, work.context());
            assert!(
                work.as_ref()
                    .queue()
                    .expect("bio finalizer domain closed before producers stopped"),
                "bio finalizer queued twice"
            );
            return true;
        }

        drop(bio.finalize(memory, result));
        false
    }

    /// 唯一完成者锁外执行回调及 folio 收权，Release 发布父完成后移出 FS 占用。
    /// 返回的占用由 accepted 生成者或 part 保活至各自队列尾，最后 put 不在状态锁内。
    fn finalize(&self, mut memory: BioMemory, result: IoResult) -> Option<FsClaim> {
        match self.end_io {
            Some(EndIo::Inline(callback)) => callback(&result, &memory),
            Some(EndIo::Task(callback)) => {
                // child 尾部属于专用 slot Work，其它尾部属于构造期 Bio Work。
                sched::assert_switchable();
                callback(&result, &memory);
            }
            None => {}
        }

        let mut remaining = result.completed;
        for mapped in &mut memory {
            let completed = remaining.min(mapped.len());
            remaining = remaining.saturating_sub(mapped.len());
            // SAFETY: complete 已结束全部 child 与生成者并真实收权，回调全部返回；
            // 读入沿父连续前缀，WB 沿同一父为该映射登记的真实接受与实际错误结束。
            unsafe {
                mapped.complete_folio(IoResult {
                    completed,
                    error: result.error,
                })
            };
        }

        let claim = {
            let mut state = self.state.lock();
            assert_eq!(state.phase, Phase::Finishing);
            assert!(state.memory.is_none() && state.result.is_none());
            state.memory = Some(memory);
            state.result = Some(result);
            state.phase = Phase::Complete;
            state.fs_claim.take()
        };
        self.ready.store(true, Ordering::Release);
        self.waiters.wake_all();
        claim
    }
}

/// 消费队列化的真实 Bio 拥有，归还生成者/临时 Disk；Work 票尾部自动释放一次性执行预留。
fn finalizer_callback(work: Pin<&Work>) {
    let pointer = work.context() as *const BioInner;
    // SAFETY: complete 已把真实 raw strong 和原 allocator 整票放入本状态；
    // 该 owning 票保地址至回调取走，Work 自身另有真实运行票。
    let inner = unsafe { &*pointer };
    let (ticket, memory, result, owner) = {
        let mut state = inner.state.lock();
        assert_eq!(state.phase, Phase::Finishing);
        let ticket = state
            .finalizer_ticket
            .take()
            .expect("finalizer lost its raw ticket");
        assert_eq!(ticket.pointer, pointer);
        (
            ticket,
            state.memory.take().expect("finalizer memory missing"),
            state.result.take().expect("finalizer result missing"),
            state.completion_owner.take(),
        )
    };

    let bio = ticket.into_arc();
    let claim = bio.finalize(memory, result);
    if let Some(owner) = owner {
        owner.finish_generation_tail(&bio);
    }
    drop(claim);
}

impl BioState {
    /// 记录最小错误字节位置，后部反序成功不跨过前部失败形成虚假成功前缀。
    fn record_error(&mut self, offset: usize, error: BlockError) {
        if self.first_error.is_none_or(|(first, _)| offset < first) {
            self.first_error = Some((offset, error));
        }
    }

    /// 消费一个已登记义务，只在最后归零时把唯一原页向锁外完成者移交。
    fn end_obligation(&mut self, len: usize) -> Option<(BioMemory, IoResult)> {
        assert!(0 < self.remaining, "bio obligation ended twice");
        self.remaining -= 1;
        if self.remaining != 0 {
            return None;
        }
        assert!(!self.generating);
        self.phase = Phase::Finishing;
        let result = match self.first_error {
            Some((completed, error)) => IoResult {
                completed,
                error: Some(error),
            },
            None => IoResult {
                completed: len,
                error: None,
            },
        };
        Some((
            self.memory.take().expect("completed bio memory missing"),
            result,
        ))
    }
}
