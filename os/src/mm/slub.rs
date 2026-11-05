//! 固定大小缓存的本核对象链、共享归还链与非冻结 partial 责任。

use core::alloc::Layout;
use core::ptr::NonNull;
use core::sync::atomic::{AtomicUsize, Ordering};
use spin::Once;

use crate::arch;
use crate::boot::MAX_CPUS;
use crate::cpu;
use crate::mm::bootmem::PAGE_SIZE;
use crate::mm::page::{PageDesc, SlabLocation, SlabState};
use crate::mm::{alloc_pages, pages};
use crate::sync::{IrqSaveGuard, SpinLock};

pub(crate) const SIZES: [usize; 12] = [8, 16, 32, 64, 96, 128, 192, 256, 512, 1024, 2048, 4096];
pub(crate) const NONE: usize = usize::MAX;
pub(crate) const CPU_PARTIAL_LIMIT: usize = 2;
pub(crate) const MIN_PARTIAL: usize = 2;
const MAX_SLAB_ORDER: usize = 3;
const MIN_OBJECTS_PER_CPU_SCALE: usize = 4;
const WASTE_FRACTIONS: [usize; 4] = [16, 8, 4, 2];

#[derive(Clone, Copy)]
pub(crate) struct Geometry {
    pub(crate) stride: usize,
    pub(crate) preferred_order: usize,
    pub(crate) minimum_order: usize,
    #[cfg(feature = "ktest")]
    pub(crate) objects: usize,
}

struct CpuSlab {
    active: usize,
    local: usize,
    local_tail: usize,
    local_count: usize,
    partial: usize,
    partial_count: usize,
}

impl CpuSlab {
    /// 建立不持有 backing 的本核控制对象。
    const fn new() -> Self {
        Self {
            active: NONE,
            local: 0,
            local_tail: 0,
            local_count: 0,
            partial: NONE,
            partial_count: 0,
        }
    }

    /// 在本核控制锁下摘取一个已初始化链字的空闲对象。
    fn pop(&mut self) -> Option<NonNull<u8>> {
        let pointer = NonNull::new(self.local as *mut u8)?;
        // SAFETY: local 只接收独占的空闲对象链；链字在发布前已写入，CPU guard
        // 排除本核分配及归还，远端只访问 slab shared 链。
        self.local = unsafe { (self.local as *const usize).read() };
        self.local_count -= 1;
        if self.local == 0 {
            self.local_tail = 0;
        }
        Some(pointer)
    }

    /// 只摘离 active 控制字段，将冻结责任交给普通执行流的小票据。
    fn take_active(&mut self) -> ActiveTicket {
        let ticket = ActiveTicket {
            head: self.active,
            local: self.local,
            tail: self.local_tail,
            count: self.local_count,
        };
        self.active = NONE;
        self.local = 0;
        self.local_tail = 0;
        self.local_count = 0;
        ticket
    }

    /// 摘离本核控制字段，将一次归并责任移交票据，不访问任何对象或 slab 链。
    fn take_all(&mut self) -> FlushTicket {
        let ticket = FlushTicket {
            active: self.take_active(),
            partial: self.partial,
        };
        self.partial = NONE;
        self.partial_count = 0;
        ticket
    }
}

struct ActiveTicket {
    head: usize,
    local: usize,
    tail: usize,
    count: usize,
}

struct FlushTicket {
    active: ActiveTicket,
    partial: usize,
}

impl FlushTicket {
    /// 建立尚未接收任何 CPU 责任的稳定发送者票据。
    const fn new() -> Self {
        Self {
            active: ActiveTicket {
                head: NONE,
                local: 0,
                tail: 0,
                count: 0,
            },
            partial: NONE,
        }
    }
}

struct NodePartial {
    head: usize,
    tail: usize,
    count: usize,
}

struct Cache {
    geometry: Geometry,
    cpus: &'static [Once<SpinLock<CpuSlab>>; MAX_CPUS],
    node: SpinLock<NodePartial>,
    backing: AtomicUsize,
}

impl Cache {
    /// 借用实际 CPU 已发布的控制槽；本核及远端排空仍使用同一份短锁。
    fn cpu(&self, owner: usize) -> &SpinLock<CpuSlab> {
        self.cpus[owner].get().expect("SLUB CPU is not initialized")
    }
}

static CPU_SLABS: [[Once<SpinLock<CpuSlab>>; MAX_CPUS]; SIZES.len()] =
    [const { [const { Once::new() }; MAX_CPUS] }; SIZES.len()];

static CACHES: Once<[Cache; SIZES.len()]> = Once::new();

/// 返回能容纳指定字节数的最小真实页阶。
fn order_for(bytes: usize) -> usize {
    let mut order = 0;
    while (PAGE_SIZE << order) < bytes {
        order += 1;
    }
    order
}

/// 按固定核数裁剪最小对象数，再逐级放宽浪费比例选择 preferred 阶。
fn calculate_geometry(stride: usize) -> Geometry {
    let fls = usize::BITS as usize - cpu::count().leading_zeros() as usize;
    let minimum_order = order_for(stride);
    let min_objects =
        (MIN_OBJECTS_PER_CPU_SCALE * (fls + 1)).min((PAGE_SIZE << MAX_SLAB_ORDER) / stride);
    let min_order = order_for(min_objects * stride);
    let mut preferred_order = minimum_order;
    'fractions: for fraction in WASTE_FRACTIONS {
        for order in min_order..=MAX_SLAB_ORDER {
            let bytes = PAGE_SIZE << order;
            if bytes % stride <= bytes / fraction {
                preferred_order = order;
                break 'fractions;
            }
        }
    }
    Geometry {
        stride,
        preferred_order,
        minimum_order,
        #[cfg(feature = "ktest")]
        objects: (PAGE_SIZE << preferred_order) / stride,
    }
}

/// BSP 在页入口可用后逐槽初始化实际 CPU，再发布小型 cache 与 node 控制表。
///
/// # Safety
/// 仅 BSP 在 IRQ 关闭、AP 未释放且无对象消费者时调用一次；页描述符、CPU 表与
/// buddy/PCP 已初始化，实际 RAM 的直接映射永久可写且一一对应。
pub(crate) unsafe fn initialize() {
    assert!(CACHES.get().is_none(), "SLUB initialized twice");
    CACHES.call_once(|| {
        core::array::from_fn(|class| {
            let cpus = &CPU_SLABS[class];
            for owner in cpu::actual_set().iter() {
                cpus[owner].call_once(|| SpinLock::new(CpuSlab::new()));
            }
            Cache {
                geometry: calculate_geometry(SIZES[class]),
                cpus,
                node: SpinLock::new(NodePartial {
                    head: NONE,
                    tail: NONE,
                    count: 0,
                }),
                backing: AtomicUsize::new(0),
            }
        })
    });
}

/// 选择同时满足容量、对象步长和 backing 基址对齐的缓存。
pub(crate) fn class_for(layout: Layout) -> Option<usize> {
    if PAGE_SIZE < layout.align() {
        return None;
    }
    SIZES
        .iter()
        .position(|size| layout.size() <= *size && size % layout.align() == 0)
}

/// 借用一次发布后不变的缓存几何和具体锁对象。
fn cache(class: usize) -> &'static Cache {
    &CACHES.get().expect("SLUB is not initialized")[class]
}

/// 按真实 PFN 单独定位永久描述符，不跨 section 做指针运算。
fn descriptor(head: usize) -> &'static PageDesc {
    pages().get(head).expect("slab PFN has no descriptor")
}

/// 在独占空闲链上寻找末对象；仅用于普通流的 shared 接收。
///
/// # Safety
/// 链上每个对象仍由调用者独占，首 word 已初始化且 usize 对齐，链完整以零终止；
/// 调用者须持排除其他链访问的 CPU 与 slab guard。
unsafe fn chain_tail(mut pointer: usize) -> usize {
    let mut tail = 0;
    while pointer != 0 {
        tail = pointer;
        // SAFETY: 调用者保证整条链的有效性和独占，当前 pointer 来自有效链字。
        pointer = unsafe { (pointer as *const usize).read() };
    }
    tail
}

/// 在 CPU→slab 锁域将全部 shared 对象交给 local，计数仍包含这些本地空闲对象。
fn install(cpu: &mut CpuSlab, owner: usize, head: usize, state: &mut SlabState) {
    assert_eq!(cpu.active, NONE, "install overwrites an active slab");
    assert!(state.shared != 0, "partial slab has no free objects");
    cpu.active = head;
    cpu.local = state.shared;
    cpu.local_count = state.objects - state.inuse;
    // SAFETY: CPU 与 slab guard 排除两条链的访问；shared 对象已经结束用户访问，
    // 且链字均已初始化，移交前仍由 state 独占。
    cpu.local_tail = unsafe { chain_tail(state.shared) };
    state.shared = 0;
    state.inuse = state.objects;
    state.location = SlabLocation::Active(owner);
    state.next = NONE;
    state.previous = NONE;
}

/// 在 node→slab 锁域摘除唯一 node 成员，邻接字段也分别受其 slab 锁保护。
fn remove_node(node: &mut NodePartial, head: usize, state: &mut SlabState) {
    assert!(
        matches!(state.location, SlabLocation::NodePartial),
        "slab is not a node member"
    );
    if state.previous == NONE {
        assert_eq!(node.head, head, "node head differs from member");
        node.head = state.next;
    } else {
        descriptor(state.previous)
            .usage()
            .slab_mut()
            .expect("node predecessor vanished")
            .next = state.next;
    }
    if state.next == NONE {
        assert_eq!(node.tail, head, "node tail differs from member");
        node.tail = state.previous;
    } else {
        descriptor(state.next)
            .usage()
            .slab_mut()
            .expect("node successor vanished")
            .previous = state.previous;
    }
    node.count -= 1;
    state.next = NONE;
    state.previous = NONE;
    state.location = SlabLocation::Detached;
}

/// 在 node→slab 锁域尾插一个独占摘离的 partial，保持有限 shrink 的前进顺序。
fn insert_node(node: &mut NodePartial, head: usize, state: &mut SlabState) {
    assert!(
        matches!(state.location, SlabLocation::Detached),
        "slab already has a list owner"
    );
    state.previous = node.tail;
    state.next = NONE;
    if node.tail == NONE {
        node.head = head;
    } else {
        descriptor(node.tail)
            .usage()
            .slab_mut()
            .expect("node tail vanished")
            .next = head;
    }
    node.tail = head;
    node.count += 1;
    state.location = SlabLocation::NodePartial;
}

/// 在全部 MM guard 退出后清除逐页关联并消费真实 backing，按原阶归还。
fn destroy(state: SlabState) {
    let head = state.backing.phys_addr() / PAGE_SIZE;
    let count = 1 << state.backing.order();
    for pfn in head..head + count {
        descriptor(pfn).slab_head.store(NONE, Ordering::Release);
    }
    cache(state.cache)
        .backing
        .fetch_sub(count, Ordering::Relaxed);
    drop(state);
}

/// 普通流按当前计数分类唯一 Detached 责任；只对 empty 应用 node 保留政策。
fn put_node(class: usize, head: usize, shrink: bool) {
    let discarded = {
        let mut node = cache(class).node.lock();
        let mut slot = descriptor(head).usage();
        let state = slot.slab_mut().expect("detached slab vanished");
        assert!(
            matches!(state.location, SlabLocation::Detached),
            "detached slab has another owner"
        );
        if state.inuse == 0 && (shrink || MIN_PARTIAL <= node.count) {
            slot.take_slab()
        } else {
            if state.shared == 0 {
                state.location = SlabLocation::Full;
            } else {
                insert_node(&mut node, head, state);
            }
            None
        }
    };
    if let Some(state) = discarded {
        destroy(state);
    }
}

/// 普通流拼回票据的 local 并解除冻结责任，远端归还仍在同一 slab 锁下合并。
fn deactivate(class: usize, ticket: ActiveTicket) {
    if ticket.head == NONE {
        return;
    }
    {
        let mut slot = descriptor(ticket.head).usage();
        let state = slot.slab_mut().expect("active ticket vanished");
        assert!(
            matches!(state.location, SlabLocation::Active(_)),
            "ticket slab is not active"
        );
        if ticket.local != 0 {
            // SAFETY: ticket 独占摘出的 local 链，tail 由本核链操作持续维护；
            // slab guard 排除 shared 写者，tail 所在对象没有 live 用户访问。
            unsafe { (ticket.tail as *mut usize).write(state.shared) };
            state.shared = ticket.local;
            state.inuse -= ticket.count;
        }
        state.location = SlabLocation::Detached;
    }
    put_node(class, ticket.head, false);
}

/// 普通流逐项消费已摘离的 CPU partial 链，逐个短 slab 锁后交给 node 政策。
fn put_partials(class: usize, mut head: usize) {
    while head != NONE {
        let next = {
            let mut slot = descriptor(head).usage();
            let state = slot.slab_mut().expect("CPU partial ticket vanished");
            assert!(
                matches!(state.location, SlabLocation::CpuPartial(_)),
                "partial ticket lost its owner"
            );
            let next = state.next;
            state.next = NONE;
            state.previous = NONE;
            state.location = SlabLocation::Detached;
            next
        };
        put_node(class, head, false);
        head = next;
    }
}

/// 锁外申请 preferred backing，失败后只尝试一次 minimum，并初始化真实几何和逐页关联。
fn new_slab(class: usize) -> Option<usize> {
    let cache = cache(class);
    let geometry = cache.geometry;
    let mut backing = alloc_pages(geometry.preferred_order, false).or_else(|| {
        if geometry.minimum_order < geometry.preferred_order {
            alloc_pages(geometry.minimum_order, false)
        } else {
            None
        }
    })?;
    let head = backing.phys_addr() / PAGE_SIZE;
    let count = 1 << backing.order();
    let objects = backing.len() / geometry.stride;
    let start = backing.as_mut_ptr().cast::<u8>() as usize;
    for index in 0..objects {
        let pointer = start + index * geometry.stride;
        let next = if index + 1 < objects {
            pointer + geometry.stride
        } else {
            0
        };
        // SAFETY: backing 独占完整连续块，stride 至少 usize 且满足其对齐；
        // index 在真实容量内，尚未发布对象，写入的首 word 不触及其他对象。
        unsafe { (pointer as *mut usize).write(next) };
    }
    {
        let mut slot = descriptor(head).usage();
        slot.install_slab(SlabState {
            backing,
            cache: class,
            objects,
            inuse: 0,
            shared: start,
            location: SlabLocation::Detached,
            next: NONE,
            previous: NONE,
        });
    }
    for pfn in head..head + count {
        descriptor(pfn).slab_head.store(head, Ordering::Release);
    }
    cache.backing.fetch_add(count, Ordering::Relaxed);
    Some(head)
}

/// 非阻塞地依次尝试 local、shared、CPU partial、node partial 与锁外真实 backing。
pub(crate) fn allocate(class: usize) -> Option<NonNull<u8>> {
    let cache = cache(class);
    loop {
        let exhausted = {
            let _irq = IrqSaveGuard::new();
            let owner = cpu::current().id;
            let mut local = cache.cpu(owner).lock();
            if let Some(pointer) = local.pop() {
                return Some(pointer);
            }
            if local.active != NONE {
                let mut slot = descriptor(local.active).usage();
                let state = slot.slab_mut().expect("CPU active slab vanished");
                assert!(
                    state.location == SlabLocation::Active(owner),
                    "CPU active owner differs"
                );
                if state.shared != 0 {
                    let head = local.active;
                    local.active = NONE;
                    install(&mut local, owner, head, state);
                    return local.pop();
                }
                Some(local.take_active())
            } else {
                if local.partial != NONE {
                    let head = local.partial;
                    let mut slot = descriptor(head).usage();
                    let state = slot.slab_mut().expect("CPU partial slab vanished");
                    assert!(
                        state.location == SlabLocation::CpuPartial(owner),
                        "CPU partial owner differs"
                    );
                    local.partial = state.next;
                    local.partial_count -= 1;
                    install(&mut local, owner, head, state);
                    return local.pop();
                }
                let mut node = cache.node.lock();
                if node.head != NONE {
                    let head = node.head;
                    let mut slot = descriptor(head).usage();
                    let state = slot.slab_mut().expect("node partial slab vanished");
                    remove_node(&mut node, head, state);
                    install(&mut local, owner, head, state);
                    return local.pop();
                }
                None
            }
        };
        if let Some(ticket) = exhausted {
            deactivate(class, ticket);
            continue;
        }
        let head = new_slab(class)?;
        {
            let _irq = IrqSaveGuard::new();
            let owner = cpu::current().id;
            let mut local = cache.cpu(owner).lock();
            if local.active == NONE {
                let mut slot = descriptor(head).usage();
                install(
                    &mut local,
                    owner,
                    head,
                    slot.slab_mut().expect("prepared slab vanished"),
                );
                return local.pop();
            }
        }
        // 申请期间 IRQ 可能已装入其他 active；新块按普通 partial 政策处理，不覆盖本核链。
        put_node(class, head, false);
    }
}

/// 将完整 slab 的首次归还责任先发布为 CPU partial，再取得本核控制锁插链。
/// 调用者在本核 IRQ 关闭期间调用，避免责任声明与控制插入之间被 flush 摘离。
fn put_cpu_partial(class: usize, owner: usize, head: usize) -> usize {
    let mut local = cache(class).cpu(owner).lock();
    let drained = if CPU_PARTIAL_LIMIT <= local.partial_count {
        let old = local.partial;
        local.partial = NONE;
        local.partial_count = 0;
        old
    } else {
        NONE
    };
    {
        let mut slot = descriptor(head).usage();
        let state = slot.slab_mut().expect("claimed CPU partial vanished");
        assert!(
            state.location == SlabLocation::CpuPartial(owner),
            "CPU partial claim changed"
        );
        state.next = local.partial;
        state.previous = NONE;
    }
    local.partial = head;
    local.partial_count += 1;
    drained
}

/// 在 slab guard 下将唯一 live 对象写入 shared，并同步扣除 inuse。
///
/// # Safety
/// pointer 是 state 的唯一待释放 live 对象，全部用户访问已结束且尚未归还；
/// 调用者持有该 slab guard，首 word 满足 usize 对齐并位于可写 backing。
unsafe fn free_shared(state: &mut SlabState, pointer: usize) {
    assert!(0 < state.inuse, "slab inuse underflow");
    // SAFETY: 调用者交出 live 对象唯一释放权；slab guard 排除 shared 链并发写者。
    unsafe { (pointer as *mut usize).write(state.shared) };
    state.shared = pointer;
    state.inuse -= 1;
}

/// 通过逐页关联定位真实头；本核 active 归 local，其他对象按真实成员责任归 shared。
///
/// # Safety
/// pointer 必须是本 SLUB 返回且仍 live 的对象首地址；调用者交出唯一释放权，
/// 所有访问已经结束，不得在本次归还以后读取或再次释放该对象。
pub(crate) unsafe fn deallocate(pointer: NonNull<u8>) {
    let address = pointer.as_ptr() as usize;
    let pfn = arch::virt_to_phys(address) / PAGE_SIZE;
    let head = descriptor(pfn).slab_head.load(Ordering::Acquire);
    assert!(head != NONE, "object has no slab head");
    let class = descriptor(head)
        .usage()
        .slab()
        .expect("live object has no slab state")
        .cache;
    let drained = {
        let _irq = IrqSaveGuard::new();
        let owner = cpu::current().id;
        {
            let mut local = cache(class).cpu(owner).lock();
            if local.active == head {
                // SAFETY: caller 已结束此对象全部访问并交出唯一权；本核 guard 排除
                // local 链并发访问，远端只写 shared，首 word 满足 usize 对齐。
                unsafe { (address as *mut usize).write(local.local) };
                if local.local == 0 {
                    local.local_tail = address;
                }
                local.local = address;
                local.local_count += 1;
                return;
            }
        }
        let mut slot = descriptor(head).usage();
        let state = slot.slab_mut().expect("live object slab vanished");
        if matches!(state.location, SlabLocation::NodePartial) {
            // 对象尚未计入 free，保证放开快照锁后 backing 仍 live；按 node→slab 重取。
            drop(slot);
            let (discarded, claimed) = {
                let mut node = cache(class).node.lock();
                let mut slot = descriptor(head).usage();
                let state = slot.slab_mut().expect("live node object vanished");
                // SAFETY: caller 保留唯一待释放对象权，尚未提交 shared；当前 slab 锁
                // 覆盖重新读取的角色和计数，node 锁允许成员变化。
                unsafe { free_shared(state, address) };
                if matches!(state.location, SlabLocation::Full) {
                    // 锁外期间原 node slab 可能已被耗尽并 deactivate，首次 free 仍须认领责任。
                    state.location = SlabLocation::CpuPartial(owner);
                    (None, true)
                } else if matches!(state.location, SlabLocation::NodePartial)
                    && state.inuse == 0
                    && MIN_PARTIAL < node.count
                {
                    remove_node(&mut node, head, state);
                    (slot.take_slab(), false)
                } else {
                    (None, false)
                }
            };
            let drained = if claimed {
                put_cpu_partial(class, owner, head)
            } else {
                NONE
            };
            // IRQ guard 也必须退出后才允许 OwnedPages 析构。
            drop(_irq);
            if let Some(state) = discarded {
                destroy(state);
            }
            put_partials(class, drained);
            return;
        }
        let was_full = matches!(state.location, SlabLocation::Full);
        // SAFETY: caller 持有该 live 对象唯一释放权，slab guard 覆盖 shared 与 inuse。
        unsafe { free_shared(state, address) };
        if was_full {
            state.location = SlabLocation::CpuPartial(owner);
            drop(slot);
            put_cpu_partial(class, owner, head)
        } else {
            NONE
        }
    };
    put_partials(class, drained);
}

/// 在本核短保护下摘离十二份控制字段，将一次归并责任移交发送者稳定票据。
///
/// # Safety
/// argument 指向同步 cpu::call 返回前有效、此 callback 独占写入的十二项票据；
/// 不得与另一 callback 或调用者并发访问同一数组。
unsafe fn detach_cpu(argument: *mut ()) {
    // SAFETY: 同步 call 保持发送者数组存活；当前 callback 独占写入，完成后 release
    // 发布，发送者只有 acquire 观察 call 完成后才读取。
    let tickets = unsafe { &mut *argument.cast::<[FlushTicket; SIZES.len()]>() };
    let owner = cpu::current().id;
    for (class, ticket) in tickets.iter_mut().enumerate() {
        *ticket = cache(class).cpu(owner).lock().take_all();
    }
}

/// 普通可服务 IPI 的执行流有限摘离每核缓存；合并和 backing 归还均在 callback 外。
/// 调用者不得持 MM 短锁；并发对象操作可在返回前重新建立缓存。
pub fn flush_all() {
    for owner in cpu::actual_set().iter() {
        let mut tickets = [const { FlushTicket::new() }; SIZES.len()];
        // SAFETY: owner 属于固定在线 CPU 表，调用者可服务 IPI 且不持 MM 锁；
        // 栈数组保持有效并只由该同步 callback 写入，call 完成后才读取。
        unsafe {
            cpu::call(
                owner,
                detach_cpu,
                (&mut tickets as *mut [FlushTicket; SIZES.len()]).cast(),
            )
        };
        for (class, ticket) in IntoIterator::into_iter(tickets).enumerate() {
            deactivate(class, ticket.active);
            put_partials(class, ticket.partial);
        }
    }
}

/// 普通执行流先 flush，再按各 node 的初始有限预算逐项摘离；live partial 尾插保留。
/// 停止参与者后完整归还 empty；并发新增 slab 不延长本次预算。
#[cfg_attr(
    not(feature = "ktest"),
    expect(
        dead_code,
        reason = "内存及 I/O 测试回收空 slab 并比较原页账，内部 flush 和摘离票据随此入口保留"
    )
)]
pub fn shrink_all() {
    flush_all();
    for class in 0..SIZES.len() {
        let budget = cache(class).node.lock().count;
        for _ in 0..budget {
            let head = {
                let mut node = cache(class).node.lock();
                if node.head == NONE {
                    break;
                }
                let head = node.head;
                let mut slot = descriptor(head).usage();
                remove_node(
                    &mut node,
                    head,
                    slot.slab_mut().expect("shrink node slab vanished"),
                );
                head
            };
            put_node(class, head, true);
        }
    }
}

/// 汇总低频创建/销毁计数；并发时是有限快照，参与者停稳后才是精确 backing 页数。
pub(crate) fn backing_pages() -> usize {
    (0..SIZES.len())
        .map(|class| cache(class).backing.load(Ordering::Relaxed))
        .sum()
}

#[cfg(feature = "ktest")]
#[derive(Clone, Copy)]
pub(crate) struct SlabSnapshot {
    pub(crate) head_pfn: usize,
    pub(crate) cache: usize,
    pub(crate) order: usize,
    pub(crate) objects: usize,
    pub(crate) inuse: usize,
    pub(crate) shared: usize,
    pub(crate) location: SlabLocation,
    pub(crate) next: usize,
    pub(crate) previous: usize,
}

#[cfg(feature = "ktest")]
#[derive(Clone, Copy)]
pub(crate) struct CpuSnapshot {
    pub(crate) active: usize,
    pub(crate) local_count: usize,
    pub(crate) partial: usize,
    pub(crate) partial_count: usize,
}

#[cfg(feature = "ktest")]
#[derive(Clone, Copy)]
pub(crate) struct NodeSnapshot {
    pub(crate) partial: usize,
    pub(crate) count: usize,
}

/// 返回实际生产缓存几何，不重建另一份选阶算法。
#[cfg(feature = "ktest")]
pub(crate) fn geometry(class: usize) -> Geometry {
    cache(class).geometry
}

/// 持真实 slab 锁读取仍由消费者保留的 live 对象所在块。
#[cfg(feature = "ktest")]
pub(crate) fn slab_snapshot(pointer: NonNull<u8>) -> SlabSnapshot {
    let pfn = arch::virt_to_phys(pointer.as_ptr() as usize) / PAGE_SIZE;
    let head = descriptor(pfn).slab_head.load(Ordering::Acquire);
    slab_head_snapshot(head).expect("snapshot live slab vanished")
}

/// 持真实头 descriptor 的用途锁观察 slab 成员，其他用途与已归页状态不授予 slab 视图。
#[cfg(feature = "ktest")]
pub(crate) fn slab_head_snapshot(head: usize) -> Option<SlabSnapshot> {
    let slot = pages().get(head)?.usage();
    let state = slot.slab()?;
    Some(SlabSnapshot {
        head_pfn: head,
        cache: state.cache,
        order: state.backing.order(),
        objects: state.objects,
        inuse: state.inuse,
        shared: state.shared,
        location: state.location,
        next: state.next,
        previous: state.previous,
    })
}

/// 持指定本核控制锁取得小快照，远端只观察且不修改其 local 链。
#[cfg(feature = "ktest")]
pub(crate) fn cpu_snapshot(class: usize, owner: usize) -> CpuSnapshot {
    let local = cache(class).cpu(owner).lock();
    CpuSnapshot {
        active: local.active,
        local_count: local.local_count,
        partial: local.partial,
        partial_count: local.partial_count,
    }
}

/// 持真实 node 锁读取成员头和计数，不将多个快照视为全局原子状态。
#[cfg(feature = "ktest")]
pub(crate) fn node_snapshot(class: usize) -> NodeSnapshot {
    let node = cache(class).node.lock();
    NodeSnapshot {
        partial: node.head,
        count: node.count,
    }
}
