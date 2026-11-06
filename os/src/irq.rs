//! 外部中断（External IRQ）域管理、共享中断处理函数（Action）分发与同步注销。
//!
//! # 模块职责与架构定位
//! 本模块管理全局外部中断控制器（如 RISC-V PLIC 或 LoongArch EIOINTC/PCH-PIC）下的中断源描述符与驱动中断回调：
//! - 上游调用者：
//!   - 启动入口（`boot`）调用 `initialize` 初始化外部中断控制器并发布中断源数量上界。
//!   - 设备驱动（UART、VirtIO、块设备、网卡等）调用 `map_source`、`register`、`IrqRegistration::enable` / `disable_sync` / `unregister_sync` 注册与回收中断处理函数。
//!   - 架构硬中断入口调用 `handle_external` 认领（Claim）、分发并完成（Complete）外部设备中断。
//!   - 观测与 `/proc/interrupts` 接口调用 `source_count` 与 `source_snapshot` 读取每核中断计数。
//! - 下游依赖：调用 `arch::external_init`、`external_mask`、`external_unmask`、`external_claim`、`external_complete` 操作底层硬件中断控制器。
//!
//! # 核心设计与生命周期状态机
//! 1. **共享中断描述符（`Descriptor`）与无锁回调执行**：
//!    每个外部中断源最多支持 `MAX_ACTIONS`（8 个）共享处理函数。当硬中断到来时，`handle_external` 在持有中断源描述符自旋锁（`SOURCES[i]`）期间将已启用（`enabled == true`）的槽位引用计数 `active` 加 1 并复制出回调指针，随后**释放描述符自旋锁**，在锁外依次调用设备中断处理函数，完成后重新获取锁递减 `active` 并执行硬件 `external_complete`。
//! 2. **注册句柄状态机（`RegistrationState`）与同步等待**：
//!    ```text
//!    register() ──> [ Live (enabled = false) ]
//!                      │                ▲
//!            enable()  │                │ active == 0 后恢复
//!                      ▼                │
//!                   [ Live (enabled = true) ]
//!                      │                │
//!                      │ disable_sync() ┴─> [ Disabling (enabled = false, 自旋等待 active == 0) ]
//!                      │
//!                      └ unregister_sync() ──> [ Removing (enabled = false, 自旋等待 active == 0) ]
//!                                                    │
//!                                                    │ active == 0: 清空 slot.action 并从 order 数组移除
//!                                                    ▼
//!                                              [ Closed ] (槽位可被后续 register 安全复用)
//!    ```
//!    这种“持锁摘除准入并递增/递减 `active` + 锁外自旋等待 `active == 0`”的协议，保证了 `unregister_sync()` 返回后没有任何 CPU 仍在执行该回调，驱动可以安全地释放回调上下文（`context`）内存而不会发生悬垂指针（UAF）。
//!
//! # 锁获取顺序（Lock Ordering）与并发契约
//! 1. **锁层级顺序**：
//!    `IrqRegistration::state`（单个注册句柄自旋锁） ➔ `SOURCES[number]`（中断源 `Descriptor` 自旋锁） ➔ 架构中断控制器寄存器操作（`external_mask` / `external_unmask`）。
//!    在 `testing::enable_pair` 中同时锁定两个句柄时，严格按 `(first.state -> second.state -> SOURCES[number])` 顺序获取，且调用方必须保证不以相反顺序并发调用。
//! 2. **禁止在锁内或同线回调中阻塞等待**：
//!    - 设备中断处理函数（`Action::handler`）在所有 `IrqRegistration::state` 与 `SOURCES` 自旋锁之外执行。
//!    - 严禁在同一个中断源的 `handler` 内部调用 `enable`、`disable_sync` 或 `unregister_sync`，否则会导致自旋死锁。

use crate::arch;
use crate::boot::{BootInfo, MAX_CPUS, MAX_INTERRUPT_SOURCES};
use crate::cpu;
use crate::klog::Level;
use crate::sync::SpinLock;
use core::hint::spin_loop;
use core::sync::atomic::{AtomicU32, Ordering};

/// 单个外部中断源最多允许挂载的共享中断处理函数（Action）数量。
const MAX_ACTIONS: usize = 8;
/// 单次进入 `handle_external` 硬中断入口最多连续认领（Claim）的外部中断次数，防止持续中断风暴饿死主循环。
const MAX_CLAIMS_PER_ENTRY: usize = 64;
/// 已初始化的外部中断控制器支持的中断源数量上界（为 `0` 表示尚未初始化）。
static SOURCE_COUNT: AtomicU32 = AtomicU32::new(0);

/// 经过范围校验的外部中断源编号（元组字段 `0` 为硬件中断控制器内的原始 Source ID，满足 `0 < id < SOURCE_COUNT` 或架构合法范围）。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct IrqNumber(u32);

#[cfg(feature = "ktest")]
impl IrqNumber {
    /// 返回底层硬件中断源编号（仅供内核自测验证路由使用）。
    pub(crate) fn source(self) -> u32 {
        self.0
    }
}

/// 中断源映射或中断处理函数注册时的错误类型。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum IrqError {
    /// 外部中断控制器域尚未初始化（`SOURCE_COUNT == 0`）。
    NotInitialized,
    /// 请求的中断源编号超出控制器有效范围（或在 RISC-V PLIC 下为保留的 0 号源）。
    InvalidSource,
    /// 目标中断源上的共享回调槽位（`MAX_ACTIONS`）已满。
    ActionsFull,
}

/// 单个设备中断回调及其上下文指针。
#[derive(Clone, Copy)]
struct Action {
    /// 中断处理函数指针：接收 `context` 裸指针，若本设备确实产生了中断并已完成处理则返回 `true`。
    handler: unsafe fn(*mut ()) -> bool,
    /// 传递给 `handler` 的设备私有上下文指针，生命周期由注册方维持至 `unregister_sync` 返回。
    context: *mut (),
}

// SAFETY:
// 1. 注册方通过 `register` 的 Safety 契约保证 `context` 指向的对象在 `unregister_sync` 返回前始终有效且满足跨核访问同步要求。
// 2. `Action` 仅在持有中断源 `Descriptor` 自旋锁时被复制，并由执行中断的 CPU 调用。
unsafe impl Send for Action {}

/// 中断源描述符中的单个回调槽位。
struct Slot {
    /// 当前槽位登记的中断回调；为 `None` 表示空槽位。
    action: Option<Action>,
    /// 当前槽位是否已开放中断准入（仅当 `true` 时，`handle_external` 才会分发给该槽位）。
    enabled: bool,
    /// 当前正在锁外执行该槽位 `handler` 的活跃计数；`disable_sync` / `unregister_sync` 必须等待其归零。
    active: usize,
}

impl Slot {
    /// 构造一个未绑定回调、未启用且活跃计数为 0 的空槽位。
    const fn empty() -> Self {
        Self {
            action: None,
            enabled: false,
            active: 0,
        }
    }
}

/// 单个外部中断源的完整状态描述符（由 `SOURCES` 中的自旋锁保护）。
struct Descriptor {
    /// 固定容量的回调槽位数组。
    slots: [Slot; MAX_ACTIONS],
    /// 按注册先后顺序记录活跃槽位下标的紧凑数组（前 `count` 项有效），保证回调按注册顺序执行。
    order: [usize; MAX_ACTIONS],
    /// 当前已注册的回调槽位数量（`0..=MAX_ACTIONS`）。
    count: usize,
    /// 标记当前中断源是否正由某个 CPU 在锁外执行中断回调（防止重入并在处理期间保持硬件屏蔽）。
    handling: bool,
    /// 是否已经打印过“未注册处理函数却收到中断”的警告日志（避免重复刷屏）。
    reported: bool,
    /// 每个逻辑 CPU 认领（Claim）该中断源的累计次数（按逻辑 CPU 编号索引）。
    interrupts: [usize; MAX_CPUS],
    /// 该中断源是否曾经被注册过回调或被硬件触发过（供 `/proc/interrupts` 过滤从未使用的空源）。
    observed: bool,
}

impl Descriptor {
    /// 编译期构造一个初始为空的中断源描述符。
    const fn empty() -> Self {
        Self {
            slots: [const { Slot::empty() }; MAX_ACTIONS],
            order: [0; MAX_ACTIONS],
            count: 0,
            handling: false,
            reported: false,
            interrupts: [0; MAX_CPUS],
            observed: false,
        }
    }

    /// 根据当前描述符状态更新硬件中断控制器的屏蔽位：
    /// 仅当没有 CPU 正在处理该中断（`!self.handling`）且至少有一个槽位处于 `enabled` 状态时，才解除硬件屏蔽（Unmask）；否则屏蔽（Mask）该中断线。
    fn update_mask(&self, source: u32) {
        if !self.handling && self.slots.iter().any(|slot| slot.enabled) {
            arch::external_unmask(source);
        } else {
            arch::external_mask(source);
        }
    }
}

/// 全局外部中断源描述符表，按硬件中断源编号索引，每项由独立的自旋锁保护。
static SOURCES: [SpinLock<Descriptor>; MAX_INTERRUPT_SOURCES] =
    [const { SpinLock::new(Descriptor::empty()) }; MAX_INTERRUPT_SOURCES];

/// 单个外部中断源的统计信息快照（用于 `/proc/interrupts` 等只读观测接口）。
pub(crate) struct SourceSnapshot {
    /// 各逻辑 CPU 累计认领该中断源的次数。
    pub(crate) interrupts: [usize; MAX_CPUS],
    /// 该中断源是否曾被注册或触发过。
    pub(crate) observed: bool,
}

/// 返回当前已初始化的外部中断源数量上界（为 `0` 表示外部中断域尚未初始化）。
pub(crate) fn source_count() -> u32 {
    SOURCE_COUNT.load(Ordering::Acquire)
}

/// 在目标中断源的自旋锁临界区内复制中断计数与观测标志快照；若中断源编号非法则返回 `None`。
pub(crate) fn source_snapshot(source: u32) -> Option<SourceSnapshot> {
    let number = map_source(source).ok()?;
    let descriptor = SOURCES[number.0 as usize].lock();
    Some(SourceSnapshot {
        interrupts: descriptor.interrupts,
        observed: descriptor.observed,
    })
}

/// 返回指定中断源上当前已注册（含尚未 `enable`）的共享回调数量，供内核自测使用。
#[cfg(feature = "ktest")]
pub(crate) fn test_action_count(number: IrqNumber) -> usize {
    SOURCES[number.0 as usize]
        .lock()
        .slots
        .iter()
        .filter(|slot| slot.action.is_some())
        .count()
}

/// 内核自测专用的共享中断观测与双回调原子启用辅助模块。
#[cfg(feature = "ktest")]
pub(crate) mod testing {
    use super::{IrqNumber, IrqRegistration, RegistrationState, SOURCES};

    /// 汇总指定中断源在所有在线逻辑 CPU 上的累计认领（Claim）总次数。
    pub(crate) fn dispatch_count(number: IrqNumber) -> usize {
        SOURCES[number.0 as usize].lock().interrupts[..crate::cpu::count()]
            .iter()
            .fold(0usize, |total, &count| total.wrapping_add(count))
    }

    /// 在同一个描述符自旋锁临界区内同时启用同一中断源上的两个处于 `Live` 状态的注册句柄，并统一更新一次硬件屏蔽位。
    ///
    /// 若两个句柄指向同一对象、不属于同一中断源、槽位相同或任一句柄非 `Live` 状态，则返回 `false` 且不修改任何状态。
    ///
    /// # 锁顺序与调用前提
    /// 严格按 `first.state` ➔ `second.state` ➔ `SOURCES[number]` 顺序加锁；调用方必须独占这两个句柄，禁止以相反参数顺序并发调用 `enable_pair` 或在同线中断回调内调用。
    pub(crate) fn enable_pair(first: &IrqRegistration, second: &IrqRegistration) -> bool {
        if core::ptr::eq(first, second)
            || first.number != second.number
            || first.slot == second.slot
        {
            return false;
        }
        let first_state = first.state.lock();
        let second_state = second.state.lock();
        if *first_state != RegistrationState::Live || *second_state != RegistrationState::Live {
            return false;
        }
        let mut descriptor = SOURCES[first.number.0 as usize].lock();
        descriptor.slots[first.slot].enabled = true;
        descriptor.slots[second.slot].enabled = true;
        descriptor.update_mask(first.number.0);
        true
    }
}

/// 中断注册句柄（[`IrqRegistration`]）的内部生命周期状态。
#[derive(Clone, Copy, Eq, PartialEq)]
enum RegistrationState {
    /// 句柄有效，可以正常启用（`enable`）或停用（`disable_sync`）。
    Live,
    /// 正在执行 `disable_sync`：已关闭槽位准入（`enabled = false`），正等待锁外正在执行的回调归零（`active == 0`）。
    Disabling,
    /// 正在执行 `unregister_sync`：已关闭槽位准入，正等待正在执行的回调归零以便彻底清空槽位。
    Removing,
    /// 已完成注销（`unregister_sync`），底层槽位已释放并可被新注册复用，当前句柄不再访问描述符槽位。
    Closed,
}

/// 单个已注册共享中断回调的所有权凭证（Token）。
///
/// 驱动在释放回调上下文（`context`）内存前，必须先停止设备中断源并显式调用 [`Self::unregister_sync`]；直接丢弃（Drop）本结构体不会自动撤销中断注册或等待正在执行的回调结束。
pub struct IrqRegistration {
    /// 所属的外部中断源编号。
    number: IrqNumber,
    /// 在所属中断源 `Descriptor::slots` 数组中的槽位下标。
    slot: usize,
    /// 保护当前句柄生命周期状态转换的自旋锁（锁层级高于 `SOURCES[number]`）。
    state: SpinLock<RegistrationState>,
}

impl IrqRegistration {
    /// 开放当前回调槽位的中断准入，并同步更新硬件中断控制器的屏蔽位；若句柄正在停止或已关闭则忽略。
    pub fn enable(&self) {
        let state = self.state.lock();
        if *state != RegistrationState::Live {
            return;
        }
        let mut descriptor = SOURCES[self.number.0 as usize].lock();
        descriptor.slots[self.slot].enabled = true;
        descriptor.update_mask(self.number.0);
    }

    /// 关闭当前回调槽位的中断准入，并自旋等待所有正在执行该回调的 CPU 退出（`active == 0`）；同中断线上的其他共享回调不受影响。
    ///
    /// 严禁在当前中断源的中断处理函数内部调用，且调用方不得持有该回调退出所需的锁。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "UART 共享中断测试单独关闭 action 并等待真实执行引用结束"
        )
    )]
    pub fn disable_sync(&self) {
        self.stop(false);
    }

    /// 彻底移除当前回调槽位并自旋等待所有正在执行该回调的 CPU 退出（`active == 0`）。
    ///
    /// 本函数返回后，内核保证不再调用该回调，驱动可安全释放 `context` 指向的资源。严禁在当前中断源的中断处理函数内部调用。
    pub fn unregister_sync(&self) {
        self.stop(true);
    }

    /// 查询当前回调槽位是否处于有效且已开放准入（`enabled`）的状态。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(dead_code, reason = "UART 共享中断测试观察新旧 action 的独立准入状态")
    )]
    pub fn is_enabled(&self) -> bool {
        let state = self.state.lock();
        *state == RegistrationState::Live
            && SOURCES[self.number.0 as usize].lock().slots[self.slot].enabled
    }

    /// 同步停用（`remove == false`）或彻底注销（`remove == true`）当前回调的内部实现。
    fn stop(&self, remove: bool) {
        // Step 1: 按 `self.state -> SOURCES[self.number]` 锁顺序将状态切换为 Disabling/Removing，清除槽位 enabled 标志并更新硬件屏蔽位。
        {
            let mut state = self.state.lock();
            if *state == RegistrationState::Closed {
                return;
            }
            if remove {
                *state = RegistrationState::Removing;
            } else if *state == RegistrationState::Live {
                *state = RegistrationState::Disabling;
            }
            let mut descriptor = SOURCES[self.number.0 as usize].lock();
            descriptor.slots[self.slot].enabled = false;
            descriptor.update_mask(self.number.0);
        }

        // Step 2: 在自旋锁外循环等待正在锁外执行回调的计数 `active` 归零；一旦归零，在锁内完成槽位清理或恢复为 Live 状态。

        loop {
            {
                let mut state = self.state.lock();
                if matches!(*state, RegistrationState::Live | RegistrationState::Closed) {
                    return;
                }
                let mut descriptor = SOURCES[self.number.0 as usize].lock();
                if descriptor.slots[self.slot].active == 0 {
                    if *state == RegistrationState::Removing {
                        descriptor.slots[self.slot].action = None;
                        let position = descriptor.order[..descriptor.count]
                            .iter()
                            .position(|&slot| slot == self.slot)
                            .expect("IRQ action missing from source order");
                        let count = descriptor.count;
                        descriptor.order.copy_within(position + 1..count, position);
                        descriptor.count -= 1;
                        // 在同一对锁内将槽位清空并将句柄标记为 Closed，保证槽位被后续 register 复用后旧句柄绝不会再触碰该槽位。
                        *state = RegistrationState::Closed;
                    } else {
                        *state = RegistrationState::Live;
                    }
                    return;
                }
            }
            spin_loop();
        }
    }
}

/// 初始化外部中断控制器并发布全局外部中断源数量上界；初始化完成时所有外部中断源均处于硬件屏蔽状态。
///
/// # Safety
/// 1. `info` 必须包含经启动阶段验证的真实中断控制器配置（`source_count` 在 `(0, MAX_INTERRUPT_SOURCES]` 范围内），且对应的 MMIO/IOCSR 地址映射已建立。
/// 2. 只能由主核（BSP）在本地硬中断关闭、辅核（AP）尚未开始处理外部中断时调用唯一一次。
pub unsafe fn initialize(info: &BootInfo) {
    assert_eq!(
        SOURCE_COUNT.load(Ordering::Relaxed),
        0,
        "IRQ domain already initialized"
    );
    assert!(
        0 < info.interrupts.source_count
            && info.interrupts.source_count as usize <= MAX_INTERRUPT_SOURCES
    );
    // SAFETY: 调用方保证 `info` 中的中断控制器硬件信息与地址映射有效，且当前由 BSP 在关中断状态下独占执行一次性初始化。
    unsafe { arch::external_init(info) };
    SOURCE_COUNT.store(info.interrupts.source_count, Ordering::Release);
}

/// 验证原始硬件中断源编号 `source` 并将其包装为合法的 [`IrqNumber`]（RISC-V 下额外拒绝保留的 0 号源）。
pub fn map_source(source: u32) -> Result<IrqNumber, IrqError> {
    let count = SOURCE_COUNT.load(Ordering::Acquire);
    if count == 0 {
        return Err(IrqError::NotInitialized);
    }
    if count <= source || (cfg!(target_arch = "riscv64") && source == 0) {
        return Err(IrqError::InvalidSource);
    }
    Ok(IrqNumber(source))
}

/// 在指定外部中断源的空闲槽位中注册一个中断处理函数（初始处于未启用状态，需随后调用 [`IrqRegistration::enable`] 开放准入）。
///
/// # Safety
/// 1. `handler` 与 `context` 必须在对应的 [`IrqRegistration::unregister_sync`] 调用返回前始终保持有效，且 `context` 指向的数据满足跨核与中断并发访问要求。
/// 2. `handler` 必须在硬中断上下文中快速完成且不可睡眠，返回 `true` 表示本设备已确认并处理了此次中断，返回 `false` 表示非本设备中断。
/// 3. `handler` 内部严禁调用同一中断源的 `enable`、`disable_sync` 或 `unregister_sync`，也不得请求任何可能被正在调用 `disable_sync` / `unregister_sync` 的线程持有的锁（避免死锁）。
pub unsafe fn register(
    number: IrqNumber,
    handler: unsafe fn(*mut ()) -> bool,
    context: *mut (),
) -> Result<IrqRegistration, IrqError> {
    let action = Action { handler, context };
    let mut descriptor = SOURCES[number.0 as usize].lock();
    let slot = descriptor
        .slots
        .iter()
        .position(|slot| slot.action.is_none())
        .ok_or(IrqError::ActionsFull)?;
    let registration = IrqRegistration {
        number,
        slot,
        state: SpinLock::new(RegistrationState::Live),
    };
    descriptor.slots[slot].action = Some(action);
    let count = descriptor.count;
    descriptor.order[count] = slot;
    descriptor.count += 1;
    descriptor.observed = true;
    Ok(registration)
}

/// 外部硬中断分发入口：循环从硬件中断控制器认领（Claim）待处理中断（单次最多 `MAX_CLAIMS_PER_ENTRY` 个），在锁外执行所有已启用的共享回调，并向控制器发送完成（Complete）确认。
pub fn handle_external() {
    for _ in 0..MAX_CLAIMS_PER_ENTRY {
        // Step 1: 向底层中断控制器认领一个挂起的外部中断源；若无待处理中断则立即退出循环。
        let Some(source) = arch::external_claim() else {
            break;
        };
        let number = map_source(source).expect("controller claimed source outside IRQ domain");
        let mut calls = [None; MAX_ACTIONS];
        let mut count = 0;

        // Step 2: 获取该中断源描述符锁，屏蔽该硬件中断线，统计本核中断次数，并为所有已启用槽位递增 `active` 活跃引用计数后拍下回调快照。
        {
            let mut descriptor = SOURCES[number.0 as usize].lock();
            arch::external_mask(source);
            assert!(!descriptor.handling, "IRQ source routed to concurrent CPUs");
            descriptor.handling = true;
            let owner = cpu::current().id;
            descriptor.interrupts[owner] = descriptor.interrupts[owner].wrapping_add(1);
            descriptor.observed = true;
            // 检查 `enabled` 与递增 `active` 必须在同一次持锁内原子完成，防止并发的 `unregister_sync` 在回调执行前提前释放 `context`。
            for index in 0..descriptor.count {
                let slot = descriptor.order[index];
                let entry = &mut descriptor.slots[slot];
                if entry.enabled {
                    entry.active += 1;
                    calls[count] = Some((slot, entry.action.expect("enabled IRQ action missing")));
                    count += 1;
                }
            }
        }

        // Step 3: 在不持有任何中断描述符锁或注册句柄锁的状态下，按注册顺序依次调用快照中的设备中断处理函数，每执行完一个立即加锁递减其 `active` 计数。
        let mut handled = false;
        for &(slot, action) in calls[..count].iter().flatten() {
            // SAFETY:
            // 1. Step 2 已在持有 `SOURCES[number]` 锁时将对应槽位的 `active` 加 1，并发的 `unregister_sync` 必须等待 `active == 0` 才会返回，因此在下面递减 `active` 之前 `action.context` 保证有效。
            // 2. 当前已释放 `SOURCES[number]` 锁，满足在锁外调用设备处理函数的锁顺序契约。
            handled |= unsafe { (action.handler)(action.context) };
            SOURCES[number.0 as usize].lock().slots[slot].active -= 1;
        }

        // Step 4: 重新获取中断源描述符锁，向硬件控制器发送完成确认（Complete），清除 `handling` 标志并根据最新槽位状态恢复硬件屏蔽位；若无人处理且无注册回调则记录一次警告。
        let report = {
            let mut descriptor = SOURCES[number.0 as usize].lock();
            // SAFETY:
            // 1. `source` 来自本轮循环 Step 1 在当前 CPU 上成功认领（Claim）且尚未完成（Complete）的真实中断源。
            // 2. 所有共享回调均已执行完毕，且当前持有 `SOURCES[number]` 自旋锁，不会与其他认领或屏蔽操作冲突。
            unsafe { arch::external_complete(source) };
            descriptor.handling = false;
            descriptor.update_mask(source);
            let report = !handled && descriptor.count == 0 && !descriptor.reported;
            descriptor.reported |= report;
            report
        };
        if report {
            crate::klog!(Level::Warn, "IRQ source {} is not registered", source);
        }
    }
}
