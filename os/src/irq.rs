//! 有界外部 IRQ domain、共享 action 与同步注销；处理者在全部短锁之外执行。

use crate::arch;
use crate::boot::{BootInfo, MAX_CPUS, MAX_INTERRUPT_SOURCES};
use crate::cpu;
use crate::klog::Level;
use crate::sync::SpinLock;
use core::hint::spin_loop;
use core::sync::atomic::{AtomicU32, Ordering};

const MAX_ACTIONS: usize = 8;
const MAX_CLAIMS_PER_ENTRY: usize = 64;
static SOURCE_COUNT: AtomicU32 = AtomicU32::new(0);

/// 当前外部控制器 domain 内已经核对范围的源身份。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct IrqNumber(u32);

#[cfg(feature = "ktest")]
impl IrqNumber {
    /// 返回当前真实 domain 的源号，供硬件测试记录已核对的设备路由。
    pub(crate) fn source(self) -> u32 {
        self.0
    }
}

/// IRQ 资源登记的有界失败原因。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum IrqError {
    NotInitialized,
    InvalidSource,
    ActionsFull,
}

#[derive(Clone, Copy)]
struct Action {
    handler: unsafe fn(*mut ()) -> bool,
    context: *mut (),
}

// SAFETY: 注册者保持 context 到同步注销完成，共享访问由 handler 的实际协议保护。
unsafe impl Send for Action {}

struct Slot {
    action: Option<Action>,
    enabled: bool,
    active: usize,
}

impl Slot {
    /// 初始化尚未发布处理者的固定槽。
    const fn empty() -> Self {
        Self {
            action: None,
            enabled: false,
            active: 0,
        }
    }
}

struct Descriptor {
    slots: [Slot; MAX_ACTIONS],
    order: [usize; MAX_ACTIONS],
    count: usize,
    handling: bool,
    reported: bool,
    interrupts: [usize; MAX_CPUS],
    observed: bool,
}

impl Descriptor {
    /// 建立不需要运行期分配的源状态。
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

    /// 复查当前准入状态，只有完成流之外才允许重新开放硬件线。
    fn update_mask(&self, source: u32) {
        if !self.handling && self.slots.iter().any(|slot| slot.enabled) {
            arch::external_unmask(source);
        } else {
            arch::external_mask(source);
        }
    }
}

static SOURCES: [SpinLock<Descriptor>; MAX_INTERRUPT_SOURCES] =
    [const { SpinLock::new(Descriptor::empty()) }; MAX_INTERRUPT_SOURCES];

/// 单个外部源的逐核真实 claim 快照；观察记录在同步注销后仍然保留。
pub(crate) struct SourceSnapshot {
    pub(crate) interrupts: [usize; MAX_CPUS],
    pub(crate) observed: bool,
}

/// 返回已发布的真实控制器源上界，尚未初始化时没有可观察的外部源。
pub(crate) fn source_count() -> u32 {
    SOURCE_COUNT.load(Ordering::Acquire)
}

/// 在单个源短锁内复制计数与观察事实，不把 descriptor 或锁移交给渲染者。
pub(crate) fn source_snapshot(source: u32) -> Option<SourceSnapshot> {
    let number = map_source(source).ok()?;
    let descriptor = SOURCES[number.0 as usize].lock();
    Some(SourceSnapshot {
        interrupts: descriptor.interrupts,
        observed: descriptor.observed,
    })
}

/// 只在串行设备准备验收中数真实 action，包括尚未 enable 的登记，不推断全线状态。
#[cfg(feature = "ktest")]
pub(crate) fn test_action_count(number: IrqNumber) -> usize {
    SOURCES[number.0 as usize]
        .lock()
        .slots
        .iter()
        .filter(|slot| slot.action.is_some())
        .count()
}

/// 真实共享 IRQ 的测试观测与双 action 准入；保持生产登记和同步停止责任。
#[cfg(feature = "ktest")]
pub(crate) mod testing {
    use super::{IrqNumber, IrqRegistration, RegistrationState, SOURCES};

    /// 汇总此源的逐核真实 claim 次数，不推断任一设备 ISR 或共享 action 数。
    pub(crate) fn dispatch_count(number: IrqNumber) -> usize {
        SOURCES[number.0 as usize].lock().interrupts[..crate::cpu::count()]
            .iter()
            .fold(0usize, |total, &count| total.wrapping_add(count))
    }

    /// 在同一 descriptor 临界区开放两个不同的 Live action，最后一次复查硬件屏蔽。
    /// 源不同、token 或槽相同、任一 token 非 Live 时返回 false，不改变任何准入。
    ///
    /// 调用 fixture 必须独占两 token，不能并发 stop 或以逆序调用另一个 enable_pair。
    /// 锁序为 first token→second token→descriptor→controller；不得在同线 handler 中
    /// 调用，也不得持有 handler 退出依赖的锁。可持有原 Disk 的 token 存储锁，因为
    /// 原 IRQ handler 不取得这些外部存储锁；设备状态须已按原 enable 合同准备完毕。
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

#[derive(Clone, Copy, Eq, PartialEq)]
enum RegistrationState {
    Live,
    Disabling,
    Removing,
    Closed,
}

/// 一份共享 action 的移动拥有者；必须显式停止设备并同步注销后才能释放 context。
/// 丢弃 token 不停止设备、不等待 handler，也不撤销登记。
pub struct IrqRegistration {
    number: IrqNumber,
    slot: usize,
    state: SpinLock<RegistrationState>,
}

impl IrqRegistration {
    /// 发布已准备好的设备处理者；同步停止或已关闭的 token 不会重新开放。
    pub fn enable(&self) {
        let state = self.state.lock();
        if *state != RegistrationState::Live {
            return;
        }
        let mut descriptor = SOURCES[self.number.0 as usize].lock();
        descriptor.slots[self.slot].enabled = true;
        descriptor.update_mask(self.number.0);
    }

    /// 关闭本 action 的准入并等待其执行引用归零，其他共享处理者继续有效。
    /// 不得从同线 handler 调用，也不得持有 handler 退出所需的资源。
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

    /// 摘除本 action 并等待其执行引用归零；返回后 context 可以释放。
    /// 不得从同线 handler 调用；设备 producer 必须由设备拥有者先停止。
    pub fn unregister_sync(&self) {
        self.stop(true);
    }

    /// 读取本 action 的实际准入状态，关闭的旧 token 不访问复用槽。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(dead_code, reason = "UART 共享中断测试观察新旧 action 的独立准入状态")
    )]
    pub fn is_enabled(&self) -> bool {
        let state = self.state.lock();
        *state == RegistrationState::Live
            && SOURCES[self.number.0 as usize].lock().slots[self.slot].enabled
    }

    /// 在 token→descriptor→controller 锁序下撤准入，全部等待均位于锁外。
    fn stop(&self, remove: bool) {
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
                        // token 与槽在同一锁序中关闭，复用后旧 token 永远不再访问槽。
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

/// 初始化真实控制器并发布唯一外部 domain；所有源初始屏蔽。
///
/// # Safety
/// info 是永久发布的实际资源，MMIO/IOCSR 映射有效；BSP 的 IRQ 关闭，
/// AP 尚未处理外部源，且该 domain 仅初始化一次。
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
    // SAFETY: 调用者保证实际资源、映射和 BSP 独占初始化，尚无准入的 action。
    unsafe { arch::external_init(info) };
    SOURCE_COUNT.store(info.interrupts.source_count, Ordering::Release);
}

/// 将资源的真实 source 转成当前唯一 domain 的合法身份；不接受 CPU exception cause。
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

/// 在固定槽中登记一份尚未启用的共享 action，完整准备后由 token 开放准入。
///
/// # Safety
/// handler/context 必须在 unregister_sync 返回前有效，共享数据遵守设备同步协议。
/// handler 必须在硬 IRQ 中有界执行且返回自身设备是否处理了请求；不得调用同线的
/// enable/disable_sync/unregister_sync 或等待停止调用者持有的资源，允许 is_enabled 查询。
/// 设备状态必须在 enable 前完成准备。
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

/// 有界消费本核真实 claim；每源每次 claim 只计一次，同线所有准入 action 都运行。
pub fn handle_external() {
    for _ in 0..MAX_CLAIMS_PER_ENTRY {
        let Some(source) = arch::external_claim() else {
            break;
        };
        let number = map_source(source).expect("controller claimed source outside IRQ domain");
        let mut calls = [None; MAX_ACTIONS];
        let mut count = 0;
        {
            let mut descriptor = SOURCES[number.0 as usize].lock();
            arch::external_mask(source);
            assert!(!descriptor.handling, "IRQ source routed to concurrent CPUs");
            descriptor.handling = true;
            let owner = cpu::current().id;
            descriptor.interrupts[owner] = descriptor.interrupts[owner].wrapping_add(1);
            descriptor.observed = true;
            // 取得执行引用与准入检查不可拆锁，注销不能在快照与计数之间释放 context。
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
        let mut handled = false;
        for &(slot, action) in calls[..count].iter().flatten() {
            // SAFETY: 源锁内已取得本 action 的执行引用；注册者直到 active=0 的同步注销
            // 返回才释放 context；此处没有持有 token、descriptor 或 controller 锁。
            handled |= unsafe { (action.handler)(action.context) };
            SOURCES[number.0 as usize].lock().slots[slot].active -= 1;
        }
        let report = {
            let mut descriptor = SOURCES[number.0 as usize].lock();
            // SAFETY: source 来自本核尚未完成的真实 claim，固定 BSP 路由保证不会跨核完成；
            // 全部 action 已归还引用，源锁保护完成与当前准入状态的复查。
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
