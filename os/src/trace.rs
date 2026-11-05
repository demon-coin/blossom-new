//! 默认关闭的固定每 CPU 事件环；普通任务串行控制采集与停稳快照。

use alloc::vec::Vec;
use core::fmt::{self, Write};
use core::ptr;
use core::sync::atomic::{AtomicBool, Ordering};

use crate::boot::MAX_CPUS;
use crate::fs::FsError;
use crate::sync::{IrqSaveGuard, SleepMutex, SpinLock};
use crate::task::{self, TaskId};
use crate::{cpu, time};

const CAPACITY: usize = 128;

// 七个固定字段的十进制最大值及分隔符不超过 192 字节。
const RECORD_TEXT_CAPACITY: usize = 192;
const CPU_TEXT_CAPACITY: usize = 80;
const HEADER_TEXT_CAPACITY: usize = 64;

static ENABLED: AtomicBool = AtomicBool::new(false);
static CONTROL: SleepMutex<()> = SleepMutex::new(());
static RINGS: [SpinLock<Ring>; MAX_CPUS] = [const { SpinLock::new(Ring::new()) }; MAX_CPUS];

/// 固定事件编号，文本不依赖动态名称或运行期 schema。
#[derive(Clone, Copy)]
#[repr(u8)]
pub(crate) enum Kind {
    TaskDispatch = 1,
    WaitResume = 2,
    NetRxBatch = 3,
    NetTxCompleteBatch = 4,
    TcpState = 5,
    ProtocolDeadline = 6,
}

/// 记录只拥有标量；任务退出、端点退休或用户空间变化不影响已提交内容。
#[derive(Clone, Copy)]
struct Record {
    seq: u64,
    monotonic_ns: u64,
    cpu: usize,
    task: Option<TaskId>,
    kind: Kind,
    object: u64,
    value: u64,
}

/// 序号只在本环内递增，槽覆盖始终以完整记录为单位。
#[derive(Clone, Copy)]
struct Ring {
    records: [Option<Record>; CAPACITY],
    next_seq: u64,
}

impl Ring {
    /// 静态建立空环；START 在停稳状态下恢复同样的初始序号。
    const fn new() -> Self {
        Self {
            records: [None; CAPACITY],
            next_seq: 1,
        }
    }

    /// 从真实提交序号得到当前保留数量，不维护另一份计数。
    fn retained(&self) -> usize {
        (self.next_seq - 1).min(CAPACITY as u64) as usize
    }

    /// 在本 CPU 的 IRQ-save 短锁内完整提交，序号耗尽直接暴露错误。
    fn push(&mut self, mut record: Record) {
        let seq = self.next_seq;
        let next_seq = seq.checked_add(1).expect("trace sequence exhausted");
        record.seq = seq;
        self.records[((seq - 1) % CAPACITY as u64) as usize] = Some(record);
        self.next_seq = next_seq;
    }
}

/// 预先准备文本容量后只追加固定格式，格式化阶段不重新分配。
struct SnapshotText(Vec<u8>);

impl Write for SnapshotText {
    /// 容量由固定字段上界准备；上界错误不得静默截断或额外申请内存。
    fn write_str(&mut self, text: &str) -> fmt::Result {
        assert!(
            text.len() <= self.0.capacity() - self.0.len(),
            "trace text exceeds reserved capacity"
        );
        self.0.extend_from_slice(text.as_bytes());
        Ok(())
    }
}

/// 查询采集开关，不取得生产者锁，也不读取时间或任务身份。
pub(crate) fn enabled() -> bool {
    ENABLED.load(Ordering::Acquire)
}

/// 记录已初始化 CPU 上的正常事件，不分配、不日志、不调用生产者回调。
/// 仅普通执行段及已核合法的 hardirq 可调用；NMI、panic、timekeeper 写侧、
/// trace 内部和 CSD 屏障回调不得调用，以免等待被自身打断的写者或递归进入环。
pub(crate) fn record(kind: Kind, object: u64, value: u64) {
    if !ENABLED.load(Ordering::Relaxed) {
        return;
    }

    // 进入 IRQ-save 后才能固定 CPU；STOP 的本核 CSD 不能越过本次完整提交。
    let _irq = IrqSaveGuard::new();
    let cpu = cpu::current().id;
    if !ENABLED.load(Ordering::Acquire) {
        return;
    }
    let record = Record {
        seq: 0,
        monotonic_ns: time::now().as_nanos(),
        cpu,
        task: task::current_task_id(),
        kind,
        object,
        value,
    };
    RINGS[cpu].lock().push(record);
}

/// 普通可睡眠任务在停稳后原位清空实际 CPU 的环，再发布新采集；已经采集时返回 Busy。
pub(crate) fn start() -> Result<(), FsError> {
    let _control = CONTROL.lock();
    if enabled() {
        return Err(FsError::Busy);
    }

    // 同一控制锁保证前次 STOP 的全部 CSD 已返回，不会与清环并行提交。
    for owner in cpu::actual_set().iter() {
        let mut ring = RINGS[owner].lock();
        ring.records.fill(None);
        ring.next_seq = 1;
    }
    ENABLED.store(true, Ordering::Release);
    Ok(())
}

/// 在目标核 IRQ-off 域形成短执行屏障，不访问参数、不睡眠、不递归调用。
///
/// # Safety
/// 只能经 cpu::call 在已在线 CPU 上调用；不得从此动作再次记录 trace。
unsafe fn freeze_cpu(_: *mut ()) {}

/// 普通可睡眠任务先禁止新提交，再等待全部在线 CPU 越过短 CSD 屏障。
/// 调用者能服务本地 IPI，等待期间不持环锁；重复 STOP 在已停稳时成功。
pub(crate) fn stop() -> Result<(), FsError> {
    let _control = CONTROL.lock();
    if !enabled() {
        return Ok(());
    }
    ENABLED.store(false, Ordering::Release);
    for target in cpu::actual_set().iter() {
        assert!(
            cpu::get(target).online.load(Ordering::Acquire),
            "trace stop requires online CPUs"
        );
        // SAFETY: 普通任务只在全部 CPU 上线后执行，CPU 集合在运行期固定；上面核实在线。
        // CONTROL 的 SleepMutex 已检查可切换/IRQ 开启，等待时无环锁，调用者可服务 IPI。
        // 回调是永久函数且不访问空参数；cpu::call 固定自己的栈节点并同步等到回调最后访问结束，
        // 本调用不借出其它栈存储，回调不睡眠、不 uaccess、不递归 call 或 record。
        unsafe { cpu::call(target, freeze_cpu, ptr::null_mut()) };
    }
    Ok(())
}

/// 普通任务在停稳后生成独立 ASCII 文本；采集中返回 Busy，分配失败不交出部分文本。
/// 控制锁排除重新 START，每个 CPU 只在短锁下复制固定环，锁外完成长格式化。
pub(crate) fn snapshot() -> Result<Vec<u8>, FsError> {
    let _control = CONTROL.lock();
    if enabled() {
        return Err(FsError::Busy);
    }

    let cpus = cpu::count();
    let mut capacity = HEADER_TEXT_CAPACITY + cpus * CPU_TEXT_CAPACITY;
    for ring in &RINGS[..cpus] {
        capacity += ring.lock().retained() * RECORD_TEXT_CAPACITY;
    }

    let mut bytes = Vec::new();
    bytes
        .try_reserve_exact(capacity)
        .map_err(|_| FsError::NoMemory)?;

    let mut text = SnapshotText(bytes);
    writeln!(
        text,
        "ktrace version=1 unit=ns cpus={} capacity={}",
        cpus, CAPACITY
    )
    .expect("trace header formatting failed");
    for (cpu, source) in RINGS[..cpus].iter().enumerate() {
        let ring = *source.lock();
        let retained = ring.retained() as u64;
        let overwritten = ring.next_seq - 1 - retained;
        writeln!(
            text,
            "cpu id={} next_seq={} overwritten={}",
            cpu, ring.next_seq, overwritten
        )
        .expect("trace CPU formatting failed");
        for seq in ring.next_seq - retained..ring.next_seq {
            let record = ring.records[((seq - 1) % CAPACITY as u64) as usize]
                .expect("trace retained record is missing");
            assert_eq!(record.seq, seq, "trace slot sequence mismatch");
            write!(
                text,
                "record seq={} ns={} cpu={} task=",
                record.seq, record.monotonic_ns, record.cpu
            )
            .expect("trace identity formatting failed");
            match record.task {
                Some(task) => write!(text, "{}", task.as_usize()),
                None => text.write_str("0"),
            }
            .expect("trace task formatting failed");
            writeln!(
                text,
                " kind={} object={} value={}",
                record.kind as u8, record.object, record.value
            )
            .expect("trace event formatting failed");
        }
    }
    Ok(text.0)
}
