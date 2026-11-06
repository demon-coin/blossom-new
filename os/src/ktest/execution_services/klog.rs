//! 内核日志（`klog`）环形缓冲区、后台打印任务（Printer Task）、`/kmsg` 设备与 UART 中断背压联合测试。
//!
//! ## 模块职责与架构定位
//! 本模块属于执行服务测试套件（`execution_services`），负责验证内核日志子系统在多上下文（普通任务、调度器/工作队列自旋锁内、高精度定时器硬中断）零堆分配写入、UTF-8 字符边界截断、UART 硬件 FIFO 背压阻塞、`/kmsg` 非阻塞读取与关中断延期 `poll` 唤醒，以及宿主机串口控制协议透传和极端故障（OOM / Timekeeper 写锁重入）下的行为。
//! - **上游调用者**：`ktest` 框架通过 `records`、`uart` 和 `fatal` 三个测试套件入口驱动本模块。
//! - **下游依赖**：内核日志环与后台打印服务（`crate::klog`）、控制台与 UART 驱动（`crate::console`）、虚拟文件系统 `/kmsg` 设备与 `poll` 机制（`crate::fs`）、高精度定时器（`crate::timer`）、调度器（`crate::sched`）及内存分配器（`crate::mm`）。
//!
//! ## 核心设计与关键状态机
//!
//! ### 1. 零分配多上下文日志写入与 UTF-8 截断（`test_records`）
//! - 普通任务、调度器锁内、工作队列池锁内与硬中断回调共用同一个固定容量的环形缓冲区（单条正文上限 `TEXT_CAPACITY = 512` 字节）。
//! - 写入 180 个 3 字节中文字符 `'界'`（共 540 字节）时，格式化器在 510 字节（170 个完整字符）处沿 UTF-8 字符边界截断并置 `truncated = true`，全程不触发任何 `GlobalAlloc` 堆分配。
//!
//! ### 2. UART 硬件背压、Printer 阻塞与 `/kmsg` 游标追赶（`test_uart`）
//! ```text
//! [宿主机串口暂停读取 (KTEST io <id> pause)]
//!    │ 1. 收到宿主机 'H' 握手字节并触发 RX IRQ 后，关闭 RX 中断并填满硬件 UART TX FIFO
//!    ▼
//! [提交 "X26 partial PPP..." (held)]
//!    │ 2. 后台 Printer 写入部分字节后因 TX FIFO 满而阻塞在 TX 中断等待上，并在 PausePoint::AfterPartialWrite 暂停
//!    │ 3. 此时 `/kmsg` 仍可独立非阻塞读出完整的 `held` 记录，且 `business_worker` 正常完成 4096 次计算
//!    ▼
//! [连续提交 72 条 "X26 record=0..71" (环形缓冲区容量为 64)]
//!    ├── 首条在 IrqSaveGuard 关中断下提交：验证日志唤醒 IPI 在开中断后才延后触发 `/kmsg` 的 PollContext
//!    ├── 快游标 (fast) 与 `/kmsg`：每提交一条立即读取，72 条全部无丢失读出 (`lost == 0`)
//!    └── 慢游标 (slow) 与暂停中的 Printer：因环容量为 64，前 8 条 (`0..8`) 被覆盖；
//!        释放暂停后 Printer 检测到 `gaps == 8`，跳过被覆盖记录并追赶输出剩余 64 条 (`8..72`)
//! ```
//!
//! ### 3. 宿主机串口二进制透传握手协议（`test_raw_lookalikes`）
//! ```text
//! Guest (内核测试)                                  Host (宿主机测试运行器)
//! ────────────────                                  ──────────────────────
//! 1. 关闭串口中断，输出协议头:
//!    "# KTEST io <id> <rx|tx> <len> <crc32c> 0\n" ──> 解析协议头，切换为原始字节流模式
//! 2. 轮询等待握手字节 'H'                        <── 发送 'H' 确认就绪
//! 3. 按方向收/发伪造 KTAP 控制文本 (RAW_LOOKALIKES) <─> 校验或发送原始字节流（不触发 KTAP 解析）
//! 4. 读取宿主机 "KTEST IO OK <id>\n" 确认行       <── 校验 CRC32C 通过后发送确认行
//! 5. 清除 RawStage 状态并重新开启串口 RX 中断
//! ```
//!
//! ## 锁获取顺序与并发契约
//! - `Scenario` 内部包含三把独立的自旋锁：`producer`（保存生产者任务句柄）、`raw`（保存串口透传阶段进度 `RawStage`）、`held`（保存背压暂停时的首条日志记录副本）。
//! - 这三把自旋锁保护各自独立的字段，严禁相互嵌套获取；持锁期间仅读写简单标量或克隆引用，不调用任何可能睡眠或触发调度的函数。

use alloc::sync::Arc;
use core::fmt::{self, Write};
use core::hint::black_box;
use core::pin::Pin;
use core::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};

use crate::fs::poll::{PollContext, PollMask, PollTable};
use crate::fs::{FileRef, FsError, SeekFrom, O_NONBLOCK};
use crate::klog::{self as log, Cursor, Level};
use crate::sync::{Completion, SpinLock};
use crate::task::{self, State, Task};
use crate::time::{self as clock, Duration};
use crate::timer::{Timer, TimerKind};
use crate::{arch, console, cpu, mm, sched};

/// 用于测试多字节 UTF-8 字符边界截断的格式化辅助结构体。
struct UnicodeBody;

impl fmt::Display for UnicodeBody {
    /// 连续输出 180 个 3 字节中文字符 `'界'`（共 540 字节，超过单条日志 512 字节容量上限），用于验证截断时保留最后完整字符（170 个 `'界'`，510 字节）。
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        for _ in 0..180 {
            formatter.write_str("界")?;
        }
        Ok(())
    }
}

/// 将字节切片（元组字段 `0`）格式化为小写十六进制字符串的流式包装器（避免额外堆分配）。
struct BodyHex<'a>(&'a [u8]);

impl fmt::Display for BodyHex<'_> {
    /// 将内部字节切片逐字节格式化为两位小写十六进制文本。
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        for byte in self.0 {
            write!(formatter, "{:02x}", byte)?;
        }
        Ok(())
    }
}

/// 带 10 秒硬件计数器超时的条件轮询辅助函数：在超时上限内不断调用 `sched::yield_now()` 让出 CPU，直到 `predicate` 返回 `true`。
fn until(mut predicate: impl FnMut() -> bool, name: &str) {
    let end = arch::read_counter()
        .checked_add(arch::counter_frequency().checked_mul(10).unwrap())
        .unwrap();
    while !predicate() {
        assert!(
            arch::read_counter() < end,
            "klog execution timeout: {}",
            name
        );
        sched::yield_now();
    }
}

/// 普通日志生产者任务入口：在收到 `start` 信号后依次产生普通日志、超长 UTF-8 日志、调度器锁内日志与工作队列锁内日志，随后等待 `exit` 信号退出。
fn producer() -> i32 {
    *scenario().producer.lock() = Some(task::current_task());
    if scenario().stop.load(Ordering::Acquire) {
        return 125;
    }
    scenario().start.wait();
    if scenario().stop.load(Ordering::Acquire) {
        return 125;
    }
    let mut value = 37usize;
    crate::klog!(Level::Info, "X25 ordinary value={}", value);
    value = black_box(99);
    black_box(value);
    crate::klog!(Level::Warn, "{}", UnicodeBody);
    sched::test_klog();
    crate::workqueue::testing::test_klog();
    scenario().produced.store(true, Ordering::Release);
    if !scenario().stop.load(Ordering::Acquire) {
        scenario().exit.wait();
    }
    125
}

/// 高精度定时器硬中断回调函数：断言当前处于硬中断上下文，记录当前任务与 CPU 编号，并写入一条高精度定时器日志。
fn precise_callback(_: Pin<&Timer>) {
    assert!(0 < cpu::current().hardirq_depth.load(Ordering::Relaxed));
    scenario().timer_task.store(
        task::current_task_id().map_or(0, |id| id.as_usize()),
        Ordering::Relaxed,
    );
    scenario()
        .timer_cpu
        .store(cpu::current().id, Ordering::Relaxed);
    let sequence = crate::klog!(Level::Info, "X25 precise timer");
    scenario().timer_seq.store(sequence, Ordering::Release);
}

/// 测试多上下文（普通任务、调度器锁内、工作队列池锁内、高精度定时器硬中断）共用日志环时的零堆分配特性、UTF-8 边界截断与多游标独立读取。
fn test_records() {
    // Step 1: 创建高精度定时器与生产者子任务，等待生产者任务就绪阻塞以及后台日志/工作队列服务静默。
    let timer = scenario()
        .nodes
        .timer(0, TimerKind::Precise, precise_callback, 0);
    let worker = crate::ktest::spawn_result_on(0, |_| Ok(producer())).unwrap();
    until(
        || {
            scenario()
                .producer
                .lock()
                .as_ref()
                .is_some_and(|task| sched::test_state(task) == State::Blocked)
        },
        "producer prepared and blocked",
    );
    log::testing::wait_service_blocked();
    super::workqueue::wait_services_blocked();

    // Step 2: 记录起始日志序号、时间戳与内存分配尝试计数，唤醒生产者任务并触发 5ms 高精度定时器，验证 5 条日志写入全程零堆分配（`allocation_attempts == 0`）。
    let first = log::next_sequence();
    let start_ns = clock::now().as_nanos();
    let attempts = mm::allocation_attempts();
    scenario().start.complete();
    until(
        || scenario().produced.load(Ordering::Acquire),
        "task producers complete",
    );
    assert!(timer
        .as_ref()
        .arm(crate::timer::after(Duration::from_millis(5)).unwrap())
        .unwrap());
    until(
        || scenario().timer_seq.load(Ordering::Acquire) != 0,
        "precise log callback",
    );
    let allocation_attempts = mm::allocation_attempts().checked_sub(attempts).unwrap();
    assert_eq!(allocation_attempts, 0);
    let end_ns = clock::now().as_nanos();
    let last = scenario().timer_seq.load(Ordering::Acquire);
    assert_eq!(last, first + 4);
    assert_eq!(log::next_sequence(), last + 1);

    // Step 3: 使用两个独立的 `Cursor` 从头遍历这 5 条记录，核对序号、日志级别、单调时间戳、任务/CPU 归属以及多字节 UTF-8 截断结果（510 字节 / 170 个 `'界'`）。
    let levels = [
        Level::Info,
        Level::Warn,
        Level::Debug,
        Level::Debug,
        Level::Info,
    ];
    let mut cursors = [Cursor::from_sequence(first), Cursor::from_sequence(first)];
    for cursor in &mut cursors {
        let mut previous_ns = start_ns;
        for (index, level) in levels.iter().enumerate() {
            let read = cursor.read().expect("committed record missing");
            let record = read.record;
            assert_eq!(read.lost, 0);
            assert_eq!(record.seq, first + index as u64);
            assert_eq!(record.level, *level);
            assert_eq!(record.len, record.text().len());
            assert!(record.len <= log::TEXT_CAPACITY);
            assert!(record.text().is_char_boundary(record.len));
            assert!(start_ns <= record.monotonic_ns && record.monotonic_ns <= end_ns);
            assert!(0 < record.monotonic_ns);
            assert!(previous_ns <= record.monotonic_ns);
            previous_ns = record.monotonic_ns;
            if index < 4 {
                assert_eq!(record.task, Some(worker.task_id()));
                assert_eq!(record.cpu, 0);
            } else {
                assert_eq!(
                    record.task.map_or(0, |id| id.as_usize()),
                    scenario().timer_task.load(Ordering::Acquire)
                );
                assert_eq!(record.cpu, scenario().timer_cpu.load(Ordering::Acquire));
            }
            assert_eq!(record.truncated, index == 1);
            match index {
                0 => assert_eq!(record.text(), "X25 ordinary value=37"),
                1 => {
                    assert_eq!(record.len, 510);
                    assert_eq!(record.text().chars().count(), 170);
                    assert!(record.text().chars().all(|character| character == '界'));
                }
                // 调度器锁内与工作队列锁内日志的具体正文格式在下方诊断行中以十六进制导出供宿主机比对。
                2 | 3 => {}
                _ => assert_eq!(record.text(), "X25 precise timer"),
            }
        }
        assert!(cursor.read().is_none());
        assert!(cursor.read().is_none());
    }

    // Step 4: 同步关闭定时器并等待其异步引用归还，唤醒并回收生产者任务，最后驱动后台打印服务追赶输出全部记录。
    timer.as_ref().close_sync();
    // SAFETY:
    // 1. 克隆 `Pin<Arc<Timer>>` 并调用 `into_inner_unchecked` 仅用于通过 `Arc::strong_count` 观察底层引用计数是否降回 3；
    // 2. 绝不移动或修改 `Timer` 本体，且在计数达标后立即释放 `observer`，不违反 `Pin` 固定内存的契约。
    let observer = unsafe { Pin::into_inner_unchecked(timer.clone()) };
    until(
        || Arc::strong_count(&observer) == 3,
        "timer running reference returned",
    );
    drop(observer);
    scenario().exit.complete();
    assert_eq!(worker.join().unwrap(), 125);
    let retained = scenario().producer.lock().take();
    drop((retained, timer));

    log::testing::catch_up(last + 1);
    log::testing::wait_service_blocked();
    let mut committed = Cursor::from_sequence(first);
    for index in 0..5 {
        let read = committed
            .read()
            .expect("committed record missing after printer catchup");
        let record = read.record;
        if index == 2 || index == 3 {
            crate::ktest::report::diagnostic(format_args!("execution-record record index={} seq={} level={} ns={} cpu={} task={} truncated={} body_hex={}", index, record.seq, record.level.as_str(), record.monotonic_ns, record.cpu, record.task.map_or(0, |id| id.as_usize()), u8::from(record.truncated), BodyHex(record.text().as_bytes())));
        } else {
            crate::ktest::report::diagnostic(format_args!("execution-record record index={} seq={} level={} ns={} cpu={} task={} truncated={}", index, record.seq, record.level.as_str(), record.monotonic_ns, record.cpu, record.task.map_or(0, |id| id.as_usize()), u8::from(record.truncated)));
        }
    }
}

/// 在后台日志打印任务因 UART 背压阻塞期间运行的计算工作任务，通过完成 4096 次迭代算术运算证明调度器仍可正常调度其他就绪任务。
fn business_worker() -> i32 {
    scenario().business_start.wait();
    let mut value = black_box(7usize);
    for index in 0..4096 {
        value = black_box(value.wrapping_mul(17).wrapping_add(index));
        scenario().business.fetch_add(1, Ordering::Release);
    }
    black_box(value);
    126
}

/// 循环向控制台写入换行符直至硬件 UART 发送 FIFO 满（`try_write` 返回 0），返回成功填入的字节总数。
fn fill_uart() -> usize {
    const LIMIT: usize = 1024 * 1024;
    let end = arch::read_counter()
        .checked_add(arch::counter_frequency().checked_mul(2).unwrap())
        .unwrap();
    let mut filled = 0;
    let mut yield_at = 256;
    let newlines = [b'\n'; 16];
    loop {
        assert!(arch::read_counter() < end, "UART fill deadline exceeded");
        assert!(
            filled < LIMIT,
            "UART fill limit reached without backpressure"
        );
        let bytes = &newlines[..(LIMIT - filled).min(newlines.len())];
        let accepted = console::try_write(bytes);
        assert!(accepted <= bytes.len());
        if accepted == 0 {
            assert!(0 < filled, "UART rejected the initial fill");
            return filled;
        }
        filled += accepted;
        if yield_at <= filled {
            yield_at += 256;
            sched::yield_now();
        }
    }
}

/// 使用栈上定长缓冲区格式化期望的编号日志正文 `"X26 record=<index>"`，并与实际读取的日志正文逐字节比对。
fn assert_numbered_body(message: &str, index: u64) {
    let mut expected = log::StackText::<32>::new();
    expected
        .write_fmt(format_args!("X26 record={}", index))
        .unwrap();
    assert_eq!(message.as_bytes(), expected.as_bytes());
}

/// 从打开的 `/kmsg` 设备读取一条格式化记录，校验其输出与 `<level>,<seq>,< timestamp_us>,-;<text>\n` 标准格式严格一致。
fn assert_kmsg_record(file: &FileRef, record: &log::Record) {
    let mut bytes = [0u8; log::TEXT_CAPACITY * 4 + 96];
    let length = file.read(&mut bytes).expect("kmsg read failed");
    let mut expected = log::StackText::<{ log::TEXT_CAPACITY * 4 + 96 }>::new();
    expected
        .write_fmt(format_args!(
            "6,{},{},-;{}\n",
            record.seq,
            record.monotonic_ns / 1000,
            record.text()
        ))
        .unwrap();
    assert_eq!(&bytes[..length], expected.as_bytes());
}

/// 测试硬件 UART 发送背压、后台打印任务部分写入阻塞、`/kmsg` 非阻塞读取与关中断延期 `poll` 通知，以及环形缓冲区覆盖后的游标追赶。
fn test_uart() -> ::ktest::TestResult {
    // Step 1: 以非阻塞模式打开 `/kmsg` 并寻道至末尾，初始化 `PollTable`，启动后台计算任务，并向宿主机发送串口暂停指令以制造硬件 UART 背压。
    let kmsg = super::open_observation(b"/kmsg", O_NONBLOCK).expect("kmsg open failed");
    assert_eq!(kmsg.seek(SeekFrom::End(0)), Ok(0));
    let notified = PollContext::try_new().expect("kmsg poll context failed");
    let mut table = PollTable::new(notified.clone(), 1).expect("kmsg poll table failed");
    let business = crate::ktest::spawn_result_on(0, |_| Ok(business_worker())).unwrap();
    let received = console::received();
    let owner = scenario();
    owner.h_before.store(received, Ordering::Release);
    owner.backpressure.store(true, Ordering::Release);
    let id = crate::ktest::active_case().ok_or(::ktest::TestError("backpressure outside case"))?;
    crate::ktest::report::write(format_args!("  # KTEST io {} pause 0 00000000 3000\n", id));
    until(
        || received < console::received() && console::last_byte() == b'H',
        "fresh UART RX IRQ hold acknowledgement",
    );

    // Step 2: 收到宿主机 `'H'` 握手字节后关闭 UART 接收中断，填满硬件 TX FIFO，随后提交长日志 `held` 并等待后台打印任务在部分写入后因 TX 等待而阻塞暂停。
    console::ktest_disable_receive_interrupts();
    owner.h_received.store(true, Ordering::Release);
    let before = console::testing::snapshot();
    let fill_bytes = fill_uart();
    let filled = console::testing::snapshot();
    assert_eq!(
        filled.accepted.checked_sub(before.accepted).unwrap(),
        fill_bytes
    );
    assert!(before.zero_writes < filled.zero_writes);

    let printer_before = log::testing::snapshot();
    log::testing::arm_pause(log::testing::PausePoint::AfterPartialWrite);
    let payload = [b'P'; 450];
    let text = core::str::from_utf8(&payload).unwrap();
    let held = crate::klog!(Level::Info, "X26 partial {}", text);
    {
        let saved = Cursor::from_sequence(held).read().unwrap();
        assert_eq!(saved.lost, 0);
        assert_eq!(saved.record.text().strip_prefix("X26 partial "), Some(text));
        scenario().held.lock().replace(saved.record);
    }
    until(
        || log::testing::is_paused() && log::testing::service_state() == Some(State::Blocked),
        "printer blocked after real partial TX wait",
    );
    let paused = log::testing::snapshot();
    assert_eq!(paused.seq, held);
    assert!(0 < paused.offset && paused.offset < paused.length);
    assert!(printer_before.tx_waits < paused.tx_waits);
    let partial = console::testing::snapshot();
    assert!(filled.zero_writes < partial.zero_writes);
    assert!(before.tx_interrupts < partial.tx_interrupts);
    assert!(before.wakes < partial.wakes);

    // Step 3: 验证后台打印任务阻塞期间，`/kmsg` 仍能立即读出 `held` 记录并在无新数据时返回 `WouldBlock`，同时将 `table` 注册到 `/kmsg` 等待队列。
    assert_eq!(
        kmsg.pollable().unwrap().poll(None).unwrap(),
        PollMask::POLLIN | PollMask::POLLRDNORM
    );
    let saved = Cursor::from_sequence(held).read().unwrap().record;
    assert_kmsg_record(&kmsg, &saved);
    assert_eq!(kmsg.read(&mut [0u8; 1]), Err(FsError::WouldBlock));
    assert_eq!(
        kmsg.pollable().unwrap().poll(Some(&mut table)).unwrap(),
        PollMask::default()
    );
    assert_eq!(table.len(), 1);
    notified.clear_triggered();

    // Step 4: 唤醒后台计算任务完成 4096 次运算；连续提交 72 条编号日志（首条在关中断下提交以验证延期 IPI 通知），用快游标 `fast` 和 `/kmsg` 逐条无丢失读取。
    let mut slow = Cursor::from_sequence(held + 1);
    let mut fast = Cursor::from_sequence(held + 1);
    scenario().business_start.complete();
    assert_eq!(business.join().unwrap(), 126);
    assert_eq!(scenario().business.load(Ordering::Acquire), 4096);
    for index in 0..72u64 {
        let sequence = if index == 0 {
            // 在本核关中断期间提交日志，验证日志通知 IPI 不会在关中断期间立即触发 `PollContext`，而是在恢复中断后延后送达。
            let irq = crate::sync::IrqSaveGuard::new();
            let sequence = crate::klog!(Level::Info, "X26 record={}", index);
            assert!(!notified.triggered());
            drop(irq);
            until(|| notified.triggered(), "deferred kmsg poll notification");
            sequence
        } else {
            crate::klog!(Level::Info, "X26 record={}", index)
        };
        assert_eq!(sequence, held + index + 1);
        assert_eq!(fast.available(), Some(0));
        assert_eq!(fast.available(), Some(0));
        let read = fast.read().unwrap();
        assert_eq!(read.record.seq, sequence);
        assert_eq!(read.lost, 0);
        assert_numbered_body(read.record.text(), index);
        assert_eq!(
            kmsg.pollable().unwrap().poll(None).unwrap(),
            PollMask::POLLIN | PollMask::POLLRDNORM
        );
        assert_kmsg_record(&kmsg, &read.record);
        assert_eq!(kmsg.read(&mut [0u8; 1]), Err(FsError::WouldBlock));
        assert_eq!(fast.available(), None);
        assert!(fast.read().is_none());
    }
    assert!(fast.read().is_none());

    // Step 5: 验证慢游标 `slow` 准确观测到前 8 条记录被环形缓冲区（容量 64）覆盖（`lost == 8`），并能顺序读出剩余的 64 条存活记录（序号 `held + 9..held + 73`）。
    assert_eq!(slow.available(), Some(8));
    assert_eq!(slow.available(), Some(8));
    let first = slow.read().unwrap();
    assert_eq!(first.lost, 8);
    assert_eq!(first.record.seq, held + 9);
    assert_numbered_body(first.record.text(), 8);
    for index in 1..64u64 {
        let read = slow.read().unwrap();
        assert_eq!(read.lost, 0);
        assert_eq!(read.record.seq, held + 9 + index);
        assert_numbered_body(read.record.text(), 8 + index);
    }
    assert!(slow.read().is_none());
    assert!(slow.read().is_none());
    let still = log::testing::snapshot();
    assert_eq!(
        (still.seq, still.offset, still.length),
        (paused.seq, paused.offset, paused.length)
    );
    assert_eq!(log::testing::service_state(), Some(State::Blocked));
    assert!(notified.triggered());
    assert_eq!(
        kmsg.pollable().unwrap().poll(None).unwrap(),
        PollMask::default()
    );

    // Step 6: 释放 `/kmsg` 句柄并解除后台打印任务暂停，等待其通过 UART TX 中断唤醒追赶输出剩余全部记录，最后与宿主机完成背压阶段握手确认并输出诊断统计。
    drop((table, notified, kmsg));
    log::testing::release_pause();
    let boundary = held + 73;
    log::testing::catch_up(boundary);
    log::testing::wait_service_blocked();
    let after = console::testing::snapshot();
    let complete = log::testing::snapshot();
    let zero_writes = after.zero_writes.checked_sub(before.zero_writes).unwrap();
    let tx_interrupts = after
        .tx_interrupts
        .checked_sub(before.tx_interrupts)
        .unwrap();
    let wakes = after.wakes.checked_sub(before.wakes).unwrap();
    let gaps = complete.gaps.checked_sub(printer_before.gaps).unwrap();
    assert!(0 < zero_writes && 0 < tx_interrupts && 0 < wakes);
    assert_eq!(gaps, 8);
    assert_eq!(complete.next_seq, boundary);
    assert_eq!(complete.progress - printer_before.progress, 65);
    assert!(printer_before.bytes < complete.bytes);
    assert_eq!(console::received(), received + 1);
    finish_backpressure()?;

    crate::ktest::report::diagnostic(format_args!(
        "execution-backpressure fill-bytes={}",
        fill_bytes
    ));
    {
        let saved = owner.held.lock();
        let record = saved
            .as_ref()
            .ok_or(::ktest::TestError("held record owner absent"))?;
        let metadata = (
            record.seq,
            record.level,
            record.monotonic_ns,
            record.cpu,
            record.task,
            record.truncated,
        );
        drop(saved);
        crate::ktest::report::diagnostic(format_args!("execution-backpressure record index=0 seq={} level={} ns={} cpu={} task={} truncated={} lost=0", metadata.0, metadata.1.as_str(), metadata.2, metadata.3, metadata.4.map_or(0, |id| id.as_usize()), u8::from(metadata.5)));
    }
    let mut surviving = Cursor::from_sequence(held + 9);
    for index in 1..=64 {
        let read = surviving
            .read()
            .ok_or(::ktest::TestError("surviving UART record missing"))?;
        assert_eq!(read.lost, 0);
        let record = read.record;
        assert_numbered_body(record.text(), index + 7);
        crate::ktest::report::diagnostic(format_args!("execution-backpressure record index={} seq={} level={} ns={} cpu={} task={} truncated={} lost={}", index, record.seq, record.level.as_str(), record.monotonic_ns, record.cpu, record.task.map_or(0, |id| id.as_usize()), u8::from(record.truncated), if index == 1 { gaps } else { 0 }));
    }
    assert!(surviving.read().is_none());
    crate::ktest::report::diagnostic(format_args!("UART observed held_seq={} offset={} length={} submitted=72 capacity=64 lost={} business={} zero_writes={} tx_interrupts={} wakes={} printer_gaps={} catchup={} progress={} rx_ack=1", held, paused.offset, paused.length, gaps, owner.business.load(Ordering::Acquire), zero_writes, tx_interrupts, wakes, gaps, boundary, complete.progress - printer_before.progress));
    Ok(())
}

/// 包含伪造的 KTAP 版本头、测试计划、完成标记与 Panic 字样的原始测试负载，用于验证串口透传模式不会被误解析为测试控制指令。
const RAW_LOOKALIKES: &[u8] = b"KTAP version 1\n1..1\nok 1 - payload\n  # KTEST complete\nKTEST FATAL panic\npanicked at fixture\n";

/// 串口原始字节流透传测试阶段的状态记录。
#[derive(Clone, Copy)]
struct RawStage {
    /// 传输方向：`true` 表示从宿主机接收（RX），`false` 表示向宿主机发送（TX）。
    rx: bool,
    /// 是否已收到宿主机的 `'H'` 握手确认字节。
    handshake: bool,
    /// 当前已成功接收或发送的 `RAW_LOOKALIKES` 字节偏移量。
    offset: usize,
}

/// 计算给定字节切片的 CRC32C（Castagnoli 多项式 `0x82f6_3b78`）校验和，供宿主机核对串口透传负载。
fn crc32c(bytes: &[u8]) -> u32 {
    let mut state = u32::MAX;
    for byte in bytes {
        state ^= u32::from(*byte);
        for _ in 0..8 {
            state = (state >> 1) ^ if state & 1 != 0 { 0x82f6_3b78 } else { 0 };
        }
    }
    !state
}

/// 在串口接收中断关闭的状态下，轮询等待硬件 UART 接收 FIFO 非空并读取单个控制字节。
fn control_byte() -> Result<u8, ::ktest::TestError> {
    super::wait(console::receive_pending, "UART control input timed out")?;
    Ok(crate::ktest::control_read_byte())
}

/// 按当前 `RawStage` 记录的方向与偏移量，通过硬件 UART 逐字节接收（并可选校验）或发送剩余的 `RAW_LOOKALIKES` 负载。
fn raw_payload(verify: bool) -> ::ktest::TestResult {
    let owner = scenario();
    let end = arch::read_counter() + arch::counter_frequency() * 15;
    loop {
        let stage = owner
            .raw
            .lock()
            .as_ref()
            .copied()
            .ok_or(::ktest::TestError("UART raw stage absent"))?;
        if RAW_LOOKALIKES.len() <= stage.offset {
            return Ok(());
        }
        if end <= arch::read_counter() {
            return Err(::ktest::TestError("UART raw payload timed out"));
        }
        if stage.rx {
            if !console::receive_pending() {
                sched::yield_now();
                continue;
            }
            let actual = crate::ktest::control_read_byte();
            owner.raw.lock().as_mut().unwrap().offset += 1;
            if verify && actual != RAW_LOOKALIKES[stage.offset] {
                return Err(::ktest::TestError("UART raw payload full-byte mismatch"));
            }
        } else {
            let count = console::try_write(&RAW_LOOKALIKES[stage.offset..]);
            if RAW_LOOKALIKES.len() - stage.offset < count {
                return Err(::ktest::TestError("UART accepted invalid raw prefix"));
            }
            owner.raw.lock().as_mut().unwrap().offset += count;
            if count == 0 {
                sched::yield_now();
            }
        }
    }
}

/// 完成 UART 背压测试阶段的收尾握手：确保已收到宿主机 `'H'` 字节并关闭 RX 中断，读取宿主机 `KTEST IO OK <id>` 确认行后恢复 RX 中断。
fn finish_backpressure() -> ::ktest::TestResult {
    let owner = scenario();
    if !owner.backpressure.load(Ordering::Acquire) {
        return Ok(());
    }
    if !owner.h_received.load(Ordering::Acquire) {
        super::wait(
            || {
                owner.h_before.load(Ordering::Acquire) < console::received()
                    && console::last_byte() == b'H'
            },
            "backpressure fresh H IRQ timed out",
        )?;
        console::ktest_disable_receive_interrupts();
        owner.h_received.store(true, Ordering::Release);
    }
    read_ack()?;
    owner.backpressure.store(false, Ordering::Release);
    console::enable_receive_interrupts();
    Ok(())
}

/// 从硬件 UART 轮询读取一行宿主机确认文本，校验其格式为 `"KTEST IO OK <active_case_id>"`。
fn read_ack() -> ::ktest::TestResult {
    let mut line = [0u8; 64];
    let mut length = 0;
    loop {
        let byte = control_byte()?;
        if byte == b'\n' {
            break;
        }
        if length == line.len() {
            return Err(::ktest::TestError("UART ACK exceeds bound"));
        }
        line[length] = byte;
        length += 1;
    }
    let digits = line[..length]
        .strip_prefix(b"KTEST IO OK ")
        .ok_or(::ktest::TestError("host rejected UART bytes"))?;
    if digits.is_empty() {
        return Err(::ktest::TestError("UART ACK has no case"));
    }
    let mut id = 0u32;
    for byte in digits {
        if !byte.is_ascii_digit() {
            return Err(::ktest::TestError("UART ACK case is malformed"));
        }
        id = id
            .checked_mul(10)
            .and_then(|id| id.checked_add(u32::from(*byte - b'0')))
            .ok_or(::ktest::TestError("UART ACK case overflow"))?;
    }
    if Some(id) != crate::ktest::active_case() {
        return Err(::ktest::TestError("UART ACK case mismatch"));
    }
    Ok(())
}

/// 完成串口原始字节流透传阶段的收尾清理：补齐未完成的握手与剩余负载传输，校验宿主机确认行，清空 `RawStage` 并恢复串口接收中断。
fn finish_raw() -> ::ktest::TestResult {
    let owner = scenario();
    let stage = owner
        .raw
        .lock()
        .as_ref()
        .copied()
        .ok_or(::ktest::TestError("UART raw stage absent"))?;
    console::disable_interrupts();
    if !stage.handshake {
        if control_byte()? != b'H' {
            return Err(::ktest::TestError("UART raw handshake mismatch"));
        }
        owner.raw.lock().as_mut().unwrap().handshake = true;
    }
    raw_payload(false)?;
    read_ack()?;
    owner.raw.lock().take();
    console::enable_receive_interrupts();
    Ok(())
}

/// 执行串口原始字节流透传测试：向宿主机发送 `KTEST io` 协议头并完成 `'H'` 握手后，在指定方向（`rx` 或 `tx`）上完整传输并校验 `RAW_LOOKALIKES` 伪控制文本。
fn test_raw_lookalikes(rx: bool) -> ::ktest::TestResult {
    console::disable_interrupts();
    let id = crate::ktest::active_case().ok_or(::ktest::TestError("UART raw outside case"))?;
    scenario().raw.lock().replace(RawStage {
        rx,
        handshake: false,
        offset: 0,
    });
    crate::ktest::report::write(format_args!(
        "  # KTEST io {} {} {} {:08x} 0\n",
        id,
        if rx { "rx" } else { "tx" },
        RAW_LOOKALIKES.len(),
        crc32c(RAW_LOOKALIKES)
    ));
    if control_byte()? != b'H' {
        return Err(::ktest::TestError("UART raw handshake mismatch"));
    }
    scenario().raw.lock().as_mut().unwrap().handshake = true;
    raw_payload(true)?;
    finish_raw()
}

/// 日志与串口测试场景的全局共享控制状态。
struct Scenario {
    /// 测试期间创建的定时器等异步节点管理器。
    nodes: super::Nodes,
    /// 通知生产者任务提前终止的原子停止标志。
    stop: AtomicBool,
    /// 普通日志生产者任务的句柄，由自旋锁保护。
    producer: SpinLock<Option<Arc<Task>>>,
    /// 协调者通知生产者任务开始写入日志的完成量。
    start: Completion,
    /// 协调者通知生产者任务可以退出栈帧的完成量。
    exit: Completion,
    /// 生产者任务已完成全部任务侧日志写入的标志。
    produced: AtomicBool,
    /// 高精度定时器硬中断回调写入的日志序号（0 表示尚未触发）。
    timer_seq: AtomicU64,
    /// 高精度定时器硬中断触发时所在 CPU 当前运行任务的 `TaskId` 数值。
    timer_task: AtomicUsize,
    /// 高精度定时器硬中断触发时所在的 CPU 核编号。
    timer_cpu: AtomicUsize,
    /// 协调者通知 `business_worker` 开始算术计算的完成量。
    business_start: Completion,
    /// `business_worker` 已完成的算术迭代次数计数。
    business: AtomicUsize,
    /// 当前正在进行的串口原始字节流透传阶段状态，由自旋锁保护。
    raw: SpinLock<Option<RawStage>>,
    /// 背压测试中触发后台打印任务部分写入阻塞的首条长日志副本，由自旋锁保护。
    held: SpinLock<Option<log::Record>>,
    /// 当前是否处于宿主机串口暂停背压测试阶段。
    backpressure: AtomicBool,
    /// 发送串口暂停命令前记录的控制台累计接收字节数，用于检测新到达的 `'H'` 握手字节。
    h_before: AtomicUsize,
    /// 背压测试阶段是否已收到宿主机的 `'H'` 握手字节并关闭了 RX 中断。
    h_received: AtomicBool,
}

/// 指向当前活动测试场景 `Scenario` 的全局原子裸指针，供无捕获参数的定时器回调与任务入口访问。
static CURRENT: core::sync::atomic::AtomicPtr<Scenario> =
    core::sync::atomic::AtomicPtr::new(core::ptr::null_mut());

/// 从全局 `CURRENT` 原子指针恢复当前测试场景的 `Arc<Scenario>` 强引用。
fn scenario() -> Arc<Scenario> {
    let pointer = CURRENT.load(Ordering::Acquire);
    assert!(!pointer.is_null(), "execution fixture absent");
    // SAFETY:
    // 1. `ServiceFixture::set_up` 通过 `Arc::into_raw` 将有效指针存入 `CURRENT`，且上方 `assert!(!pointer.is_null())` 确认了指针非空；
    // 2. `ServiceFixture::tear_down` 仅在所有定时器回调同步关闭且所有子任务 `join` 结束后才清空并释放该指针，因此调用期间底层 `Scenario` 始终有效；
    // 3. 先调用 `Arc::increment_strong_count` 增加强引用计数，再通过 `Arc::from_raw` 构造 `Arc`，保证返回的临时 `Arc` 释放时不会销毁 `CURRENT` 持有的引用。
    unsafe {
        Arc::increment_strong_count(pointer);
        Arc::from_raw(pointer)
    }
}

/// 日志与串口测试套件的夹具（Fixture），负责管理单次测试的 `Scenario` 状态与系统资源基线。
struct ServiceFixture {
    /// 当前测试用例的共享场景状态。
    state: Arc<Scenario>,
    /// 测试开始前记录的内存与工作队列元数据资源基线。
    initial: super::Baseline,
}

impl ::ktest::Fixture for ServiceFixture {
    /// 初始化测试夹具：将当前任务迁移至 CPU 0，记录系统资源基线，分配 `Scenario` 并将其原始指针注册到全局 `CURRENT` 中。
    fn set_up(_: &mut ::ktest::TestContext<'_>, _: &()) -> Result<Self, ::ktest::TestError> {
        sched::test_migrate_to(0);
        let initial = super::baseline();
        let state = Arc::try_new(Scenario {
            nodes: super::Nodes::new(1, 0, 0, 0)?,
            stop: AtomicBool::new(false),
            producer: SpinLock::new(None),
            start: Completion::new(),
            exit: Completion::new(),
            produced: AtomicBool::new(false),
            timer_seq: AtomicU64::new(0),
            timer_task: AtomicUsize::new(0),
            timer_cpu: AtomicUsize::new(0),
            business_start: Completion::new(),
            business: AtomicUsize::new(0),
            raw: SpinLock::new(None),
            held: SpinLock::new(None),
            backpressure: AtomicBool::new(false),
            h_before: AtomicUsize::new(0),
            h_received: AtomicBool::new(false),
        })
        .map_err(|_| ::ktest::TestError("NoMemory: execution state"))?;
        let pointer = Arc::into_raw(state.clone()).cast_mut();
        assert!(CURRENT
            .compare_exchange(
                core::ptr::null_mut(),
                pointer,
                Ordering::AcqRel,
                Ordering::Acquire
            )
            .is_ok());
        Ok(Self { state, initial })
    }

    /// 静默阶段（Quiesce）：解除打印服务暂停与生产者阻塞，同步关闭定时器节点，并完成未结束的串口透传或背压握手协议。
    fn quiesce(&mut self, _: &mut ::ktest::TestContext<'_>) -> ::ktest::TestResult {
        self.state.stop_sources();
        self.state.nodes.stop()?;
        let raw_active = self.state.raw.lock().is_some();
        if raw_active {
            finish_raw()?;
        }
        finish_backpressure()?;
        Ok(())
    }

    /// 拆卸阶段（Tear Down）：等待异步节点引用归还与后台打印服务追赶完毕，校验无残留等待者或未释放串口状态，注销 `CURRENT` 指针并核对系统资源回归初始基线。
    fn tear_down(self, _: &mut ::ktest::TestContext<'_>) -> ::ktest::TestResult {
        let mut owner = core::mem::ManuallyDrop::new(self);
        owner.state.nodes.tails()?;
        let retained = owner.state.producer.lock().take();
        drop(retained);
        log::testing::catch_up(log::next_sequence());
        log::testing::wait_service_blocked();
        if owner.state.start.test_waiting()
            + owner.state.exit.test_waiting()
            + owner.state.business_start.test_waiting()
            != 0
        {
            return Err(::ktest::TestError("log producer wait registrations remain"));
        }
        if owner.state.backpressure.load(Ordering::Acquire) || owner.state.raw.lock().is_some() {
            return Err(::ktest::TestError("UART ownership did not converge"));
        }

        let initial = owner.initial;
        let pointer = CURRENT.swap(core::ptr::null_mut(), Ordering::AcqRel);
        assert_eq!(pointer, Arc::as_ptr(&owner.state).cast_mut());
        // SAFETY:
        // 1. `pointer` 是由 `set_up` 中唯一一次 `Arc::into_raw` 存入 `CURRENT` 的裸指针，上方 `assert_eq!` 已验证其与 `owner.state` 地址一致；
        // 2. 此时所有定时器回调与子任务均已停止并回收，没有任何并发路径再通过 `scenario()` 访问该指针；
        // 3. `Arc::from_raw(pointer)` 回收 `CURRENT` 持有的强引用，随后 `ManuallyDrop::drop(&mut owner)` 释放夹具自身持有的最后一个强引用。
        unsafe {
            drop(Arc::from_raw(pointer));
            core::mem::ManuallyDrop::drop(&mut owner);
        }
        super::reclaimed(initial)
    }
}

impl ::ktest::Fixture<super::SuiteBaseline> for ServiceFixture {
    /// 使用套件级资源基线初始化测试夹具（委托给单元资源实现）。
    fn set_up(
        context: &mut ::ktest::TestContext<'_>,
        _: &super::SuiteBaseline,
    ) -> Result<Self, ::ktest::TestError> {
        <Self as ::ktest::Fixture<()>>::set_up(context, &())
    }

    /// 执行套件级测试夹具的静默阶段（委托给单元资源实现）。
    fn quiesce(&mut self, context: &mut ::ktest::TestContext<'_>) -> ::ktest::TestResult {
        <Self as ::ktest::Fixture<()>>::quiesce(self, context)
    }

    /// 执行套件级测试夹具的拆卸阶段（委托给单元资源实现）。
    fn tear_down(self, context: &mut ::ktest::TestContext<'_>) -> ::ktest::TestResult {
        <Self as ::ktest::Fixture<()>>::tear_down(self, context)
    }
}

impl Scenario {
    /// 设置全局停止标志，取消后台日志打印服务的暂停点，并唤醒所有尚在等待的生产者任务与计算任务使其安全退出。
    fn stop_sources(&self) {
        if self.stop.swap(true, Ordering::AcqRel) {
            return;
        }
        log::testing::cancel_pause();
        let producer = self.producer.lock().as_ref().map(Arc::clone);
        if producer
            .as_ref()
            .is_some_and(|task| sched::test_state(task) != State::Exited)
        {
            self.start.complete();
            self.exit.complete();
        }
        if self.business.load(Ordering::Acquire) < 4096 {
            self.business_start.complete();
        }
    }
}

impl ServiceFixture {
    /// 在固定绑定于 CPU 0 的协调者子任务中执行测试闭包 `action`，若异常中止则自动调用 `Scenario::stop_sources` 解除阻塞。
    fn fixed(
        &self,
        action: impl FnOnce() -> ::ktest::TestResult + Send + 'static,
    ) -> ::ktest::TestResult {
        super::on_zero(self.state.clone(), Scenario::stop_sources, action)
    }
}

#[::ktest::suite(fixture = ServiceFixture, suite_resources = crate::ktest::execution_services::SuiteBaseline, context = "Thread", tags("execution", "klog"), timeout_ms = 30000)]
pub(crate) mod records {
    use super::*;

    /// 测试目标：验证多上下文（普通任务、调度器锁内、工作队列锁内、高精度定时器硬中断）写入内核日志环时的零堆分配保证、多字节 UTF-8 字符边界截断以及多游标独立读取一致性。
    /// 场景步骤：
    /// 1. Given: 生产者任务与后台打印服务均处于阻塞静默状态，记录当前日志序号 `first` 与全局堆分配尝试次数 `attempts`；
    /// 2. When: 唤醒生产者依次提交普通日志、540 字节超长中文字符 `'界'` 日志、调度器锁内日志、工作队列池锁内日志，并在 5ms 后由高精度定时器硬中断提交第 5 条日志；
    /// 3. Then: 生产期间堆分配尝试增量为 0；两个独立 `Cursor` 均无丢失地顺序读出全部 5 条记录，其中超长中文日志精确截断在 510 字节（170 个完整 `'界'` 字符）且标记 `truncated = true`。
    #[test]
    fn allocation_free_record_identity(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(|| {
            test_records();
            Ok(())
        })
    }
}

#[::ktest::suite(fixture = ServiceFixture, context = "Thread", tags("execution", "uart", "klog"), resources("uart"), isolation = "Case", timeout_ms = 45000)]
pub(crate) mod uart {
    use super::*;

    /// 测试目标：验证慢速硬件 UART 造成发送 FIFO 背压时，后台日志打印任务的阻塞与恢复、`/kmsg` 非阻塞读取与关中断延期 `poll` 通知，以及环形缓冲区覆盖后的丢失计数与追赶行为。
    /// 场景步骤：
    /// 1. Given: 通知宿主机暂停读取串口并填满硬件 UART TX FIFO，提交包含 450 个 `'P'` 的日志 `held` 使后台打印任务在部分写入后因 TX 等待而阻塞暂停；
    /// 2. When: 通过 `/kmsg` 读取 `held` 并注册 `PollTable`；运行 `business_worker` 完成 4096 次计算；随后连续提交 72 条编号日志（首条在关中断下提交），使用快游标 `fast` 与 `/kmsg` 逐条读取，再使用慢游标 `slow` 读取并释放后台打印任务暂停；
    /// 3. Then: 关中断期间日志提交不会立即触发 `PollContext`，开中断后延后通知才送达；`fast` 与 `/kmsg` 完整读出 72 条记录；`slow` 与后台打印任务均检测到 8 条记录被容量为 64 的环形缓冲区覆盖（`lost == 8` / `gaps == 8`），并完整输出剩余 64 条记录。
    #[test]
    fn socket_printer_backpressure(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(test_uart)
    }

    /// 测试目标：验证串口原始字节流透传协议（`KTEST io`）在发送（`tx`）和接收（`rx`）两个方向上能够完整传输包含伪造 KTAP 控制行与 Panic 文本的数据，而不会被宿主机误判为测试框架控制指令。
    /// 场景步骤：
    /// 1. Given: 关闭串口中断，向宿主机输出 `# KTEST io <id> <rx|tx> <len> <crc32c> 0` 协议头并等待宿主机返回 `'H'` 握手字节；
    /// 2. When: 沿指定方向（`tx` 发送或 `rx` 接收并逐字节比对）通过硬件 UART 传输完整的 `RAW_LOOKALIKES` 字节流；
    /// 3. Then: 字节流传输无误，宿主机校验 CRC32C 通过后返回 `KTEST IO OK <id>` 确认行，串口恢复正常接收中断状态。
    #[test]
    #[case("tx", false)]
    #[case("rx", true)]
    fn raw_control_lookalikes(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
        rx: bool,
    ) -> ::ktest::TestResult {
        fixture.fixed(move || test_raw_lookalikes(rx))
    }
}

#[::ktest::suite(fixture = crate::ktest::execution_services::workqueue::ServiceFixture, context = "Thread", tags("execution", "fatal", "klog"), isolation = "Case", timeout_ms = 30000)]
pub(crate) mod fatal {
    use super::*;
    use crate::ktest::execution_services::workqueue::ServiceFixture as Pressure;

    /// 测试目标：验证在后台日志打印任务已阻塞静默的状态下，耗尽全部物理页与 SLUB 堆缓存后触发的标准内存分配失败能够直接进入全局 OOM 处理路径并正常终止。
    /// 场景步骤：
    /// 1. Given: 等待后台日志打印任务进入 `State::Blocked` 静默状态；
    /// 2. When: 收缩内存缓存并调用 `exhaust_fault()` 耗尽全部物理页（`alloc_pages` 返回 `None`）及 12 个级别的堆对象缓存，随后尝试分配 `Box::new([0u8; 4096])`；
    /// 3. Then: 分配进入全局 `alloc_error_handler` 并输出预期的 `"oom"` 致命错误，不会因日志打印任务重入分配或死锁而卡死。
    #[test]
    #[ktest(expected_fatal = "oom")]
    fn printer_real_oom(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut Pressure,
    ) -> ::ktest::TestResult {
        fixture.fixed(|| {
            log::testing::wait_service_blocked();
            assert_eq!(log::testing::service_state(), Some(State::Blocked));
            crate::ktest::report::diagnostic(format_args!(
                "printer_blocked=1 task={} cpu={}",
                task::current_task().id().as_usize(),
                cpu::current().id
            ));

            mm::shrink_all();
            let (pages, objects, heap_null) =
                crate::ktest::execution_services::workqueue::exhaust_fault();
            let page_null = usize::from(mm::alloc_pages(0, false).is_none());
            assert_eq!(page_null, 1);
            assert_eq!(heap_null, 12);
            assert_eq!(log::testing::service_state(), Some(State::Blocked));
            crate::console::emergency_write(format_args!(
                "printer OOM state pages={} objects={} page_null={} heap_null={}\n",
                pages, objects, page_null, heap_null
            ));
            let value = alloc::boxed::Box::new([black_box(0u8); 4096]);
            black_box(&value);
            panic!("standard allocation unexpectedly survived exhausted heap");
        })
    }

    /// 测试目标：验证当 Timekeeper 唯一写者在持有顺序锁（SeqLock）写锁期间发生故障（序列号处于奇数状态）时，致命错误处理路径不会死锁在单调时钟读取或后台日志打印等待上。
    /// 场景步骤：
    /// 1. Given: 等待后台日志打印任务进入 `State::Blocked` 静默状态；
    /// 2. When: 调用 `clock::testing::arm_writer_fault()` 注入写者故障，并调用 `clock::testing::maintain()` 触发写锁内的奇数序列号异常；
    /// 3. Then: 内核立即捕获并报告预期的 `"timekeeper"` 致命错误，不会在尝试读取单调时钟或等待打印任务时发生死锁。
    #[test]
    #[ktest(expected_fatal = "timekeeper")]
    fn writer_odd_sequence(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut Pressure,
    ) -> ::ktest::TestResult {
        fixture.fixed(|| {
            log::testing::wait_service_blocked();
            assert_eq!(log::testing::service_state(), Some(State::Blocked));
            crate::ktest::report::diagnostic(format_args!(
                "printer_blocked=1 task={} cpu={}",
                task::current_task().id().as_usize(),
                cpu::current().id
            ));
            clock::testing::arm_writer_fault();
            clock::testing::maintain();
            panic!("timekeeper writer fault did not fire");
        })
    }
}
