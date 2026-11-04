//! 真实记录、普通 printer Task 与 UART 中断的联合验收，不替换生产后端。

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

struct UnicodeBody;

impl fmt::Display for UnicodeBody {
    /// 固定输出超过正文容量的多字节文本，截断必须保留最后一个完整字符。
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        for _ in 0..180 {
            formatter.write_str("界")?;
        }
        Ok(())
    }
}

struct BodyHex<'a>(&'a [u8]);
impl fmt::Display for BodyHex<'_> {
    /// 只在生产区间结束后流式编码独立ring正文，不复制UART内容或申请格式化缓冲。
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        for byte in self.0 {
            write!(formatter, "{:02x}", byte)?;
        }
        Ok(())
    }
}

/// 测试协调具有固定原始 counter 截止，业务阻塞仍走真实 Completion 和 scheduler。
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

/// 独立普通生产者在预备完成后阻塞，记录完成仍保持存储直到协调者同步退出。
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

/// 精确硬回调只读标量、记录和发表完成，没有睡眠或对象构造。
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

/// 四个真实任务侧记录及一个精确 IRQ 记录共享同一环，生产阶段 GlobalAlloc 尝试必须为零。
fn test_records() {
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
                // 锁内标签不是业务合同；下方独立环正文供宿主逐字节核对真实UART。
                2 | 3 => {}
                _ => assert_eq!(record.text(), "X25 precise timer"),
            }
        }
        assert!(cursor.read().is_none());
        assert!(cursor.read().is_none());
    }
    timer.as_ref().close_sync();
    // SAFETY: 克隆只观察同一固定 Arc 的强计数，不移出 Timer；close_sync 已停止真实 callback 尾部。
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

/// printer 真阻塞期间另一个普通 Task 完成明确业务计算，不能用控制者忙等冒充进度。
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

/// 在固定 counter 截止及字节上界内填同一真实 UART，只有硬件首次拒收才算形成背压。
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

/// 用已有定长栈缓冲核对完整编号正文，等长旧正文也不能通过 cursor 校验。
fn assert_numbered_body(message: &str, index: u64) {
    let mut expected = log::StackText::<32>::new();
    expected
        .write_fmt(format_args!("X26 record={}", index))
        .unwrap();
    assert_eq!(message.as_bytes(), expected.as_bytes());
}

/// 真正打开的 kmsg 每次交付完整文本记录，期望元数据来自同一已提交环记录。
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

/// 慢 UART 导致真实 TX 等待；独立 kmsg 在同一停点读整记录并接受原 IPI 延期通知。
fn test_uart() -> ::ktest::TestResult {
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
    let mut slow = Cursor::from_sequence(held + 1);
    let mut fast = Cursor::from_sequence(held + 1);
    scenario().business_start.complete();
    assert_eq!(business.join().unwrap(), 126);
    assert_eq!(scenario().business.load(Ordering::Acquire), 4096);
    for index in 0..72u64 {
        let sequence = if index == 0 {
            // 本核 IRQ-save 阻止原日志 IPI 执行，记录提交不能直接通知 poll 源。
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

const RAW_LOOKALIKES: &[u8] = b"KTAP version 1\n1..1\nok 1 - payload\n  # KTEST complete\nKTEST FATAL panic\npanicked at fixture\n";

#[derive(Clone, Copy)]
struct RawStage {
    rx: bool,
    handshake: bool,
    offset: usize,
}

/// 同一个UART阶段逐字节摘要，CRC32C初值全一、最终取反，覆盖完整业务payload。
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

/// RX关闭后只读真实FIFO的一个控制字节；原始counter限制等待，不与IRQ RX争消费。
fn control_byte() -> Result<u8, ::ktest::TestError> {
    super::wait(console::receive_pending, "UART control input timed out")?;
    Ok(crate::ktest::control_read_byte())
}

/// 固定payload拥有仍在catch外；错字节先保存真实消费进度，清理只能消费尚未消费部分。
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

/// 一个具体opaque阶段只在同65真实行已经到达host以后取得ACK；不重新发送held或替代printer。
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

/// H已由该具体阶段接收且RX已经关闭，ACK沿同一个真实FIFO完整消费，错误不能伪清理成功。
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

/// ACK完全消费之前保持RX关闭，错误ACK保留阶段拥有并让现有case-abort终结VM。
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

/// 看似KTAP、complete与fatal的字节必须由现有raw阶段全文拥有；方向分别使用host的expected/payload文件。
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

struct Scenario {
    nodes: super::Nodes,
    stop: AtomicBool,
    producer: SpinLock<Option<Arc<Task>>>,
    start: Completion,
    exit: Completion,
    produced: AtomicBool,
    timer_seq: AtomicU64,
    timer_task: AtomicUsize,
    timer_cpu: AtomicUsize,
    business_start: Completion,
    business: AtomicUsize,
    raw: SpinLock<Option<RawStage>>,
    held: SpinLock<Option<log::Record>>,
    backpressure: AtomicBool,
    h_before: AtomicUsize,
    h_received: AtomicBool,
}
static CURRENT: core::sync::atomic::AtomicPtr<Scenario> =
    core::sync::atomic::AtomicPtr::new(core::ptr::null_mut());

/// 只克隆fixture已保存的真实into_raw拥有；所有回调关闭且child原join后才撤登记。
fn scenario() -> Arc<Scenario> {
    let pointer = CURRENT.load(Ordering::Acquire);
    assert!(!pointer.is_null(), "execution fixture absent");
    // SAFETY: 同VM顺序case，fixture在catch外持Arc与登记票；quiesce先停所有源，原scope join后才撤指针。
    unsafe {
        Arc::increment_strong_count(pointer);
        Arc::from_raw(pointer)
    }
}
struct ServiceFixture {
    state: Arc<Scenario>,
    initial: super::Baseline,
}
impl ::ktest::Fixture for ServiceFixture {
    /// 每个case从真实常驻基线重新准备独立计数、等待源及原节点owner表。
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
    /// 先合作停止真实生产者并释放停点/gate，再同步停止原节点；不在失败时收栈。
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
    /// runner已join每个原Task；尾票归还以后才释放context/owner并比较完整两账。
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
        // SAFETY: 此指针对应setup保存的唯一登记票；所有异步源停止、原scope已join且无回调借用。
        unsafe {
            drop(Arc::from_raw(pointer));
            core::mem::ManuallyDrop::drop(&mut owner);
        }
        super::reclaimed(initial)
    }
}

impl ::ktest::Fixture<super::SuiteBaseline> for ServiceFixture {
    /// 普通日志suite复用同一具体case拥有；UART的独立Case仍使用Unit资源。
    fn set_up(
        context: &mut ::ktest::TestContext<'_>,
        _: &super::SuiteBaseline,
    ) -> Result<Self, ::ktest::TestError> {
        <Self as ::ktest::Fixture<()>>::set_up(context, &())
    }
    /// suite账务不改变原printer、UART及Timer的具体停止次序。
    fn quiesce(&mut self, context: &mut ::ktest::TestContext<'_>) -> ::ktest::TestResult {
        <Self as ::ktest::Fixture<()>>::quiesce(self, context)
    }
    /// case原拥有全部归还后，suite资源另承担跨例的最终账务。
    fn tear_down(self, context: &mut ::ktest::TestContext<'_>) -> ::ktest::TestResult {
        <Self as ::ktest::Fixture<()>>::tear_down(self, context)
    }
}

impl Scenario {
    /// 当前动作已退栈后一次释放真实printer停点与业务gate，不关闭端口TX或释放任何原owner。
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
    /// 真实fixed-0协调Task独立持有Scenario，UART/日志动作不借Any父的fixture或context栈。
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
    /// 五条真实Task/调度锁/pool锁/精确IRQ记录同环发表、零allocation_attempts并保持UTF8截断与独立游标。
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
    /// 同一 UART 真背压时 kmsg 独立交付全文与延期通知，保留原450P、lost及4096次业务进度。
    #[test]
    fn socket_printer_backpressure(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(test_uart)
    }
    /// 两方向都以真实UART硬件消费完整伪控制文本，不允许交给通用KTAP parser。
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
    /// 真printer已经Blocked以后摘尽原页和十二heap类，标准Box4096进入原alloc_error_handler。
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
    /// 唯一生产writer在写锁内真实奇序号故障，不能展开或等待单调clock/printer。
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
