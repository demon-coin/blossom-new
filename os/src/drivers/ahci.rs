//! 单端口、非 NCQ AHCI；原块层拥有请求、八个软件槽及可睡 progress Work。

mod ata;
mod registers;

use alloc::sync::Arc;
use core::mem::ManuallyDrop;
use core::pin::{pin, Pin};
use core::ptr;
use core::sync::atomic::{AtomicU32, Ordering};

use crate::arch;
use crate::block::{BlkOperation, BlockError, IoResult, QueueLimits};
use crate::boot::AhciResources;
use crate::irq::{self, IrqNumber};
use crate::mm::dma::{CoherentDma, DmaDirection, DmaDomain, PayloadRange};
use crate::sync::{SleepMutex, SpinLock};
use crate::time::{self, Duration, MonotonicInstant};
use crate::timer::{self, Timer, TimerKind};
use crate::wait::{WaitNode, WaitQueue, Waiter, Waker};
use registers::{Host, Port};
use registers::{
    CI, CLB, CLBU, CMD, COMPLETION, CR, ERROR, FB, FBU, FR, FRE, IE, INTERRUPTS, PORT_IS, SACT, ST,
    TASK_BUSY, TASK_ERROR, TFD,
};

const COMMAND_TIMEOUT: Duration = Duration::from_secs(5);
const STOP_TIMEOUT: Duration = Duration::from_secs(2);
/// 完整 ABAR 包含全局窗口和规范允许的全部端口窗口。
pub(crate) const MMIO_BYTES: usize =
    registers::PORT_BASE + registers::MAX_PORTS as usize * registers::PORT_BYTES;
const COMMAND_SLOTS: usize = 32;
const COMMAND_HEADER_BYTES: usize = 32;
const COMMAND_LIST_BYTES: usize = COMMAND_SLOTS * COMMAND_HEADER_BYTES;
const RECEIVED_FIS_BYTES: usize = 256;
const COMMAND_TABLE_HEADER_BYTES: usize = 128;
const PRDT_ENTRY_BYTES: usize = 16;
const COMMAND_WRITE_SHIFT: u32 = 6;
const PRDT_LENGTH_SHIFT: u32 = 16;
const PRDBC_OFFSET: usize = core::mem::size_of::<u32>();
const ACTIVE_COMMAND: u32 = 1;
const DMA_CACHE_LINE_BYTES: usize = 64;

#[derive(Clone, Copy, Eq, PartialEq)]
enum StopPhase {
    Stopped,
    Running,
    Command,
    Fis,
}

struct State {
    phase: StopPhase,
    published: bool,
    accepting: bool,
    interrupts: bool,
    issued: bool,
}

/// 四个独立整页块避免非一致 cacheline 混合 CPU 与设备拥有。
struct Controls {
    list: CoherentDma,
    fis: CoherentDma,
    table: CoherentDma,
    identify: CoherentDma,
}

impl Controls {
    /// 所有可失败控制存储都在任何地址发表之前从唯一原页体系取得。
    fn allocate(domain: DmaDomain) -> Result<Self, BlockError> {
        let controls = Self {
            list: CoherentDma::allocate(domain, COMMAND_LIST_BYTES)?,
            fis: CoherentDma::allocate(domain, RECEIVED_FIS_BYTES)?,
            table: CoherentDma::allocate(
                domain,
                COMMAND_TABLE_HEADER_BYTES + ata::MAX_PRDT * PRDT_ENTRY_BYTES,
            )?,
            identify: CoherentDma::allocate(domain, ata::IDENTIFY_BYTES)?,
        };
        for (owner, alignment) in [
            (&controls.list, COMMAND_LIST_BYTES as u64),
            (&controls.fis, RECEIVED_FIS_BYTES as u64),
            (&controls.table, COMMAND_TABLE_HEADER_BYTES as u64),
            (&controls.identify, core::mem::align_of::<u16>() as u64),
        ] {
            if owner.region().device.value() % alignment != 0 {
                return Err(BlockError::Misaligned);
            }
        }
        Ok(controls)
    }

    /// 准备期独占或运行期可睡锁保护且端口停止时填真实 PRDT，设备地址保留完整高字。
    fn prdt(&self, index: usize, address: u64, bytes: usize) -> Result<(), BlockError> {
        if ata::MAX_PRDT <= index || bytes == 0 || ata::PRDT_BYTES < bytes {
            return Err(BlockError::InvalidRange);
        }
        if address & 1 != 0 || bytes & 1 != 0 {
            return Err(BlockError::Misaligned);
        }
        let words = [
            address as u32,
            (address >> 32) as u32,
            0,
            (bytes - 1) as u32,
        ];
        // SAFETY: 准备期未发表或调用者持可睡控制锁，CR/FR 已停；index 已核，table owner 保活整块。
        unsafe {
            let destination = self
                .table
                .region()
                .cpu
                .as_ptr()
                .add(COMMAND_TABLE_HEADER_BYTES + index * PRDT_ENTRY_BYTES)
                .cast::<u32>();
            for (offset, word) in words.into_iter().enumerate() {
                ptr::write_volatile(destination.add(offset), word.to_le());
            }
        }
        Ok(())
    }

    /// 已收权的固定控制页接受新命令，PRDT 数量只覆盖本次实际设备段。
    fn command(
        &self,
        fis: &[u8; ata::H2D_FIS_BYTES],
        write: bool,
        entries: usize,
    ) -> Result<(), BlockError> {
        if ata::MAX_PRDT < entries {
            return Err(BlockError::InvalidRange);
        }
        let table = self.table.region().device.value();
        let header = [
            (ata::H2D_FIS_BYTES / core::mem::size_of::<u32>()) as u32
                | (u32::from(write) << COMMAND_WRITE_SHIFT)
                | ((entries as u32) << PRDT_LENGTH_SHIFT),
            0,
            table as u32,
            (table >> 32) as u32,
        ];
        // SAFETY: 准备期独占或 execute 持可睡锁且引擎已停；三个完整控制 owner 保活。
        // 设备尚无本轮访问权，CPU 不建立活过交权的切片。
        unsafe {
            ptr::copy_nonoverlapping(fis.as_ptr(), self.table.region().cpu.as_ptr(), fis.len());
            ptr::write_bytes(self.fis.region().cpu.as_ptr(), 0, RECEIVED_FIS_BYTES);
            let destination = self.list.region().cpu.as_ptr().cast::<u32>();
            for (offset, word) in header.into_iter().enumerate() {
                ptr::write_volatile(destination.add(offset), word.to_le());
            }
            self.table
                .sync_for_device(0, self.table.byte_len(), DmaDirection::ToDevice)?;
            self.list
                .sync_for_device(0, self.list.byte_len(), DmaDirection::Bidirectional)?;
            self.fis
                .sync_for_device(0, self.fis.byte_len(), DmaDirection::FromDevice)?;
        }
        Ok(())
    }

    /// 在 CR/FR 均确认归零后收取 PRDBC；错误命令的已传输字节不等于成功写入前缀。
    fn transferred(&self) -> Result<usize, BlockError> {
        // SAFETY: 仅在 stop 完整确认后调用；原 owner 存活且期间没有 CPU 写入设备控制范围。
        unsafe {
            self.list
                .sync_for_cpu(0, self.list.byte_len(), DmaDirection::Bidirectional)?;
            self.fis
                .sync_for_cpu(0, self.fis.byte_len(), DmaDirection::FromDevice)?;
            Ok(u32::from_le(ptr::read_volatile(
                self.list
                    .region()
                    .cpu
                    .as_ptr()
                    .add(PRDBC_OFFSET)
                    .cast::<u32>(),
            )) as usize)
        }
    }
}

/// IRQ 只访问短锁、寄存器与 mailbox；控制页的 CPU 访问由可睡锁独占。
/// Root 必须强持本 Arc 到请求、IRQ、Work 全尾且 reset_complete 确认停止。
pub(crate) struct Ahci {
    port: Port,
    interrupt: IrqNumber,
    domain: DmaDomain,
    identify: Option<ata::Identify>,
    controls: ManuallyDrop<Controls>,
    control: SleepMutex<()>,
    state: SpinLock<State>,
    mailbox: AtomicU32,
    waiters: WaitQueue,
}

impl Ahci {
    /// 消费真实 ABAR/IRQ 并识别介质；只有端口扫描确认无介质才返回 None，硬件错误不作缺盘。
    /// 选端口后先建立 FIS 接收并做一次端口复位再识别。
    ///
    /// # Safety
    /// 资源来自唯一库存，PCI BAR 已解析；ABAR 完整窗口永久强序映射且独占，固件已不再使用。
    /// 资源的 DMA 总线为已规范化的恒等物理窗口；coherency 和额外平台 mask 必须符合真实设备。
    /// 返回 Arc 的拥有者须先保活它再注册 IRQ，直到真实停止及全部外部访问尾部。
    pub(crate) unsafe fn prepare(
        resources: &AhciResources,
    ) -> Result<Option<Arc<Self>>, BlockError> {
        crate::sched::assert_switchable();
        let interrupt =
            irq::map_source(resources.interrupt_source).map_err(|_| BlockError::InvalidRange)?;
        // None 表示平台没有额外收紧；实际硬件上限始终由 CAP.S64A 决定。
        let device_mask = match resources.dma_mask {
            Some(mask) => mask,
            None => u64::MAX,
        };
        if device_mask == 0 || device_mask & device_mask.wrapping_add(1) != 0 {
            return Err(BlockError::InvalidRange);
        }
        // SAFETY: 调用者保证本次资源为实际 ABAR，完整强序窗口和独占硬件的寿命覆盖控制器。
        let host = unsafe { Host::new(resources.registers) }?;
        let (cap, ports) = host.initialize(resources.ports_implemented)?;
        let mask = device_mask.min(if cap & registers::CAP_64BIT != 0 {
            u64::MAX
        } else {
            u32::MAX as u64
        });
        let domain = DmaDomain::new(
            mask,
            0,
            usize::MAX,
            0,
            DMA_CACHE_LINE_BYTES,
            resources.dma_coherent,
        )?;
        let mut selected = None;
        for index in 0..registers::MAX_PORTS {
            if ports & (1 << index) == 0 {
                continue;
            }
            let port = host.port(index)?;
            port.power_up(cap);
            if registers::poll(Duration::from_secs(1), || port.connected()) {
                selected = Some(port);
                break;
            } else if !registers::media_absent(port.read(registers::SSTS)) {
                return Err(BlockError::NotReady);
            }
        }
        let Some(port) = selected else {
            // power_up 只设置链路电源位；仍逐端口确认 CR/FR 停止后才能报告无介质。
            // 此时尚未分配或发表任何本驱动的控制和 IDENTIFY 页。
            for index in 0..registers::MAX_PORTS {
                if ports & (1 << index) == 0 {
                    continue;
                }
                let port = host.port(index)?;
                if !port.stop_polled() {
                    return Err(BlockError::StopUnconfirmed);
                }
                port.clear_pending();
            }
            return Ok(None);
        };
        let controls = Controls::allocate(domain)?;
        port.start_fis_rx(
            controls.list.region().device.value(),
            controls.fis.region().device.value(),
        );
        if let Err(error) = port.comreset() {
            if !port.stop_polled() {
                core::mem::forget(controls);
                panic!("AHCI comreset stop unconfirmed; control pages retained");
            }
            crate::early_println!(
                "AHCI probe failed stage=comreset cmd={:#x} ssts={:#x} sig={:#x} tfd={:#x}",
                port.read(CMD),
                port.read(registers::SSTS),
                port.read(registers::SIG),
                port.read(TFD),
            );
            return Err(error);
        }
        if !registers::poll(Duration::from_secs(1), || port.taskfile_ready()) {
            if !port.stop_polled() {
                core::mem::forget(controls);
                panic!("AHCI taskfile stop unconfirmed; control pages retained");
            }
            crate::early_println!(
                "AHCI probe failed stage=taskfile cmd={:#x} ssts={:#x} sig={:#x} tfd={:#x}",
                port.read(CMD),
                port.read(registers::SSTS),
                port.read(registers::SIG),
                port.read(TFD),
            );
            return Err(BlockError::NotReady);
        }
        if port.read(registers::SIG) != registers::ATA_SIGNATURE {
            if !port.stop_polled() {
                core::mem::forget(controls);
                panic!("AHCI signature stop unconfirmed; control pages retained");
            }
            crate::early_println!(
                "AHCI probe failed stage=signature cmd={:#x} ssts={:#x} sig={:#x} tfd={:#x}",
                port.read(CMD),
                port.read(registers::SSTS),
                port.read(registers::SIG),
                port.read(TFD),
            );
            return Err(BlockError::Unsupported);
        }
        // 构造 Arc 前收回设备访问权，保持识别命令准备期间控制页由 CPU 独占。
        // Arc 分配失败或命令准备提前返回时，控制页也只能在确认停止后释放。
        if !port.stop_polled() {
            core::mem::forget(controls);
            panic!("AHCI probe stop unconfirmed; control pages retained");
        }
        let mut controller = Arc::try_new(Self {
            port,
            interrupt,
            domain,
            identify: None,
            controls: ManuallyDrop::new(controls),
            control: SleepMutex::new(()),
            state: SpinLock::new(State {
                phase: StopPhase::Stopped,
                published: false,
                accepting: false,
                interrupts: false,
                issued: false,
            }),
            mailbox: AtomicU32::new(0),
            waiters: WaitQueue::new(),
        })
        .map_err(|_| BlockError::NoMemory)?;
        match controller.identify_polled() {
            Ok(identify) => {
                Arc::get_mut(&mut controller)
                    .expect("unpublished AHCI Arc escaped")
                    .identify = Some(identify);
                Ok(Some(controller))
            }
            Err(BlockError::StopUnconfirmed) => {
                // fail-fast: 已发表的控制/IDENTIFY 页仍可能被访问，强引用不能经 Err 展开析构。
                let retained = Arc::into_raw(controller);
                panic!(
                    "AHCI identify stop unconfirmed; owner retained at {:p}",
                    retained
                );
            }
            Err(error) => Err(error),
        }
    }

    /// 返回 IDENTIFY 得到的正常扇区、容量及独立 FLUSH 能力，不重新读硬件。
    pub(crate) fn limits(&self) -> QueueLimits {
        self.identify.expect("AHCI identify missing").limits()
    }

    /// 原 payload 的设备映射和同步必须使用本控制器核对后的同一地址域。
    pub(crate) fn domain(&self) -> DmaDomain {
        self.domain
    }

    /// 返回唯一 IRQ 库存已经验证的外部中断身份。
    pub(crate) fn interrupt(&self) -> IrqNumber {
        self.interrupt
    }

    /// Core 已保活 Arc 并准备 IRQ action 后发表设备；第一条命令才启动 DMA 引擎。
    pub(crate) fn publish(&self) -> Result<(), BlockError> {
        let mut state = self.state.lock();
        if state.published || state.phase != StopPhase::Stopped || !self.port.connected() {
            return Err(BlockError::NotReady);
        }
        self.port.clear_pending();
        self.mailbox.store(0, Ordering::Release);
        state.published = true;
        state.accepting = true;
        self.port.enable_host_irq();
        Ok(())
    }

    /// 共享 IRQ 只认领本端口实际原因，W1C 后记录 mailbox 并唤醒，不读取 payload。
    pub(crate) fn acknowledge(&self) -> bool {
        let claimed = {
            let _state = self.state.lock();
            let status = self.port.read(PORT_IS);
            if status == 0 {
                false
            } else {
                self.port.write(PORT_IS, status);
                self.port.ack_host();
                self.mailbox.fetch_or(status, Ordering::AcqRel);
                true
            }
        };
        if claimed {
            self.waiters.wake_all();
        }
        claimed
    }

    /// 只改变本设备的中断屏蔽；控制器命令执行期间 Core 必须保留中断使能。
    pub(crate) fn set_interrupts(&self, enabled: bool) {
        let mut state = self.state.lock();
        state.interrupts = enabled;
        self.port.write(
            IE,
            if enabled && state.accepting && state.phase == StopPhase::Running {
                INTERRUPTS
            } else {
                0
            },
        );
        let _ = self.port.read(IE);
    }

    /// 登记源后开放中断并复查一次早到状态，不制造 used 或周期完成轮询。
    pub(crate) fn arm_and_check(&self) -> bool {
        self.set_interrupts(true);
        self.acknowledge();
        self.mailbox.load(Ordering::Acquire) != 0
    }

    /// 短队列锁内禁止新 doorbell 并请求清 ST；不睡眠，不等待 CR 或 FR。
    pub(crate) fn request_reset(&self) {
        {
            let mut state = self.state.lock();
            state.accepting = false;
            state.interrupts = false;
            self.begin_stop(&mut state);
        }
        self.waiters.wake_all();
    }

    /// 每次只推进有限 MMIO 阶段；CR=0 之后才清 FRE，FR=0 才允许归还原页。
    pub(crate) fn reset_complete(&self) -> bool {
        let mut state = self.state.lock();
        match state.phase {
            StopPhase::Running => false,
            StopPhase::Command => {
                if self.port.read(CMD) & CR != 0 {
                    return false;
                }
                self.port.write(CMD, self.port.read(CMD) & !FRE);
                let _ = self.port.read(CMD);
                state.phase = StopPhase::Fis;
                false
            }
            StopPhase::Fis => {
                if self.port.read(CMD) & FR != 0 {
                    return false;
                }
                arch::dma_read_barrier();
                state.phase = StopPhase::Stopped;
                state.issued = false;
                true
            }
            StopPhase::Stopped => true,
        }
    }

    /// 原 progress Work 锁外提交一个直接 payload 命令，并沿原 Waiter 等待真实 IRQ。
    ///
    /// # Safety
    /// ranges 必须来自 Core 仍完整拥有的 accepted payload，并已按 domain() 交权。
    /// 交权期间无冲突 CPU 访问；返回 StopUnconfirmed 时 Core 必须继续保留全部 payload、槽和本 Arc，
    /// 不完成或归页，直到后续真实停止。其它返回均已确认硬件结束这些范围的访问。
    pub(crate) unsafe fn execute(
        &self,
        operation: BlkOperation,
        sector: u64,
        bytes: usize,
        ranges: &[PayloadRange],
    ) -> IoResult {
        let _control = self.control.lock();
        if let Err(error) = self.prepare_transfer(operation, sector, bytes, ranges) {
            return failed(error);
        }
        {
            let mut state = self.state.lock();
            if !state.published || !state.accepting {
                return failed(BlockError::QueueStopped);
            }
            if !state.interrupts || state.phase != StopPhase::Stopped {
                return failed(BlockError::NotReady);
            }
            if self.port.read(TFD) & TASK_BUSY != 0
                || self.port.read(CI) != 0
                || self.port.read(SACT) != 0
            {
                return failed(BlockError::DeviceFault);
            }
            self.issue(&mut state, true);
        }
        let deadline = time::now()
            .checked_add(COMMAND_TIMEOUT)
            .expect("AHCI command deadline overflow");
        let completion = self.wait_irq(deadline);
        if completion.is_err() {
            self.request_reset();
        } else {
            let mut state = self.state.lock();
            self.begin_stop(&mut state);
        }
        if !self.stop_bounded(true) {
            self.request_reset();
            return failed(BlockError::StopUnconfirmed);
        }
        let transferred = match self.controls.transferred() {
            Ok(transferred) => transferred,
            Err(error) => {
                self.request_reset();
                return failed(error);
            }
        };
        if let Err(error) = completion {
            // ATA 错误或超时不证明任何写入成功前缀，绝不重放未知写。
            return failed(error);
        }
        if self.port.read(TFD) & (TASK_ERROR | TASK_BUSY) != 0
            || self.port.read(PORT_IS) & ERROR != 0
            || self.mailbox.load(Ordering::Acquire) & ERROR != 0
        {
            self.request_reset();
            return failed(BlockError::Io);
        }
        if bytes < transferred {
            self.request_reset();
            return failed(BlockError::DeviceFault);
        }
        if transferred != bytes {
            self.request_reset();
        }
        IoResult {
            completed: transferred,
            error: if transferred == bytes {
                None
            } else {
                Some(BlockError::ShortIo)
            },
        }
    }

    /// 校验原块限额后直接规范化真实设备段，不拷贝 payload 或分配 SG 容器。
    fn prepare_transfer(
        &self,
        operation: BlkOperation,
        sector: u64,
        bytes: usize,
        ranges: &[PayloadRange],
    ) -> Result<(), BlockError> {
        {
            let state = self.state.lock();
            if !state.accepting || state.phase != StopPhase::Stopped {
                return Err(BlockError::QueueStopped);
            }
        }
        let identify = self.identify.expect("AHCI identify missing");
        identify
            .limits()
            .validate(operation, sector, bytes, ranges.iter().copied(), 0)?;
        if ata::MAX_BYTES < bytes || ata::MAX_PRDT < ranges.len() {
            return Err(BlockError::InvalidRange);
        }
        let command = identify.command(operation, sector, bytes)?;
        let mut entries = 0;
        for range in ranges {
            let mut address = self
                .domain
                .map(range.physical, range.length as usize)?
                .value();
            let mut remaining = range.length as usize;
            while remaining != 0 {
                let length = remaining.min(ata::PRDT_BYTES);
                self.controls.prdt(entries, address, length)?;
                address = address
                    .checked_add(length as u64)
                    .ok_or(BlockError::InvalidRange)?;
                remaining -= length;
                entries += 1;
            }
        }
        self.controls
            .command(&command, operation == BlkOperation::Write, entries)
    }

    /// 在独占短锁内重装地址、开启 FRE/ST，最后只写新命令的 CI 位。
    fn issue(&self, state: &mut State, interrupts: bool) {
        self.port.clear_pending();
        self.mailbox.store(0, Ordering::Release);
        let list = self.controls.list.region().device.value();
        let fis = self.controls.fis.region().device.value();
        self.port.write(CLBU, (list >> 32) as u32);
        self.port.write(CLB, list as u32);
        self.port.write(FBU, (fis >> 32) as u32);
        self.port.write(FB, fis as u32);
        self.port.write(CMD, self.port.read(CMD) | FRE);
        let _ = self.port.read(CMD);
        self.port.write(CMD, self.port.read(CMD) | ST);
        let _ = self.port.read(CMD);
        state.phase = StopPhase::Running;
        state.issued = true;
        self.port.write(IE, if interrupts { INTERRUPTS } else { 0 });
        arch::dma_write_barrier();
        self.port.write(CI, ACTIVE_COMMAND);
        let _ = self.port.read(CI);
    }

    /// 命令是否完成只在实际中断原因存在时检查 CI；单独 CI 清零不能补救丢失的 IRQ。
    fn completion(&self) -> Option<Result<(), BlockError>> {
        let state = self.state.lock();
        if !state.accepting || state.phase != StopPhase::Running {
            return Some(Err(BlockError::QueueStopped));
        }
        let status = self.mailbox.load(Ordering::Acquire);
        if status & ERROR != 0 {
            return Some(Err(BlockError::Io));
        }
        if state.issued && status & COMPLETION != 0 && self.port.read(CI) & ACTIVE_COMMAND == 0 {
            return Some(if self.port.read(TFD) & TASK_ERROR == 0 {
                Ok(())
            } else {
                Err(BlockError::Io)
            });
        }
        None
    }

    /// 每轮提前通知只重挂原绝对期限；定时器仅产生超时，不读取寄存器或补发完成。
    fn wait_irq(&self, deadline: MonotonicInstant) -> Result<(), BlockError> {
        loop {
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY: self 保活源，栈节点固定；局部 registration 在节点和 waiter 退出前真实撤链。
            let registration = unsafe { self.waiters.register(node.as_mut()) };
            let waker = pin!(waiter.waker());
            let timeout = pin!(Timer::new(
                crate::cpu::current().id,
                TimerKind::Precise,
                wake_timeout,
                waker.as_ref().get_ref() as *const Waker as usize,
            ));
            // SAFETY: timeout/Waker 栈地址固定，退出本轮前同步取消等待真实回调尾部。
            unsafe { timeout.as_ref().arm_borrowed(deadline) }.expect("AHCI timeout rejected");
            self.acknowledge();
            let mut result = self.completion();
            if result.is_none() && time::now() < deadline {
                waiter.park();
                // 到期只报告故障，不以期限唤醒后的寄存器探测制造成功完成。
                result = if deadline <= time::now() {
                    Some(Err(BlockError::Io))
                } else {
                    self.completion()
                };
            } else if result.is_none() {
                result = Some(Err(BlockError::Io));
            }
            drop(registration);
            drop(waiter);
            timeout.as_ref().cancel_sync();
            if let Some(result) = result {
                return result;
            }
        }
    }

    /// 与 request_reset 共用停止第一步；锁内只清 ST，不等待设备推进。
    fn begin_stop(&self, state: &mut State) {
        self.port.write(IE, 0);
        if state.phase == StopPhase::Running {
            self.port.write(CMD, self.port.read(CMD) & !ST);
            let _ = self.port.read(CMD);
            state.phase = StopPhase::Command;
        }
    }

    /// 可睡执行者在队列锁外等待停止阶段；准备期使用有限忙等，运行期沿原 timer 睡眠。
    fn stop_bounded(&self, sleep: bool) -> bool {
        let deadline = time::now()
            .checked_add(STOP_TIMEOUT)
            .expect("AHCI stop deadline overflow");
        loop {
            if self.reset_complete() {
                return true;
            }
            if deadline <= time::now() {
                return false;
            }
            if sleep {
                timer::sleep(Duration::from_millis(1));
            } else {
                core::hint::spin_loop();
            }
        }
    }

    /// 稳定 Arc 和全部 owner 准备完成后才能发表 IDENTIFY 的控制与数据地址。
    fn identify_polled(&self) -> Result<ata::Identify, BlockError> {
        self.controls.prdt(
            0,
            self.controls.identify.region().device.value(),
            ata::IDENTIFY_BYTES,
        )?;
        // SAFETY: IDENTIFY owner 尚无设备访问，完整零页独占 cacheline，Arc 已稳定保活全部存储。
        unsafe {
            self.controls.identify.sync_for_device(
                0,
                self.controls.identify.byte_len(),
                DmaDirection::FromDevice,
            )
        }?;
        self.controls.command(&ata::fis(ata::IDENTIFY), false, 1)?;
        {
            let mut state = self.state.lock();
            self.issue(&mut state, false);
        }
        let completed = registers::poll(COMMAND_TIMEOUT, || {
            self.port.read(PORT_IS) & ERROR != 0 || self.port.read(CI) & ACTIVE_COMMAND == 0
        });
        let failed_command = !completed
            || self.port.read(PORT_IS) & ERROR != 0
            || self.port.read(TFD) & TASK_ERROR != 0;
        {
            let mut state = self.state.lock();
            self.begin_stop(&mut state);
        }
        if !self.stop_bounded(false) {
            return Err(BlockError::StopUnconfirmed);
        }
        let transferred = self.controls.transferred()?;
        // SAFETY: ST/CR/FRE/FR 已完整停止，IDENTIFY owner 原页仍存活，期间无 CPU 访问。
        unsafe {
            self.controls.identify.sync_for_cpu(
                0,
                self.controls.identify.byte_len(),
                DmaDirection::FromDevice,
            )
        }?;
        if failed_command {
            return Err(BlockError::Io);
        }
        if transferred != ata::IDENTIFY_BYTES {
            return Err(BlockError::ShortIo);
        }
        let mut words = [0; ata::IDENTIFY_WORDS];
        // SAFETY: 完成同步且完整停止后独占读取 512B；CPU 不保留越过本作用域的 DMA 切片。
        unsafe {
            let source = self.controls.identify.region().cpu.as_ptr().cast::<u16>();
            for (index, word) in words.iter_mut().enumerate() {
                *word = u16::from_le(ptr::read_volatile(source.add(index)));
            }
        }
        ata::Identify::parse(&words)
    }
}

impl Drop for Ahci {
    /// 未停稳时 ManuallyDrop 继续保留原控制页；确认停止和全部外部尾部后才消费 owner。
    fn drop(&mut self) {
        assert!(
            self.state.lock().phase == StopPhase::Stopped,
            "dropping active AHCI controller"
        );
        // SAFETY: 最后 Arc 消失前拥有者已确认 IRQ/Work 全尾；状态确认 CR/FR 均停，CPU 无并发访问。
        unsafe { ManuallyDrop::drop(&mut self.controls) };
    }
}

/// 期限回调只通知原等待身份；同步取消返回前，固定栈 Waker 始终有效。
fn wake_timeout(timer: Pin<&Timer>) {
    // SAFETY: wait_irq 固定 Waker 栈地址，cancel_sync 确认真实回调尾部后才退出该轮。
    let waker = unsafe { &*(timer.context() as *const Waker) };
    waker.notify();
}

/// 没有可信成功前缀的命令失败统一保持零完成字节。
fn failed(error: BlockError) -> IoResult {
    IoResult {
        completed: 0,
        error: Some(error),
    }
}

#[cfg(feature = "ktest")]
#[::ktest::suite(context = "Boot", tags("block", "ahci", "protocol"), timeout_ms = 1000)]
mod protocol {
    /// 直接检查正式 ATA 编码的 4Kn、LBA48 高位、独立 FLUSH 和不支持字段，无硬件访问。
    #[test]
    fn identify_geometry_and_flush() {
        super::ata::self_check();
    }

    /// SATA DET=0 才表示无介质，未完成链路协商、PHY 离线和失效 MMIO 均不能当缺盘。
    #[test]
    fn media_absence_boundary() {
        assert!(super::registers::media_absent(0));
        assert!(!super::registers::media_absent(1));
        assert!(!super::registers::media_absent(0x103));
        assert!(!super::registers::media_absent(4));
        assert!(!super::registers::media_absent(u32::MAX));
    }
}
