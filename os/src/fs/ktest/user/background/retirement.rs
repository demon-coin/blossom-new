//! Ext4 卷服务停止、预留缓冲池（`BioReserve`）物理页回收与弱引用（`Weak`）生命周期顺序测试。
//!
//! # 模块职责与架构定位
//! 本模块隶属于 `fs::ktest::user::background` 测试套件，专门验证文件系统卸载、Inode 缓存退役（`InodeCache` Retirement）与块层预留缓冲池（`BioReserve`）的精细生命周期契约：
//! 1. **卷服务退役与最后元数据缓存弱引用（`Node Weak`）释放顺序**：当可读写挂载的 Ext4 卷正常卸载且 `InodeCache` 彻底退役后，`Ext4Volume`、`MetadataCache` 与 `WritebackDomain` 立即停止并销毁；但若外部观察者仍持有一个 `MetadataCache` 索引节点控制块的 `Weak` 引用，则 `BioReserve` 中预留的 $C = 64$ 个元数据物理页（`Folio`）会一直保活，直到该最后的 `Weak` 引用在远端 CPU 的关中断核间调用（IPI / CSD）中被释放，64 个物理页才一次性归还给物理页分配器（Buddy / PCP）。
//! 2. **回写域停止唤醒阻塞借用者与拒绝过期 I/O 上下文**：当回写预留通道（`BioLane`）配额耗尽导致后台回写线程或借用者阻塞时，外部 `InodeCache` 退役触发的回写域停止（`stop_sequence` 递增）必须立即以 `BlockError::QueueStopped` 唤醒阻塞的借用者，等待在途 I/O 完成后彻底停机；此后旧代的 `MetadataIoContext` 请求将被直接拒绝（`BlockError::NotReady`），且同一卷重新绑定新的 `InodeCache` 后不会复活旧等待者或旧上下文。
//! 3. **预留块请求（`Bio`）两阶段回收（负载 `Vec` 与控制块 `Weak` 分离）**：当 8 个使用预留池配额的父 `Bio` 完成磁盘 I/O 后，仅归还 DMA 内存 `Vec`（使 `payload_live` 清零）并不会释放预留槽位（`control_live` 仍为 8，`free_wb == 0`），第 9 个并发请求仍会阻塞；直到第 0 个 `Bio` 的最后一个强引用与最后一个 `Weak` 引用在远端 CPU 上彻底释放，释放出 1 个完整的预留控制单元，第 9 个请求才能获得该槽位并复用相同的底层预留缓冲区地址完成 I/O。
//! 4. **同一 Ext4 卷的 `InodeCache` 退役与重绑定**：无论前任 `InodeCache` 是通过主动清理空槽退役、首次加载 Inode 遭遇堆内存耗尽（OOM）后退役，还是在外部仍持有活动 `InodeRef` 期间发起退役（此时竞争缓存试图绑定同一卷会收到 `FsError::Busy`，直至外部 `InodeRef` 释放完成退役），一旦前任缓存彻底退役，同一 `Ext4Volume` 均可安全绑定新的 `InodeCache` 并恢复正常回写服务。
//!
//! # 核心机制：`BioReserve` 预留资源与 `Weak` 引用计数生命周期
//! ```text
//!   [卸载与卷销毁阶段]
//!   vfs.unmount() -> InodeCache 退役 -> WritebackDomain 停止
//!   drop(Ext4Volume) + drop(MetadataCache)
//!         |
//!         v
//!   Ext4Volume / MetadataCache / WritebackDomain 强引用归零销毁
//!   但若外部仍持有 1 个 Node Weak:
//!     IndexAllocator 控制块未释放 -> 持有的 BioReserve 强引用 = 1
//!     -> 64 个预留元数据物理页 (ref_count = 2) 暂不归还伙伴系统
//!         |
//!         |-- remote_observer_put() 在远端 CPU 关中断回调中 drop(Weak)
//!         v
//!     IndexAllocator 释放 -> BioReserve 强引用归零 -> 64 个物理页全部归还 Buddy/PCP!
//!
//!   [预留 Bio 槽位 (8 槽) 回收状态机]
//!   8 个 Bio 提交并完成 I/O:       payload_live = 8, control_live = 8, free_wb = 0
//!   take_memory() 并 drop(Vec):   payload_live = 0, control_live = 8, free_wb = 0 (第 9 个请求仍阻塞)
//!   drop(BioCompletion 强引用):   payload_live = 0, control_live = 8, free_wb = 0 (因仍有 1 个 Weak)
//!   remote_observer_put(Weak):    payload_live = 0, control_live = 7, free_wb = 1 -> 唤醒第 9 个请求!
//! ```
//!
//! # 锁顺序与并发约束
//! - **远端 CPU 关中断回调（`put_observer`）不可睡眠契约**：通过 `cpu::call` 发送到目标 CPU 执行的同步回调函数运行在关中断（IRQ Disabled）上下文中；被释放的最后一个 `Weak` 引用在触发底层分配器（`IndexAllocator` 或 `BioAllocator`）回收控制块及物理页时，仅执行无睡眠的内存归还操作，严禁在析构路径中获取睡眠锁、发起块 I/O 或等待线程汇合。

use super::*;
use crate::block::{Bio, BioAllocator, BioLease, BlkOperation};
use crate::mm::dma::MappedPages;
use crate::workqueue::testing::metadata_stats;

/// 读取指定磁盘块的元数据缓存并以原字节值原地写回以标记为脏（`Dirty`），随后开启写回状态并返回绑定了卷回写域的 [`crate::mm::FolioWriteback`] 凭证。
fn unchanged_metadata_writeback(
    volume: &Ext4Volume,
    block: u64,
) -> Result<crate::mm::FolioWriteback, FsError> {
    let size = volume.geometry().block_size;
    let buffer = volume.metadata_cache().get(block, size, None)?;
    buffer.update(volume.claim(), |bytes| {
        let first = bytes[0];
        bytes[..1].copy_from_slice(&[first]);
    })?;
    if PAGE_SIZE < buffer.offset() + size {
        return Err(FsError::InvalidOperation);
    }
    let mut content = buffer.folio().lock_content();
    // SAFETY:
    // 1. 前置 `volume.metadata_cache().get(...)` 与 `buffer.update(...)` 已将该元数据块读入缓存页并建立 `Dirty` 标记；
    // 2. 前置 `if PAGE_SIZE < buffer.offset() + size` 已校验 `[buffer.offset(), buffer.offset() + size)` 完整落在单页边界内；
    // 3. 当前已持有 `content = buffer.folio().lock_content()` 页内容锁，排斥并发内容修改，且元数据缓存页永不建立用户态页表映射，DMA 写回期间不会发生并发脏改。
    let writeback = unsafe { content.begin_writeback(buffer.offset(), size)? }
        .ok_or(FsError::InvalidOperation)?;
    Ok(writeback.with_writeback_domain(volume.writeback_domain(), false))
}

/// 使用一个预留池租约（`BioLease`）与已开启写回的元数据页凭证构造单个块层写请求（`Bio`），并在构造完成后立即释放 `lease` 句柄。
fn reserved_metadata_bio(
    volume: &Ext4Volume,
    block: u64,
    writeback: crate::mm::FolioWriteback,
    lease: BioLease,
) -> Result<Bio, FsError> {
    let mut memory = lease.memory()?;
    let mapped = MappedPages::from_writeback(writeback).map_err(|(_, writeback)| {
        drop(writeback);
        FsError::InvalidOperation
    })?;
    memory.push(mapped);
    // SAFETY:
    // 1. `MappedPages::from_writeback(writeback)` 已接管已初始化且处于写回状态的元数据物理页，未向外部暴露任何可写 CPU 裸指针别名；
    // 2. 块偏移由 `volume.view().absolute_byte(...)` 校验并转换为合法扇区号，操作类型为 `BlkOperation::Write`；
    // 3. `volume.claim()` 保证底层块设备与关联页面在所有子请求（Child Bio）及完成回调彻底执行完毕前持续有效，且完成回调参数为 `None`（不额外派生工作队列任务）。
    let bio = unsafe {
        Bio::try_new(
            volume.claim().disk(),
            BlkOperation::Write,
            volume.view().absolute_byte(
                block * volume.geometry().block_size as u64,
                volume.geometry().block_size,
            )? / crate::block::SECTOR_BYTES,
            memory,
            0,
            None,
        )
    }?;
    drop(lease);
    Ok(bio)
}

/// 在关中断的核间同步调用（CSD）回调中取出并释放 `argument` 指向的观察者句柄 `Option<T>`。
///
/// # Safety
/// 1. 调用方必须保证当前处于 `cpu::call` 的关中断回调上下文（`!crate::arch::irq_enabled()`）；
/// 2. `argument` 必须指向调用栈上有效、对齐且在 `cpu::call` 返回前独占存活的 `Option<T>` 实例，执行期间无任何其他线程并发访问该指针；
/// 3. `T` 的析构函数（`Drop`）必须仅执行非阻塞的分配器槽位或物理页归还操作，严禁睡眠、等待 I/O 或获取文件系统睡眠锁。
unsafe fn put_observer<T: Send>(argument: *mut ()) {
    assert!(!crate::arch::irq_enabled());
    // SAFETY:
    // 1. 调用方 `remote_observer_put` 将栈上可变引用 `&mut Option<T>` 转换为 `argument` 传入，并在 `cpu::call` 同步等待期间不再访问该变量，保证此处可变借用独占且生命周期有效；
    // 2. 调用方契约保证 `T` 为最后一个 `Weak` 观察者，其 `Drop` 仅归还底层分配器控制块与物理页，不会在关中断上下文中睡眠。
    let observer = unsafe { &mut *argument.cast::<Option<T>>() };
    drop(observer.take());
}

/// 通过同步核间调用（`cpu::call`）将 `observer` 中的观察者句柄移交至指定的在线 CPU `target` 上执行释放。
///
/// # Safety
/// 1. `target` 必须是在线的有效 CPU 编号（`target < cpu::count()`）；
/// 2. `T` 的析构（`Drop`）必须满足关中断上下文约束：仅归还底层分配器槽位或物理页，不得睡眠、等待 I/O 或汇合线程；
/// 3. 当前 CPU 不得持有任何会被 `T::drop` 递归获取的自旋锁，且必须能够正常响应核间中断（IPI）。
unsafe fn remote_observer_put<T: Send>(observer: &mut Option<T>, target: usize) {
    // SAFETY:
    // 1. 调用方保证 `target` 为在线 CPU 编号，`observer` 指向当前栈帧上的独占 `Option<T>`，且 `cpu::call` 会同步阻塞直到目标核执行完 `put_observer::<T>`；
    // 2. 调用方保证 `T` 的析构函数仅运行非睡眠的分配器与物理页回收逻辑，满足 `put_observer` 的安全前提。
    unsafe {
        cpu::call(
            target,
            put_observer::<T>,
            core::ptr::from_mut(observer).cast(),
        )
    };
}

#[ktest::suite(fixture = BackgroundOwner, suite_resources = UserSuite, context = "Thread", tags("fs-background", "fs-sync", "fs-path"), resources("root"), timeout_ms = 240000)]
pub(super) mod cases {
    use super::*;

    /// 测试目标：验证可读写挂载的 Ext4 卷在卸载与 `InodeCache` 退役后立即停止卷回写服务，而由 `BioReserve` 预留的 64 个元数据物理页则精确保持到最后一个 `MetadataCache` 节点 `Weak` 引用在远端 CPU 上释放时才归还给物理页分配器。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：在只读根文件系统 `/mnt/extra` 挂载可读写的第二块 Ext4 磁盘 `volume`，修改挂载点根目录元数据并同步落盘；随后读取卷末尾的冷元数据块，记录其所在的 `BioReserve` 槽位与物理页地址，并获取该缓存节点的 `Weak` 观察引用 `observer`；
    /// 2. When：释放元数据缓冲区句柄并调用 `metadata.prune_unused()` 回收节点强引用（此时 `observer.upgrade()` 为 `None`，但因 `observer` 仍持有一个 `Weak` 引用，`metadata_control_live == 1`）；接着卸载 `/mnt/extra`，销毁 `Vfs`、`InodeCache`、`MetadataCache`、`Ext4Volume` 与 `BioReserve` 的所有强引用；最后通过 `remote_observer_put` 在另一颗 CPU 的关中断回调中释放 `observer`；
    /// 3. Then：
    ///    - 在释放 `observer` 之前，`Ext4Volume`、`MetadataCache` 与 `WritebackDomain` 均已销毁（`Weak::upgrade().is_none()`），但 `BioReserve` 仍因 `IndexAllocator` 控制块未释放而保持 `strong_count() == 1`，64 个预留元数据物理页的引用计数均为 2；
    ///    - 在远端 CPU 释放 `observer` 后，`BioReserve` 强引用计数归零，系统空闲物理页总数（Buddy + PCP）立即增加至少 64 页。
    #[test]
    #[ktest(tags("W34", "fs-reserve-retire"), resources("extra"), min_cpus = 2)]
    fn unmounted_volume_retires_before_remote_last_node_weak_returns_source_pages(
        context: &mut TestContext<'_>,
        fixture: &mut BackgroundOwner,
    ) -> TestResult {
        // Step 1: 打开只读宿主卷与可读写挂载卷，构建独立 Vfs 与 InodeCache 并将可读写卷挂载到 /mnt/extra。
        let root_disk = fixture.write.user.path.owner.disk()?.clone();
        let extra_disk = fixture
            .write
            .user
            .path
            .owner
            .disks
            .get(1)
            .ok_or(TestError("reserve retirement requires ordinary extra disk"))?
            .clone();
        let host = checked(
            context,
            Ext4Volume::open(BlockView::whole(root_disk), AccessMode::ReadOnly, 64),
        )?;
        let volume = checked(
            context,
            Ext4Volume::open(
                BlockView::whole(extra_disk.clone()),
                AccessMode::ReadWrite,
                64,
            ),
        )?;
        fixture.write.cold_volume = Some(volume.clone());
        if volume.geometry().block_size != PAGE_SIZE {
            return Err(TestError(
                "source retirement requires ordinary 4KiB extra image",
            ));
        }
        let cache = Arc::try_new(checked(context, InodeCache::try_new(16))?)
            .map_err(|_| TestError("mounted retirement inode cache OOM"))?;
        fixture.retired = Some(cache.watch());
        let watch = cache.watch();
        let vfs = checked(
            context,
            Vfs::try_new(
                crate::fs::Filesystem::Ext4(host.clone()),
                cache.clone(),
                128,
                4,
                AccessMode::ReadOnly,
            ),
        )?;
        let fs_context = checked(context, FsContext::new(vfs.root(), vfs.root()))?;
        let covered = checked(
            context,
            vfs.resolve(&fs_context, b"/mnt/extra", ResolveMode::Follow),
        )?;

        checked(
            context,
            vfs.mount(
                &covered,
                crate::fs::Filesystem::Ext4(volume.clone()),
                AccessMode::ReadWrite,
            ),
        )?;
        let mounted = checked(
            context,
            vfs.resolve(&fs_context, b"/mnt/extra", ResolveMode::Follow),
        )?;

        let inode = checked(context, mounted.inode())?.clone();
        checked(
            context,
            inode.update_metadata(volume.claim(), |attributes| attributes.mode ^= 0o100),
        )?;
        checked(context, volume.sync())?;
        drained(&volume)?;

        // Step 2: 读取卷末尾冷元数据块并获取其缓存节点 Weak 引用，验证清理强引用后节点控制块仍因 Weak 存在而保活。
        let metadata = volume.metadata_cache().clone();
        let source = metadata
            .reserve()
            .ok_or(TestError("mounted RW source absent"))?;
        fixture.source = Some(source.clone());
        if source.capacity() != 64 {
            return Err(TestError("source retirement requires actual C=64 reserve"));
        }
        let block = volume.geometry().blocks_count - 1;
        let buffer = checked(
            context,
            metadata.get(block, PAGE_SIZE, None).map_err(FsError::from),
        )?;
        let physical = buffer.folio().phys_addr();
        let slot = source
            .metadata_slot_for(buffer.folio())
            .ok_or(TestError("cold metadata is not an original source page"))?;
        let mut observer = Some(
            metadata
                .ktest_node(block * PAGE_SIZE as u64)
                .ok_or(TestError("cold metadata has no genuine Node Weak"))?,
        );

        let held = source.snapshot();
        drop(buffer);
        metadata.prune_unused();
        context.expect_eq(observer.as_ref().unwrap().upgrade().is_none(), true);
        context.expect_eq(source.snapshot().metadata_control_live, 1);
        context.expect_eq(source.metadata_folio(slot).phys_addr(), physical);

        // Step 3: 验证持有活动 Inode 时卸载返回 Busy；释放引用后成功卸载并等待 InodeCache 与卷回写域彻底退役停止。
        context.expect_eq(vfs.unmount(&covered).err(), Some(FsError::Busy));
        drop(inode);
        drop(mounted);
        checked(context, vfs.unmount(&covered))?;
        let after_unmount = checked(
            context,
            vfs.resolve(&fs_context, b"/mnt/extra", ResolveMode::Follow),
        )?;
        context.expect_eq(after_unmount.ptr_eq(&covered), true);

        drop(after_unmount);
        drop(covered);
        drop(fs_context);
        drop(vfs);
        drop(host);
        drop(cache);
        checked(context, watch.wait_retired())?;
        drop(watch);
        drop(fixture.retired.take());

        extra_disk.test_flush_completions();
        let stopped = volume.writeback_domain().snapshot();
        context.expect_eq(
            (stopped.active, stopped.pending, stopped.running),
            (false, false, false),
        );

        // Step 4: 释放卷、元数据缓存与预留池的全部外部强引用，验证 64 个预留物理页因最后的 Node Weak 未释放而继续保活。
        let weak_volume = Arc::downgrade(&volume);
        let weak_metadata = Arc::downgrade(&metadata);
        let weak_domain = Arc::downgrade(volume.writeback_domain());
        let weak_source = Arc::downgrade(&source);
        let mut original_pages = [0; 64];
        for (index, address) in original_pages.iter_mut().enumerate() {
            let folio = source.metadata_folio(index);
            *address = folio.phys_addr();
            context.expect_eq(folio.ref_count(), 2);
        }
        context.expect_eq(
            original_pages
                .iter()
                .enumerate()
                .all(|(index, address)| !original_pages[..index].contains(address)),
            true,
        );
        context.diagnostic(format_args!(
            "W34 retired-before-last-weak source={:p} C={} observed-slot={} observed-pfn={} original-pages={:?} layout-held={:?} source-tail={:?} stopped={:?} driver={:?} work-metadata={:?}",
            Arc::as_ptr(&source), source.capacity(), slot, physical / PAGE_SIZE, original_pages,
            held, source.snapshot(), stopped, extra_disk.test_snapshot(), metadata_stats(),
        ));

        drop(metadata);
        drop(volume);
        drop(fixture.write.cold_volume.take());
        drop(fixture.source.take());
        drop(source);
        context.expect_eq(weak_volume.upgrade().is_none(), true);
        context.expect_eq(weak_metadata.upgrade().is_none(), true);
        context.expect_eq(weak_domain.upgrade().is_none(), true);
        context.expect_eq(weak_source.strong_count(), 1);

        // Step 5: 在远端 CPU 的关中断回调中释放最后的 Node Weak，验证 BioReserve 销毁且 64 个物理页归还给物理内存分配器。
        let before_put = mm::stats();
        let target = (cpu::current().id + 1) % cpu::count();
        // SAFETY:
        // 1. `target` 由 `(cpu::current().id + 1) % cpu::count()` 计算得出，保证为有效在线 CPU；
        // 2. `InodeCache`、回写域及所有后台 Work 均已退役停止，`observer` 持有的最后一个 `Node Weak` 在释放时仅回收 `IndexAllocator` 控制块及 `BioReserve` 中的 64 个无主物理页，不会睡眠或获取文件系统锁。
        unsafe { remote_observer_put(&mut observer, target) };
        let after_put = mm::stats();
        context.expect_eq(weak_source.upgrade().is_none(), true);
        context.expect_eq(weak_source.strong_count(), 0);
        let free_before = before_put.buddy_free_pages + before_put.pcp_cached_pages;
        let free_after = after_put.buddy_free_pages + after_put.pcp_cached_pages;
        context.expect_eq(free_before + 64 <= free_after, true);
        context.diagnostic(format_args!(
            "W34 final-node-weak remote-put-cpu={} callback-irq-save=1 sleeping-callback=0 source-expired={} C-original-pages=64 actual-free-before={} actual-free-after={} before={:?} after={:?} volume-metadata-work-expired-before-put=1 normal-unmount=1",
            target, weak_source.upgrade().is_none(), free_before, free_after, before_put, after_put,
        ));
        Ok(())
    }

    /// 测试目标：验证在回写预留通道（`BioLane`）耗尽或在途元数据 I/O 挂起期间，外部 `InodeCache` 退役能够唤醒阻塞的借用者、等待在途 I/O 完成并使旧代 `MetadataIoContext` 失效；且同一卷重新绑定新 `InodeCache` 后可执行零提交或堆耗尽（OOM）条件下的有界回写轮次。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：打开可读写 Ext4 卷并绑定第一个 `InodeCache`（`cache`），记录初始 `old_context`；先在回写域活跃状态下主动清理根 Inode（`prune_unused`）确认空缓存不误停回写域；随后占用全部 `BioLane` 使 `free_wb == 0`；
    /// 2. When & Then（阶段一：零配额下退役停止）：
    ///    - 启动 `borrower` 线程调用 `borrow_storage()` 使其因 `free_wb == 0` 阻塞，并发起 `request_progress` 使后台回写进入 `running` 状态；
    ///    - 启动 `stopper` 线程销毁 `cache` 并等待其退役；验证回写域停止后唤醒 `borrower`（返回 `Err(FsError::Io(BlockError::QueueStopped))`），`wait_progress` 报告 `stopped == true`，且使用 `old_context` 读取冷元数据块立即被拒绝（`Err(BlockError::NotReady)`，不产生磁盘读提交）；
    /// 3. When & Then（阶段二：重绑定新缓存与 OOM/零提交有界轮次）：
    ///    - 创建新的 `InodeCache`（`next`）重新绑定同一卷并加载根 Inode，确认回写域重新变为 `active == true`，而旧 `boundary` 与 `old_context` 依然保持失效状态；
    ///    - 在普通模式（`oom == false`）或耗尽堆内存模式（`oom == true`）下请求一轮回写进度并等待完成，验证 `finished_rounds` 正常递增且无脏页时以 0 字节提交正常结束；
    /// 4. When & Then（阶段三：在途元数据写回挂起期间退役）：
    ///    - 清理 `next` 中的根 Inode，挂起块设备完成回调并更新冷元数据块触发后台写回提交（`inflight > 0`）；
    ///    - 启动 `io_stopper` 线程销毁 `next`，验证 `io_stopper` 在在途 I/O 完成前保持阻塞，同时 `wait_progress(accepted_boundary).stopped` 立即置为 `true`；
    ///    - 恢复块设备完成回调后，`io_stopper` 顺利完成退役，所有在途请求与预留池控制块全部归零。
    #[test]
    #[case("zero_submit", false)]
    #[case("real_heap_oom", true)]
    #[ktest(tags("W34", "fs-reserve-stop", "fs-oom"), min_cpus = 2)]
    fn stopped_source_wakes_original_borrower_and_rejects_late_metadata_context(
        context: &mut TestContext<'_>,
        fixture: &mut BackgroundOwner,
        oom: bool,
    ) -> TestResult {
        let disk = fixture.write.user.path.owner.disk()?.clone();
        let volume = checked(
            context,
            Ext4Volume::open(BlockView::whole(disk.clone()), AccessMode::ReadWrite, 64),
        )?;
        fixture.write.cold_volume = Some(volume.clone());
        let cache = Arc::try_new(checked(context, InodeCache::try_new(4))?)
            .map_err(|_| TestError("source stop canonical cache OOM"))?;
        let watch = cache.watch();
        fixture.retired = Some(cache.watch());
        drop(checked(context, cache.iget(&volume, 2))?);
        let domain = volume.writeback_domain().clone();
        let source = volume
            .metadata_cache()
            .reserve()
            .ok_or(TestError("source stop has no reserve"))?;
        fixture.source = Some(source.clone());
        let old_context = domain.metadata_io_context(domain.snapshot().stop_sequence);
        let cold_block = volume.geometry().blocks_count - 1;
        context.expect_eq(
            checked(
                context,
                volume
                    .metadata_cache()
                    .cached_buffer(cold_block, volume.geometry().block_size)
                    .map_err(FsError::from),
            )?
            .is_none(),
            true,
        );
        let before = domain.snapshot();

        // Step 1: 在首个 InodeCache 仍存活时主动清理根 Inode，验证仅清空缓存槽位不会停止卷回写服务。
        context.diagnostic(format_args!(
            "W34 stop-stage=old-canonical-evict source={:p} oom={} slots={} active={}",
            Arc::as_ptr(&source),
            oom,
            cache.len(),
            before.active,
        ));
        context.expect_eq(cache.prune_unused(), 1);
        checked(context, watch.wait())?;
        context.expect_eq(watch.contains(2), false);
        context.expect_eq(cache.is_empty(), true);
        context.expect_eq(domain.snapshot().active, true);
        if !cache.is_empty() {
            return Err(TestError(
                "old canonical slot survived active-domain eviction",
            ));
        }

        // Step 2: 占满回写预留通道使 free_wb 为 0，启动 borrower 线程阻塞在预留池上，随后销毁首个 InodeCache 触发停止。
        fixture.lane = Some(checked(
            context,
            source.acquire_wb_lane(|| false).map_err(FsError::from),
        )?);
        context.expect_eq(source.snapshot().free_wb, 0);
        let control = WaveControl::try_new()?;
        fixture.control = Some(control.clone());
        let borrowing = domain.clone();
        let borrower = fixture
            .write
            .consume_sync(0, move || borrowing.borrow_storage().map(drop))?;
        let stopping = cache.watch();
        let stop_control = control.clone();
        let stopper = fixture.write.consume_sync(1, move || {
            stop_control.gate.wait();
            drop(cache);
            stopping.wait_retired()?;
            Ok(())
        })?;

        until("original storage borrower did not block on zero WB", || {
            borrower.blocked() && 0 < source.ktest_waiting()
        })?;
        let boundary = domain.request_progress(&volume);
        until(
            "original callback did not block behind borrowed scratch",
            || domain.snapshot().running,
        )?;
        let blocked = domain.snapshot();
        context.diagnostic(format_args!(
            "W34 stop-stage=old-owner-retire source={:p} zero-wb={} borrower-blocked={} source-waiting={} callback-running={} old-canonical-absent={}",
            Arc::as_ptr(&source), source.snapshot().free_wb == 0, borrower.blocked(),
            source.ktest_waiting(), blocked.running, !watch.contains(2),
        ));

        control.gate.complete();
        until(
            "external stop did not wake and finish original borrower and callback",
            || borrower.result().is_some() && stopper.result().is_some(),
        )?;
        fixture.write.join_consumers()?;
        context.expect_eq(
            borrower.result(),
            Some(Err(FsError::Io(BlockError::QueueStopped))),
        );
        context.expect_eq(stopper.result(), Some(Ok(())));
        let stopped = domain.snapshot();
        let old_progress = domain.wait_progress(boundary);
        context.expect_eq(old_progress.stopped, true);
        context.expect_eq(before.stop_sequence < stopped.stop_sequence, true);
        context.expect_eq(before.finished_rounds < stopped.finished_rounds, true);
        context.expect_eq(stopped.running, false);
        context.expect_eq(stopped.pending, false);
        context.expect_eq(source.ktest_waiting(), 0);

        let reads = disk.test_snapshot().submitted;
        context.expect_eq(
            volume
                .metadata_cache()
                .get(cold_block, volume.geometry().block_size, Some(old_context))
                .err(),
            Some(BlockError::NotReady),
        );
        context.expect_eq(disk.test_snapshot().submitted, reads);

        drop(fixture.lane.take());
        checked(context, watch.wait_retired())?;
        drop(watch);
        drop(fixture.retired.take());

        // Step 3: 将同一 Ext4 卷绑定到新建的第二个 InodeCache，验证回写服务恢复激活，但旧代等待边界与旧 I/O 上下文仍保持失效。
        context.diagnostic(format_args!(
            "W34 stop-stage=new-canonical-bind source={:p} old-stop-sequence={} active={}",
            Arc::as_ptr(&source),
            stopped.stop_sequence,
            domain.snapshot().active,
        ));
        let next = Arc::try_new(checked(context, InodeCache::try_new(4))?)
            .map_err(|_| TestError("new source canonical cache OOM"))?;
        let next_watch = next.watch();
        fixture.retired = Some(next.watch());
        let root = checked(context, next.iget(&volume, 2))?;
        context.expect_eq(domain.snapshot().active, true);
        context.expect_eq(domain.wait_progress(boundary).stopped, true);
        context.expect_eq(
            borrower.result(),
            Some(Err(FsError::Io(BlockError::QueueStopped))),
        );
        context.expect_eq(
            volume
                .metadata_cache()
                .get(cold_block, volume.geometry().block_size, Some(old_context))
                .err(),
            Some(BlockError::NotReady),
        );
        drop(checked(
            context,
            volume
                .metadata_cache()
                .get(cold_block, volume.geometry().block_size, None)
                .map_err(FsError::from),
        )?);

        // Step 4: 在零脏页或堆内存耗尽（OOM）条件下触发一轮回写，验证回写轮次仍能有界完成并推进 finished_rounds。
        let pressure = if oom {
            Some(
                fixture
                    .write
                    .user
                    .path
                    .owner
                    .pressure
                    .take()
                    .ok_or(TestError("finite round real heap pressure missing"))?,
            )
        } else {
            None
        };
        let evidence = Arc::try_new(SpinLock::new(None))
            .map_err(|_| TestError("finite round evidence OOM"))?;
        let actor_evidence = evidence.clone();
        let round_domain = domain.clone();
        let round_volume = volume.clone();
        let finite = fixture.write.consume_sync(0, move || {
            let mut pressure = pressure;
            let allocation = pressure.as_mut().map(|pressure| pressure.exhaust());
            let start = round_domain.snapshot();
            let boundary = round_domain.request_progress(&round_volume);
            let progress = round_domain.wait_progress(boundary);
            let finish = round_domain.snapshot();
            drop(pressure);
            *actor_evidence.lock() = Some((allocation, start, finish, progress));
            Ok(())
        })?;

        until(
            "finite zero-submit/OOM round did not publish its actual finished boundary",
            || finite.result().is_some(),
        )?;
        fixture.write.join_consumers()?;
        context.expect_eq(finite.result(), Some(Ok(())));
        let (allocation, start, finish, progress) = evidence
            .lock()
            .take()
            .ok_or(TestError("finite round has no real boundary evidence"))?;
        context.expect_eq(allocation, oom.then_some((true, true)));
        context.expect_eq(start.finished_rounds < finish.finished_rounds, true);
        context.expect_eq(
            (
                progress.submitted_bytes,
                progress.completed_bytes,
                progress.stopped,
                progress.error,
            ),
            (0, 0, false, false),
        );
        context.expect_eq(
            (finish.round_selected_data, finish.round_selected_metadata),
            (0, 0),
        );

        // Step 5: 清理第二个 InodeCache 的根 Inode，挂起块设备完成回调并提交脏元数据块，验证缓存退役会等待在途 I/O 彻底结束。
        drop(root);
        context.diagnostic(format_args!(
            "W34 stop-stage=new-canonical-evict source={:p} slots={} active={} finite-finished-rounds={}",
            Arc::as_ptr(&source), next.len(), domain.snapshot().active, finish.finished_rounds,
        ));
        context.expect_eq(next.prune_unused(), 1);
        checked(context, next_watch.wait())?;
        context.expect_eq(next_watch.contains(2), false);
        context.expect_eq(next.is_empty(), true);
        context.expect_eq(domain.snapshot().active, true);
        if !next.is_empty() {
            return Err(TestError(
                "new canonical slot survived active-domain eviction",
            ));
        }

        let buffer = checked(
            context,
            volume
                .metadata_cache()
                .get(cold_block, volume.geometry().block_size, None)
                .map_err(FsError::from),
        )?;
        disk.test_suspend_completions();
        checked(
            context,
            buffer
                .update(volume.claim(), |bytes| {
                    let first = bytes[0];
                    bytes[..1].copy_from_slice(&[first]);
                })
                .map_err(FsError::from),
        )?;
        drop(buffer);
        let accepted_boundary = domain.request_progress(&volume);
        until(
            "background callback did not accept original metadata before stop",
            || {
                domain.snapshot().running
                    && 0 < disk.test_snapshot().inflight
                    && 0 < source.snapshot().control_live
            },
        )?;

        let accepted = disk.test_snapshot();
        context.diagnostic(format_args!(
            "W34 stop-stage=accepted-owner-retire source={:p} old-canonical-absent={} accepted-driver={:?} source-layout={:?}",
            Arc::as_ptr(&source), !next_watch.contains(2), accepted, source.snapshot(),
        ));

        let io_stopping = next.watch();
        let io_stopper = fixture.write.consume_sync(1, move || {
            drop(next);
            io_stopping.wait_retired()?;
            Ok(())
        })?;
        until(
            "external stop did not wait for accepted IO/Work tail",
            || finish.stop_sequence < domain.snapshot().stop_sequence && io_stopper.blocked(),
        )?;
        context.expect_eq(io_stopper.result(), None);
        context.expect_eq(domain.wait_progress(accepted_boundary).stopped, true);

        disk.test_resume_completions();
        fixture.write.join_consumers()?;
        disk.test_flush_completions();
        context.expect_eq(io_stopper.result(), Some(Ok(())));
        context.expect_eq(domain.snapshot().active, false);
        context.expect_eq(domain.snapshot().pending, false);
        context.expect_eq(domain.snapshot().running, false);
        context.expect_eq(disk.test_snapshot().inflight, 0);
        context.expect_eq(source.snapshot().control_live, 0);
        context.expect_eq(source.snapshot().payload_live, 0);

        checked(context, next_watch.wait_retired())?;
        drop(next_watch);
        drop(fixture.retired.take());
        context.diagnostic(format_args!(
            "W34 stop source={:p} oom={} allocation-null={:?} borrower-blocked=1 callback-running-with-scratch-owned=1 before={:?} blocked={:?} stopped={:?} finite-start={:?} finite-finished={:?} progress={:?} accepted-driver={:?} finished-driver={:?} source-layout={:?} old-context-after-stop-and-rebind=NotReady old-waiter-stopped=1 canonical-evicted-before-owner-stop=1 canonical-retired-before-rebind=1 accepted-stop-by-owner=1",
            Arc::as_ptr(&source), oom, allocation, before, blocked, stopped, start, finish, progress,
            accepted, disk.test_snapshot(), source.snapshot(),
        ));

        fixture.control.take();
        fixture.source.take();
        Ok(())
    }

    /// 测试目标：验证当 8 个使用预留池配额的元数据写请求（`Bio`）完成磁盘 I/O 后，仅归还负载 `Vec` 和释放强引用仍不足以放行第 9 个请求，必须等到最后一个 `Bio` 弱引用（`Weak`）释放底层分配器控制单元后，第 9 个请求才能复用该预留槽位完成提交。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：打开未挂载（后台回写未激活）的可读写 Ext4 卷，获取其 `BioReserve` 与包含 8 个配额单元的 `BioLane`；预先为第 9 号块准备脏元数据写回凭证 `ninth`，并创建等待门控的消费者线程 `actor`（唤醒后将申请通道、租用槽位并提交第 9 号块的 `Bio`）；
    /// 2. When：
    ///    - 主线程依次租用全部 8 个预留单元，为第 1..=8 号块构造并提交 8 个预留 `Bio`，等待全部 8 个请求完成磁盘 I/O（此时 `inflight == 0, control_live == 8, payload_live == 8`）；
    ///    - 主线程调用 `take_memory()` 取出并释放全部 8 个请求的负载 `Vec`，验证 `payload_live` 降为 0 但 `control_live` 仍为 8；
    ///    - 主线程提取第 0 个完成句柄的 `Weak` 引用 `weak`，释放 `BioLane` 并放行 `actor` 线程，确认 `actor` 因 `free_wb == 0` 阻塞在预留池上；
    ///    - 主线程释放第 0 个 `BioCompletion` 的强引用，确认因 `weak` 仍存在，`control_live` 仍为 8 且 `actor` 继续阻塞；
    ///    - 主线程通过 `remote_observer_put` 在远端 CPU 的关中断回调中释放 `weak`；
    /// 3. Then：释放 `weak` 立即归还第 0 号预留控制单元，唤醒 `actor` 线程获得长度为 1 的 `BioLane`，且第 9 个 `Bio` 复用了与第 0 个请求完全相同的预留负载缓冲区首地址（`ninth_payload == payload_addresses[0]`）完成写入。
    #[test]
    #[ktest(tags("W33", "W34", "fs-reserve-observer"), min_cpus = 2)]
    fn eight_completed_reserved_parents_block_ninth_until_real_weak_tail(
        context: &mut TestContext<'_>,
        fixture: &mut BackgroundOwner,
    ) -> TestResult {
        let disk = fixture.write.user.path.owner.disk()?.clone();
        let volume = checked(
            context,
            Ext4Volume::open(BlockView::whole(disk.clone()), AccessMode::ReadWrite, 64),
        )?;
        fixture.write.cold_volume = Some(volume.clone());
        if volume.geometry().block_size != PAGE_SIZE {
            return Err(TestError(
                "reserve observer requires ordinary 4KiB root image",
            ));
        }
        let source = volume
            .metadata_cache()
            .reserve()
            .ok_or(TestError("RW metadata has no real reserve"))?;
        context.expect_eq(volume.writeback_domain().snapshot().active, false);
        context.expect_eq(volume.writeback_domain().snapshot().rounds, 0);
        fixture.source = Some(source.clone());
        let lane = checked(
            context,
            source.acquire_wb_lane(|| false).map_err(FsError::from),
        )?;
        context.expect_eq(lane.len(), 8);
        fixture.lane = Some(lane);
        let control = WaveControl::try_new()?;
        fixture.control = Some(control.clone());

        // Step 1: 准备第 9 号块的写回凭证与等待门控的第 9 个消费者线程。
        let ninth = checked(context, unchanged_metadata_writeback(&volume, 9))?;
        let record = Arc::try_new(SpinLock::new(None))
            .map_err(|_| TestError("ninth acceptance observation OOM"))?;
        let actor_record = record.clone();
        let actor_source = source.clone();
        let actor_volume = volume.clone();
        let actor_control = control.clone();
        let actor = fixture.write.consume_sync(0, move || {
            actor_control.gate.wait();
            let lane =
                actor_source.acquire_wb_lane(|| actor_control.cancelled.load(Ordering::Acquire))?;
            let count = lane.len();
            let (_, lease) =
                lane.lease_available(|| actor_control.cancelled.load(Ordering::Acquire))?;
            let bio = reserved_metadata_bio(&actor_volume, 9, ninth, lease)?;
            let completion = bio.completion();
            actor_volume.claim().submit_bio(bio);
            let result = completion.wait();
            let memory = completion.take_memory()?;
            let reserved = matches!(memory.allocator(), BioAllocator::Reserved { .. });
            *actor_record.lock() = Some((
                count,
                completion.test_acceptance(),
                result,
                reserved,
                memory.as_ptr() as usize,
            ));
            drop(memory);
            drop(completion);
            drop(lane);
            result.error.map_or(Ok(()), |error| Err(FsError::Io(error)))
        })?;

        // Step 2: 租用全部 8 个预留槽位构造并提交 8 个元数据写 Bio，等待所有磁盘 I/O 完成。
        let before = disk.test_snapshot();
        for index in 0..8 {
            let block = index as u64 + 1;
            let writeback = checked(context, unchanged_metadata_writeback(&volume, block))?;
            let lease = checked(
                context,
                fixture
                    .lane
                    .as_ref()
                    .unwrap()
                    .try_lease(index, || false)
                    .map_err(FsError::from),
            )?
            .ok_or(TestError("prepared WB book lost a complete unit"))?;
            let bio = checked(
                context,
                reserved_metadata_bio(&volume, block, writeback, lease),
            )?;
            let completion = bio.completion();
            fixture.observers[index] = Some(completion.clone());
            volume.claim().submit_bio(bio);
            let result = completion.wait();
            context.expect_eq((result.completed, result.error), (PAGE_SIZE, None));
            context.expect_eq(completion.test_acceptance().2, PAGE_SIZE);
        }

        disk.test_flush_completions();
        let completed = disk.test_snapshot();
        let held = source.snapshot();
        context.expect_eq(completed.inflight, 0);
        context.expect_eq(held.control_live, 8);
        context.expect_eq(held.payload_live, 8);

        // Step 3: 取出并释放 8 个已完成 Bio 的 DMA 内存 Vec，验证 payload_live 归零但 control_live 仍保持为 8。
        let mut returned = [const { None }; 8];
        let mut payload_addresses = [0; 8];
        for (index, (memory, completion)) in returned.iter_mut().zip(&fixture.observers).enumerate()
        {
            let owned = checked(
                context,
                completion
                    .as_ref()
                    .unwrap()
                    .take_memory()
                    .map_err(FsError::from),
            )?;
            context.expect_eq(
                matches!(owned.allocator(), BioAllocator::Reserved { .. }),
                true,
            );
            payload_addresses[index] = owned.as_ptr() as usize;
            *memory = Some(owned);
        }
        context.expect_eq(source.snapshot().payload_live, 8);
        drop(returned);
        context.expect_eq(source.snapshot().payload_live, 0);
        context.expect_eq(source.snapshot().control_live, 8);

        // Step 4: 保留第 0 个 Bio 的 Weak 引用，放行第 9 个消费者并释放第 0 个强引用，验证只要 Weak 未释放则第 9 个消费者始终阻塞。
        let mut weak = Some(fixture.observers[0].as_ref().unwrap().ktest_weak());
        drop(fixture.lane.take());
        control.gate.complete();
        until(
            "ninth consumer never blocked on the original empty source",
            || actor.blocked() && 0 < source.ktest_waiting(),
        )?;
        let blocked = disk.test_snapshot();
        context.expect_eq(actor.result(), None);
        context.expect_eq(source.snapshot().free_wb, 0);
        context.expect_eq(blocked.metadata_submitted, completed.metadata_submitted);

        drop(fixture.observers[0].take());
        context.expect_eq(source.snapshot().control_live, 8);
        context.expect_eq(source.snapshot().free_wb, 0);
        context.expect_eq(actor.blocked(), true);

        // Step 5: 在远端 CPU 的关中断回调中释放最后的 Bio Weak，验证槽位立即归还并唤醒第 9 个消费者复用同一预留缓冲区完成 I/O。
        let target = (cpu::current().id + 1) % cpu::count();
        // SAFETY:
        // 1. `target` 为有效在线 CPU 编号；
        // 2. 第 0 个 `Bio` 的负载 `Vec` 已在上方 `drop(returned)` 中释放，块设备完成队列已冲刷，且最后一个强引用已在第 720 行释放；此处释放最后一个 `Weak` 仅归还 `BioAllocator` 控制槽位并唤醒等待队列，不会睡眠或阻塞。
        unsafe { remote_observer_put(&mut weak, target) };
        until(
            "one real allocator tail did not admit and finish the ninth parent",
            || actor.result().is_some(),
        )?;
        fixture.write.join_consumers()?;
        context.expect_eq(actor.result(), Some(Ok(())));
        let (count, acceptance, result, reserved, ninth_payload) = record
            .lock()
            .take()
            .ok_or(TestError("ninth parent has no actual acceptance evidence"))?;
        context.expect_eq(count, 1);
        context.expect_eq(
            (acceptance.2, result.completed, result.error, reserved),
            (PAGE_SIZE, PAGE_SIZE, None, true),
        );
        context.expect_eq(ninth_payload, payload_addresses[0]);

        disk.test_flush_completions();
        let after = source.snapshot();
        context.expect_eq(after.control_live, 7);
        context.expect_eq(after.payload_live, 0);
        context.expect_eq(after.free_wb, 1);
        context.expect_eq(
            completed.metadata_submitted < disk.test_snapshot().metadata_submitted,
            true,
        );
        context.diagnostic(format_args!(
            "W33-W34 observer source={:p} domain-inactive=1 background-rounds={} real-wb-parents=8 ninth-blocked=1 returned-Vec-before-control=1 last-strong-before-last-weak=1 ninth-book={} ninth-reserved={} payload-addresses={:?} ninth-payload={} driver-before={:?} driver-completed={:?} driver-blocked={:?} driver-after={:?} layout-held={:?} layout-after={:?} weak-put-cpu={}",
            Arc::as_ptr(&source), volume.writeback_domain().snapshot().rounds, count, reserved, payload_addresses, ninth_payload,
            before, completed, blocked, disk.test_snapshot(), held, after, target,
        ));

        for observer in &mut fixture.observers {
            drop(observer.take());
        }
        fixture.control.take();
        fixture.source.take();
        Ok(())
    }

    /// 测试目标：验证 `InodeCache` 在空缓存主动清理、首次加载遭遇 OOM 失败以及存在外部活动 `InodeRef` 三种退役路径下，均能在旧缓存彻底退役后允许同一 `Ext4Volume` 安全绑定新的 `InodeCache`。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：打开可读写 Ext4 卷 `volume` 并创建第一个 `InodeCache`（`cache`），根据参数 `path` 执行三种前置路径：
    ///    - `path == 0`（`pruned_empty`）：加载根 Inode 修改元数据并同步落盘后，释放 `InodeRef` 并调用 `prune_unused()` 清空缓存槽位；
    ///    - `path == 1`（`first_load_oom`）：在耗尽堆内存（`pressure.exhaust()`）条件下首次调用 `cache.iget(&volume, 2)`，验证返回 `Err(FsError::NoMemory)`；
    ///    - `path == 2`（`live_inode`）：加载根 Inode 修改元数据并同步落盘后，外部继续持有该 `InodeRef`（`held = Some(inode)`）；
    /// 2. When：销毁 `cache` 句柄；若 `path == 2`，在释放外部 `inode` 前先尝试用新缓存 `competing` 绑定同一卷，验证因旧缓存尚未完成退役而返回 `Err(FsError::Busy)`，随后释放外部 `inode` 并等待 `watch.wait_retired()` 完成；
    /// 3. Then：旧缓存彻底退役后，新建的 `InodeCache`（`next`）能够成功绑定同一 `volume` 并正常加载根 Inode（确认为目录类型），最后正常退役清理。
    #[test]
    #[case("pruned_empty", 0)]
    #[case("first_load_oom", 1)]
    #[case("live_inode", 2)]
    #[ktest(tags("W30", "fs-background-retire", "fs-oom"), min_cpus = 2)]
    fn retired_cache_allows_same_volume_new_canonical_domain(
        context: &mut TestContext<'_>,
        fixture: &mut BackgroundOwner,
        path: usize,
    ) -> TestResult {
        let volume = checked(
            context,
            Ext4Volume::open(
                BlockView::whole(fixture.write.user.path.owner.disk()?.clone()),
                AccessMode::ReadWrite,
                64,
            ),
        )?;
        fixture.write.cold_volume = Some(volume.clone());
        let cache = Arc::try_new(checked(context, InodeCache::try_new(4))?)
            .map_err(|_| TestError("retirement cache owner OOM"))?;
        let watch = cache.watch();

        // Step 1: 按测试路径分支准备初始 InodeCache 状态（空槽清理、首次 iget OOM 或保留活动 Inode 引用）。
        let held = if path == 1 {
            let mut pressure = fixture
                .write
                .user
                .path
                .owner
                .pressure
                .take()
                .ok_or(TestError("retirement allocator pressure was not prepared"))?;
            let observed = Arc::try_new(SpinLock::new(None))
                .map_err(|_| TestError("retirement OOM observation allocation failed"))?;
            let record = observed.clone();
            let original = cache.clone();
            let filesystem = volume.clone();

            let consumer = fixture.write.consume_sync(0, move || {
                let evidence = pressure.exhaust();
                let result = original.iget(&filesystem, 2).err();
                drop(pressure);
                *record.lock() = Some((evidence, result));
                Ok(())
            })?;
            fixture.write.join_consumers()?;
            context.expect_eq(consumer.result(), Some(Ok(())));
            let (evidence, result) = observed.lock().take().ok_or(TestError(
                "retirement OOM producer did not publish its real result",
            ))?;
            context.expect_eq(evidence, (true, true));
            context.expect_eq(result, Some(FsError::NoMemory));
            None
        } else {
            let inode = checked(context, cache.iget(&volume, 2))?;
            checked(
                context,
                inode.update_metadata(volume.claim(), |attributes| {
                    attributes.mode ^= 0o100;
                }),
            )?;
            checked(context, volume.sync())?;
            drained(&volume)?;
            context.expect_eq(
                0 < volume.writeback_domain().snapshot().completed_bytes,
                true,
            );
            if path == 0 {
                drop(inode);
                context.expect_eq(cache.prune_unused(), 1);
                context.expect_eq(cache.len(), 0);
                None
            } else {
                Some(inode)
            }
        };

        // Step 2: 释放首个 InodeCache；若仍有外部活动 Inode，验证竞争缓存绑定被拒（Busy），待释放 Inode 后完成退役。
        drop(cache);
        if let Some(inode) = held {
            context.expect_eq(watch.contains(inode.number()), true);
            let competing = checked(context, InodeCache::try_new(4))?;
            context.expect_eq(competing.iget(&volume, 2).err(), Some(FsError::Busy));
            drop(competing);
            drop(inode);
        }
        checked(context, watch.wait_retired())?;
        drop(watch);

        // Step 3: 创建新的 InodeCache 绑定同一 Ext4Volume，验证可正常加载根 Inode 并完成后续退役。
        let next = checked(context, InodeCache::try_new(4))?;
        let next_watch = next.watch();
        let inode = checked(context, next.iget(&volume, 2))?;
        context.expect_eq(
            checked(context, inode.metadata_snapshot())?.kind,
            InodeKind::Directory,
        );

        drop(inode);
        drop(next);
        checked(context, next_watch.wait_retired())?;
        context.diagnostic(format_args!(
            "background-retirement path={} same-volume={} real-domain-rebind=1 snapshot={:?}",
            path,
            volume.device(),
            volume.writeback_domain().snapshot(),
        ));
        Ok(())
    }
}
