//! 启动阶段（Boot）固件设备树（FDT/DTB）解析、内存预留与 DMA 一致性策略测试套件。
//!
//! # 模块定位
//! 本模块隶属于内核自检框架（`ktest`）的启动阶段用例集（`boot_cases`），用于在启动早期验证平台设备树
//! （Flattened Device Tree, DTB）解析器、物理内存与保留区容量边界、动态预留分配、`no-map` 页表直映扣除
//! 以及设备 DMA 缓存一致性继承策略（`dma-coherent` / `dma-noncoherent` / `interconnects`）。
//! 上游由 `ktest` 启动阶段分发器调用，下游调用 `platform::ktest_*` 发现与校验接口、`boot::BootInfo`
//! 预留管理接口以及 `arch` 直映区域计算接口。
//!
//! # 核心机制与 FDT 二进制布局
//! 为避免破坏内核已发布的全局 `boot::info()` 快照或原始 DTB 物理内存，每个测试用例通过 `Dtb` 夹具
//! （`Fixture`）在 2 MiB 的静态缓冲区 `COPY` 中复制一份独立的 DTB 字节副本，并在原地执行属性插入、替换或
//! `FDT_NOP` 抹除操作以构造各种边界条件与损坏输入：
//!
//! ```text
//! Flattened Device Tree (DTB) 副本布局 (大端序 32-bit 字对齐)
//! ┌──────────────────────────────────────────────────────────────┐ 0
//! │ FDT Header (40 bytes / 10 words)                             │
//! │   +0x04 totalsize         +0x08 off_dt_struct                │
//! │   +0x0c off_dt_strings    +0x10 off_mem_rsvmap               │
//! │   +0x20 size_dt_strings   +0x24 size_dt_struct               │
//! ├──────────────────────────────────────────────────────────────┤ off_mem_rsvmap
//! │ Memory Reservation Block                                     │
//! ├──────────────────────────────────────────────────────────────┤ off_dt_struct
//! │ Structure Block                                              │
//! │   [FDT_PROP (3) | len | nameoff] + value (4-byte padded)     │
//! │   (插入/变长替换属性时整体平移后续 Structure 与 Strings 块)   │
//! ├──────────────────────────────────────────────────────────────┤ off_dt_strings
//! │ Strings Block (以 '\0' 结尾的属性名字符串池)                 │
//! └──────────────────────────────────────────────────────────────┘ length (<= CAPACITY 2 MiB)
//! ```
//!
//! # 锁与并发契约
//! - `COPY_LOCK`：保护静态大缓冲区 `COPY` 以及两个 `BootInfo` 快照槽 `ACTUAL`、`EXPECTED` 的唯一可变借用。
//! - 本模块所有用例均在启动阶段（单核 BSP、中断关闭）串行运行；`Dtb::set_up` 获取 `COPY_LOCK` 并将守卫
//!   保存在 `Dtb::_exclusive` 中，直到单个用例执行完毕析构 `Dtb` 时才释放锁，期间严禁跨用例保留切片或指针。

use crate::sync::{SpinLock, SpinLockGuard};
use crate::{arch, boot, platform};
use ::ktest::{Fixture, TestContext, TestError};
use core::convert::{TryFrom, TryInto};
use core::mem::MaybeUninit;

const CAPACITY: usize = 2 * 1024 * 1024;
const FDT_WORD_BYTES: usize = core::mem::size_of::<u32>();
const FDT_ALIGN_MASK: usize = FDT_WORD_BYTES - 1;

const FDT_TOTALSIZE: usize = FDT_WORD_BYTES;
const FDT_OFF_DT_STRUCT: usize = 2 * FDT_WORD_BYTES;
const FDT_OFF_DT_STRINGS: usize = 3 * FDT_WORD_BYTES;
const FDT_OFF_MEM_RSVMAP: usize = 4 * FDT_WORD_BYTES;
const FDT_SIZE_DT_STRINGS: usize = 8 * FDT_WORD_BYTES;
const FDT_SIZE_DT_STRUCT: usize = 9 * FDT_WORD_BYTES;
const FDT_HEADER_BYTES: usize = 10 * FDT_WORD_BYTES;
const FDT_PROPERTY_BYTES: usize = 3 * FDT_WORD_BYTES;
const FDT_PROP: usize = 3;
const FDT_NOP: u32 = 4;

static COPY_LOCK: SpinLock<()> = SpinLock::new(());
static mut COPY: [u8; CAPACITY] = [0; CAPACITY];
static mut ACTUAL: MaybeUninit<boot::BootInfo> = MaybeUninit::uninit();
static mut EXPECTED: MaybeUninit<boot::BootInfo> = MaybeUninit::uninit();

/// 设备树（DTB）可写副本测试夹具。
///
/// 每个测试实例在 `set_up` 时从当前启动的真实 DTB 复制一份独占字节序列到静态缓冲区 `COPY`，
/// 并提供两个未初始化的 `BootInfo` 静态槽供解析对比使用。
struct Dtb {
    /// 指向完整 2 MiB 静态缓冲区 `COPY` 的独占可变切片；实际有效 DTB 内容位于 `..self.length` 前缀，
    /// 尾部剩余空间供原地插入或扩展属性时平移数据使用。
    bytes: &'static mut [u8],
    /// 当前 DTB 副本的有效字节总长度（对应 FDT 头部的 `totalsize` 字段，单位：字节）。
    length: usize,
    /// 供被测解析路径写入输出结果的独占 `BootInfo` 静态槽。
    actual: &'static mut MaybeUninit<boot::BootInfo>,
    /// 供基准对照路径写入预期结果的独占 `BootInfo` 静态槽。
    expected: &'static mut MaybeUninit<boot::BootInfo>,
    /// 持有 `COPY_LOCK` 的自旋锁守卫，确保在夹具生命周期内独占访问 `COPY`、`ACTUAL` 和 `EXPECTED`。
    _exclusive: SpinLockGuard<'static, ()>,
}

impl Fixture for Dtb {
    /// 获取静态缓冲区独占锁，并将内核启动信息中的原始 DTB 完整复制到测试工作区。
    fn set_up(_: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        let exclusive = COPY_LOCK.lock();
        let info = boot::info();
        let length = info
            .dtb
            .end
            .checked_sub(info.dtb.start)
            .filter(|&n| n <= CAPACITY)
            .ok_or(TestError("DTB copy exceeds capacity"))?;
        // SAFETY:
        // 1. `info.dtb` 在内核早期启动阶段已通过校验并作为保留区永久映射，且上面已确认 `length <= CAPACITY`；
        // 2. 静态数组 `COPY` 位于内核 `.bss` 段，与原始 DTB 物理区不重叠；
        // 3. `exclusive` 持有了 `COPY_LOCK`，且所有访问 `COPY` 的代码均受该锁保护，在 `Self` 析构前不存在别名访问。
        let bytes = unsafe {
            let copy = core::ptr::addr_of_mut!(COPY).cast::<u8>();
            core::ptr::copy_nonoverlapping(
                arch::phys_to_virt(info.dtb.start) as *const u8,
                copy,
                length,
            );
            core::slice::from_raw_parts_mut(copy, CAPACITY)
        };
        Ok(Self {
            bytes,
            length,
            // SAFETY: `exclusive` 已持有 `COPY_LOCK`，保证了当前夹具实例对静态槽 `ACTUAL` 的唯一可变引用。
            actual: unsafe { &mut *core::ptr::addr_of_mut!(ACTUAL) },
            // SAFETY: `EXPECTED` 与 `COPY`、`ACTUAL` 内存互不重叠，且同样受 `COPY_LOCK` 独占保护直到夹具析构。
            expected: unsafe { &mut *core::ptr::addr_of_mut!(EXPECTED) },
            _exclusive: exclusive,
        })
    }
}

impl Dtb {
    /// 在 `/chosen` 节点中定位 `bootargs` 属性的值偏移与长度；若该属性不存在则先插入一个空属性，
    /// 返回 `(属性值在副本中的起始字节偏移, 当前属性值的字节长度)`。
    fn prepare_bootargs(&mut self) -> (usize, usize) {
        self.prepare_property("/chosen", "bootargs")
    }

    /// 在指定路径节点 `path` 中查找属性 `name`；若属性已存在则直接返回其值偏移与长度，
    /// 若不存在则在节点的首个属性之前原地插入一个长度为 0 的新属性条目，并将属性名追加到字符串块末尾。
    fn prepare_property(&mut self, path: &str, name: &str) -> (usize, usize) {
        // Step 1: 解析当前 DTB，若目标属性已存在则直接返回其值在副本中的偏移和长度；否则记录首个属性头部的插入偏移。
        let offset = {
            let tree = fdt::Fdt::new(&self.bytes[..self.length]).expect("actual DTB invalid");
            let node = tree.find_node(path).expect("actual DTB node absent");
            if let Some(property) = node.property(name) {
                return (
                    (property.value.as_ptr() as usize)
                        .checked_sub(self.bytes.as_ptr() as usize)
                        .expect("DTB property pointer outside copy"),
                    property.value.len(),
                );
            }
            let first = node
                .properties()
                .next()
                .expect("actual DTB node properties absent");
            (first.value.as_ptr() as usize)
                .checked_sub(self.bytes.as_ptr() as usize)
                .and_then(|n| n.checked_sub(FDT_PROPERTY_BYTES))
                .expect("DTB property offset overflow")
        };

        // Step 2: 读取并校验 FDT 头部各区块（保留表、结构块、字符串块）的偏移、大小与 4 字节对齐约束。
        let word = |field: usize| {
            let end = field
                .checked_add(FDT_WORD_BYTES)
                .expect("DTB header offset overflow");
            u32::from_be_bytes(self.bytes[..self.length][field..end].try_into().unwrap()) as usize
        };
        let structure = word(FDT_OFF_DT_STRUCT);
        let strings = word(FDT_OFF_DT_STRINGS);
        let strings_size = word(FDT_SIZE_DT_STRINGS);
        let structure_size = word(FDT_SIZE_DT_STRUCT);
        let structure_end = structure
            .checked_add(structure_size)
            .expect("DTB structure overflow");
        let strings_end = strings
            .checked_add(strings_size)
            .expect("DTB strings overflow");
        let value = offset
            .checked_add(FDT_PROPERTY_BYTES)
            .expect("DTB inserted property overflow");
        assert_eq!(word(FDT_TOTALSIZE), self.length);
        assert_eq!(word(offset), FDT_PROP);
        assert_eq!(structure & FDT_ALIGN_MASK, 0);
        assert_eq!(structure_size & FDT_ALIGN_MASK, 0);
        assert_eq!(offset & FDT_ALIGN_MASK, 0);
        assert!(FDT_HEADER_BYTES <= structure && word(FDT_OFF_MEM_RSVMAP) < structure);
        assert!(structure <= offset && value <= structure_end);
        assert!(structure_end <= strings && strings_end <= self.length);

        // Step 3: 计算插入 12 字节空属性头及在字符串块末尾追加属性名后的新偏移与总长度，确认不超过缓冲区容量。
        let name_length = name
            .len()
            .checked_add(1)
            .expect("DTB property name overflow");
        let name_start = strings_end
            .checked_add(FDT_PROPERTY_BYTES)
            .expect("DTB property name offset overflow");
        let name_end = name_start
            .checked_add(name_length)
            .expect("DTB property name overflow");
        let total = self
            .length
            .checked_add(FDT_PROPERTY_BYTES)
            .expect("DTB insertion overflow")
            .max(name_end);
        assert!(total <= self.bytes.len());
        let fields = [
            (FDT_TOTALSIZE, total),
            (
                FDT_OFF_DT_STRINGS,
                strings
                    .checked_add(FDT_PROPERTY_BYTES)
                    .expect("DTB strings offset overflow"),
            ),
            (
                FDT_SIZE_DT_STRINGS,
                strings_size
                    .checked_add(name_length)
                    .expect("DTB strings size overflow"),
            ),
            (
                FDT_SIZE_DT_STRUCT,
                structure_size
                    .checked_add(FDT_PROPERTY_BYTES)
                    .expect("DTB structure size overflow"),
            ),
            (offset, FDT_PROP),
            (
                offset
                    .checked_add(FDT_WORD_BYTES)
                    .expect("DTB property length offset overflow"),
                0,
            ),
            (
                offset
                    .checked_add(2 * FDT_WORD_BYTES)
                    .expect("DTB property name field overflow"),
                strings_size,
            ),
        ]
        .map(|(field, word)| {
            let end = field
                .checked_add(FDT_WORD_BYTES)
                .expect("DTB field write overflow");
            assert!(end <= total);
            (
                field,
                end,
                u32::try_from(word).expect("DTB inserted field exceeds u32"),
            )
        });

        // Step 4: 向后平移插入点之后的结构块与字符串块，写入属性名字符串并更新 FDT 头部与新属性描述符。
        self.bytes.copy_within(offset..self.length, value);
        self.bytes[name_start..name_end - 1].copy_from_slice(name.as_bytes());
        self.bytes[name_end - 1] = 0;
        for (field, end, word) in fields {
            self.bytes[field..end].copy_from_slice(&word.to_be_bytes());
        }
        self.length = total;
        (value, 0)
    }

    /// 在 DTB 副本中将偏移 `offset` 处原长度为 `old_length` 的属性值替换为新字节序列 `value`，
    /// 自动按 4 字节对齐平移后续结构块尾部与字符串块，并同步更新 FDT 头部及属性长度字段。
    fn replace_property(&mut self, offset: usize, old_length: usize, value: &[u8]) {
        // Step 1: 读取 FDT 头部偏移信息，校验目标属性头标识（`FDT_PROP`）、原长度及对齐状态。
        let word = |field: usize| {
            let end = field
                .checked_add(FDT_WORD_BYTES)
                .expect("DTB header offset overflow");
            u32::from_be_bytes(self.bytes[field..end].try_into().unwrap()) as usize
        };
        let structure = word(FDT_OFF_DT_STRUCT);
        let strings = word(FDT_OFF_DT_STRINGS);
        let structure_size = word(FDT_SIZE_DT_STRUCT);
        let structure_end = structure
            .checked_add(structure_size)
            .expect("DTB structure overflow");
        let header = offset
            .checked_sub(FDT_PROPERTY_BYTES)
            .expect("DTB property header underflow");
        let length_field = offset
            .checked_sub(2 * FDT_WORD_BYTES)
            .expect("DTB property length underflow");
        assert_eq!(word(FDT_TOTALSIZE), self.length);
        assert_eq!(word(header), FDT_PROP);
        assert_eq!(word(length_field), old_length);
        assert!(structure <= header && structure_end <= strings && strings <= self.length);
        assert!(word(FDT_OFF_MEM_RSVMAP) < structure);
        assert_eq!(offset & FDT_ALIGN_MASK, 0);

        // Step 2: 计算旧属性值与新属性值按 4 字节向上取整后的占用跨度，推导平移后的区块尺寸与头部字段值。
        let old_padded = old_length
            .checked_add(FDT_ALIGN_MASK)
            .expect("DTB property overflow")
            & !FDT_ALIGN_MASK;
        let new_padded = value
            .len()
            .checked_add(FDT_ALIGN_MASK)
            .expect("DTB replacement overflow")
            & !FDT_ALIGN_MASK;
        let old_end = offset
            .checked_add(old_padded)
            .expect("DTB old property overflow");
        let new_end = offset
            .checked_add(new_padded)
            .expect("DTB new property overflow");
        assert!(old_end <= structure_end);
        let resized = |size: usize| {
            size.checked_sub(old_padded)
                .and_then(|n| n.checked_add(new_padded))
                .expect("DTB resized block overflow")
        };
        let total = resized(self.length);
        let structure_size = resized(structure_size);
        let strings = resized(strings);
        assert!(total <= self.bytes.len());
        let value_end = offset.checked_add(value.len()).expect("DTB value overflow");
        let fields = [
            (
                FDT_TOTALSIZE,
                u32::try_from(total).expect("DTB total exceeds u32"),
            ),
            (
                FDT_OFF_DT_STRINGS,
                u32::try_from(strings).expect("DTB strings exceeds u32"),
            ),
            (
                FDT_SIZE_DT_STRUCT,
                u32::try_from(structure_size).expect("DTB structure exceeds u32"),
            ),
            (
                length_field,
                u32::try_from(value.len()).expect("DTB property exceeds u32"),
            ),
        ];

        // Step 3: 平移后续字节，写入新属性值与对齐填充零字节，最后回写更新后的大端序头部字段。
        self.bytes.copy_within(old_end..self.length, new_end);
        self.bytes[offset..value_end].copy_from_slice(value);
        self.bytes[value_end..new_end].fill(0);
        for (field, value) in fields {
            let end = field
                .checked_add(FDT_WORD_BYTES)
                .expect("DTB header write overflow");
            self.bytes[field..end].copy_from_slice(&value.to_be_bytes());
        }
        self.length = total;
    }

    /// 在指定节点 `path` 中设置属性 `name` 的值为 `value`（若属性不存在则先插入空属性再替换）。
    fn put_property(&mut self, path: &str, name: &str, value: &[u8]) {
        let (offset, length) = self.prepare_property(path, name);
        self.replace_property(offset, length, value);
    }

    /// 使用 `FDT_NOP`（`4`）覆盖指定节点 `path` 下的属性 `name`（含属性头与 4 字节对齐的值区），
    /// 在不改变其它节点和属性偏移的情况下将该属性从逻辑树中抹除。
    fn remove_property(&mut self, path: &str, name: &str) {
        let range = {
            let tree = fdt::Fdt::new(&self.bytes[..self.length]).expect("actual DTB invalid");
            let node = tree.find_node(path).expect("actual DTB node absent");
            node.property(name).map(|property| {
                let value = property.value.as_ptr() as usize - self.bytes.as_ptr() as usize;
                value - FDT_PROPERTY_BYTES
                    ..(value + property.value.len() + FDT_ALIGN_MASK) & !FDT_ALIGN_MASK
            })
        };
        if let Some(range) = range {
            for token in self.bytes[range].chunks_exact_mut(FDT_WORD_BYTES) {
                token.copy_from_slice(&FDT_NOP.to_be_bytes());
            }
        }
    }
}

/// 调用平台发现入口从给定的 DTB 字节切片 `bytes` 与指定主核硬件编号 `bsp` 重新解析启动资源，
/// 将结果就地写入调用方提供的独占槽 `output` 并返回可变引用，避免修改全局 `boot::info()` 或在栈上拷贝大结构体。
fn parse<'a>(
    bytes: &[u8],
    output: &'a mut MaybeUninit<boot::BootInfo>,
    bsp: usize,
) -> &'a mut boot::BootInfo {
    let live = boot::info();
    platform::ktest_discover_tree(bytes, live.raw_args, bsp, live.dtb.start, output)
        .expect("actual DTB resource parse failed")
}

/// 逐项断言重新解析出的 `actual` 与基准 `expected` 中所有由 DTB 决定的硬件资源描述完全一致。
fn same_resources(
    context: &mut TestContext<'_>,
    actual: &boot::BootInfo,
    expected: &boot::BootInfo,
) {
    context.expect_eq(actual.cpu_count, expected.cpu_count);
    context.expect_eq(actual.platform, expected.platform);
    context.expect_eq(actual.initrd, expected.initrd);
    context.expect_eq(actual.uart.reg_io_width, expected.uart.reg_io_width);
    context.expect_eq(actual.uart.reg_shift, expected.uart.reg_shift);
    context.expect_eq(actual.uart.clock_frequency, expected.uart.clock_frequency);
    context.expect_eq(actual.uart.baud, expected.uart.baud);
    context.expect_eq(actual.uart.dw_apb, expected.uart.dw_apb);
    context.expect_eq(
        actual.interrupts.source_count,
        expected.interrupts.source_count,
    );
    context.expect_eq(actual.board.ccache, expected.board.ccache);
    context.expect_eq(actual.board.cache_line_size, expected.board.cache_line_size);
    for (left, right) in actual
        .board
        .liointc
        .iter()
        .zip(expected.board.liointc.iter())
    {
        context.expect_eq(left.registers, right.registers);
        context.expect_eq(left.isr, right.isr);
        context.expect_eq(left.parent_interrupt, right.parent_interrupt);
        context.expect_eq(left.parent_map, right.parent_map);
    }
    context.expect_eq(actual.hardware_ids, expected.hardware_ids);
    context.expect_eq(actual.memory, expected.memory);
    context.expect_eq(actual.memory_count, expected.memory_count);
    context.expect_eq(actual.dtb, expected.dtb);
    context.expect_eq(actual.rtc, expected.rtc);
    context.expect_eq(actual.interrupts.uart, expected.interrupts.uart);
    context.expect_eq(actual.interrupts.controller, expected.interrupts.controller);
    context.expect_eq(
        actual.interrupts.parent_controller,
        expected.interrupts.parent_controller,
    );
    context.expect_eq(
        actual.interrupts.uart_source,
        expected.interrupts.uart_source,
    );
    context.expect_eq(
        actual.interrupts.plic_contexts,
        expected.interrupts.plic_contexts,
    );
    context.expect_eq(actual.pci.configuration, expected.pci.configuration);
    context.expect_eq(actual.pci.range_count, expected.pci.range_count);
    for (left, right) in actual.pci.ranges.iter().zip(&expected.pci.ranges) {
        context.expect_eq(left.flags, right.flags);
        context.expect_eq(left.bus_start, right.bus_start);
        context.expect_eq(left.cpu, right.cpu);
    }
    #[cfg(target_arch = "riscv64")]
    context.expect_eq(actual.bootargs(), expected.bootargs());
    context.expect_eq(actual.counter_frequency, expected.counter_frequency);
}

// 在注册测试套件前通过宏参数注入平台专属用例（如 QEMU virt 的 RTC 校验），使通用用例与平台用例共享同一模块路径。
macro_rules! firmware_cases {
    ($($qemu_rtc:tt)*) => {
#[::ktest::suite(context = "Boot", fixture = Dtb, tags("boot", "firmware"), timeout_ms = 10000)]
pub(crate) mod cases {
    use super::*;

    /// 测试目标：验证固件传入的原始寄存器参数、BSP 硬件核编号与 DTB 重新解析所得结果完全一致。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 启动阶段保存的全局 `boot::info()` 快照与复制出的 `Dtb` 夹具副本。
    /// - When: 使用当前 BSP 硬件 ID 重新解析 DTB 副本。
    /// - Then: 解析出的硬件资源、原始参数数组 `raw_args`、BSP 硬件 ID（固定排在 `hardware_ids[0]`）与全局快照一致；
    ///   在 RISC-V 64 下同时校验 `a0` 为 hartid、`a1` 为 DTB 物理起始地址。
    #[test]
    fn raw_arguments(context: &mut TestContext<'_>, fixture: &mut Dtb) {
        let live = boot::info();
        let parsed = parse(&fixture.bytes[..fixture.length], fixture.actual, live.bsp_hardware_id);
        same_resources(context, &parsed, live);
        context.expect_eq(parsed.raw_args, live.raw_args);
        context.expect_eq(parsed.bsp_hardware_id, live.bsp_hardware_id);
        context.expect_eq(parsed.hardware_ids[0], live.bsp_hardware_id);
        #[cfg(target_arch = "riscv64")]
        {
            context.expect_eq(live.raw_args[0], live.bsp_hardware_id);
            context.expect_eq(live.raw_args[1], live.dtb.start);
        }
        assert_eq!(arch::boot_hardware_id(live.raw_args), live.bsp_hardware_id);
        context.diagnostic(format_args!(
            "firmware bsp={} args={:#x},{:#x},{:#x},{:#x} dtb={:#x}..{:#x}",
            live.bsp_hardware_id,
            live.raw_args[0],
            live.raw_args[1],
            live.raw_args[2],
            live.raw_args[3],
            live.dtb.start,
            live.dtb.end
        ));
    }

    /// 测试目标：验证物理 RAM 区间、固件保留区（含内核镜像、DTB、initrd）、RTC 与 PCI 地址窗口的解析完整性与互不越界不变量。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 全局 `boot::info()` 快照与重新解析的 `BootInfo`。
    /// - When: 检查内核镜像、DTB、可选 `initrd` 与 RAM 区间、保留区列表的包含关系，以及 RTC 与 RAM 的重叠情况。
    /// - Then: 内核与 `initrd` 完全位于有效 RAM 区间内且均已登记在 `reserved` 表中；重新解析的所有保留区及 `no_map` 属性
    ///   均包含在全局保留表中；RTC MMIO 窗口与任何 RAM 区间均不重叠。
    #[test]
    fn resource_regions(context: &mut TestContext<'_>, fixture: &mut Dtb) {
        let live = boot::info();
        let parsed = parse(&fixture.bytes[..fixture.length], fixture.actual, live.bsp_hardware_id);
        same_resources(context, &parsed, live);

        let kernel = boot::kernel_region();
        assert!(live.memory[..live.memory_count]
            .iter()
            .any(|r| r.start <= kernel.start && kernel.end <= r.end));
        assert!(live.reserved[..live.reserved_count].contains(&kernel));
        assert!(live.reserved[..live.reserved_count].contains(&live.dtb));
        if let Some(initrd) = live.initrd {
            assert!(live.reserved[..live.reserved_count].contains(&initrd));
            assert!(live.memory[..live.memory_count]
                .iter()
                .any(|ram| ram.start <= initrd.start && initrd.end <= ram.end));
        }
        assert!(parsed.reserved[..parsed.reserved_count]
            .iter()
            .zip(&parsed.reserved_no_map[..parsed.reserved_count])
            .all(|(region, no_map)| live.reserved[..live.reserved_count]
                .iter()
                .zip(&live.reserved_no_map[..live.reserved_count])
                .any(|(actual, actual_no_map)| region == actual && no_map == actual_no_map)));

        for region in &live.memory[..live.memory_count] {
            assert!(region.start < region.end);
            context.diagnostic(format_args!(
                "RAM start={:#x} end={:#x}",
                region.start, region.end
            ));
        }
        for (region, no_map) in live.reserved[..live.reserved_count]
            .iter()
            .zip(&live.reserved_no_map[..live.reserved_count])
        {
            context.diagnostic(format_args!(
                "RESERVED start={:#x} end={:#x} no_map={}",
                region.start, region.end, no_map
            ));
        }
        context.diagnostic(format_args!(
            "PCI config={:#x}..{:#x} windows={}",
            live.pci.configuration.start, live.pci.configuration.end, live.pci.range_count
        ));
        assert!(!live.memory[..live.memory_count]
            .iter()
            .any(|ram| ram.overlaps(live.rtc)));
        context.diagnostic(format_args!(
            "RTC start={:#x} end={:#x}",
            live.rtc.start, live.rtc.end
        ));
        for range in &live.pci.ranges[..live.pci.range_count] {
            context.diagnostic(format_args!(
                "PCI WINDOW flags={:#x} bus={:#x} physical={:#x}..{:#x}",
                range.flags, range.bus_start, range.cpu.start, range.cpu.end
            ));
        }
    }

    /// 测试目标：验证各硬件平台（VisionFive2 / Ls2k1000la / QemuVirt）的 CPU 拓扑映射、中断控制器与串口参数解析正确。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 全局 `boot::info()` 与重新解析的 `BootInfo`。
    /// - When: 核对逻辑核号到硬件核号的无重复映射，并根据平台类型检查特定外设参数。
    /// - Then: 每个逻辑核的 `CpuLocal::hardware_id` 与 `BootInfo::hardware_ids` 一一对应且无重复；
    ///   VisionFive2 具备 64 字节缓存行、32 位宽寄存器偏移的 DW-APB UART 及合法 hartid（1..=4）；
    ///   Ls2k1000la 具备双 bank LIOINTC 路由与足够宽的寄存器窗口；QemuVirt 无板级 L2 Cache 控制器。
    #[test]
    fn platform_resources(context: &mut TestContext<'_>, fixture: &mut Dtb) {
        let live = boot::info();
        let parsed = parse(&fixture.bytes[..fixture.length], fixture.actual, live.bsp_hardware_id);
        same_resources(context, &parsed, live);

        for (logical, hardware) in live.hardware_ids[..live.cpu_count].iter().enumerate() {
            context.expect_eq(crate::cpu::get(logical).hardware_id, *hardware);
            assert!(!live.hardware_ids[..logical].contains(hardware));
        }
        match live.platform {
            boot::Platform::VisionFive2 => {
                assert!(live.hardware_ids[..live.cpu_count]
                    .iter()
                    .all(|id| (1..=4).contains(id)));
                context.expect_eq(live.board.cache_line_size, 64);
                context.expect_eq(live.uart.reg_io_width, 4);
                context.expect_eq(live.uart.reg_shift, 2);
                assert!(live.uart.dw_apb);
                assert!(live.interrupts.source_count <= boot::MAX_INTERRUPT_SOURCES as u32);
            }
            boot::Platform::Ls2k1000la => {
                assert!(live.hardware_ids[..live.cpu_count].iter().all(|id| *id < 2));
                for (bank, resource) in live.board.liointc.iter().enumerate() {
                    context.expect_eq(resource.parent_interrupt, bank as u32 + 2);
                    assert!(0x38 <= resource.registers.end - resource.registers.start);
                }
            }
            boot::Platform::QemuVirt => {
                assert_eq!(live.board.ccache, boot::Region::default());
            }
        }
    }

    /// 测试目标：验证设备树 `/chosen/bootargs` 命令行长度在容量上限（4095 字节）时完整保留，超出上限（4096 字节）时被拒绝且缓冲区清零。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 在 `Dtb` 副本中定位或插入 `/chosen/bootargs` 属性。
    /// - When: 将 `bootargs` 替换为长度为 `length`（4095 或 4096）加末尾 `'\0'` 的字符串，调用 `ktest_discover_tree` 解析。
    /// - Then: 当 `length == 4095` 时解析成功，内容、长度及末尾 NUL 字节匹配；当 `length == 4096` 时返回错误，
    ///   且输出槽中的 `bootargs` 保持全零状态。
    #[test]
    #[case("maximum", 4095usize)]
    #[case("overflow", 4096usize)]
    fn bootargs_capacity(context: &mut TestContext<'_>, fixture: &mut Dtb, length: usize) {
        let live = boot::info();
        let (offset, old_length) = fixture.prepare_bootargs();
        let mut command = [0u8; boot::MAX_BOOTARGS_BYTES + 1];
        for (index, byte) in command[..length].iter_mut().enumerate() {
            *byte = b'a' + (index % 26) as u8;
        }
        let value_length = length.checked_add(1).expect("bootargs length overflow");
        fixture.replace_property(offset, old_length, &command[..value_length]);
        assert!(platform::ktest_validate(&fixture.bytes[..fixture.length]).is_ok());

        let result = platform::ktest_discover_tree(
            &fixture.bytes[..fixture.length],
            live.raw_args,
            live.bsp_hardware_id,
            live.dtb.start,
            fixture.actual,
        );
        if length < boot::MAX_BOOTARGS_BYTES {
            let parsed = result.expect("capacity-sized bootargs rejected");
            context.expect_eq(parsed.bootargs(), &command[..length]);
            context.expect_eq(parsed.bootargs_len, length);
            context.expect_eq(parsed.bootargs[length], 0);
        } else {
            assert!(result.is_err(), "oversized bootargs accepted");
            // SAFETY:
            // 1. `ktest_discover_tree` 在检查 `bootargs` 之前已将独占槽 `fixture.actual` 完整初始化为默认 `BootInfo`；
            // 2. `result` 已在上一行断言为 `Err`，不存在任何指向 `fixture.actual` 的活动可变引用。
            let parsed = unsafe { fixture.actual.assume_init_ref() };
            context.expect_eq(parsed.bootargs_len, 0);
            assert!(parsed.bootargs.iter().all(|byte| *byte == 0));
        }
    }

    /// 测试目标：验证物理内存（RAM）区间表在达到容量上限（32 段）时正常合并/接纳，超出上限（33 段）时被解析器拒绝。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 找出包含内核镜像的 RAM 段，并计算内核、DTB 与 `initrd` 占用的受保护物理范围。
    /// - When: 在避开受保护范围的头部或尾部切分出 `extra` 个 4 KiB 独立物理页段，替换 DTB 中 `memory` 节点的 `reg` 属性，
    ///   使总 RAM 段数达到 `count`（32 或 33），随后重新解析。
    /// - Then: 当 `count == 32` 时解析成功，各 RAM 段互不重叠、总字节数守恒且仍完整覆盖内核镜像；
    ///   当 `count == 33` 时因超出 `MAX_MEMORY_REGIONS` 而返回错误。
    #[test]
    #[case("32_ranges", 32usize)]
    #[case("33_ranges", 33usize)]
    fn ram_capacity(context: &mut TestContext<'_>, fixture: &mut Dtb, count: usize) {
        let live = boot::info();
        let kernel = boot::kernel_region();
        let ram = *live.memory[..live.memory_count].iter()
            .find(|ram| ram.start <= kernel.start && kernel.end <= ram.end)
            .expect("kernel RAM absent");
        let extra = count.checked_sub(live.memory_count).expect("RAM input already exceeds case");
        let split_bytes = extra.checked_mul(4096).expect("RAM split overflow");
        let mut protected_start = kernel.start;
        let mut protected_end = kernel.end;
        for region in core::iter::once(live.dtb).chain(live.initrd) {
            if ram.overlaps(region) {
                protected_start = protected_start.min(region.start);
                protected_end = protected_end.max(region.end);
            }
        }
        let head = split_bytes <= protected_start.checked_sub(ram.start).expect("RAM start overflow");
        let split = if head {
            ram.start.checked_add(split_bytes).expect("RAM head overflow")
        } else {
            assert!(split_bytes <= ram.end.checked_sub(protected_end).expect("RAM end overflow"));
            ram.end.checked_sub(split_bytes).expect("RAM tail overflow")
        };

        // Step 1: 构造包含 `extra` 个额外 4 KiB 分段的 `memory/reg` 属性字节序列并写回 DTB 副本。
        let mut replacement = [0u8; (boot::MAX_MEMORY_REGIONS + 1) * 16];
        let (offset, old_length, replacement_length) = {
            let tree = fdt::Fdt::new(&fixture.bytes[..fixture.length]).expect("actual DTB invalid");
            let (node, tuple) = tree.all_nodes()
                .filter(|node| node.property("device_type").and_then(|p| p.as_str()) == Some("memory"))
                .find_map(|node| {
                    node.raw_reg()?.find(|tuple| {
                        let address = tuple.address.iter().fold(0usize, |n, b| (n << 8) | usize::from(*b));
                        let size = tuple.size.iter().fold(0usize, |n, b| (n << 8) | usize::from(*b));
                        address == ram.start && size == ram.end - ram.start
                    }).map(|tuple| (node, tuple))
                }).expect("kernel RAM reg absent");
            let property = node.property("reg").expect("RAM reg absent");
            let address_bytes = tuple.address.len();
            let size_bytes = tuple.size.len();
            assert!(matches!(address_bytes, 4 | 8) && matches!(size_bytes, 4 | 8));
            let stride = address_bytes.checked_add(size_bytes).expect("RAM tuple overflow");
            let prefix = tuple.address.as_ptr() as usize - property.value.as_ptr() as usize;
            let suffix = prefix.checked_add(stride).expect("RAM suffix overflow");
            let growth = extra.checked_mul(stride).expect("RAM property growth overflow");
            let length = property.value.len().checked_add(growth).expect("RAM property overflow");
            assert!(length <= replacement.len());
            replacement[..prefix].copy_from_slice(&property.value[..prefix]);
            let mut cursor = prefix;
            for index in 0..=extra {
                let region = if head && index == extra {
                    boot::Region { start: split, end: ram.end }
                } else if !head && index == 0 {
                    boot::Region { start: ram.start, end: split }
                } else {
                    let page = if head { index } else { index - 1 };
                    let base = if head { ram.start } else { split };
                    let start = base.checked_add(page.checked_mul(4096).expect("RAM page overflow"))
                        .expect("RAM address overflow");
                    boot::Region { start, end: start.checked_add(4096).expect("RAM size overflow") }
                };
                for (value, width) in [(region.start, address_bytes), (region.end - region.start, size_bytes)] {
                    assert!(width == 8 || u32::try_from(value).is_ok());
                    let end = cursor.checked_add(width).expect("RAM cell overflow");
                    replacement[cursor..end].copy_from_slice(&(value as u64).to_be_bytes()[8 - width..]);
                    cursor = end;
                }
            }
            replacement[cursor..length].copy_from_slice(&property.value[suffix..]);
            (
                property.value.as_ptr() as usize - fixture.bytes.as_ptr() as usize,
                property.value.len(),
                length,
            )
        };
        fixture.replace_property(offset, old_length, &replacement[..replacement_length]);
        assert!(platform::ktest_validate(&fixture.bytes[..fixture.length]).is_ok());

        // Step 2: 重新运行平台设备树发现流程，校验 32 段上限通过与 33 段超限拒绝。
        let result = platform::ktest_discover_tree(
            &fixture.bytes[..fixture.length], live.raw_args, live.bsp_hardware_id,
            live.dtb.start, fixture.actual,
        );
        if count <= boot::MAX_MEMORY_REGIONS {
            let parsed = result.expect("legal RAM split rejected");
            context.expect_eq(parsed.memory_count, count);
            assert!(parsed.memory[..count].iter().any(|r| r.start <= kernel.start && kernel.end <= r.end));
            for (index, region) in parsed.memory[..count].iter().enumerate() {
                assert!(region.start < region.end);
                assert!(live.memory[..live.memory_count].iter()
                    .any(|old| old.start <= region.start && region.end <= old.end));
                assert!(!parsed.memory[..index].iter().any(|old| old.overlaps(*region)));
            }
            context.expect_eq(
                parsed.memory[..count].iter().map(|r| r.end - r.start).sum::<usize>(),
                live.memory[..live.memory_count].iter().map(|r| r.end - r.start).sum::<usize>(),
            );
        } else {
            assert!(result.is_err(), "oversized RAM inventory accepted");
        }
    }

    /// 测试目标：验证普通保留区与 `no-map` 保留区共享 `MAX_RESERVED_REGIONS` 容量上限，且超限追加失败时具有强原子性（不破坏已有记录）。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 清空 `actual` 的保留区计数，并在可用 RAM 中规划 `MAX_RESERVED_REGIONS + 1` 个连续 4 KiB 页。
    /// - When: 交替调用 `reserve` 与 `reserve_no_map` 填满全部 `MAX_RESERVED_REGIONS` 个槽位，随后尝试再插入第 `MAX + 1` 个区域。
    /// - Then: 前 `MAX_RESERVED_REGIONS` 次插入全部成功且属性一致；超限的 `reserve` 与 `reserve_no_map` 均返回 `Err`，
    ///   且保留区计数、区间数组与 `no_map` 标记数组保持原状不被修改。
    #[test]
    #[case("ordinary_first", false)]
    #[case("no_map_first", true)]
    fn reservation_capacity(context: &mut TestContext<'_>, fixture: &mut Dtb, first_no_map: bool) {
        let live = boot::info();
        let actual = parse(&fixture.bytes[..fixture.length], fixture.actual, live.bsp_hardware_id);
        let expected = parse(&fixture.bytes[..fixture.length], fixture.expected, live.bsp_hardware_id);
        actual.reserved_count = 0;
        let slots = boot::MAX_RESERVED_REGIONS.checked_add(1).expect("reservation count overflow");
        let bytes = slots.checked_mul(4096).expect("reservation size overflow");
        let ram = live.memory[..live.memory_count].iter()
            .find(|ram| bytes <= ram.end - ram.start).expect("reservation RAM absent");
        for index in 0..boot::MAX_RESERVED_REGIONS {
            let start = ram.start.checked_add(index.checked_mul(4096).expect("reservation offset overflow"))
                .expect("reservation address overflow");
            let region = boot::Region { start, end: start.checked_add(4096).expect("reservation end overflow") };
            let no_map = (index & 1 == 0) == first_no_map;
            if no_map {
                actual.reserve_no_map(region).expect("legal no-map reservation rejected");
            } else {
                actual.reserve(region).expect("legal reservation rejected");
            }
            expected.reserved[index] = region;
            expected.reserved_no_map[index] = no_map;
            context.expect_eq(actual.reserved_count, index + 1);
        }
        expected.reserved_count = boot::MAX_RESERVED_REGIONS;
        context.expect_eq(&actual.reserved[..], &expected.reserved[..]);
        context.expect_eq(&actual.reserved_no_map[..], &expected.reserved_no_map[..]);

        let start = ram.start.checked_add(boot::MAX_RESERVED_REGIONS.checked_mul(4096)
            .expect("last reservation offset overflow")).expect("last reservation address overflow");
        let overflow = boot::Region {
            start, end: start.checked_add(4096).expect("last reservation end overflow"),
        };
        assert!(actual.reserve(overflow).is_err());
        assert!(actual.reserve_no_map(overflow).is_err());
        context.expect_eq(actual.reserved_count, expected.reserved_count);
        context.expect_eq(&actual.reserved[..], &expected.reserved[..]);
        context.expect_eq(&actual.reserved_no_map[..], &expected.reserved_no_map[..]);
    }

    /// 测试目标：验证动态内存预留分配器能够跨乱序但物理相邻的 RAM 段分配连续区域，但不会跨越中间存在物理空洞的两个 RAM 段。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 构造包含两个 4 KiB RAM 段的 `BootInfo`（逆序存放）：当 `adjacent == true` 时两段物理紧邻，
    ///   当 `adjacent == false` 时两段之间隔开 1 个 4 KiB 空洞。
    /// - When: 请求分配长度为 1.5 页（6 KiB）、按页对齐的动态保留区。
    /// - Then: 物理相邻时成功合并并返回跨两段的区间 `base..base + size`；存在物理空洞时返回 `None`。
    #[test]
    #[case("adjacent", true)]
    #[case("hole", false)]
    fn dynamic_ram_boundaries(
        context: &mut TestContext<'_>,
        fixture: &mut Dtb,
        adjacent: bool,
    ) {
        let page = crate::mm::PAGE_SIZE;
        let base = (boot::kernel_region().end + page - 1) & !(page - 1);
        let base = base + 16 * page;
        let info = parse(
            &fixture.bytes[..fixture.length], fixture.actual, boot::info().bsp_hardware_id,
        );
        info.reserved_count = 0;
        info.memory_count = 2;
        let second = base + if adjacent { page } else { 2 * page };
        info.memory[..2].copy_from_slice(&[
            boot::Region { start: second, end: second + page },
            boot::Region { start: base, end: base + page },
        ]);

        let size = page + page / 2;
        let selected = platform::ktest_find_reserved_region(
            info, size, page, boot::Region { start: base, end: base + 3 * page }, false,
        ).expect("dynamic RAM selection failed");
        let expected = adjacent.then_some(boot::Region { start: base, end: base + size });
        context.expect_eq(selected, expected);
    }

    /// 测试目标：验证动态预留分配器自动避开内核镜像、DTB、`initrd` 以及已登记的保留区间，按低地址优先顺序依次分配空闲窗口。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 单段 RAM 覆盖 `kernel.start..base + 8 * page`，其中 `base..base + 3 * page` 已被 DTB、`initrd` 和已有保留区占用。
    /// - When: 连续两次调用 `ktest_find_reserved_region` 申请 1 页大小的动态保留区，并将第一次结果登记到 `reserved` 表。
    /// - Then: 第一次分配返回紧随占用区之后的第 4 页（`base + 3*page..base + 4*page`），第二次返回第 5 页（`base + 4*page..base + 5*page`）。
    #[test]
    fn dynamic_occupied_regions(context: &mut TestContext<'_>, fixture: &mut Dtb) {
        let page = crate::mm::PAGE_SIZE;
        let kernel = boot::kernel_region();
        let base = (kernel.end + page - 1) & !(page - 1);
        let info = parse(
            &fixture.bytes[..fixture.length], fixture.actual, boot::info().bsp_hardware_id,
        );
        let window = boot::Region { start: kernel.start, end: base + 8 * page };
        info.memory_count = 1;
        info.memory[0] = window;
        info.reserved_count = 0;
        info.dtb = boot::Region { start: base, end: base + page };
        info.initrd = Some(boot::Region { start: base + page, end: base + 2 * page });
        info.reserve(info.dtb).unwrap();
        info.reserve(info.initrd.unwrap()).unwrap();
        info.reserve(boot::Region { start: base + 2 * page, end: base + 3 * page }).unwrap();

        let first = platform::ktest_find_reserved_region(info, page, page, window, false)
            .expect("dynamic reservation failed").expect("dynamic reservation absent");
        context.expect_eq(first, boot::Region { start: base + 3 * page, end: base + 4 * page });
        info.reserve(first).unwrap();
        let second = platform::ktest_find_reserved_region(info, page, page, window, false)
            .expect("second dynamic reservation failed");
        context.expect_eq(second, Some(boot::Region { start: base + 4 * page, end: base + 5 * page }));
    }

    /// 测试目标：验证当某物理页内存在 `no-map` 保留区（或本次请求本身为 `no-map`）时，同一物理页内的剩余字节不可被混用，必须整页避开。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 在第 0 页内的 `base + 0x801..base + 0x900` 登记一段保留区（根据 `existing_no_map` 设为 `no-map` 或普通保留）。
    /// - When: 尝试在同页的 `base + 0x940..base + 0xb40` 窗口内申请一段属性相反（`!existing_no_map`）的 64 字节区域；
    ///   随后将搜索窗口扩大到 `base + 2 * page` 再次申请。
    /// - Then: 限制在同一页内时因页级映射属性冲突返回 `None`；允许跨页后跳过整个第 0 页，返回第 1 页起始处的 `base + page..base + page + 64`。
    #[test]
    #[case("existing_no_map", true)]
    #[case("requested_no_map", false)]
    fn dynamic_no_map_pages(
        context: &mut TestContext<'_>,
        fixture: &mut Dtb,
        existing_no_map: bool,
    ) {
        let page = crate::mm::PAGE_SIZE;
        let base = (boot::kernel_region().end + page - 1) & !(page - 1);
        let base = base + 16 * page;
        let info = parse(
            &fixture.bytes[..fixture.length], fixture.actual, boot::info().bsp_hardware_id,
        );
        info.memory_count = 1;
        info.memory[0] = boot::Region { start: base, end: base + 4 * page };
        info.reserved_count = 0;
        let occupied = boot::Region { start: base + 0x801, end: base + 0x900 };
        if existing_no_map {
            info.reserve_no_map(occupied).unwrap();
        } else {
            info.reserve(occupied).unwrap();
        }

        let blocked = platform::ktest_find_reserved_region(
            info, 64, 64, boot::Region { start: base + 0x940, end: base + 0xb40 },
            !existing_no_map,
        ).expect("same-page selection failed");
        context.expect_eq(blocked, None);

        let selected = platform::ktest_find_reserved_region(
            info, 64, 64, boot::Region { start: base + 0x940, end: base + 2 * page },
            !existing_no_map,
        ).expect("next-page selection failed");
        context.expect_eq(selected, Some(boot::Region { start: base + page, end: base + page + 64 }));
    }

    $($qemu_rtc)*

    /// 测试目标：验证 DTB 边界校验器能够拒绝任何被截断的不完整字节前缀。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 校验通过的完整 DTB 副本（长度为 `fixture.length`）。
    /// - When: 遍历所有严格小于 `fixture.length` 的截断长度 `0..fixture.length`，依次调用 `platform::ktest_validate`。
    /// - Then: 所有截断前缀均返回 `Err`。
    #[test]
    fn truncated_prefixes(_: &mut TestContext<'_>, fixture: &mut Dtb) {
        assert!(platform::ktest_validate(&fixture.bytes[..fixture.length]).is_ok());
        for end in 0..fixture.length {
            assert!(
                platform::ktest_validate(&fixture.bytes[..end]).is_err(),
                "accepted truncated DTB prefix {}",
                end
            );
        }
    }

    /// 测试目标：验证当 FDT 头部中的任一区块偏移或大小字段损坏为越界值（`u32::MAX`）时，`ktest_validate` 均能准确拒绝。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 完整的 DTB 副本。
    /// - When: 将指定头部字段 `field`（结构块偏移、字符串块偏移、保留表偏移、字符串块大小或结构块大小）覆写为 `u32::MAX`。
    /// - Then: `platform::ktest_validate` 返回 `Err`。
    #[test]
    #[case("structure_offset", 8usize)]
    #[case("strings_offset", 12usize)]
    #[case("reservation_offset", 16usize)]
    #[case("strings_size", 32usize)]
    #[case("structure_size", 36usize)]
    fn block_boundary(_: &mut TestContext<'_>, fixture: &mut Dtb, field: usize) {
        fixture.bytes[field..field + 4].copy_from_slice(&u32::MAX.to_be_bytes());
        assert!(
            platform::ktest_validate(&fixture.bytes[..fixture.length]).is_err(),
            "accepted damaged DTB block {}",
            field
        );
    }

    /// 测试目标：验证无论固件以第一个硬件核还是最后一个硬件核作为启动主核（BSP），解析器都能将 BSP 置于 `hardware_ids[0]` 并保留全部 CPU。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 原始 DTB 副本与当前系统的硬件核 ID 列表。
    /// - When: 分别传入首个硬件核或末尾硬件核作为 `bsp` 参数调用 `parse`。
    /// - Then: 解析出的 CPU 总数不变，`actual.hardware_ids[0]` 等于传入的 `bsp`，且原硬件核集合完整保留。
    #[test]
    #[case("first", false)]
    #[case("last", true)]
    fn bsp_order(context: &mut TestContext<'_>, fixture: &mut Dtb, last: bool) {
        let live = boot::info();
        let hardware = live.hardware_ids[if last { live.cpu_count - 1 } else { 0 }];
        let actual = parse(&fixture.bytes[..fixture.length], fixture.actual, hardware);
        context.expect_eq(actual.cpu_count, live.cpu_count);
        context.expect_eq(actual.hardware_ids[0], hardware);
        for id in &live.hardware_ids[..live.cpu_count] {
            assert!(actual.hardware_ids[..actual.cpu_count].contains(id));
        }
    }

    /// 测试目标：验证当设备树中的 CPU 硬件编号（`reg`）为大于 `MAX_CPUS` 的稀疏非连续数值时，解析器仍能正确关联中断控制器并重排 BSP。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 定位 DTB 副本中 `/cpus` 下所有 CPU 节点的 `reg` 属性偏移。
    /// - When: 将各 CPU 的硬件编号原地改写为 `MAX_CPUS + 1 + logical * 3` 的稀疏大编号，保持 `phandle` 与中断控制器引用不变，重新解析。
    /// - Then: 解析成功，所有外设资源与预期一致，`hardware_ids[0]` 为指定的稀疏 BSP 编号，且所有硬件 ID 均大于 `MAX_CPUS`。
    #[test]
    #[case("first", false)]
    #[case("last", true)]
    fn sparse_hardware_ids(context: &mut TestContext<'_>, fixture: &mut Dtb, last: bool) {
        let live = boot::info();
        let bsp_logical = if last { live.cpu_count - 1 } else { 0 };
        let expected = parse(&fixture.bytes[..fixture.length], fixture.expected, live.hardware_ids[bsp_logical]);
        let mut offsets = [0usize; boot::MAX_CPUS];
        let mut lengths = [0usize; boot::MAX_CPUS];
        {
            let tree = fdt::Fdt::new(&fixture.bytes[..fixture.length]).expect("actual DTB invalid");
            let cpus = tree.find_node("/cpus").expect("actual CPUs absent");
            for node in cpus.children() {
                if node.property("device_type").and_then(|p| p.as_str()) != Some("cpu") {
                    continue;
                }
                let reg = node.property("reg").expect("actual CPU reg absent");
                let hardware = reg.value.iter().fold(0usize, |value, byte| {
                    (value << 8) | usize::from(*byte)
                });
                if let Some(logical) = live.hardware_ids[..live.cpu_count]
                    .iter().position(|id| *id == hardware)
                {
                    offsets[logical] = reg.value.as_ptr() as usize - fixture.bytes.as_ptr() as usize;
                    lengths[logical] = reg.value.len();
                }
            }
        }

        for logical in 0..live.cpu_count {
            let length = lengths[logical];
            assert!(length == 4 || length == 8);
            let hardware = (boot::MAX_CPUS + 1 + logical * 3) as u64;
            let bytes = hardware.to_be_bytes();
            fixture.bytes[offsets[logical]..offsets[logical] + length]
                .copy_from_slice(&bytes[8 - length..]);
        }
        for hardware in &mut expected.hardware_ids[..expected.cpu_count] {
            let logical = live.hardware_ids[..live.cpu_count]
                .iter().position(|id| *id == *hardware).unwrap();
            *hardware = boot::MAX_CPUS + 1 + logical * 3;
        }

        let bsp = boot::MAX_CPUS + 1 + bsp_logical * 3;
        expected.bsp_hardware_id = bsp;
        let actual = parse(&fixture.bytes[..fixture.length], fixture.actual, bsp);
        same_resources(context, &actual, &expected);
        context.expect_eq(actual.bsp_hardware_id, bsp);
        context.expect_eq(actual.hardware_ids[0], bsp);
        assert!(actual.hardware_ids[..actual.cpu_count].iter().all(|id| boot::MAX_CPUS < *id));
        context.diagnostic(format_args!(
            "sparse CPU parser count={} bsp={} last={}",
            actual.cpu_count, bsp, actual.hardware_ids[actual.cpu_count - 1]
        ));
    }

    /// 测试目标：验证将根节点 `/` 的 `compatible` 属性用 `FDT_NOP` 标记抹除后，其余硬件资源的发现结果不受影响。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 预先解析原始 DTB 副本得到基准快照 `original`。
    /// - When: 定位根节点 `/` 的 `compatible` 属性区间，将其全部 32 位字替换为 `FDT_NOP`（`4`），再次调用 `parse`。
    /// - Then: 重新解析的资源与 `original` 完全一致。
    #[test]
    fn nop_property(context: &mut TestContext<'_>, fixture: &mut Dtb) {
        let live = boot::info();
        let hardware = live.hardware_ids[live.cpu_count - 1];
        let original = parse(&fixture.bytes[..fixture.length], fixture.expected, hardware);
        let (start, end) = {
            let tree = fdt::Fdt::new(&fixture.bytes[..fixture.length]).expect("actual DTB invalid");
            let property = tree
                .find_node("/")
                .and_then(|n| n.property("compatible"))
                .expect("actual root compatible absent");
            let value = property.value.as_ptr() as usize - fixture.bytes.as_ptr() as usize;
            (value - 12, (value + property.value.len() + 3) & !3)
        };
        for token in fixture.bytes[start..end].chunks_exact_mut(4) {
            token.copy_from_slice(&4u32.to_be_bytes());
        }

        let actual = parse(&fixture.bytes[..fixture.length], fixture.actual, hardware);
        same_resources(context, &actual, &original);
    }
}
    };
}

/// RISC-V 64 架构下物理内存直接映射（Direct Map）的 `no-map` 整页扣除与活动对象冲突保护测试套件。
#[cfg(target_arch = "riscv64")]
#[::ktest::suite(context = "Boot", fixture = Dtb, tags("boot", "firmware"), timeout_ms = 10000)]
pub(crate) mod no_map {
    use super::*;

    /// 测试目标：验证乱序、重叠、相邻及跨 RAM 段的 `no-map` 保留区会按 4 KiB 整页并集从内核直接映射区间中扣除，而普通保留区不影响直接映射。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 登记一段 `0..0x28000` 的普通保留区，以及 7 段乱序、非页对齐或相互重叠的 `no-map` 保留区。
    /// - When: 对三个候选 RAM 窗口调用 `arch::direct_map_regions` 计算可建立直接映射的子区间。
    /// - Then: 普通保留区仍保留在直接映射中，所有触及 `no-map` 的物理页均被整页剔除，最终精确产出 3 段页对齐的直映区间。
    #[test]
    fn no_map_union(context: &mut TestContext<'_>, fixture: &mut Dtb) {
        let info = parse(
            &fixture.bytes[..fixture.length],
            fixture.actual,
            boot::info().bsp_hardware_id,
        );
        let base = platform::LINK_LAYOUT.ram_phys_base as usize;
        info.reserved_count = 0;
        info.reserve(boot::Region {
            start: base,
            end: base + 0x28000,
        })
        .unwrap();
        for (start, end) in [
            (0x21001, 0x22fff),
            (0x5001, 0x8000),
            (0x1f001, 0x21001),
            (0x7000, 0xa001),
            (0xb000, 0xc000),
            (0xf001, 0x21000),
            (0xfffff, 0x102001),
        ] {
            info.reserve_no_map(boot::Region {
                start: base + start,
                end: base + end,
            })
            .unwrap();
        }

        let mut actual = [boot::Region::default(); 3];
        let mut count = 0;
        for (start, end) in [(0, 0x10000), (0x20000, 0x28000), (0x100000, 0x102000)] {
            for region in arch::direct_map_regions(
                info,
                boot::Region {
                    start: base + start,
                    end: base + end,
                },
            ) {
                assert!(count < actual.len(), "unexpected direct-map fragment");
                actual[count] = region;
                count += 1;
            }
        }
        context.expect_eq(count, 3);
        context.expect_eq(
            actual,
            [
                boot::Region {
                    start: base,
                    end: base + 0x5000,
                },
                boot::Region {
                    start: base + 0xc000,
                    end: base + 0xf000,
                },
                boot::Region {
                    start: base + 0x23000,
                    end: base + 0x28000,
                },
            ],
        );
    }

    /// 测试目标：验证在最大 RAM 段数（`MAX_MEMORY_REGIONS`）与最大 `no-map` 空洞数（`MAX_RESERVED_REGIONS`）满载分裂下，
    /// 页表预留预算能够覆盖全部碎片及对应的 Sv39 L2/L1/L0 页表页数量。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 构造 `MAX_MEMORY_REGIONS` 个非 2 MiB 对齐的 RAM 段，并在每段内部的 4 MiB 边界处打入 `holes` 个跨边界 `no-map` 空洞。
    /// - When: 调用 `arch::ktest_ram_mapping_budget(info)` 计算映射碎片数与所需页表页总数。
    /// - Then: 碎片总数等于 `MAX_MEMORY_REGIONS + MAX_RESERVED_REGIONS`；页表预算等于 1 个 L2 根表 + 跨及的 L1 表数 + 每个碎片两端所需的 2 个 L0 表。
    #[test]
    fn no_map_mapping_capacity(context: &mut TestContext<'_>, fixture: &mut Dtb) {
        let info = parse(
            &fixture.bytes[..fixture.length],
            fixture.actual,
            boot::info().bsp_hardware_id,
        );
        let base = platform::LINK_LAYOUT.ram_phys_base as usize;
        info.reserved_count = 0;
        info.memory_count = boot::MAX_MEMORY_REGIONS;
        let holes = boot::MAX_RESERVED_REGIONS / boot::MAX_MEMORY_REGIONS;
        assert_eq!(holes * boot::MAX_MEMORY_REGIONS, boot::MAX_RESERVED_REGIONS);
        let bank_bytes = (holes + 1) * 0x400000;
        let stride = bank_bytes.next_power_of_two();
        for bank in 0..info.memory_count {
            let start = base + bank * stride;
            info.memory[bank] = boot::Region {
                start: start + 4096,
                end: start + bank_bytes - 4096,
            };
            for hole in 1..=holes {
                let boundary = start + hole * 0x400000;
                info.reserve_no_map(boot::Region {
                    start: boundary - 4096,
                    end: boundary + 4096,
                })
                .unwrap();
            }
        }

        let (fragments, tables) = arch::ktest_ram_mapping_budget(info);
        context.expect_eq(
            fragments,
            boot::MAX_MEMORY_REGIONS + boot::MAX_RESERVED_REGIONS,
        );
        // 每个映射碎片的两端均非 2 MiB 对齐且落在不同的 2 MiB 槽位，因此每个碎片需要 2 个 L0 页表；同时累加整个直映虚拟地址范围跨及的 L1 页表数。
        let first = arch::phys_to_virt(info.memory[0].start) >> 30;
        let last = arch::phys_to_virt(info.memory[info.memory_count - 1].end - 1) >> 30;
        context.expect_eq(tables, 1 + last - first + 1 + 2 * fragments);
    }

    /// 测试目标：验证启动期活动对象（内核镜像、DTB、`initrd`）所在物理页严禁与 `no-map` 区域共享同一物理页，而紧邻页边界之外的 `no-map` 则允许通过。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 设置内核镜像、非页对齐起始的 DTB（`0x4001..0x5000`）与 `initrd` 的物理区间。
    /// - When:
    ///   1. 在不相交的 `0..0x1001` 登记 `no-map`，校验通过；
    ///   2. 依次在内核、DTB、`initrd` 的末尾最后 1 字节处登记 `no-map`，校验被拒；
    ///   3. 在与 DTB 共享同一起始页的前半页（`0x4000..0x4001`）登记 `no-map`，校验被拒；
    ///   4. 在紧随内核结束页边界之后的 `kernel.end..kernel.end + 1` 登记 `no-map`，校验通过。
    /// - Then: 凡是与活动对象落在同一 4 KiB 物理页内的 `no-map` 均被 `validate_boot_no_map` 拒绝，页边界外的 `no-map` 正常放行。
    #[test]
    fn no_map_live_objects(_: &mut TestContext<'_>, fixture: &mut Dtb) {
        let info = parse(
            &fixture.bytes[..fixture.length],
            fixture.actual,
            boot::info().bsp_hardware_id,
        );
        let base = platform::LINK_LAYOUT.ram_phys_base as usize;
        let kernel = boot::Region {
            start: base + 0xc000,
            end: base + 0xe000,
        };
        info.dtb = boot::Region {
            start: base + 0x4001,
            end: base + 0x5000,
        };
        info.initrd = Some(boot::Region {
            start: base + 0x8000,
            end: base + 0x9000,
        });
        info.reserved_count = 0;
        info.reserve(boot::Region {
            start: base,
            end: base + 0x10000,
        })
        .unwrap();
        info.reserve_no_map(boot::Region {
            start: base,
            end: base + 0x1001,
        })
        .unwrap();
        assert!(arch::validate_boot_no_map(info, kernel).is_ok());

        for target in [kernel, info.dtb, info.initrd.unwrap()] {
            info.reserved_count = 2;
            info.reserve_no_map(boot::Region {
                start: target.end - 1,
                end: target.end,
            })
            .unwrap();
            assert!(arch::validate_boot_no_map(info, kernel).is_err());
        }

        info.reserved_count = 2;
        info.reserve_no_map(boot::Region {
            start: base + 0x4000,
            end: info.dtb.start,
        })
        .unwrap();
        assert!(arch::validate_boot_no_map(info, kernel).is_err());

        info.reserved_count = 2;
        info.reserve_no_map(boot::Region {
            start: kernel.end,
            end: kernel.end + 1,
        })
        .unwrap();
        assert!(arch::validate_boot_no_map(info, kernel).is_ok());
    }
}

#[cfg(blossom_platform = "qemu-virt")]
firmware_cases! {
    /// 测试目标：验证 QEMU virt 平台下当 RTC 设备节点缺失、寄存器地址未对齐、窗口过短或与 RAM 重叠时，平台资源发现流程会拒绝启动。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 定位 DTB 副本中的 RTC 节点（RISC-V 为 `google,goldfish-rtc`，LoongArch64 为 `loongson,ls7a-rtc`）。
    /// - When: 根据 `mode` 破坏 `compatible` 字符串（0）、将基址设为奇数未对齐（1）、将窗口长度缩减为 4 字节（2）或将基址改写到 RAM 起始处（3），重新解析。
    /// - Then: 四种损坏模式下 `platform::ktest_discover_tree` 均返回 `Err`。
    #[test]
    #[case("missing", 0u8)]
    #[case("unaligned", 1u8)]
    #[case("short_window", 2u8)]
    #[case("ram_overlap", 3u8)]
    fn rtc_resource_rejection(_: &mut TestContext<'_>, fixture: &mut Dtb, mode: u8) {
        let live = boot::info();
        #[cfg(target_arch = "riscv64")]
        let binding = "google,goldfish-rtc";
        #[cfg(target_arch = "loongarch64")]
        let binding = "loongson,ls7a-rtc";
        let (offset, length) = {
            let tree = fdt::Fdt::new(&fixture.bytes[..fixture.length]).expect("actual DTB invalid");
            let rtc = tree.find_compatible(&[binding]).expect("actual RTC absent");
            let property = rtc
                .property(if mode == 0 { "compatible" } else { "reg" })
                .expect("actual RTC resource absent");
            (
                property.value.as_ptr() as usize - fixture.bytes.as_ptr() as usize,
                property.value.len(),
            )
        };
        if mode == 0 {
            fixture.bytes[offset] = b'x';
        } else {
            assert_eq!(
                length, 16,
                "fixed virt RTC reg is not two address and two size cells"
            );
            match mode {
                1 => fixture.bytes[offset + 7] |= 1,
                2 => fixture.bytes[offset + 8..offset + 16].copy_from_slice(&4u64.to_be_bytes()),
                3 => fixture.bytes[offset..offset + 8]
                    .copy_from_slice(&(live.memory[0].start as u64).to_be_bytes()),
                _ => unreachable!(),
            }
        }

        assert!(platform::ktest_discover_tree(
            &fixture.bytes[..fixture.length],
            live.raw_args,
            live.bsp_hardware_id,
            live.dtb.start,
            fixture.actual,
        )
        .is_err());
    }
}

#[cfg(not(blossom_platform = "qemu-virt"))]
firmware_cases! {}

/// 设备树 DMA 缓存一致性属性（`dma-coherent` / `dma-noncoherent`）与 `interconnects` 父链解析测试套件。
#[::ktest::suite(context = "Boot", fixture = Dtb, tags("boot", "firmware", "dma-policy"), timeout_ms = 10000)]
pub(crate) mod dma_policy {
    use super::*;

    const PROVIDER: u32 = 0xffff_fffe;
    const CONSUMER: u32 = 0xffff_fffd;

    /// 测试目标：验证当设备节点及其祖先链均未声明任何 DMA 一致性属性时，`ktest_dma_coherent` 返回调用方传入的架构默认值。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 移除根节点 `/` 上的 `dma-coherent`、`dma-noncoherent` 与 `interconnect-names` 属性。
    /// - When: 分别以 `default = false` 和 `default = true` 查询根节点的 DMA 一致性。
    /// - Then: 返回结果与传入的 `default` 参数完全一致。
    #[test]
    fn architecture_defaults(context: &mut TestContext<'_>, fixture: &mut Dtb) {
        for name in ["dma-coherent", "dma-noncoherent", "interconnect-names"] {
            fixture.remove_property("/", name);
        }
        for default in [false, true] {
            context.expect_eq(
                platform::ktest_dma_coherent(&fixture.bytes[..fixture.length], "/", default),
                Ok(default),
            );
        }
    }

    /// 测试目标：验证子节点显式声明的 DMA 属性优先于祖先节点声明，且同一节点同时存在两个属性时 `dma-coherent` 优先。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 根节点 `/` 与子节点 `/chosen`。
    /// - When:
    ///   1. `/` 设为 `dma-coherent`、`/chosen` 设为 `dma-noncoherent`，查询 `/chosen`；
    ///   2. `/` 设为 `dma-noncoherent`、`/chosen` 设为 `dma-coherent`，查询 `/chosen`；
    ///   3. 移除 `/chosen` 上的属性使其继承根节点 `/` 的 `dma-noncoherent`，查询 `/chosen`。
    /// - Then: 依次返回 `Ok(false)`、`Ok(true)`、`Ok(false)`。
    #[test]
    fn closest_explicit_attribute(context: &mut TestContext<'_>, fixture: &mut Dtb) {
        fixture.put_property("/", "dma-coherent", &[]);
        fixture.remove_property("/chosen", "dma-coherent");
        fixture.put_property("/chosen", "dma-noncoherent", &[]);
        context.expect_eq(
            platform::ktest_dma_coherent(&fixture.bytes[..fixture.length], "/chosen", true),
            Ok(false),
        );

        fixture.put_property("/", "dma-noncoherent", &[]);
        fixture.remove_property("/", "dma-coherent");
        fixture.put_property("/chosen", "dma-coherent", &[]);
        context.expect_eq(
            platform::ktest_dma_coherent(&fixture.bytes[..fixture.length], "/chosen", false),
            Ok(true),
        );

        fixture.remove_property("/chosen", "dma-noncoherent");
        fixture.remove_property("/chosen", "dma-coherent");
        context.expect_eq(
            platform::ktest_dma_coherent(&fixture.bytes[..fixture.length], "/chosen", true),
            Ok(false),
        );
    }

    /// 测试目标：验证当节点通过 `interconnect-names` 指定了 `"dma-mem"` 路径时，解析器会跳过前面无关的 interconnect 条目并沿目标 `phandle` 继承一致性属性。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 将 `/cpus` 配置为 `#interconnect-cells = 2`、句柄为 `PROVIDER` 的互连提供者；在 `/chosen` 上设置
    ///   `interconnect-names = "other\0dma-mem\0"`，其中第 0 项为 0 句柄（无参数），第 1 项指向 `PROVIDER` 并携带 2 个参数 cell。
    /// - When: 先将 `/cpus` 设为 `dma-noncoherent`（根节点为 `dma-coherent`）查询 `/chosen`；再将 `/cpus` 改为 `dma-coherent`（根节点为 `dma-noncoherent`）查询 `/chosen`。
    /// - Then: `/chosen` 始终忽略结构父节点 `/` 的属性，准确继承 `dma-mem` 目标节点 `/cpus` 的属性（先 `Ok(false)` 后 `Ok(true)`）。
    #[test]
    fn selected_dma_parent(context: &mut TestContext<'_>, fixture: &mut Dtb) {
        fixture.put_property("/", "dma-coherent", &[]);
        fixture.remove_property("/chosen", "dma-coherent");
        fixture.remove_property("/chosen", "dma-noncoherent");
        fixture.put_property("/cpus", "phandle", &PROVIDER.to_be_bytes());
        fixture.put_property("/cpus", "#interconnect-cells", &2u32.to_be_bytes());
        fixture.remove_property("/cpus", "dma-coherent");
        fixture.put_property("/cpus", "dma-noncoherent", &[]);
        fixture.put_property("/chosen", "interconnect-names", b"other\0dma-mem\0");
        let mut links = [0u8; 16];
        links[4..8].copy_from_slice(&PROVIDER.to_be_bytes());
        links[8..12].copy_from_slice(&3u32.to_be_bytes());
        links[12..].copy_from_slice(&4u32.to_be_bytes());
        fixture.put_property("/chosen", "interconnects", &links);
        context.expect_eq(
            platform::ktest_dma_coherent(&fixture.bytes[..fixture.length], "/chosen", true),
            Ok(false),
        );

        fixture.remove_property("/cpus", "dma-noncoherent");
        fixture.put_property("/cpus", "dma-coherent", &[]);
        fixture.put_property("/", "dma-noncoherent", &[]);
        fixture.remove_property("/", "dma-coherent");
        context.expect_eq(
            platform::ktest_dma_coherent(&fixture.bytes[..fixture.length], "/chosen", false),
            Ok(true),
        );
    }

    /// 测试目标：验证当 `"dma-mem"` 选中的 `interconnects` 属性被截断、指向空句柄、指向不存在的 `phandle` 或提供者的 `#interconnect-cells` 格式非法时，解析器直接报错而非回退到结构父节点。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 根节点 `/` 设为 `dma-coherent`，`/chosen` 通过 `interconnect-names = "dma-mem"` 指向需要 2 个参数 cell 的 `/cpus`（共需 12 字节）。
    /// - When: 依次将 `interconnects` 截断为 0/3/4/8 字节、将目标句柄设为 0、设为不存在的 `PROVIDER - 2`，或将 `/cpus` 的 `#interconnect-cells` 破坏为 1 字节长度。
    /// - Then: 所有损坏情形下 `platform::ktest_dma_coherent` 均返回 `Err`。
    #[test]
    fn selected_parent_errors(context: &mut TestContext<'_>, fixture: &mut Dtb) {
        fixture.put_property("/", "dma-coherent", &[]);
        fixture.remove_property("/chosen", "dma-coherent");
        fixture.remove_property("/chosen", "dma-noncoherent");
        fixture.put_property("/cpus", "phandle", &PROVIDER.to_be_bytes());
        fixture.put_property("/cpus", "#interconnect-cells", &2u32.to_be_bytes());
        fixture.put_property("/chosen", "interconnect-names", b"dma-mem\0");
        let mut link = [0u8; 12];
        link[..4].copy_from_slice(&PROVIDER.to_be_bytes());
        for length in [0, 3, 4, 8] {
            fixture.put_property("/chosen", "interconnects", &link[..length]);
            context.expect_eq(
                platform::ktest_dma_coherent(&fixture.bytes[..fixture.length], "/chosen", false)
                    .is_err(),
                true,
            );
        }

        fixture.put_property("/chosen", "interconnects", &0u32.to_be_bytes());
        context.expect_eq(
            platform::ktest_dma_coherent(&fixture.bytes[..fixture.length], "/chosen", false)
                .is_err(),
            true,
        );

        fixture.put_property("/chosen", "interconnects", &(PROVIDER - 2).to_be_bytes());
        context.expect_eq(
            platform::ktest_dma_coherent(&fixture.bytes[..fixture.length], "/chosen", false)
                .is_err(),
            true,
        );

        fixture.put_property("/chosen", "interconnects", &link);
        fixture.put_property("/cpus", "#interconnect-cells", &[2]);
        context.expect_eq(
            platform::ktest_dma_coherent(&fixture.bytes[..fixture.length], "/chosen", false)
                .is_err(),
            true,
        );
    }

    /// 测试目标：验证当两个节点的 `dma-mem` 互连引用形成环路时解析器能检测并返回错误，而若起始节点自身已有显式 DMA 属性则直接返回而不受后续坏链影响。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 将 `/chosen`（`CONSUMER`）与 `/cpus`（`PROVIDER`）的 `dma-mem` `interconnects` 设为互相指向对方，且两者均无显式 DMA 属性。
    /// - When: 先查询 `/chosen` 的 DMA 一致性；随后在 `/chosen` 上添加显式 `dma-noncoherent` 属性并再次查询。
    /// - Then: 存在无属性环路时返回 `Err`；添加本地显式属性后立即返回 `Ok(false)`。
    #[test]
    fn parent_cycle_and_local_override(context: &mut TestContext<'_>, fixture: &mut Dtb) {
        for path in ["/chosen", "/cpus"] {
            fixture.remove_property(path, "dma-coherent");
            fixture.remove_property(path, "dma-noncoherent");
            fixture.put_property(path, "#interconnect-cells", &0u32.to_be_bytes());
            fixture.put_property(path, "interconnect-names", b"dma-mem\0");
        }
        fixture.put_property("/chosen", "phandle", &CONSUMER.to_be_bytes());
        fixture.put_property("/cpus", "phandle", &PROVIDER.to_be_bytes());
        fixture.put_property("/chosen", "interconnects", &PROVIDER.to_be_bytes());
        fixture.put_property("/cpus", "interconnects", &CONSUMER.to_be_bytes());
        context.expect_eq(
            platform::ktest_dma_coherent(&fixture.bytes[..fixture.length], "/chosen", true)
                .is_err(),
            true,
        );

        fixture.put_property("/chosen", "dma-noncoherent", &[]);
        context.expect_eq(
            platform::ktest_dma_coherent(&fixture.bytes[..fixture.length], "/chosen", true),
            Ok(false),
        );
    }
}
