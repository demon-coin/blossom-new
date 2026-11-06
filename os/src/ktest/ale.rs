//! 非对齐访存异常（Address Alignment Exception, ALE）指令解码与软件模拟器的单元测试。
//!
//! # 模块职责与架构定位
//! 本模块属于 `ktest` 启动期（`Boot` 上下文）自检套件，用于独立验证 `crate::arch::ale` 中的
//! LoongArch64 访存指令解码器（`Instruction::decode`）与字节级软件模拟流水线，无需依赖当前硬件
//! CPU 是否触发物理非对齐异常。
//! - **上游调用者**：`ktest` 启动期测试运行器。
//! - **下游被测模块**：`crate::arch::ale`（指令解码、源操作数读取、按字节访存与用户态上下文提交）。
//!
//! # 核心机制与指令编码格式
//! 模拟器通过 `UserMemory` trait 按字节递增访问用户内存。本测试使用内存模型 `Memory` 模拟 32 字节
//! 窗口及指定地址权限错误，覆盖以下三类 32 位 LoongArch 访存指令格式与四阶段执行流水：
//!
//! ```text
//! 1. 立即数格式 (2RI12, 如 ld.d / st.d / fld.s):
//!    +-------------------+-------------------+---------+---------+
//!    |  opcode (31..22)  |    si12 (21..10)  | rj(9..5)| rd(4..0)|
//!    +-------------------+-------------------+---------+---------+
//!
//! 2. 指针偏移格式 (2RI14, 如 ldptr.w / stptr.d):
//!    +---------------+-----------------------+---------+---------+
//!    | opcode(31..24)|      si14 (23..10)    | rj(9..5)| rd(4..0)|
//!    +---------------+-----------------------+---------+---------+
//!
//! 3. 寄存器索引格式 (3R, 如 ldx.h / stx.d / fldx.d):
//!    +---------------------------+-----------+---------+---------+
//!    |       opcode (31..15)     | rk(14..10)| rj(9..5)| rd(4..0)|
//!    +---------------------------+-----------+---------+---------+
//!
//! 模拟执行四阶段 (emulate):
//!   Instruction::decode(word)        -> 失败则立即返回 Unsupported（不触碰内存、不改 PC）
//!   instruction.source_value(ctx)    -> 读取源通用/浮点寄存器快照（r0 恒为 0）
//!   instruction.access(mem, addr, v) -> 按小端序逐字节读写 [addr .. addr + width]（遇错即停并返回 Fault）
//!   instruction.complete(ctx, value) -> 写回目标寄存器并令 ctx.pc += 4，返回 Emulated
//! ```
//!
//! # 锁获取顺序与并发契约
//! 本模块所有测试均在单核栈上的纯内存桩对象（`Memory` 与 `UserContext`）上同步运行，不涉及任何锁或跨核共享状态。

use crate::arch::ale::{EmulationResult, Instruction, UserMemory};
use crate::arch::UserContext;
use crate::mm::space::VmError;

/// 用于测试非对齐访存模拟器的固定大小用户内存桩。
struct Memory {
    /// 模拟内存窗口映射的起始用户虚拟地址（单位：字节）。
    base: usize,
    /// 固定 32 字节的后备存储数组。
    bytes: [u8; 32],
    /// 可选的故障注入虚拟地址：当访问命中该地址时强制返回 `VmError::Permission`。
    fail_at: Option<usize>,
}

impl Memory {
    /// 在指定的起始虚拟地址 `base` 处创建一个全零初始化的 32 字节模拟内存窗口（可跨越 4KiB 页边界）。
    fn new(base: usize) -> Self {
        Self {
            base,
            bytes: [0; 32],
            fail_at: None,
        }
    }
}

impl UserMemory for Memory {
    /// 读取指定用户虚拟地址处的单个字节；若命中故障注入地址则返回 `VmError::Permission`，越界则返回 `VmError::Unmapped`。
    fn read_byte(&mut self, address: usize) -> Result<u8, VmError> {
        if self.fail_at == Some(address) {
            return Err(VmError::Permission);
        }
        let offset = address.checked_sub(self.base).ok_or(VmError::Unmapped)?;
        self.bytes.get(offset).copied().ok_or(VmError::Unmapped)
    }

    /// 向指定用户虚拟地址写入单个字节；若命中故障注入地址则返回 `VmError::Permission`，越界则返回 `VmError::Unmapped`。
    fn write_byte(&mut self, address: usize, value: u8) -> Result<(), VmError> {
        if self.fail_at == Some(address) {
            return Err(VmError::Permission);
        }
        let offset = address.checked_sub(self.base).ok_or(VmError::Unmapped)?;
        *self.bytes.get_mut(offset).ok_or(VmError::Unmapped)? = value;
        Ok(())
    }
}

/// 按照异常处理路径的标准四阶段顺序（解码 -> 读取源寄存器 -> 逐字节访存 -> 提交上下文并推进 PC）模拟执行一条 32 位指令。
fn emulate(
    context: &mut UserContext,
    memory: &mut Memory,
    address: usize,
    word: u32,
) -> EmulationResult {
    let Some(instruction) = Instruction::decode(word) else {
        return EmulationResult::Unsupported;
    };
    let source = instruction.source_value(context);
    let Ok(value) = instruction.access(memory, address, source) else {
        return EmulationResult::Fault;
    };
    instruction.complete(context, value);
    EmulationResult::Emulated
}

/// 构造 `2RI12` 立即数寻址格式（10 位操作码、非零 12 位立即数与非零基址寄存器 `rj = 3`）的 32 位机器码。
fn immediate(opcode: u32, rd: u32) -> u32 {
    (opcode << 22) | (0xabc << 10) | (3 << 5) | rd
}

/// 构造 `2RI14` 指针偏移寻址格式（8 位操作码、非零 14 位偏移与非零基址寄存器 `rj = 3`）的 32 位机器码。
fn pointer(opcode: u32, rd: u32) -> u32 {
    (opcode << 24) | (0x2345 << 10) | (3 << 5) | rd
}

/// 构造 `3R` 寄存器索引寻址格式（17 位操作码、非零索引寄存器 `rk = 11` 与基址寄存器 `rj = 3`）的 32 位机器码。
fn indexed(opcode: u32, rd: u32) -> u32 {
    (opcode << 15) | (11 << 10) | (3 << 5) | rd
}

#[::ktest::suite(context = "Boot", tags("arch", "ale"))]
pub(crate) mod cases {
    use super::*;

    /// 测试目标：
    /// 验证 64 位双字存储指令 `st.d` 在仅 4 字节对齐（非 8 字节对齐）地址处能按小端序完整写入 8 字节并推进 PC。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：初始化基址为 `0x2000` 的内存窗口，设置 `PC = 0x1000`、`gpr[12] = 0x8877_6655_4433_2211`；
    /// 2. When：在非 8 字节对齐地址 `0x200c` 处模拟执行 `st.d $r12`（`immediate(0xa7, 12)`）；
    /// 3. Then：返回 `Emulated`，偏移 `12..20` 写入对应小端字节，两侧相邻字节保持为 0，且 `PC` 前进至 `0x1004`。
    #[test]
    fn unaligned_store_double() {
        let mut memory = Memory::new(0x2000);
        let mut context = UserContext::new(0x1000, 0);
        context.gpr[12] = 0x8877_6655_4433_2211;

        assert_eq!(
            emulate(&mut context, &mut memory, 0x200c, immediate(0xa7, 12)),
            EmulationResult::Emulated
        );
        assert_eq!(
            &memory.bytes[12..20],
            &0x8877_6655_4433_2211u64.to_le_bytes()
        );
        assert_eq!(context.pc, 0x1004);
        assert_eq!(memory.bytes[11], 0);
        assert_eq!(memory.bytes[20], 0);
    }

    /// 测试目标：
    /// 验证半字（2B）、字（4B）、双字（8B）的有符号与无符号整数加载指令在立即数、指针偏移和索引三种编码格式下的符号扩展与零扩展行为。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：遍历 12 种整数加载指令编码，在非对齐地址 `0x2001` 起填充对应位宽的全 `0xff` 字节；
    /// 2. When：对每种指令调用 `emulate` 写入目标寄存器 `gpr[7]`；
    /// 3. Then：有符号加载将高位符号扩展为全 1（`usize::MAX`），无符号加载将高位清零，且 `PC` 前进至 `0x1004`。
    #[test]
    fn integer_load_extension() {
        let cases = [
            (immediate(0xa1, 7), 2, usize::MAX),
            (immediate(0xa9, 7), 2, 0xffff),
            (immediate(0xa2, 7), 4, usize::MAX),
            (immediate(0xaa, 7), 4, 0xffff_ffff),
            (immediate(0xa3, 7), 8, usize::MAX),
            (pointer(0x24, 7), 4, usize::MAX),
            (pointer(0x26, 7), 8, usize::MAX),
            (indexed(0x7008, 7), 2, usize::MAX),
            (indexed(0x7048, 7), 2, 0xffff),
            (indexed(0x7010, 7), 4, usize::MAX),
            (indexed(0x7050, 7), 4, 0xffff_ffff),
            (indexed(0x7018, 7), 8, usize::MAX),
        ];
        for (word, width, expected) in cases {
            let mut memory = Memory::new(0x2000);
            memory.bytes[1..1 + width].fill(0xff);
            let mut context = UserContext::new(0x1000, 0);

            assert_eq!(
                emulate(&mut context, &mut memory, 0x2001, word),
                EmulationResult::Emulated
            );
            assert_eq!(context.gpr[7], expected);
            assert_eq!(context.pc, 0x1004);
        }
    }

    /// 测试目标：
    /// 验证半字（2B）、字（4B）、双字（8B）整数存储指令在立即数、指针偏移和索引格式下仅写出源寄存器对应低位字节，不污染相邻内存。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：设置 `gpr[7] = 0xfedc_ba98_7654_3210`，遍历 8 种不同位宽和寻址格式的整数存储指令；
    /// 2. When：在非对齐地址 `0x2001` 处执行存储模拟；
    /// 3. Then：内存中仅 `[1..1 + width]` 区间写入 `gpr[7]` 的低 `width` 个小端字节，偏移 `0` 与 `1 + width` 保持为 0，`PC` 前进至 `0x1004`。
    #[test]
    fn integer_store_widths() {
        let cases = [
            (immediate(0xa5, 7), 2),
            (immediate(0xa6, 7), 4),
            (immediate(0xa7, 7), 8),
            (pointer(0x25, 7), 4),
            (pointer(0x27, 7), 8),
            (indexed(0x7028, 7), 2),
            (indexed(0x7030, 7), 4),
            (indexed(0x7038, 7), 8),
        ];
        let bytes = 0xfedc_ba98_7654_3210u64.to_le_bytes();
        for (word, width) in cases {
            let mut memory = Memory::new(0x2000);
            let mut context = UserContext::new(0x1000, 0);
            context.gpr[7] = u64::from_le_bytes(bytes) as usize;

            assert_eq!(
                emulate(&mut context, &mut memory, 0x2001, word),
                EmulationResult::Emulated
            );
            assert_eq!(&memory.bytes[1..1 + width], &bytes[..width]);
            assert_eq!(memory.bytes[0], 0);
            assert_eq!(memory.bytes[1 + width], 0);
            assert_eq!(context.pc, 0x1004);
        }
    }

    /// 测试目标：
    /// 验证单精度（4B）与双精度（8B）浮点访存指令的存储宽度，以及加载时对最低 64 位通道（`lanes[0]`）的覆盖和扩展行为，且不破坏高位的向量通道。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：初始化 `fpr[0].lanes = [0xfedc_ba98_8765_4321, 1, 2, 3]`，遍历立即数和索引格式的单/双精度浮点读写指令对；
    /// 2. When：先在非对齐地址 `0x2001` 执行浮点存储，随后篡改 `lanes[0]` 并从同一地址执行浮点加载；
    /// 3. Then：存储仅写出 `lanes[0]` 的低 4 或 8 字节；加载恢复 `lanes[0]`（单精度负数高 32 位按符号扩展为 `0xffff_ffff`），高通道 `lanes[1..4]` 保持 `[1, 2, 3]` 不变。
    #[test]
    fn floating_register_widths() {
        let cases = [
            (immediate(0xad, 0), immediate(0xac, 0), 4),
            (immediate(0xaf, 0), immediate(0xae, 0), 8),
            (indexed(0x7070, 0), indexed(0x7060, 0), 4),
            (indexed(0x7078, 0), indexed(0x7068, 0), 8),
        ];
        for (store, load, width) in cases {
            let mut memory = Memory::new(0x2000);
            let mut context = UserContext::new(0x1000, 0);
            context.fpr[0].lanes = [0xfedc_ba98_8765_4321, 1, 2, 3];

            assert_eq!(
                emulate(&mut context, &mut memory, 0x2001, store),
                EmulationResult::Emulated
            );
            assert_eq!(
                &memory.bytes[1..1 + width],
                &0xfedc_ba98_8765_4321u64.to_le_bytes()[..width]
            );
            assert_eq!(memory.bytes[0], 0);
            assert_eq!(memory.bytes[1 + width], 0);

            context.fpr[0].lanes[0] = 0x1122_3344_5566_7788;
            assert_eq!(
                emulate(&mut context, &mut memory, 0x2001, load),
                EmulationResult::Emulated
            );
            assert_eq!(
                context.fpr[0].lanes,
                [
                    if width == 4 {
                        0xffff_ffff_8765_4321
                    } else {
                        0xfedc_ba98_8765_4321
                    },
                    1,
                    2,
                    3,
                ]
            );
            assert_eq!(context.pc, 0x1008);
        }
    }

    /// 测试目标：
    /// 验证单精度浮点加载（`fld.s` / `fldx.s`）读取正数（最高位为 0）时，会清除目标浮点寄存器最低 64 位槽原有的高 32 位旧数据。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：内存 `0x2001..0x2005` 写入 `0x1234_5678`，且 `fpr[5].lanes` 初始为 `[u64::MAX, 1, 2, 3]`；
    /// 2. When：分别执行立即数与索引格式的单精度浮点加载指令；
    /// 3. Then：`fpr[5].lanes[0]` 的高 32 位被清零并变为 `0x0000_0000_1234_5678`，其余向量通道保持不变。
    #[test]
    fn floating_single_positive() {
        for word in [immediate(0xac, 5), indexed(0x7060, 5)] {
            let mut memory = Memory::new(0x2000);
            memory.bytes[1..5].copy_from_slice(&0x1234_5678u32.to_le_bytes());
            let mut context = UserContext::new(0x1000, 0);
            context.fpr[5].lanes = [u64::MAX, 1, 2, 3];

            assert_eq!(
                emulate(&mut context, &mut memory, 0x2001, word),
                EmulationResult::Emulated
            );
            assert_eq!(context.fpr[5].lanes, [0x1234_5678, 1, 2, 3]);
            assert_eq!(context.pc, 0x1004);
        }
    }

    /// 测试目标：
    /// 验证跨越 4096 字节页边界的 8 字节非对齐读写能够按递增地址逐字节完成传输，且写后读回的数据完全一致。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：创建起始地址为 `4090` 的内存窗口，令 8 字节访问区间 `4093..4101` 横跨 `4096` 页边界；
    /// 2. When：先在地址 `4093` 执行 `stptr.d $r5` 写入 8 字节，再对同一地址执行 `ldptr.d $r6` 读回；
    /// 3. Then：跨页区间写入完整小端字节序列，且读回的 `gpr[6]` 与 `gpr[5]` 相等，`PC` 累计前进 8 字节。
    #[test]
    fn crossing_page_boundary() {
        let mut memory = Memory::new(4090);
        let mut context = UserContext::new(0x1000, 0);
        context.gpr[5] = 0x8070_6050_4030_2010;

        assert_eq!(
            emulate(&mut context, &mut memory, 4093, pointer(0x27, 5)),
            EmulationResult::Emulated
        );
        assert_eq!(
            &memory.bytes[3..11],
            &0x8070_6050_4030_2010u64.to_le_bytes()
        );

        assert_eq!(
            emulate(&mut context, &mut memory, 4093, pointer(0x26, 6)),
            EmulationResult::Emulated
        );
        assert_eq!(context.gpr[6], context.gpr[5]);
        assert_eq!(context.pc, 0x1008);
    }

    /// 测试目标：
    /// 验证以零寄存器 `$r0` 作为目标寄存器加载时不会修改 `gpr[0]`，作为源寄存器存储时恒写入全 0 字节。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：将模拟内存填满 `0xff`，并人为设置 `context.gpr[0] = 0x1234`；
    /// 2. When：先对地址 `0x2001` 执行 `ld.d $r0`，再对同一地址执行 `st.d $r0`；
    /// 3. Then：加载完成后 `context.gpr[0]` 仍保持 `0x1234` 不被覆盖，存储完成后内存 `0x2001..0x2009` 被写入 8 个零字节。
    #[test]
    fn zero_register() {
        let mut memory = Memory::new(0x2000);
        memory.bytes.fill(0xff);
        let mut context = UserContext::new(0x1000, 0);
        context.gpr[0] = 0x1234;

        assert_eq!(
            emulate(&mut context, &mut memory, 0x2001, immediate(0xa3, 0)),
            EmulationResult::Emulated
        );
        assert_eq!(context.gpr[0], 0x1234);

        assert_eq!(
            emulate(&mut context, &mut memory, 0x2001, immediate(0xa7, 0)),
            EmulationResult::Emulated
        );
        assert_eq!(&memory.bytes[1..9], &[0; 8]);
    }

    /// 测试目标：
    /// 验证非访存指令或单字节访存指令等不支持的指令编码在解码阶段即返回 `Unsupported`，不会触发内存访问或修改寄存器现场。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：准备全 0、全 1、非法操作码及无需软件模拟的单字节加载/存储编码，并在访问地址 `0x2001` 设置权限故障注入；
    /// 2. When：调用 `Instruction::decode` 与 `emulate`；
    /// 3. Then：`decode` 返回 `None`，`emulate` 返回 `EmulationResult::Unsupported`（未触发 `Fault`），且 `PC`、`gpr[7]` 与内存内容均保持原状。
    #[test]
    fn unsupported_instruction() {
        for word in [
            0,
            u32::MAX,
            pointer(0x20, 7),
            indexed(0x7000, 7),
            immediate(0xab, 7),
            immediate(0xa0, 7),
            immediate(0xa4, 7),
            immediate(0xa8, 7),
        ] {
            let mut memory = Memory::new(0x2000);
            memory.fail_at = Some(0x2001);
            let mut context = UserContext::new(0x1000, 0);
            context.gpr[7] = 0x1234;
            assert!(Instruction::decode(word).is_none());
            assert_eq!(
                emulate(&mut context, &mut memory, 0x2001, word),
                EmulationResult::Unsupported
            );
            assert_eq!(context.pc, 0x1000);
            assert_eq!(context.gpr[7], 0x1234);
            assert_eq!(memory.bytes, [0; 32]);
        }
    }

    /// 测试目标：
    /// 验证访存中途遇到页权限错误或地址越界（含 `usize` 溢出边界）时返回 `Fault`，加载指令不修改目标寄存器与 `PC`，存储指令仅保留故障地址之前已写入的字节前缀。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：创建基址为 `4090` 的内存窗口，在页边界地址 `4096` 处注入权限故障，设置 `gpr[7] = usize::MAX`；
    /// 2. When：依次在地址 `4093` 模拟执行 `ld.d $r7`、`st.d $r7`，以及在 `usize::MAX - 2` 处模拟跨地址空间上溢的 `st.d $r7`；
    /// 3. Then：三次调用均返回 `EmulationResult::Fault`；加载失败不破坏 `gpr[7]`；`4093` 处的存储仅写入故障前的 `4093..4096` 共 3 个 `0xff` 字节，`4096` 及之后保持为 0；全程 `PC` 保持 `0x1000` 不变。
    #[test]
    fn memory_error() {
        let mut memory = Memory::new(4090);
        memory.fail_at = Some(4096);
        let mut context = UserContext::new(0x1000, 0);
        context.gpr[7] = usize::MAX;

        assert_eq!(
            emulate(&mut context, &mut memory, 4093, immediate(0xa3, 7)),
            EmulationResult::Fault
        );
        assert_eq!(context.gpr[7], usize::MAX);

        assert_eq!(
            emulate(&mut context, &mut memory, 4093, immediate(0xa7, 7)),
            EmulationResult::Fault
        );
        assert_eq!(&memory.bytes[3..11], &[0xff, 0xff, 0xff, 0, 0, 0, 0, 0]);
        assert_eq!(context.pc, 0x1000);

        assert_eq!(
            emulate(
                &mut context,
                &mut memory,
                usize::MAX - 2,
                immediate(0xa7, 7)
            ),
            EmulationResult::Fault
        );
        assert_eq!(context.pc, 0x1000);
    }
}
