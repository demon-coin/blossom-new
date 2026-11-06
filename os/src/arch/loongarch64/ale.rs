//! LoongArch64 用户态地址非对齐异常（ALE, `ECODE_ALE`）的指令解码与逐字节软件模拟模块。
//!
//! # 模块职责与架构定位
//! 当处理器未实现或未启用硬件非对齐访存扩展时，用户态执行非自然对齐的标量半字（2B）、字（4B）或双字（8B）读写指令会触发
//! 地址非对齐异常（`ECODE_ALE`）。本模块负责将 32 位机器指令字解码为统一的访存描述符（`Instruction`），
//! 并通过抽象接口 `UserMemory` 按小端（Little-Endian）字节序逐字节完成读写模拟与上下文写回。
//! - **上游调用者**：
//!   - `task::user::emulate_ale`：在用户态捕获 `ECODE_ALE` 时取指、调用本模块解码并模拟访存，失败时按结果派发 `SIGBUS`（`Unsupported`）或 `SIGSEGV`（`Fault`）；
//!   - `ktest::ale`：在单元测试中传入内存桩验证各类指令编码、跨页访问及异常边界。
//! - **下游依赖**：依赖 `arch::UserContext` 读写通用/浮点寄存器与 PC，以及 `mm::space::VmError` 表达虚拟内存访问错误。
//!
//! # 核心设计与指令格式 / 模拟状态机
//! ## 1. 支持解码的三类 LoongArch64 标量访存指令格式
//! 触发 ALE 时，硬件已将故障有效虚拟地址写入 `CSR_BADV`，因此软件无需重新计算基址寄存器 `rj` 与偏移量，
//! 只需从 32 位指令字中识别操作码（Opcode）与低 5 位目标/源寄存器编号 `rd`/`fd`（Bits `[4:0]`）：
//! ```text
//! (1) 2R-I12 格式 (ld.h/w/d/hu/wu, st.h/w/d, fld.s/d, fst.s/d)
//!  31                22 21                10 9         5 4         0
//! +--------------------+--------------------+-----------+-----------+
//! |   Opcode (10 bits) |     si12 (12 bits) | rj (5b)   | rd/fd (5b)|
//! +--------------------+--------------------+-----------+-----------+
//!
//! (2) 2R-I14 格式 (ldptr.w/d, stptr.w/d)
//!  31            24 23                    10 9         5 4         0
//! +----------------+------------------------+-----------+-----------+
//! | Opcode (8 bits)|       si14 (14 bits)   | rj (5b)   | rd (5b)   |
//! +----------------+------------------------+-----------+-----------+
//!
//! (3) 3R 索引格式 (ldx.h/w/d/hu/wu, stx.h/w/d, fldx.s/d, fstx.s/d)
//!  31                            15 14     10 9         5 4         0
//! +--------------------------------+---------+-----------+-----------+
//! |         Opcode (17 bits)       | rk (5b) | rj (5b)   | rd/fd (5b)|
//! +--------------------------------+---------+-----------+-----------+
//! ```
//! 原子指令（LL/SC、AM*）、向量指令（LSX/LASX）与边界检查指令不支持非对齐模拟，`Instruction::decode` 直接返回 `None`。
//!
//! ## 2. 三阶段解耦模拟流程（避免持锁缺页睡眠）
//! 由于逐字节访存可能跨越 4 KiB 页边界并触发缺页处理（可能睡眠），调用者不能在持有 `UserContext` 锁的期间直接访问用户内存。
//! 因此模拟过程严格拆分为三个不持有睡眠锁的独立阶段：
//! ```text
//! [指令字 word]
//!      |
//!      v
//! 1. Instruction::decode(word) -----> 不支持指令? -----> 返回 EmulationResult::Unsupported (派发 SIGBUS)
//!      |
//!      v
//! 2. instruction.source_value(&ctx)   (短暂读取 UserContext，提取 Store 源寄存器快照后立即释放借用)
//!      |
//!      v
//! 3. instruction.access(mem, badv, src) (在无 UserContext 借用的普通任务上下文中逐字节读写，允许缺页睡眠)
//!      |                              |
//!      | 全部字节成功                  +--> 遇 VmError? -> 返回 EmulationResult::Fault (派发 SIGSEGV，PC 与目标寄存器保持原值)
//!      v
//! 4. instruction.complete(&mut ctx, val) (短暂可变借用 UserContext：Load 符号/零扩展写回寄存器，且 PC += 4)
//!      |
//!      v
//! 返回 EmulationResult::Emulated
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 本模块本身不持有任何锁。
//! - `source_value` 与 `complete` 仅在内存中读写传入的 `UserContext` 引用，耗时极短且不睡眠；
//!   `access` 在无 `UserContext` 借用的情况下回调 `UserMemory`，允许底层 `Mm::fault` 获取地址空间锁并睡眠等待页面就绪。

use crate::arch::UserContext;
use crate::mm::space::VmError;

/// `2R-I12` 格式访存指令操作码的起始位偏移（Bits `[31:22]`，共 10 位）。
const REG2I12_OPCODE_SHIFT: usize = 22;
/// `2R-I14` 格式指针访存指令操作码的起始位偏移（Bits `[31:24]`，共 8 位）。
const REG2I14_OPCODE_SHIFT: usize = 24;
/// `3R` 索引格式访存指令操作码的起始位偏移（Bits `[31:15]`，共 17 位）。
const REG3_OPCODE_SHIFT: usize = 15;
/// 提取指令最低 5 位目标/源寄存器编号（`rd` 或 `fd`，Bits `[4:0]`）的掩码。
const REGISTER_MASK: u32 = (1 << 5) - 1;
/// 单条 LoongArch64 指令的字节长度（固定 4 字节）。
const INSTRUCTION_BYTES: usize = core::mem::size_of::<u32>();

/// `ld.h`（有符号半字加载，2B）的 10 位操作码。
const LD_H: u32 = 0xa1;
/// `ld.w`（有符号字加载，4B）的 10 位操作码。
const LD_W: u32 = 0xa2;
/// `ld.d`（双字加载，8B）的 10 位操作码。
const LD_D: u32 = 0xa3;
/// `st.h`（半字存储，2B）的 10 位操作码。
const ST_H: u32 = 0xa5;
/// `st.w`（字存储，4B）的 10 位操作码。
const ST_W: u32 = 0xa6;
/// `st.d`（双字存储，8B）的 10 位操作码。
const ST_D: u32 = 0xa7;
/// `ld.hu`（无符号半字加载，2B）的 10 位操作码。
const LD_HU: u32 = 0xa9;
/// `ld.wu`（无符号字加载，4B）的 10 位操作码。
const LD_WU: u32 = 0xaa;
/// `fld.s`（单精度浮点加载，4B）的 10 位操作码。
const FLD_S: u32 = 0xac;
/// `fst.s`（单精度浮点存储，4B）的 10 位操作码。
const FST_S: u32 = 0xad;
/// `fld.d`（双精度浮点加载，8B）的 10 位操作码。
const FLD_D: u32 = 0xae;
/// `fst.d`（双精度浮点存储，8B）的 10 位操作码。
const FST_D: u32 = 0xaf;

/// `ldptr.w`（14 位立即数有符号字加载，4B）的 8 位操作码。
const LDPTR_W: u32 = 0x24;
/// `stptr.w`（14 位立即数字存储，4B）的 8 位操作码。
const STPTR_W: u32 = 0x25;
/// `ldptr.d`（14 位立即数双字加载，8B）的 8 位操作码。
const LDPTR_D: u32 = 0x26;
/// `stptr.d`（14 位立即数双字存储，8B）的 8 位操作码。
const STPTR_D: u32 = 0x27;

/// `ldx.h`（寄存器索引有符号半字加载，2B）的 17 位操作码。
const LDX_H: u32 = 0x7008;
/// `ldx.w`（寄存器索引有符号字加载，4B）的 17 位操作码。
const LDX_W: u32 = 0x7010;
/// `ldx.d`（寄存器索引双字加载，8B）的 17 位操作码。
const LDX_D: u32 = 0x7018;
/// `stx.h`（寄存器索引半字存储，2B）的 17 位操作码。
const STX_H: u32 = 0x7028;
/// `stx.w`（寄存器索引字存储，4B）的 17 位操作码。
const STX_W: u32 = 0x7030;
/// `stx.d`（寄存器索引双字存储，8B）的 17 位操作码。
const STX_D: u32 = 0x7038;
/// `ldx.hu`（寄存器索引无符号半字加载，2B）的 17 位操作码。
const LDX_HU: u32 = 0x7048;
/// `ldx.wu`（寄存器索引无符号字加载，4B）的 17 位操作码。
const LDX_WU: u32 = 0x7050;
/// `fldx.s`（寄存器索引单精度浮点加载，4B）的 17 位操作码。
const FLDX_S: u32 = 0x7060;
/// `fldx.d`（寄存器索引双精度浮点加载，8B）的 17 位操作码。
const FLDX_D: u32 = 0x7068;
/// `fstx.s`（寄存器索引单精度浮点存储，4B）的 17 位操作码。
const FSTX_S: u32 = 0x7070;
/// `fstx.d`（寄存器索引双精度浮点存储，8B）的 17 位操作码。
const FSTX_D: u32 = 0x7078;

/// 用户态地址非对齐异常（ALE）软件模拟的执行结果。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum EmulationResult {
    /// 指令解码与全部字节读写均成功完成，目标寄存器已更新且用户 `PC` 已前进 4 字节。
    Emulated,
    /// 故障指令不属于支持模拟的标量访存指令（或用户 `PC` 本身非对齐），上层应保留原现场并派发 `SIGBUS`。
    Unsupported,
    /// 取指或逐字节读写用户内存时发生不可恢复的虚拟内存错误（`VmError`），上层应派发 `SIGSEGV`。
    Fault,
}

/// 面向 ALE 模拟器的逐字节用户内存访问接口。
///
/// 实现者可在单字节访问内部处理缺页异常（包括写时复制 COW）；若中途某个字节返回错误，
/// 本次指令模拟立即中止，此前已成功写入的前缀字节保留在内存中，不执行软件回滚。
pub(crate) trait UserMemory {
    /// 从用户虚拟地址 `address` 读取 1 个字节；若发生缺页且无法恢复，返回对应的 `VmError`。
    fn read_byte(&mut self, address: usize) -> Result<u8, VmError>;
    /// 在检查写权限并完成必要的写时复制（COW）后，向用户虚拟地址 `address` 写入 1 个字节 `value`。
    fn write_byte(&mut self, address: usize, value: u8) -> Result<(), VmError>;
}

/// 解码后的单条非对齐标量访存指令描述符。
pub(crate) struct Instruction {
    /// 目标或源寄存器编号（`0..32`），对应整数通用寄存器 `gpr[register]` 或浮点寄存器 `fpr[register]`。
    register: usize,
    /// 访存字节宽度（取值为 `2`、`4` 或 `8` 字节）。
    width: usize,
    /// 加载完成写回寄存器时是否执行符号扩展（`true` 为有符号扩展至 64 位，`false` 为零扩展）。
    signed: bool,
    /// 访存方向（`true` 表示向内存写入的 Store 指令，`false` 表示从内存读取的 Load 指令）。
    write: bool,
    /// 寄存器堆类别（`true` 表示访问浮点寄存器 `fpr`，`false` 表示访问整数通用寄存器 `gpr`）。
    floating: bool,
}

impl Instruction {
    /// 解码 32 位指令字 `word`；若属于支持模拟的立即数、指针或寄存器索引标量访存指令，返回 `Some(Instruction)`，否则返回 `None`。
    ///
    /// 实际访存有效地址无需从指令中重新计算，直接由调用方传入硬件异常保存的 `CSR_BADV`。
    pub(crate) fn decode(word: u32) -> Option<Self> {
        // Step 1: 依次匹配 2R-I12（高 10 位）、2R-I14（高 8 位）与 3R（高 17 位）指令操作码，提取宽度、符号、方向与浮点标志。
        let (width, signed, write, floating) = match word >> REG2I12_OPCODE_SHIFT {
            LD_H => (2, true, false, false),
            LD_W => (4, true, false, false),
            LD_D => (8, true, false, false),
            ST_H => (2, false, true, false),
            ST_W => (4, false, true, false),
            ST_D => (8, false, true, false),
            LD_HU => (2, false, false, false),
            LD_WU => (4, false, false, false),
            FLD_S => (4, true, false, true),
            FST_S => (4, false, true, true),
            FLD_D => (8, false, false, true),
            FST_D => (8, false, true, true),
            _ => match word >> REG2I14_OPCODE_SHIFT {
                LDPTR_W => (4, true, false, false),
                STPTR_W => (4, false, true, false),
                LDPTR_D => (8, true, false, false),
                STPTR_D => (8, false, true, false),
                _ => match word >> REG3_OPCODE_SHIFT {
                    LDX_H => (2, true, false, false),
                    LDX_W => (4, true, false, false),
                    LDX_D => (8, true, false, false),
                    STX_H => (2, false, true, false),
                    STX_W => (4, false, true, false),
                    STX_D => (8, false, true, false),
                    LDX_HU => (2, false, false, false),
                    LDX_WU => (4, false, false, false),
                    FLDX_S => (4, true, false, true),
                    FLDX_D => (8, false, false, true),
                    FSTX_S => (4, false, true, true),
                    FSTX_D => (8, false, true, true),
                    _ => return None,
                },
            },
        };

        // Step 2: 提取低 5 位寄存器编号并构造指令描述符。
        Some(Self {
            register: (word & REGISTER_MASK) as usize,
            width,
            signed,
            write,
            floating,
        })
    }

    /// 在开始访存前从用户态上下文 `context` 中读取 Store 指令要写入的 64 位源数据快照（Load 指令固定返回 `0`）。
    ///
    /// 若为整数零寄存器（`$r0`），架构定义为恒零，直接返回 `0` 而不读取保存槽。
    pub(crate) fn source_value(&self, context: &UserContext) -> u64 {
        if !self.write {
            0
        } else if self.floating {
            context.fpr[self.register].lanes[0]
        } else if self.register == 0 {
            0
        } else {
            context.gpr[self.register] as u64
        }
    }

    /// 在不持有 `UserContext` 借用的普通任务上下文中，按小端字节序对 `[address, address + width)` 逐字节执行读写。
    ///
    /// 允许底层 `memory` 在跨页边界时触发缺页处理并睡眠；非对齐逐字节访问不提供跨字节原子性保证。
    pub(crate) fn access(
        &self,
        memory: &mut impl UserMemory,
        address: usize,
        source: u64,
    ) -> Result<u64, VmError> {
        // Step 1: 检查 `[address, address + width - 1]` 是否发生 `usize` 算术溢出。
        address
            .checked_add(self.width - 1)
            .ok_or(VmError::InvalidRange)?;
        let mut value = 0;

        // Step 2: 按小端序从低地址到高地址逐字节写入或读取组装。
        for offset in 0..self.width {
            if self.write {
                memory.write_byte(
                    address + offset,
                    (source >> (offset * u8::BITS as usize)) as u8,
                )?;
            } else {
                value |=
                    (memory.read_byte(address + offset)? as u64) << (offset * u8::BITS as usize);
            }
        }

        Ok(value)
    }

    /// 在全部字节访存成功后，将 Load 结果（经符号扩展或零扩展）写回目标寄存器，并将用户态 `PC` 前进 4 字节。
    ///
    /// - 浮点加载（`fld.s`/`fld.d`/`fldx.s`/`fldx.d`）写回目标浮点寄存器的最低 64 位通道 `lanes[0]`，保留高维向量通道不变。
    /// - 整数加载若目标为 `$r0` 则丢弃结果，保持零寄存器不变。
    pub(crate) fn complete(&self, context: &mut UserContext, value: u64) {
        // Step 1: 若为 Load 指令，根据 `signed` 与 `width` 完成 64 位扩展并写回对应寄存器。
        if !self.write {
            let shift = (core::mem::size_of::<u64>() - self.width) * u8::BITS as usize;
            let value = if self.signed {
                (((value << shift) as i64) >> shift) as u64
            } else {
                value
            };

            if self.floating {
                context.fpr[self.register].lanes[0] = value;
            } else if self.register != 0 {
                context.gpr[self.register] = value as usize;
            }
        }
        // Step 2: 访存模拟已完成，将用户程序计数器推进一条指令的长度（4 字节）。
        context.pc = context.pc.wrapping_add(INSTRUCTION_BYTES);
    }
}
