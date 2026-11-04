//! 用户非对齐访存的纯解码与逐字节模拟，不访问真实用户虚拟指针。

use crate::arch::UserContext;
use crate::mm::space::VmError;

const REG2I12_OPCODE_SHIFT: usize = 22;
const REG2I14_OPCODE_SHIFT: usize = 24;
const REG3_OPCODE_SHIFT: usize = 15;
const REGISTER_MASK: u32 = (1 << 5) - 1;
const INSTRUCTION_BYTES: usize = core::mem::size_of::<u32>();
const LD_H: u32 = 0xa1;
const LD_W: u32 = 0xa2;
const LD_D: u32 = 0xa3;
const ST_H: u32 = 0xa5;
const ST_W: u32 = 0xa6;
const ST_D: u32 = 0xa7;
const LD_HU: u32 = 0xa9;
const LD_WU: u32 = 0xaa;
const FLD_S: u32 = 0xac;
const FST_S: u32 = 0xad;
const FLD_D: u32 = 0xae;
const FST_D: u32 = 0xaf;
const LDPTR_W: u32 = 0x24;
const STPTR_W: u32 = 0x25;
const LDPTR_D: u32 = 0x26;
const STPTR_D: u32 = 0x27;
const LDX_H: u32 = 0x7008;
const LDX_W: u32 = 0x7010;
const LDX_D: u32 = 0x7018;
const STX_H: u32 = 0x7028;
const STX_W: u32 = 0x7030;
const STX_D: u32 = 0x7038;
const LDX_HU: u32 = 0x7048;
const LDX_WU: u32 = 0x7050;
const FLDX_S: u32 = 0x7060;
const FLDX_D: u32 = 0x7068;
const FSTX_S: u32 = 0x7070;
const FSTX_D: u32 = 0x7078;

/// 模拟成功才提交现场；不支持的指令与用户访存错误分别由异常层选择信号。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum EmulationResult {
    Emulated,
    Unsupported,
    Fault,
}

/// 字节访问可以处理缺页；错误终止本次模拟，已写入的前缀不回滚。
pub(crate) trait UserMemory {
    /// 读取一个用户字节，保留原地址空间错误。
    fn read_byte(&mut self, address: usize) -> Result<u8, VmError>;
    /// 在写权限及 COW 处理完成后写入一个用户字节。
    fn write_byte(&mut self, address: usize, value: u8) -> Result<(), VmError>;
}

pub(crate) struct Instruction {
    register: usize,
    width: usize,
    signed: bool,
    write: bool,
    floating: bool,
}

impl Instruction {
    /// 识别立即数、指针及索引访存编码，地址直接采用异常保存的 BADV。
    pub(crate) fn decode(word: u32) -> Option<Self> {
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
        Some(Self {
            register: (word & REGISTER_MASK) as usize,
            width,
            signed,
            write,
            floating,
        })
    }

    /// 在短暂现场借用中取得存储源；整数零寄存器不依赖保存槽的内容。
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

    /// 在无现场借用的普通任务上下文访问各字节，允许缺页睡眠且不提供原子性。
    pub(crate) fn access(
        &self,
        memory: &mut impl UserMemory,
        address: usize,
        source: u64,
    ) -> Result<u64, VmError> {
        address
            .checked_add(self.width - 1)
            .ok_or(VmError::InvalidRange)?;
        let mut value = 0;
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

    /// 全部访问成功后扩展加载结果并写回现场；浮点加载覆盖最低 64 位槽。
    pub(crate) fn complete(&self, context: &mut UserContext, value: u64) {
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
        context.pc = context.pc.wrapping_add(INSTRUCTION_BYTES);
    }
}
