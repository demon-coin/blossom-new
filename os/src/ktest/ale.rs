//! 同一解码与模拟核心的内存模型，不依赖 CPU 是否支持硬件非对齐访问。

use crate::arch::ale::{EmulationResult, Instruction, UserMemory};
use crate::arch::UserContext;
use crate::mm::space::VmError;

struct Memory {
    base: usize,
    bytes: [u8; 32],
    fail_at: Option<usize>,
}

impl Memory {
    /// 小范围模型可放在任意用户地址，包括 4KiB 页边界两侧。
    fn new(base: usize) -> Self {
        Self {
            base,
            bytes: [0; 32],
            fail_at: None,
        }
    }
}

impl UserMemory for Memory {
    /// 未映射或指定失败的地址保留错误，不生成默认数据。
    fn read_byte(&mut self, address: usize) -> Result<u8, VmError> {
        if self.fail_at == Some(address) {
            return Err(VmError::Permission);
        }
        let offset = address.checked_sub(self.base).ok_or(VmError::Unmapped)?;
        self.bytes.get(offset).copied().ok_or(VmError::Unmapped)
    }

    /// 只提交当前地址，便于核对失败时已完成的真实前缀。
    fn write_byte(&mut self, address: usize, value: u8) -> Result<(), VmError> {
        if self.fail_at == Some(address) {
            return Err(VmError::Permission);
        }
        let offset = address.checked_sub(self.base).ok_or(VmError::Unmapped)?;
        *self.bytes.get_mut(offset).ok_or(VmError::Unmapped)? = value;
        Ok(())
    }
}

/// 使用与生产相同的源快照、访存、现场提交顺序执行一条给定指令。
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

/// 编码非零基址和立即数字段，解码只能用真实 opcode 与 rd 字段。
fn immediate(opcode: u32, rd: u32) -> u32 {
    (opcode << 22) | (0xabc << 10) | (3 << 5) | rd
}

/// 指针格式使用独立的八位 opcode，低 24 位均属于操作数。
fn pointer(opcode: u32, rd: u32) -> u32 {
    (opcode << 24) | (0x2345 << 10) | (3 << 5) | rd
}

/// 索引格式保留 rj 与 rk，目的字段仍为低五位。
fn indexed(opcode: u32, rd: u32) -> u32 {
    (opcode << 15) | (11 << 10) | (3 << 5) | rd
}

#[::ktest::suite(context = "Boot", tags("arch", "ale"))]
pub(crate) mod cases {
    use super::*;

    /// st.d 在四字节对齐地址提交八字节，成功只推进一条指令。
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

    /// 整数加载按实际位宽扩展，包含立即数、指针和索引的全部加载形式。
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

    /// 各整数存储编码只写对应低位，指针和索引形式共用真实字节访问。
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

    /// 浮点存储只写对应低位，单精度加载符号扩展并覆盖最低 64 位槽。
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

    /// 单精度正值加载清除最低槽旧高位，立即数和索引形式保持相同扩展规则。
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

    /// 跨 4096 边界的八字节地址必须逐一递增，读回与写入顺序一致。
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

    /// 零寄存器加载仍读取内存但丢弃结果，存储时恒取零。
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

    /// 不支持的编码返回 Unsupported；即使地址不可访问也不触发访存或改变现场。
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

    /// 访问失败返回 Fault，不推进 PC 或提交加载结果，存储失败保留原已写前缀。
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
