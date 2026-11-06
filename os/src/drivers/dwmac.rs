//! Synopsys DesignWare 千兆以太网控制器（DWMAC4 / DWMAC1000）单队列原生驱动模块。
//!
//! ## 模块职责与架构定位
//! 本模块作为实板平台（VisionFive 2 上的 JH7110 DWMAC4 与 LS2K1000LA 上的 DWMAC1000）的底层网卡驱动，向上通过 `drivers::net_device::NetDevice` 接入内核协议栈 `NetStack`，向下依赖 `mm::dma` 物理页与 DMA 缓存同步、`irq` 外部中断管理、`workqueue` 工作队列、`timer` 定时器、`drivers::board_setup` 板级时钟/引脚配置，以及子模块 `descriptors`（描述符环）和 `phy`（MDIO 与裕太微 PHY 管理）。
//! 许可与修改声明见 `dwmac/LICENSE`。
//!
//! ## 核心状态机与 DMA 槽位生命周期
//!
//! ```text
//! 1. 设备生命周期状态机 (Phase)：
//!
//!    prepare()            start()             stop_sync()           SWR 复位确认完成
//!   ──────────► Prepared ────────► Live ──────────────────► Stopping ──────────────► Stopped
//!                  │                │                          ▲
//!                  │ 准备/发布失败   │ DMA 总线错误/传输故障      │
//!                  └────────────────┴───────► Faulted ─────────┘
//!
//! 2. 收发载荷槽位状态机 (SlotState) 与物理页所有权流转：
//!
//!   [TX 发送路径]
//!     Free ──(try_transmit 认领)──► Preparing ──(锁外写入帧 + begin_device_access)──► Ready
//!       ▲                                                                              │
//!       │                                                      (publish_ready 连续前缀置 OWN=1)
//!       │                                                                              ▼
//!       └──(归还空槽)── Processing ◄──(complete_tx 见 OWN=0)───────────────────── DeviceOwned
//!                    (锁外 finish_device_access)
//!
//!   [RX 接收路径]
//!     Free ──(publish 预投递)──► Processing ──(begin_device_access + arm_rx)──► DeviceOwned
//!                                    ▲                                               │
//!                                    │                  (receive_one 见 OWN=0，锁外 finish_device_access
//!                                    └─────────────────── 送交协议栈后重新 begin_device_access 归还硬件)
//! ```
//!
//! - **物理页防泄漏与防 UAF 契约**：所有 32 个 RX 与 32 个 TX 缓冲区物理页（`MappedPages`）及两个 `Ring` 在 `prepare` 期间一次性分配并置于 `Inner` 中。在锁外执行帧拷贝或 DMA 缓存同步期间，物理页由栈上 RAII 守卫 `SlotLease` 临时持有；一旦中途报错，`SlotLease::drop` 会自动将物理页按当前真实 DMA 所有权归还回原槽，绝不在硬件仍在访问时提前释放物理页。只有在 `stop_sync` 等待所有回调退出且确认硬件 DMA 软复位（`SWR` 清零）完成后，才统一收回 DMA 访问权并销毁 `Inner`。
//! - **NAPI 风格中断合并**：硬中断处理函数（ISR）仅应答硬件中断原因、屏蔽通道中断并唤醒 CPU0 上的唯一 `progress` Work；数据帧收发与 PHY 链路轮询全部在 `progress` Work 中按预算（`RX_BUDGET = 16`）批处理，处理完毕后由 `rearm` 重新开中断并做二次状态复查以消除竞态窗口。
//!
//! ## 锁获取顺序（Lock Ordering）与并发契约
//! 本模块包含三把锁，严格遵循以下由外向内的获取层级，严禁反向嵌套或在持有自旋锁时睡眠：
//! 1. `Dwmac::control`（`SleepMutex<()>`）：最外层可睡眠互斥锁，仅串行化 `start()` 与 `stop_sync()` 生命周期控制路径；
//! 2. `Dwmac::registration`（`SpinLock<Option<IrqRegistration>>`）与 `Dwmac::state`（`SpinLock<State>`）：同级叶子自旋锁，两者**互不嵌套**（获取其中一把前必须已释放另一把）；
//! 3. `Dwmac::state` 临界区内仅更新槽状态游标或执行极短的 MMIO 读写，帧载荷拷贝、协议栈上送（`receive_ethernet`）、发送队列唤醒（`kick_egress`）、DMA 缓存刷新及睡眠等待一律在 `state` 锁外执行。

mod descriptors;
mod phy;
pub(crate) use self::phy::PHY_ADDRESS_COUNT;

use alloc::{
    sync::{Arc, Weak},
    vec::Vec,
};
use core::{mem::ManuallyDrop, pin::Pin, ptr, slice};
use spin::Once;
use virtio_drivers::BufferDirection;

use crate::{
    arch,
    boot::{self, DeviceSetup, GmacResources, Platform, Region},
    drivers::board_setup,
    irq::{self, IrqError, IrqNumber, IrqRegistration},
    mm::{
        self,
        dma::{DmaDomain, DmaError, MappedPages},
    },
    net::{stats::InterfaceStats, NetStack, NetStackRef, ETHERNET_FRAME_CAPACITY},
    sync::{SleepMutex, SpinLock},
    time::{self, Duration},
    timer::{self, Timer, TimerKind},
    workqueue::Work,
};

use self::{
    descriptors::{device_owned, Ring},
    phy::{Link, Phy, SPEED_10, SPEED_100, SPEED_1000},
};

const RING_LEN: usize = 32;
const BUFFER_LEN: usize = 2048;
const FRAME_MIN: usize = smoltcp::wire::ETHERNET_HEADER_LEN;
const FRAME_MAX: usize = ETHERNET_FRAME_CAPACITY;
const RX_BUDGET: usize = 16;
const REGISTER_BYTES: usize = core::mem::size_of::<u32>();

const MAC_CONFIG: usize = 0;
const MAC_ADDRESS_ENABLE: u32 = 1 << 31;
const MAC_FILTER_MULTICAST: u32 = 1 << 4;
const MAC_FILTER_HASH_OR_PERFECT: u32 = 1 << 10;
const MAC_PORT_SELECT: u32 = 1 << 15;
const MAC_FAST_ETHERNET: u32 = 1 << 14;
const DMA_BUS_MODE: usize = 0x1000;
const DMA_SOFTWARE_RESET: u32 = 1;
const DMA_BURST_LENGTH: u32 = 8;
const DMA_RESET_TIMEOUT: Duration = Duration::from_secs(2);

const DWMAC4_PACKET_FILTER: usize = 0x8;
const DWMAC4_TX_FLOW_CONTROL: usize = 0x70;
const DWMAC4_RX_FLOW_CONTROL: usize = 0x90;
const DWMAC4_RX_QUEUE_CONTROL0: usize = 0xa0;
const DWMAC4_RX_QUEUE_CONTROL1: usize = 0xa4;
const DWMAC4_INTERRUPT_ENABLE: usize = 0xb4;
const DWMAC4_VERSION: usize = 0x110;
const DWMAC4_HW_FEATURE1: usize = 0x120;
const DWMAC4_ADDRESS_HIGH: usize = 0x300;
const DWMAC4_MTL_TX_OPERATION: usize = 0xd00;
const DWMAC4_MTL_TX_WEIGHT: usize = 0xd18;
const DWMAC4_MTL_RX_OPERATION: usize = 0xd30;
const DWMAC4_DMA_CHANNEL_CONTROL: usize = 0x1100;
const DWMAC4_DMA_TX_CONTROL: usize = 0x1104;
const DWMAC4_DMA_RX_CONTROL: usize = 0x1108;
const DWMAC4_DMA_TX_BASE_HIGH: usize = 0x1110;
const DWMAC4_DMA_TX_BASE: usize = 0x1114;
const DWMAC4_DMA_RX_BASE_HIGH: usize = 0x1118;
const DWMAC4_DMA_RX_BASE: usize = 0x111c;
const DWMAC4_DMA_TX_TAIL: usize = 0x1120;
const DWMAC4_DMA_RX_TAIL: usize = 0x1128;
const DWMAC4_DMA_TX_RING_LENGTH: usize = 0x112c;
const DWMAC4_DMA_RX_RING_LENGTH: usize = 0x1130;
const DWMAC4_DMA_INTERRUPT_ENABLE: usize = 0x1134;
const DWMAC4_DMA_STATUS: usize = 0x1160;

const DWMAC4_STATUS_ACK_MASK: u32 = 0xffff;
const DWMAC4_DMA_NORMAL_INTERRUPT: u32 = 1 << 15;
const DWMAC4_DMA_ABNORMAL_INTERRUPT: u32 = 1 << 14;
const DWMAC4_DMA_FATAL_BUS_ERROR: u32 = 1 << 12;
const DWMAC4_DMA_RX_BUFFER_UNAVAILABLE: u32 = 1 << 7;
const DWMAC4_DMA_RX_INTERRUPT: u32 = 1 << 6;
const DWMAC4_DMA_TX_INTERRUPT: u32 = 1;
const DWMAC4_MAC_TRANSMIT: u32 = 1 << 1;
const DWMAC4_MAC_RECEIVE: u32 = 1;
const DWMAC4_MAC_DUPLEX: u32 = 1 << 13;
const DWMAC4_DMA_START_TX: u32 = 1;
const DWMAC4_DMA_START_RX: u32 = 1;
const DWMAC4_DMA_DESCRIPTOR_CACHE: u32 = 1 << 19;
const DWMAC4_DMA_SKIP_SHIFT: u32 = 18;
const DWMAC4_DMA_BURST_SHIFT: u32 = 16;
const DWMAC4_DMA_SECOND_PACKET: u32 = 1 << 4;
const DWMAC4_DMA_RX_BUFFER_SHIFT: u32 = 1;
const DWMAC4_FIFO_BASE_BYTES: usize = 128;
const DWMAC4_FIFO_SIZE_MASK: u32 = 0x1f;
const DWMAC4_TX_FIFO_SIZE_SHIFT: u32 = 6;
const DWMAC4_MTL_FIFO_UNIT_BYTES: usize = 256;
const DWMAC4_MTL_TX_SIZE_MAX: usize = 0x1ff;
const DWMAC4_MTL_TX_SIZE_SHIFT: u32 = 16;
const DWMAC4_MTL_RX_SIZE_MAX: usize = 0x3ff;
const DWMAC4_MTL_RX_SIZE_SHIFT: u32 = 20;
const DWMAC4_MTL_TX_QUEUE_ENABLE: u32 = 2 << 2;
const DWMAC4_MTL_TX_STORE_FORWARD: u32 = 1 << 1;
const DWMAC4_MTL_RX_STORE_FORWARD: u32 = 1 << 5;
const DWMAC4_MTL_TX_WEIGHT_DEFAULT: u32 = 0x10;
const DWMAC4_RX_QUEUE_DCB: u32 = 2;
const DWMAC4_MULTICAST_BROADCAST_QUEUE: u32 = 1 << 20;

const DWMAC1000_FRAME_FILTER: usize = 0x4;
const DWMAC1000_FLOW_CONTROL: usize = 0x18;
const DWMAC1000_VERSION: usize = 0x20;
const DWMAC1000_INTERRUPT_MASK: usize = 0x3c;
const DWMAC1000_ADDRESS_HIGH: usize = 0x40;
const DWMAC1000_MMC_RX_INTERRUPT_MASK: usize = 0x10c;
const DWMAC1000_MMC_TX_INTERRUPT_MASK: usize = 0x110;
const DWMAC1000_MMC_RX_IPC_INTERRUPT_MASK: usize = 0x200;
const DWMAC1000_DMA_TX_POLL: usize = 0x1004;
const DWMAC1000_DMA_RX_POLL: usize = 0x1008;
const DWMAC1000_DMA_RX_BASE: usize = 0x100c;
const DWMAC1000_DMA_TX_BASE: usize = 0x1010;
const DWMAC1000_DMA_STATUS: usize = 0x1014;
const DWMAC1000_DMA_CONTROL: usize = 0x1018;
const DWMAC1000_DMA_INTERRUPT_ENABLE: usize = 0x101c;
const DWMAC1000_DMA_HW_FEATURE: usize = 0x1058;

const DWMAC1000_STATUS_ACK_MASK: u32 = 0x1ffff;
const DWMAC1000_DMA_NORMAL_INTERRUPT: u32 = 1 << 16;
const DWMAC1000_DMA_ABNORMAL_INTERRUPT: u32 = 1 << 15;
const DWMAC1000_DMA_FATAL_BUS_ERROR: u32 = 1 << 13;
const DWMAC1000_DMA_RX_INTERRUPT: u32 = 1 << 6;
const DWMAC1000_DMA_TX_UNDERFLOW: u32 = 1 << 5;
const DWMAC1000_DMA_TX_INTERRUPT: u32 = 1;
const DWMAC1000_MAC_TRANSMIT: u32 = 1 << 3;
const DWMAC1000_MAC_RECEIVE: u32 = 1 << 2;
const DWMAC1000_MAC_DUPLEX: u32 = 1 << 11;
const DWMAC1000_MAC_DISABLE_CARRIER_SENSE: u32 = 1 << 16;
const DWMAC1000_DMA_START_TX: u32 = 1 << 13;
const DWMAC1000_DMA_START_RX: u32 = 1 << 1;
const DWMAC1000_ENHANCED_DESCRIPTORS: u32 = 1 << 24;
const DWMAC1000_DMA_SKIP_SHIFT: u32 = 2;
const DWMAC1000_DMA_ALTERNATE_DESCRIPTOR: u32 = 1 << 7;
const DWMAC1000_DMA_TX_BURST_SHIFT: u32 = 8;
const DWMAC1000_DMA_RX_BURST_SHIFT: u32 = 17;
const DWMAC1000_DMA_SEPARATE_BURST: u32 = 1 << 23;
const DWMAC1000_DMA_RX_STORE_FORWARD: u32 = 1 << 25;
const DWMAC1000_DMA_TX_STORE_FORWARD: u32 = 1 << 21;
const DWMAC1000_DMA_SECOND_FRAME: u32 = 1 << 2;
const DWMAC1000_MAC_INTERRUPT_DISABLE: u32 = 0x60f;

const MAC_VERSION_MASK: u32 = 0xff;
const DWMAC_CORE_5_10: u32 = 0x51;
const DWMAC_CORE_5_20: u32 = 0x52;
const DWMAC_CORE_3_70: u32 = 0x37;

const STARFIVE_INTERFACE_BITS: u32 = 3;
const STARFIVE_INTERFACE_MASK: u32 = (1 << STARFIVE_INTERFACE_BITS) - 1;
const STARFIVE_INTERFACE_RGMII: u32 = 1;
const STARFIVE_INTERFACE_RMII: u32 = 4;
const STARFIVE_TX_CLOCK_INDEX: usize = 3;
pub(crate) const DWMAC1000_MMIO_MIN_BYTES: usize = DWMAC1000_DMA_HW_FEATURE + REGISTER_BYTES;

const RGMII_GIGABIT_CLOCK_HZ: u64 = 125_000_000;
const MII_FAST_ETHERNET_CLOCK_HZ: u64 = 25_000_000;
const MII_ETHERNET_CLOCK_HZ: u64 = MII_FAST_ETHERNET_CLOCK_HZ / 10;

/// DWMAC 千兆网卡驱动错误类型，区分内存不足、板级资源配置、DMA 同步、中断注册、硬件 I/O 及停止超时等不同故障原因。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GmacError {
    /// 内核堆或物理页分配失败。
    NoMemory,
    /// 设备树资源缺失、寄存器窗口越界、硬件版本不匹配或时钟/PHY 参数不受支持。
    Resource(&'static str),
    /// 除内存耗尽外的 DMA 地址映射或缓存一致性维护错误。
    Dma(DmaError),
    /// 外部中断号映射或中断处理函数注册失败。
    Irq(IrqError),
    /// 运行期或初始化期间的硬件通信故障（如 MDIO 超时、PHY 协商异常、DMA 致命总线错误或非法帧长）。
    Io(&'static str),
    /// 发起 DMA 软件复位后在超时时间内仍未观测到硬件 `SWR` 位清零，此时底层 DMA 物理页必须继续保留而不可释放。
    StopUnconfirmed,
}

impl From<DmaError> for GmacError {
    /// 将底层 `DmaError` 转换为 `GmacError`：内存分配不足提升为顶层 `NoMemory`，其余保留在 `Dma` 变体中。
    fn from(error: DmaError) -> Self {
        match error {
            DmaError::NoMemory => Self::NoMemory,
            _ => Self::Dma(error),
        }
    }
}

/// Synopsys DesignWare 以太网控制器硬件代数。
#[derive(Clone, Copy, Eq, PartialEq)]
enum Generation {
    /// DWMAC4 / QoS 架构（如 StarFive JH7110 集成的 5.10 / 5.20 核心）。
    Dwmac4,
    /// DWMAC1000 架构（如龙芯 LS2K1000LA 集成的 3.70a 核心）。
    Dwmac1000,
}

/// 已验证边界与映射有效性的 MAC 控制器 MMIO 寄存器访问句柄。
///
/// 仅由 `Dwmac::prepare` 在核实平台资源与永久强序映射后构造。
#[derive(Clone, Copy)]
struct Registers {
    /// 控制器 MMIO 物理地址区间 `[start, end)`（单位：字节）。
    region: Region,
    /// 控制器硬件代数，决定寄存器偏移表与位域格式。
    generation: Generation,
}

impl Registers {
    /// 以 32 位对齐的 `read_volatile` 读取偏移 `offset`（字节）处的设备寄存器，不构造指向 MMIO 空间的 Rust 引用。
    fn read(self, offset: usize) -> u32 {
        assert!(
            offset % REGISTER_BYTES == 0
                && offset + REGISTER_BYTES <= self.region.end - self.region.start
        );
        // SAFETY:
        // 1. 前置 `assert!` 已验证 `offset` 满足 4 字节对齐且 `[offset, offset + 4)` 完全落在 `self.region` 窗口内；
        // 2. `Dwmac::prepare` 的调用契约保证 `self.region` 在整个设备生命周期内具有永久有效的强序 MMIO 虚拟映射且由本驱动独占。
        unsafe {
            ptr::read_volatile((arch::mmio_address(self.region.start) + offset) as *const u32)
        }
    }

    /// 在插入 DMA 写屏障后向偏移 `offset`（字节）处的 32 位寄存器执行 `write_volatile`，并在写后执行全屏障以排序描述符发布与硬件通知。
    fn write(self, offset: usize, value: u32) {
        assert!(
            offset % REGISTER_BYTES == 0
                && offset + REGISTER_BYTES <= self.region.end - self.region.start
        );
        arch::dma_write_barrier();
        // SAFETY:
        // 1. 前置 `assert!` 已验证 `offset` 4 字节对齐且未超出已映射的 `self.region` 边界；
        // 2. `Dwmac::prepare` 调用契约保证该 MMIO 窗口永久映射且无其他驱动并发写同一硬件实例。
        unsafe {
            ptr::write_volatile(
                (arch::mmio_address(self.region.start) + offset) as *mut u32,
                value,
            )
        };
        arch::dma_barrier();
    }

    /// 返回当前控制器代数对应的 DMA 中断使能寄存器字节偏移。
    fn interrupt_register(self) -> usize {
        match self.generation {
            Generation::Dwmac4 => DWMAC4_DMA_INTERRUPT_ENABLE,
            Generation::Dwmac1000 => DWMAC1000_DMA_INTERRUPT_ENABLE,
        }
    }

    /// 返回当前控制器代数对应的 DMA 中断状态寄存器字节偏移。
    fn status_register(self) -> usize {
        match self.generation {
            Generation::Dwmac4 => DWMAC4_DMA_STATUS,
            Generation::Dwmac1000 => DWMAC1000_DMA_STATUS,
        }
    }

    /// 返回 DMA 状态寄存器中支持写 1 清零（W1C）的中断原因位掩码，排除只读的通道运行状态字段。
    fn status_mask(self) -> u32 {
        match self.generation {
            Generation::Dwmac4 => DWMAC4_STATUS_ACK_MASK,
            Generation::Dwmac1000 => DWMAC1000_STATUS_ACK_MASK,
        }
    }

    /// 返回通道正常工作时需开启的 DMA 中断位掩码（含收发完成、接收缓冲不可用/发送下溢异常及致命总线错误）。
    fn interrupt_mask(self) -> u32 {
        match self.generation {
            Generation::Dwmac4 => {
                DWMAC4_DMA_NORMAL_INTERRUPT
                    | DWMAC4_DMA_ABNORMAL_INTERRUPT
                    | DWMAC4_DMA_FATAL_BUS_ERROR
                    | DWMAC4_DMA_RX_BUFFER_UNAVAILABLE
                    | DWMAC4_DMA_RX_INTERRUPT
                    | DWMAC4_DMA_TX_INTERRUPT
            }
            Generation::Dwmac1000 => {
                DWMAC1000_DMA_NORMAL_INTERRUPT
                    | DWMAC1000_DMA_ABNORMAL_INTERRUPT
                    | DWMAC1000_DMA_FATAL_BUS_ERROR
                    | DWMAC1000_DMA_RX_INTERRUPT
                    | DWMAC1000_DMA_TX_UNDERFLOW
                    | DWMAC1000_DMA_TX_INTERRUPT
            }
        }
    }

    /// 读取并通过写 1 清零（W1C）应答当前已触发的本设备 DMA 中断原因位；若共享中断线上非本设备触发则返回 `0`。
    fn acknowledge(self) -> u32 {
        let reason = self.read(self.status_register()) & self.status_mask();
        if reason != 0 {
            self.write(self.status_register(), reason);
        }
        reason
    }

    /// 在 MAC 控制器内部开启或屏蔽 DMA 通道中断输出，不影响共享外部中断线上的其他设备。
    fn interrupts(self, enabled: bool) {
        self.write(
            self.interrupt_register(),
            if enabled { self.interrupt_mask() } else { 0 },
        );
    }

    /// 判断 DMA 状态位掩码 `reason` 中是否包含不可恢复的致命总线错误（Fatal Bus Error）。
    fn fatal(self, reason: u32) -> bool {
        reason
            & match self.generation {
                Generation::Dwmac4 => DWMAC4_DMA_FATAL_BUS_ERROR,
                Generation::Dwmac1000 => DWMAC1000_DMA_FATAL_BUS_ERROR,
            }
            != 0
    }

    /// 返回 MAC 地址 0 高位寄存器（`MAC_Address0_High`）在当前代数下的字节偏移。
    fn address_register(self) -> usize {
        match self.generation {
            Generation::Dwmac4 => DWMAC4_ADDRESS_HIGH,
            Generation::Dwmac1000 => DWMAC1000_ADDRESS_HIGH,
        }
    }

    /// 从硬件 `MAC_Address0_High/Low` 寄存器读取当前保存的 6 字节以太网 MAC 地址。
    fn mac_address(self) -> [u8; 6] {
        let high = self.read(self.address_register()).to_le_bytes();
        let low = self
            .read(self.address_register() + REGISTER_BYTES)
            .to_le_bytes();
        [low[0], low[1], low[2], low[3], high[0], high[1]]
    }

    /// 将 6 字节单播以太网地址写入硬件 `MAC_Address0_Low/High` 寄存器并置位地址使能位。
    fn set_mac_address(self, mac: [u8; 6]) {
        self.write(
            self.address_register() + REGISTER_BYTES,
            u32::from_le_bytes([mac[0], mac[1], mac[2], mac[3]]),
        );
        self.write(
            self.address_register(),
            MAC_ADDRESS_ENABLE | u32::from(u16::from_le_bytes([mac[4], mac[5]])),
        );
    }

    /// 关闭 DMA 通道中断并清除 DMA 与 MAC 的收发启动位，阻止硬件发起新的帧收发（注意：仅清零启动位并不代表在途 DMA 传输已彻底排空）。
    fn stop_dma(self) {
        self.interrupts(false);
        match self.generation {
            Generation::Dwmac4 => {
                self.write(
                    DWMAC4_DMA_TX_CONTROL,
                    self.read(DWMAC4_DMA_TX_CONTROL) & !DWMAC4_DMA_START_TX,
                );
                self.write(
                    DWMAC4_DMA_RX_CONTROL,
                    self.read(DWMAC4_DMA_RX_CONTROL) & !DWMAC4_DMA_START_RX,
                );
                self.write(
                    MAC_CONFIG,
                    self.read(MAC_CONFIG) & !(DWMAC4_MAC_TRANSMIT | DWMAC4_MAC_RECEIVE),
                );
            }
            Generation::Dwmac1000 => {
                self.write(
                    DWMAC1000_DMA_CONTROL,
                    self.read(DWMAC1000_DMA_CONTROL)
                        & !(DWMAC1000_DMA_START_TX | DWMAC1000_DMA_START_RX),
                );
                self.write(
                    MAC_CONFIG,
                    self.read(MAC_CONFIG) & !(DWMAC1000_MAC_TRANSMIT | DWMAC1000_MAC_RECEIVE),
                );
            }
        }
    }

    /// 向 `DMA_BUS_MODE` 寄存器置位软件复位位（`SWR`），请求复位 MAC 与 DMA 内部状态机（调用前须确保 PHY RXC 时钟持续输入）。
    fn request_reset(self) {
        self.write(DMA_BUS_MODE, self.read(DMA_BUS_MODE) | DMA_SOFTWARE_RESET);
    }

    /// 在可睡眠上下文（初始化或同步停止流程）轮询等待硬件清除 `DMA_SOFTWARE_RESET` 位；超过 2 秒未清零则返回 `StopUnconfirmed`。
    fn wait_reset(self) -> Result<(), GmacError> {
        let deadline = time::now().checked_add(DMA_RESET_TIMEOUT).unwrap();
        while self.read(DMA_BUS_MODE) & DMA_SOFTWARE_RESET != 0 {
            if deadline <= time::now() {
                return Err(GmacError::StopUnconfirmed);
            }
            timer::sleep(Duration::from_millis(1));
        }
        arch::dma_barrier();
        Ok(())
    }

    /// 初始化单队列 RX/TX FIFO、帧过滤规则、突发长度及描述符环基址/尾指针寄存器，配置完成后仍保持 MAC 与 DMA 处于未启动状态。
    fn configure(self, inner: &Inner, mac: [u8; 6]) -> Result<(), GmacError> {
        self.interrupts(false);
        self.acknowledge();
        self.set_mac_address(mac);
        match self.generation {
            Generation::Dwmac4 => {
                // Step 1: 读取硬件特性寄存器计算板载 RX/TX FIFO 容量，启用 MTL Store-and-Forward 模式并映射队列 0。
                let feature = self.read(DWMAC4_HW_FEATURE1);
                let rx_fifo = DWMAC4_FIFO_BASE_BYTES
                    .checked_shl(feature & DWMAC4_FIFO_SIZE_MASK)
                    .ok_or(GmacError::Resource("invalid RX FIFO feature"))?;
                let tx_fifo = DWMAC4_FIFO_BASE_BYTES
                    .checked_shl((feature >> DWMAC4_TX_FIFO_SIZE_SHIFT) & DWMAC4_FIFO_SIZE_MASK)
                    .ok_or(GmacError::Resource("invalid TX FIFO feature"))?;
                if rx_fifo < DWMAC4_MTL_FIFO_UNIT_BYTES || tx_fifo < DWMAC4_MTL_FIFO_UNIT_BYTES {
                    return Err(GmacError::Resource("GMAC FIFO is too small"));
                }
                self.write(
                    DWMAC4_MTL_TX_OPERATION,
                    (((tx_fifo / DWMAC4_MTL_FIFO_UNIT_BYTES - 1).min(DWMAC4_MTL_TX_SIZE_MAX)
                        as u32)
                        << DWMAC4_MTL_TX_SIZE_SHIFT)
                        | DWMAC4_MTL_TX_QUEUE_ENABLE
                        | DWMAC4_MTL_TX_STORE_FORWARD,
                );
                self.write(DWMAC4_MTL_TX_WEIGHT, DWMAC4_MTL_TX_WEIGHT_DEFAULT);
                self.write(
                    DWMAC4_MTL_RX_OPERATION,
                    (((rx_fifo / DWMAC4_MTL_FIFO_UNIT_BYTES - 1).min(DWMAC4_MTL_RX_SIZE_MAX)
                        as u32)
                        << DWMAC4_MTL_RX_SIZE_SHIFT)
                        | DWMAC4_MTL_RX_STORE_FORWARD,
                );
                self.write(DWMAC4_RX_QUEUE_CONTROL0, DWMAC4_RX_QUEUE_DCB);
                self.write(DWMAC4_RX_QUEUE_CONTROL1, DWMAC4_MULTICAST_BROADCAST_QUEUE);

                // Step 2: 配置帧过滤器接收广播、全部组播及本机单播地址，关闭流控与硬件自动剥离 FCS。
                self.write(
                    DWMAC4_PACKET_FILTER,
                    MAC_FILTER_HASH_OR_PERFECT | MAC_FILTER_MULTICAST,
                );
                self.write(DWMAC4_TX_FLOW_CONTROL, 0);
                self.write(DWMAC4_RX_FLOW_CONTROL, 0);
                self.write(DWMAC4_INTERRUPT_ENABLE, 0);

                // Step 3: 配置 DMA 通道描述符跳过长度（DSL）、AXI 突发长度、单接收缓冲区大小及收发环 64 位基址与环长。
                self.write(DMA_BUS_MODE, DWMAC4_DMA_DESCRIPTOR_CACHE);
                self.write(
                    DWMAC4_DMA_CHANNEL_CONTROL,
                    inner.rx_ring.skip_length() << DWMAC4_DMA_SKIP_SHIFT,
                );
                self.write(
                    DWMAC4_DMA_TX_CONTROL,
                    (DMA_BURST_LENGTH << DWMAC4_DMA_BURST_SHIFT) | DWMAC4_DMA_SECOND_PACKET,
                );
                self.write(
                    DWMAC4_DMA_RX_CONTROL,
                    (DMA_BURST_LENGTH << DWMAC4_DMA_BURST_SHIFT)
                        | ((BUFFER_LEN as u32) << DWMAC4_DMA_RX_BUFFER_SHIFT),
                );

                self.write(
                    DWMAC4_DMA_TX_BASE_HIGH,
                    (inner.tx_ring.address(0) >> u32::BITS) as u32,
                );
                self.write(DWMAC4_DMA_TX_BASE, inner.tx_ring.address(0) as u32);
                self.write(
                    DWMAC4_DMA_RX_BASE_HIGH,
                    (inner.rx_ring.address(0) >> u32::BITS) as u32,
                );
                self.write(DWMAC4_DMA_RX_BASE, inner.rx_ring.address(0) as u32);
                self.write(DWMAC4_DMA_TX_RING_LENGTH, RING_LEN as u32 - 1);
                self.write(DWMAC4_DMA_RX_RING_LENGTH, RING_LEN as u32 - 1);
                self.write(DWMAC4_DMA_TX_TAIL, inner.tx_ring.address(0) as u32);
                self.write(
                    DWMAC4_DMA_RX_TAIL,
                    inner.rx_ring.address(RING_LEN - 1) as u32,
                );
                self.write(MAC_CONFIG, 0);
            }
            Generation::Dwmac1000 => {
                if self.read(DWMAC1000_DMA_HW_FEATURE) & DWMAC1000_ENHANCED_DESCRIPTORS == 0 {
                    return Err(GmacError::Resource(
                        "GMAC enhanced descriptors are unavailable",
                    ));
                }

                // DWMAC1000 启用 32 字节增强描述符（ATDS），DSL 按 4 字节字计数（非一致性 64B 隔离槽需跳过 8 个字），并开启收发 Store-and-Forward。
                self.write(
                    DMA_BUS_MODE,
                    (inner.rx_ring.skip_length() << DWMAC1000_DMA_SKIP_SHIFT)
                        | DWMAC1000_DMA_ALTERNATE_DESCRIPTOR
                        | (DMA_BURST_LENGTH << DWMAC1000_DMA_TX_BURST_SHIFT)
                        | (DMA_BURST_LENGTH << DWMAC1000_DMA_RX_BURST_SHIFT)
                        | DWMAC1000_DMA_SEPARATE_BURST,
                );
                self.write(DWMAC1000_DMA_RX_BASE, inner.rx_ring.address(0) as u32);
                self.write(DWMAC1000_DMA_TX_BASE, inner.tx_ring.address(0) as u32);
                self.write(
                    DWMAC1000_DMA_CONTROL,
                    DWMAC1000_DMA_RX_STORE_FORWARD
                        | DWMAC1000_DMA_TX_STORE_FORWARD
                        | DWMAC1000_DMA_SECOND_FRAME,
                );

                self.write(DWMAC1000_INTERRUPT_MASK, DWMAC1000_MAC_INTERRUPT_DISABLE);
                self.write(
                    DWMAC1000_FRAME_FILTER,
                    MAC_FILTER_HASH_OR_PERFECT | MAC_FILTER_MULTICAST,
                );
                self.write(DWMAC1000_FLOW_CONTROL, 0);
                self.write(DWMAC1000_MMC_RX_INTERRUPT_MASK, u32::MAX);
                self.write(DWMAC1000_MMC_TX_INTERRUPT_MASK, u32::MAX);
                self.write(DWMAC1000_MMC_RX_IPC_INTERRUPT_MASK, u32::MAX);
                self.write(MAC_CONFIG, DWMAC1000_MAC_DISABLE_CARRIER_SENSE);
            }
        }
        Ok(())
    }

    /// 置位 TX 与 RX DMA 启动控制位（必须在所有初始 RX 物理页和描述符环已完成同步并移交硬件之后调用）。
    fn start_dma(self) {
        match self.generation {
            Generation::Dwmac4 => {
                self.write(
                    DWMAC4_DMA_TX_CONTROL,
                    self.read(DWMAC4_DMA_TX_CONTROL) | DWMAC4_DMA_START_TX,
                );
                self.write(
                    DWMAC4_DMA_RX_CONTROL,
                    self.read(DWMAC4_DMA_RX_CONTROL) | DWMAC4_DMA_START_RX,
                );
            }
            Generation::Dwmac1000 => {
                self.write(
                    DWMAC1000_DMA_CONTROL,
                    self.read(DWMAC1000_DMA_CONTROL)
                        | DWMAC1000_DMA_START_TX
                        | DWMAC1000_DMA_START_RX,
                );
                self.write(DWMAC1000_DMA_RX_POLL, 0);
            }
        }
    }

    /// 通知硬件有新的接收或发送描述符就绪：DWMAC4 更新环形队列尾指针地址 `tail`，DWMAC1000 写轮询需求寄存器唤醒挂起的引擎。
    fn notify(self, receive: bool, tail: u64) {
        match self.generation {
            Generation::Dwmac4 => self.write(
                if receive {
                    DWMAC4_DMA_RX_TAIL
                } else {
                    DWMAC4_DMA_TX_TAIL
                },
                tail as u32,
            ),
            Generation::Dwmac1000 => self.write(
                if receive {
                    DWMAC1000_DMA_RX_POLL
                } else {
                    DWMAC1000_DMA_TX_POLL
                },
                0,
            ),
        }
    }

    /// 根据最新自协商链路结果配置 MAC 速率（10/100/1000M）、全/半双工及收发使能位；无物理载波（`None`）时暂停 MAC 收发。
    fn set_link(self, link: Option<Link>) {
        let (enable, duplex) = match self.generation {
            Generation::Dwmac4 => (DWMAC4_MAC_TRANSMIT | DWMAC4_MAC_RECEIVE, DWMAC4_MAC_DUPLEX),
            Generation::Dwmac1000 => (
                DWMAC1000_MAC_RECEIVE | DWMAC1000_MAC_TRANSMIT,
                DWMAC1000_MAC_DUPLEX,
            ),
        };
        let mut control =
            self.read(MAC_CONFIG) & !(enable | duplex | MAC_PORT_SELECT | MAC_FAST_ETHERNET);
        self.write(MAC_CONFIG, control);
        if let Some(link) = link {
            control |= enable;
            if link.full_duplex {
                control |= duplex;
            }
            if link.speed < SPEED_1000 {
                control |= MAC_PORT_SELECT;
            }
            if link.speed == SPEED_100 {
                control |= MAC_FAST_ETHERNET;
            }
            self.write(MAC_CONFIG, control);
        }
    }
}

/// `Dwmac` 驱动整体运行生命周期状态。
#[derive(Clone, Copy, Eq, PartialEq)]
enum Phase {
    /// 资源、物理页、描述符环与 PHY 已初始化就绪，但尚未向硬件启用 DMA 或开启外部中断。
    Prepared,
    /// DMA 与外部中断已启动，允许并发提交发送帧及后台处理收发与链路事件。
    Live,
    /// 初始化发布或运行期发生不可恢复故障，已关闭硬件中断并拒绝新发送请求，等待 `stop_sync` 回收资源。
    Faulted,
    /// 正在执行 `stop_sync` 同步停机流程：停止新生产者、注销回调并等待在途工作与 DMA 软复位完成。
    Stopping,
    /// 所有异步执行流已退出且硬件 DMA 软复位已确认完成，底层 DMA 物理页与描述符环已安全释放。
    Stopped,
}

/// 单个 RX/TX 描述符槽位及其关联物理页的所有权状态。
#[derive(Clone, Copy, Eq, PartialEq)]
enum SlotState {
    /// 槽位空闲且物理页归属 CPU，可被 `try_transmit`（TX）或初始化投递（RX）认领。
    Free,
    /// TX 生产者已认领该槽并将物理页临时移入栈上 `SlotLease`，正在锁外填充以太网帧并执行发前 DMA 同步。
    Preparing,
    /// TX 帧内容与发前 DMA 同步已完成，等待按环形队列连续顺序写入描述符环并置位 `OWN`。
    Ready,
    /// 描述符已置位 `OWN` 并发布给硬件，关联物理页处于设备 DMA 访问期，CPU 严禁读写其载荷。
    DeviceOwned,
    /// 硬件已清除 `OWN` 位（或正处于初始预投递过程），唯一后台 `progress` Work 已取出物理页在锁外执行 DMA 同步或协议栈上送。
    Processing,
}

/// 单个收发环形队列槽位的元数据与固定载荷页容器。
struct Slot {
    /// 当前槽位在生产者、DMA 引擎与后台消费者之间的流转状态。
    state: SlotState,
    /// 预分配的单个物理页缓冲区（容量覆盖 `BUFFER_LEN = 2048` 字节）；当处于 `Preparing` 或 `Processing` 锁外操作期时被临时 `take()` 到 `SlotLease` 中。
    page: Option<MappedPages>,
    /// `page` 在当前设备 `DmaDomain` 下的起始总线地址（单位：字节），在初始化时固定计算并复用。
    address: u64,
    /// 当前待发送帧的字节长度（仅 TX 槽位在 `Preparing` 至完成统计期间有效，范围 `FRAME_MIN..=FRAME_MAX`）。
    frame_len: usize,
}

/// 由 `Dwmac::state` 持有的核心收发队列与 DMA 内存集合。
///
/// 所有描述符环控制内存与 64 个收发数据物理页均集中存储在此结构体内；运行期报错不会提前移出或释放任何正在被硬件引用的物理页。
struct Inner {
    /// 当前控制器对应的 32 位受限 DMA 地址与缓存一致性域。
    domain: DmaDomain,
    /// 长度为 `RING_LEN` 的接收描述符环。
    rx_ring: Ring,
    /// 长度为 `RING_LEN` 的发送描述符环。
    tx_ring: Ring,
    /// 长度为 `RING_LEN` 的接收槽位数组，与 `rx_ring` 下标一一对应。
    rx: Vec<Slot>,
    /// 长度为 `RING_LEN` 的发送槽位数组，与 `tx_ring` 下标一一对应。
    tx: Vec<Slot>,
    /// 后台 Work 下一次检查并消费的接收环槽位下标（`0..RING_LEN`）。
    rx_next: usize,
    /// 发送生产者下一次认领空闲槽位的发送环下标（`0..RING_LEN`）。
    tx_claim: usize,
    /// 下一个待按序写入 `tx_ring` 并置位 `OWN` 移交硬件的发送环下标（`0..RING_LEN`）。
    tx_publish: usize,
    /// 后台 Work 下一次回收已完成发送帧的发送环下标（`0..RING_LEN`）。
    tx_complete: usize,
}

impl Drop for Inner {
    /// 析构前断言全部 64 个 RX/TX 载荷物理页均已收回 CPU 访问权，防止在 DMA 仍活跃时误释放物理内存导致内存踩踏。
    fn drop(&mut self) {
        assert!(
            self.rx.iter().chain(self.tx.iter()).all(|slot| slot
                .page
                .as_ref()
                .is_some_and(|page| !page.is_device_owned())),
            "dropping active GMAC payloads"
        );
    }
}

/// 受 `Dwmac::state` 自旋锁保护的可变运行时状态。
struct State {
    /// 驱动当前所处的生命周期状态。
    phase: Phase,
    /// 收发环形队列与 DMA 物理页集合；使用 `ManuallyDrop` 包装，确保仅在确认 DMA 软复位完成或从未启动时才显式析构。
    inner: Option<ManuallyDrop<Inner>>,
    /// 标记是否已向硬件发布过 DMA 地址或注册过运行态（为 `true` 时必须在释放物理页前成功执行硬件 DMA 软复位）。
    must_reset: bool,
    /// 标记 `stop_sync` 是否已完成定时器、外部中断与工作队列的同步关闭，支持 `stop_sync` 重入重试。
    closed: bool,
    /// 标记是否有待后台 `progress` Work 处理的数据收发事件。
    pending: bool,
    /// 自上次 `progress` Work 取走以来累积的硬件 DMA 中断状态位或掩码。
    reason: u32,
    /// 标记秒级链路定时器是否已到期、需要在后台 `progress` Work 中执行一次 MDIO PHY 采样。
    phy_due: bool,
    /// 最近一次采样确认的物理链路速率与双工状态；网线断开或自协商未完成时为 `None`。
    link: Option<Link>,
    /// 记录首次触发停机或故障状态的错误原因，供后续失败的发送调用直接返回。
    failure: Option<GmacError>,
    /// 网卡收发包数、字节数、错误数与丢包数累计统计。
    statistics: InterfaceStats,
}

/// Synopsys DWMAC 千兆以太网控制器驱动实例。
///
/// 上层网络子系统持有一份稳定的 `Arc<Dwmac>` 强引用直至 `stop_sync` 成功返回；本结构体对 `NetStack` 仅保存 `Weak` 弱引用以打破循环引用，并将 `Arc::as_ptr` 原始地址作为中断、Work 与 Timer 的回调上下文。
pub struct Dwmac {
    /// 保护设备运行状态、队列游标、中断挂起标志及统计计数的短临界区自旋锁（持锁期间严禁睡眠）。
    state: SpinLock<State>,
    /// 串行化 `start` 与 `stop_sync` 控制面操作的可睡眠互斥锁（锁层级位于 `state` 与 `registration` 之外）。
    control: SleepMutex<()>,
    /// 保存已注册外部中断句柄的自旋锁，用于在启动时开启中断及停机时取出执行 `unregister_sync`（不与 `state` 嵌套）。
    registration: SpinLock<Option<IrqRegistration>>,
    /// 绑定在 CPU0 网络工作队列上的唯一后台任务，集中执行 TX 回收、RX 收包上送与 PHY 链路刷新。
    progress: Once<Pin<Arc<Work>>>,
    /// 绑定在 CPU0 上的秒级精确定时器，周期性置位 `phy_due` 并唤醒 `progress` Work 采样链路状态。
    link_timer: Once<Pin<Arc<Timer>>>,
    /// 指向所属网络协议栈的弱引用，用于上送以太网帧（`receive_ethernet`）与唤醒待发送队列（`kick_egress`）。
    net: Weak<NetStack>,
    /// 经边界校验的 MAC 控制器 MMIO 寄存器访问句柄。
    registers: Registers,
    /// 从板级设备树解析并规范化后的 GMAC 硬件资源（MMIO 区间、时钟、复位、syscon 与 PHY 属性）。
    resources: GmacResources,
    /// 映射后的内核外部中断号。
    interrupt: IrqNumber,
    /// 设备树指定或从硬件寄存器读出的合法 6 字节单播 MAC 地址。
    mac: [u8; 6],
    /// 板载 Clause 22 PHY 管理对象（仅在 `prepare` 期间与单实例 `progress` Work 中串行访问）。
    phy: Phy,
}

impl Dwmac {
    /// 在普通可睡眠任务上下文中完成硬件探测、PHY 配置、DMA 物理页与描述符环分配以及工作队列/定时器创建。
    ///
    /// 本函数仅完成离线资源准备并执行首次 DMA 软件复位，此时尚未向硬件写入描述符环地址，也未注册或开启外部中断。
    ///
    /// # Safety
    /// 1. `resources` 必须来自平台唯一的设备树硬件清单，且相关的 MMIO 寄存器窗口、syscon、时钟控制器及引脚复用寄存器均已建立永久强序映射；
    /// 2. 选定的 MAC 控制器、板载 PHY 及关联的板级控制位必须由本驱动实例独占，无其他驱动并发访问；
    /// 3. 调用方（网络子系统根对象）必须在调用 `start` 之前持续持有返回的 `Arc<Self>` 强引用，并保持存活直至所有发送生产者、定时器回调、工作队列任务和中断处理函数彻底执行完毕退出且 `stop_sync` 确认硬件 DMA 复位完成。
    pub unsafe fn prepare(
        net: &NetStackRef,
        resources: &GmacResources,
        cpu: usize,
    ) -> Result<Arc<Self>, GmacError> {
        crate::sched::assert_switchable();
        assert_eq!(cpu, 0, "GMAC progress must use CPU0");
        // Step 1: 识别平台对应的控制器代数，校验 MMIO 寄存器区间对齐与最小长度，并映射外部中断源。
        let generation = match boot::info().platform {
            Platform::VisionFive2 => Generation::Dwmac4,
            Platform::Ls2k1000la => Generation::Dwmac1000,
            Platform::QemuVirt => {
                return Err(GmacError::Resource("QEMU has no selected native GMAC"))
            }
        };
        let minimum = match generation {
            Generation::Dwmac4 => DWMAC4_DMA_STATUS + REGISTER_BYTES,
            Generation::Dwmac1000 => DWMAC1000_MMIO_MIN_BYTES,
        };
        if resources.registers.start == 0
            || resources.registers.start % REGISTER_BYTES != 0
            || resources
                .registers
                .end
                .checked_sub(resources.registers.start)
                .is_none_or(|length| length < minimum)
        {
            return Err(GmacError::Resource("GMAC MMIO resource is incomplete"));
        }

        let interrupt = irq::map_source(resources.interrupt_source).map_err(GmacError::Irq)?;
        // Step 2: 应用板级时钟/复位/引脚设置，配置 JH7110 接口模式寄存器，并校验硬件核心版本号与合法单播 MAC 地址。
        // SAFETY: 调用方已按 `prepare` 的 `# Safety` 契约保证 `resources.setup` 中的时钟、复位和引脚控制器窗口永久映射且由本设备独占。
        unsafe { board_setup::apply_setup(&resources.setup, Duration::ZERO) }
            .map_err(GmacError::Resource)?;
        let registers = Registers {
            region: resources.registers,
            generation,
        };
        // SAFETY: 调用方保证 `resources.syscon` 已建立永久强序映射，且目标接口模式位域由当前 GMAC 独占。
        unsafe { configure_interface(resources, generation) }?;
        let version = registers.read(match generation {
            Generation::Dwmac4 => DWMAC4_VERSION,
            Generation::Dwmac1000 => DWMAC1000_VERSION,
        }) & MAC_VERSION_MASK;
        if !matches!(
            (generation, version),
            (Generation::Dwmac4, DWMAC_CORE_5_10 | DWMAC_CORE_5_20)
                | (Generation::Dwmac1000, DWMAC_CORE_3_70)
        ) {
            return Err(GmacError::Resource(
                "selected GMAC hardware version is unsupported",
            ));
        }

        let mac = resources
            .mac_address
            .unwrap_or_else(|| registers.mac_address());
        if !valid_mac(mac) {
            return Err(GmacError::Resource(
                "GMAC has no valid DT or hardware MAC address",
            ));
        }

        // Step 3: 暂停旧 DMA 收发，计算 MDIO CSR 分频档位并初始化 PHY（确保无网线时仍输出 RXC 时钟），随后执行首次 MAC DMA 软件复位。
        registers.stop_dma();
        let csr_divider = if generation == Generation::Dwmac4 {
            if resources.setup.clocks.count == 0 {
                return Err(GmacError::Resource("GMAC CSR clock is missing"));
            }
            // SAFETY: 前一步 `apply_setup` 已启用该时钟条目且调用方保证时钟控制器 MMIO 永久映射有效。
            let frequency = unsafe { board_setup::clock_rate(&resources.setup.clocks.entries[0]) }
                .map_err(GmacError::Resource)?;
            phy::clock_range(frequency)?
        } else {
            // LS2K1000LA 平台的 GMAC CSR 输入时钟位于 100–150 MHz 区间，固定采用标准 /62 分频档位。
            phy::CSR_100_150M
        };
        let phy = Phy::prepare(registers, resources, csr_divider)?;
        // 板级设置已开启 CSR/MDIO 时钟，且 `Phy::prepare` 已解除 PHY 掉电/隔离并禁止无载波切钟；
        // 此时 RGMII RXC 参考时钟已稳定提供，可以安全触发依赖 RXC 的首次 DMA 软件复位（SWR）。
        registers.request_reset();
        registers.wait_reset()?;

        // Step 4: 校验非一致性缓存行隔离约束，创建 32 位 DMA 域，一次性预分配全部描述符环与 64 个收发数据物理页。
        let line = boot::info().board.cache_line_size;
        if !resources.dma_coherent
            && (line == 0
                || descriptors::ISOLATED_DESCRIPTOR_BYTES < line
                || !line.is_power_of_two())
        {
            return Err(GmacError::Resource(
                "GMAC cacheline does not fit descriptor isolation",
            ));
        }
        // 两块实板控制器的 DMA 描述符基址与部分路径受 32 位总线地址限制，因此将 DMA 掩码截断在 `u32::MAX` 以内。
        let domain = DmaDomain::new(
            resources.dma_mask.min(u32::MAX as u64),
            0,
            usize::MAX,
            0,
            if resources.dma_coherent { 1 } else { line },
            resources.dma_coherent,
        )?;
        let inner = prepare_inner(domain, generation, resources.dma_coherent)?;

        // Step 5: 构造 `Arc<Dwmac>` 实例，并将稳定的裸指针上下文绑定到 CPU0 的后台 `Work` 与秒级链路 `Timer`。
        let nic = Arc::try_new(Self {
            state: SpinLock::new(State {
                phase: Phase::Prepared,
                inner: Some(ManuallyDrop::new(inner)),
                must_reset: false,
                closed: false,
                pending: false,
                reason: 0,
                phy_due: false,
                link: None,
                failure: None,
                statistics: InterfaceStats::default(),
            }),
            control: SleepMutex::new(()),
            registration: SpinLock::new(None),
            progress: Once::new(),
            link_timer: Once::new(),
            net: Arc::downgrade(net),
            registers,
            resources: *resources,
            interrupt,
            mac,
            phy,
        })
        .map_err(|_| GmacError::NoMemory)?;

        let context = Arc::as_ptr(&nic) as usize;
        let work = Work::try_new(net.work_queue(), cpu, progress_callback, context)
            .map_err(|_| GmacError::NoMemory)?;
        let timer = Timer::try_new(cpu, TimerKind::Precise, link_expired, context)
            .map_err(|_| GmacError::NoMemory)?;
        nic.progress.call_once(|| work);
        nic.link_timer.call_once(|| timer);
        Ok(nic)
    }

    /// 注册外部中断处理函数、同步并发布初始接收描述符环，随后启动硬件 DMA 引擎与中断（需在持有稳定 `Arc<Self>` 期间调用）。
    ///
    /// 若发布中途失败，所有已分配的物理页继续保留在 `Inner` 中并转入 `Faulted` 状态，留待后续 `stop_sync` 统一复位回收。
    pub fn start(self: &Arc<Self>) -> Result<(), GmacError> {
        let _control = self.control.lock();
        if self.state.lock().phase != Phase::Prepared {
            return Err(GmacError::Io("GMAC is not prepared"));
        }
        // Step 1: 以初始禁用（disabled）状态注册外部中断回调，此时硬件不会立即派发中断。
        // SAFETY:
        // 1. `Arc::as_ptr(self)` 指向堆上地址固定的 `Dwmac` 实例；
        // 2. 上层调用方强引用保活该 `Arc`，直到 `stop_sync` 调用 `unregister_sync` 等待所有中断处理函数彻底退出后才会释放。
        let registration =
            unsafe { irq::register(self.interrupt, irq_callback, Arc::as_ptr(self) as *mut ()) }
                .map_err(GmacError::Irq)?;
        *self.registration.lock() = Some(registration);

        // Step 2: 标记硬件需要复位清理，将 32 个接收缓冲区同步移交设备并写入 MAC/DMA 配置寄存器。
        self.state.lock().must_reset = true;
        if let Err(error) = self.publish() {
            self.record_failure(error);
            return Err(error);
        }

        // Step 3: 切换至 `Live` 运行态，启动硬件收发 DMA、开启外部中断线，并排队首次后台任务完成初始链路采样。
        {
            let mut state = self.state.lock();
            state.phase = Phase::Live;
            state.pending = true;
            state.phy_due = true;
        }
        self.registers.start_dma();
        self.registration.lock().as_ref().unwrap().enable();
        self.queue_progress();
        Ok(())
    }

    /// 返回初始化期间校验通过的 6 字节本机单播 MAC 地址。
    pub fn mac_address(&self) -> [u8; 6] {
        self.mac
    }

    /// 读取并返回当前累计的网卡收发帧与错误统计快照。
    pub fn statistics(&self) -> InterfaceStats {
        self.state.lock().statistics
    }

    /// 尝试认领一个空闲发送槽并在锁外调用 `writer` 填充长度为 `frame_len` 字节的以太网帧，随后发布给硬件发送。
    ///
    /// - 当发送环满无空闲槽位时，不调用 `writer` 并返回 `Ok(false)`；成功提交返回 `Ok(true)`。
    /// - **回调契约**：`writer` 在自旋锁外执行，但不得睡眠、不得访问用户态内存或反向重入网络栈，且不得在返回后保留传入的可变字节切片。
    pub fn try_transmit(
        &self,
        frame_len: usize,
        writer: impl FnOnce(&mut [u8]),
    ) -> Result<bool, GmacError> {
        if !(FRAME_MIN..=FRAME_MAX).contains(&frame_len) {
            return Err(GmacError::Io("invalid Ethernet frame length"));
        }
        // Step 1: 在短自旋锁临界区内认领下一个 `Free` 发送槽，将其标记为 `Preparing` 并取出物理页句柄。
        let (index, page, domain) = {
            let mut state = self.state.lock();
            if state.phase != Phase::Live {
                return Err(state.failure.unwrap_or(GmacError::Io("GMAC is stopping")));
            }
            let inner: &mut Inner = state.inner.as_mut().unwrap();
            let index = inner.tx_claim;
            let slot = &mut inner.tx[index];
            if slot.state != SlotState::Free {
                return Ok(false);
            }
            slot.state = SlotState::Preparing;
            slot.frame_len = frame_len;
            inner.tx_claim = (index + 1) % RING_LEN;
            (index, slot.page.take().unwrap(), inner.domain)
        };

        // Step 2: 用 `SlotLease` 托管物理页（异常早退时自动归槽），在锁外调用 `writer` 填入帧字节并执行发前 DMA 同步。
        let mut lease = SlotLease {
            nic: self,
            receive: false,
            index,
            page: Some(page),
        };
        {
            let bytes = lease.page.as_mut().unwrap().bytes_mut();
            // SAFETY:
            // 1. 前置检查保证 `frame_len <= FRAME_MAX <= BUFFER_LEN`，且预分配物理页已在分配时完整清零；
            // 2. 当前槽位处于 `Preparing` 状态，物理页已从槽中 `take()` 由栈上 `lease` 独占，硬件尚未获取该槽控制权，无并发别名。
            let bytes =
                unsafe { slice::from_raw_parts_mut(bytes.as_mut_ptr().cast::<u8>(), frame_len) };
            writer(bytes);
        }

        // SAFETY: 上方代码块已结束，`writer` 借用的可变切片已销毁，CPU 侧不再保留任何指向该页的引用；`stop_sync` 会等待所有 `Preparing` 槽位退出后才复位回收。
        if let Err(error) = unsafe { lease.page.as_mut().unwrap().begin_device_access(domain) } {
            let error = GmacError::from(error);
            self.record_failure(error);
            return Err(error);
        }

        // Step 3: 重新加锁归还物理页并置为 `Ready`，将环上从 `tx_publish` 起连续就绪的槽位按序置位 `OWN` 移交硬件并敲门通知。
        let result = {
            let mut state = self.state.lock();
            let live = state.phase == Phase::Live;
            let inner: &mut Inner = state.inner.as_mut().unwrap();
            let slot = &mut inner.tx[index];
            slot.page = lease.page.take();
            slot.state = SlotState::Ready;
            if live {
                publish_ready(inner)
            } else {
                Err(GmacError::Io("GMAC stopped during TX preparation"))
            }
        };
        match result {
            Ok(Some(tail)) => self.notify(false, tail),
            Ok(None) => (),
            Err(error) => {
                self.record_failure(error);
                return Err(error);
            }
        }
        Ok(true)
    }

    /// 同步停止网卡收发、等待所有异步回调退出、执行硬件 DMA 软复位并收回全部物理页的 CPU 访问权（运行于可睡眠任务上下文）。
    ///
    /// 若硬件 DMA 软复位在超时时间内未能确认完成（返回 `Err(GmacError::StopUnconfirmed)`），整个 `Inner` 及其物理页将原封不动保留以防止硬件踩踏已释放内存；调用方后续可再次调用 `stop_sync` 继续等待复位完成。
    pub fn stop_sync(&self) -> Result<(), GmacError> {
        crate::sched::assert_switchable();
        let _control = self.control.lock();
        // Step 1: 切入 `Stopping` 状态并屏蔽通道中断，在锁外同步关闭链路定时器、注销外部中断并等待后台 `progress` Work 彻底退出。
        let close = {
            let mut state = self.state.lock();
            if state.phase == Phase::Stopped {
                return Ok(());
            }
            state.phase = Phase::Stopping;
            self.registers.interrupts(false);
            let close = !state.closed;
            state.closed = true;
            close
        };
        if close {
            self.link_timer.get().unwrap().as_ref().close_sync();
            let registration = self.registration.lock().take();
            if let Some(registration) = registration {
                registration.unregister_sync();
            }
            self.progress.get().unwrap().as_ref().close_sync();
        }

        // Step 2: 在不持有自旋锁的情况下睡眠等待所有已在锁外执行 `writer` 的 `Preparing` 发送生产者退出临界流程。
        loop {
            let busy = {
                let state = self.state.lock();
                state
                    .inner
                    .as_ref()
                    .unwrap()
                    .tx
                    .iter()
                    .any(|slot| slot.state == SlotState::Preparing)
            };
            if !busy {
                break;
            }
            timer::sleep(Duration::from_millis(1));
        }
        // Step 3: 若曾向硬件发布过队列或中断，停止 MAC/DMA 收发并触发软件复位（`SWR`），等待硬件确认在途 DMA 全部终止。
        if self.state.lock().must_reset {
            self.registers.stop_dma();
            self.registers.request_reset();
            self.registers.wait_reset()?;
        }

        // Step 4: 此时中断、Work、Timer 与发送生产者均已退出且硬件 DMA 已复位停止；逐一收回全部 64 个收发物理页的 CPU 访问权。
        for receive in [true, false] {
            for index in 0..RING_LEN {
                let page = {
                    let mut state = self.state.lock();
                    let inner: &mut Inner = state.inner.as_mut().unwrap();
                    let slot = if receive {
                        &mut inner.rx[index]
                    } else {
                        &mut inner.tx[index]
                    };
                    slot.page
                        .take()
                        .expect("GMAC page missing after all CPU tails")
                };
                let mut lease = SlotLease {
                    nic: self,
                    receive,
                    index,
                    page: Some(page),
                };
                if lease.page.as_ref().unwrap().is_device_owned() {
                    // SAFETY:
                    // 1. 前序步骤已完成 `close_sync` / `unregister_sync`、`Preparing` 排空以及硬件 `SWR` 软复位清零确认；
                    // 2. 设备侧 DMA 与其余 CPU 路径均已彻底停止访问该物理页，栈上 `lease` 独占持有该页直至同步完成归槽。
                    unsafe { lease.page.as_mut().unwrap().finish_device_access() }?;
                }
            }
        }

        // Step 5: 标记设备进入 `Stopped` 终态，取出并正式析构 `Inner`，将所有物理页与描述符环控制内存归还内核分配器。
        let owner = {
            let mut state = self.state.lock();
            state.must_reset = false;
            state.phase = Phase::Stopped;
            state.inner.take().unwrap()
        };
        drop(ManuallyDrop::into_inner(owner));
        Ok(())
    }

    /// 在启动流程中将全部 32 个接收缓冲区逐一在锁外同步移交设备访问、装填接收描述符环，并完成 MAC/DMA 寄存器初始化。
    fn publish(&self) -> Result<(), GmacError> {
        for index in 0..RING_LEN {
            let (page, domain) = {
                let mut state = self.state.lock();
                let inner: &mut Inner = state.inner.as_mut().unwrap();
                inner.rx[index].state = SlotState::Processing;
                (inner.rx[index].page.take().unwrap(), inner.domain)
            };
            let mut lease = SlotLease {
                nic: self,
                receive: true,
                index,
                page: Some(page),
            };
            // SAFETY: 接收物理页在预分配时已清零且当前由栈上 `lease` 独占，无任何 CPU 切片引用；此时尚未向硬件写入描述符环基址。
            unsafe { lease.page.as_mut().unwrap().begin_device_access(domain) }?;
            let mut state = self.state.lock();
            let inner: &mut Inner = state.inner.as_mut().unwrap();
            let slot = &mut inner.rx[index];
            slot.page = lease.page.take();
            slot.state = SlotState::DeviceOwned;
            inner.rx_ring.arm_rx(index, slot.address)?;
        }

        let state = self.state.lock();
        self.registers
            .configure(state.inner.as_ref().unwrap(), self.mac)
    }

    /// 外部硬中断处理入口：仅应答本设备 DMA 中断状态位、屏蔽通道中断并唤醒后台 `progress` Work，不做帧拷贝、内存分配或睡眠等待。
    fn handle_interrupt(&self) -> bool {
        let schedule = {
            let mut state = self.state.lock();
            let reason = self.registers.acknowledge();
            if reason == 0 {
                return false;
            }
            self.registers.interrupts(false);
            state.reason |= reason;
            state.pending = true;
            state.phase == Phase::Live
        };
        if schedule {
            self.queue_progress();
        }
        true
    }

    /// 运行在 CPU0 网络工作队列中的唯一后台处理函数：检查致命总线错误、按需刷新 PHY 链路，并按预算回收已完成发送槽与处理接收帧。
    fn progress(&self) -> Result<(), GmacError> {
        // Step 1: 在自旋锁内原子取走并清零挂起的数据标志、累积中断原因与 PHY 定时采样标志。
        let (data, reason, phy_due) = {
            let mut state = self.state.lock();
            if state.phase != Phase::Live {
                return Ok(());
            }
            let values = (state.pending, state.reason, state.phy_due);
            state.pending = false;
            state.reason = 0;
            state.phy_due = false;
            values
        };
        // Step 2: 优先检查不可恢复的 DMA 致命总线错误；若秒级定时器到期则通过 MDIO 采样并更新物理链路。
        if self.registers.fatal(reason) {
            return Err(GmacError::Io("GMAC fatal DMA bus error"));
        }
        if phy_due {
            self.refresh_link()?;
        }
        // Step 3: 回收所有硬件已归还（`OWN == 0`）的发送槽并唤醒协议栈待发队列，随后按 `RX_BUDGET` 预算收包上送并重武装中断。
        if data {
            let mut completed = 0;
            while completed < RING_LEN && self.complete_tx()? {
                completed += 1;
            }
            if completed != 0 {
                if let Some(net) = self.net.upgrade() {
                    net.kick_egress();
                }
            }

            let mut received = 0;
            while received < RX_BUDGET && self.receive_one()? {
                received += 1;
            }
            self.rearm(received == RX_BUDGET)?;
        }
        Ok(())
    }

    /// 尝试按 `tx_complete` 游标顺序回收一个硬件已完成发送（`OWN == 0`）的槽位：锁外收回物理页 DMA 访问权，归还空闲槽并更新发送统计。
    fn complete_tx(&self) -> Result<bool, GmacError> {
        let (index, page, status, length) = {
            let mut state = self.state.lock();
            if state.phase != Phase::Live {
                return Ok(false);
            }
            let inner: &mut Inner = state.inner.as_mut().unwrap();
            let index = inner.tx_complete;
            if inner.tx[index].state != SlotState::DeviceOwned {
                return Ok(false);
            }
            let status = inner.tx_ring.status(index)?;
            if device_owned(status) {
                return Ok(false);
            }
            let slot = &mut inner.tx[index];
            slot.state = SlotState::Processing;
            (index, slot.page.take().unwrap(), status, slot.frame_len)
        };
        let mut lease = SlotLease {
            nic: self,
            receive: false,
            index,
            page: Some(page),
        };
        // SAFETY:
        // 1. 前置检查已确认 `tx_ring.status(index)` 的 `OWN` 位清零，硬件已结束对该单帧缓冲区的 DMA 读取；
        // 2. 单实例 `progress` Work 是唯一的完成消费者，槽位保持 `Processing` 状态且物理页由 `lease` 独占，`stop_sync` 会先等待本 Work 退出。
        unsafe { lease.page.as_mut().unwrap().finish_device_access() }?;

        let mut state = self.state.lock();
        let inner: &mut Inner = state.inner.as_mut().unwrap();
        inner.tx[index].page = lease.page.take();
        inner.tx[index].state = SlotState::Free;
        inner.tx_complete = (index + 1) % RING_LEN;
        if descriptors::transmit_error(status) {
            state.statistics.tx_errors = state.statistics.tx_errors.wrapping_add(1);
        } else {
            state.statistics.tx_packets = state.statistics.tx_packets.wrapping_add(1);
            state.statistics.tx_bytes = state.statistics.tx_bytes.wrapping_add(length as u64);
        }
        Ok(true)
    }

    /// 尝试消费 `rx_next` 指向的一个已完成接收槽：锁外收回物理页 DMA 访问权、将合法以太网帧送交协议栈，随后将同一物理页重新同步挂回接收环。
    fn receive_one(&self) -> Result<bool, GmacError> {
        // Step 1: 在锁内检查当前 `rx_next` 槽是否已由硬件清除 `OWN` 位；若已完成则置为 `Processing` 并取出物理页。
        let (index, page, status, domain) = {
            let mut state = self.state.lock();
            if state.phase != Phase::Live {
                return Ok(false);
            }
            let inner: &mut Inner = state.inner.as_mut().unwrap();
            let index = inner.rx_next;
            let slot = &mut inner.rx[index];
            if slot.state != SlotState::DeviceOwned {
                return Ok(false);
            }
            let status = inner.rx_ring.status(index)?;
            if device_owned(status) {
                return Ok(false);
            }
            slot.state = SlotState::Processing;
            (index, slot.page.take().unwrap(), status, inner.domain)
        };
        let mut lease = SlotLease {
            nic: self,
            receive: true,
            index,
            page: Some(page),
        };
        // Step 2: 在锁外收回物理页的 CPU 访问权，校验帧长与错误位后借用载荷切片送入协议栈 `receive_ethernet`。
        // SAFETY:
        // 1. 前置检查确认该接收描述符的 `OWN` 位已清零，硬件已结束对该缓冲区的 DMA 写入；
        // 2. 单实例 `progress` Work 独占持有 `lease` 中的物理页，`stop_sync` 在本函数返回前不会并发介入。
        unsafe { lease.page.as_mut().unwrap().finish_device_access() }?;
        if let Some(length) = descriptors::received(self.registers.generation, status) {
            {
                let mut state = self.state.lock();
                state.statistics.rx_packets = state.statistics.rx_packets.wrapping_add(1);
                state.statistics.rx_bytes = state.statistics.rx_bytes.wrapping_add(length as u64);
            }
            // SAFETY:
            // 1. 物理页在分配时已完整清零，且上一行 `finish_device_access` 已完成收包后的缓存失效与内存同步；
            // 2. `descriptors::received` 已验证 `length <= FRAME_MAX <= BUFFER_LEN`，且协议栈 `receive_ethernet` 仅在本次同步调用期间只读借用切片。
            let bytes = unsafe { lease.page.as_ref().unwrap().bytes() };
            let accepted = self
                .net
                .upgrade()
                .is_some_and(|net| net.receive_ethernet(&bytes[..length]).is_ok());
            if !accepted {
                let mut state = self.state.lock();
                state.statistics.rx_dropped = state.statistics.rx_dropped.wrapping_add(1);
            }
        } else {
            let mut state = self.state.lock();
            state.statistics.rx_errors = state.statistics.rx_errors.wrapping_add(1);
            state.statistics.rx_dropped = state.statistics.rx_dropped.wrapping_add(1);
        }

        // Step 3: 若设备仍处于 `Live` 运行态，重新将物理页移交设备访问，写回接收描述符 `OWN = 1` 并推进尾指针通知硬件。
        if self.state.lock().phase != Phase::Live {
            return Ok(true);
        }
        // SAFETY: 上方 `bytes` 只读切片作用域已结束，无残留 CPU 引用；`lease` 独占持有该物理页，且 `stop_sync` 必须等待当前 Work 任务执行完毕退出。
        unsafe { lease.page.as_mut().unwrap().begin_device_access(domain) }?;
        let tail = {
            let mut state = self.state.lock();
            let live = state.phase == Phase::Live;
            let inner: &mut Inner = state.inner.as_mut().unwrap();
            let slot = &mut inner.rx[index];
            slot.page = lease.page.take();
            slot.state = SlotState::DeviceOwned;
            if !live {
                return Ok(true);
            }
            inner.rx_ring.arm_rx(index, slot.address)?;
            inner.rx_next = (index + 1) % RING_LEN;
            inner.rx_ring.address(index)
        };
        self.notify(true, tail);
        Ok(true)
    }

    /// 在单轮收发批处理结束后重新开启通道中断，并立即复查中断状态与队头描述符以消除开中断前的早完成竞态；若仍有待处理工作或本轮耗尽预算，则再次排队 `progress` Work。
    fn rearm(&self, budget_full: bool) -> Result<(), GmacError> {
        let again = {
            let mut state = self.state.lock();
            if state.phase != Phase::Live {
                return Ok(());
            }
            self.registers.interrupts(true);
            let reason = self.registers.acknowledge();
            state.reason |= reason;
            let inner: &Inner = state.inner.as_ref().unwrap();
            let rx = &inner.rx[inner.rx_next];
            let tx = &inner.tx[inner.tx_complete];
            let ready = (rx.state == SlotState::DeviceOwned
                && !device_owned(inner.rx_ring.status(inner.rx_next)?))
                || (tx.state == SlotState::DeviceOwned
                    && !device_owned(inner.tx_ring.status(inner.tx_complete)?));
            if budget_full || reason != 0 || ready || state.pending {
                self.registers.interrupts(false);
                state.pending = true;
                true
            } else {
                false
            }
        };
        if again {
            self.queue_progress();
        }
        Ok(())
    }

    /// 在后台 `progress` Work 中采样 PHY 协商状态；若链路速率或双工发生变化则在锁外重配 PHY/TX 时钟与 MAC 控制寄存器，最后重装下一秒链路定时器。
    fn refresh_link(&self) -> Result<(), GmacError> {
        let link = self.phy.sample()?;
        let changed = self.state.lock().link != link;
        if changed {
            self.registers.set_link(None);
            if let Some(link) = link {
                self.phy.configure_link(link)?;
                self.update_tx_clock(link)?;
            }
            self.registers.set_link(link);
            self.state.lock().link = link;
            if let Some(net) = self.net.upgrade() {
                net.kick_egress();
            }
        }

        let state = self.state.lock();
        if state.phase == Phase::Live {
            self.link_timer
                .get()
                .unwrap()
                .as_ref()
                .arm(time::now().checked_add(Duration::from_secs(1)).unwrap())
                .map_err(|_| GmacError::Io("GMAC PHY timer is closed"))?;
        }
        Ok(())
    }

    /// 当 JH7110 DWMAC4 使用 SoC 内部发送时钟源时，根据协商达成的千兆/百兆/十兆速率将 TX 时钟切换为 125 / 25 / 2.5 MHz。
    fn update_tx_clock(&self, link: Link) -> Result<(), GmacError> {
        if self.registers.generation != Generation::Dwmac4 || self.resources.tx_use_rgmii_clock {
            return Ok(());
        }
        if self.resources.setup.clocks.count < STARFIVE_TX_CLOCK_INDEX + 1 {
            return Err(GmacError::Resource("GMAC TX clock is missing"));
        }
        let mut setup = DeviceSetup::default();
        setup.clocks.count = 1;
        setup.clocks.entries[0] = self.resources.setup.clocks.entries[STARFIVE_TX_CLOCK_INDEX];
        setup.clocks.entries[0].rate = Some(match link.speed {
            SPEED_1000 => RGMII_GIGABIT_CLOCK_HZ,
            SPEED_100 => MII_FAST_ETHERNET_CLOCK_HZ,
            SPEED_10 => MII_ETHERNET_CLOCK_HZ,
            _ => return Err(GmacError::Io("invalid negotiated GMAC speed")),
        });
        // SAFETY:
        // 1. `self.resources.setup.clocks` 来自 `prepare` 验证过的规范化硬件清单，关联时钟控制器 MMIO 具有永久强序映射；
        // 2. 构造的局部 `setup` 仅修改本 GMAC 独占的 TX 时钟频率，不触碰复位或引脚；且仅在唯一的 `progress` Work 中串行调用。
        unsafe { board_setup::apply_setup(&setup, Duration::ZERO) }.map_err(GmacError::Resource)
    }

    /// 在持有 `state` 自旋锁的短临界区内读取最新发布位置并更新硬件环形队列尾指针（或轮询需求寄存器），防止并发发送者将旧尾指针覆盖到新边界之前。
    fn notify(&self, receive: bool, tail: u64) {
        let state = self.state.lock();
        if state.phase != Phase::Live {
            return;
        }
        let tail = if receive {
            tail
        } else {
            let inner: &Inner = state.inner.as_ref().unwrap();
            let next = if inner.tx_publish == 0 {
                RING_LEN
            } else {
                inner.tx_publish
            };
            inner.tx_ring.address(next)
        };
        self.registers.notify(receive, tail);
    }

    /// 将后台 `progress` Work 投递到网络工作队列（若已在队列中或正在运行则由 `Work` 内部自动合并调度）。
    fn queue_progress(&self) {
        let _ = self.progress.get().unwrap().as_ref().queue();
    }

    /// 记录首次出现的驱动故障原因，将活跃或就绪状态切换为 `Faulted` 并立即关闭硬件通道中断，保留全部 DMA 物理页等待 `stop_sync` 安全回收。
    fn record_failure(&self, error: GmacError) {
        let mut state = self.state.lock();
        state.failure.get_or_insert(error);
        if matches!(state.phase, Phase::Live | Phase::Prepared) {
            state.phase = Phase::Faulted;
        }
        self.registers.interrupts(false);
    }
}

impl Drop for Dwmac {
    /// 析构 `Dwmac` 实例：断言硬件未遗留未确认的 DMA 复位需求且不处于运行态，随后释放尚未析构的 `Inner` 物理页资源。
    fn drop(&mut self) {
        let owner = {
            let mut state = self.state.lock();
            assert!(
                !state.must_reset,
                "dropping GMAC before confirmed DMA reset"
            );
            assert!(
                matches!(state.phase, Phase::Prepared | Phase::Stopped),
                "dropping live GMAC context"
            );
            state.inner.take()
        };
        if let Some(owner) = owner {
            drop(ManuallyDrop::into_inner(owner));
        }
    }
}

/// 锁外操作期间临时托管单个槽位 `MappedPages` 物理页的栈上 RAII 守卫。
///
/// 当在自旋锁外执行帧拷贝、协议栈上送或 DMA 缓存同步时，若中途发生错误提前返回，`Drop` 实现会自动将物理页按其当前真实的 DMA 所有权状态放回原槽，防止物理页丢失或对仍在途的 DMA 缓冲区提前执行析构。
struct SlotLease<'a> {
    /// 所属网卡驱动实例引用，用于在 `Drop` 归还物理页时重新获取 `state` 自旋锁。
    nic: &'a Dwmac,
    /// 槽位所属方向：`true` 表示归属接收环 `inner.rx`，`false` 表示归属发送环 `inner.tx`。
    receive: bool,
    /// 槽位在对应接收或发送环中的下标（`0..RING_LEN`）。
    index: usize,
    /// 临时托管的物理页缓冲区；正常路径通过 `take()` 显式归槽，异常回退路径在 `drop` 中自动归槽。
    page: Option<MappedPages>,
}

impl Drop for SlotLease<'_> {
    /// 若正常路径未显式取走 `page`，则在栈展开或错误返回时自动加锁将物理页归还回原槽，并按 `is_device_owned()` 恢复对应的 `DeviceOwned` 或 `Free` 状态。
    fn drop(&mut self) {
        if let Some(page) = self.page.take() {
            let mut state = self.nic.state.lock();
            let inner: &mut Inner = state.inner.as_mut().unwrap();
            let slot = if self.receive {
                &mut inner.rx[self.index]
            } else {
                &mut inner.tx[self.index]
            };
            assert!(slot.page.is_none(), "GMAC slot already has a page");
            slot.state = if page.is_device_owned() {
                SlotState::DeviceOwned
            } else {
                SlotState::Free
            };
            slot.page = Some(page);
        }
    }
}

/// 一次性分配收发描述符环以及全部 64 个收发数据物理页槽位，此时尚未向硬件寄存器暴露任何 DMA 地址。
fn prepare_inner(
    domain: DmaDomain,
    generation: Generation,
    coherent: bool,
) -> Result<Inner, GmacError> {
    Ok(Inner {
        domain,
        rx_ring: Ring::new(domain, generation, coherent)?,
        tx_ring: Ring::new(domain, generation, coherent)?,
        rx: prepare_pool(domain, BufferDirection::DeviceToDriver)?,
        tx: prepare_pool(domain, BufferDirection::DriverToDevice)?,
        rx_next: 0,
        tx_claim: 0,
        tx_publish: 0,
        tx_complete: 0,
    })
}

/// 为单个方向（RX 或 TX）预分配 `RING_LEN` 个位于设备 DMA 可寻址窗口内的清零物理页，并预计算好对应的总线地址。
fn prepare_pool(domain: DmaDomain, direction: BufferDirection) -> Result<Vec<Slot>, GmacError> {
    let mut slots = Vec::new();
    slots
        .try_reserve_exact(RING_LEN)
        .map_err(|_| GmacError::NoMemory)?;
    let (start, end) = domain.allocation_window();
    for _ in 0..RING_LEN {
        let pages = mm::alloc_pages_in(0, true, start, end).ok_or(GmacError::NoMemory)?;
        let page = MappedPages::new(pages, 0, BUFFER_LEN, direction)?;
        let address = page.segment(domain)?.address.value();
        slots.push(Slot {
            state: SlotState::Free,
            page: Some(page),
            address,
            frame_len: 0,
        });
    }
    Ok(slots)
}

/// 按硬件环形队列顺序扫描从 `tx_publish` 起连续处于 `Ready` 状态的发送槽，依次写入发送描述符并置位 `OWN`（吸收并发发送者乱序完成填充的情况），返回最新推进后的环尾地址。
fn publish_ready(inner: &mut Inner) -> Result<Option<u64>, GmacError> {
    let mut tail = None;
    for _ in 0..RING_LEN {
        let index = inner.tx_publish;
        let slot = &mut inner.tx[index];
        if slot.state != SlotState::Ready {
            break;
        }
        inner.tx_ring.arm_tx(index, slot.address, slot.frame_len)?;
        slot.state = SlotState::DeviceOwned;
        inner.tx_publish = (index + 1) % RING_LEN;
        tail = Some(inner.tx_ring.address(index + 1));
    }
    Ok(tail)
}

/// 在 JH7110（DWMAC4）平台上根据设备树属性配置 syscon 寄存器中的物理层接口模式字段（RGMII 或 RMII）；DWMAC1000 平台无需此操作。
///
/// # Safety
/// 调用方必须保证 `resources.syscon` 物理区间已建立永久有效的强序 MMIO 映射，且 `resources.sysreg` 指定的接口模式位域由本 GMAC 实例独占访问。
unsafe fn configure_interface(
    resources: &GmacResources,
    generation: Generation,
) -> Result<(), GmacError> {
    if generation != Generation::Dwmac4 {
        return Ok(());
    }
    let offset = resources.sysreg[0] as usize;
    let shift = resources.sysreg[1];
    if resources.syscon.start == 0
        || offset % REGISTER_BYTES != 0
        || u32::BITS - STARFIVE_INTERFACE_BITS < shift
        || resources
            .syscon
            .end
            .checked_sub(resources.syscon.start)
            .is_none_or(|length| length < offset + REGISTER_BYTES)
    {
        return Err(GmacError::Resource(
            "GMAC interface syscon field is invalid",
        ));
    }

    let address = arch::mmio_address(resources.syscon.start) + offset;
    let mode = if resources.phy_mode == boot::PhyMode::Rmii {
        STARFIVE_INTERFACE_RMII
    } else {
        STARFIVE_INTERFACE_RGMII
    };
    // SAFETY:
    // 1. 前置检查已验证 `syscon.start != 0`、`offset` 4 字节对齐、`shift` 未越界且 `[offset, offset + 4)` 位于 `syscon` 窗口内；
    // 2. 调用方契约保证 `syscon` 已永久强序映射且该位域由本设备独占，使用 `read_volatile`/`write_volatile` 不产生内存引用别名。
    unsafe {
        let value = ptr::read_volatile(address as *const u32);
        ptr::write_volatile(
            address as *mut u32,
            (value & !(STARFIVE_INTERFACE_MASK << shift)) | (mode << shift),
        );
    }
    arch::dma_barrier();
    Ok(())
}

/// 校验 6 字节以太网 MAC 地址是否为合法的非零单播地址（首字节最低位 I/G 组播标志位必须为 `0`）。
fn valid_mac(mac: [u8; 6]) -> bool {
    mac != [0; 6] && mac[0] & 1 == 0
}

/// 外部中断控制器的 C 风格回调包装函数：将上下文裸指针还原为 `&Dwmac` 并转发至 `handle_interrupt`。
///
/// # Safety
/// `context` 必须是由 `Dwmac::start` 通过 `Arc::as_ptr` 注册的合法 `Dwmac` 实例指针，且上层强引用必须在整个中断回调执行期间及 `unregister_sync` 返回前保持该实例有效且地址不变。
unsafe fn irq_callback(context: *mut ()) -> bool {
    // SAFETY: 调用方遵守中断注册契约，保证 `context` 指向由上层 `Arc<Dwmac>` 持续保活的有效实例，直到 `unregister_sync` 同步等待全部中断回调退出。
    unsafe { &*context.cast::<Dwmac>() }.handle_interrupt()
}

/// 绑定在 CPU0 网络工作队列上的后台任务入口：执行收发处理与链路刷新，遇到不可恢复错误时将设备置为 `Faulted` 并打印错误日志。
fn progress_callback(work: Pin<&Work>) {
    // SAFETY: `work.context()` 保存的是 `Dwmac::prepare` 中 `Arc::as_ptr(&nic)` 生成的稳定地址；上层强引用保活该实例，且 `stop_sync` 在释放前必先调用 `progress.close_sync()` 等待本回调彻底退出。
    let nic = unsafe { &*(work.context() as *const Dwmac) };
    if let Err(error) = nic.progress() {
        nic.record_failure(error);
        crate::klog!(
            crate::klog::Level::Error,
            "GMAC progress failed: {:?}",
            error
        );
    }
}

/// 秒级 PHY 链路定时器的到期回调：在设备处于 `Live` 运行态时置位 `phy_due` 采样标志并唤醒后台 `progress` Work（不在定时器软中断内直接访问 MDIO）。
fn link_expired(timer: Pin<&Timer>) {
    // SAFETY: `timer.context()` 保存的是 `Dwmac::prepare` 中 `Arc::as_ptr(&nic)` 生成的稳定地址；上层强引用保活该实例，且 `stop_sync` 在释放前必先调用 `link_timer.close_sync()` 等待本回调彻底退出。
    let nic = unsafe { &*(timer.context() as *const Dwmac) };
    let schedule = {
        let mut state = nic.state.lock();
        if state.phase == Phase::Live {
            state.phy_due = true;
            true
        } else {
            false
        }
    };
    if schedule {
        nic.queue_progress();
    }
}
