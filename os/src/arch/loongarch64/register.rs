//! LoongArch64 架构控制状态寄存器（CSR）、扩展单元、CPUCFG、IOCSR 与内存屏障常量定义。
//!
//! # 模块职责与架构定位
//! 本模块集中声明 `arch::loongarch64` 各子模块（`mmu`、`trap`、`user`、`counter`、`uaccess` 等）共同依赖的硬件寄存器编号、
//! 位域偏移与掩码常量。
//! - **上游调用者**：`arch::loongarch64` 及其下属子模块、异常/信号分发层。
//! - **下游依赖**：纯常量定义模块，不依赖任何运行时状态或其他子系统。
//!
//! # 核心寄存器位域与保存寄存器约定
//! ## 1. `CSR_CRMD`（`0x0`，当前模式信息）与 `CSR_PRMD`（`0x1`，例外前模式信息）
//! ```text
//! CSR_CRMD (0x0):
//!  31                9 8     7 6     5  4   3   2  1    0
//! +-------------------+-------+-------+---+---+---+------+
//! |     Reserved      | DATM  | DATF  |PG |DA |IE | PLV  |
//! +-------------------+-------+-------+---+---+---+------+
//!  - Bits [1:0] (PLV) : 当前特权等级（0 = 内核态 PLV0，3 = 用户态 PLV3）。
//!  - Bit  2     (IE)  : 全局中断使能位（CRMD_IE = 1 << 2）。
//!  - Bit  3     (DA)  : 直接地址翻译模式使能（CRMD_DA = 1 << 3，机器复位与 TLB 重填入口自动置 1）。
//!  - Bit  4     (PG)  : 映射地址翻译模式使能（CRMD_PG = 1 << 4，内核正常运行与用户态置 1）。
//!  - Bits [6:5] (DATF): 直接地址翻译模式下取指存储访问类型（CRMD_DATF_CACHED = 1 << 5，一致可缓存）。
//!  - Bits [8:7] (DATM): 直接地址翻译模式下访存存储访问类型（CRMD_DATM_CACHED = 1 << 7，一致可缓存）。
//!
//! CSR_PRMD (0x1):
//!  - Bits [1:0] (PPLV): 异常发生前的特权等级（PRMD_PPLV = 0x3）。
//!  - Bit  2     (PIE) : 异常发生前的中断使能状态（PRMD_PIE = 1 << 2）。
//! ```
//!
//! ## 2. `CSR_ESTAT`（`0x5`，例外状态）
//! ```text
//!  31       22 21           16 15   13 12   11 10   9       2 1     0
//! +-----------+---------------+-------+---+---+---+----------+-------+
//! |  EsubCode |  Ecode (6b)   | Rsvd  |IPI|TI |Rsv| HWI7..0  | SWI1..0|
//! +-----------+---------------+-------+---+---+---+----------+-------+
//!  - Bits [12:0]  (IS)   : 中断状态位图（ESTAT_IRQ_MASK = 0x1fff），其中 Bit 2/3 为 HWI0/HWI1，Bit 11 为定时器中断 TI，Bit 12 为核间中断 IPI。
//!  - Bits [21:16] (Ecode): 一级异常编码（右移 ESTAT_ECODE_SHIFT = 16，掩码 ESTAT_ECODE_MASK = 0x3f；为 0 表示硬中断）。
//! ```
//!
//! ## 3. 内核数据保存寄存器 `CSR_KS0..KS3`（`0x30..0x33`）与 `CSR_TLBRSAVE`（`0x8b`）协议
//! ```text
//! +--------------+--------+--------------------------------------------------------------+
//! | 寄存器       | 编号   | 内核使用约定                                                 |
//! +--------------+--------+--------------------------------------------------------------+
//! | CSR_KS0      | 0x30   | 固定存放当前 CPU 的 per-CPU 结构指针（CpuLocal），跨异常保持 |
//! | CSR_KS1      | 0x31   | 用户态运行期间存放当前线程的 UserContext 指针                |
//! | CSR_KS2      | 0x32   | 用户异常入口暂存用户 $t0；TLB 重填异常暂存 $t1               |
//! | CSR_KS3      | 0x33   | 用户异常入口窗口期间暂存受信内核栈指针，防止用户态伪造 SP    |
//! | CSR_TLBRSAVE | 0x8b   | TLB 重填异常专用暂存寄存器，用于保存重填快路径借用的 $t0     |
//! +--------------+--------+--------------------------------------------------------------+
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 本模块仅包含编译期常量定义，无锁或可变状态。

/// 当前模式信息寄存器（`CRMD`）：控制当前特权级（`PLV`）、全局中断使能（`IE`）及地址翻译模式（`DA`/`PG`）。
pub(crate) const CSR_CRMD: u16 = 0x0;
/// 例外前模式信息寄存器（`PRMD`）：记录触发普通异常前的特权级（`PPLV`）和中断使能（`PIE`），在 `ertn` 时写回 `CRMD`。
pub(super) const CSR_PRMD: u16 = 0x1;
/// 扩展部件使能寄存器（`EUEN`）：控制基础浮点（`FPE`）、128 位向量（`SXE`/LSX）与 256 位向量（`ASXE`/LASX）指令可用性。
pub(crate) const CSR_EUEN: u16 = 0x2;
/// 例外配置寄存器（`ECFG`）：控制各局部中断源使能位（`LIE`）与异常入口间距配置（`VS`）。
pub(super) const CSR_ECFG: u16 = 0x4;
/// 例外状态寄存器（`ESTAT`）：记录待处理的硬/软中断挂起位图（`IS`）以及当前异常的一级/二级编码（`Ecode`/`EsubCode`）。
pub(super) const CSR_ESTAT: u16 = 0x5;
/// 例外返回地址寄存器（`ERA`）：记录普通异常触发时的指令 PC，`ertn` 返回时跳转至此地址。
pub(super) const CSR_ERA: u16 = 0x6;
/// 出错虚拟地址寄存器（`BADV`）：记录触发地址错误或页异常（非 TLB 重填）时的故障虚拟地址。
pub(super) const CSR_BADV: u16 = 0x7;
/// 普通例外入口地址寄存器（`EENTRY`）：配置除 TLB 重填与机器错误之外所有异常与中断的统一入口虚拟地址。
pub(super) const CSR_EENTRY: u16 = 0xc;
/// TLB 索引寄存器（`TLBIDX`）：配置或反映 TLB 指令操作的索引、页大小（`PS`）与存在位（`NE`）。
pub(super) const CSR_TLBIDX: u16 = 0x10;
/// 地址空间标识符寄存器（`ASID`）：存放当前地址空间的 ASID（本实现固定置 0 并配合切换时刷新非全局 TLB）。
pub(super) const CSR_ASID: u16 = 0x18;
/// 低半区页表基址寄存器（`PGDL`）：存放虚拟地址最高位为 0（用户空间）的三级根页表物理基址。
pub(super) const CSR_PGDL: u16 = 0x19;
/// 高半区页表基址寄存器（`PGDH`）：存放虚拟地址最高位为 1 的根页表物理基址（内核仅用 `DMW`，故固定指向永久无效根表）。
pub(super) const CSR_PGDH: u16 = 0x1a;
/// 页表遍历控制低半部分寄存器（`PWCL`）：配置叶表（`PT`）与低级目录（`Dir1`/`Dir2`）的起始位偏移与索引位宽。
pub(super) const CSR_PWCL: u16 = 0x1c;
/// 页表遍历控制高半部分寄存器（`PWCH`）：配置高级目录（`Dir3`/`Dir4`）的起始位偏移与索引位宽。
pub(super) const CSR_PWCH: u16 = 0x1d;
/// STLB 页大小寄存器（`STLBPS`）：配置组相联 TLB（STLB）的页大小编码（`12` 对应 4 KiB 页）。
pub(super) const CSR_STLBPS: u16 = 0x1e;
/// 处理器编号寄存器（`CPUID`）：只读寄存器，低位存放当前物理核的硬件编号（`CoreID`）。
pub(super) const CSR_CPUID: u16 = 0x20;
/// 内核数据保存寄存器 0（`KS0`）：固定保存当前 CPU 的 `CpuLocal` 结构内核虚拟地址。
pub(crate) const CSR_KS0: u16 = 0x30;
/// 内核数据保存寄存器 1（`KS1`）：在进入用户态时保存当前线程的 `UserContext` 指针。
pub(super) const CSR_KS1: u16 = CSR_KS0 + 1;
/// 内核数据保存寄存器 2（`KS2`）：在用户态异常入口或 TLB 重填异常中作为通用寄存器（`$t0`/`$t1`）的临时交换槽。
pub(super) const CSR_KS2: u16 = CSR_KS0 + 2;
/// 内核数据保存寄存器 3（`KS3`）：在用户态异常切换窗口期间保存受信内核栈顶地址。
pub(super) const CSR_KS3: u16 = CSR_KS0 + 3;
/// 定时器配置寄存器（`TCFG`）：配置核内定时器的使能位（`En`）、周期模式（`Periodic`）与初始倒计时滴答数（`InitVal`）。
pub(super) const CSR_TCFG: u16 = 0x41;
/// 计时器补偿寄存器（`CNTC`）：配置用户态/内核态 `rdtime` 指令读取恒定频率计数器时的软件偏移量。
pub(super) const CSR_CNTC: u16 = 0x43;
/// 定时中断清除寄存器（`TICLR`）：向 Bit 0（`CLR`）写 1 清除当前核挂起的本地定时器中断。
pub(super) const CSR_TICLR: u16 = 0x44;
/// TLB 重填例外入口地址寄存器（`TLBRENTRY`）：存放 TLB 重填异常的物理入口地址（触发时处理器自动进入 `DA=1, PG=0` 模式）。
pub(super) const CSR_TLBRENTRY: u16 = 0x88;
/// TLB 重填例外出错虚拟地址寄存器（`TLBRBADV`）：记录触发 TLB 重填异常时的故障虚拟地址（独立于普通异常的 `BADV`）。
pub(super) const CSR_TLBRBADV: u16 = 0x89;
/// TLB 重填例外返回地址寄存器（`TLBRERA`）：Bit 0（`IsTLBR`）标识当前处于重填上下文，高位记录重填返回 PC（独立于 `ERA`）。
pub(super) const CSR_TLBRERA: u16 = 0x8a;
/// TLB 重填例外数据保存寄存器（`TLBRSAVE`）：专供 TLB 重填处理程序暂存 1 个通用寄存器（`$t0`）。
pub(super) const CSR_TLBRSAVE: u16 = 0x8b;
/// TLB 重填例外表项低位寄存器 0（`TLBRELO0`）：存放待填入 TLB 的偶数虚拟页（Even Page）物理页表项。
pub(super) const CSR_TLBRELO0: u16 = 0x8c;
/// TLB 重填例外表项低位寄存器 1（`TLBRELO1`）：存放待填入 TLB 的奇数虚拟页（Odd Page）物理页表项。
pub(super) const CSR_TLBRELO1: u16 = 0x8d;
/// TLB 重填例外表项高位寄存器（`TLBREHI`）：存放待填入 TLB 的双页虚拟页号（`VPPN`）及重填页大小（`PS`）。
pub(super) const CSR_TLBREHI: u16 = 0x8e;
/// TLB 重填例外前模式信息寄存器（`TLBRPRMD`）：记录触发 TLB 重填前的 `PPLV` 与 `PIE`（独立于 `PRMD`）。
pub(super) const CSR_TLBRPRMD: u16 = 0x8f;
/// 直接映射配置窗口 0（`DMW0`）：内核将其配置为 `PLV0` 可访问的强序非缓存（Strongly-ordered UnCached）设备 MMIO 窗口。
pub(crate) const CSR_DMW0: u16 = 0x180;
/// 直接映射配置窗口 1（`DMW1`）：内核将其配置为 `PLV0` 可访问的一致可缓存（Coherent Cached）物理 RAM 线性映射窗口。
pub(crate) const CSR_DMW1: u16 = CSR_DMW0 + 1;
/// 直接映射配置窗口 2（`DMW2`）：本内核未使用，初始化时清零关闭。
pub(super) const CSR_DMW2: u16 = CSR_DMW0 + 2;
/// 直接映射配置窗口 3（`DMW3`）：本内核未使用，初始化时清零关闭。
pub(super) const CSR_DMW3: u16 = CSR_DMW0 + 3;

/// 用户态特权等级值（`PLV3`）。
pub(super) const PLV_USER: usize = 3;
/// `CSR_PRMD` / `CSR_TLBRPRMD` 中例外前特权等级字段（`PPLV`，Bits `[1:0]`）的位掩码。
pub(super) const PRMD_PPLV: usize = 0x3;
/// `CSR_PRMD` 中例外前中断使能位（`PIE`，Bit 2）。
pub(super) const PRMD_PIE: usize = 1 << 2;
/// 通过 `ertn` 返回 `PLV3` 用户态并同时开启全局中断的 `CSR_PRMD` 组合值（`PPLV=3 | PIE=1`）。
pub(super) const PRMD_USER: usize = PLV_USER | PRMD_PIE;
/// `CSR_CRMD` 中的全局中断使能位（`IE`，Bit 2）。
pub(crate) const CRMD_IE: usize = 1 << 2;
/// `CSR_CRMD` 中的直接地址翻译模式位（`DA`，Bit 3）。
pub(super) const CRMD_DA: usize = 1 << 3;
/// `CSR_CRMD` 中的映射地址翻译模式位（`PG`，Bit 4）。
pub(super) const CRMD_PG: usize = 1 << 4;
/// `CSR_CRMD` 中地址翻译模式字段（`DA | PG`，Bits `[4:3]`）的组合掩码。
pub(super) const CRMD_ADDRESS_MODE: usize = CRMD_DA | CRMD_PG;
/// `CSR_CRMD` 中直接地址翻译模式下取指存储访问类型设为一致可缓存（`DATF = 1`，Bit 5）。
pub(super) const CRMD_DATF_CACHED: usize = 1 << 5;
/// `CSR_CRMD` 中直接地址翻译模式下访存存储访问类型设为一致可缓存（`DATM = 1`，Bit 7）。
pub(super) const CRMD_DATM_CACHED: usize = 1 << 7;
/// 启动汇编切入映射地址翻译模式（`PG=1, DA=0, PLV=0, IE=0`）并保持 `DATF/DATM` 可缓存的 `CSR_CRMD` 初始值。
pub(super) const CRMD_BOOT: usize = CRMD_PG | CRMD_DATF_CACHED | CRMD_DATM_CACHED;

/// `CSR_DMW0..3` 中允许 `PLV0` 内核态使用该窗口的使能位（Bit 0）。
pub(super) const DMW_PLV0: usize = 1;
/// `CSR_DMW0..3` 中将存储访问类型（`MAT`）设为一致可缓存（Coherent Cached，Bit 4）。
pub(super) const DMW_CACHED: usize = 1 << 4;
/// `CSR_DMW1` 内核普通物理内存直接映射窗口的属性组合（`PLV0` 可访问 + 一致可缓存）。
pub(super) const DMW_KERNEL_RAM: usize = DMW_PLV0 | DMW_CACHED;

/// `CSR_EUEN` 中的基础浮点部件使能位（`FPE`，Bit 0）。
pub(crate) const EUEN_FP: usize = 1;
/// `CSR_EUEN` 中的 128 位向量扩展（LSX）使能位（`SXE`，Bit 1）。
pub(crate) const EUEN_LSX: usize = 1 << 1;
/// `CSR_EUEN` 中的 256 位高级向量扩展（LASX）使能位（`ASXE`，Bit 2）。
pub(crate) const EUEN_LASX: usize = 1 << 2;
/// 仅开启标量浮点单元的 `CSR_EUEN` 组合值。
pub(super) const EUEN_SCALAR: usize = EUEN_FP;
/// 开启标量浮点与 128 位 LSX 向量单元的 `CSR_EUEN` 组合值。
pub(super) const EUEN_SIMD: usize = EUEN_FP | EUEN_LSX;
/// 开启标量浮点、128 位 LSX 与 256 位 LASX 向量单元的 `CSR_EUEN` 组合值。
pub(super) const EUEN_WIDE_SIMD: usize = EUEN_SIMD | EUEN_LASX;

/// `cpucfg` 指令的叶号 `2`：查询处理器支持的浮点与向量指令集特性字。
pub(super) const CPUCFG_FEATURES: usize = 2;
/// `CPUCFG.2` 中的基础浮点（FP）支持标志位（Bit 0）。
pub(super) const CPUCFG_FP: u64 = 1;
/// `CPUCFG.2` 中的 128 位向量扩展（LSX）支持标志位（Bit 6）。
pub(super) const CPUCFG_LSX: u64 = 1 << 6;
/// `CPUCFG.2` 中的 256 位向量扩展（LASX）支持标志位（Bit 7）。
pub(super) const CPUCFG_LASX: u64 = 1 << 7;

/// `cpucfg` 指令的叶号 `4`：查询恒定频率计时器的晶振基准频率（`CCFREQ`，单位 Hz）。
pub(super) const CPUCFG_COUNTER_BASE: usize = 4;
/// `cpucfg` 指令的叶号 `5`：查询恒定频率计时器的倍频系数（`CCMUL`）与分频系数（`CCDIV`）。
pub(super) const CPUCFG_COUNTER_RATIO: usize = 5;
/// `CPUCFG.5` 中提取低 16 位倍频分子（`CCMUL`，Bits `[15:0]`）的位掩码。
pub(super) const CPUCFG_COUNTER_MUL_MASK: u64 = 0xffff;
/// `CPUCFG.5` 中提取高 16 位分频分母（`CCDIV`，Bits `[31:16]`）的右移位数。
pub(super) const CPUCFG_COUNTER_DIV_SHIFT: usize = 16;

/// `CSR_CPUID` 中提取硬件核编号（`CoreID`）所使用的位宽（10 位，覆盖 `0..1024` 个硬件核 ID）。
pub(super) const HARDWARE_CPU_ID_BITS: usize = 10;
/// 系统可索引的硬件 CPU ID 上界数量（`1 << 10 = 1024`）。
pub(super) const HARDWARE_CPU_ID_COUNT: usize = 1 << HARDWARE_CPU_ID_BITS;
/// 从 `CSR_CPUID` 中提取低 10 位硬件核编号的位掩码（`0x3ff`）。
pub(super) const HARDWARE_CPU_ID_MASK: usize = HARDWARE_CPU_ID_COUNT - 1;

/// `CSR_ESTAT` 中一级异常编码字段（`Ecode`）的起始位偏移（Bit 16）。
pub(super) const ESTAT_ECODE_SHIFT: usize = 16;
/// `CSR_ESTAT` 中一级异常编码字段（`Ecode`，共 6 位）的位掩码（`0x3f`）。
pub(super) const ESTAT_ECODE_MASK: usize = 0x3f;
/// `CSR_ESTAT` 与 `CSR_ECFG` 中全部 13 个中断源（`SWI0..1`、`HWI0..7`、`PMI`、`TI`、`IPI`）的位掩码（Bits `[12:0]`）。
pub(super) const ESTAT_IRQ_MASK: usize = (1 << 13) - 1;

/// 外部硬中断线 `HWI0` 在 `ECFG`/`ESTAT` 中的中断位编号（Bit 2，用于 LIOINTC 传统中断控制器）。
pub(crate) const CPU_HWI0: u32 = 2;
/// 外部硬中断线 `HWI1` 在 `ECFG`/`ESTAT` 中的中断位编号（Bit 3，用于 EXTIOI 扩展 I/O 中断控制器）。
pub(crate) const CPU_HWI1: u32 = CPU_HWI0 + 1;
/// 外部硬中断线 `HWI0` 的位掩码（`1 << 2`）。
pub(super) const IRQ_HWI0: usize = 1 << CPU_HWI0;
/// 外部硬中断线 `HWI1` 的位掩码（`1 << 3`）。
pub(super) const IRQ_HWI1: usize = 1 << CPU_HWI1;
/// 核内定时器中断（`TI`）在 `ECFG`/`ESTAT` 中的位掩码（Bit 11）。
pub(super) const IRQ_TIMER: usize = 1 << 11;
/// 核间中断（`IPI`）在 `ECFG`/`ESTAT` 中的位掩码（Bit 12）。
pub(super) const IRQ_IPI: usize = 1 << 12;

/// 一级异常码 `0x1`（`PIL`）：Load 操作页无效异常（TLB 命中但 `V=0`）。
pub(crate) const ECODE_PIL: usize = 1;
/// 一级异常码 `0x2`（`PIS`）：Store 操作页无效异常（TLB 命中但 `V=0`）。
pub(crate) const ECODE_PIS: usize = 2;
/// 一级异常码 `0x3`（`PIF`）：取指（Fetch）操作页无效异常（TLB 命中但 `V=0`）。
pub(crate) const ECODE_PIF: usize = 3;
/// 一级异常码 `0x4`（`PME`）：页修改异常（Store 命中 `V=1` 且权限允许，但 `D=0`，如触发 COW）。
pub(crate) const ECODE_PME: usize = 4;
/// 一级异常码 `0x5`（`PNR`）：页不可读异常（Load 命中 `V=1` 但 `NR=1`）。
pub(crate) const ECODE_PNR: usize = 5;
/// 一级异常码 `0x6`（`PNX`）：页不可执行异常（取指命中 `V=1` 但 `NX=1`）。
pub(crate) const ECODE_PNX: usize = 6;
/// 一级异常码 `0xb`（`SYS`）：系统调用异常（执行 `syscall` 指令触发）。
pub(crate) const ECODE_SYS: usize = 11;
/// 一级异常码 `0xd`（`INE`）：指令不存在或非法指令异常（含未开启 FP/LSX/LASX 时执行对应指令）。
#[cfg(feature = "ktest")]
pub(crate) const ECODE_INE: usize = 13;

/// `CSR_TLBRERA` 的 Bit 0（`IsTLBR`）：置 1 标识当前处于 TLB 重填异常上下文，影响 `ertn` 的返回行为。
pub(super) const TLBRERA_IS_TLBR: usize = 1;

/// `CSR_TCFG` 的 Bit 0（`En`）：核内定时器使能位。
pub(super) const TCFG_ENABLE: usize = 1;
/// `CSR_TICLR` 的 Bit 0（`CLR`）：写 1 清除本地定时器中断挂起状态。
pub(super) const TICLR_CLEAR: usize = 1;
/// `CSR_TCFG` 倒计时初始值的最小粒度步长（低 2 位由 `En` 与 `Periodic` 占用，计数值按 4 对齐）。
pub(super) const TIMER_QUANTUM: u64 = 1 << 2;
/// 单次编程 `CSR_TCFG` 允许的最大倒计时滴答差值（48 位计数器上界减去对齐粒度 `TIMER_QUANTUM`）。
pub(super) const TIMER_MAX_DELTA: u64 = (1u64 << 48) - TIMER_QUANTUM;

/// IOCSR 核间中断状态寄存器（`0x1000`，32 位只读）：反映当前核收到的 IPI 向量挂起位。
pub(super) const IOCSR_IPI_STATUS: usize = 0x1000;
/// IOCSR 核间中断使能寄存器（`0x1004`，32 位读写）：控制当前核允许响应的 IPI 向量位。
pub(super) const IOCSR_IPI_ENABLE: usize = 0x1004;
/// IOCSR 核间中断清除寄存器（`0x100c`，32 位只写）：写 1 清除对应的挂起 IPI 向量位。
pub(super) const IOCSR_IPI_CLEAR: usize = 0x100c;
/// IOCSR 核间信箱 1（`0x1028`，对应 `MailBox2..3` 组合成的 64 位槽）：从核启动时存放启动栈顶地址。
pub(super) const IOCSR_MAILBOX1: usize = 0x1028;
/// IOCSR 核间信箱 2（`0x1030`，对应 64 位槽）：从核启动时存放每核引导记录（`SecondaryBootRecord`）指针。
pub(super) const IOCSR_MAILBOX2: usize = 0x1030;
/// IOCSR 核间中断发送寄存器（`0x1040`，32 位只写）：向目标硬件核发送指定向量的 IPI。
pub(super) const IOCSR_IPI_SEND: usize = 0x1040;
/// IOCSR 核间信箱发送寄存器（`0x1048`，64 位只写）：向目标硬件核的指定 32 位信箱槽位写入数据。
pub(super) const IOCSR_MAIL_SEND: usize = 0x1048;

/// `IOCSR_IPI_SEND` / `IOCSR_MAIL_SEND` 的阻塞等待完成标志位（Bit 31）。
pub(super) const IPI_SEND_BLOCKING: u32 = 1 << 31;
/// `IOCSR_IPI_SEND` / `IOCSR_MAIL_SEND` 中目标硬件核编号（`CPU`）的起始位偏移（Bit 16）。
pub(super) const IPI_SEND_CPU_SHIFT: usize = 16;
/// 每个 CPU 拥有的 64 位核间信箱寄存器总数（共 4 个 64 位信箱，即 8 个 32 位子槽）。
pub(super) const MAILBOX_COUNT: usize = 4;
/// `IOCSR_MAIL_SEND` 中 32 位信箱子槽索引（`Box`）的起始位偏移（Bit 2）。
pub(super) const MAIL_SEND_BOX_SHIFT: usize = 2;
/// `IOCSR_MAIL_SEND` 中 32 位待发送数据字段的起始位偏移（Bit 32）。
pub(super) const MAIL_SEND_DATA_SHIFT: usize = 32;
/// 提取 64 位值高 32 位以装入 `IOCSR_MAIL_SEND` 数据域的位掩码（`0xffff_ffff_0000_0000`）。
pub(super) const MAIL_SEND_HIGH_MASK: u64 = (u32::MAX as u64) << MAIL_SEND_DATA_SHIFT;

/// 一级异常码 `0x7`（`PPI`）：页特权等级不合规异常（如 `PLV3` 用户态访问 `PLV0` 页）。
pub(crate) const ECODE_PPI: usize = 7;
/// 一级异常码 `0x8`（`ADE`）：取指或访存地址错误异常（如 `PLV3` 用户态直接访问高地址内核空间）。
pub(crate) const ECODE_ADE: usize = 8;
/// 一级异常码 `0x9`（`ALE`）：地址非对齐异常（访存地址未按指令数据宽度自然对齐）。
pub(crate) const ECODE_ALE: usize = 9;
/// 一级异常码 `0xa`（`BCE`）：边界检查异常。
pub(crate) const ECODE_BCE: usize = 10;
/// 一级异常码 `0xc`（`BRK`）：断点异常（执行 `break` 指令触发）。
pub(crate) const ECODE_BRK: usize = 12;
/// 一级异常码 `0x12`（`FPE`）：浮点指令异常。
pub(crate) const ECODE_FPE: usize = 18;

/// 浮点控制状态寄存器 `fcsr0` 中五类 IEEE 754 浮点异常使能位（`Enables`，Bits `[4:0]`）的掩码。
pub(crate) const FCSR_ENABLE_MASK: u32 = 0x1f;
/// 浮点控制状态寄存器 `fcsr0` 中当次浮点异常原因字段（`Cause`，Bits `[28:24]`）的起始位偏移。
pub(crate) const FCSR_CAUSE_SHIFT: usize = 24;

/// `dbar` 提示字 `0x0a`：写-写完成型内存屏障（等待此前所有写操作完成，再执行后续写操作，用于 DMA 描述符发布）。
pub(super) const DBAR_COMPLETION_WRITE: usize = 0x0a;
/// `dbar` 提示字 `0x05`：读-读完成型内存屏障（等待此前所有读操作完成，再执行后续读操作，用于 DMA 完成状态读取）。
pub(super) const DBAR_COMPLETION_READ: usize = 0x05;
/// `dbar` 提示字 `0x14`：Load-Acquire 同步内存屏障（阻止前序读操作与后续任何读/写操作重排）。
pub(super) const DBAR_ACQUIRE: usize = 0x14;
/// `dbar` 提示字 `0x12`：Store-Release 同步内存屏障（阻止前序任何读/写操作与后续写操作重排）。
pub(super) const DBAR_RELEASE: usize = 0x12;
