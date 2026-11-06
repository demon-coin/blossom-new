# 人工上板、根卷与应用验证

本指南描述当前项目面向实体开发板的构建、打包与人工启动接口（并非固定验收报告）。当前支持的架构与平台组合为 `riscv64`/`visionfive2` 与 `loongarch64`/`ls2k1000la`；QEMU 仿真则共用 [开发指南](development.md) 中的统一运行入口。两块实体板的固件版本、OS 设备树（DTB）、物理内存容量、存储介质与串口参数均须结合实际硬件设备核实；宿主机侧的编译与镜像打包脚本不会主动连接串口、不会直接写物理磁盘、不会执行固件 `saveenv`，也不依赖内核测试框架 `ktest`。

## 1. 首次接触先记录实际输入

首次连接开发板进入 U-Boot 固件控制台时，先执行以下只读命令查看当前固件支持的指令集与默认环境变量：

```text
version
bdinfo
printenv bootcmd bootargs loadaddr kernel_addr_r fdt_addr ramdisk_addr_r
help bootm
help bootelf
help fdt
```

请完整记录开发板硬件修订号、实际 CPU 核数与 RAM 范围、固件版本、串口波特率参数、OS DTB 来源、固件保留内存区（reserved-memory）以及可用的下载暂存空间。若当前固件缺少某条命令，应如实记录该限制，切勿凭其他项目的经验假设固件支持。串口电平标准与排针引脚定义请以对应开发板的硬件手册为准。

在 U-Boot 中，控制设备树（control FDT）仅供固件自身驱动使用，而工作设备树（working FDT）才是移交给操作系统的设备树。当我们提供独立的 OS DTB 时，应将其下载到独立内存地址；若确需基于固件的 control FDT 修改，也必须先将其复制到不重叠的空闲内存区域后再按已知硬件拓扑修改，严禁直接覆盖固件正在引用的设备树。[1]

如果开发板上已有可启动的原厂 Linux 系统，可以先运行并记录 `uname -m`、`/proc/iomem`、设备树 `model` 属性以及 `lsblk -o NAME,SIZE,TYPE,FSTYPE,MOUNTPOINTS`。这些信息有助于确认板上实际的存储介质与分区布局（无需在 BLOSSOM 用户态中额外实现同名工具）。

### OS DTB 的资源约定

设备树保留完整的硬件拓扑描述，无需根据当前内核驱动的覆盖范围手动删减节点。对于 `/reserved-memory` 节点：若包含 `reg` 属性，内核将直接保留该静态物理地址区间；若不含 `reg` 属性，内核则在早期内存初始化阶段按照 `size`、`alignment` 及可选的 `alloc-ranges` 属性完成动态预留。动态内存预留必须避开物理内存孔洞、内核镜像、固件驻留区、DTB、initrd 以及已登记的静态保留区，且带有 `no-map` 属性的区间按整页对齐避让；若物理内存无法满足预留要求，内核将在启动阶段直接报错停止（当前实现不提供 CMA 页面迁移或可复用内存池）。

在 VisionFive 2 上，L2 缓存控制器（ccache）驱动仅映射 `reg` 属性的第 0 段控制寄存器区，其余窗口描述保留在设备树中即可。当前内核仅初始化并驱动明确指定的 `ethernet1` 网口与 `mmc1` SDIO 控制器；设备树中保留的其他外设节点（包括处于 `enabled` 状态的 `GMAC0`）不会被当前驱动认领。当所选的目标设备节点不存在或状态明确为 `disabled` 时，内核按设备缺席处理；但如果所选设备的寄存器资源配置有误或硬件初始化失败，内核会直接报错，不会私自切换到其他网口或回退到另一设备。

## 2. 分清离线文件与运行期设备

| 对象 | 角色与语义说明 |
|---|---|
| `kernel.elf` | 带有完整符号表的内核 ELF 文件，其 `entry`、`LOAD` 段与 `BSS` 范围是内存布局的唯一事实来源 |
| `kernel.bin` | 按物理 `LOAD` 段相对偏移展平的纯二进制镜像，不含 ELF 头与调试信息容器 |
| `kernel.uImage` | 封装了目标架构标识、装载地址（load）、入口地址（entry）与 CRC32 校验值的 U-Boot 镜像容器 |
| `board.dtb` | 本次启动移交给操作系统的设备树文件（预留了写入 initrd 范围等属性所需的扩展空间） |
| `root.ext4` | 首字节为文件系统卷起点的裸 `ext4` 镜像（主超级块位于卷内字节偏移 `1024` 处）；既可封装进 RAM 盘，也可直接写入物理磁盘分区 |
| `root.uInitrd` | 由 64 字节 U-Boot ramdisk 头部加上未压缩的 raw `ext4` 镜像组成；固件向内核传递其内部有效负载（payload）的物理地址范围 |
| 整卡或整盘镜像 | 可能包含 MBR/GPT 分区表与引导扇区，不等同于单个分区的 raw `ext4` 文件系统镜像 |
| 官方比赛测试盘 | 包含原始比赛程序、动态链接器、共享库（DSO）与测试清单的只读基准输入；不得重新制盘篡改以冒充原始评测输入 |

请注意区分文件实际格式与文件名后缀：切勿将 raw `ext4` 镜像单纯改名后当作 `gzip` 压缩包、UBI 闪存镜像或厂商固件升级包使用。此外，U-Boot 固件在启动阶段从 SD 卡或 TFTP 网络加载内核镜像到内存，与内核启动后在运行期通过自身驱动读写 SD 卡或 SATA 硬盘属于完全独立的软件路径——使用 RAM 根文件系统（`ram0`）启动成功，并不代表内核的原生块设备驱动、DMA 传输或外设中断已经验证通过。

同时必须严格区分 CPU 物理地址、CPU 虚拟地址与外设 DMA 地址：设备树（FDT）或 PCI BAR 给出的是外设寄存器的物理 MMIO 地址，内核完成页表映射后通过 CPU 虚拟地址访问；而外设发起 DMA 传输使用的是该设备所属 DMA 域（domain）经过地址窗口偏移、位宽掩码（mask）约束后的设备侧地址。对于非缓存一致性（non-coherent）外设，DMA 传输前后还必须执行真实的 CPU 缓存刷写与失效（cache maintenance）操作，不能仅用内存屏障指令（`fence` / `dbar`）代替。将平台布局常量整理为纯 Rust 代码或调整编辑器配置，均不会改变上述底层硬件地址语义。

## 3. 普通构建、装配和离线包装

仅修改内核源码时，只需重新编译对应开发板平台的内核，无需重新制作根文件系统镜像：

```sh
make build-kernel ARCH=riscv64 TARGET_PLATFORM=visionfive2 KERNEL_BUILD_PROFILE=release
make build-kernel ARCH=loongarch64 TARGET_PLATFORM=ls2k1000la KERNEL_BUILD_PROFILE=release
```

Rust 工具链、目标标准库、交叉 GCC 与 libc 运行库均按 [开发指南](development.md) 事先准备，构建过程不会联网下载或升级工具链。用户态根目录树与 raw `ext4` 镜像分步生成，以下命令输出的目录与镜像均属于项目管理的构建产物：

```sh
make assemble-root-filesystem ARCH=riscv64 ROOTFS_OUTPUT_DIR=build/board-root-rv
make create-ext4-image ARCH=riscv64 \
  ROOTFS_TREE_DIR=build/board-root-rv SYSTEM_ROOT_EXT4_IMAGE=build/board-root-rv.ext4
make assemble-root-filesystem ARCH=loongarch64 ROOTFS_OUTPUT_DIR=build/board-root-la
make create-ext4-image ARCH=loongarch64 \
  ROOTFS_TREE_DIR=build/board-root-la SYSTEM_ROOT_EXT4_IMAGE=build/board-root-la.ext4
```

两张镜像各自以 `/` 作为单一根目录，包含匹配目标架构的 `/sbin/init`、交互式 Shell、Rust `std` 执行器及配置文件，不再混装另一架构的目录。若已有外部准备好的完整根目录树，可直接传给 `ROOTFS_TREE_DIR`；制盘步骤仅原样打包输入目录，不会自动向其补充程序或修改输入文件。生成的镜像保持无日志（no-journal）ext4 格式，内核也不启用日志重放（replay/recovery）。

仓库内已收录当前支持的两块开发板的设备树源码。首先从仓内源码编译生成对应的 OS DTB：

```sh
make build-board-dtb ARCH=riscv64 TARGET_PLATFORM=visionfive2
make build-board-dtb ARCH=loongarch64 TARGET_PLATFORM=ls2k1000la
```

默认输出路径分别为 `build/board/riscv64/visionfive2/board-input.dtb` 和 `build/board/loongarch64/ls2k1000la/board-input.dtb`（也可通过 `BOARD_DTB_OUTPUT` 显式指定输出路径）。宿主机需在 `PATH` 中提供 `cc`、`dtc` 以及打包 RAM 根所需的 `fdtput`（也可通过 `HOSTCC`、`DTC` 变量指定已安装的工具路径）。DTB 编译过程完全基于仓内源码，不会读取个人 `.docs/`、`ref/`、旧 `build/` 目录或外部厂商 SDK，也不会联网下载工具；若编译失败，脚本会保留错误诊断并退出，绝不会用旧的 DTB 产物充当本次构建结果。

VisionFive 2 的顶层设备树及其对应的固定版本 StarFive SDK（提交 `4cecf169f38eb94b40e307f5f870055e4d9d64f1`）DTS/DTSI、引脚定义与绑定头文件均位于 `os/dts/` 目录，完整保留了原有的硬件节点描述与版权/SPDX 声明。LS2K1000LA 的设备树由本机此前实测使用的板级源码与 Linux `7.3-rc4` SoC DTSI 组成，其中 AHCI 控制器的 `ports-implemented = <1>` 硬件属性依据官方 DP-V10 手册填写；需要注意的是，该设备树中记录的物理内存范围与固件保留区来自此前实测的 DP 开发板，不能未经核实直接套用到其他板型、硬件修订版或不同版本的固件上。所有第三方设备树源码均遵循各自的 SPDX 许可证，许可证全文存放在 `os/dts/LICENSES/`；LS2K1000LA 的板级顶层文件原本不含 SPDX 标头，文档如实记录其来源，不将其归属为已确认的厂商固件源码。

打包用于 RAM 根文件系统启动的固件镜像包时，需显式指定前面生成的根镜像与开发板设备树：

```sh
make package-board-boot-images ARCH=riscv64 TARGET_PLATFORM=visionfive2 \
  KERNEL_BUILD_PROFILE=release BOARD_BOOT_IMAGES_OUTPUT_DIR=build/vf2-boot \
  BOARD_ROOT_EXT4_IMAGE=build/board-root-rv.ext4 BOARD_ROOT_DEVICE=/dev/ram0 \
  BOARD_ROOT_MODE=rw BOARD_OS_DTB=build/board/riscv64/visionfive2/board-input.dtb
make package-board-boot-images ARCH=loongarch64 TARGET_PLATFORM=ls2k1000la \
  KERNEL_BUILD_PROFILE=release BOARD_BOOT_IMAGES_OUTPUT_DIR=build/2k-boot \
  BOARD_ROOT_EXT4_IMAGE=build/board-root-la.ext4 BOARD_ROOT_DEVICE=/dev/ram0 \
  BOARD_ROOT_MODE=rw BOARD_OS_DTB=build/board/loongarch64/ls2k1000la/board-input.dtb
```

此处 `BOARD_ROOT_EXT4_IMAGE` 是已存在的只读输入镜像，而非制盘输出目标。所有上板镜像统一通过根目录 Makefile 的 `package-board-boot-images` 目标生成；打包脚本直接解析最终普通内核 ELF 的公开符号与 `LOAD` 段获取内存布局，不引入 `ktest` 测试包，也不要求 ELF 包含测试专属段或栈展开表。

每次执行打包都会在输出目录生成 `kernel.elf`、`kernel.bin`、`kernel.uImage` 与 `boot.env`；当提供了对应的根镜像与设备树输入时，还会一并生成 `root.uInitrd` 与 `board.dtb`。`boot.env` 仅记录计算好的内存装载地址与内核命令行参数，不包含自动执行的启动命令；若缺少设备树或未明确指定根设备，不能仅凭生成了 `boot.env` 就认为镜像包已具备完整启动条件。此外，LoongArch 的 `kernel.uImage` 容器头使用真实的 LoongArch 架构编码，不会伪装成 MIPS 镜像。

当指定了 RAM 根镜像输入时，打包脚本会在输出的 `board.dtb` 副本中自动写入本次 `root.uInitrd` 内部有效负载（payload）的精确物理地址范围（已扣除开头的 64 字节 legacy U-Boot 头），同时保持源输入 DTB 和源 ext4 镜像文件不变。该步骤依赖 `PATH` 中的 `fdtput` 工具；若写入失败，打包脚本会立即中止并报错，不会输出残缺的启动包。

生成的 `boot.env` 会清晰区分 U-Boot 下载容器的暂存地址、物理装载与入口地址（`kernel_load` / `kernel_entry`）、ELF 符号入口地址以及包含 `.bss` 在内的内核内存末尾地址（`kernel_mem_end`），并列出 DTB 与 initrd 的地址范围。两块开发板统一使用环境变量 `fdt_addr` 作为设备树的 TFTP 下载地址与固件交接地址。在地址编码上，LoongArch 的 `kernel_addr_r`、`ramdisk_addr_r`、`fdt_addr` 使用可缓存的直接映射窗口（cached DMW）虚拟地址，而 `uImage` 容器头与设备树中的 initrd 属性则记录物理地址；RISC-V 的各运行地址则统一为物理地址。打包脚本会自动校验内核、initrd 与 DTB 各区间互不重叠，但这并不能代替实板物理内存核对——上板前仍须结合开发板实际可用 RAM、固件保留区以及完整的内核 `.bss` 范围进行确认。

## 4. RAM 根：两板分别人工启动

目标开发板固件需支持文本环境变量导入（`env import -t`）以及对应的 `bootm` 启动交接方式，且固件原有的 `${loadaddr}` 必须位于可用的暂存 RAM 区内。操作时先通过 TFTP 下载并导入本次生成的 `boot.env`，确认每条命令执行成功后再执行下一步（注意在执行 `env import` 前，`${filesize}` 必须保持为刚刚下载的 `boot.env` 文件字节数）。所有操作仅修改当前 RAM 中的临时环境变量，**严禁执行 `saveenv`**：

```text
setenv serverip <开发机TFTP地址>
setenv ipaddr <开发板固件地址>
tftpboot ${loadaddr} boot.env
env import -t ${loadaddr} ${filesize}
tftpboot ${kernel_addr_r} kernel.uImage
tftpboot ${fdt_addr} board.dtb
tftpboot ${ramdisk_addr_r} root.uInitrd
fdt addr ${fdt_addr}
setenv bootargs 'root=/dev/ram0 rootdir=/ rw init=/sbin/init blossom.boot=shell'
```

上述步骤显式设置了 `blossom.boot=shell` 以进入交互式 Shell，避免依赖外部输入 DTB 中的默认启动参数。完成镜像下载后，请根据具体开发板型号执行对应的 `bootm` 命令（切勿混用两块开发板的启动参数格式）：[2]

**VisionFive 2：**

```text
bootm ${kernel_addr_r} ${ramdisk_addr_r} ${fdt_addr}
```

**LS2K1000LA（对应 U-Boot `g2ed41674`）：**

```text
bootm ${kernel_addr_r} ${ramdisk_addr_r}
```

这版龙芯 U-Boot 固件通过环境变量 `fdt_addr` 定位 OS DTB，并采用 LoongArch 专属的命令行 / EFI system table 方式向内核交接参数；若在该固件上强行使用三参数形式的 `bootm`，固件在读取命令行指针时会触发内存访问异常。因此切勿照搬其他开发板的通用命令格式；若更换了开发板固件版本，必须重新核对启动交接 ABI。此外，若在固件阶段改用 FAT/ext4 分区读取或 USB 存储加载镜像，仅相当于替换了 U-Boot 的下载命令（如将 `tftpboot` 换为 `fatload`），并不代表内核启动后已支持相应文件系统的运行期读写。

内核命令行中的 `blossom.boot=shell` 会由统一的 BusyBox `PID 1` 及 `/sbin/start-session` 脚本解析并启动交互式 Shell（不会将 Shell 直接作为 `PID 1` 运行）。如果不传 `blossom.boot=shell`，系统默认会话会尝试启动比赛执行器 `/sbin/contest-run /dev/vda`；在没有 `/dev/vda` 比赛盘的实体开发板上，切勿省略会话参数并误以为系统会自动退回 Shell。在交互式 Shell 中退出不会触发比赛运行；需要正常同步停机时，请依次执行 `sync` 和 `/bin/busybox halt`（注意软件层面的正常 Halt 并不等同于硬件物理断电或主板自动冷重启）。

启动后请核对串口日志中的 `ram0` 设备识别、initrd 有效负载地址范围、根文件系统挂载及 `INIT` 进程启动状态。在 RAM 根文件系统中创建文件并停机后，下次重新通过 TFTP 加载原始 `root.uInitrd` 启动时，此前写入的文件自然会消失——这属于内存盘（RAM disk）正常的易失性特征，而非持久化存储故障。由于 RAM 根镜像（包含其中的空闲空间）会常驻占用物理内存，建议按需控制镜像体积，仅打包当前验证所需的程序。

## 5. VF2 SD 与 LA SATA 持久根

持久根分区的选择必须依据目标存储介质上的真实分区表确定，既不能假设磁盘上存在原厂系统分区，也不能假设它是空白磁盘。在 VisionFive 2 上，可使用固件支持的 `mmc list`、`mmc info`、`part list mmc <实际编号>` 查看 SD 卡分区；在 LS2K1000LA 上，则使用 `scsi reset`、`scsi info`、`scsi device 0`、`scsi part 0` 核对板载 SATA 硬盘分区（注意搭载 U-Boot 固件的板载 SPI Flash 与用于存放根文件系统的 SATA 硬盘是完全独立的两个存储设备）。

在 LoongArch 的 OS DTB 中，无论通过 PCI 枚举还是直接 MMIO 映射访问 AHCI SATA 控制器，内核驱动都会读取设备树节点中的 `ports-implemented` 属性（单个 `u32` 位掩码）；该属性值必须依据对应开发板型号与修订版的官方硬件手册填写，并通过实板验证。如果硬件 `PI` 寄存器读出为 `0` 且设备树未声明 `ports-implemented` 属性，驱动初始化将直接报错失败——内核绝不会因为 `CAP` 寄存器报告支持一个端口就盲目猜测使用 `port 0`，也不会依赖用户在启动前先去 U-Boot 控制台手动执行一次 `scsi reset` 来帮内核初始化 `PI` 寄存器。

需要特别注意，即便当前启动选择 `/dev/ram0` 作为根文件系统，设备树中的 AHCI SATA 控制器资源依然必须配置正确；内核在启动自检阶段会初始化所有启用的存储控制器，不会因为某个控制器未被选作根设备就跳过它或忽略其初始化错误。

在为 LS2K1000LA 部署 SATA 持久根时，开发宿主机仅负责生成 `root.ext4` 镜像并提供 TFTP 下载服务，无需拆卸开发板上的 SSD 连接到宿主机，也不在宿主机上执行 `dd` 写盘。首次部署根分区时，直接在开发板 U-Boot 中通过 TFTP 将裸 `root.ext4` 镜像下载到空闲 RAM，再执行 `scsi write <RAM地址> <起始LBA> <扇区数>` 写入目标分区（日常启动时直接挂载该分区即可，无需重复写盘）。注意写入磁盘分区的必须是纯 `root.ext4` 文件系统镜像，严禁将带有 64 字节 U-Boot 头的 `root.uInitrd` 写入磁盘分区。

在本轮实板验证中，已在所选的 Kingchuxing SATA 硬盘上划分并验证了 `/dev/sda2` 分区：起始 LBA 为 `0x3b62800`，长度为 `0x40000` 个 512 字节扇区（即 `128 MiB`）。经用户确认，该操作仅将原有的第一分区表项末尾向回收缩了未分配空间，并未移动或缩容第一分区内已有的 ext4 文件系统，且原有的第三分区（swap）表项保持不变。修改前后的 MBR 扇区备份、磁盘读回校验及本轮镜像包哈希记录在 `build/sata-p2-live/final-evidence.json` 中。需要注意，上述 LBA 数值仅对应这块特定硬盘的分区布局，不能直接套用到其他硬盘上；日常启动无需重复修改分区表或重刷根镜像，后续若需更新磁盘上的根镜像，写入前必须先核对目标分区的实际起始 LBA 与扇区容量。

仓库内的基础 OS DTB 不包含遗留的 `linux,initrd-start` 与 `linux,initrd-end` 属性；在打包不使用 RAM 根的持久盘启动包时，仍需确认输出 DTB 中不含旧的 initrd 地址范围，并将内核参数 `root=` 明确指向目标持久磁盘分区。若使用外部提供的 DTB 且其中带有旧的 initrd 属性，调用者必须先在独立副本中将其清除（打包脚本不会自动删除外部 DTB 已有的属性）。持久盘启动包同样使用 `make package-board-boot-images` 生成，通过 `BOARD_OS_DTB`、`BOARD_ROOT_DEVICE`、`BOARD_ROOT_MODE` 指定参数，并显式传入空值 `BOARD_ROOT_EXT4_IMAGE=` 以排除 ramdisk 打包。建议将 RAM 根启动包与持久盘启动包输出到不同的目录，防止混用两者的 `boot.env` 与 `board.dtb`。

从持久磁盘分区直接挂载根文件系统启动时，无需加载 initrd。以下命令中的分区名仅为示例，上板时必须替换为实际确认的目标分区（内核与 DTB 依然可通过 TFTP 加载到内存）：

```text
# VF2：实际 SD 分区示例
setenv bootargs 'root=/dev/mmcblk0p1 rootdir=/ rw init=/sbin/init blossom.boot=shell'
bootm ${kernel_addr_r} - ${fdt_addr}

# LA：实际 SATA 分区示例
setenv bootargs 'root=/dev/sda2 rootdir=/ rw init=/sbin/init blossom.boot=shell'
bootm ${kernel_addr_r}
```

VisionFive 2 命令中的连字符 `-` 表示不传递 RAM disk 参数；而上述 LS2K1000LA 固件则直接省略第二个参数。如果 `root=` 指定的磁盘分区在硬件上不存在，内核会直接报错停止，绝不会私自改选磁盘上的最大分区或静默回退到 RAM 根。当需要生成持久根专用的 `boot.env` 时，只需在打包时设置实际的 `BOARD_ROOT_DEVICE` 并省略 `BOARD_ROOT_EXT4_IMAGE`（将根镜像写入物理磁盘分区仍是独立的部署步骤）。操作全程请使用独立的 OS DTB 地址，不要覆盖固件自身仍在使用的 control FDT。

## 6. 持久性与普通用户测试

验证磁盘分区持久化写入时，先在以读写（`RW`）模式挂载的持久根文件系统中写入标记文件并执行落盘同步：

```sh
mkdir -p /root/board-check
printf '%s\n' 'persistent board root' > /root/board-check/marker
sync
/bin/busybox halt
```

观察串口日志确认内核已完成文件系统同步、外设停用与各核 CPU 停止后，再次重新启动并挂载**同一个持久磁盘分区**（期间不要重新刷写源 `root.ext4` 镜像，也不要还原分区内容）。上述命令请求的是操作系统正常同步停机（Halt），并不代表硬件自动断电或自动冷重启成功；人工按复位键（RESET）、固件热重启与物理断电重启应分别独立记录。第二次启动进入系统后，读取 `/root/board-check/marker` 核对内容，并运行跨页读写、共享内存映射（`mmap`）、`rename`/`truncate`、`fsync`/`sync` 等测试程序。需要区分的是：同一轮运行中命中内存页缓存（Page Cache）的热读取不能代替跨重启的冷启动磁盘重读，而 U-Boot 固件能识别到 SSD 也不代表内核的 AHCI 驱动请求已经完成。

镜像中已安装的用户态自测套件可作为普通应用程序在开发板 Shell 中手动运行，无需引入开发板专属的测试执行器或特殊通信协议：

```sh
/sbin/user-selftests --list
/sbin/user-selftests --collection fs
cd /test/selftests/fs
./file_shared
```

`/sbin/user-selftests` 会解析并汇总各测试程序的真实 TAP 输出与退出码，运行结束后直接返回 Shell 而不会自动关机；辅助程序（helper）不计入默认通过数统计。比赛评测配置文件依然位于 `/etc/contest/`，在开发板上运行前需先按照实际块设备挂载包含原始测试程序、动态链接器与共享库（DSO）的分区，再调用 `/sbin/contest-run` 执行对应的测试计划。需要注意功能边界：在内核未实现 `loop` 回环块设备时，不能声称可以直接 `mount` 普通磁盘镜像文件；同理，仅使用过 USB 存储介质也不能等同于原生 SD 卡、SATA 硬盘或 USB 根文件系统均已验证通过。

日常迭代时，若仅修改了内核代码，只需通过 TFTP 更新内核镜像及必要的 DTB；若仅修改了单个用户程序或配置文件，只需在持久根中更新对应文件并执行 `sync`（RAM 根模式则重新打包加载 `root.uInitrd`）。即使需要停机更新持久存储介质上的文件，也应保留分区内其余已有的测试修改，无需每次测试都全盘重刷整个根分区。突然断电场景下未同步脏数据的崩溃恢复、ext4 日志（journal）以及额外的压力或覆盖率测试框架均不在当前支持范围内。

## 7. 运行边界与记录

### 仓内设备树的本次 VF2 验证

当前版本的 `release` 内核配合仓库内源码编译的 VisionFive 2 设备树，已在实板上分别完成了 RAM 根模式与 SD 卡持久根模式的启动验证：两种模式均成功启动全部 4 个应用核（`READY`）、挂载指定的唯一读写（`RW`）根文件系统并进入 `PID 1` / BusyBox Shell；文件共享测试（`file_shared`，2 项 TAP 断言）与 CPU 亲和性测试（`affinity`，5 项 TAP 断言）均全部通过，两个测试程序及 Shell 均以退出码 `0` 正常结束，并在 `sync` 返回 `0` 后完整执行了正常停机（Halt）。其中第 1 次启动使用 `/dev/ram0` 作为根文件系统；随后在不重刷 SD 卡的前提下，经人工按下 RESET 键进行第 2 次启动并挂载 `/dev/mmcblk0p1`，此前保存在 SD 卡上的标记文件只读校验完全一致。两次运行的内核 ELF、根镜像、DTB 哈希、固件 CRC32 校验值及完整串口命令记录详见 `build/dts-inrepo/{vf2-ram-native,vf2-sd-native}.json` 与 `vf2-uart.log`。

在 SD 卡启动阶段，U-Boot 固件开机自动扫描曾打印一条 `Card did not respond to voltage select! : -110` 报错（后续未在固件中重复执行卡探测）；但本内核启动后通过自身驱动成功识别了该 SD 卡、挂载根分区并跑通了全部文件自测（此处仅如实记录现象，不代表已修复固件自身的电压切换报错）。由于板上实时时钟（RTC）尚未校准外部标准时间，启动日志中的 `source=unset` 仅表示使用默认零起点计时，不代表可信的 UTC 墙上时间。此外，这两次软件 Halt 配合期间的人工按键 RESET，不等同于硬件自动冷重启（Restart）、自动断电（PowerOff）或突发掉电恢复测试。

这两轮验证最终均以 `/bin/busybox halt` 完成正常停机，串口设备已释放交还，本轮日志记录已正常关闭（开发机上用户原有的 TFTP 服务保持运行）。对于 LS2K1000LA，新整理的仓内设备树源码已完成离线等价核对与两种启动模式的镜像打包，但本轮尚未在 2K1000LA 实体板上重新上电运行。完整状态记录详见 `build/dts-inrepo/evidence.json`。

### RTC 授时边界修复后的完整 OS DT 实板结果

修复实时时钟（RTC）日历校验边界后，VisionFive 2 普通 `debug` 内核（ELF SHA256 为 `f9e09e65387c23c5f451d3e11cb0193c7d39cbc6d3e6152f548bfcc635e07e75`）已使用同一份完整设备树在实板上运行成功。打包前后的设备树均保持完整的 `292` 个节点与 `1924` 个属性（未裁剪 RTC、CMA、ccache 别名或两个处于 `enabled` 状态的 GMAC 节点）；本次运行不使用 initrd，直接挂载已有的 `/dev/mmcblk0p1` 作为唯一读写（`ReadWrite`）根文件系统。启动前在 U-Boot 中核对 `kernel.uImage` 与 `board.dtb` 的全文 CRC32 分别为 `0407a341` 与 `861f605e`，与宿主机构建产物完全一致，仅执行一次 `bootm` 即一次成功。

修复方案参考 Linux 内核的通用做法，将可选的硬件 RTC 墙上时间授时失败与系统单调递增计数器（monotonic counter）的初始化解耦：内核依然保留真实的硬件寄存器采样与公历日期合法性校验，但当读出未初始化的无效日期、RTC 硬件缺席或读取超时报错时，内核会打印明确的诊断日志并以未授时的 Unix 纪元零点（`source=unset`）继续启动，既不会向硬件 RTC 强行写入假日期，也不会用编译时间戳伪造系统时间。本次实板运行采样解码得到的原始日历字段为 `[2000, 0, 0, 0, 40, 4]`，因月份与日期为 `0` 不满足公历规则，日志明确记录 `source=unset`（注意该数组是解码后的日历字段而非两个原始硬件寄存器的十六进制转储，不能用来倒推下一小节旧内核失败时的原始采样值，也不代表硬件 RTC 日期已被校准）。此后系统的 `CLOCK_REALTIME` 仅由零起点加上单调时钟流逝时间推算，不能作为已校准的 UTC 时间使用。

在本次运行中，内核正确登记了 `0x70000000..0x90000000` 保留内存区，唤醒全部 4 个应用核达到 `READY cpus=4`，完成原生 SD 卡根挂载、`GMAC1` 网卡 PHY 初始化，并顺利进入 `PID 1` / BusyBox Shell。Shell 交互与 `true` 命令执行正常，已安装的 `fs:file_shared`（2 项 TAP 断言）与 `process:affinity`（5 项 TAP 断言）全部通过，两个测试程序退出码均为 `0`，汇总结果为 `pass 2 / fail 0 / error 0 / status 0`。系统运行期间，`/proc/uptime` 从 `267.45 1069.10` 正常推进至 `271.63 1085.78`，执行 `busybox sleep 1` 正常返回 `0`（此处仅验证单调时钟推进，不将命令执行间隔当作高精度时钟频率校准证明）。由于当前用户态配置未编译 `date` 与 `uptime` 两个 BusyBox 小程序（applet），直接调用它们仍返回退出码 `127`（本次未额外扩展用户态 applet，因此不宣称日期命令行工具已验证通过）。

测试末尾执行 `sync` 返回 `0`，随后执行一次 `busybox halt`，完整走完 `PID 1` 退出（`0`）、网络子系统停止（`NET`）、根文件系统同步卸载（`ROOTFS`）、跨核停机确认（`CPU ack 14`）及当前核时钟事件源关闭全流程，随后释放串口并关闭本轮临时 TFTP 与日志记录（同样不等同于硬件自动 Restart、PowerOff 或掉电测试）。本轮针对 RTC 修复的回归验证未重复运行内核网络吞吐测试，因此不将下一节旧版内核 ELF 的 ARP/ICMP/TCP 实测数据直接挂靠到本次新 ELF 名下；全程也未改写 SD 卡分区表、未刷写固件、未执行 `saveenv` 或修改硬件 RTC 寄存器。完整的输入文件哈希、串口命令序列、原始 UART 日志以及双架构定向回归结果保存在 `build/vf2-rtc/{final-evidence.json,package-identity.json,uart.log}` 中。

### 修前完整 OS DT 的实板失败

在修复 RTC 边界前，基于源码提交 `43389aa1` 编译的 VisionFive 2 普通 `debug` 内核（ELF SHA256 为 `2ca3e55d973c71f1dcf7f7506624bdd7ac8ee8a373c2bede0903ea405bea6ddd`）曾使用相同的完整设备树（`292` 个节点 / `1924` 个属性，完整保留 CMA 动态预留、ccache 三段 `reg` 与两个 `enabled` GMAC）在实板上执行过一次无 initrd 启动。启动前固件核对 `kernel.uImage` 与 `board.dtb` 的 CRC32 分别为 `ab3e5a2d` 与 `861f605e`，与宿主机输入一致。

该次运行在串口输出了 `BOOT platform=VisionFive2 cpus=4 bsp=1`、`MEMORY` 与 `BUDDY` 初始化日志，随后在 `FAIL panic: panicked at src/time.rs:337:49` 处中止。该处代码是 RTC 原始日历数据向时间戳转换时的 `.expect(...)` 调用：硬件采样虽然返回了数据，但未通过通用公历合法性校验；由于该版本的 panic 日志未打印解码出的六个年月日时分秒字段，仅凭当次日志无法断定究竟是硬件 RTC 未设置初始时间还是寄存器采样/解码存在偏差。

因此，修前的那次运行属于启动中途失败（Fail）：当时内核仅解析到了 4 个 CPU 节点，尚未唤醒从核（AP）达到 `READY` 状态，也尚未执行块设备与 SD 根挂载、`GMAC` 网络收发、`PID 1`/TTY/Shell 启动及正常停机（U-Boot 固件阶段能够使用 `GMAC1` 完成 TFTP 下载，并不代表内核态网卡驱动已工作）。当时并未采取禁用设备树 RTC 节点、手动设置硬件时间、修改驱动、重刷 SD 卡或更换固件等规避手段。该次失败的原始串口日志与输入文件哈希保存在 `build/vf2-full-native/{native-summary.json,uart.log,package-identity.json}` 中；上一小节修复后的成功运行则独立保存在新目录中，不会覆盖或抹除本次历史失败记录。

### 此前各 ELF 的实板记录

下表及后续关于网卡 PHY、网络协议栈与停机流程的记录，均对应各自历史测试时点所使用的特定内核 ELF 与设备树输入，不自动迁移作为其他内核版本或新设备树输入的验证结论：

| 独立验证路径 | 对应历史版本的实板验证状态 |
|---|---|
| VF2 RAM 根 | 已通过正式 `bootm` 启动全部 4 个应用核并进入 `PID 1`/TTY/Shell；已完成文件读写与 `sync`、重新加载同一原始 RAM 镜像后的易失性验证及正常软件 Halt；通过 SBI 触发硬件冷重启（cold restart）时遇到的 PMIC 报错现象如实保留 |
| VF2 SD 分区直接根 | 已验证挂载 `/dev/mmcblk0p1` 作为唯一读写根、连续两次无 initrd `bootm` 启动、运行现有 Rust `std` 与普通用户程序，以及写入 `marker` 与 `8229` 字节文件并在执行 `sync` + `halt` + 人工按键 RESET 后冷启动重读校验一致；未验证硬件自动 SBI restart、自动断电（poweroff）或突发掉电恢复 |
| LA RAM 根 | 已在 U-Boot `g2ed41674` 下通过 `bootm` 启动进入 `PID 1`/Shell，并验证了子进程退出回收及 RAM 盘文件读写与 `sync`；实板停机与重启流程尚未验证 |
| LA SATA 分区直接根 | 已验证挂载 `/dev/sda2` 连续两次真实 `bootm` 启动进入 `PID 1`/Shell、文件读写落盘，以及执行正常同步 Restart 后重新启动读回校验一致；未验证突发掉电恢复或其余外设寄存器窗口 |

每次实板测试均需归档所选内核 ELF、磁盘镜像与 DTB 的哈希身份、实际 `bootargs`、完整 UART 串口日志、存储设备与分区 LBA 范围、I/O 请求完成情况以及停机/重启方式。原生网卡（MAC/PHY）与内核网络栈必须依据实板内核态收发证据记录（固件阶段的 `ping` 通不能证明内核网络栈可用；下文列出的 VisionFive 2 网络实测数据仅代表该次测试的特定内核版本，不能外推至 LoongArch 或其他网口）。同理，多核（SMP）验证需要观察到实际的跨核任务迁移或并行执行，不能仅看启动日志中的 CPU 上线数量。

当前 VisionFive 2 设备树保留了真实的 `ethernet0` / `ethernet1` 别名（alias），单网卡驱动明确绑定 `ethernet1`（在系统内注册为逻辑接口 `eth0`），而 `GMAC0` 节点即使保持 `enabled` 状态也不会被当前驱动认领。早期 `da986c64` 版本内核在实板测试时曾使用仅启用 `GMAC1`（`/soc/ethernet@16040000`）的裁剪版设备树，但这并非当前新版完整设备树的前提条件。若所选网卡硬件损坏或初始化失败，驱动会直接返回原始错误，不会私自切换到另一网口或回退到本地回环接口（`lo`）。以下 PHY 识别与网络收发数据专门对应 `da986c64` 版本内核：驱动实测读出 PHY ID 为 `0x4f51e91b`、PHY 地址为 `0`、MDC 时钟分频系数为 `CSR divider 4`（PHY 地址取自设备树子节点属性 `reg = <0>`，而非节点名称中的 `ethernet-phy@1`）；当设备树未提供 MAC 地址时，驱动从控制器硬件寄存器读出 MAC 地址 `6c:cf:39:00:7a:7d`，且与宿主机端二层抓包看到的源 MAC 完全一致。该次实验未修改 PHY 或 DMA 硬件参数，也未逐一读回全部时钟复位（CRG）寄存器字段或链路协商速率寄存器。

该次网络实测的完整证据归档于 `build/stage11-vf2/vf6-native-final-evidence.json`，对应的内核 ELF SHA256 为 `da986c641625b80b2309c5f14f41a08a8b67933a0005fc9ad135fecf518fd7d0`。该内核同样遵循无 initrd 的三参数 `bootm` 流程直接挂载 `/dev/mmcblk0p1` 读写根启动（无需重刷 SD 卡）。由于系统内置网络初始化脚本默认配置的 `10.0.2.15/24` 与网关 `10.0.2.2` 仅适用于 QEMU `user` 网络模式，实板测试时先通过 `route -n` 确认该默认路由存在并将其删除，随后为开发板配置了实板局域网地址 `192.168.77.2/24` 以直连宿主机 `192.168.77.1`，最终路由表中仅保留 `lo` 与 `192.168.77.0/24` 直连路由。后续在实板上配置网络时，应根据当前路由表的实际条目操作，不要盲目删除不存在的路由或忽略命令报错，也无需改动宿主机防火墙规则。

在该次测试中，ARP 解析与双向 ICMP 回显均实测通过（ICMP 报文在以太网二层帧长 `98` 字节、IP 包长 `84` 字节、有效负载 `56` 字节下全文校验一致）。TCP 短连接测试中，宿主机主动连接开发板 `18087` 端口，双向各传输 `25` 字节数据完整无误，宿主机顺利收到回复与连接关闭（EOF），开发板测试进程以状态码 `0` 退出。在另一组大流量 TCP 传输测试中，宿主机向开发板发送 `87194` 字节数据，开发板校验后回复 `VF6_BIG_106006\n` 并以状态码 `0` 退出；包含末尾换行符（LF）与后缀哨兵在内的两端 `87195` 字节流全文比对完全相等。该次大流量传输期间，网卡接收统计（RX）从 `731` 字节 / `11` 帧增长至 `91419` 字节 / `75` 帧，发送统计（TX）从 `3237` 字节 / `52` 帧增长至 `4506` 字节 / `75` 帧；接收端净增的 `64` 个以太网帧证明了容量为 `32` 槽位的接收描述符环（RX ring）能够正常循环复用与推进（但这仅证明环槽位成功回绕复用，不等同于对每个描述符物理地址或所有硬件故障注入场景的穷尽验证）。测试结束时，网卡收发错误计数（`errors`）与丢包计数（`drops`）均为 `0`，中断源 `78` 累计触发 `81` 次 IRQ，中断源 `75` 累计触发 `995` 次 IRQ；同一轮运行中执行的 `file_shared`（2 项 TAP）与 `affinity`（5 项 TAP）自测也全部以退出码 `0` 通过（`summary pass 2 / status 0`）。

需要如实说明的是，开发板主动发起连接宿主机（Host 入站方向）的场景当时仍有一项未完成：开发板尝试连接宿主机 `18086` 端口未能建立连接，且同一时间窗口内在宿主机 WSL `eth2` 接口上未抓到入站 `SYN` 报文，具体根因尚未定论（文档如实保留该未完成状态，不将其写成已修复，也不武断归咎于宿主机防火墙；反向的宿主机连开发板 TCP 成功不能替代该方向的测试结果）。此外，当前精简的用户态环境不支持 `ifconfig` 无参数状态展示，也未编译 `cmp` 与 `stat` 小程序；测试中对包含换行符的长 ASCII 字节流采用后缀哨兵配合全文字符串相等性比对进行校验，未因此额外扩展 BusyBox 小程序、错误码或内核 ABI。

该 `da986c64` 版本内核在测试末尾同样执行 `sync` 返回 `0`，并顺利完成正常 Halt（`PID 1` 退出 `0`、`NET` 停止、`ROOTFS` 同步卸载、`CPU ack 14` 及本核事件源关闭），随后释放交还了串口。此前通过 SBI 请求冷重启时遇到的 PMIC 报错现象保持原样记录（未重复尝试）；该轮同样未进行物理断电（poweroff）、自动重启或掉电恢复实验。

至此，VisionFive 2 阶段计划的 `C01–C48` 共 `48/48` 项检查点已全部完成；对应的五域独立终审结论为 `correct`（置信度 `0.97`），原始记录保存在 `vf6-c47-final-five-domain-review.json` 中。相关源码与文档已通过 `jj` 保存，开发板串口在完整 Halt 状态下释放交还（详见 `vf6-final-handback.json`）。四个构建配置的 Clippy/格式检查、双架构公共五项行为验证（共 `10 Pass`）及最终代码格式检查均与当前各自的构建产物哈希对应（其中公共启动测试中的 `resources={}` 仅代表基础启动检查，不计入内存管理或元数据专项等价账目）。LS2K1000LA 既有的验证结果与分区表备份保持不变，LoongArch 的不可执行页保护（NX）仍作为已知暂缓项记录（不计为 Pass）；历史代码图索引工具中的 `UNKNOWN`/`private`/跨模块误报或编辑器 LSP 未启动状态不作为代码安全性证明依据。

排查实板问题时，请严格遵循 **固件装载 → 内核入口 / FDT 解析 → 物理内存与 CPU 初始化 → 中断控制器（IRQ） → 外设控制器命令与 DMA 传输 → ext4 文件系统 → ELF 加载与动态运行库 → 用户态会话选择** 的自底向上顺序逐层定位；遇到错误应在对应的真实层级修复，切勿通过削减 CPU 核数、硬编码猜测地址、盲目重放未知的磁盘写操作或悄悄切换根文件系统来掩盖故障。内核具名测试框架（`ktest`）仅支持双架构 QEMU 环境，不提供实体板串口加载与传输通道。LoongArch NX 仍属暂缓项（非 Pass）；宿主机离线打包成功或 QEMU 仿真通过均不能代替上述四条实体板运行路径的真实硬件验证。

[1]: https://docs.u-boot.org/en/latest/usage/cmd/fdt.html
[2]: https://docs.u-boot.org/en/v2024.01/usage/cmd/bootm.html
