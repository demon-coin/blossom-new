# 人工上板、根卷与应用验证

本指南使用当前生产接口，不是实板通过记录。支持组合为 riscv64/visionfive2、loongarch64/ls2k1000la；QEMU 共用 [开发指南](development.md) 的唯一运行入口。两板的固件、OS DTB、RAM、介质和串口必须由实际设备确认，构建与包装不连接串口、不刷盘、不执行 saveenv，也不依赖 ktest。

## 1. 首次接触先记录实际输入

在固件中只查看支持的命令和当前设置：

```text
version
bdinfo
printenv bootcmd bootargs loadaddr kernel_addr_r fdt_addr ramdisk_addr_r
help bootm
help bootelf
help fdt
```

保存板修订、实际 CPU/RAM、固件版本、串口参数、OS DTB 来源、保留区及可用装载空间。某固件缺少命令就保留该事实，不从另一个项目推断支持。串口电平与针脚以本板手册为准。

U-Boot control FDT 服务固件自身，working FDT 才交给 OS；有实际 OS DTB 时独立载入，若需复制 control FDT 则先复制到不重叠空间，再按已知硬件修改，不覆盖固件仍引用的树。[1]

能运行原厂 Linux 时，可记录 `uname -m`、`/proc/iomem`、设备树 model 及 `lsblk -o NAME,SIZE,TYPE,FSTYPE,MOUNTPOINTS`。这些信息用于选择实际介质和分区，不要求 BLOSSOM 增加同名工具。

## 2. 分清离线文件与运行期设备

| 对象 | 语义 |
|---|---|
| `kernel.elf` | 带符号内核，entry/LOAD/BSS 范围是布局事实来源 |
| `kernel.bin` | 按物理 LOAD 位置排列的文件字节，不含 ELF/调试容器 |
| `kernel.uImage` | 带实际架构、load、entry 和 CRC 的固件容器 |
| `board.dtb` | 本次提供的 OS DTB，保留包装所需扩展空间 |
| `root.ext4` | 首字节为 ext4 卷起点；可放 RAM 或部署到明确分区 |
| `root.uInitrd` | U-Boot ramdisk 头加未压缩 raw ext4；固件传 payload 范围 |
| 整卡或整 SSD 镜像 | 可能含分区表和引导区，不等于一个 raw ext4 卷 |
| 官方比赛盘 | 原程序、解释器、DSO 与清单输入；不得重制来替换原评测来源 |

文件后缀不证明格式，不把 ext4 改名当 gzip、UBI 或厂商升级包。固件从 SD/TFTP 载入内核，与内核运行期从 SD/SATA 访问文件是不同路径；RAM 根不证明原生盘、DMA 或 IRQ 已运行。

CPU 物理地址、CPU 虚拟地址和设备 DMA 地址也不同：FDT/PCI 给出物理 MMIO 资源，内核映射后由 CPU 虚拟地址访问；DMA 地址来自本设备 domain 的窗口、mask 和映射。非一致设备还需要真实 cache 维护，不能以 fence/dbar 代替。纯 Rust 布局输入和编辑器改造不改变这些硬件地址语义。

## 3. 普通构建、装配和离线包装

只改内核时，不必重制根卷：

```sh
make build-kernel ARCH=riscv64 TARGET_PLATFORM=visionfive2 KERNEL_BUILD_PROFILE=release
make build-kernel ARCH=loongarch64 TARGET_PLATFORM=ls2k1000la KERNEL_BUILD_PROFILE=release
```

工具链、目标 std、交叉 GCC 和 libc 来源按开发指南准备，不下载或升级工具链。用户树和 raw ext4 分开构建；以下输出为本次拥有的可重建路径：

```sh
make assemble-root-filesystem ARCH=riscv64 ROOTFS_OUTPUT_DIR=build/board-root-rv
make create-ext4-image ARCH=riscv64 \
  ROOTFS_TREE_DIR=build/board-root-rv SYSTEM_ROOT_EXT4_IMAGE=build/board-root-rv.ext4
make assemble-root-filesystem ARCH=loongarch64 ROOTFS_OUTPUT_DIR=build/board-root-la
make create-ext4-image ARCH=loongarch64 \
  ROOTFS_TREE_DIR=build/board-root-la SYSTEM_ROOT_EXT4_IMAGE=build/board-root-la.ext4
```

两卷各自以 `/` 为根，含匹配 ISA 的 `/sbin/init`、Shell、std 执行器和配置，不含另一架构目录。已有外部完整树直接交 `ROOTFS_TREE_DIR`，制盘不向其补程序或改写输入。保留支持的无 journal 格式，不启用 replay/recovery。

准备实际 OS DTB 后包装 RAM 启动输入；下列 DTB 路径必须替换为真实输入：

```sh
make package-board-boot-images ARCH=riscv64 TARGET_PLATFORM=visionfive2 \
  KERNEL_BUILD_PROFILE=release BOARD_BOOT_IMAGES_OUTPUT_DIR=build/vf2-boot \
  BOARD_ROOT_EXT4_IMAGE=build/board-root-rv.ext4 BOARD_ROOT_DEVICE=/dev/ram0 \
  BOARD_ROOT_MODE=rw BOARD_OS_DTB=/absolute/actual-visionfive2-os.dtb
make package-board-boot-images ARCH=loongarch64 TARGET_PLATFORM=ls2k1000la \
  KERNEL_BUILD_PROFILE=release BOARD_BOOT_IMAGES_OUTPUT_DIR=build/2k-boot \
  BOARD_ROOT_EXT4_IMAGE=build/board-root-la.ext4 BOARD_ROOT_DEVICE=/dev/ram0 \
  BOARD_ROOT_MODE=rw BOARD_OS_DTB=/absolute/actual-ls2k1000la-os.dtb
```

`BOARD_ROOT_EXT4_IMAGE` 是只读输入，不是制盘输出。构建和包装统一通过根 Makefile 的 `package-board-boot-images`；包装从最终普通 ELF 的公开符号和 LOAD 取得实际布局，不导入测试包、不要求测试节或展开信息。

每次包装产生 `kernel.elf`、`kernel.bin`、`kernel.uImage`、`boot.env`，有对应输入才产生 `root.uInitrd`、`board.dtb`。`boot.env` 只提供装载地址和内核参数，不生成启动命令；缺 DTB 或明确 root 时不能据此认定可部署成功。LoongArch 容器使用实际架构编码，不伪装成 MIPS。

有 RAM 根输入时，包装器在输出 DTB 副本中写入本次 `root.uInitrd` 的准确 payload 范围，排除 64 字节 legacy 头；输入 DTB 和卷均保持不变。此步骤需要 PATH 中的 `fdtput`，失败不会发表本次包。

`boot.env` 区分容器暂存地址、物理 `kernel_load/kernel_entry`、ELF entry 和完整 `kernel_mem_end`，并列出 DTB/initrd 范围。两板统一使用 `fdt_addr`，供下载和固件交接使用。LoongArch 的 `kernel_addr_r`、`ramdisk_addr_r`、`fdt_addr` 使用 cached DMW 地址，容器头和 initrd 属性保留物理地址；RISC-V 运行地址仍为物理地址。包装检查对象间不重叠，但不能证明它们落在实板可用 RAM 内，须与实际 RAM、保留区和整个 BSS 范围共同核对。

## 4. RAM 根：两板分别人工启动

固件须支持文本环境导入和对应的 `bootm` 交接方式，已有 `loadaddr` 须位于可用暂存 RAM。先下载并导入本次 `boot.env`，每条成功后才继续；`${filesize}` 在导入前必须仍是环境文件的长度。只修改当前 RAM 环境，不执行 `saveenv`：

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

本节显式设置 `blossom.boot=shell`，不假设外部输入的 DTB 已选择 Shell 会话。完成装载后，直接执行对应板的 `bootm`，不把两板的启动参数混用。[2]

**VisionFive 2：**

```text
bootm ${kernel_addr_r} ${ramdisk_addr_r} ${fdt_addr}
```

**2K1000LA 的 U-Boot `g2ed41674`：**

```text
bootm ${kernel_addr_r} ${ramdisk_addr_r}
```

此版龙芯固件由 `fdt_addr` 指定 OS DTB，使用 LoongArch 的命令行／EFI system table 交接。实际三参数形式在固件命令行指针读取处触发异常；不能仅凭通用帮助套用其他板命令。更换固件须重新核对 ABI。改用固件 FAT/ext4/USB 加载只替换 load 命令，不代表内核运行期支持这些设备。

`blossom.boot=shell` 经同一 BusyBox PID1/start-session 选择 Shell，不把 Shell 改成 PID1。默认会话会运行比赛执行器 `/sbin/contest-run /dev/vda`，板上没有该比赛设备时不得省略会话选择后假定自动进入 Shell。Shell 退出不转跑比赛；正常结束用 `sync` 和 `/bin/busybox poweroff`。

观察实际 `ram0`、initrd payload 范围、根卷及 INIT。写一份文件后结束，再重新加载原 RAM 卷时文件消失，这是 RAM 易失性，不是持久盘失败。RAM 卷包括空闲空间在内都占 RAM，只装当前需要的程序。

## 5. VF2 SD 与 LA SATA 持久根

介质布局按实际盘面确定，不假设有原厂系统，也不假设是空盘。VF2 用固件实际支持的 `mmc list`、`mmc info`、`part list mmc <实际编号>`；2K1000LA 沿用 `scsi reset`、`scsi info`、`scsi device 0`、`scsi part 0` 核对板上 SATA。U-Boot 的 SPI Flash 与 SATA 根卷不是同一个存储对象。

LA 的 OS DTB 中，PCI 与直接 MMIO 路径都接收 SATA 节点的 `ports-implemented`（单个 u32 bitmask），值须来自匹配板型与修订的资料，并经实际配置验收。PI 为零且未声明该属性时仍失败，不能因 CAP 报告一个端口就自动猜测 port0，也不能依赖先运行固件 `scsi reset` 初始化 PI。

RAM 根同样需要正确的 AHCI 资源；内核不会跳过未选作根的设备，也不会忽略其资源或初始化错误。

2K1000LA 的主机只生成镜像和提供 TFTP，不拆 SSD 接宿主，不在宿主执行 `dd`。首次部署由板卡 U-Boot 将 raw ext4 下载到可用 RAM，再执行 `scsi write <RAM地址> <起始LBA> <扇区数>`；普通启动不重复写盘。写入对象是 ext4 卷本身，不能使用带 legacy 头的 `root.uInitrd`。

历史板端记录为 SCSI 设备 0、DOS 第一分区起始 LBA `0x40003f`，若明确选择覆写该旧根卷，当前内核根名为 `/dev/sda1`；这不表示本轮镜像已部署到该分区，也不构成覆盖授权。写入前必须核对当前磁盘、分区边界、扇区大小和镜像长度。不能因固件位于 SPI 就把 SSD 认定为空盘并从 LBA 0 覆盖，也不无依据地声称 SATA 上有原厂系统。较大卷按可用 RAM 分块传输，每块的长度和写入 LBA 独立计算，不沿用历史镜像的最后一块长度。

无 RAM 根的独立 OS DTB 必须删除 `/chosen/linux,initrd-start` 与 `linux,initrd-end`，并将 `root=` 改为同一明确持久分区；不能直接沿用 RAM 启动树中的旧范围。统一使用 `make package-board-boot-images`，以 `BOARD_OS_DTB`、`BOARD_ROOT_DEVICE`、`BOARD_ROOT_MODE` 指定输入，并用 `BOARD_ROOT_EXT4_IMAGE=` 排除 ramdisk。先前的 RAM 包和磁盘包使用不同输出目录，避免混用环境文件与设备树。

直接根启动不加载 initrd。分区名仅示例，必须替换为实际目标；kernel/DTB 仍可由 TFTP 载入：

```text
# VF2：实际 SD 分区示例
setenv bootargs 'root=/dev/mmcblk0p2 rootdir=/ rw init=/sbin/init blossom.boot=shell'
bootm ${kernel_addr_r} - ${fdt_addr}

# LA：实际 SATA 分区示例
setenv bootargs 'root=/dev/sda1 rootdir=/ rw init=/sbin/init blossom.boot=shell'
bootm ${kernel_addr_r}
```

VF2 示例中的 `-` 表示无 RAM disk；上述 LA 固件不传第二参数。不存在的明确分区应失败，不能改选最大分区或 RAM 根。需要持久根的 `boot.env` 时，包装选择实际 `BOARD_ROOT_DEVICE`，不传 `BOARD_ROOT_EXT4_IMAGE`；根卷部署仍是独立步骤。保持实际 OS DTB，不覆盖固件仍在使用的 control FDT。

## 6. 持久性与普通用户测试

在明确 RW 的根中写入并同步：

```sh
mkdir -p /root/board-check
printf '%s\n' 'persistent board root' > /root/board-check/marker
sync
/bin/busybox poweroff
```

确认真实结束，再启动**同一持久分区**，不重新部署来源 root.ext4、不刷新或还原卷。读取 marker，并运行已有跨页图样、共享映射、rename/truncate、fsync/sync 相关程序；停止后从外部读取和冷启动重读分别记录。页缓存热读不能代替冷重开，固件看到 SSD 也不能代替内核 AHCI 请求完成。

已安装的用户自测作为普通程序人工运行，不加板专属执行器或测试协议：

```sh
/sbin/user-selftests --list
/sbin/user-selftests --collection fs
cd /test/selftests/fs
./file_shared
```

`user-selftests` 返回真实程序/TAP 汇总但不自行关机；辅助程序不计默认通过。比赛配置仍在 `/etc/contest/`，先按真实设备挂载和选择原盘、解释器、DSO，再运行对应 plan。只提供镜像文件而没有 loop 设备时，不能宣称可以直接 mount 任意镜像；仅有 USB 介质也不叫原生 SD/SATA/USB 根已通过。

改内核只更新内核与必要 DTB；改用户程序或配置只更新对应文件并 sync，RAM 模式重载更新卷。需要关机更新持久介质时保留其余实际修改，不为每次测试重新刷整卷。断电未同步数据的恢复、journal 和新的压力/覆盖率框架不在本轮范围。

## 7. 运行边界与记录

| 必须分开的路径 | 当前硬件状态 |
|---|---|
| VF2 RAM 根 | 待用户实际固件/OS DTB/RAM 启动 |
| VF2 SD 分区直接根 | 待原生 SD、DMA/IRQ 与同介质同步重启 |
| LA RAM 根 | 本轮 `g2ed41674` 经 bootm 到 PID1/Shell、子进程退出及 RAM 文件写读/sync 已验；实板停止和重启未验 |
| LA SATA 分区直接根 | 待原生 AHCI、DMA/IRQ 与同介质同步重启 |

每次保留所选 ELF/卷/DTB 身份、实际 bootargs、完整 UART、设备/分区范围、请求与完成、真实结束及重启输入。原生 MAC/PHY/网络还需各自实板证据，固件 ping 不证明内核网络。SMP 要观察任务迁核或真实并行，不只看上线数量。

定位按固件装载 → 入口/FDT → MM/CPU → IRQ → 控制器命令与 DMA → ext4 → ELF/运行库 → 用户选择逐层判断；错误归原层处理，不降核、猜地址、重放未知写或切换根来掩盖失败。ktest 仅双 QEMU，不提供 serial/loader/实体板 transport。LA NX 既有暂缓非 Pass；离线输入和 QEMU 软件验证不替四条实板路径签名。

[1]: https://docs.u-boot.org/en/latest/usage/cmd/fdt.html
[2]: https://docs.u-boot.org/en/v2024.01/usage/cmd/bootm.html
