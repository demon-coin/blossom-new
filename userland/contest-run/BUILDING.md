# 比赛 runner 构建

本目录是独立的用户态 Rust `std` 包 `blossom-contest-run`（库名为 `contest_run`，可执行程序名为 `contest-run`）。与使用裸机（bare-metal）目标的内核不同，本包面向标准 Linux musl 用户态环境编译，不设置默认的编译 target，也不读取内核的 `MODE`、`PANIC` 或 `build-std` 配置。

本包的直接第三方依赖固定为：`serde 1.0.217`、`toml 0.8.19`、`libc 0.2.169` 与 `glob 0.3.2`。目录下的 `Cargo.lock` 与独立的 `vendor/` 目录均由 Cargo 正式生成并随仓库源码提交；`.cargo/config.toml` 配置为仅从本地 `vendor/` 读取离线依赖，构建过程绝不联网下载依赖、安装工具链或补充组件。编译器版本固定为事先准备好的 `nightly-2025-01-18`；若宿主机缺少该版本工具链、对应的官方 Linux musl 目标标准库或交叉链接器，构建将直接报错退出，不会用旧编译产物顶替。

与内核构建一致，Makefile 直接调用 `PATH` 中已就绪的 `cargo` 与 `rustc`（根目录 `rust-toolchain.toml` 声明了固定版本）。Makefile 不会传递 `+toolchain` 参数，不强制要求通过 `rustup` 代理调用，也不会自动切换编译器版本；当环境中存在 `rustup` 代理时，构建脚本会显式设置 `RUSTUP_AUTO_INSTALL=0` 禁止自动联网安装。开发者可以使用个人 Nix 环境提供这些工具，但项目构建本身不依赖 Nix。

在仓库根目录下执行以下命令即可单独编译比赛执行器：

```sh
make build-contest-runner ARCH=riscv64 RUNNER_SYSROOT=/absolute/prepared/sysroot RUNNER_LINKER=riscv64-linux-musl-gcc
make build-contest-runner ARCH=loongarch64 RUNNER_SYSROOT=/absolute/prepared/sysroot RUNNER_LINKER=loongarch64-linux-musl-gcc
```

未显式指定 `RUNNER_SYSROOT` 时，默认使用当前 `rustc` 自带的 sysroot（注意：Rust 目标标准库的 sysroot、交叉 GCC 的 C 语言 sysroot 以及客体动态运行库资产三者是各自独立的输入）。`RUNNER_LINKER` 默认取 `$(MUSL_CROSS)gcc`，其中 `MUSL_CROSS` 在未显式指定时会根据 `ARCH` 自动选择 `MUSL_CROSS_RV` 或 `MUSL_CROSS_LA`。两个架构的编译 target 分别为 `riscv64gc-unknown-linux-musl` 与 `loongarch64-unknown-linux-musl`，编译生成的二进制文件位于 `build/contest-run/<target>/release/contest-run`。两个目标均固定启用 `+crt-static`、`relocation-model=static`、`link-self-contained=yes` 与 `-no-pie` 静态链接参数，release 配置采用 `panic=abort`，且在 LoongArch 目标上保持默认的 `LSX`/`LASX` 向量扩展设置（不额外关闭）。

执行 `make build-guest-userland ARCH=...` 会将编译好的执行器安装到 `GUEST_USERLAND_OUTPUT_DIR/sbin/contest-run`（不再安装旧版同名 Shell 脚本或编译旧版 C 语言辅助启动器）。BusyBox 与 Cargo 项目各自通过真实的源文件依赖判断是否需要增量重编；如果仅修改了 `userland/contest/` 下的 `.toml` 比赛配置文件，只需重新执行根文件系统装配（`assemble-root-filesystem`）、制作镜像（`create-ext4-image`）并在启动 QEMU 时显式刷新系统工作盘即可，无需重新编译 `contest-run`。相应地，`make build-kernel` 仅负责编译内核。

执行 `make all` 会同时编译双架构的 release 版 `contest-run`，并生成导出的四项核心交付物：`kernel-rv`、`kernel-la`、`disk.img` 与 `disk-la.img`（支持通过 `RUNNER_SYSROOT_RV`、`RUNNER_SYSROOT_LA`、`RUNNER_LINKER_RV`、`RUNNER_LINKER_LA` 分别为两个架构指定工具路径）。`assemble-root-filesystem` 会将完整的比赛配置目录复制到系统根文件系统的 `/etc/contest/` 下，其中默认执行计划为 `/etc/contest/plan.toml`；若需运行其他计划，必须通过 `contest-run` 的 `--plan` 参数显式指定，执行器不会仅因为目录中存在其他配置文件就自动将其纳入运行。

`make create-ext4-image ROOTFS_TREE_DIR=... SYSTEM_ROOT_EXT4_IMAGE=...` 仅以只读方式将指定的完整目录树打包为 ext4 镜像，不会在制盘阶段自动回填 `contest-run` 二进制或配置文件。使用 `make run-qemu` 启动时：`QEMU_ARGS='--working-disk-action fresh'` 会全新复制所选源镜像；`reuse` 会保留并复用全部现有工作盘；`refresh-system` 则在持有同一目录锁的前提下仅替换系统工作盘 `root.raw`，保持比赛工作盘 `contest.raw` 及源镜像不变。一旦制盘或工作盘刷新失败，流程会立即报错中止，不会静默回退去使用旧的系统工作盘继续启动。

`make clean-build-artifacts` 仅清理明确的编译链接中间产物与项目管理的安装子目录，不会删除工作盘副本、系统镜像、工具链、sysroot、运行日志或测试报告。`make clean-qemu-working-disks ARCH=... QEMU_ARGS='--working-disk-dir ...'` 则会尝试获取与 QEMU 启动相同的非阻塞目录锁，仅删除工作目录下的 `root.raw` 与 `contest.raw`（保留目录本身与 `.lock` 锁文件 inode）；若有正在运行的 QEMU 实例持有该锁，清理命令将直接报错拒绝执行。

## 板平台与 DEVICE

`contest-run` 是架构级通用用户态程序，仅根据 `ARCH` 编译为对应架构的 Linux-musl 静态 ELF，无需针对不同的 `TARGET_PLATFORM`（如 QEMU 与实体开发板）分别编译不同版本的执行器。其命令行接口统一为：

```text
/sbin/contest-run DEVICE [--plan FILE] [--file FILE ...] [--list]
/sbin/contest-run DEVICE --root ROOT --cwd CWD -- COMMAND [ARG...]
```

其中 `DEVICE` 参数支持传入系统中实际存在的块设备节点，包括 `/dev/ram0`、`/dev/mmcblk0`、`/dev/sda`、原有的 `/dev/vda` 以及它们的分区设备（例如 `ram0p1`、`mmcblk0p1`、`sda3`、`vda1`）。命令行解析层仅校验参数为合法 UTF-8 且是位于 `/dev` 下的直接设备路径；具体的设备名称、分区号合法性以及设备是否存在，由内核的 `mount` 系统调用与 `DeviceSelector` 统一校验——若指定的分区或设备不存在，会直接返回错误，绝不会擅自改挂整块磁盘、其他磁盘或回退到 RAM 盘。原有的 `--plan`/`--list` 模式、单次命令模式下的工作卷内相对 `--root`/`--cwd`、原样传递 `argv` 以及进程退出码规则均保持不变（注意 `--root /musl` 始终指代所挂载目标设备内的 `/musl` 目录，切勿写成父系统挂载点路径 `/mnt/contest/musl`）。

内核启动时的根设备（`root`）与附加块设备参数由 `main` 入口一次性解析后传递给存储层（Storage）与文件系统层（FS）：只有当开发板上确实未插入 SD 卡或未连接 SATA 硬盘时，才按物理介质缺席处理；未被选作根文件系统的空卡槽不会阻碍显式指定的 `/dev/ram0` 根启动，而检测到的非根可用块设备仍会正常注册到 `/dev` 供用户态后续 `mount` 使用。反之，若启动所需的根设备缺失，或遇到硬件资源、协议及停机阶段的错误，内核仍会按既定规则报错停止，不会私自改选 RAM 盘继续运行。

例如，在已经挂载并识别到对应块设备的客体系统中，执行 `/sbin/contest-run /dev/ram0 --plan /etc/contest/plan.toml --list` 即可在 `/dev/ram0` 上复用同一套测试计划配置（此处仅展示命令用法，不代表某一特定实体板上的所有块设备均已完成实板验收）。

制作实体开发板使用的根文件系统镜像时，先将包含同架构 `contest-run`、配置文件与 `/sbin/init` 的完整目录树通过 `make create-ext4-image ROOTFS_TREE_DIR=... SYSTEM_ROOT_EXT4_IMAGE=...` 打包为无日志（no-journal）的 raw `ext4` 镜像，再通过 `make package-board-boot-images ... BOARD_ROOT_EXT4_IMAGE=... BOARD_OS_DTB=... BOARD_ROOT_DEVICE=/dev/ram0` 生成离线启动包。磁盘镜像、内核 ELF 与启动打包目录三者职责独立；打包步骤不会自动触发重新制盘，不会修改源目录树，也不会直接写入物理存储介质。双架构实体板的完整构建与启动命令请参阅 [上板指南](../../docs/boards.md)。打包时传入的 `BOARD_OS_DTB` 必须是匹配目标开发板硬件的真实 OS 设备树。

内核具名测试仅通过 `tools/run_kernel_tests.py`（`list`/`run`）在双架构 QEMU 虚拟机中运行，不提供面向实体开发板的串口传输通道。开发板硬件接线、U-Boot 固件操作、存储介质准备以及四种根模式启动与持久化重启验证，均需在真实硬件上人工执行并保存完整的串口日志；在缺少实体硬件或真实输入时，不能用空脚本模拟替代。在解读串口停机日志时，只有观察到 `success/all-online` 才证明完成了全核软件停机流程；异常路径下的 `failure/current-cpu` 仅代表当前核中止，不代表其余 CPU 核或外设已安全停止；而串口超时（timeout）、连接断开（EOF/close）也不等同于开发板已完成物理断电。宿主机离线打包产物与 QEMU 仿真运行结果不能代替实体板验证结论。
