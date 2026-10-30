# 比赛 runner 构建

本目录是独立的 Rust std 包 `blossom-contest-run`，库名 `contest_run`，程序名 `contest-run`。内核仍使用自己的裸机目标；这里没有默认 Linux target，也不使用内核的 `MODE`、`PANIC` 或 `build-std`。

直接依赖固定为 serde 1.0.217、toml 0.8.19、libc 0.2.169、glob 0.3.2。`Cargo.lock` 和独立 `vendor/` 必须由 Cargo 正式生成并随源码提供；`.cargo/config.toml` 只读取这份 vendor，构建不下载依赖、安装工具链或组件。工具链固定为已准备的 `nightly-2025-01-18`；缺少工具链、官方目标标准库或交叉链接器会直接失败，不使用旧程序替代。

与内核相同，Make 直接使用 `PATH` 中已准备的 Cargo/Rust；根 `rust-toolchain.toml` 记录固定版本。Make 不传 `+toolchain`，不要求 rustup 代理，不自动切换编译器；使用 rustup 代理时仍以 `RUSTUP_AUTO_INSTALL=0` 禁止自动安装。个人 Nix 环境可以提供这些工具，但不是构建依赖。

从仓库根目录执行：

```sh
make build-contest-runner ARCH=riscv64 RUNNER_SYSROOT=/absolute/prepared/sysroot RUNNER_LINKER=riscv64-linux-musl-gcc
make build-contest-runner ARCH=loongarch64 RUNNER_SYSROOT=/absolute/prepared/sysroot RUNNER_LINKER=loongarch64-linux-musl-gcc
```

不指定 `RUNNER_SYSROOT` 时使用当前 `rustc` 的 sysroot；它与交叉 GCC 的 C sysroot 和动态运行库资产是不同输入。`RUNNER_LINKER` 默认是 `$(MUSL_CROSS)gcc`，而 `MUSL_CROSS` 未指定时按 `ARCH` 选择 `MUSL_CROSS_RV` 或 `MUSL_CROSS_LA`。目标分别为 `riscv64gc-unknown-linux-musl`、`loongarch64-unknown-linux-musl`；程序位于 `build/contest-run/<target>/release/contest-run`。两个目标都固定使用 `+crt-static`、`relocation-model=static`、`link-self-contained=yes`、`-no-pie`，release 使用 `panic=abort`，不关闭 LoongArch 的 LSX/LASX。

`make build-guest-userland ARCH=...` 安装当前程序到 `GUEST_USERLAND_OUTPUT_DIR/sbin/contest-run`，不安装同名旧 Shell 或构建旧 C helper。BusyBox 与 Cargo 分别通过真实依赖判定是否重新编译；修改 `userland/contest/` 后需要重新装配、制卷并明确刷新系统工作盘，不必重编程序。`make build-kernel` 只编译内核。

`make all` 使用两架构 release runner 并导出 `kernel-rv`、`kernel-la`、`disk.img`、`disk-la.img`；可分别指定 `RUNNER_SYSROOT_RV`、`RUNNER_SYSROOT_LA`、`RUNNER_LINKER_RV`、`RUNNER_LINKER_LA`。`assemble-root-filesystem` 将完整配置树复制到系统根 `/etc/contest/`，默认计划为 `/etc/contest/plan.toml`；显式计划由 runner 的 `--plan` 选择，不从目录存在推导执行集合。

`make create-ext4-image ROOTFS_TREE_DIR=... SYSTEM_ROOT_EXT4_IMAGE=...` 只读完整输入，不自动回填 runner 或配置。普通启动由 `make run-qemu QEMU_ARGS='--working-disk-action fresh'` 复制所选盘；`reuse` 保全部工作盘，`refresh-system` 在原锁下只替换 `root.raw`，不修改 `contest.raw` 或原输入。制盘或刷新失败直接终止，不改用旧系统盘继续运行。

`make clean-build-artifacts` 只清明确编译链接产物和自有安装子树，不删工作盘、系统镜像、工具链、sysroot、日志或报告。`make clean-qemu-working-disks ARCH=... QEMU_ARGS='--working-disk-dir ...'` 取得启动时同一非阻塞锁，只删除 `root.raw`、`contest.raw`；目录和 `.lock` inode 保留，运行中的 QEMU 持锁时清理失败。

## 板平台与 DEVICE

runner只按 `ARCH` 选择同架构Linux-musl用户ELF，不因 `TARGET_PLATFORM` 建第二执行器。正式命令保持：

```text
/sbin/contest-run DEVICE [--plan FILE] [--file FILE ...] [--list]
/sbin/contest-run DEVICE --root ROOT --cwd CWD -- COMMAND [ARG...]
```

DEVICE现在可用实际存在的 `/dev/ram0`、`/dev/mmcblk0`、`/dev/sda`、原 `/dev/vda` 及分区（例如ram0p1、mmcblk0p1、sda3、vda1）。CLI只要求UTF-8和 `/dev` 下直接路径；精确名字/分区号及存在性由内核mount/DeviceSelector检验，缺失不切整盘、其它盘或RAM。原plan/list、显式工作卷内root/cwd、字面argv及退出结果不变，`--root /musl` 不能改传父系统的 `/mnt/contest/musl`。

内核启动的root/extra由main一次解析后传Storage及FS；只有真实无SD卡/SATA介质才视为absence。未选无介质不阻明确RAM根，实际可用的未选盘仍登记供后续mount；所需缺盘与其它资源/协议/停止错误仍失败，不改选RAM。该合同的源码已随四项初审纠正写入，修后Native门禁和连续五域复核尚待，不代表板启动或四项已验收。

例如在已经挂入实际设备的guest内，`/sbin/contest-run /dev/ram0 --plan /etc/contest/plan.toml --list` 复用同一计划。这里只列用法，不证明该设备已在实板可用；已有双目标runner重建与RV RAM/分区运行归各自第一波冻结输入，不代Native合流后的构建/Clippy/运行或实板验收。

板用完整自有系统树经 `make create-ext4-image ROOTFS_TREE_DIR=... SYSTEM_ROOT_EXT4_IMAGE=...` 制作无journal raw ext4，再经 `make package-board-boot-images ... BOARD_ROOT_EXT4_IMAGE=... BOARD_OS_DTB=... BOARD_ROOT_DEVICE=/dev/ram0` 离线包装。输入树须已有同架构runner、配置和init。卷、内核 ELF 与包装目录角色独立；包装不自动制盘、不修改源树或写物理介质，完整双板命令见[上板指南](../../docs/boards.md)。OS DTB必须由用户提供真实OS树，不能拿参考树冒充本板现场。

内核具名测试只走 `tools/run_kernel_tests.py list/run` 的双 QEMU，不提供板串口transport。板连接、固件、介质和四根/持久重启须人工执行并保实际日志；没有硬件或输入不能用空脚本代替。串口 `success/all-online` 只证明其软件停止scope，fatal `failure/current-cpu` 不证明其它CPU/设备停止，timeout/EOF/close也不证明物理断电。离线产物和QEMU结果不签实板通过。
