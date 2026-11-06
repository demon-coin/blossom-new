# 开发、测试与比赛执行

本指南描述当前项目的公开开发、测试与比赛运行接口（并非阶段验收报告）。内核支持 `riscv64` 与 `loongarch64` 双架构；用户态始终由 BusyBox `init` 作为 `PID 1` 运行，正式比赛执行器是独立的 Rust `std` 程序 `/sbin/contest-run`。内核具名测试（`ktest`）、独立用户自测（`user-selftests`）和比赛评测结果分别独立记录，编译通过或目录展开不能代替实际运行验证。

## 1. 环境与构建职责

所有宿主端构建与运行命令均在仓库根目录执行。开发机需准备固定版本的 Rust 工具链 `nightly-2025-01-18`、GNU Make、Python3、两架构 musl 交叉 GCC 工具链，以及 QEMU `9.2.1`。制作 ext4 磁盘镜像需要使用 `mke2fs`、`debugfs`、`e2fsck`、`dumpe2fs`；检查运行库需要使用支持目标架构 ELF 的 `readelf`。开发板设备树（DTB）从仓内源码编译生成，宿主需提供 C 预处理器（`cc`）和 `dtc`，打包 RAM 根文件系统镜像（`uInitrd`）还需提供 `fdtput`。上述工具通过 `PATH` 或显式 Make 参数提供；Makefile 不会调用系统包管理器，Nix 仅作为可选的个人开发环境。

Cargo 依赖统一使用仓内 `vendor` 目录。当通过 Rustup 代理调用且允许自动安装时，Rustup 可根据根目录 `rust-toolchain.toml` 联网安装缺失的固定工具链组件和目标标准库（Cargo 的离线配置不会限制 Rustup）。但在直接单独构建用户态 `std` 执行器时，构建脚本会通过 `RUSTUP_AUTO_INSTALL=0` 禁用 Rustup 自动安装，因此所需的目标标准库必须事先准备就绪。

普通内核构建与 `ktest` 测试内核构建都需要该固定工具链的 `rust-src` 组件，由 `os/.cargo/config.toml` 统一从源码重新编译 `core`、`alloc` 及 `compiler-builtins-mem`。LoongArch 架构统一关闭 `ual`（非对齐访存）特性，确保内核与标准库都生成严格对齐的访存指令；不能只对内核代码禁用非对齐访存却继续链接预编译的标准库。Makefile 会清除外部环境变量中的 `RUSTFLAGS` 与 `CARGO_ENCODED_RUSTFLAGS`，改用 Cargo 原生的 `--config` 数组合并 panic/unwind 策略与目标 ISA 约束。若直接运行 Cargo 构建内核，仍须以 `os/` 为工作目录；独立的用户态 `std` 程序则消费已准备好的目标标准库和 sysroot，不读取内核的 Cargo 配置。

单架构构建默认根据 `ARCH` 选择交叉工具链前缀 `MUSL_CROSS_RV` 或 `MUSL_CROSS_LA`，默认值分别为 `riscv64-linux-musl-` 和 `loongarch64-linux-musl-`；运行库路径参数 `MUSL_LIBC`、`GLIBC_LIBDIR` 同样默认选择对应的 `_RV` 或 `_LA` 变量。显式传入单架构参数（包括设为空值）优先于上述默认值；`make all` 则分别读取两套架构的专属参数。未显式指定运行库来源时，准备脚本会查找其文档中列出的固定默认容器路径，若文件缺失则立即报错退出。当本地工具链命名或安装路径与默认值不同时，应显式覆盖对应的前缀与运行库路径参数。

```sh
make help
make build-kernel ARCH=riscv64
make build-kernel ARCH=loongarch64 KERNEL_BUILD_PROFILE=release
make build-guest-userland ARCH=riscv64
make prepare-guest-runtime-libraries ARCH=riscv64
```

| 入口 | 职责说明 |
|---|---|
| `build-kernel` | 仅构建所选架构与平台的内核；默认为 `debug`、`abort` 配置，输出至 `build/kernel/ARCH/PLATFORM/PROFILE/kernel.elf` |
| `build-guest-userland` | 增量构建 BusyBox、仓库内 C 用户程序、release 模式的 Rust `std` 比赛执行器与用户自测执行器 |
| `prepare-guest-runtime-libraries` | 从显式指定的本地 libc 来源复制并校验匹配目标架构的动态运行库资产 |
| `assemble-root-filesystem` | 汇总用户程序、运行库和配置文件，全量更新项目管理的单根文件系统输出目录 |
| `create-ext4-image` | 仅将指定的完整目录树打包为不含日志（no-journal）的 raw ext4 磁盘镜像 |
| `build-qemu-artifacts` | 生成指定架构的固定 `release`/`abort` 内核以及单根系统 ext4 镜像 |
| `build-board-dtb` | 从仓内完整设备树源码编译所选实体开发板的 OS DTB，不读取个人 SDK 或旧构建目录 |
| `all` | 构建并导出双架构交付物：`kernel-rv`、`kernel-la`、`disk.img`、`disk-la.img` |

导出的两张磁盘镜像（`disk.img` 与 `disk-la.img`）各自仅包含对应架构的根目录 `/`，不再采用同一镜像内并存 `/riscv64` 与 `/loongarch64` 双子根的布局。普通内核 ELF 不启用 `ktest` 特性；显式执行的 `debug` 构建也不会覆盖固定的 `release` 交付产物。

用户态 Rust `std` 程序的编译目标为 `riscv64gc-unknown-linux-musl` 与 `loongarch64-unknown-linux-musl`。`RUNNER_SYSROOT` 用于指定包含 Rust 目标标准库的 sysroot 路径（未指定时默认使用当前 `rustc` 的 sysroot）。`RUNNER_LINKER` 默认为所选 `MUSL_CROSS` 拼接 `gcc`，也可独立指定交叉链接器路径；交叉链接器的 C sysroot、Rust 目标标准库的 sysroot 以及客体动态运行库资产三者职责独立，需分别准备。比赛执行器的直接依赖、`Cargo.lock` 及离线 `vendor/` 位于 `userland/contest-run` 目录；若直接运行 Cargo 命令，必须以 `userland/contest-run` 为工作目录，清除外部交叉编译 `RUSTFLAGS`，并传入 `--locked --offline`。

默认情况下，BusyBox、项目自带的 C 用户程序以及两个 Rust `std` 执行器（`contest-run` 与 `user-selftests`）均采用静态链接；而外部比赛测试盘中可能包含动态链接的 ELF 程序。动态程序在客体系统内运行时，必须能在其当前根目录视图下找到 ELF 程序头 `PT_INTERP` 指定的动态链接器（interpreter）及所需的共享库（可在宿主端使用 `readelf -l -d` 核对产物）。部分交叉编译器包装脚本可能会将宿主机的绝对路径写入解释器路径或运行库搜索路径（RPATH）；因此在自行构建动态程序时，应使用与客体根文件系统布局匹配的目标工具链和运行库，而对外部比赛输入则应保持原样运行，不得通过修改原比赛二进制文件来获取测试结果。

完整构建双架构交付产物的本地命令示例：

```sh
make all \
  MUSL_CROSS_RV=/srv/toolchains/riscv64/bin/riscv64-buildroot-linux-musl- \
  MUSL_CROSS_LA=/srv/toolchains/loongarch64/bin/loongarch64-linux-musl- \
  MUSL_LIBC_RV=/srv/contest-runtime/riscv64/musl/libc.so \
  GLIBC_LIBDIR_RV=/srv/contest-runtime/riscv64/glibc/lib \
  MUSL_LIBC_LA=/srv/contest-runtime/loongarch64/musl/libc.so \
  GLIBC_LIBDIR_LA=/srv/contest-runtime/loongarch64/glibc/lib \
  RUNNER_SYSROOT_RV=/srv/rust-std/nightly-2025-01-18 \
  RUNNER_SYSROOT_LA=/srv/rust-std/nightly-2025-01-18
```

使用时请将上述路径替换为开发机上的实际路径。关于运行库来源与动态链接器的详细约定请参阅 [sources.md](../userland/runtime/sources.md)。内核、用户程序、运行库与磁盘镜像属于各自独立的构建阶段，日常开发只需重建受代码改动影响的部分。

## 2. 单根树、卷与会话

```sh
make assemble-root-filesystem ARCH=riscv64 ROOTFS_OUTPUT_DIR=build/my-root-rv
make create-ext4-image ARCH=riscv64 \
  ROOTFS_TREE_DIR=build/my-root-rv \
  SYSTEM_ROOT_EXT4_IMAGE=build/my-root-rv.ext4
```

`assemble-root-filesystem` 会全量替换其管理的输出目录（`ROOTFS_OUTPUT_DIR`），因此从源配置中删除文件后，重新装配也会同步清理输出树中的旧文件；该步骤绝不会向外部输入目录回写或填充文件。`create-ext4-image` 仅负责将指定的目录树（`ROOTFS_TREE_DIR`）打包为 ext4 镜像，不会隐式触发代码编译、补齐缺失程序或重新执行装配。若直接将外部准备好的完整目录树传给 `ROOTFS_TREE_DIR`，该目录内必须已经包含对应架构所需的 `init`、可执行程序及依赖库。

ext4 镜像默认大小为 `256MiB`，可通过 `EXT4_SIZE_MIB` 传入正整数进行覆盖；像 `8`、`16`、`32` MiB 这类小容量镜像能否容纳输入目录，完全由实际的 `e2fsprogs` 工具链（`mke2fs`）根据实际文件大小决定，构建脚本不设人为的 `128MiB` 下限或固定 inode 数量限制。若制盘失败，脚本会保留原有的旧镜像文件并输出底层工具的诊断信息。装配、制盘与自测准备均允许父级工作区路径包含常规符号链接（symlink），但会拒绝最终输出路径本身是符号链接、输入输出目录重叠或文件互为别名的情况，确保目录树、磁盘镜像和日志文件各自拥有独立的物理路径。

系统启动后，统一的 `/etc/inittab` 会调用 `/sbin/start-session` 读取真实的 `/proc/cmdline`，根据内核命令行参数选择启动比赛执行器（runner）或进入交互式 Shell。在此过程中，BusyBox `init` 始终作为 `PID 1` 负责执行 `/etc/init.d/rcS` 初始化、回收孤儿进程以及停机前执行 `/etc/init.d/rcK`。如果读取 `/proc/cmdline` 失败或遇到未知的会话模式参数，`/sbin/start-session` 会直接报错退出，而不会擅自切换到其他会话模式。在交互式 Shell 模式下，退出 Shell 不会自动转去执行比赛评测；若需正常停机，请在 Shell 中依次执行 `sync` 和 `/bin/busybox poweroff`。

## 3. 唯一生产 QEMU 入口

QEMU 启动入口仅消费已构建好的内核与磁盘镜像，不会隐式触发代码编译或重新制盘：

```sh
make run-qemu ARCH=riscv64
make run-qemu ARCH=loongarch64 QEMU_ARGS='--shell'
make run-qemu ARCH=riscv64 QEMU_ARGS='--shell --system-only'
make run-qemu ARCH=riscv64 QEMU_ARGS='--debug'
python3 tools/run_qemu.py --arch loongarch64 --shell --debug --gdb-port 1235
```

`make run-qemu` 仅负责转发参数，所有默认配置与参数校验均由 `tools/run_qemu.py` 统一实现。默认配置下，脚本加载对应架构导出的 release 内核 ELF、系统根镜像和比赛测试盘，虚拟机配置为 `1` 核 CPU、`1 GiB` 内存、`user` 模式网络、`UTC` 时钟及 `-no-reboot`，双盘挂载顺序为比赛盘 `/dev/vda`、系统盘 `/dev/vdb`，不额外附加内核命令行参数。内核默认将 `/dev/vdb` 的 `/` 目录作为系统根文件系统，并以只读（`RO`）模式挂载。

| 选项 | 作用说明 |
|---|---|
| `--kernel-elf`、`--system-image`、`--contest-image` | 显式指定已存在的输入文件路径（仅作为只读输入来源，而非构建输出路径） |
| `--shell`、`--init-program` | 选择进入交互式 Shell 会话或指定自定义初始化入口程序，二者互斥 |
| `--system-only` | 仅挂载系统盘（此时系统盘在客体内成为 `/dev/vda`），通常与 `--shell` 或 `--init-program` 配合使用 |
| `--root-mode rw` | 显式允许以读写（`RW`）模式挂载系统根文件系统，不改变宿主机工作盘的复制策略 |
| `--debug`、`--gdb-port` | 对所选内核 ELF 启用 QEMU `-S` 启动暂停与 GDB 调试服务，不切换构建 profile、不重新编译替代 ELF，也不会因暂停等待调试器而触发超时 |
| `--cpus`、`--memory`、`--network`、`--host-forward` | 覆盖虚拟机的 CPU 核数、内存大小、网络后端模式或宿主端口转发规则 |
| `--print-command` | 仅打印最终组装的 QEMU 命令行，不创建目录、不加锁、不复制工作盘也不启动 QEMU |

使用 GDB 调试时，请选用匹配目标架构的调试器加载同一份内核 ELF 符号表，连接到指定端口后显式执行 `continue` 开始运行。需要注意，从 QEMU 控制台强制退出或由宿主机发送信号终止 QEMU 进程属于外部强制终止，不等同于客体操作系统的正常关机流程。

工作盘参数（`--working-disk-dir` 与 `--working-disk-action`）专门用于管理运行时与源镜像隔离的独立工作副本：

```sh
python3 tools/run_qemu.py --arch riscv64 --shell \
  --working-disk-dir build/manual-rv --working-disk-action fresh
python3 tools/run_qemu.py --arch riscv64 --shell \
  --working-disk-dir build/manual-rv --working-disk-action reuse
python3 tools/run_qemu.py --arch riscv64 --shell \
  --working-disk-dir build/manual-rv --working-disk-action refresh-system
```

三种工作盘策略的行为边界如下：
- `fresh`：将本次所选的系统源镜像和比赛源镜像全新复制到工作目录中。
- `reuse`：直接复用工作目录中已有的工作盘副本，不读取源镜像，也不会因为用户程序或配置文件重新编译而自动更新磁盘内容。
- `refresh-system`：仅用最新的系统源镜像替换系统工作盘（`root.raw`），同时完整保留比赛工作盘（`contest.raw`）中已有的数据修改。

若所选策略前置条件不满足（例如首次运行尚未生成工作盘时指定 `reuse`），脚本会直接报错退出，不会私自回退到其他策略。工作目录的非阻塞文件锁会一直持有到 QEMU 进程真正退出为止，确保源镜像文件绝不会被直接交给虚拟机写入。

```sh
make clean-build-artifacts
make clean-qemu-working-disks ARCH=riscv64 QEMU_ARGS='--working-disk-dir build/manual-rv'
```

`clean-build-artifacts` 仅清理可重建的编译中间产物，保留磁盘镜像、工作盘副本、sysroot、运行日志与测试报告；`clean-qemu-working-disks` 则在成功获取工作目录非阻塞锁后，仅删除该目录下明确属于工作盘的镜像文件（`root.raw` 与 `contest.raw`），保留目录本身、锁文件及其他无关文件；若检测到仍有运行中的 QEMU 实例持有该目录锁，则会拒绝清理并报错。

## 4. 按被测对象选择入口

| 被测对象 | 测试入口与验证边界 |
|---|---|
| 宿主机脚本参数、协议解析与文件操作 | `python3 -m unittest discover -s tools/ktest`；仅验证宿主机端工具逻辑，不涉及客体内核 ABI、进程调度或硬件驱动 |
| 内核内部数据结构、错误路径与并发交错 | `list-kernel-tests` / `run-kernel-tests`（仅在双架构 QEMU 上运行）；由测试夹具（Fixture）在停止后台线程、等待回收并释放对象后校验内核资源状态 |
| 标准 Linux 用户态行为与系统调用契约 | 独立的 `user-selftests` 执行器、默认 C 测试程序与标准 TAP 协议输出；无需挂载比赛盘，也不启用内核 `ktest` 特性 |
| 用户态初始栈布局、原始 errno、signal/clone 寄存器现场 | 由 `user-tests` 构建极简探测程序（raw probe），通过对应的内核具名测试夹具加载执行，同时在内核态校验内部状态 |
| 官方比赛测试盘与原始判题协议 | `/sbin/contest-run`，严格按照当前挂载的比赛盘内容、TOML 计划与对应判题协议执行并解析结果 |

### 内核具名测试

```sh
make list-kernel-tests ARCH=riscv64 CPUS=2 TEST='*final_address'
make run-kernel-tests ARCH=riscv64 CPUS=2 TEST='*final_address'
```

宿主机驱动脚本为 `tools/run_kernel_tests.py`。未显式传入 `--kernel` 参数时，脚本会调用内部目标 `build-kernel-test-elf` 构建测试内核；`KERNEL_BUILD_PROFILE` 与 `KTEST_PANIC_STRATEGY` 控制内核编译选项，而 `CPUS` 与 `MEMORY` 仅控制 QEMU 虚拟机资源。`TEST` 支持通配符（glob）过滤，`KTEST_ARGS` 可进一步传递精确用例名、分片（shard）等参数。建议先通过 `list-kernel-tests` 查看当前测试 ELF 中的完整用例名称列表：目录中标记为 `ignore` 的用例会展示跳过原因，执行测试时默认排除 `ignore` 用例；若命令行指定的任意精确用例名不存在，脚本会直接报错退出，不会将其静默缩减为部分成功。

超时参数 `--startup-timeout`、`--setup-timeout`、`--case-timeout`、`--cleanup-timeout`、`--build-timeout` 均以**秒**为单位（脚本内部变量均带有明确的 `_seconds` 后缀，而公开 CLI 选项名以及客体测试目录导出的 `timeout_ms` 协议字段保持不变）。

磁盘、UART 与网络等外部设备资源均按各测试用例声明的契约按需挂载；若宿主机缺少用例必需的输入文件，测试会直接报错失败，而不会当作普通 Skip 放行。常规测试的最终判定依据来自完整的 KTAP 协议输出及宿主机侧的外部观测校验；无法识别的杂乱日志不会生成通过结果。仅当某个用例输出了完整的中止/清理记录、且宿主机确认当前 QEMU 进程已彻底退出后，测试驱动才会为剩余尚未开始的用例启动新的 QEMU 实例继续执行；进度存疑的用例保持 `NotRun` 状态，不会在后台静默重跑。

### 独立用户自测

```sh
make build-user-selftests ARCH=riscv64
make install-user-selftests ARCH=riscv64
make run-user-selftests ARCH=riscv64
make run-user-selftests ARCH=loongarch64 USER_SELFTEST_ARGS='--collection fs'
```

`USER_SELFTEST_KERNEL_ELF` 用于指定已构建好的普通内核 ELF（`run-user-selftests` 目标本身不编译内核）。宿主机端轻量入口 `tools/run_user_selftests.py` 每次运行前都会准备包含普通 `PID 1` 配置的根文件系统，通过统一的生产 QEMU 入口、BusyBox `init` 及 `rcS`/`rcK` 脚本完成自测并正常关机，在真实等待 QEMU 进程退出后，直接从本次运行的系统工作盘中读取最终结果。根文件树、源镜像与工作目录可分别通过 `USER_SELFTEST_ROOTFS_DIR`、`USER_SELFTEST_EXT4_IMAGE`、`USER_SELFTEST_WORKING_DISK_DIR` 指定，串口日志（UART）保存在工作目录内。

单个自测程序的运行时间预算默认为 `60` 秒，可通过 `USER_SELFTEST_ARGS='--program-timeout-seconds 120'` 覆盖，实际生效的超时值会记录在程序测试报告中。整次宿主机运行的总时间预算默认为 `900` 秒，可通过 `USER_SELFTEST_HOST_ARGS='--host-timeout-seconds 1200'` 覆盖；镜像准备、QEMU 运行与结果读取共用同一个单调递增的绝对截止时间，串口持续输出日志不会延长该期限。QEMU 参数依然通过 `QEMU_ARGS` 传递；自动化自测运行仅允许 `fresh` 或 `refresh-system` 两种工作盘策略，若显式传入 `reuse` 会直接报错退出。普通 `run-qemu` 入口的三种工作盘策略、交互式 Shell 与 GDB 调试行为保持不变。

在客体系统的交互式 Shell 中，既可以直接运行单个测试 ELF，也可以通过 `/sbin/user-selftests` 调度同一份安装清单：

```sh
cd /test/selftests/fs
./file_shared
/sbin/user-selftests --list
/sbin/user-selftests --program fs:file_shared
```

默认自测程序、辅助 ELF（helper）及测试数据文件均在根目录 Makefile 中统一声明。安装阶段会生成 `/test/selftests/kselftest-list.txt` 清单文件（每行格式为 `collection:program`）；辅助程序不会计入默认通过率统计。`/sbin/user-selftests` 负责解析子进程输出的真实 TAP 协议内容与进程退出码，其自身退出码 `0`、`1`、`2`、`4` 分别代表全部通过、存在失败、基础设施错误、全部跳过（Skip），该执行器退出后不会自行调用关机。

自测程序的编译缓存目录 `USER_SELFTEST_BUILD_DIR` 默认位于 `build/user-selftest-cache/ARCH`，与用户态安装树 `GUEST_USERLAND_OUTPUT_DIR` 物理分离。每次执行安装时，都会根据当前 Makefile 的声明全新重建安装树下的 `test/selftests` 与 `test/scripts` 目录；因此从配置中移除某个自测程序或脚本后，重新执行安装、装配与制盘不会残留旧的缓存文件。扩展辅助程序（extended helpers）依然安装在 `/bin` 目录下，由 `extended-bin.txt` 记录其安装与清理范围（该文件并非测试执行清单）。切勿使用 `kselftest-list.txt` 去裁剪整个安装目录树，因为不在默认执行清单中的辅助程序、脚本和数据文件仍是测试运行所必需的依赖。

在自动化自测模式下，客体 `init` 脚本会在运行测试前删除旧的状态文件，待 `/sbin/user-selftests` 执行完毕后将 `run=<32位本轮标识> status=<0|1|2|4>` 完整写入 `/test/selftests/exit-status`，随后执行正常关机。宿主机脚本持有工作盘锁直至 QEMU 进程退出且磁盘结果读取完毕，并严格校验本轮生成的 32 位标识（token）：凡是状态文件缺失、格式错误、标识属于旧轮次、QEMU 退出码非零或宿主机读盘失败的情况，一律判定为运行失败（不能仅凭 QEMU 进程返回 `0` 或串口日志中出现成功字样就判定通过）。`run_user_selftests.py` 会原样透传客体的四种退出状态（`0`/`1`/`2`/`4`），若发生宿主机侧错误则返回 `2`（注意 GNU Make 在子命令返回非零时外层退出码固定为 `2`，原始的用户态退出码可在日志中查看）。

使用同一套 C 源码的宿主机 Linux 参考入口仅在开发机本地编译并直接运行测试程序，不启动内核、BusyBox 或 `/sbin/user-selftests` 执行器：

```sh
make build-linux-user-selftests
make run-linux-user-selftests
```

宿主机原生运行结果仅用于对照标准 Linux 行为，不能作为目标架构（RISC-V / LoongArch）内核的验证依据；其中合法的 UDP 大报文分片测试依赖宿主机网络栈配置支持，不得通过改小测试报文长度来迁就受限的宿主环境。

## 5. 比赛配置、进程与协议

```sh
/sbin/contest-run /dev/vda --list
/sbin/contest-run /dev/vda --plan /etc/contest/plan-onsite.toml --list
/sbin/contest-run /dev/vda --plan /etc/contest/plan.toml \
  --file sets/basic.toml --file sets/libctest.toml
```

比赛测试计划与集合配置位于 [userland/contest](../userland/contest/)。命令行接口格式为 `DEVICE [--plan FILE] [--file FILE ...] [--list]`；传入 `--list` 时仅挂载目标设备并解析、校验本次配置与磁盘文件列表，不执行环境准备（prepare/setup）、内核 Hook、测试子进程或系统关机。默认计划只会加载配置文件中显式引用的集合，不会因为目录中存在其他 `.toml` 文件而自动将其纳入运行。

配置文件中的相对路径（`files`）均相对于当前入口文件解析，解析与合并顺序依次为：入口 `step`、`files`、`roots`、`include`。`exclude` 过滤规则仅作用于当前 `step`；若精确名称或通配模式未匹配到任何用例、最终选择集为空或存在定义冲突，执行器会直接报错退出。`override` 仅用于修改已选中用例的参数（如超时或重复次数），不会重新引入已被排除的用例。`exec` 模式严格保持原始 `argv` 数组直接调用 `execve`，仅在 `shell` 模式下才会调用显式指定的 Shell 解释器。工作目录（`cwd`）、绝对路径符号链接、动态链接器（`PT_INTERP`）及共享库（DSO）均在目标子根目录（chroot）内实际解析；若目标根缺少动态链接器，执行器会如实报告 `exec` 错误，而不会通过修改环境变量或借用其他 libc 族的库来掩盖问题。

每一轮测试严格按照 `setup` → `before` → 启动被测子进程并收集输出 → 对应的 `after` → `cleanup` 顺序执行；被测进程的 `wait` 状态、`exec` 错误码（errno）、超时事件以及 Hook/清理阶段的错误均分别独立记录。每个用例在启动前都会建立一次基于单调时钟（monotonic clock）的绝对截止时间，串口或标准输出持续产生日志不会延长该期限；到达截止时间或测试结束时，执行器会对所属范围内的进程执行真实的信号终止（`kill`）与回收等待（`wait`）。

在输出协议处理上，`Plain` 与 `LTP` 模式直接让子进程继承标准输出/错误文件描述符；需要解析输出正文的 `Basic`、固定 `Pre2025` 与 `Tap` 模式则通过有界缓冲区捕获并转发输出。各判题协议严格遵循其原始判定规则，标准输出中夹带的普通 `PASS`/`FAIL` 文本不会覆盖协议解析结果。配置层面的显式跳过（Skip）、动态链接器缺失与进程启动后运行失败属于三种截然不同的状态，绝不混为一谈。

内核专属测试辅助（Hook）仅在确有必要时通过 [contest_hooks.rs](../os/src/contest_hooks.rs) 与配置项 `hooks=true` 启用，不为普通测试用例添加空操作分支。Hook 逻辑运行在控制者进程的普通可睡眠任务上下文中（而非目标进程的缺页异常或中断上下文），因此全局状态观测结果中可能包含系统后台任务的活动。关于运行库准备与源码模块结构的详细说明请参阅 [runtime 开发导读](../userland/runtime/development.md)。

## 6. 四平台布局与人工上板

内核的编译 target 为 `riscv64gc-unknown-none-elf` 或 `loongarch64-unknown-none`；平台参数 `TARGET_PLATFORM` 默认为 `qemu-virt`，实体板则对应选择匹配架构的 `visionfive2`（RISC-V）或 `ls2k1000la`（LoongArch）。在配置代码编辑器（如 rust-analyzer）时，应针对所选裸机 target 开启 build scripts 与 proc-macro 支持，并向构建脚本和静态分析传入相同的 `BLOSSOM_PLATFORM` 环境变量，切勿使用宿主机 target 分析 `no_std` 内核代码。

`os/src/platform/layout.rs` 是四个平台内存与设备地址布局的唯一纯 Rust 定义来源，`os/build.rs` 仅读取该文件在 `OUT_DIR` 下生成链接脚本输入；离线打包脚本则直接解析最终生成的 `kernel.elf` 程序头与符号表，无需在外部重复维护装载地址常量。在编写驱动与底层代码时，必须严格区分源码常量定义、MMIO 物理地址、CPU 虚拟地址以及设备侧 DMA 地址，四者不可混用。

`os/src/arch/` 下的 `entry.rs`、`trap.rs` 和 `user/entry.rs` 采用具名 `#[naked]` 裸函数定义机器级入口，通过 `sym` 与 `const` 操作数直接关联 Rust 处理函数与上下文结构体偏移。引导核（BSP）的 `.bss` 段清零以及 RISC-V 分页开启前的引导页表（bootstrap page table）填充均使用普通 Rust 控制流编写；汇编代码仅保留整字内存存储、通用与控制寄存器保存恢复、栈指针切换及特权级返回指令。固定字节序列拷贝与 LoongArch 精确的 `idle` 唤醒窗口则保留独立的最小 `global_asm!` 块。

机器级入口在建立可信内核栈并完整保存寄存器现场之前，绝对不能调用普通 Rust 函数。RISC-V 的分页前初始化函数按高地址虚拟内存（VMA）链接，由裸入口通过链接脚本提供的物理地址别名调用；因此这部分代码必须完全自包含且位置无关（PIC），不得依赖全局指针（`gp`）、线程局部存储（TLS）、高地址静态数据或普通辅助函数（从而在保留正式测试栈展开记录的同时，无需关闭 unwind 或为测试维护第二套启动代码）。LoongArch 的三个异常与 TLB 重填入口向量分别通过专用链接段和链接器 `ASSERT` 保证 `4KiB` 对齐，且 `idle` 块严格限制在 `32` 字节窗口内。任何涉及底层入口或链接布局的修改，都必须检查最终 ELF 的符号地址、反汇编指令、跳转目标与段布局，并实际运行验证系统启动、中断处理、上下文切换及返回用户态全流程；仅凭编译通过不能证明底层协议正确。

实体开发板的内核构建、固件容器打包、RAM 根/持久磁盘根启动及普通用户程序验证步骤，请查阅 [上板与根卷指南](boards.md)。

实体板构建与离线镜像打包不依赖 `ktest`（`ktest` 框架也不支持实体板传输协议）。仓内提供的设备树源码仅对应文档记录的特定板型与固件版本，实际可用内存装载范围、磁盘分区表及串口参数必须结合具体硬件设备核实；实体板启动、DMA/IRQ/PHY 驱动运行及同一存储介质的持久化重启验证，均以实际硬件运行日志为准。请完整保存运行日志、输入镜像哈希与真实的错误现象：QEMU 仿真通过或离线打包成功不能代替实体板验证结论；LoongArch 的不可执行页保护（NX）目前仍属暂缓项（不计为 Pass）。此外，本项目当前不扩展 ext4 日志恢复（journal/recovery）、恶意损坏磁盘防护或额外的测试管理框架。

## 7. 语义常量与等价修改

硬件寄存器偏移、控制位域、系统调用 ABI 命令号、内存布局参数与物理单位应定义为表意清晰的具名常量，并优先复用所属模块已有的定义；衍生大小与位掩码应由基础常量通过常量表达式推导得出。需要注意，数值相等并不代表语义相同——例如文件系统块大小即便恰好为 4096 字节，也不应直接借用页表模块的 `PAGE_SIZE` 常量。对于语义直观的 `0`/`1`、局部数组下标、局部汇编数字标签以及独立测试用例中的字面预期值，应保持直观可读，无需机械替换为具名常量。

在进行常量提取或等价代码重构时，必须严格保持原有的整数类型、有符号/无符号语义、溢出检查行为、寄存器保存现场、错误码返回值及并发同步顺序。内联汇编操作数与链接脚本各自受特定的编译期求值规则约束；修改后必须核对最终 ELF 的反汇编指令、符号地址、对齐方式和段装载权限，并实际运行受影响的内核测试与普通用户态程序，不能仅凭 `cargo fmt` 或 `cargo clippy` 通过就认定修改等价。此外，[Linux coding-style §12](https://docs.kernel.org/process/coding-style.html#macros-enums-and-rtl) 针对的是 C 语言的宏与枚举；本项目中 Rust 的 `const`、强类型封装与模块划分均根据内核自身架构职责设计。

## 8. Clippy 诊断、局部例外与严格检查

执行静态检查时，需按目标架构（target）分别检查普通内核以及 `ktest-dev`、`ktest-release` 测试配置；当修改涉及 panic 策略边界时，还需一并检查 `ktest-abort-dev` 与 `ktest-abort-release`。同一份源码在不同架构（ISA）、目标平台（platform）和编译特性（feature）下的调用方并不相同：某单一配置下报告的 `unused_imports` 或 `dead_code` 并不能证明该定义在全项目中无用。同时，`os/src/platform/layout.rs` 等共享布局模块还会被宿主机运行的 `build.rs` 直接引用，因此切勿根据宿主机架构裁剪目标平台的常量或类型定义。

在进行 Clippy 清理时，优先修复冗余借用、相同类型的显式转换、明确保持 Rust 2018 借用语义的数组迭代，以及可证明求值顺序完全一致的表达式简化。对于具有明确局部职责的复杂组合类型，可使用 `type` 类型别名提升可读性，无需为此引入额外的包装结构体或抽象层。凡涉及 `?` 运算符改写、`Option`/`Result` 组合子、`Vec::append`、比较分支重构以及 `unsafe` 借用调整，都必须仔细核对锁守卫（Lock Guard）、临时对象、最后一个强引用（`Arc`）的持有范围与析构（Drop）顺序，避免意外延长或缩短锁临界区。条件编译属性（`#[cfg(...)]`）应严格对齐真实的测试或架构调用方；只有确认在所有编译组合下均无调用方的内部死代码才应删除。

绝不能为了消除 Clippy 警告而擅自改动硬件寄存器或用户态 ABI、受外部协议约束的内存布局、原子操作内存序（Ordering）、系统调用错误码、具名测试用例名称或测试断言。若内部结构体存在完全无读取方、无裸指针访问且无内存布局依赖的冗余字段，可将该字段及其无效写入一并移除；若某类建议（如将大对象改为 `Vec<Box<T>>`、拆分大枚举或大错误类型、缩减函数参数个数）会改变对象的稳定内存地址、引入新的堆分配失败点、破坏无堆（no-heap）启动环境或打乱并发同步职责，则应保留原设计并添加带明确技术理由的局部抑制属性，切勿通过添加空调用或无业务意义的包装层来规避告警。删除手写的测试符号锚点（anchor）时，必须逐一核对宏生成的描述符槽位、链接器保留段（`KEEP`）、最终 ELF 符号表、测试目录元数据以及实际的用例筛选行为。若修复确实涉及诊断输出格式的合理变更，应作为有意修改单独记录，并在真实输入下对比修改前后的输出内容，避免为拼接格式化文本而额外引入临时堆内存分配。

### 合理例外与失败策略

`os/Cargo.toml` 中配置的 `[lints.rust] warnings = "deny"` 会自动约束内核主包及 `build.rs`（适用于所有 Cargo 与 Make 入口），同时允许第三方依赖包保持其原有 lint 策略（构建系统不会传递全局 `RUSTFLAGS=-Dwarnings` 给外部依赖）。因此，内核包中任何新引入的 Rust/Clippy 警告以及未命中的 `#[expect(...)]`（即 `unfulfilled_lint_expectations`）都会直接导致编译检查失败。需要注意，编译器前端的非 lint 警告不受 `[lints.rust]` 表控制：例如当前 LoongArch 目标下由 `-Ctarget-feature=-ual` 触发的编译器特性警告，应与内核源码自身的 lint 状态及第三方依赖警告区分记录，既不能将其掩盖并宣称全编译过程零警告，也不能为了消除该提示而改动 ABI 对齐参数。

对于稳定存在且有充分技术依据的 lint 例外，应在具体的结构体字段、枚举变体、方法或函数上使用精准的 `#[expect(具体lint名称, reason = "具体技术约束说明")]`。若某段代码仅在 `ktest` 下被调用，可使用简单的 `#[cfg_attr(not(feature = "ktest"), expect(...))]`；若由于宏展开、宿主机 `build.rs` 与目标机共享代码或跨架构条件编译导致 lint 触发状态随编译组合变化，则使用同样附带 `reason` 说明的局部 `#[allow(具体lint名称, reason = "...")]`。严禁对整个模块笼统添加 `#[allow(dead_code)]`，也严禁使用 `clippy::all`、`unused` 等宽泛分组或编写无意义的假调用来掩盖问题。

需要注意，对某个函数标记 `dead_code` 例外后，编译器会将该函数视为可达根节点，进而自动标记其调用链上的所有下游函数为已使用；因此只需在最上游的入口函数处声明一次 `#[expect(dead_code, reason = "...")]`，不要在上下游函数上重复堆叠。对于通过 `Debug` 格式化输出提供实际诊断价值的字段可以保留，但仅挂载 `#[derive(Debug)]` 而从未被格式化输出的结构体不足以作为保留无用字段的理由。标准 ABI 常量表应清晰反映哪些命令或标志位当前尚未实现，而内部无实际消费者的废弃缓存则应直接清理，不要以“未来可能会用到”为由保留死代码。

普通 RISC-V 内核的 Clippy 检查命令示例（检查 LoongArch 时将 target 替换为 `loongarch64-unknown-none`，检查实体板时将 `BLOSSOM_PLATFORM` 替换为对应的正式平台名）：

```sh
cd os
BLOSSOM_PLATFORM=qemu-virt cargo clippy --locked --target riscv64gc-unknown-none-elf --profile dev
```

检查内核测试配置时，使用 `--profile ktest-dev`（或 `ktest-release`、`ktest-abort-dev`、`ktest-abort-release`）并加上 `--features ktest`（仅支持 `qemu-virt` 平台）；对应的实际可执行测试镜像仍需通过 `build-kernel-test-elf` 链接生成，不能用 `cargo clippy` 代替真实构建与运行测试。每次代码修改后，均应覆盖受影响的架构、平台、profile 与 feature 组合验证 lint 预期。相关规范参考自 [Linux Lints](https://www.kernel.org/doc/html/latest/rust/coding-guidelines.html#lints)、[Rust 诊断属性](https://doc.rust-lang.org/reference/attributes/diagnostics.html)、[Clippy 分类](https://doc.rust-lang.org/clippy/lints.html) 与 [Asterinas 属性规范](https://asterinas.github.io/book/to-contribute/coding-guidelines/for-maintainability/rust-specific/attributes-and-macros.html)。

## 9. 逻辑段落与纯空行整理

在源码排版上，函数与函数之间、函数内部的不同逻辑阶段之间、以及不同类别的常量定义组之间通常使用**一个空行**分隔；而属于同一逻辑阶段内的连续变量准备或参数校验则保持紧凑排列。例如，“分配与准备资源”、“获取自旋锁后二次检查并更新状态”、“释放锁后清理旧资源”属于三个清晰的逻辑阶段，应当用空行分段；但切忌按每一个 `if` 语句、每一个 `const` 定义、每一个闭合大括号 `}` 或固定行数机械地插入空行。完整的 `if/else` 链、`match` 分支以及紧密耦合的关中断（IRQ）、内存屏障（barrier）与 `unsafe` 操作序列应保持连续；`// SAFETY:` 注释必须紧贴其修饰的 `unsafe` 块，文档注释（`///`）与属性标记（`#[...]`）必须紧贴其修饰的代码项。

[Linux Rust 编码规范](https://www.kernel.org/doc/html/latest/rust/coding-guidelines.html#style-formatting) 要求使用默认配置的 `rustfmt`；[Rust 官方风格指南](https://doc.rust-lang.org/style-guide/#blank-lines) 允许在语句与代码项之间保留零个或一个空行；[Asterinas 逻辑段落规范](https://asterinas.github.io/book/to-contribute/coding-guidelines/for-maintainability/layout.html#logical-paragraphs) 则进一步阐述了按操作子步骤划分代码段落的可读性原则（Linux C 编码规范中的空行建议仅供背景参考，不作为 Rust 代码的硬性规定）。项目无需修改 `blank_lines_lower_bound` 等非标格式化参数，自动化格式工具也不能代替开发者对代码逻辑阶段的判断。

执行纯空行整理时，必须完整保留所有非空代码行及其先后顺序，同时原样保留所有注释、普通字符串/原始字符串（raw string）及内联汇编字符串字面量——多行字符串字面量内部的空行属于程序数据负载的一部分，绝不能用“删除全部空白字符后文本相等”这种粗糙方式来检验代码等价性。对于过长的函数，应先按照真实的控制流阶段划分逻辑段落，不得借排版之名顺带拆分函数、修改标识符命名、重排语句顺序或篡改注释内容。

在移动代码行号前，需先排查 `line!()`、`Location::caller()`、`#[track_caller]` 以及任何直接读取源码文件行号的消费者。在确认不影响测试契约的前提下，编译器诊断行号、DWARF 调试信息及测试目录元数据中的 `line` 字段可以随空行调整自然更新，但完整的测试用例名称、参数、`source` 路径、稳定属性与断言预期必须保持不变；同时需严格区分参与编译的 `include!` 宏与作为静态数据读取的 `include_str!`/`include_bytes!` 宏。构建产物应如实记录当次编译生成的 SHA256 摘要，无需承诺空行调整前后 ELF 二进制字节完全一致。

代码格式检查命令（仅执行只读检查，不直接覆写源码）：

```sh
cd os
cargo fmt --all -- --check
```

对于第三方依赖或历史遗留的非空行格式差异，保留检查日志即可，不要在纯空行整理任务中顺带修改上游依赖。静态检查同样沿用相同的工具链、平台、profile 与 feature 组合对比修改前后的 Clippy 诊断，不新增 `allow`/`expect` 属性或变更 lint 级别配置。完成静态等价核对后，仍需实际编译并运行受影响的关键内核路径与用户态测试。

## 10. 代码注释与文档编写规范

为了让操作系统初学者与后续维护者能够在 10 分钟内顺畅理解各模块的职责、并发不变量与硬件边界，全项目源码注释与技术文档遵循以下统一规范：

### 10.1 自然准确的语言表达（严禁电报体与防卫性黑话）
- **使用主谓宾完整的现代汉语**：注释应当用自然、通顺、直观的现代汉语（配合准确的英文/Linux 标准术语，如 Page/Folio、Inode、Dentry、SpinLock、Workqueue）直接陈述技术事实，避免把句子压缩成只有两三个字的“电报体”。
- **禁用自造词与黑话**：
  - 描述 DMA 缓冲区交接时，写明**“收回或移交物理页的 DMA 访问权（从设备交还 CPU，或从 CPU 移交设备）”**，不使用“原页收权/交权”等自造缩略语；
  - 描述异步与中断生命周期时，写明**“异步回调、中断处理函数或 Work 任务彻底执行完毕并退出”**，不使用“真尾/全尾/票尾/尾后”；
  - 描述引用计数析构时，写明**“最后一个强引用（`Arc`）被释放（Drop）时”**，不使用“末拥有/末 put/末释放”；
  - 描述锁临界区时，写明**“自旋锁（`SpinLock`）临界区（持锁时间极短、不可睡眠）”**，不使用“短锁/窄锁”；
  - 描述部分传输进展时，写明**“遇到错误前已成功传输的字节数（Partial Progress）”**，不使用“正前缀”。
- **消除防卫性自证废话**：严禁在注释中堆砌“不制造 / 不伪造 / 不冒充 / 不猜 / 不建链 / 不回退”等面向判题机的防卫性否定句。应当直接从正面解释**代码实际执行了什么操作、背后的操作系统原理是什么，以及如果违反该约束会导致什么具体后果（如死锁、释放后使用 UAF、DMA 越界踩内存或页表别名冲突）**。
- **杜绝内部编号与口水话**：代码注释中不出现内部开发阶段编号（如“阶段四”、“Stage 7”、“D8”）或问答式 AI 套话（如“这里需要注意”、“我们可以看到”、“简单来说”），可用“参考经典的 `errseq_t` 位图算法”、“采用单飞（Singleflight）合并加载模式”等标准技术表述说明设计背景。

### 10.2 文件顶部模块综述（`//!`）
每个源码文件顶部必须包含结构清晰的 `//!` 模块级文档注释，涵盖以下三个核心维度：
1. **模块职责与架构定位**：说明本模块在内核或用户态工具中承担的职责，明确其上游调用者（如系统调用层、VFS、中断入口）与下游依赖（如块设备驱动、页框分配器）。
2. **核心状态机与内存/位域布局**：若模块包含关键状态机（如 `Bio` 父子请求拆分、队列生命周期 `QueuePhase`、`Inode` 状态转换）、内存/栈帧布局或硬件位域编码（如 32 位回写错误状态字 `WritebackError`、CSR/PTE 位格式），**必须使用清晰的 Markdown 列表或 ASCII 示意图画出来**。
3. **锁获取顺序（Lock Ordering）与并发契约**：若模块涉及多把锁的协作（如睡眠锁与自旋锁、父目录锁与子目录项锁、局部索引锁与全局 LRU 锁），必须在文件顶部明确列出加锁层级顺序及持锁期间的禁止事项（如持自旋锁严禁睡眠或触发缺页）。

### 10.3 结构体、枚举与全部字段/变体注释（`///`）
- **全覆盖无死角**：文件中的每一个 `struct`、`enum` 本体，以及**每一个结构体字段（field）和枚举变体（variant）**都必须配备清晰的 `///` 文档注释，彻底消灭无注释的“裸奔字段”。
- **明确单位与保护关系**：字段注释需指明其具体业务含义、物理或逻辑单位（如字节、512 字节扇区、4KiB 物理页、毫秒/纳秒）、由哪把锁负责并发保护，或者遵循怎样的生命周期与所有权不变量。

### 10.4 函数文档（`///`）、逻辑阶段注释（`//`）与 `// SAFETY:` 规范
- **函数级文档注释**：每个函数（无论 `pub` 还是私有辅助函数）都必须有通顺的 `///` 文档注释，说明函数的功能、调用上下文要求（普通可睡眠任务上下文、关中断上下文，或调用前必须持有哪些锁），以及关键参数和返回值的语义。
- **`unsafe fn` 契约**：所有 `unsafe fn` 的文档注释必须包含标准的 `# Safety` 章节，分点列出调用方在调用该函数前必须满足的具体前提条件。
- **函数内逻辑阶段注释**：对于较长的函数或复杂算法（如 FDT 设备树扫描、多级页表遍历、块 I/O 拆分、ext4 目录/Extent 查找、网络协议状态机、系统调用主路径），应在函数体内部的各个逻辑段落前添加 `// Step 1: ...` 等阶段说明注释，解释为什么必须按照该先后顺序执行，而不必逐行翻译显而易见的 Rust 语法。
- **具体可查的 `// SAFETY:` 注释**：每个 `unsafe` 代码块前的 `// SAFETY:` 注释严禁像背诵免责声明一样堆砌抽象词汇（如“方向、容量、初始化、保活均满足”）。必须给出具体依据（超过一句话时优先采用 `1. ... 2. ...` 分点说明）：明确指出**具体是哪一行前置检查（如 `assert!`、切片长度/对齐边界判断）、哪把已持有的锁（如 `state` 自旋锁），或哪一条调用方 `# Safety` 契约**保证了当前裸指针有效、满足对齐要求、生命周期内底层物理页/内存对象不被释放，且不存在并发可变别名冲突。

### 10.5 测试代码（`ktest`）专属注释准则
- **夹具与辅助类型**：测试文件中的每个辅助结构体、测试夹具（Fixture）及其全部字段均须补充清晰的 `///` 文档注释，说明其模拟的硬件环境或并发同步角色。
- **结构化测试规格说明**：每个测试用例函数（`#[case]` 或测试入口）的 `///` 文档注释应改写为直观的测试规格说明（推荐采用 **测试目标 / 场景步骤（Given-When-Then） / 预期结果** 结构），让读者一眼看懂该测试旨在构造怎样的并发交错、边界输入或硬件故障注入，以及预期的内核状态与返回值。
