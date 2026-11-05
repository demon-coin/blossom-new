# 开发、测试与比赛执行

本指南描述当前公开接口，不是验收报告。内核支持 riscv64、loongarch64；BusyBox init 保持 PID1，正式比赛执行器是独立 Rust std 程序 `/sbin/contest-run`。内核测试、普通用户自测和比赛结果分别记录，编译或目录展开不能代替运行。

## 1. 环境与构建职责

宿主命令在仓库根执行。准备固定 `nightly-2025-01-18`、GNU Make、Python3、两架构 musl 交叉 GCC，以及需要的 QEMU 9.2.1。制盘使用 `mke2fs`、`debugfs`、`e2fsck`、`dumpe2fs`；运行库检查使用支持目标 ELF 的 `readelf`。板卡设备树从仓内源码生成，宿主需提供 C 预处理器和 `dtc`，RAM 根包装还需 `fdtput`。工具由 PATH 或明确参数提供；Make 不调用系统包管理器，Nix 仅是个人环境。

Cargo 依赖使用仓内 vendor。使用 Rustup 代理且允许自动安装时，Rustup 可按根 `rust-toolchain.toml` 联网准备缺失的固定工具链组件和目标标准库；Cargo 离线配置不限制 Rustup。直接单独构建 std 执行器时仍禁用 Rustup 自动安装，所需目标须事先准备。

普通内核与 ktest 都需要该固定工具链的 `rust-src`，由 `os/.cargo/config.toml` 统一重编 `core/alloc` 及 `compiler-builtins-mem`。LoongArch 统一关闭 `ual`，使内核和标准库都生成严格对齐的访存；不能只约束内核后继续链接预编译的标准库。Make 清除外部 `RUSTFLAGS`、`CARGO_ENCODED_RUSTFLAGS`，通过 Cargo 原生 `--config` 数组合并 panic/unwind 参数与目标 ISA 约束。直接 Cargo 构建仍从 `os/` 执行；独立用户 std 程序继续消费已准备的目标标准库和 sysroot，不读取内核配置。

单架构构建默认按 `ARCH` 选择 `MUSL_CROSS_RV` 或 `MUSL_CROSS_LA`，分别默认为 `riscv64-linux-musl-` 和 `loongarch64-linux-musl-`；`MUSL_LIBC`、`GLIBC_LIBDIR` 同样默认选择对应的 `_RV` 或 `_LA` 来源。直接设置单架构参数（包括设置为空）优先于上述默认值；`make all` 分别使用两架构参数。未提供运行库来源时，准备器使用其文档中的固定默认路径，缺失则报错。工具链命名或安装路径不同时，应明确覆盖相应前缀和运行库来源。

```sh
make help
make build-kernel ARCH=riscv64
make build-kernel ARCH=loongarch64 KERNEL_BUILD_PROFILE=release
make build-guest-userland ARCH=riscv64
make prepare-guest-runtime-libraries ARCH=riscv64
```

| 入口 | 责任 |
|---|---|
| `build-kernel` | 只构建所选内核；默认 debug、abort，输出 `build/kernel/ARCH/PLATFORM/PROFILE/kernel.elf` |
| `build-guest-userland` | 增量构建 BusyBox、C 程序、release std 比赛执行器与用户自测执行器 |
| `prepare-guest-runtime-libraries` | 从明确的本地 libc 来源准备匹配架构的库 |
| `assemble-root-filesystem` | 消费用户程序、库和配置，替换项目拥有的单根输出树 |
| `create-ext4-image` | 只消费指定完整树，生成无 journal 的 raw ext4 卷 |
| `build-qemu-artifacts` | 生成一个架构的固定 release/abort 内核和单根系统卷 |
| `build-board-dtb` | 从仓内完整源码生成所选实体板OS DTB，不读取个人SDK或旧构建树 |
| `all` | 生成并导出 `kernel-rv`、`kernel-la`、`disk.img`、`disk-la.img` |

两张导出卷分别只有对应架构的根 `/`，不再复制 `/riscv64`、`/loongarch64` 双根。普通 ELF 不启用 ktest；显式 debug 构建不替换固定交付。

std 程序目标为 `riscv64gc-unknown-linux-musl`、`loongarch64-unknown-linux-musl`。`RUNNER_SYSROOT` 指定 Rust 目标标准库所在的 sysroot；不指定时使用当前 `rustc` 的 sysroot。`RUNNER_LINKER` 默认是所选 `MUSL_CROSS` 加 `gcc`，可独立指定交叉链接器；链接器的 C sysroot 与 Rust sysroot、动态运行库资产分别准备。直接依赖、锁文件及 vendor 在 `userland/contest-run`，直接 Cargo 命令必须以该目录为工作目录，清除外部交叉 `RUSTFLAGS`，使用 `--locked --offline`。

默认 BusyBox、项目 C 用户程序和两个 std 执行器均静态链接；外部测试盘也可能包含动态程序。动态程序须在其客体根内找到 `PT_INTERP` 指定的解释器及匹配的依赖库，可用 `readelf -l -d` 核对实际产物。交叉编译器包装器可能把宿主路径写进解释器或运行库搜索路径；构建动态程序时应使用与客体根相匹配的目标工具链和运行库，不修改原比赛输入冒充原始结果。

完整交付的本地输入示例：

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

这些路径必须替换为实际输入。运行库来源和解释器约束见 [sources.md](../userland/runtime/sources.md)；内核、用户程序、库和卷是不同产物，只重建受改动影响的部分。

## 2. 单根树、卷与会话

```sh
make assemble-root-filesystem ARCH=riscv64 ROOTFS_OUTPUT_DIR=build/my-root-rv
make create-ext4-image ARCH=riscv64 \
  ROOTFS_TREE_DIR=build/my-root-rv \
  SYSTEM_ROOT_EXT4_IMAGE=build/my-root-rv.ext4
```

装配替换其明确拥有的输出，删除源配置也会删除树中的旧文件；不向外部输入树回填内容。制盘不隐含构建、补程序或重新装配。外部完整树直接作为 `ROOTFS_TREE_DIR`，必须已含该 ISA 的 init、程序和依赖。

默认卷大小为256MiB，`EXT4_SIZE_MIB` 可指定正整数；8/16/32MiB等小卷能否容纳输入由真实 e2fsprogs 决定，不设128MiB门槛或固定 inode 数。失败保留旧卷和工具诊断。装配、制卷和自测准备允许普通 symlink 父工作区；最终输出链接、实际输入输出重叠或文件别名会被拒绝，树、卷和日志各有独立路径。

同一 inittab 经 `/sbin/start-session` 读取真实 `/proc/cmdline`，选择 runner 或显式 Shell。BusyBox 仍拥有 rcS、子进程回收和 rcK；读取失败或未知会话值报错，不改跑另一会话。Shell 退出不自动运行比赛；需要自然停机时执行 `sync` 和 `/bin/busybox poweroff`。

## 3. 唯一生产 QEMU 入口

运行只消费已有输入，不隐含构建或制盘：

```sh
make run-qemu ARCH=riscv64
make run-qemu ARCH=loongarch64 QEMU_ARGS='--shell'
make run-qemu ARCH=riscv64 QEMU_ARGS='--shell --system-only'
make run-qemu ARCH=riscv64 QEMU_ARGS='--debug'
python3 tools/run_qemu.py --arch loongarch64 --shell --debug --gdb-port 1235
```

`make run-qemu` 只转交参数，实际默认值和校验由 `tools/run_qemu.py` 定义。默认使用对应导出 ELF、系统卷和比赛盘，1 CPU、1 GiB、user 网络、UTC、no-reboot，双盘顺序为比赛 `/dev/vda`、系统 `/dev/vdb`，不额外传 append。内核默认系统根为 `/dev/vdb`、卷内 `/`，默认 RO。

| 选项 | 改动 |
|---|---|
| `--kernel-elf`、`--system-image`、`--contest-image` | 明确已有来源文件；不是输出路径 |
| `--shell`、`--init-program` | 选择用户会话或入口，二者互斥 |
| `--system-only` | 只附系统盘，系统设备变为 `/dev/vda`；通常与 Shell 或明确入口组合 |
| `--root-mode rw` | 明确允许系统根写入，不改变盘动作 |
| `--debug`、`--gdb-port` | 对同一所选 ELF 使用 `-S` 和 GDB server，不切 profile、不构建替代 ELF、不因暂停自动超时 |
| `--cpus`、`--memory`、`--network`、`--host-forward` | 覆盖对应 VM 资源或网络参数 |
| `--print-command` | 仅打印命令，不建目录、锁盘、复制或启动 QEMU |

GDB 使用匹配架构的调试器和同一 ELF 符号，连接所选端口后明确继续。QEMU 控制台强制退出或外部终止不算用户自然关机。

工作盘参数只管理独立副本：

```sh
python3 tools/run_qemu.py --arch riscv64 --shell \
  --working-disk-dir build/manual-rv --working-disk-action fresh
python3 tools/run_qemu.py --arch riscv64 --shell \
  --working-disk-dir build/manual-rv --working-disk-action reuse
python3 tools/run_qemu.py --arch riscv64 --shell \
  --working-disk-dir build/manual-rv --working-disk-action refresh-system
```

fresh 只复制本次所选来源；reuse 只复用工作盘，不读取来源，也不因程序或配置更新换盘；refresh-system 只替换系统工作盘，保留比赛修改。失败不回退另一动作。工作目录锁持有到 QEMU 真实结束，来源盘不交给 Guest 写。

```sh
make clean-build-artifacts
make clean-qemu-working-disks ARCH=riscv64 QEMU_ARGS='--working-disk-dir build/manual-rv'
```

前者仅清可重建编译产物，保留卷、工作盘、sysroot、日志和报告；后者取得同一非阻塞锁后只删明确的工作盘角色，保留目录、锁和无关文件，活盘拒绝清理。

## 4. 按被测对象选择入口

| 被测对象 | 入口与结果边界 |
|---|---|
| Host 参数、协议、文件操作 | `python3 -m unittest discover -s tools/ktest`；不证明 Guest ABI、调度或硬件 |
| 内核对象、错误路径和精确交错 | `list-kernel-tests` / `run-kernel-tests`，仅双 QEMU；实际 fixture 停止、join、释放后核资源 |
| 普通 Linux 用户行为 | 独立 `user-selftests`、默认 C ELF 与真实 TAP；不需要比赛盘或 ktest |
| 初始栈、raw errno、signal/clone 现场 | `user-tests` 构建 raw probe，由原具名 fixture 执行并保留内核观察 |
| 比赛盘与原判题协议 | `/sbin/contest-run`，按本次原盘、TOML 和原协议解释 |

### 内核具名测试

```sh
make list-kernel-tests ARCH=riscv64 CPUS=2 TEST='*final_address'
make run-kernel-tests ARCH=riscv64 CPUS=2 TEST='*final_address'
```

Host 为 `tools/run_kernel_tests.py`。未传 `--kernel` 时只调用内部 `build-kernel-test-elf`；`KERNEL_BUILD_PROFILE`、`KTEST_PANIC_STRATEGY` 选择实际构建，CPUS/MEMORY 只控制 VM。`TEST` 为 glob，`KTEST_ARGS` 转交精确选择、分片等参数。先列出本次 ELF 的完整名；目录显示 ignore 及理由，执行默认排除 ignore，任一不存在的精确名报错，不缩小成部分成功。

`--startup-timeout`、`--setup-timeout`、`--case-timeout`、`--cleanup-timeout`、`--build-timeout` 均以秒计；内部字段明确带 `_seconds`，公开选项名与客体目录的 `timeout_ms` 协议保持不变。

磁盘、UART、网络资源按实际 case 合同提供，缺少输入不是支持性 Skip。普通 Guest 终态来自完整 KTAP，外部观察还可判失败；未知日志不生成结果。只有完整中止/清理记录并确认 QEMU 结束后，Host 才继续已证实未开始的后缀，未知进度保留 NotRun，不透明重放。

### 独立用户自测

```sh
make build-user-selftests ARCH=riscv64
make install-user-selftests ARCH=riscv64
make run-user-selftests ARCH=riscv64
make run-user-selftests ARCH=loongarch64 USER_SELFTEST_ARGS='--collection fs'
```

`USER_SELFTEST_KERNEL_ELF` 指向已有普通 ELF；该目标不构建内核。薄 Host 入口 `tools/run_user_selftests.py` 每次准备新的普通 PID1 输入，经唯一生产 QEMU、BusyBox、rcS/rcK 运行并自然停机，真实等待 QEMU 后读取本次系统工作盘。输出树、来源卷和工作目录分别由 `USER_SELFTEST_ROOTFS_DIR`、`USER_SELFTEST_EXT4_IMAGE`、`USER_SELFTEST_WORKING_DISK_DIR` 指定，UART 保存在工作目录。

每程序预算默认 60 秒，可用 `USER_SELFTEST_ARGS='--program-timeout-seconds 120'` 覆盖；实际值进入程序报告。整次 Host 预算默认 900 秒，可用 `USER_SELFTEST_HOST_ARGS='--host-timeout-seconds 1200'` 覆盖，准备工具、QEMU 和结果读取共用一次绝对期限，日志不续期。QEMU 选项仍经 `QEMU_ARGS` 传递；自动运行只接受 `fresh` 或 `refresh-system`，显式 `reuse` 报错，不暗改动作。普通 `run-qemu` 的三种盘动作、Shell 和 GDB 不变。

客体可直接执行默认 ELF，或选择同一安装清单：

```sh
cd /test/selftests/fs
./file_shared
/sbin/user-selftests --list
/sbin/user-selftests --program fs:file_shared
```

默认程序、辅助 ELF 和数据只由根 Make 声明，安装时生成 `/test/selftests/kselftest-list.txt`，每行 `collection:program`；辅助程序不计默认通过。执行器解析真实 TAP 和进程退出，返回 0/1/2/4 分别为通过、失败、基础设施错误、全 Skip，本身不关机。

编译缓存 `USER_SELFTEST_BUILD_DIR` 默认是 `build/user-selftest-cache/ARCH`，与 `GUEST_USERLAND_OUTPUT_DIR` 安装树分开。每次安装按当前声明重建 `test/selftests`、`test/scripts`，撤销程序或脚本后重新安装、装配、制卷不会带上旧缓存文件。extended helpers 仍安装在 `/bin`，`extended-bin.txt` 仅记录它们的安装删除责任，不是执行清单。不要用 `kselftest-list.txt` 过滤完整安装树；不在默认清单内的 helper、脚本和数据仍须保留。

自动 init 动作在执行前删除旧状态，结束后写 `/test/selftests/exit-status` 的完整一行 `run=<32位本轮标识> status=<0|1|2|4>`，再正常关机。Host 持工作盘锁到真实 wait 和盘面读取完成，只接受本轮标识；缺失、畸形、旧轮次、QEMU 非零和 Host 读盘错误均不能通过。薄入口正常传播四种用户状态，Host 错误返回 2；GNU Make 对非零 recipe 的外层出口为 2，日志仍保留原用户状态。QEMU 退出 0 或 UART 中的成功文字不替代盘面结果。

同源码的 Linux 参考入口只构建、运行本机 C 程序，不启动内核、BusyBox 或用户执行器：

```sh
make build-linux-user-selftests
make run-linux-user-selftests
```

宿主原生结果不当目标 ISA 证据；合法 UDP 大报文需要宿主网络支持，不通过缩小被测报文改变断言。

## 5. 比赛配置、进程与协议

```sh
/sbin/contest-run /dev/vda --list
/sbin/contest-run /dev/vda --plan /etc/contest/plan-onsite.toml --list
/sbin/contest-run /dev/vda --plan /etc/contest/plan.toml \
  --file sets/basic.toml --file sets/libctest.toml
```

输入及注释在 [userland/contest](../userland/contest/)。CLI 为 `DEVICE [--plan FILE] [--file FILE ...] [--list]`；list 只挂载和读取本次真实来源，不执行准备、Hook、测试或关机。默认计划不因目录存在而自动加入未引用清单。

files 相对入口文件解释，顺序为入口 step、files、roots、include；exclude 只影响本 step，精确名/模式无匹配、空选择或冲突报错。override 只作用已选 ID，不重新选择。exec 保 argv，shell 才调用明确解释器；cwd、绝对链接、PT_INTERP 和 DSO 按子根实际解析，缺 loader 不用环境变量或另一 libc 假成功。

每轮沿 setup → before → 子进程与输出收尾 → 对应 after → cleanup 执行；原 wait、exec errno、超时、Hook/清理错误分别记录。启动前建立一次单调绝对期限，日志不续期，结束时真实 kill/wait 所有权内的进程；进程组不等于任意脱离后台树。

Plain/LTP 保原输出端点；需要正文的 Basic、固定 Pre2025、Tap 使用有界捕获/转发。协议保原判定规则，普通 PASS/FAIL 字样不改签结果。纯配置 Skip、缺解释器和已启动失败不能混为同一种通过。

内核专属 Hook 仅在确需时使用 [contest_hooks.rs](../os/src/contest_hooks.rs) 和 `hooks=true`，不按普通 case 名新增空分支。它运行在普通可睡眠控制者上下文，不在目标 fault/IRQ 时刻；全局观察也可能包含后台任务。运行库及源码导航见 [runtime 开发导读](../userland/runtime/development.md)。

## 6. 四平台布局与人工上板

内核 target 为 `riscv64gc-unknown-none-elf` 或 `loongarch64-unknown-none`；`TARGET_PLATFORM` 为默认 `qemu-virt` 或匹配架构的 `visionfive2`、`ls2k1000la`。编辑器为所选 target 开启 build scripts/proc-macro，并向构建脚本与分析传同一 `BLOSSOM_PLATFORM`，不要从宿主 target 分析 `no_std` 内核。

`os/src/platform/layout.rs` 是四平台纯 Rust 布局来源，build.rs 仅在 OUT_DIR 生成链接输入；离线包装读取最终 ELF，不另抄装载地址。源码布局、MMIO 物理地址、CPU 虚拟地址和 DMA 设备地址不能互换。

`os/src/arch/` 的 `entry.rs`、`trap.rs` 和 `user/entry.rs` 以具名 `#[naked]` 函数拥有机器入口，通过 `sym/const` 关联 Rust 处理函数和现场布局。BSP 的 BSS 清零及 RV bootstrap 填表使用普通 Rust 控制流，复用原静态资源；只有完整字存储、寄存器现场、切栈和特权返回保留必要汇编。固定复制字节和 LA 精确 idle 窗口仍有独立的最小 `global_asm!`。

机器入口在建立受信栈、保存完整现场之前不得调用普通 Rust。RV 的分页前初始化函数按高 VMA 链接，由裸入口从链接器提供的物理别名调用；其代码必须自包含、位置无关，不能依赖 GP/TLS、高地址数据或普通辅助函数。这样保留正式测试展开记录，不关闭 unwind 或为测试换另一套算法。LA 三个向量分别由专用链接段和符号 ASSERT 保证 4KiB 对齐，idle 保留 32 字节窗口。修改后须核最终 ELF 的地址、指令、真实目标和布局，并运行实际启动、中断、切换与用户返回；仅通过编译不能证明协议保持。

板内核、固件容器、RAM/持久根和普通用户程序的人工步骤见[上板与根卷指南](boards.md)。

板构建与离线包装不依赖ktest；ktest不支持实体板传输。仓内设备树只适用于各自记录的板卡及固件事实，实际装载范围、分区和串口参数须按设备确认；实板、DMA/IRQ/PHY及同介质持久重启只按各自输入和运行证据记录。保留完整日志、输入身份和实际失败，QEMU或离线产物不签实板Pass；LA NX既有暂缓不计Pass。不扩journal/recovery、恶意坏盘防护或新的测试管理框架。

## 7. 语义常量与等价修改

寄存器、位域、ABI命令、布局和单位使用表达合同的具名常量，优先复用所属模块的已有定义；相关大小和掩码由基础定义推导。相同数值不等于相同语义，文件块大小不能仅因与页大小相等就改用`PAGE_SIZE`。明显的0/1、局部索引、汇编数字标签和独立测试预期不机械替换。

这种修改仍须保持类型、符号、溢出、机器现场、错误返回和同步顺序。汇编操作数及链接脚本有各自的求值约束；改后核最终ELF的指令、真实目标、对齐和装载权限，并运行受影响的内核与普通用户路径，不能只靠fmt或Clippy。[Linux coding-style §12](https://docs.kernel.org/process/coding-style.html#macros-enums-and-rtl)针对C宏和枚举；Rust的`const`、类型与模块归属按本项目职责选择，不把这些选择冒称为Linux原文要求。

## 8. Clippy 诊断与等价清理

按所选 target 检查普通内核与 `ktest-dev/ktest-release`；改动涉及 panic 配置边界时也检查 `ktest-abort-*`。同一源码在不同 ISA、平台和 feature 下的消费者不同：某一配置的 `unused_imports` 或 `dead_code` 不能证明定义可删。共享布局还会被宿主构建脚本读取，不能按宿主 ISA 裁掉目标定义。

优先改冗余借用、同型转换、明确保持 Rust 2018 借用语义的数组迭代，以及可证明同求值顺序的简单表达式。有明确局部角色的复杂类型可用透明类型别名命名，不新增包装结构或抽象层。问号、组合子、`append`、比较分支和 `unsafe` 借用仍须核对锁 guard、临时值、最后拥有与析构次序。配置限制只跟随真实的测试或架构消费者；无消费者的内部入口才删除。

不为消除诊断改变硬件或用户 ABI、有外部约束的布局、原子观察、错误返回、测试名字与断言。内部冗余字段若无消费者、裸指针或布局依赖，可以连同无效写入删除；记录内部布局可能变化，保留真实生产状态和拥有协议。删除手写测试 anchor 时，必须分别核对宏生成槽、链接保留、最终 ELF、完整目录元数据及实际选择行为。

`Vec<Box<T>>`、大枚举、大错误值或多参数建议若会改变稳定地址、分配失败点、无堆环境或同步责任，应保留并说明理由，不靠批量抑制、空调用或无业务意义的包装取绿。错误诊断的修复单独记录为有意改变输出，保留触发条件和失败点，用真实输入核对修前、修后内容，不为格式化新增临时堆字符串。固定同配置统计与原日志分开保存，修后仍须构建实际产物、运行受影响内核路径和普通用户程序，并观察停止与资源回收；Clippy 通过不等于零警告或功能等价。
