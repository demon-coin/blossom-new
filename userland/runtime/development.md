# 当前源码与运行根导读

关于日常构建、三类测试体系、多文件 TOML 评测计划、QEMU 工作盘恢复/刷新以及四项核心交付产物的操作命令，请统一参阅 [开发、测试与比赛执行](../../docs/development.md)。关于动态运行库的具体来源、官方测试盘镜像哈希以及已知失败边界，请参阅 [sources.md](sources.md)。这两份公开文档均可独立阅读，无需预先查阅本地阶段验收记录或外部开发计划。

本项目是支持 `riscv64` 与 `loongarch64` 双架构的 Rust `no_std` 宏内核。用户态比赛执行器位于独立的 Rust `std` 包 `userland/contest-run` 中，编译安装路径为 `/sbin/contest-run`；系统启动后始终由 BusyBox `init` 作为 `PID 1` 运行（早期版本中基于 Shell 的固定 selector 脚本与 C 语言 `contest-exec` 转发程序已退役，不再作为正式入口）。在工程验证中，程序编译通过、目录树装配成功、QEMU 虚拟机运行通过以及实体开发板验证通过属于四个不同层级的结论，必须分别独立核实。

## 活模块导航

内核当前实际参与编译的模块树以 [os/src/main.rs](../../os/src/main.rs) 中的顶层 `mod` 声明及其子模块为准（工作区中遗留的未引用文件或静态索引残留并不参与实际编译）：

| 阅读方向 | 当前活跃源码模块与核心职责 |
|---|---|
| 启动与硬件抽象 | [boot.rs](../../os/src/boot.rs)、[platform.rs](../../os/src/platform.rs)、[arch.rs](../../os/src/arch.rs) 负责平台初始化与双架构后端选择；[cpu.rs](../../os/src/cpu.rs)、[irq.rs](../../os/src/irq.rs)、[console.rs](../../os/src/console.rs)、[sync.rs](../../os/src/sync.rs) 负责多核 CPU 状态、中断控制器、串口控制台与并发同步原语。 |
| 物理内存与用户地址空间 | [mm.rs](../../os/src/mm.rs) 下的 `bootmem`、`page`、`buddy`、`pcp`、`slub` 负责物理内存与内核堆分配，`folio`、`dma` 负责页簇与外设 DMA 缓冲区，`space`、`vma`、`user_table`、`uaccess` 负责进程虚拟地址空间、页表与用户态安全拷贝；[cache.rs](../../os/src/cache.rs) 管理共享元数据缓存。 |
| 任务调度与内核服务 | [task.rs](../../os/src/task.rs) 与 [sched.rs](../../os/src/sched.rs) 是唯一的任务管理与调度入口；[wait.rs](../../os/src/wait.rs)、[time.rs](../../os/src/time.rs)、[accounting.rs](../../os/src/accounting.rs)、[timer.rs](../../os/src/timer.rs)、[workqueue.rs](../../os/src/workqueue.rs) 分别管理等待队列、系统时钟、CPU 时间统计、软件定时器与异步延迟工作队列。 |
| 块设备与文件系统 | [drivers.rs](../../os/src/drivers.rs)（含 PCI 与 VirtIO 驱动）、[storage.rs](../../os/src/storage.rs)、[block.rs](../../os/src/block.rs) 提供底层块设备抽象与 `Bio` 请求拆分；[fs.rs](../../os/src/fs.rs) 下的 `ext4`、`inode`、`dentry`、`path`、`file`、`file_table` 以及 `writeback`、`background`、`readahead` 负责 VFS 路径解析、ext4 读写、预读与脏页回写。 |
| 用户态进程与系统调用 | [task/exec.rs](../../os/src/task/exec.rs)、[task/process.rs](../../os/src/task/process.rs)、[task/signal.rs](../../os/src/task/signal.rs)、[task/futex.rs](../../os/src/task/futex.rs) 围绕统一的 `Task` 与 `Mm` 结构实现进程生命周期、ELF 加载、信号投递与快速用户态互斥锁；[syscall.rs](../../os/src/syscall.rs) 为系统调用统一分派入口。动态链接 ELF 直接使用目标根目录内真实的 `PT_INTERP` 解释器与共享库。 |
| 伪文件系统、网络与内核观测 | `fs` 下的 `devfs`、`procfs`、`memfs`、`pipe`、`poll`、`epoll`、`tty` 提供标准系统设备与多路复用接口；[net.rs](../../os/src/net.rs) 下的 `core`、`socket`、`tcp`、`udp`、`unix` 配合 [drivers/virtio_net.rs](../../os/src/drivers/virtio_net.rs)（及板载网卡驱动）提供网络协议栈支持；[klog.rs](../../os/src/klog.rs)、[trace.rs](../../os/src/trace.rs) 及 `/dev/kmsg`、`/dev/ktrace` 提供内核日志与追踪观测接口。 |
| 系统生命周期管理 | [fs/system.rs](../../os/src/fs/system.rs) 负责挂载并切换真实根文件系统视图；`main` 负责启动 `PID 1`、等待并回收子进程、停止网络子系统、同步并卸载文件系统，最后发起底层关机请求。 |
| 内核具名测试（`ktest`） | [ktest.rs](../../os/src/ktest.rs) 仅在启用 `ktest` feature 时参与编译，宿主机侧驱动脚本为 [tools/run_kernel_tests.py](../../tools/run_kernel_tests.py)。例如文件写入相关测试位于 [user/write](../../os/src/fs/ktest/user/write/)（共享夹具 `WriteOwner` 定义在 `write.rs` 中）；网络与串口接收测试夹具均按用例实际需求组合。 |
| 比赛用例选择与执行 | [model.rs](../contest-run/src/model.rs) 定义配置模型与 `ResolvedCase`；[config.rs](../contest-run/src/config.rs) 负责 TOML 解码与一次性静态校验；[selection.rs](../contest-run/src/selection.rs) 根据真实磁盘目录与清单展开待测集合；[execute.rs](../contest-run/src/execute.rs) 负责子进程创建、标准流重定向、超时控制与每轮前后清理；[runtime.rs](../contest-run/src/runtime.rs) 负责挂载目标块设备、准备系统根环境与动态链接器缺失诊断；[report.rs](../contest-run/src/report.rs) 实现各类判题协议解析，[custom.rs](../contest-run/src/custom.rs) 存放特定用例的用户态前置操作。测试计划与集合定义位于 [userland/contest](../contest/)。 |
| 内核专属测试辅助（Hook） | [contest_hooks.rs](../../os/src/contest_hooks.rs) 通过 `devfs` 的 `/dev/testhook` 节点接收完整控制消息并直接分派；Hook 在控制者进程的普通 `Task` 上下文中执行并复用内核 `trace` 机制（而非在被测进程触发异常或中断的瞬间切入，也不是通用的状态回滚框架）。 |
| 用户态诊断工具与冒烟测试 | [userland/tools](../tools/) 存放 `klog-read` 与 `ktrace-control`（安装路径为 `/bin/klog-read` 与 `/bin/ktrace-control`）；[tests/user](../../tests/user/) 下的 `libc-smoke`、`network-smoke`、`observation-smoke` 包含用户态测试断言，其中 libc 与网络测试按功能模块拆分为多个独立的 C 编译单元。 |

需要注意，工作区中若存在未挂载到 `main.rs` 模块树的文件（如 `mm/zram.rs`、`fs/swap.rs`、`lang_items.rs` 或早期废弃的设备/文件系统代码），它们并不参与内核编译，不能据此认为内核已支持 zram、swap、用户态 RTC 设备或全部 Linux ABI。日常开发与问题修复应直接在上述活跃模块中进行，不要恢复早期已废弃的 `HAL`、`TCB`、`MemorySet`、旧版用户缓冲区抽象或旧版测试执行入口。

## 明确 root/cwd/argv 的一次命令

当需要绕过 `.toml` 测试计划、直接在目标测试盘上单独验证某一条具体命令时，可使用 `contest-run` 的单次命令模式：

```text
DEVICE --root ROOT --cwd CWD -- COMMAND [ARG...]
```

其中 `DEVICE` 为块设备路径（如 `/dev/vd[a-z]`）；`ROOT` 与 `CWD` 均必须显式指定为绝对路径（`ROOT` 不得包含 `..` 路径组件），且命令前必须带有 `--` 分隔符与非空命令名。此处 `ROOT` 是指所挂载目标块设备内部的相对根目录，而 `CWD` 是完成 `chroot` 换根之后的当前工作目录（与内核启动参数中的系统 `ROOTDIR` 无关）。例如，当 `/dev/vda` 比赛盘中包含相应测试程序时，可在客体 Shell 中执行：

```sh
/sbin/contest-run /dev/vda --root /musl --cwd / -- \
  ./runtest.exe -w entry-dynamic.exe stat
```

该模式会在 `contest-run` 内部直接构造一个 `ResolvedCase` 并调用同一套正式的 Rust 执行函数（不会转去调用外部 Shell 或 C 语言包装器）。由于单次命令模式默认采用通用的 `Plain` 输出协议，它不会自动套用 `libctest` 专属的“退出码 `1` 结合输出正文判定通过”规则；若需使用特定测试集的判定规则，请通过声明了 `kind="libctest"` 的 `.toml` 计划文件运行。

执行过程中，`contest-run` 父进程始终保持在系统根文件系统中，仅在 `fork` 出的被测子进程内执行真实的 `chroot`、`chdir` 与 `execve`。命令行参数数组（`argv`）直接原样传递给 `execve`，不经过任何字符串拼接或 Shell `eval` 展开，因此空字符串参数、包含空格的参数及通配符等特殊字符均能原封不动地传给被测程序；仅当命令显式指定为 `/bin/sh`（或计划中的 `shell` 类型步骤）时，才要求目标子根目录内存在 Shell 解释器。对于动态链接的 ELF 程序，内核会在其切换后的子根目录内查找 `PT_INTERP` 指定的动态链接器与共享库——如果目标子根内缺失所需的链接器或库，执行器不会自动向比赛盘复制文件、不会猜测调用 `/busybox`，也不会用其他 libc 族的库强行覆盖，而是将 `execve` 失败与子进程自身返回退出码 `127` 明确区分并如实记录。

单次命令模式使用的 `Plain` 协议会直接继承标准输入（`stdin`），并将标准输出/错误（`stdout`/`stderr`）通过带 `CLOEXEC` 标志的 `dup` 直接交给被测子进程，既不拦截解析输出正文，也不会修改父进程所借用标准流底层打开文件描述（Open File Description, OFD）的状态标志。测试计划中的 `LTP` 用例同样采用这条零拷贝直通路径；只有需要解析输出正文以判定结果的 `Basic` 与固定 `Pre2025` 等协议，才会创建有界内存队列并派生一个仅调用基础 `libc` 读写的输出转发（relay）子进程（父进程仅对自身独占的管道描述符设置非阻塞 `O_NONBLOCK` 标志）。测试结束或超时到达时，父进程会对输出转发子进程执行完整的管道排空（drain）或定向信号终止并 `wait` 回收；若输出转发过程发生 I/O 错误，将独立记录报告，不会篡改被测主进程的原始 `wait` 退出状态。`contest-run` 始终按单线程顺序逐一执行用例，不引入多线程并发池或后台全系统 `/proc` 进程树扫描（关于绝对超时与进程组回收的详细说明请参阅 [公共操作指南](../../docs/development.md#5-比赛配置进程与协议)）。

仓库自带的 [contest-argv](../../tests/user/scripts/contest-argv) 脚本专门用于校验参数原样传递与工作目录切换。该脚本仅安装在项目**自有的非评分测试根** `/tests` 目录下（且该根内实际包含 `/bin/sh`），切勿将其注入官方比赛测试盘中：

```sh
/sbin/contest-run /dev/vda --root / --cwd /tests -- \
  /bin/sh ./contest-argv /tests 'two words' '' '*' ';not-eval'
```

单次命令执行完毕后会直接返回调用方 Shell，不会自动触发系统关机；而通过 `.toml` 计划批量运行时，则由计划中的 `finish` 字段明确指定结束后是执行 `poweroff` 还是 `return`。两种模式共用相同的块设备挂载、绝对超时控制、子进程回收以及逆序卸载（`umount`）逻辑：主测试结果与清理阶段的卸载错误分别独立记录，且只执行标准的正常卸载（不使用强制或延迟 lazy 卸载掩盖资源泄漏）。在进入客体系统手动调试前，请使用完整的自有系统镜像并通过 `--shell`（或 `blossom.boot=shell`）显式进入交互式 Shell；切勿在默认自动运行比赛计划并即将关机的 `inittab` 会话中并发启动第二个 `contest-run` 实例。

## 学生与 agent 的最短修改路线

1. **仅调整测试集合选择、命令行参数、`root`/`cwd`、超时时间或重复次数**：直接修改 `userland/contest/` 下的 `.toml` 配置文件，先用 `--list` 核对展开后的实际用例列表，再定向运行测试。虽然装配时会将整个 `userland/contest/` 目录打包进 `/etc/contest/`，但只有被入口计划文件显式引用的配置才会生效。注意：如果复用了已有的 QEMU 工作盘目录，必须显式传入 `--working-disk-action refresh-system` 才会将更新后的系统根镜像写入 `root.raw`（使用 `reuse` 不会更新系统盘内容）。
2. **新增判题结果解析协议或特定用例的用户态环境准备**：修改 `userland/contest-run/src/` 下对应的具体模块，并补充宿主机侧单元测试；涉及真实的 `execve`、超时控制、`PT_INTERP` 动态链接器解析与输出捕获的行为，仍须在 QEMU 客体系统中实际运行验证。编写前置操作时请注意区分执行上下文：`case.setup` / `case.cleanup` 运行在切换后的用例子根目录（case root）内，而 `roots.NAME.prepare` 则运行在父系统根目录内。
3. **实现新的内核系统调用 ABI 或修复并发/内存缺陷**：直接在内核对应的活跃子系统模块中修改，并补充针对性的最小 `ktest` 内核测试或 `user-selftests` 回归测试。运行内核测试前先通过 `make list-kernel-tests` 获取完整的具名用例路径，再按名称定向执行。修复问题时必须保持真实的系统调用 ABI、准确的错误码（errno）、确定性的并发交错控制、失败路径的资源回滚以及完整的线程停止与内存释放，切勿通过人为清零统计计数器的方式来凑出表面通过。
4. **仅需读取内核日志或控制追踪缓冲区**：直接使用 `userland/tools/` 下的现成工具（`/bin/klog-read` 与 `/bin/ktrace-control`）；若需编写带断言的回归测试，请放在 `tests/user/` 目录下。无需将所有 C 语言测试程序重写为 Rust，也不要为每一个普通比赛用例都在内核中添加空的 `ktest` 或 `testhook` 分支。

关于具体的编译命令、Cargo 工作目录要求、`sysroot`/`linker` 参数、四项交付产物构建、单根文件系统装配与制盘以及带锁工作盘清理，请直接查阅 [公共操作指南](../../docs/development.md)。

## 新盘与能力边界

当面对一张新的整盘 `ext4` 测试镜像，若仅涉及 ELF 程序路径、目录结构、运行参数、子根目录位置变化，或者盘内仅包含单一 libc 族（如仅含 musl 或仅含 glibc），通常只需调整 `userland/contest/` 下的配置文件与显式准备步骤即可适配；但若新测试依赖尚未实现的系统调用、新的磁盘文件系统格式、尚不支持的分区格式或特性（当前已支持标准 MBR 主分区与 GPT 1.0 主分区表，不支持 MBR 扩展分区链）或新的开发板外设控制器，则需要编写对应的内核模块代码。制作项目自有的新系统根镜像统一使用 `make create-ext4-image EXT4_PROFILE=seed4k ROOTFS_TREE_DIR=... SYSTEM_ROOT_EXT4_IMAGE=...`（切勿使用该命令重新打包官方比赛测试盘，也不要将内核对特定 `legacy1k` 等格式的兼容支持外推为支持任意 `ext4` 特性）。

对于因内核尚未实现用户态 RTC 设备节点或相关 `ioctl` ABI 而导致的测试失败（如 `hwclock`），应如实归因为内核 ABI 未实现，而非 `contest-run` 的子根路径配置错误。此外，项目生成的标准 ext4 镜像不创建日志（journal），内核也不包含日志校验重放（replay）、断电崩溃恢复引擎或针对恶意构造损坏磁盘镜像的防御加固；正常的 `sync` 落盘、QEMU 工作盘 `reuse` 复用以及 `refresh-system` 刷新均不等同于崩溃恢复功能。内核在 `auxv` 中提供的 `AT_RANDOM` 仅为教学用途的确定性非加密字节序列。请严格按照已实际运行的测试范围记录结论，不要外推宣称已完整支持全部 Linux/Rust `std`/LTP 特性、伪终端（PTY）、命名空间（namespace）或所有实体板路径。
