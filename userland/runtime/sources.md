# 运行库与原测试输入来源

关于日常构建、三类测试体系、评测计划配置与工作盘管理命令，请参阅 [公共操作指南](../../docs/development.md)；关于当前源码模块结构与单次命令运行方式，请参阅 [运行根导读](development.md)。本文档专门说明动态运行库资产的准备流程、官方比赛测试盘的版本与哈希身份，以及历史测试输入的已知边界。运行库准备逻辑由同目录下的脚本 [prepare_runtime_libraries.py](prepare_runtime_libraries.py) 实现（注意：在宿主机上成功准备运行库资产，并不等同于客体系统内的测试已经运行通过）。

运行库与原始测试程序固定采用官方仓库 [oscomp/testsuits-for-oskernel（提交 `8b58dd16d26d30f7c74d48d5832d870d3051b703`）](https://github.com/oscomp/testsuits-for-oskernel/tree/8b58dd16d26d30f7c74d48d5832d870d3051b703) 以及官方评测容器 `docker.educg.net/cg/os-contest:20250624`。`prepare_runtime_libraries.py` 仅从显式指定的本地路径复制所需的库文件，不会搜索其他版本、不会联网下载或安装工具链，也绝不会重新编译或修改原始测试的 ELF 二进制文件、`PT_INTERP` 解释器路径或共享库（DSO）。

正式的比赛执行器 `/sbin/contest-run` 是由 `userland/contest-run` 编译生成的 Rust `std` 静态链接程序，固定使用 `nightly-2025-01-18` 工具链；Makefile 直接调用 `PATH` 中已就绪的 `cargo`/`rustc`（不传 `+toolchain` 参数，也不启用自动安装），因此对应架构的 Linux musl 目标标准库（sysroot）与交叉链接器必须事先准备就绪。`userland/contest-run` 目录下的 `Cargo.lock` 与包含 17 个依赖包的 `vendor/` 目录是由 Cargo 正式生成的离线输入（其中锁定了 `toml_edit 0.22.20`）。**请特别注意：本文下方准备的客体动态运行库输出目录并非 `RUNNER_SYSROOT`，不能用来替代编译 `contest-run` 所需的 Rust 目标标准库。** `contest-run` 自身采用静态链接，与被测程序采用动态链接完全互不冲突；内核的裸机编译 target 与用户态 Linux target 各司其职，日常 Make 构建也不依赖 Nix 环境。

## 官方测试盘与身份

官方 [测试仓库说明文档](https://github.com/oscomp/testsuits-for-oskernel/blob/pre-2025/README.md) 描述了整盘无分区表的 `ext4` 比赛镜像结构。当前项目固定选用 [pre-20250615 Release](https://api.github.com/repos/oscomp/testsuits-for-oskernel/releases/tags/pre-20250615) 发布的两张镜像资产：

| 架构 | 官方发布资产 | 发布压缩包 SHA-256 摘要 |
|---|---|---|
| `riscv64` | [sdcard-rv.img.xz](https://github.com/oscomp/testsuits-for-oskernel/releases/download/pre-20250615/sdcard-rv.img.xz) | `2b59aee4d26681b2d78a8faa615ac216f723a8ea157544e7867bbc8c77aac087` |
| `loongarch64` | [sdcard-la.img.xz](https://github.com/oscomp/testsuits-for-oskernel/releases/download/pre-20250615/sdcard-la.img.xz) | `3cbeae1095d788cd8255e4929bcc6dace96dbe4999b6eaec003ae6fe2cdba42b` |

下载后请先校验 Release 版本、文件名及 `.xz` 压缩包的 SHA-256 摘要，随后解压得到未压缩的 raw 整盘镜像；QEMU 启动脚本的 `--contest-image` 参数只接受解压后的匹配架构 raw 镜像，不直接接受 `.xz` 压缩文件。注意：解压官方镜像并非重新制盘，严禁使用 `create-ext4-image` 或 `mke2fs -d` 重新打包生成比赛评分盘。镜像内部实际包含哪些测试 ELF、`runtest`、Shell 解释器、动态链接器（loader）或共享库（DSO），均以挂载后的真实盘面内容为准（不能仅凭 Release 名称或默认测试计划就假定某个文件必然存在）。

宿主机启动入口每次仅将原始比赛盘复制为工作目录中的临时工作副本（`contest.raw`）再交给 QEMU 挂载。早期调试阶段曾使用过的名为 `original` 或 `prepared` 的本地重制盘或注入盘，仅保留其各自的哈希与历史日志作为排障参考，不作为官方原始测试盘统计成绩。

## supplied-first 与明确准备

当前的 Rust 比赛执行器默认遵循“优先使用测试盘自带环境（supplied-first）”原则：直接使用所选子根目录（root）内自带的原始 ELF、`PT_INTERP` 解释器路径、动态链接器（loader）、共享库（DSO）与测试脚本，不再像早期 Shell 脚本那样自动执行 `prepare_family`，也不会仅因为子目录名为 `musl` 或 `glibc` 就去覆盖盘内已有的库文件。需要注意，当 ELF 的 `PT_INTERP` 动态链接器本身缺失时，单纯设置 `LD_LIBRARY_PATH` 环境变量并不能让程序启动；此时执行器会如实记录 `exec` 启动失败，而不会将其归类为支持性跳过（Skip）。

`prepare_runtime_libraries.py` 仅负责在宿主机侧整理下方列出的辅助动态运行库资产：由 `make prepare-guest-runtime-libraries` 通过 `RUNTIME_LIBRARIES_OUTPUT_DIR` 指定输出目录，再由 `assemble-root-filesystem` 将其安装到系统根文件系统的 `/opt/runtime/{musl,glibc}` 下——**这一步并不会把运行库自动注入到比赛测试盘中**。如果某张旧版测试盘确实缺少运行库、需要在运行前补充，应在显式选用的测试计划中通过 `roots.NAME.prepare` 声明准备步骤（该命令在父系统根环境中执行，其目标路径参数显式指向 `/mnt/contest/...` 挂载点；而单个用例的 `case.setup` / `case.cleanup` 则在切换后的用例子根内执行）。对于已经自带匹配动态链接器与共享库的新版测试盘，直接原样运行即可，无需执行额外替换。

在调试动态链接程序时，请注意区分三种不同的准备方式：按 `PT_INTERP` 要求的路径建立符号链接、显式补充缺失的共享库文件、或者直接将动态链接器（loader）作为主程序调用以加载目标二进制。其中，直接调用 loader 启动程序会改变子进程看到的 `argv[0]` 与辅助向量（`auxv`），并不完全等价于内核直接加载动态 ELF。在项目自有的非评分测试根中可以进行上述实验；但对于官方比赛评分盘，切勿向盘内注入或替换程序、脚本、共享库、符号链接或测试断言后再将其当作原始测试盘成绩。凡经过显式环境准备的步骤，其输入状态与运行结果均应明确标注。

## 四组固定路径与真实请求

下表列出了官方评测容器内的四组运行库路径（同时也是 `prepare_runtime_libraries.py` 在未传入显式路径参数时按架构查找的默认源路径），以及比赛盘内原始动态 ELF 程序头（`PT_INTERP`）所请求的动态链接器路径：

| 架构与 libc 族 | 官方容器内源文件路径 | 原始动态 ELF 请求的 `PT_INTERP` 解释器路径 |
|---|---|---|
| RV `musl` | `/opt/riscv64--musl--bleeding-edge-2020.08-1/riscv64-buildroot-linux-musl/sysroot/lib/libc.so` | `/lib/ld-musl-riscv64-sf.so.1` |
| RV `glibc` | `/usr/riscv64-linux-gnu/lib` | `/lib/ld-linux-riscv64-lp64d.so.1` |
| LA `musl` | `/opt/loongarch64-linux-musl-cross/loongarch64-linux-musl/lib/libc.so` | `/lib64/ld-musl-loongarch-lp64d.so.1` |
| LA `glibc` | `/opt/gcc-13.2.0-loongarch64-linux-gnu/sysroot/usr/lib64` | `/lib64/ld-linux-loongarch-lp64d.so.1` |

在 `musl` 族中，原始动态程序依赖 `libc.so`，且 `PT_INTERP` 解释器符号链接直接指向同一份 `libc.so`。在 `glibc` 族中，必需的三个库文件为 `libc.so.6`、`libm.so.6` 以及上表对应架构同名的动态链接器 `ld-linux-*.so.1`（其中 RISC-V 原始动态程序还在动态段中显式声明了对该 loader 的依赖）。准备脚本会逐字节原样复制这些共享库文件，绝不修改 `musl` 的 `libc.so` 二进制内容，也不用静态库（`.a`）冒充动态库（`.so`）。

## 显式本地来源

在本地开发机上，通常先从上述官方容器中提取出对应架构的运行库文件，再将本地提取路径显式传给准备脚本（每个架构使用各自独立的输出目录）。例如，当本地提取目录位于 `/srv/contest-runtime/<arch>/musl/libc.so` 和 `/srv/contest-runtime/<arch>/glibc/lib/` 时，执行：

```sh
python3 userland/runtime/prepare_runtime_libraries.py --arch riscv64 \
  --musl-libc /srv/contest-runtime/riscv64/musl/libc.so \
  --glibc-libdir /srv/contest-runtime/riscv64/glibc/lib \
  --output-directory build/runtime-riscv64
python3 userland/runtime/prepare_runtime_libraries.py --arch loongarch64 \
  --musl-libc /srv/contest-runtime/loongarch64/musl/libc.so \
  --glibc-libdir /srv/contest-runtime/loongarch64/glibc/lib \
  --output-directory build/runtime-loongarch64
```

命令行中的 `--arch` 与 `--output-directory` 为必填参数（所有长选项均不接受缩写）；若省略 `--musl-libc` 或 `--glibc-libdir`，脚本只会查找上一节表格中的固定容器路径，找不到文件则直接报错退出。宿主机 `PATH` 中必须提供支持 RISC-V 与 LoongArch 双架构的 `readelf` 工具。脚本会先检查所有必需的源文件是否存在，随后对每个源文件实际调用 `readelf -h -l`，严格校验其 ELF 机器架构为 `RISC-V` 或 `LoongArch` 且文件类型为 `DYN`（共享对象），同时在标准输出/错误流中保留工具的原始诊断信息。一旦遇到架构不匹配、`readelf` 执行失败、库文件缺失或文件复制/创建符号链接失败，脚本均会返回非零退出码，不会拿旧的输出目录充当本次校验通过。

## 输出与更新责任

脚本生成的目录结构为标准用户态文件树：
- `DIR/musl/lib/libc.so`，以及同目录下指向上表 `musl` 解释器名称的相对符号链接；
- `DIR/glibc/lib/{libc.so.6,libm.so.6,对应架构的 glibc loader}`；
- 在 `loongarch64` 架构下，`musl` 与 `glibc` 子目录内还会分别创建 `lib64 -> lib` 符号链接，使原始程序请求的 `/lib64/...` 路径能够顺畅解析到 `lib/` 目录。

后续执行 `assemble-root-filesystem` 时，会将这两个运行库目录组装进项目管理的系统根文件树；而 `create-ext4-image` 仅负责将组装好的完整根文件树打包为镜像，不会触发运行库准备，也不会在内核中添加硬编码路径重写逻辑。

重复运行 `prepare_runtime_libraries.py` 时，脚本只会原地更新上述四个具名共享库文件、当前架构的 `musl` 解释器符号链接以及 LoongArch 的两个 `lib64` 符号链接，不会递归删除整个输出目录或清理其他无关文件。复制库文件时采用“先写入同目录临时文件、再原子替换目标文件”的策略（若中途失败会自动清理临时文件）；若待创建符号链接的路径上已存在同名目录，脚本会直接报错并保留原目录不变。由于中途出错退出后输出目录可能残留部分已更新的文件，上层调用者必须严格以脚本当次的退出码判断成功与否，不能仅凭目录中存在文件就继续后续打包。

## 原输入与现有失败边界

针对固定 `pre-2025` 测试集的结果解析器严格保留了原始测试脚本、ELF 二进制、共享库与判题协议，绝不通过篡改被测程序来换取表面通过（早期版本中的 Shell selector 脚本已不再作为正式入口）。在已知的测试输入边界中：
- 历史 `basic` 测试集中针对 `vfat` 文件系统与 `/dev/vda2` 分区的挂载请求，与官方提供的无分区表整盘 `ext4` 镜像格式不匹配；`hwclock` 用例则依赖尚未实现的用户态 RTC 设备 ABI，因此这两类用例如实保留原始失败状态（早期 `basic` 中的 `mount`/`umount` 暂缓项同样保持非 Pass 记录）。
- RISC-V 架构下 `gnu` 静态链接的 `clock` 原始测试程序在调用成功后会额外断言 `errno == 0`：在历史运行现场中，虽然 `clock` 相关系统调用本身成功返回了 `0`，但此前 C 运行库初始化阶段调用可选系统调用 `getrandom` 返回的 `ENOSYS` 仍保留在线程局部的 `errno` 中；内核绝不能违反 POSIX 规范在系统调用成功时主动清零用户态 `errno`，也不能返回伪造的硬件随机数去掩盖 `ENOSYS`。这一现象属于原始测试二进制与标准 ABI 交互的真实边界，不会因为更换执行器或重跑测试而自动消失。
- 在网络测试方面，当前宿主机 WSL 环境特殊的 `127.0.0.0/8` 回环路由行为曾导致大体量 UDP 报文未能完整送达宿主机侧对端进程（客体内核协议栈成功发出 6 个合法的 IP 分片，并不等同于宿主机端到端接收成功）。在临时创建的非特权 user/net 命名空间标准 `lo` 接口上，双架构 `17`、`0`、`1472`、`8193` 字节的 UDP 报文全文收发均已实测通过；但该隔离命名空间仅用于定位问题，不作为日常公共运行入口的自动回退方案。
- LoongArch 的不可执行页保护（NX）目前仍属暂缓项（不计为 Pass），`AT_RANDOM` 辅助向量仅为教学用途的非加密字节；当前项目不扩展用户态 RTC `ioctl`、VFAT/MBR 分区挂载、ext4 日志恢复（journal/recovery）、伪终端（PTY）/命名空间（namespace）或其余未列入范围的 Linux ABI。所有未运行项与已知限制均如实记录。
