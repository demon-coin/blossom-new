.DEFAULT_GOAL := all

ARCH ?= riscv64
TARGET_PLATFORM ?= qemu-virt
KERNEL_BUILD_PROFILE ?= debug
KERNEL_ELF_OUTPUT ?= build/kernel/$(ARCH)/$(TARGET_PLATFORM)/$(KERNEL_BUILD_PROFILE)/kernel.elf
KERNEL_CARGO_TARGET_DIR ?= build/kernel/$(ARCH)/$(TARGET_PLATFORM)/target
KTEST_CARGO_TARGET_DIR ?= build/ktest-target
KTEST_PANIC_STRATEGY ?= unwind
KTEST_ARGS ?=
QEMU_ARGS ?=
TEST ?=
CPUS ?= 1
MEMORY ?= $(if $(filter riscv64,$(ARCH)),512M,768M)
ROOT ?= /dev/vda
ROOT_MODE ?= ro
EXTRA_ROOT ?=

ROOTFS_OUTPUT_DIR ?= build/rootfs/$(ARCH)
ROOTFS_TREE_DIR ?= $(ROOTFS_OUTPUT_DIR)
ROOTFS_INIT_CONFIG ?=
SYSTEM_ROOT_EXT4_IMAGE ?= build/system-$(ARCH).img
EXT4_PROFILE ?= seed4k
EXT4_SIZE_MIB ?= 256
GUEST_USERLAND_OUTPUT_DIR ?= build/userland-$(ARCH)
USER_SELFTEST_BUILD_DIR ?= build/user-selftest-cache/$(ARCH)
RUNTIME_LIBRARIES_OUTPUT_DIR ?= build/runtime-$(ARCH)
BUSYBOX_BUILD_DIR ?= build/busybox-$(ARCH)
MUSL_LIBC ?=
GLIBC_LIBDIR ?=
MUSL_LIBC_RV ?=
MUSL_LIBC_LA ?=
GLIBC_LIBDIR_RV ?=
GLIBC_LIBDIR_LA ?=
MUSL_CROSS ?= $(ARCH)-linux-musl-
MUSL_CROSS_RV ?= riscv64-linux-musl-
MUSL_CROSS_LA ?= loongarch64-linux-musl-
MUSL_CFLAGS ?= -std=c11 -O2 -Wall -Wextra -static -fno-pie -no-pie
PROBE_FAMILIES := smoke io clone signal futex lifecycle write writeback device-map pipe events tty jobs
USER_OUTPUTS := $(addprefix build/user-$(ARCH)/user-,$(PROBE_FAMILIES))
PROBE_CFLAGS := -std=gnu11 -D_GNU_SOURCE -O2 -Wall -Wextra -ffreestanding -fno-builtin -fno-tree-loop-distribute-patterns -fno-stack-protector -fno-pie -fno-pic -fno-common -ffunction-sections -fdata-sections -fno-unwind-tables -fno-asynchronous-unwind-tables
PROBE_LDFLAGS := -nostdlib -nostartfiles -nodefaultlibs -static -no-pie -Wl,--no-dynamic-linker,--no-relax,--build-id=none,--gc-sections,-z,max-page-size=4096,-T,tests/user/smoke.ld
PROBE_COMMON := tests/user/probes/entry-$(ARCH).S tests/user/probes/runtime.c
PROBE_CLONE_FAMILIES := clone signal futex lifecycle write pipe events tty jobs
PROBE_EXTRA_io := tests/user/probes/io_vm.c tests/user/probes/io_access.c
PROBE_EXTRA_clone := tests/user/probes/clone-state-$(ARCH).S
PROBE_EXTRA_signal := tests/user/probes/signal_time.c build/user-$(ARCH)/signal-$(ARCH).o
PROBE_EXTRA_futex := tests/user/probes/futex-$(ARCH).S
PROBE_EXTRA_lifecycle := tests/user/probes/lifecycle_wait.c tests/user/probes/lifecycle_parent.c tests/user/probes/lifecycle-$(ARCH).S
PROBE_EXTRA_write := $(addprefix tests/user/probes/write_,buffered.c mmap.c sync.c truncate.c namespace.c)
PROBE_HEADERS_io := tests/user/probes/io_private.h tests/user/probes/io_wire.h
PROBE_HEADERS_writeback := tests/user/probes/io_wire.h
PROBE_HEADERS_write := tests/user/probes/write.h
PROBE_HEADERS_lifecycle := tests/user/probes/lifecycle.h
PROBE_HEADERS_signal := tests/user/probes/signal.h
PROBE_HEADERS_tty := tests/user/probes/tty_wire.h
PROBE_HEADERS_jobs := tests/user/probes/tty_wire.h
RUNNER_TARGET := $(if $(filter riscv64,$(ARCH)),riscv64gc-unknown-linux-musl,loongarch64-unknown-linux-musl)
RUNNER_TARGET_DIR ?= build/contest-run
RUNNER_OUTPUT := $(RUNNER_TARGET_DIR)/$(RUNNER_TARGET)/release/contest-run
USER_SELFTEST_RUNNER_OUTPUT := $(RUNNER_TARGET_DIR)/$(RUNNER_TARGET)/release/user-selftests
USER_SELFTEST_KERNEL_ELF ?= build/kernel/$(ARCH)/qemu-virt/release/kernel.elf
USER_SELFTEST_ROOTFS_DIR ?= build/user-selftest-rootfs/$(ARCH)
USER_SELFTEST_EXT4_IMAGE ?= build/user-selftests-$(ARCH).img
USER_SELFTEST_WORKING_DISK_DIR ?= build/user-selftest-work/$(ARCH)
USER_SELFTEST_ARGS ?=
USER_SELFTEST_HOST_ARGS ?=
LINUX_USER_SELFTEST_OUTPUT_DIR ?= build/user-selftests-linux
LINUX_USER_SELFTEST_CFLAGS ?= -std=c11 -O2 -Wall -Wextra
RUNNER_SYSROOT ?=
RUNNER_LINKER ?= $(MUSL_CROSS)gcc
RUNNER_SYSROOT_RV ?= $(RUNNER_SYSROOT)
RUNNER_SYSROOT_LA ?= $(RUNNER_SYSROOT)
RUNNER_LINKER_RV ?= $(MUSL_CROSS_RV)gcc
RUNNER_LINKER_LA ?= $(MUSL_CROSS_LA)gcc
BOARD_BOOT_IMAGES_OUTPUT_DIR ?= build/board/$(ARCH)/$(TARGET_PLATFORM)/$(KERNEL_BUILD_PROFILE)
BOARD_ROOT_EXT4_IMAGE ?=
BOARD_OS_DTB ?=
BOARD_ROOT_DEVICE ?=
BOARD_ROOT_MODE ?= ro
BOARD_INIT_PROGRAM ?= /sbin/init
BOARD_DTB_OUTPUT ?= build/board/$(ARCH)/$(TARGET_PLATFORM)/board-input.dtb
BOARD_DTS_SOURCE_visionfive2 := os/dts/riscv64/starfive/visionfive2-full.dts
BOARD_DTS_SOURCE_ls2k1000la := os/dts/loongarch64/loongson/ls2k1000-dp.dts
BOARD_DTS_SOURCE := $(BOARD_DTS_SOURCE_$(TARGET_PLATFORM))
HOSTCC ?= cc
DTC ?= dtc
LIBC_SOURCES := tests/user/abi/libc_smoke.c tests/user/abi/libc_common.c tests/user/process/libc_lifecycle.c tests/user/process/script.c tests/user/fs/libc_file.c tests/user/abi/libc_system.c tests/user/process/limits.c tests/user/fs/file_time.c tests/user/abi/clock.c tests/user/net/socket_timeout.c
LIBC_HEADERS := tests/user/include/libc_common.h tests/user/include/fp.h
NETWORK_SOURCES := tests/user/net/network_smoke.c $(addprefix tests/user/net/,common.c tcp.c udp.c unix.c options.c ipv6.c peer.c control.c)
NETWORK_HEADERS := tests/user/include/net_common.h
TEST_GEN_PROGS := fs/file_shared process/fork_exec_wait ipc/pipe_futex_epoll net/tcp_udp process/affinity process/resource_limits fs/descriptor_growth fs/tmpfs_growth process/exec_stack
TEST_GEN_PROGS_EXTENDED := libc-smoke network-smoke observation-smoke
TEST_FILES := $(wildcard tests/user/scripts/*)
USER_TOOLS := klog-read ktrace-control
GUEST_BIN_PROGRAMS := $(TEST_GEN_PROGS_EXTENDED) $(USER_TOOLS)
SELFTEST_OUTPUTS = $(foreach program,$(TEST_GEN_PROGS),$(call make-path,$(USER_SELFTEST_BUILD_DIR)/test/selftests/$(program)))
SELFTEST_HEADERS := tests/user/include/kselftest.h tests/user/include/selftest_io.h

empty :=
space := $(empty) $(empty)
make-path = $(subst $(space),\$(space),$(1))
absolute-path = $(if $(filter /%,$(firstword $(1))),$(1),$(CURDIR)/$(1))

# 两个 std 程序共用同一静态链接参数与已准备的工具链输入。
build-std-program = cd userland/contest-run && env -u RUSTFLAGS RUSTUP_AUTO_INSTALL=0 CARGO_TARGET_DIR="$(call absolute-path,$(RUNNER_TARGET_DIR))" CARGO_ENCODED_RUSTFLAGS="$$(printf '%s\037' '-Ctarget-feature=+crt-static' '-Crelocation-model=static' '-Clink-self-contained=yes' '-Clink-arg=-no-pie' $(if $(RUNNER_SYSROOT),'--sysroot' "$(call absolute-path,$(RUNNER_SYSROOT))"); printf '%s' "-Clinker=$(if $(findstring /,$(RUNNER_LINKER)),$(call absolute-path,$(RUNNER_LINKER)),$(RUNNER_LINKER))")" cargo build --bin $(1) --release --locked --offline --target $(RUNNER_TARGET)

ifeq ($(ARCH),riscv64)
TARGET := riscv64gc-unknown-none-elf
PROBE_ARCH_FLAGS := -march=rv64g -mabi=lp64d -msmall-data-limit=0 -mno-relax
QEMU_KERNEL_EXPORT := kernel-rv
QEMU_ROOT_EXT4_IMAGE := disk.img
else ifeq ($(ARCH),loongarch64)
TARGET := loongarch64-unknown-none
PROBE_ARCH_FLAGS := -march=loongarch64 -mabi=lp64d
PROBE_CFLAGS += -fno-tree-vectorize
QEMU_KERNEL_EXPORT := kernel-la
QEMU_ROOT_EXT4_IMAGE := disk-la.img
else
$(error ARCH must be riscv64 or loongarch64)
endif

ifneq ($(words $(TARGET_PLATFORM)),1)
$(error TARGET_PLATFORM must name exactly one platform)
endif
ifeq ($(filter riscv64:qemu-virt riscv64:visionfive2 loongarch64:qemu-virt loongarch64:ls2k1000la,$(ARCH):$(TARGET_PLATFORM)),)
$(error Unsupported ARCH/TARGET_PLATFORM combination: $(ARCH)/$(TARGET_PLATFORM))
endif
ifneq ($(TARGET_PLATFORM),qemu-virt)
ifneq ($(filter build-kernel-test-elf list-kernel-tests run-kernel-tests,$(MAKECMDGOALS)),)
$(error build-kernel-test-elf/list-kernel-tests/run-kernel-tests require TARGET_PLATFORM=qemu-virt)
endif
endif
ifeq ($(TARGET_PLATFORM),qemu-virt)
ifneq ($(filter build-board-dtb,$(MAKECMDGOALS)),)
$(error build-board-dtb requires TARGET_PLATFORM=visionfive2 or ls2k1000la)
endif
endif
ifeq ($(TARGET_PLATFORM),qemu-virt)
ifneq ($(filter package-board-boot-images,$(MAKECMDGOALS)),)
$(error package-board-boot-images requires TARGET_PLATFORM=visionfive2 or TARGET_PLATFORM=ls2k1000la)
endif
endif

ifeq ($(KERNEL_BUILD_PROFILE),release)
CARGO_PROFILE_ARG := --release
else ifneq ($(KERNEL_BUILD_PROFILE),debug)
$(error KERNEL_BUILD_PROFILE must be debug or release)
endif
ifeq ($(KTEST_PANIC_STRATEGY),unwind)
KTEST_PROFILE := $(if $(filter release,$(KERNEL_BUILD_PROFILE)),ktest-release,ktest-dev)
else ifeq ($(KTEST_PANIC_STRATEGY),abort)
KTEST_PROFILE := $(if $(filter release,$(KERNEL_BUILD_PROFILE)),ktest-abort-release,ktest-abort-dev)
else
$(error KTEST_PANIC_STRATEGY must be unwind or abort)
endif

.PHONY: all build-kernel build-guest-userland prepare-guest-runtime-libraries assemble-root-filesystem create-ext4-image build-qemu-artifacts build-board-dtb package-board-boot-images
.PHONY: build-contest-runner user-tests build-kernel-test-elf list-kernel-tests run-kernel-tests run-qemu clean-build-artifacts clean-qemu-working-disks help
.PHONY: build-user-selftests install-user-selftests run-user-selftests
.PHONY: build-linux-user-selftests run-linux-user-selftests

all:
	$(MAKE) build-qemu-artifacts ARCH=riscv64 TARGET_PLATFORM=qemu-virt MUSL_CROSS="$(MUSL_CROSS_RV)" MUSL_LIBC="$(MUSL_LIBC_RV)" GLIBC_LIBDIR="$(GLIBC_LIBDIR_RV)" RUNNER_SYSROOT="$(RUNNER_SYSROOT_RV)" RUNNER_LINKER="$(RUNNER_LINKER_RV)" ROOTFS_OUTPUT_DIR=build/rootfs/riscv64 GUEST_USERLAND_OUTPUT_DIR=build/userland-riscv64 RUNTIME_LIBRARIES_OUTPUT_DIR=build/runtime-riscv64 BUSYBOX_BUILD_DIR=build/busybox-riscv64
	$(MAKE) build-qemu-artifacts ARCH=loongarch64 TARGET_PLATFORM=qemu-virt MUSL_CROSS="$(MUSL_CROSS_LA)" MUSL_LIBC="$(MUSL_LIBC_LA)" GLIBC_LIBDIR="$(GLIBC_LIBDIR_LA)" RUNNER_SYSROOT="$(RUNNER_SYSROOT_LA)" RUNNER_LINKER="$(RUNNER_LINKER_LA)" ROOTFS_OUTPUT_DIR=build/rootfs/loongarch64 GUEST_USERLAND_OUTPUT_DIR=build/userland-loongarch64 RUNTIME_LIBRARIES_OUTPUT_DIR=build/runtime-loongarch64 BUSYBOX_BUILD_DIR=build/busybox-loongarch64

build-kernel:
	cd os && env -u RUSTFLAGS -u CARGO_ENCODED_RUSTFLAGS BLOSSOM_PLATFORM="$(TARGET_PLATFORM)" CARGO_TARGET_DIR="$(call absolute-path,$(KERNEL_CARGO_TARGET_DIR))" cargo --config 'target.$(TARGET).rustflags=["-Cpanic=abort"]' build --locked --target $(TARGET) $(CARGO_PROFILE_ARG)
	mkdir -p "$$(dirname -- "$(KERNEL_ELF_OUTPUT)")"
	cp "$(KERNEL_CARGO_TARGET_DIR)/$(TARGET)/$(KERNEL_BUILD_PROFILE)/os" "$(KERNEL_ELF_OUTPUT)"

build-qemu-artifacts:
	$(MAKE) build-kernel TARGET_PLATFORM=qemu-virt KERNEL_BUILD_PROFILE=release KERNEL_CARGO_TARGET_DIR=build/kernel/$(ARCH)/qemu-virt/target KERNEL_ELF_OUTPUT=build/kernel/$(ARCH)/qemu-virt/release/kernel.elf
	$(MAKE) assemble-root-filesystem TARGET_PLATFORM=qemu-virt
	$(MAKE) create-ext4-image ROOTFS_TREE_DIR="$(ROOTFS_OUTPUT_DIR)" SYSTEM_ROOT_EXT4_IMAGE="$(QEMU_ROOT_EXT4_IMAGE)"
	@temporary="$$(mktemp -- "./.$(QEMU_KERNEL_EXPORT).XXXXXX")" || exit $$?; \
	trap 'status=$$?; rm -f -- "$$temporary"; exit "$$status"' EXIT; \
	trap 'exit 129' HUP; trap 'exit 130' INT; trap 'exit 143' TERM; \
	cp --preserve=mode -- "build/kernel/$(ARCH)/qemu-virt/release/kernel.elf" "$$temporary" || exit $$?; \
	mv -fT -- "$$temporary" "$(QEMU_KERNEL_EXPORT)" || exit $$?

prepare-guest-runtime-libraries:
	python3 userland/runtime/prepare_runtime_libraries.py --arch $(ARCH) $(if $(MUSL_LIBC),--musl-libc "$(MUSL_LIBC)") $(if $(GLIBC_LIBDIR),--glibc-libdir "$(GLIBC_LIBDIR)") --output-directory "$(RUNTIME_LIBRARIES_OUTPUT_DIR)"

assemble-root-filesystem: build-guest-userland prepare-guest-runtime-libraries
	python3 tools/assemble_rootfs.py --arch $(ARCH) --guest-userland-directory "$(GUEST_USERLAND_OUTPUT_DIR)" --runtime-libraries-directory "$(RUNTIME_LIBRARIES_OUTPUT_DIR)" --output-directory "$(ROOTFS_OUTPUT_DIR)" $(if $(ROOTFS_INIT_CONFIG),--init-config "$(ROOTFS_INIT_CONFIG)")

create-ext4-image:
	python3 tools/create_ext4_image.py --source-tree "$(ROOTFS_TREE_DIR)" --output-image "$(SYSTEM_ROOT_EXT4_IMAGE)" --profile "$(EXT4_PROFILE)" --size-mib "$(EXT4_SIZE_MIB)"

build-board-dtb:
	@directory="$$(dirname -- "$(BOARD_DTB_OUTPUT)")"; \
	mkdir -p "$$directory" || exit $$?; \
	temporary="$$(mktemp -d "$$directory/.board-dtb.XXXXXX")" || exit $$?; \
	trap 'status=$$?; rm -rf -- "$$temporary"; exit "$$status"' EXIT; \
	trap 'exit 129' HUP; trap 'exit 130' INT; trap 'exit 143' TERM; \
	$(HOSTCC) -E -P -C -nostdinc -undef -D__DTS__ -x assembler-with-cpp \
	  -I os/dts/include -I "$$(dirname -- "$(BOARD_DTS_SOURCE)")" \
	  "$(BOARD_DTS_SOURCE)" -o "$$temporary/board-source.dts" || exit $$?; \
	$(DTC) $(if $(filter visionfive2,$(TARGET_PLATFORM)),-@) -I dts -O dtb -p 65536 \
	  -o "$$temporary/board-input.dtb" "$$temporary/board-source.dts" || exit $$?; \
	mv -fT -- "$$temporary/board-input.dtb" "$(BOARD_DTB_OUTPUT)" || exit $$?

package-board-boot-images: build-kernel
	python3 tools/package_board_boot_images.py --arch $(ARCH) --target-platform $(TARGET_PLATFORM) --kernel-elf "$(KERNEL_ELF_OUTPUT)" --output-directory "$(BOARD_BOOT_IMAGES_OUTPUT_DIR)" $(if $(BOARD_ROOT_EXT4_IMAGE),--root-ext4-image "$(BOARD_ROOT_EXT4_IMAGE)") $(if $(BOARD_OS_DTB),--os-dtb "$(BOARD_OS_DTB)") $(if $(BOARD_ROOT_DEVICE),--root-device "$(BOARD_ROOT_DEVICE)") --root-mode "$(BOARD_ROOT_MODE)" --init-program "$(BOARD_INIT_PROGRAM)"

run-qemu:
	python3 tools/run_qemu.py --arch "$(ARCH)" $(QEMU_ARGS)

clean-qemu-working-disks:
	python3 tools/working_disks.py clean --arch "$(ARCH)" $(QEMU_ARGS)

build-contest-runner:
	$(call build-std-program,contest-run)

build-kernel-test-elf:
	cd os && env -u RUSTFLAGS -u CARGO_ENCODED_RUSTFLAGS BLOSSOM_PLATFORM="$(TARGET_PLATFORM)" CARGO_TARGET_DIR="$(call absolute-path,$(KTEST_CARGO_TARGET_DIR))" cargo --config 'target.$(TARGET).rustflags=["-Cpanic=$(KTEST_PANIC_STRATEGY)", "-Cforce-unwind-tables=yes"]' build --locked --target $(TARGET) --profile $(KTEST_PROFILE) --features ktest
	mkdir -p "$$(dirname -- "$(KERNEL_ELF_OUTPUT)")"
	cp "$(KTEST_CARGO_TARGET_DIR)/$(TARGET)/$(KTEST_PROFILE)/os" "$(KERNEL_ELF_OUTPUT)"

list-kernel-tests:
	python3 tools/run_kernel_tests.py list --arch $(ARCH) --cpus $(CPUS) --memory "$(MEMORY)" --mode $(KERNEL_BUILD_PROFILE) --panic $(KTEST_PANIC_STRATEGY) --root "$(ROOT)" --root-mode "$(ROOT_MODE)" $(if $(EXTRA_ROOT),--extra-root "$(EXTRA_ROOT)") $(if $(TEST),--glob "$(TEST)") $(KTEST_ARGS)

run-kernel-tests:
	python3 tools/run_kernel_tests.py run --arch $(ARCH) --cpus $(CPUS) --memory "$(MEMORY)" --mode $(KERNEL_BUILD_PROFILE) --panic $(KTEST_PANIC_STRATEGY) --root "$(ROOT)" --root-mode "$(ROOT_MODE)" $(if $(EXTRA_ROOT),--extra-root "$(EXTRA_ROOT)") $(if $(TEST),--glob "$(TEST)") $(KTEST_ARGS)

user-tests: $(USER_OUTPUTS)

# 每族只链接自己的业务与实际桥；原始汇编不再作为业务输入或构建后备。
define build-probe
build/user-$(ARCH)/user-$(1): tests/user/probes/$(1).c $(PROBE_EXTRA_$(1)) $(PROBE_COMMON) $(if $(filter $(1),$(PROBE_CLONE_FAMILIES)),tests/user/probes/clone-$(ARCH).S) tests/user/include/probe.h $(PROBE_HEADERS_$(1)) tests/user/smoke.ld Makefile
	mkdir -p "$$(@D)"
	"$(MUSL_CROSS)gcc" $(PROBE_CFLAGS) $(PROBE_ARCH_FLAGS) -Itests/user/include $(filter %.c %.S,$(PROBE_COMMON)) tests/user/probes/$(1).c $(PROBE_EXTRA_$(1)) $(if $(filter $(1),$(PROBE_CLONE_FAMILIES)),tests/user/probes/clone-$(ARCH).S) $(PROBE_LDFLAGS) -o "$$@"
endef
$(foreach family,$(PROBE_FAMILIES),$(eval $(call build-probe,$(family))))

# 现场桥按架构基线独立汇编；向量指令由真实硬件能力分派，C 禁止自动向量化。
build/user-$(ARCH)/signal-$(ARCH).o: tests/user/probes/signal-$(ARCH).S Makefile
	mkdir -p "$(@D)"
	"$(MUSL_CROSS)gcc" $(PROBE_ARCH_FLAGS) -c "$<" -o "$@"

build-guest-userland: $(call make-path,$(BUSYBOX_BUILD_DIR)/.config) $(foreach program,$(USER_TOOLS),$(call make-path,$(GUEST_USERLAND_OUTPUT_DIR)/bin/$(program))) build-contest-runner install-user-selftests
	$(MAKE) -C dependency/busybox O="$(call absolute-path,$(BUSYBOX_BUILD_DIR))" KBUILD_OUTPUT="$(call absolute-path,$(BUSYBOX_BUILD_DIR))" ARCH="$(ARCH)" CROSS_COMPILE="$(MUSL_CROSS)" CONFIG_PREFIX="$(call absolute-path,$(GUEST_USERLAND_OUTPUT_DIR))" install
	mkdir -p "$(GUEST_USERLAND_OUTPUT_DIR)/sbin"
	install -m 755 "$(RUNNER_OUTPUT)" "$(GUEST_USERLAND_OUTPUT_DIR)/sbin/contest-run"

build-user-selftests: $(SELFTEST_OUTPUTS) $(foreach program,$(TEST_GEN_PROGS_EXTENDED),$(call make-path,$(USER_SELFTEST_BUILD_DIR)/bin/$(program)))
	$(call build-std-program,user-selftests)

install-user-selftests: build-user-selftests
	@set -e; if [ -f "$(GUEST_USERLAND_OUTPUT_DIR)/test/selftests/extended-bin.txt" ]; then \
		while IFS= read -r program; do rm -f "$(GUEST_USERLAND_OUTPUT_DIR)/bin/$$program"; done < "$(GUEST_USERLAND_OUTPUT_DIR)/test/selftests/extended-bin.txt"; \
	fi
	rm -rf "$(GUEST_USERLAND_OUTPUT_DIR)/test/selftests" "$(GUEST_USERLAND_OUTPUT_DIR)/test/scripts"
	mkdir -p "$(GUEST_USERLAND_OUTPUT_DIR)/sbin" "$(GUEST_USERLAND_OUTPUT_DIR)/bin" "$(GUEST_USERLAND_OUTPUT_DIR)/test/selftests" "$(GUEST_USERLAND_OUTPUT_DIR)/test/scripts"
	@set -e; for program in $(TEST_GEN_PROGS); do \
		mkdir -p "$(GUEST_USERLAND_OUTPUT_DIR)/test/selftests/$${program%/*}"; \
		install -m 755 "$(USER_SELFTEST_BUILD_DIR)/test/selftests/$$program" "$(GUEST_USERLAND_OUTPUT_DIR)/test/selftests/$$program"; \
	done
	@set -e; for program in $(TEST_GEN_PROGS_EXTENDED); do \
		install -m 755 "$(USER_SELFTEST_BUILD_DIR)/bin/$$program" "$(GUEST_USERLAND_OUTPUT_DIR)/bin/$$program"; \
	done
	@{ :; $(foreach program,$(TEST_GEN_PROGS_EXTENDED),printf '%s\n' '$(program)';) } > "$(GUEST_USERLAND_OUTPUT_DIR)/test/selftests/extended-bin.txt"
	install -m 755 "$(USER_SELFTEST_RUNNER_OUTPUT)" "$(GUEST_USERLAND_OUTPUT_DIR)/sbin/user-selftests"
	@set -e; { :; $(foreach program,$(TEST_GEN_PROGS),printf '%s\n' '$(subst /,:,$(program))';) } > "$(GUEST_USERLAND_OUTPUT_DIR)/test/selftests/kselftest-list.txt"
	@set -e; for source in $(TEST_FILES); do \
		mode=755; [ "$${source##*/}" != no-exec ] || mode=644; \
		install -m "$$mode" "$$source" "$(GUEST_USERLAND_OUTPUT_DIR)/test/scripts/"; \
	done

run-user-selftests: build-guest-userland prepare-guest-runtime-libraries
	python3 tools/run_user_selftests.py --arch "$(ARCH)" --guest-userland-directory "$(GUEST_USERLAND_OUTPUT_DIR)" --runtime-libraries-directory "$(RUNTIME_LIBRARIES_OUTPUT_DIR)" --output-directory "$(USER_SELFTEST_ROOTFS_DIR)" --system-image "$(USER_SELFTEST_EXT4_IMAGE)" --profile "$(EXT4_PROFILE)" --size-mib "$(EXT4_SIZE_MIB)" --kernel-elf "$(USER_SELFTEST_KERNEL_ELF)" --working-disk-dir "$(USER_SELFTEST_WORKING_DISK_DIR)" --system-only --init-program /sbin/init --root-mode rw $(USER_SELFTEST_ARGS) $(USER_SELFTEST_HOST_ARGS) $(QEMU_ARGS)

$(call make-path,$(USER_SELFTEST_BUILD_DIR)/test/selftests/%): tests/user/%.c $(SELFTEST_HEADERS)
	mkdir -p "$$(dirname -- "$@")"
	"$(MUSL_CROSS)gcc" $(MUSL_CFLAGS) -pthread -Itests/user/include "$<" -o "$@"

build-linux-user-selftests:
	$(MAKE) USER_SELFTEST_BUILD_DIR="$(LINUX_USER_SELFTEST_OUTPUT_DIR)" MUSL_CROSS= MUSL_CFLAGS="$(LINUX_USER_SELFTEST_CFLAGS)" $(foreach program,$(TEST_GEN_PROGS),"$(LINUX_USER_SELFTEST_OUTPUT_DIR)/test/selftests/$(program)")

run-linux-user-selftests: build-linux-user-selftests
	@failed=0; passed=0; directory="$(call absolute-path,$(LINUX_USER_SELFTEST_OUTPUT_DIR))/test/selftests"; \
	for program in $(TEST_GEN_PROGS); do \
		(cd "$$directory/$${program%/*}" && "$$directory/$$program"); status=$$?; \
		printf 'linux-user-selftests program=%s exit=%s\n' "$$program" "$$status"; \
		case "$$status" in 0) passed=1 ;; 4) ;; *) failed=1 ;; esac; \
	done; \
	[ "$$failed" -eq 0 ] || exit 1; \
	[ "$$passed" -eq 0 ] || exit 0; \
	exit 4

$(call make-path,$(USER_SELFTEST_BUILD_DIR)/bin/libc-smoke): $(LIBC_SOURCES) $(LIBC_HEADERS)
	mkdir -p "$$(dirname -- "$@")"
	"$(MUSL_CROSS)gcc" $(MUSL_CFLAGS) -Itests/user/include $(LIBC_SOURCES) -o "$@"

$(call make-path,$(USER_SELFTEST_BUILD_DIR)/bin/network-smoke): $(NETWORK_SOURCES) $(NETWORK_HEADERS)
	mkdir -p "$$(dirname -- "$@")"
	"$(MUSL_CROSS)gcc" $(MUSL_CFLAGS) -Itests/user/include $(NETWORK_SOURCES) -o "$@"

$(call make-path,$(GUEST_USERLAND_OUTPUT_DIR)/bin/klog-read): userland/tools/klog-read.c userland/include/blossom-trace.h
	mkdir -p "$$(dirname -- "$@")"
	"$(MUSL_CROSS)gcc" $(MUSL_CFLAGS) -Iuserland/include "$<" -o "$@"

$(call make-path,$(GUEST_USERLAND_OUTPUT_DIR)/bin/ktrace-control): userland/tools/ktrace-control.c userland/include/blossom-trace.h
	mkdir -p "$$(dirname -- "$@")"
	"$(MUSL_CROSS)gcc" $(MUSL_CFLAGS) -Iuserland/include "$<" -o "$@"

$(call make-path,$(USER_SELFTEST_BUILD_DIR)/bin/observation-smoke): tests/user/abi/observation_smoke.c userland/include/blossom-trace.h
	mkdir -p "$$(dirname -- "$@")"
	"$(MUSL_CROSS)gcc" $(MUSL_CFLAGS) -Itests/user/include -Iuserland/include "$<" -o "$@"

.DELETE_ON_ERROR: $(call make-path,$(BUSYBOX_BUILD_DIR)/.config)
$(call make-path,$(BUSYBOX_BUILD_DIR)/.config): userland/busybox.config
	mkdir -p "$(BUSYBOX_BUILD_DIR)"
	cp "$<" "$@"
	$(MAKE) -C dependency/busybox O="$(call absolute-path,$(BUSYBOX_BUILD_DIR))" KBUILD_OUTPUT="$(call absolute-path,$(BUSYBOX_BUILD_DIR))" ARCH="$(ARCH)" CROSS_COMPILE="$(MUSL_CROSS)" oldconfig

clean-build-artifacts:
	cd os && CARGO_TARGET_DIR="$(call absolute-path,$(KERNEL_CARGO_TARGET_DIR))" cargo clean
	cd userland/contest-run && RUSTUP_AUTO_INSTALL=0 CARGO_TARGET_DIR="$(call absolute-path,$(RUNNER_TARGET_DIR))" cargo clean --offline
	$(MAKE) -C dependency/busybox O="$(call absolute-path,$(BUSYBOX_BUILD_DIR))" KBUILD_OUTPUT="$(call absolute-path,$(BUSYBOX_BUILD_DIR))" ARCH="$(ARCH)" CROSS_COMPILE="$(MUSL_CROSS)" clean
	rm -f $(USER_OUTPUTS) $(addsuffix .o,$(USER_OUTPUTS)) "build/user-$(ARCH)/signal-$(ARCH).o" "$(KERNEL_ELF_OUTPUT)"
	@for program in $(GUEST_BIN_PROGRAMS) busybox; do \
		rm -f "$(GUEST_USERLAND_OUTPUT_DIR)/bin/$$program"; \
	done
	rm -f "$(GUEST_USERLAND_OUTPUT_DIR)/sbin/contest-run" "$(GUEST_USERLAND_OUTPUT_DIR)/sbin/user-selftests"
	@set -e; if [ -f "$(GUEST_USERLAND_OUTPUT_DIR)/test/selftests/extended-bin.txt" ]; then \
		while IFS= read -r program; do rm -f "$(GUEST_USERLAND_OUTPUT_DIR)/bin/$$program"; done < "$(GUEST_USERLAND_OUTPUT_DIR)/test/selftests/extended-bin.txt"; \
	fi
	rm -rf "$(USER_SELFTEST_BUILD_DIR)" "$(GUEST_USERLAND_OUTPUT_DIR)/test/selftests" "$(GUEST_USERLAND_OUTPUT_DIR)/test/scripts" "$(LINUX_USER_SELFTEST_OUTPUT_DIR)/test/selftests"

help:
	@printf '%s\n' \
	  'build-kernel ARCH=riscv64|loongarch64 TARGET_PLATFORM=qemu-virt' \
	  '  Kernel only; KERNEL_BUILD_PROFILE=debug|release defaults to debug, panic=abort.' \
	  '  KERNEL_ELF_OUTPUT defaults to build/kernel/<arch>/<platform>/<profile>/kernel.elf.' \
	  '  Board combinations: riscv64/visionfive2 and loongarch64/ls2k1000la.' \
	  'build-guest-userland                Incremental BusyBox/C and both release std programs.' \
	  '  GUEST_USERLAND_OUTPUT_DIR, BUSYBOX_BUILD_DIR, MUSL_CROSS select actual inputs/outputs.' \
	  '  RUNNER_SYSROOT and RUNNER_LINKER select prepared Rust std compiler inputs.' \
	  'build-user-selftests                Build default C tests, extended helpers and user-selftests.' \
	  'install-user-selftests              Install the same sets and generate kselftest-list.txt.' \
	  '  USER_SELFTEST_BUILD_DIR is the compilation cache; installation rebuilds only current declarations.' \
	  'run-user-selftests                  Prepare a new PID1 run, wait for QEMU, read its working-disk result.' \
	  '  USER_SELFTEST_KERNEL_ELF selects an existing ordinary ELF; no kernel build dependency.' \
	  '  USER_SELFTEST_ROOTFS_DIR, USER_SELFTEST_EXT4_IMAGE and USER_SELFTEST_WORKING_DISK_DIR own outputs.' \
	  '  USER_SELFTEST_ARGS forwards --collection/--program and --program-timeout-seconds.' \
	  '  USER_SELFTEST_HOST_ARGS accepts --host-timeout-seconds (default 900); UART is saved in the work dir.' \
	  '  QEMU_ARGS passes production options; automatic runs reject reuse (use fresh or refresh-system).' \
	  '  The Host returns Guest 0/1/2/4; Host errors return 2. GNU Make wraps any nonzero recipe as 2.' \
	  'build-linux-user-selftests          Build only the same default C programs with native gcc.' \
	  'run-linux-user-selftests            Run every native program in its collection cwd; preserve TAP/exit.' \
	  '  LINUX_USER_SELFTEST_OUTPUT_DIR defaults to build/user-selftests-linux; not target-ISA evidence.' \
	  '  LINUX_USER_SELFTEST_CFLAGS selects native flags; no kernel, BusyBox, runner or PID1 execution.' \
	  'prepare-guest-runtime-libraries     Copy checked local libc sources to RUNTIME_LIBRARIES_OUTPUT_DIR.' \
	  'assemble-root-filesystem            Build userland/libraries, then replace ROOTFS_OUTPUT_DIR.' \
	  '  ROOTFS_INIT_CONFIG optionally selects an ordinary inittab file.' \
	  'create-ext4-image                   Only consume ROOTFS_TREE_DIR; write SYSTEM_ROOT_EXT4_IMAGE.' \
	  'build-qemu-artifacts ARCH=...        Fixed release/abort QEMU kernel and one single-root disk.' \
	  'all                                Export kernel-rv/kernel-la/disk.img/disk-la.img.' \
	  'build-board-dtb                     Build the selected board DTB from in-repository sources.' \
	  '  BOARD_DTB_OUTPUT selects the output; HOSTCC and DTC select installed host tools.' \
	  'package-board-boot-images           Build board kernel and generate offline boot files.' \
	  '  BOARD_BOOT_IMAGES_OUTPUT_DIR, BOARD_ROOT_EXT4_IMAGE, BOARD_OS_DTB,' \
	  '  BOARD_ROOT_DEVICE, BOARD_ROOT_MODE and BOARD_INIT_PROGRAM map directly to packaging.' \
	  'run-qemu ARCH=...                   Run existing inputs; QEMU_ARGS passes CLI options unchanged.' \
	  '  tools/run_qemu.py --help lists session, disk, network and debugger options.' \
	  'clean-qemu-working-disks ARCH=...   Remove inactive root.raw/contest.raw under their lock.' \
	  '  QEMU_ARGS may select --working-disk-dir; keep sources, directory and unrelated files.' \
	  'user-tests                         Build the existing raw user ELF programs.' \
	  'list-kernel-tests / run-kernel-tests QEMU only; KERNEL_BUILD_PROFILE, KTEST_PANIC_STRATEGY.' \
	  '  build-kernel-test-elf is the internal build target used when no --kernel is supplied.' \
	  '  TEST selects a glob; KTEST_ARGS passes other Host options; CPUS/MEMORY are VM-only.' \
	  'clean-build-artifacts              Rebuildable compiler outputs only; keep disks and reports.'
