"""宿主 boot 回归；运行 python3 -m unittest discover -s tools/ktest。"""

import contextlib
import io
import os
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest
from unittest import mock
import zlib

from host_helpers import case_record
from kernel_elf import read_kernel_elf
from ktest.boot import validate_test_kernel_layout, verify_trap
from ktest.catalog import HostError
from package_board_boot_images import LEGACY_HEADER, U32_LIMIT, make_legacy_header, main, package_board_boot_images


def kernel_image(architecture, target_platform="qemu-virt", empty_unwind_header=False):
    """构造含实际 LOAD、普通生产节及公开符号的 ELF 字节，不充当客体链接或执行证明。"""
    rv = architecture == "riscv64"
    entry = 0x80200000 if rv else 0x9000000090000000
    text = 0xFFFFFFFF80200000 if rv else entry
    physical = 0x80400000 if rv else 0x90000000
    first = 0x2000 if rv else 0x1000
    loads = ([(5, 0x1000, entry, entry, 32, 32)] if rv else []) + [
        (5, first, text, physical, 32, 32),
        (4, first + 0x1000, text + 0x1000, physical + 0x1000, 32, 32),
        (6, first + 0x2000, text + 0x2000, physical + 0x2000, 32, 128)]
    sections = [
        (b".text", 1, 6, text, first, 32),
        (b".rodata", 1, 2, text + 0x1000, first + 0x1000, 16),
        (b".data", 1, 3, text + 0x2000, first + 0x2000, 32),
        (b".boot", 1 if rv else 8, 6 if rv else 3,
         entry if rv else text + 0x2020, 0x1000 if rv else first + 0x2020, 32),
        (b".bss", 8, 3, text + 0x2040, first + 0x2040, 32)]
    if empty_unwind_header:
        sections.append((b".eh_frame_hdr", 1, 2, text + 0x1010, first + 0x1010, 0))
    symbol_values = {"__blossom_platform": {"qemu-virt": 0, "visionfive2": 1, "ls2k1000la": 2}[target_platform],
                     "__kernel_phys_start": entry if rv else physical,
                     "__kernel_virt_start": 0xFFFFFFFF80000000 if rv else entry,
                     "__kernel_phys_end": physical + 0x2080,
                     "__kernel_virt_end": text + 0x2080,
                     "__direct_base": 0xFFFFFFFF80000000 if rv else 0x9000000000000000,
                     "__phys_ram_base": entry if rv else physical,
                     "ordinary_public_symbol": text + 4}
    symbol_strings = b"\0" + b"\0".join(name.encode() for name in symbol_values) + b"\0"
    sections += [(b".strtab", 3, 0, 0, 0x5300, len(symbol_strings)),
                 (b".symtab", 2, 0, 0, 0x5100, (len(symbol_values) + 1) * 24)]
    names = b"\0" + b"\0".join(section[0] for section in sections) + b"\0.shstrtab\0"
    image = bytearray(0x5800)
    struct.pack_into("<16sHHIQQQIHHHHHH", image, 0, b"\x7fELF\x02\x01\x01", 2,
                     243 if rv else 258, 1, entry, 64, 0x5400, 5 if rv else 0x43,
                     64, 56, len(loads), 64, len(sections) + 2, len(sections) + 1)
    for index, (flags, offset, address, pa, file_size, memory_size) in enumerate(loads):
        struct.pack_into("<IIQQQQQQ", image, 64 + index * 56, 1, flags, offset,
                         address, pa, file_size, memory_size, 0x1000)
    for index, (name, kind, flags, address, offset, length) in enumerate(sections, 1):
        link = next(i for i, section in enumerate(sections, 1) if section[0] == b".strtab") if kind == 2 else 0
        struct.pack_into("<IIQQQQIIQQ", image, 0x5400 + index * 64,
                         names.index(name), kind, flags, address, offset, length, link, 0, 1,
                         24 if kind == 2 else 0)
    struct.pack_into("<IIQQQQIIQQ", image, 0x5400 + (len(sections) + 1) * 64,
                     names.index(b".shstrtab"), 3, 0, 0, 0x5000, len(names), 0, 0, 1, 0)
    image[0x5000:0x5000 + len(names)] = names
    image[0x5300:0x5300 + len(symbol_strings)] = symbol_strings
    for index, (name, value) in enumerate(symbol_values.items(), 1):
        struct.pack_into("<IBBHQQ", image, 0x5100 + index * 24,
                         symbol_strings.index(name.encode()), 0x10, 0, 0xFFF1, value, 0)
    for index, (_, offset, _, _, file_size, _) in enumerate(loads, 1):
        image[offset:offset + file_size] = bytes([index]) * file_size
    return image, entry, loads


class BootELFTests(unittest.TestCase):
    """实际装载格式的边界检查；合成 ELF 不充当客体执行或链接证明。"""

    def test_load_contract_rejects_corruption_and_allows_empty_optional_header(self):
        """两架构拒错入口、映射、权限和表界，合法零长 eh_frame_hdr 不阻断发现。"""
        for architecture in ("riscv64", "loongarch64"):
            with self.subTest(architecture=architecture), tempfile.TemporaryDirectory() as temporary:
                image, entry, loads = kernel_image(architecture, empty_unwind_header=True)
                kernel = Path(temporary) / "kernel.elf"
                kernel.write_bytes(image)
                layout = validate_test_kernel_layout(kernel, architecture)
                self.assertEqual(layout["entry"], entry)
                self.assertEqual(layout["sections"][".eh_frame_hdr"]["size"], 0)
                self.assertEqual(layout["sections"][".bss"]["flags"], 6)
                corruptions = [
                    ("<H", 16, 3), ("<H", 18, 258 if architecture == "riscv64" else 243),
                    ("<Q", 24, entry + 4), ("<Q", 40, 1 << 63), ("<I", 64 + 4, 7),
                    ("<Q", 64 + 24, loads[0][3] + 0x1000),
                    ("<Q", 64 + 32, 64), ("<Q", 0x5400 + 64 + 8, 7)]
                for fmt, offset, value in corruptions:
                    with self.subTest(offset=offset, value=value):
                        broken = bytearray(image)
                        struct.pack_into(fmt, broken, offset, value)
                        kernel.write_bytes(broken)
                        with self.assertRaises(HostError):
                            validate_test_kernel_layout(kernel, architecture)
                kernel.write_bytes(image[:64])
                with self.assertRaises(HostError):
                    validate_test_kernel_layout(kernel, architecture)

    def test_common_reader_accepts_four_production_platforms_without_test_sections(self):
        """共同读取器保留四平台事实和公开符号，不要求测试或展开节。"""
        for architecture, platform in (("riscv64", "qemu-virt"), ("loongarch64", "qemu-virt"),
                                       ("riscv64", "visionfive2"), ("loongarch64", "ls2k1000la")):
            with self.subTest(architecture=architecture, platform=platform), tempfile.TemporaryDirectory() as temporary:
                image, entry, loads = kernel_image(architecture, platform)
                kernel = Path(temporary) / "kernel.elf"
                kernel.write_bytes(image)
                layout = read_kernel_elf(kernel, architecture)
                self.assertEqual(layout["entry"], entry)
                self.assertEqual(len(layout["loads"]), len(loads))
                self.assertEqual(layout["loads"][-1]["memory_size"], 128)
                self.assertEqual(layout["symbols"]["ordinary_public_symbol"], loads[-3][2] + 4)
                self.assertNotIn(".ktest.suites", layout["sections"])
                self.assertNotIn(".eh_frame", layout["sections"])
                self.assertNotIn(".eh_frame_hdr", layout["sections"])

    def test_common_reader_preserves_format_and_io_errors(self):
        """格式错保留 ValueError，文件不存在保留 OSError，测试边界转换为 HostError。"""
        with tempfile.TemporaryDirectory() as temporary:
            kernel = Path(temporary) / "kernel.elf"
            with self.assertRaises(OSError):
                read_kernel_elf(kernel, "riscv64")
            with self.assertRaises(HostError):
                validate_test_kernel_layout(kernel, "riscv64")
            image, _, _ = kernel_image("riscv64")
            corruptions = [("<H", 16, 3), ("<Q", 40, 1 << 63), ("<Q", 64 + 32, 1 << 63),
                           ("<Q", 64 + 24, (1 << 64) - 16), ("<Q", 64 + 48, 3),
                           ("<I", 0x5400 + 64, 1 << 31)]
            for fmt, offset, value in corruptions:
                broken = bytearray(image)
                struct.pack_into(fmt, broken, offset, value)
                kernel.write_bytes(broken)
                with self.subTest(offset=offset), self.assertRaises(ValueError):
                    read_kernel_elf(kernel, "riscv64")


def write_root_payload(directory):
    """构造包装器可识别的最小 ext4 头及载荷字节，不模拟文件系统挂载。"""
    target = directory / "root.ext4"
    payload = bytearray(4096)
    payload[1080:1082] = b"\x53\xef"
    target.write_bytes(payload)
    return target, payload


def write_os_tree(directory, with_chosen=True):
    """通过 dtc 生成可消费的树，已有 initrd 属性故意保留与本次包装无关的历史范围。"""
    chosen = """
        chosen {
            bootargs = "console=ttyS0";
            linux,initrd-start = <0x0 0x12340000>;
            linux,initrd-end = <0x0 0x12341000>;
        };
    """ if with_chosen else ""
    source = f"""/dts-v1/;
    / {{
        #address-cells = <2>;
        #size-cells = <2>;
        compatible = "blossom,packaging-test";
        {chosen}
        memory@80000000 {{
            device_type = "memory";
            reg = <0x0 0x80000000 0x0 0x20000000>;
        }};
    }};
    """
    target = directory / "OS tree.dtb"
    subprocess.run(["dtc", "-I", "dts", "-O", "dtb", "-o", str(target)],
                   input=source, text=True, capture_output=True, check=True)
    return target


def read_fdt_address(path, property_name):
    """让 fdtget 实际解析输出树的 chosen 地址，按大端 cell 顺序还原物理值。"""
    result = subprocess.run(["fdtget", "-t", "x", str(path), "/chosen", property_name],
                            text=True, capture_output=True, check=True)
    return int("".join(f"{int(cell, 16):08x}" for cell in result.stdout.split()), 16)


def read_boot_environment(directory):
    """读取可交给固件的环境变量，忽略不影响执行语义的说明文字。"""
    return dict(line.split("=", 1) for line in (directory / "boot.env").read_text().splitlines()
                if line and not line.startswith("#"))


class BoardPackagingTests(unittest.TestCase):
    """验证离线文件内容、发布边界和正式 CLI，不把生成物当作实板运行证据。"""

    def test_production_images_crc_memory_spans_and_optional_inputs(self):
        """两板最终树指向容器实际载荷，固件命令地址与物理装载范围分别满足各自协议。"""
        for architecture, platform in (("riscv64", "visionfive2"), ("loongarch64", "ls2k1000la")):
            with self.subTest(architecture=architecture), tempfile.TemporaryDirectory() as temporary:
                directory = Path(temporary)
                kernel = directory / "input kernel.elf"
                image, entry, loads = kernel_image(architecture, platform)
                kernel.write_bytes(image)
                output = directory / "boot images"
                direct_base = 0 if architecture == "riscv64" else 0x9000000000000000
                package_board_boot_images(architecture, platform, kernel, output)
                self.assertEqual((output / "kernel.elf").read_bytes(), image)
                raw = (output / "kernel.bin").read_bytes()
                self.assertEqual(len(raw), loads[-1][3] + loads[-1][4] - loads[0][3])
                for _, offset, _, physical, file_size, _ in loads:
                    start = physical - loads[0][3]
                    self.assertEqual(raw[start:start + file_size], image[offset:offset + file_size])
                container = (output / "kernel.uImage").read_bytes()
                fields = LEGACY_HEADER.unpack(container[:LEGACY_HEADER.size])
                self.assertEqual(fields[0], 0x27051956)
                self.assertEqual(fields[3:7], (len(raw), loads[0][3], loads[0][3], zlib.crc32(raw)))
                self.assertEqual(fields[7:11], (5, 26 if architecture == "riscv64" else 27, 2, 0))
                self.assertFalse(fields[11].rstrip(b"\0").startswith(b"Linux-"))
                self.assertEqual(fields[1], zlib.crc32(container[:4] + bytes(4) + container[8:LEGACY_HEADER.size]))
                self.assertEqual(container[LEGACY_HEADER.size:], raw)
                rows = read_boot_environment(output)
                self.assertNotIn("blossom_boot", rows)
                self.assertNotIn("bootargs", rows)
                memory_end = loads[-1][3] + loads[-1][5]
                self.assertEqual(int(rows["kernel_mem_end"], 16), memory_end)
                self.assertEqual(int(rows["kernel_elf_entry"], 16), entry)
                self.assertEqual(int(rows["kernel_load"], 16), fields[4])
                self.assertEqual(int(rows["kernel_entry"], 16), fields[5])
                kernel_address = int(rows["kernel_addr_r"], 16)
                self.assertEqual(kernel_address & ~(U32_LIMIT - 1), direct_base)
                self.assertLessEqual(memory_end, kernel_address - direct_base)
                dtb = write_os_tree(directory)
                original_dtb = dtb.read_bytes()
                package_board_boot_images(architecture, platform, kernel, output, os_dtb_path=dtb)
                rows = read_boot_environment(output)
                self.assertNotIn("blossom_boot", rows)
                self.assertNotIn("bootargs", rows)
                package_board_boot_images(architecture, platform, kernel, output,
                                          os_dtb_path=dtb, root_device="/dev/sda1")
                rows = read_boot_environment(output)
                command = rows["blossom_boot"].split()
                self.assertEqual(command, ["bootm", rows["kernel_addr_r"]] +
                                 (["-", rows["fdt_addr_r"]] if architecture == "riscv64" else []))
                self.assertFalse((output / "root.uInitrd").exists())
                self.assertEqual(read_fdt_address(output / "board.dtb", "linux,initrd-start"), 0x12340000)
                self.assertEqual(read_fdt_address(output / "board.dtb", "linux,initrd-end"), 0x12341000)
                root, root_bytes = write_root_payload(directory)
                package_board_boot_images(architecture, platform, kernel, output,
                                          root, dtb, "/dev/ram0")
                rows = read_boot_environment(output)
                bootargs = rows["bootargs"].split()
                self.assertEqual(dict(word.split("=", 1) for word in bootargs if "=" in word),
                                 {"root": "/dev/ram0", "rootdir": "/", "init": "/sbin/init"})
                self.assertIn("ro", bootargs)
                self.assertNotIn("rw", bootargs)
                self.assertEqual(rows["blossom_boot"].split(),
                                 ["bootm", rows["kernel_addr_r"], rows["ramdisk_addr_r"]] +
                                 ([rows["fdt_addr_r"]] if architecture == "riscv64" else []))
                addresses = [int(rows[name], 16) for name in ("kernel_addr_r", "ramdisk_addr_r", "fdt_addr_r")]
                for address in addresses:
                    self.assertEqual(address & ~(U32_LIMIT - 1), direct_base)
                kernel_physical, ramdisk_physical, fdt_physical = [address - direct_base for address in addresses]
                self.assertLessEqual(memory_end, kernel_physical)
                self.assertLessEqual(kernel_physical + len(container), ramdisk_physical)
                initrd = (output / "root.uInitrd").read_bytes()
                initrd_fields = LEGACY_HEADER.unpack(initrd[:LEGACY_HEADER.size])
                self.assertEqual(initrd_fields[3], len(root_bytes))
                self.assertEqual(initrd_fields[6], zlib.crc32(root_bytes))
                self.assertEqual(initrd_fields[7:11], (5, 26 if architecture == "riscv64" else 27, 3, 0))
                self.assertEqual(initrd_fields[1], zlib.crc32(initrd[:4] + bytes(4) + initrd[8:LEGACY_HEADER.size]))
                start = read_fdt_address(output / "board.dtb", "linux,initrd-start")
                end = read_fdt_address(output / "board.dtb", "linux,initrd-end")
                self.assertEqual(start, ramdisk_physical + LEGACY_HEADER.size)
                self.assertEqual(end, ramdisk_physical + len(initrd))
                self.assertEqual(initrd_fields[4:6], (start, start))
                self.assertEqual((int(rows["initrd_start"], 16), int(rows["initrd_end"], 16)), (start, end))
                self.assertEqual(initrd[start - ramdisk_physical:end - ramdisk_physical], root_bytes)
                self.assertLessEqual(end, fdt_physical)
                padded_dtb = (output / "board.dtb").read_bytes()
                self.assertEqual(struct.unpack_from(">I", padded_dtb, 4)[0], len(padded_dtb))
                self.assertEqual(int(rows["fdt_size"], 16), len(padded_dtb))
                header = struct.unpack_from(">10I", padded_dtb)
                used_end = max(header[2] + header[9], header[3] + header[8])
                self.assertLessEqual(used_end + 64 * 1024, len(padded_dtb))
                if architecture == "loongarch64":
                    self.assertEqual(rows["fdt_addr"], rows["fdt_addr_r"])
                self.assertEqual(kernel.read_bytes(), image)
                self.assertEqual(root.read_bytes(), root_bytes)
                self.assertEqual(dtb.read_bytes(), original_dtb)
                package_board_boot_images(architecture, platform, kernel, output, root_device="/dev/sda1")
                self.assertNotIn("blossom_boot", read_boot_environment(output))
                self.assertFalse((output / "root.uInitrd").exists())
                self.assertFalse((output / "board.dtb").exists())

    def test_ram_root_creates_missing_chosen_node(self):
        """无 chosen 的真实树由 fdtput 创建节点，地址覆盖完整的 RAM 根载荷。"""
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            kernel = directory / "kernel.elf"
            image, _, _ = kernel_image("riscv64", "visionfive2")
            kernel.write_bytes(image)
            dtb = write_os_tree(directory, with_chosen=False)
            original_dtb = dtb.read_bytes()
            root, root_bytes = write_root_payload(directory)
            output = directory / "output"
            package_board_boot_images("riscv64", "visionfive2", kernel, output, root, dtb)
            rows = read_boot_environment(output)
            self.assertNotIn("blossom_boot", rows)
            start = read_fdt_address(output / "board.dtb", "linux,initrd-start")
            end = read_fdt_address(output / "board.dtb", "linux,initrd-end")
            payload_offset = start - int(rows["ramdisk_addr_r"], 16)
            self.assertEqual(payload_offset, LEGACY_HEADER.size)
            self.assertEqual((output / "root.uInitrd").read_bytes()[payload_offset:payload_offset + end - start],
                             root_bytes)
            self.assertEqual(dtb.read_bytes(), original_dtb)

    def test_failed_packaging_leaves_published_files_unchanged(self):
        """输入或真实 FDT 工具失败不发表半成品，CLI 返回非零且源输入保持不变。"""
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            kernel = directory / "kernel.elf"
            image, _, _ = kernel_image("riscv64", "visionfive2")
            kernel.write_bytes(image)
            dtb = write_os_tree(directory)
            root, root_bytes = write_root_payload(directory)
            output = directory / "output"
            package_board_boot_images("riscv64", "visionfive2", kernel, output, root, dtb, "/dev/ram0")
            published = {path.name: path.read_bytes() for path in output.iterdir()}
            inputs = {path: path.read_bytes() for path in (kernel, root, dtb)}
            invalid_input = directory / "invalid-input"
            invalid_input.write_bytes(b"invalid")
            for options in ({"root_device": "/dev/ram0"}, {"root_device": "/dev/sda1;bad"},
                            {"root_device": "/dev/sda1", "init_program": "/sbin/init bad"},
                            {"root_ext4_image_path": invalid_input}, {"os_dtb_path": invalid_input}):
                with self.subTest(options=options), self.assertRaises(ValueError):
                    package_board_boot_images("riscv64", "visionfive2", kernel, output, **options)
                self.assertEqual({path.name: path.read_bytes() for path in output.iterdir()}, published)
            broken_dtb = directory / "broken tree.dtb"
            broken = bytearray(dtb.read_bytes())
            struct.pack_into(">I", broken, 8, len(broken) + 4)
            broken_dtb.write_bytes(broken)
            arguments = ["--arch", "riscv64", "--target-platform", "visionfive2",
                         "--kernel-elf", str(kernel), "--output-directory", str(output),
                         "--root-ext4-image", str(root), "--root-device", "/dev/ram0", "--os-dtb"]
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                main(arguments + [str(broken_dtb)])
            self.assertEqual(error.exception.code, 1)
            self.assertEqual(broken_dtb.read_bytes(), broken)
            self.assertEqual({path.name: path.read_bytes() for path in output.iterdir()}, published)
            with mock.patch.dict(os.environ, {"PATH": ""}):
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                    main(arguments + [str(dtb)])
            self.assertEqual(error.exception.code, 1)
            self.assertEqual({path.name: path.read_bytes() for path in output.iterdir()}, published)
            self.assertEqual({path: path.read_bytes() for path in inputs}, inputs)
            with self.assertRaises(ValueError):
                package_board_boot_images("loongarch64", "ls2k1000la", kernel, output)
            with self.assertRaises(ValueError):
                make_legacy_header(1, 0, 26, 2, U32_LIMIT, 0, 0)


class BootTrapTests(unittest.TestCase):
    """真实故障事件之后必须有匹配架构的完整寄存器现场。"""

    def test_boot_fatal_requires_actual_postevent_register_frame(self):
        """正确kind不能洗掉缺现场、零PC/SP、错cause或只在fatal前出现的业务现场。"""
        frames = (
            ("riscv64", "illegal_instruction", "illegal",
             "synchronous trap pc=0xffffffff802f317e sp=0xffffffff80593760 cause=0x2 bad=0x0", "0x2", "0x3"),
            ("loongarch64", "illegal_instruction", "illegal",
             "synchronous trap pc=0x90000000900a4690 sp=0x9000000090604720 cause=0xd0000 bad=0x90000000900a4690", "0xd0000", "0xc0000"),
            ("loongarch64", "unmapped_refill", "refill",
             "refill trap pc=0x90000000900a46b4 sp=0x9000000090604720 cause=0x1 bad=0x1000", "0x1", "0x0"),
        )
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            for arch, name, fatal, frame, cause, wrong_cause in frames:
                case = case_record(cid=7, name="os::ktest::boot_cases::fatal::traps::" + name)
                event = f"  # KTEST fatal 7 {fatal}\n"
                line = "  # diagnostic: " + frame + "\n"
                (directory / "uart.log").write_text(event + line, encoding="utf-8")
                verify_trap(arch, case, directory, fatal)
                for bad in ("", line + line, line.replace("cause=" + cause, "cause=" + wrong_cause),
                            line.replace("pc=" + frame.split("pc=")[1].split()[0], "pc=0x0"),
                            line.replace("sp=" + frame.split("sp=")[1].split()[0], "sp=0x0")):
                    (directory / "uart.log").write_text(line + event + bad, encoding="utf-8")
                    with self.subTest(arch=arch, name=name, bad=bad), self.assertRaises(HostError):
                        verify_trap(arch, case, directory, fatal)


if __name__ == "__main__":
    unittest.main()
