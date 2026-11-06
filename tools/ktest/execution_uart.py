"""内核执行日志（`klog`）专项测试的宿主机串口观测与背压核验夹具。

本模块专门配合三个内核执行服务日志用例（无分配记录标识、UART 打印机背压阻塞、多核定时器与工作队列混合汇聚）：
通用 KTAP 解析器只负责协议状态流转，而本模块的 `ExecutionUART` 负责从原始串口字节流中提取 `KLOG` 业务行与
`# diagnostic: execution-*` 环形缓冲区快照诊断行，并在用例结束时逐字节比对序号连续性、丢包计数及完整报文。
"""

import re

from .catalog import require

NUMBER = rb"([0-9]{1,20})"
RECORD = re.compile(rb"  # diagnostic: execution-record record index=" + NUMBER
                    + rb" seq=" + NUMBER + rb" level=(INFO|WARN|DEBUG) ns=" + NUMBER
                    + rb" cpu=" + NUMBER + rb" task=" + NUMBER
                    + rb" truncated=([01])(?: body_hex=([0-9a-f]{2,1024}))?\n")
MIXED = re.compile(rb"  # diagnostic: execution-log record seq=" + NUMBER
                   + rb" ns=" + NUMBER + rb" cpu=" + NUMBER + rb" task=" + NUMBER
                   + rb" source=(timer|work) slot=" + NUMBER + rb" round=" + NUMBER + rb"\n")
PRESSURE = re.compile(rb"  # diagnostic: execution-backpressure record index=" + NUMBER
                      + rb" seq=" + NUMBER + rb" level=(INFO) ns=" + NUMBER
                      + rb" cpu=" + NUMBER + rb" task=" + NUMBER
                      + rb" truncated=([01]) lost=" + NUMBER + rb"\n")
FILL = re.compile(rb"  # diagnostic: execution-backpressure fill-bytes=" + NUMBER + rb"\n")


def record_line(row):
    """根据诊断行采集到的环形缓冲区记录元数据与期望消息体，拼装预期的 `KLOG ...\\n` 完整字节行。"""
    task = str(row["task"]) if row["task"] else "none"
    header = (f"KLOG seq={row['seq']} level={row['level']} ns={row['ns']} "
              f"cpu={row['cpu']} task={task} truncated={row['truncated']} "
              f"lost={row['lost']} ").encode("ascii")
    return header + row["body"] + b"\n"


class ExecutionUART:
    """针对三个 `execution_services` 内核日志测试用例的宿主机侧串口观测器。"""

    def __init__(self, cpus, directory):
        """初始化观测器，保存当前虚拟机的 CPU 核心数与原始证据落盘目录。"""
        self.cpus = cpus
        self.directory = directory
        self.kind = None
        self.case = None
        self.body = False
        self.lines = []
        self.records = {}
        self.blank_bytes = 0
        self.fill_bytes = None
        self.captured = False

    def begin(self, case):
        """在每个测试用例开始时重置观测状态，并根据用例全名判断是否激活对应的日志观测模式。"""
        kinds = {
            "os::ktest::execution_services::klog::records::allocation_free_record_identity": "records",
            "os::ktest::execution_services::klog::uart::socket_printer_backpressure": "backpressure",
            "os::ktest::execution_services::mixed::cases::timer_work_task_log_convergence": "mixed",
        }
        self.kind = kinds.get(case["name"])
        self.case = case["case"]
        self.body = False
        self.lines = []
        self.records = {}
        self.blank_bytes = 0
        self.fill_bytes = None
        self.captured = False

    def observe(self, raw):
        """在用例 `body` 阶段逐行旁路观测原始串口字节，收集 `KLOG` 输出行与 `# diagnostic:` 元数据。"""
        if self.kind is None or not self.body:
            return
        limit = 5 if self.kind == "records" else (65 if self.kind == "backpressure" else 16 * self.cpus)
        # 收集非背压场景下直接打印到串口的 KLOG 行（背压场景走独立的 capture 原始采集流程）
        if raw.startswith(b"KLOG "):
            require(self.kind != "backpressure", "backpressure bytes outside its opaque fixture")
            require(len(self.lines) < limit and len(raw) <= 769, "unexpected execution log record")
            self.lines.append(raw)
        pattern = RECORD if self.kind == "records" else (PRESSURE if self.kind == "backpressure" else MIXED)
        match = pattern.fullmatch(raw)
        if match:
            # 解析内核测试在独立诊断行中导出的环形缓冲区记录元数据，用于事后与实际串口 KLOG 输出逐字节对账
            if self.kind == "mixed":
                seq, ns, cpu, task, source, slot, round_number = match.groups()
                slot, round_number = int(slot), int(round_number)
                require(slot < self.cpus and round_number < 8, "mixed log producer outside fixture")
                key = (slot, round_number, source)
                row = dict(seq=int(seq), ns=int(ns), cpu=int(cpu), task=int(task), level="INFO",
                           truncated=0, lost=0,
                           body=f"X28 {source.decode('ascii')} slot={slot} round={round_number}".encode("ascii"))
                require(row["cpu"] == slot and (source != b"work" or row["task"] != 0),
                        "mixed log producer identity mismatch")
            else:
                values = match.groups()
                key, seq, level, ns, cpu, task, truncated = values[:7]
                key = int(key)
                require(key < limit, "execution log index outside fixture")
                row = dict(seq=int(seq), ns=int(ns), cpu=int(cpu), task=int(task),
                           level=level.decode("ascii"), truncated=int(truncated), lost=0)
                if self.kind == "records":
                    bodies = (b"X25 ordinary value=37", "界".encode("utf-8") * 170,
                              None, None, b"X25 precise timer")
                    require(row["level"] == ("INFO", "WARN", "DEBUG", "DEBUG", "INFO")[key]
                            and row["truncated"] == int(key == 1), "record metadata mismatch")
                    body_hex = values[7]
                    if key in (2, 3):
                        require(body_hex is not None and len(body_hex) % 2 == 0,
                                "locked record requires complete independent ring body")
                        row["body"] = bytes.fromhex(body_hex.decode("ascii"))
                        row["body"].decode("utf-8", errors="strict")
                    else:
                        require(body_hex is None, "unexpected dynamic business body")
                        row["body"] = bodies[key]
                else:
                    row["lost"] = int(values[7])
                    require(row["truncated"] == 0 and row["lost"] == (8 if key == 1 else 0),
                            "backpressure lost/truncated metadata mismatch")
                    row["body"] = b"X26 partial " + b"P" * 450 if key == 0 else f"X26 record={key + 7}".encode("ascii")
            require(0 < row["seq"] < 1 << 64 and 0 <= row["ns"] < 1 << 64
                    and 0 <= row["task"] < 1 << 64 and row["cpu"] < self.cpus,
                    "execution log metadata outside native bounds")
            require(key not in self.records, "duplicate execution log metadata")
            self.records[key] = row
        elif raw.startswith((b"  # diagnostic: execution-record record ",
                             b"  # diagnostic: execution-log record ",
                             b"  # diagnostic: execution-backpressure record ")):
            require(False, "malformed or wrong execution fixture metadata")
        fill = FILL.fullmatch(raw)
        if fill:
            require(self.kind == "backpressure" and self.fill_bytes is None,
                    "duplicate or misplaced UART fill evidence")
            self.fill_bytes = int(fill[1])

    def capture(self, channel, declaration, deadline):
        """处理背压（`backpressure`）用例的串口暂停与恢复采集。

        先向内核发送 `H` 触发写端填充串口 FIFO 直至首次出现背压阻塞，按声明等待指定时长让内核完成丢包与恢复逻辑，
        随后直接从底层通道读取空白填充换行符及全部 65 条 `KLOG` 记录并写入 `.raw` 证据文件，最后回发 `KTEST IO OK` 确认。
        """
        require(self.kind == "backpressure" and not self.captured and self.body,
                "opaque UART capture outside backpressure fixture")
        require(declaration["direction"] == "pause" and declaration["length"] == 0
                and declaration["digest"] == "00000000" and 3000 <= declaration["delay_ms"],
                "invalid backpressure hold barrier")
        require(declaration["case"] == self.case, "backpressure case mismatch")
        acknowledgement = f"KTEST IO OK {self.case}\n".encode("ascii")
        require(len(acknowledgement) <= 16, "backpressure ACK exceeds FIFO bound")
        # 发送 'H' 通知客体开始向受阻的串口写入，宿主机暂停读取以制造真实的 UART 发送缓冲区满（背压）条件
        channel.send(b"H", deadline)
        channel.delay(declaration["delay_ms"] / 1000, deadline)
        # 暂停结束后放开读取，消费前导填充换行符并收齐 65 条业务日志记录
        with (self.directory / f"execution-uart-{self.case}.raw").open("xb") as output:
            while len(self.lines) < 65:
                channel.check(deadline)
                newline = channel.buffer.find(b"\n")
                if newline < 0:
                    require(len(channel.buffer) <= 769, "opaque execution record exceeds native bound")
                    channel.delay(0.005, deadline)
                    channel.buffer.extend(channel.receive(deadline, 64))
                    continue
                require(newline < 769, "opaque execution record exceeds native bound")
                raw = bytes(channel.buffer[:newline + 1])
                del channel.buffer[:newline + 1]
                output.write(raw)
                if not self.lines and raw == b"\n":
                    self.blank_bytes += 1
                    require(self.blank_bytes < 1024 * 1024, "UART fill reached its failure bound")
                else:
                    self.lines.append(raw)
        self.captured = True
        channel.send(acknowledgement, deadline)

    def finish(self, status):
        """在客体报告用例 `Pass` 后，核对采集到的日志条数、序号连续性、背压空白填充字节数及完整输出内容。"""
        if self.kind is None:
            return
        self.body = False
        if status != "Pass":
            return
        count = 5 if self.kind == "records" else (65 if self.kind == "backpressure" else 16 * self.cpus)
        require(len(self.lines) == count and len(self.records) == count,
                "execution log records or metadata missing")
        if self.kind == "mixed":
            keys = {(cpu, round_number, source) for cpu in range(self.cpus)
                    for round_number in range(8) for source in (b"timer", b"work")}
            require(self.records.keys() == keys, "mixed log producer/round gap")
            rows = sorted(self.records.values(), key=lambda row: row["seq"])
        else:
            require(self.records.keys() == set(range(count)), "execution log index gap")
            rows = [self.records[index] for index in range(count)]
        first = rows[0]["seq"]
        for index, row in enumerate(rows):
            offset = index + 8 if self.kind == "backpressure" and index else index
            require(row["seq"] == first + offset, "execution log sequence gap or reordering")
        if self.kind == "backpressure":
            require(self.captured and self.fill_bytes == self.blank_bytes and 0 < self.blank_bytes < 1024 * 1024,
                    "UART fill does not match its actual opaque prefix")
        # 生成期望的完整 KLOG 字节序列落盘留存，并与串口实际收到的字节流做精确比对
        expected = b"".join(record_line(row) for row in rows)
        (self.directory / f"execution-uart-{self.case}.expected").write_bytes(expected)
        require(b"".join(self.lines) == expected, "execution UART full payload mismatch")
