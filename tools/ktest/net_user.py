"""ktest 用户态网络（net-user）宿主端对端消费者与流量记录器。

本模块在 QEMU 启动前在宿主机 127.0.0.1 上绑定动态分配的 TCP 与 UDP 端口，
并通过内核启动参数将端口告知客体。在测试用例执行期间，根据用例标签
（network-nic-tcp、network-nic-udp、network-nic-native）启动对应的后台线程，
在用例绝对截止时刻内完成收发校验：
- TCP：将收发双向原始字节流分别落盘为 `.raw` 文件，并校验固定长度与内容图样；
- UDP：将每次 recvfrom/sendto 的报文边界、两端地址与十六进制内容记录到 `peer.json`。
宿主套接字一直保持到 QEMU 进程退出并完成回收后才关闭。
"""

import json
import os
import select
import socket
import threading
import time

from .catalog import HostError, require


class PeerStopped(Exception):
    """表示宿主主线程已通过取消管道通知后台网络线程停止下一次套接字操作。"""


class NetUserPeer:
    """管理单台 QEMU 虚拟机独占的宿主机环回 TCP/UDP 端点与按用例运行的后台服务线程。"""

    def __init__(self, directory):
        """在 QEMU 启动前绑定动态非阻塞 TCP/UDP 环回端口并初始化 `peer.json`；若失败则关闭已申请套接字。"""
        self.directory = directory
        self.tcp = self.udp = None
        self.active = None
        self.last = None
        self.unassigned = dict(case=None, status=None, errors=[], events=[], udp_received=0)
        self.cases = []
        self.failures = {}
        self.closed = False
        self.closed_at = None
        self.verified_at = None
        try:
            # 在 127.0.0.1 上绑定动态分配的非阻塞 TCP 监听套接字与 UDP 套接字，供 QEMU 虚拟网卡转发连接
            self.tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcp.bind(("127.0.0.1", 0))
            self.tcp.listen(1)
            self.tcp.setblocking(False)
            self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp.bind(("127.0.0.1", 0))
            self.udp.setblocking(False)
            self.addresses = dict(tcp=self.tcp.getsockname(), udp=self.udp.getsockname())
            self.ports = (self.addresses["tcp"][1], self.addresses["udp"][1])
            self._save()
        except BaseException:
            for endpoint in (self.tcp, self.udp):
                if endpoint is not None:
                    endpoint.close()
            raise

    def _save(self):
        """将当前端口、各用例网络事件、未归属报文及失败摘要序列化写入会话目录下的 `peer.json`。"""
        report = dict(addresses=self.addresses, ports=self.ports, cases=self.cases,
                      unassigned=self.unassigned, verified_at=self.verified_at,
                      closed_at=self.closed_at, failures=self.failures)
        (self.directory / "peer.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    def _error(self, row, error):
        """向指定记录追加一条带单调时钟戳的异常信息；若用例在客体侧报 Pass，则同步登记到宿主失败表。"""
        row["errors"].append(dict(type=type(error).__name__, message=str(error),
                                  repr=repr(error), errno=getattr(error, "errno", None),
                                  at=time.monotonic()))
        if row["status"] == "Pass":
            self.failures[row["case"]] = "; ".join(entry["message"] for entry in row["errors"])

    def begin(self, case, deadline):
        """在用例开始时排空残留报文，并根据用例网络标签启动对应的后台服务线程。"""
        # 确认上一个用例的网络工作线程已结束，并排空用例切换期间残留的套接字数据
        if self.active is not None:
            require(not self.active["thread"].is_alive(), "previous network peer thread is still active")
            self._release_pipe()
            self.active = None
        self._drain(self.last or self.unassigned)
        # 根据用例标签决定本次需要启动的协议服务类型（tcp、udp 或 native）
        tags = case["tags"]
        kinds = [kind for tag, kind in (("network-nic-tcp", "tcp"),
                                       ("network-nic-udp", "udp"),
                                       ("network-nic-native", "native")) if tag in tags]
        require(len(kinds) <= 1, "conflicting network peer tags")
        if not kinds:
            self.active = None
            self._save()
            return
        row = dict(case=case["case"], name=case["name"], tags=list(tags), kind=kinds[0],
                   deadline=deadline, begin_at=time.monotonic(), status=None, finish_at=None,
                   thread_entered_at=None, thread_exited_at=None, joined_at=None,
                   complete=False, errors=[], events=[], tcp_eof=False,
                   tcp_received=0, tcp_sent=0, udp_received=0, udp_sent=0)
        # 创建用于跨线程取消阻塞 select 的匿名管道，并启动后台网络服务线程
        reader, writer = os.pipe()
        self.cases.append(row)
        self.last = row
        thread = threading.Thread(target=self._serve, args=(row, reader),
                                  name=f"net-user-{case['case']}")
        self.active = dict(row=row, reader=reader, writer=writer, cancelled=False, thread=thread)
        try:
            self._save()
            thread.start()
        except BaseException as error:
            self._error(row, error)
            self._release_pipe()
            self.active = None
            raise

    def _ready(self, endpoint, row, reader, writing=False):
        """在用例绝对截止时刻内通过 select 等待套接字就绪或取消管道唤醒，超时则抛出 HostError。"""
        remaining = row["deadline"] - time.monotonic()
        require(0 < remaining, "network peer absolute deadline expired")
        readable, writable, _ = select.select(
            [reader] if writing else [endpoint, reader], [endpoint] if writing else [], [], remaining)
        if endpoint in (writable if writing else readable):
            return
        if reader in readable:
            raise PeerStopped("network peer cancelled")
        raise HostError("network peer absolute deadline expired")

    def _serve(self, row, reader):
        """后台网络服务线程入口：按协议类型分发处理，并捕获运行期或取消异常写入用例记录。"""
        row["thread_entered_at"] = time.monotonic()
        try:
            if row["kind"] == "tcp":
                self._tcp(row, reader)
                row["complete"] = True
            else:
                try:
                    self._udp(row, reader)
                except PeerStopped:
                    if row["kind"] == "udp":
                        require(row["udp_received"] == 4 and row["udp_sent"] == 4,
                                "UDP peer did not complete exactly four messages")
                    row["complete"] = True
        except (HostError, OSError, PeerStopped) as error:
            self._error(row, error)
        finally:
            if "tcp_peer" in row:
                row["events"].append(dict(kind="tcp-close", at=time.monotonic()))
            row["thread_exited_at"] = time.monotonic()

    def _tcp(self, row, reader):
        """处理单次 TCP 连接：接收并校验客体发来的 98317 字节请求流及 EOF，随后回发 65539 字节响应流。"""
        self._ready(self.tcp, row, reader)
        connection, address = self.tcp.accept()
        with connection:
            row["tcp_peer"] = address
            connection.setblocking(False)
            row["events"].append(dict(kind="tcp-accept", source=address,
                                       destination=connection.getsockname(), at=time.monotonic()))
            received_path = f"peer-{row['case']}-tcp-received.raw"
            sent_path = f"peer-{row['case']}-tcp-sent.raw"
            row["tcp_received_file"] = received_path
            row["tcp_sent_file"] = sent_path
            with (self.directory / received_path).open("xb", buffering=0) as received, \
                    (self.directory / sent_path).open("xb", buffering=0) as sent:
                # 循环接收客体发来的 TCP 请求流，落盘原始字节并校验 98317 字节的预设图样直至对端半关闭
                while True:
                    self._ready(connection, row, reader)
                    block = connection.recv(65536)
                    offset = row["tcp_received"]
                    received.write(block)
                    row["tcp_received"] += len(block)
                    row["events"].append(dict(kind="tcp-recv", offset=offset, length=len(block),
                                               source=address, destination=connection.getsockname(),
                                               at=time.monotonic()))
                    if not block:
                        row["tcp_eof"] = True
                        require(row["tcp_received"] == 98317, "TCP request ended before full payload")
                        break
                    require(row["tcp_received"] <= 98317, "TCP request exceeds expected payload")
                    expected = bytes((i * 37 + (i >> 8) + 19) & 255
                                     for i in range(offset, row["tcp_received"]))
                    require(block == expected, f"TCP request payload mismatch at offset {offset}")
                # 生成 65539 字节的响应数据并分块发回客体，发送完毕后执行写半关闭（SHUT_WR）
                response = bytes((i * 53 + (i >> 7) + 91) & 255 for i in range(65539))
                while row["tcp_sent"] < len(response):
                    self._ready(connection, row, reader, writing=True)
                    offset = row["tcp_sent"]
                    count = connection.send(memoryview(response)[offset:])
                    require(0 < count, "TCP response send made no progress")
                    sent.write(response[offset:offset + count])
                    row["tcp_sent"] += count
                    row["events"].append(dict(kind="tcp-send", offset=offset, length=count,
                                               source=connection.getsockname(), destination=address,
                                               at=time.monotonic()))
                connection.shutdown(socket.SHUT_WR)
                row["events"].append(dict(kind="tcp-shutdown-write", at=time.monotonic()))

    def _udp_record(self, row, kind, data, address):
        """记录单条 UDP 收发事件的长度、完整十六进制负载、源目地址与时间戳（含零长数据报）。"""
        sending = kind == "udp-send"
        row["events"].append(dict(kind=kind, length=len(data), bytes_hex=data.hex(),
                                   source=self.addresses["udp"] if sending else address,
                                   destination=address if sending else self.addresses["udp"],
                                   at=time.monotonic()))

    def _udp_send(self, row, reader, data, address):
        """等待 UDP 套接字可写后发送单条响应数据报，记录发送事件并校验完整发出。"""
        self._ready(self.udp, row, reader, writing=True)
        count = self.udp.sendto(data, address)
        self._udp_record(row, "udp-send", data[:count], address)
        require(count == len(data), "UDP response length mismatch")
        row["udp_sent"] += 1

    def _udp(self, row, reader):
        """运行 UDP 接收循环：在标准模式下校验 4 条固定图样报文并回显，在 native 模式下响应探测与突发请求。"""
        source = None
        while True:
            self._ready(self.udp, row, reader)
            data, address = self.udp.recvfrom(65535)
            self._udp_record(row, "udp-recv", data, address)
            index = row["udp_received"]
            row["udp_received"] += 1
            # 标准 UDP 模式：依次校验 4 条不同长度（含 0 字节与分片大小）的报文并按字节异或 0xa5 回显
            if row["kind"] == "udp":
                require(index < 4, "unexpected fifth UDP message; oversized send reached host")
                if source is None:
                    source = address
                require(address == source, "UDP request source changed between messages")
                length = (17, 0, 1472, 8193)[index]
                expected = bytes((i * 29 + (i >> 4) + 45) & 255 for i in range(length))
                require(data == expected, f"UDP request payload mismatch at message {index}")
                self._udp_send(row, reader, bytes(byte ^ 0xa5 for byte in data), address)
            # 原生网卡测试模式（native）：处理单字节探测 P、带序号探测 NP 以及突发报文请求 NB
            elif data == b"P":
                self._udp_send(row, reader, b"Q", address)
            elif data[:2] == b"NP":
                require(len(data) == 4, "native UDP indexed request must be exactly four bytes")
                index = int.from_bytes(data[2:], "little")
                require(index <= 32, "native UDP request index outside 0..32")
                self._udp_send(row, reader, b"NQ" + data[2:], address)
            else:
                require(len(data) == 4 and data[:2] == b"NB", "unexpected native UDP request")
                count = int.from_bytes(data[2:], "little")
                require(1 <= count <= 32, "native UDP burst count outside 1..32")
                for index in range(count):
                    body = bytes((index * 17 + i * 31 + 7) & 255 for i in range(64))
                    self._udp_send(row, reader, b"NR" + index.to_bytes(2, "little") + body, address)

    def _cancel(self):
        """向取消管道写入唤醒字节以中断后台线程的 select 等待，但不关闭底层网络套接字。"""
        if self.active is not None and not self.active["cancelled"]:
            os.write(self.active["writer"], b"X")
            self.active["cancelled"] = True
            self.active["row"]["cancel_at"] = time.monotonic()

    def _join(self, deadline):
        """在给定绝对截止时刻前等待后台网络线程退出，成功汇合时记录首次 join 时间并返回 True。"""
        thread = self.active["thread"]
        thread.join(timeout=max(0, deadline - time.monotonic()))
        if thread.is_alive():
            return False
        if self.active["row"]["joined_at"] is None:
            self.active["row"]["joined_at"] = time.monotonic()
        return True

    def finish(self, status, deadline):
        """在用例结束时停止并汇合后台网络线程、排空残留数据；若客体报 Pass 则要求宿主网络交互完整无误。"""
        if self.active is None:
            return
        row = self.active["row"]
        require(deadline == row["deadline"], "network peer case deadline changed")
        row["status"] = status
        row["finish_at"] = time.monotonic()
        if row["kind"] != "tcp" or status != "Pass":
            self._cancel()
        joined = self._join(deadline)
        if not joined:
            self._error(row, HostError("network peer thread exceeded absolute case deadline"))
        else:
            try:
                self._drain(row)
            except OSError as error:
                self._error(row, error)
            if status == "Pass" and not row["complete"] and not row["errors"]:
                self._error(row, HostError("network peer consumer is incomplete"))
            if status == "Pass" and row["errors"]:
                self.failures[row["case"]] = "; ".join(entry["message"] for entry in row["errors"])
            self._save()
        if status == "Pass":
            require(joined and row["complete"] and not row["errors"],
                    self.failures.get(row["case"], "network peer consumer is incomplete"))

    def _drain(self, row):
        """非阻塞轮询并读尽 UDP 与 TCP 套接字中额外排队的数据报或连接，将其记为非预期流量错误。"""
        while select.select([self.udp], [], [], 0)[0]:
            data, address = self.udp.recvfrom(65535)
            self._udp_record(row, "udp-recv", data, address)
            row["udp_received"] += 1
            self._error(row, HostError("unexpected UDP message after network consumer finished"))
        while select.select([self.tcp], [], [], 0)[0]:
            connection, address = self.tcp.accept()
            with connection:
                row["events"].append(dict(kind="unexpected-tcp-accept", source=address,
                                           destination=connection.getsockname(), at=time.monotonic()))
            self._error(row, HostError("unexpected additional TCP connection"))

    def verify(self, deadline):
        """在 QEMU 进程退出后停止残留线程、排空套接字并保存最终 `peer.json`，校验无宿主侧网络失败。"""
        if self.active is not None:
            self._cancel()
            joined = self._join(deadline)
            require(joined, "network peer thread exceeded VM cleanup deadline")
        self._drain(self.last or self.unassigned)
        self.verified_at = time.monotonic()
        self._save()
        require(not self.failures, "network peer verification failed: " + str(self.failures))

    def _release_pipe(self):
        """关闭当前用例后台线程使用的取消管道读写端文件描述符。"""
        os.close(self.active["reader"])
        os.close(self.active["writer"])

    def close(self):
        """在 QEMU 进程已回收后执行最终清理：停止后台线程、排空残留流量、关闭 TCP/UDP 套接字并落盘报告。"""
        if self.closed:
            return
        self._cancel()
        if self.active is not None:
            self.active["thread"].join()
            self.active["row"]["close_joined_at"] = time.monotonic()
            self._release_pipe()
        row = self.last or self.unassigned
        try:
            self._drain(row)
        except OSError as error:
            self._error(row, error)
        self.tcp.close()
        self.udp.close()
        self.closed = True
        self.closed_at = time.monotonic()
        self._save()
