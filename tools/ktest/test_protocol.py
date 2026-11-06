"""宿主侧 ktest 串口协议与 KTAP 状态机解析回归测试。

本模块验证 `ktest.protocol` 对串口握手行（`KTEST READY`）、KTAP 测试计划与结果行、
阶段流转控制行（`case-begin` / `phase` / `armed` / `fatal` / `case-abort` / `suite-error`）
以及 UART 流量声明（`KTEST io`）的语法与状态机约束。
运行方式：`python3 -m unittest discover -s tools/ktest`。
"""

import json
import unittest

from host_helpers import CLI, CONFIG, case_record, case_lines, parser_for
from ktest.catalog import HostError
from ktest.protocol import CaseResult, ready


class ProtocolTests(unittest.TestCase):
    """验证用例身份匹配、阶段流转顺序与顶层结束标记等协议状态机约束。"""

    def test_ready_uses_real_cpu_count(self):
        """验证 `KTEST READY` 上报的在线 CPU 数量与请求配置不符时立即抛出启动错误，而非降级为跳过。"""
        with self.assertRaises(HostError):
            ready("KTEST READY 1 arch=riscv64 cpus=1 mode=debug panic=unwind", CONFIG)


    def test_cpu_arguments_match_ready_without_narrowing(self):
        """验证正整数 `--cpus` 参数原样写入请求配置，且客体上报核数不一致时会被握手校验拒绝。"""
        for architecture in ("riscv64", "loongarch64"):
            for count in (1, 2, 3, 4, 5, 8, 16, 32, 33, 64, 65, 128):
                with self.subTest(architecture=architecture, count=count):
                    args = CLI.arguments(["run", "--arch", architecture, "--cpus", str(count)])
                    config = CLI.config(args)
                    line = (f"KTEST READY 1 arch={architecture} cpus={count} "
                            "mode=debug panic=unwind")
                    ready(line, config)
                    with self.assertRaises(HostError):
                        ready(line.replace(f"cpus={count} ", f"cpus={count + 1} "), config)

    def test_cpu_arguments_reject_nonpositive_and_fractional(self):
        """验证命令行参数解析阶段直接拒绝零、负数与非整数的 `--cpus` 输入。"""
        from contextlib import redirect_stderr
        import io

        for value in ("0", "-1", "1.5", "one"):
            with self.subTest(value=value), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as raised:
                    CLI.arguments(["run", "--arch", "riscv64", "--cpus", value])
                self.assertEqual(raised.exception.code, 2)
    def test_complete_pass_with_diagnostics(self):
        """验证普通诊断注释中的 `panic` 或 `PASS` 字样不会干扰状态机，只有走完完整生命周期才判定为通过。"""
        row = case_record()
        parser = parser_for([row])
        parser.feed("  # expected input contains panic and PASS")
        for line in case_lines(row):
            parser.feed(line)
        parser.feed("ok 1 os::tests0")
        parser.feed("# KTEST complete")
        self.assertTrue(parser.complete)
        self.assertEqual(parser.results[0]["status"], "Pass")

    def test_early_pass_missing_result_and_fake_suite_pass_rejected(self):
        """验证在清理阶段完成前提前输出 `ok`、遗漏用例结果或有失敗用例却输出顶层 `ok` 时均会报错。"""
        row = case_record()
        # 场景 1：仅进入 body 阶段、尚未走完清理阶段就提前输出用例 ok
        parser = parser_for([row])
        for line in case_lines(row)[:3]:
            parser.feed(line)
        with self.assertRaises(HostError):
            parser.feed(f"  ok 1 {row['name']}")
        # 场景 2：计划运行 2 个用例，但只完成了 1 个就提前输出套件总结果
        parser = parser_for([row, case_record(1)])
        for line in case_lines(row):
            parser.feed(line)
        with self.assertRaises(HostError):
            parser.feed("ok 1 os::tests0")
        # 场景 3：子用例结果为 Fail，但套件顶层汇总行却输出了 ok
        parser = parser_for([row])
        for line in case_lines(row, status="Fail"):
            parser.feed(line)
        with self.assertRaises(HostError):
            parser.feed("ok 1 os::tests0")

    def test_wrong_name_number_duplicate_and_indent_are_rejected(self):
        """验证 KTAP 结果行的用例名称、序号、缩进层级必须与目录严格一致，且禁止重复输出结果。"""
        row = case_record()
        for invalid in ("  ok 1 wrong", f"  ok 2 {row['name']}", f"    ok 1 {row['name']}"):
            parser = parser_for([row])
            for line in case_lines(row)[:-1]:
                parser.feed(line)
            with self.assertRaises(HostError):
                parser.feed(invalid)
        parser = parser_for([row])
        for line in case_lines(row):
            parser.feed(line)
        with self.assertRaises(HostError):
            parser.feed(case_lines(row)[-1])

    def test_missing_complete_cannot_be_success(self):
        """验证即使所有用例和套件汇总行均已输出，缺少顶层 `# KTEST complete` 或带多余后缀时仍视为未完成或错误。"""
        row = case_record()
        parser = parser_for([row])
        for line in case_lines(row):
            parser.feed(line)
        parser.feed("ok 1 os::tests0")
        self.assertFalse(parser.complete)
        with self.assertRaises(HostError):
            parser.feed("# KTEST complete trailing")

    def test_actual_fatal_validates_identity_kind_uniqueness_and_noreturn(self):
        """验证 `KTEST fatal` 事件必须匹配当前活动用例 ID 与合法故障类型，且触发后禁止再输出任何后续控制行。"""
        row = case_record(expected_fatal="illegal")
        for invalid in ("  # KTEST fatal 1 illegal", "  # KTEST fatal 0 invented"):
            with self.subTest(invalid=invalid):
                parser = parser_for([row])
                for line in case_lines(row)[:3]:
                    parser.feed(line)
                with self.assertRaises(HostError):
                    parser.feed(invalid)
        for late in ("  # KTEST fatal 0 oom", "  # KTEST phase 0 cleanup",
                     "  # KTEST armed 0 illegal"):
            with self.subTest(late=late):
                parser = parser_for([row])
                for line in case_lines(row)[:3]:
                    parser.feed(line)
                parser.feed("  # KTEST fatal 0 oom")
                self.assertEqual(parser.fatal, "oom")
                with self.assertRaises(HostError):
                    parser.feed(late)

    def test_skip_retains_reason_and_status(self):
        """验证 `SKIP` 指令能够正确解析为 `Skip` 状态并保留跳过原因，与普通 `Pass` 区分开。"""
        row = case_record()
        parser = parser_for([row])
        for line in case_lines(row, status="Skip"):
            parser.feed(line)
        parser.feed("ok 1 os::tests0")
        parser.feed("# KTEST complete")
        self.assertEqual(parser.results[0], dict(status="Skip", reason="insufficient CPUs"))

    def test_complete_result_event_keeps_identity_and_rejects_private_status(self):
        """验证合法的 KTAP 结果行生成携带用例 ID、状态和原因的 `CaseResult` 事件并清空当前活动状态，同时拒绝非法指令或私有状态行。"""
        row = case_record(7)
        samples = (("ok", "", "Pass", ""),
                   ("not ok", "", "Fail", ""),
                   ("ok", " # SKIP", "Skip", ""),
                   ("ok", " # SKIP one CPU", "Skip", "one CPU"),
                   ("not ok", " # ERROR", "Error", ""))
        for polarity, directive, status, reason in samples:
            with self.subTest(status=status, directive=directive):
                parser = parser_for([row])
                for line in case_lines(row)[:-1]:
                    self.assertNotIsInstance(parser.feed(line), CaseResult)
                event = parser.feed(f"  {polarity} 1 {row['name']}{directive}")
                self.assertEqual(event, CaseResult(7, status, reason))
                self.assertIsNone(parser.active)
                self.assertIsNone(parser.phase)
                self.assertEqual(parser.results[7], dict(status=status, reason=reason))
        for suffix in (" # SKIP bad", " # ERROR", " # UNKNOWN"):
            parser = parser_for([row])
            for line in case_lines(row)[:-1]:
                parser.feed(line)
            with self.subTest(suffix=suffix), self.assertRaises(HostError):
                parser.feed(f"  {'not ok' if 'SKIP' in suffix else 'ok'} 1 {row['name']}{suffix}")
        parser = parser_for([row])
        for line in case_lines(row)[:-1]:
            parser.feed(line)
        with self.assertRaises(HostError):
            parser.feed("  # KTEST status 7 Pass")

    def test_parameter_directive_text_is_part_of_full_result_identity(self):
        """验证参数化用例名称方括号内包含 `#`、`# SKIP` 或 `# ERROR` 时，解析器仍将其视为完整名称的一部分。"""
        for label in ("value # comment", "value # SKIP reason", "value # ERROR"):
            row = case_record(7, parameter=label, name=f"os::tests0::case7[{label}]")
            for status in ("Pass", "Fail", "Skip", "Error"):
                with self.subTest(label=label, status=status):
                    parser = parser_for([row])
                    for line in case_lines(row, status=status)[:-1]:
                        parser.feed(line)
                    event = parser.feed(case_lines(row, status=status)[-1])
                    reason = "insufficient CPUs" if status == "Skip" else ""
                    self.assertEqual(event, CaseResult(7, status, reason))
                    self.assertEqual(parser.results[7], dict(status=status, reason=reason))
                    self.assertIsNone(parser.active)
                    outer = "ok" if status in ("Pass", "Skip") else "not ok"
                    parser.feed(f"{outer} 1 os::tests0")
                    parser.feed("# KTEST complete")
                    self.assertTrue(parser.complete)

    def test_result_directive_starts_only_after_exact_full_name(self):
        """验证 `# SKIP` / `# ERROR` 指令必须紧跟在精确匹配的完整用例名之后，任何名称尾缀或篡改都会被拒绝。"""
        label = "value # comment"
        row = case_record(7, parameter=label, name=f"os::tests0::case7[{label}]")
        for polarity, suffix, status in (("ok", " # SKIP reason # detail", "Skip"),
                                         ("not ok", " # ERROR actual failure", "Error")):
            with self.subTest(suffix=suffix):
                parser = parser_for([row])
                for line in case_lines(row)[:-1]:
                    parser.feed(line)
                event = parser.feed(f"  {polarity} 1 {row['name']}{suffix}")
                self.assertEqual(event, CaseResult(
                    7, status, "reason # detail" if status == "Skip" else "actual failure"))
        invalid = (
            ("ok", row["name"] + "extra"),
            ("ok", row["name"] + " extra # SKIP reason"),
            ("ok", row["name"] + " # SKIPPER reason"),
            ("ok", row["name"] + " # ERROR"),
            ("not ok", row["name"] + " # SKIP reason"),
            ("ok", row["name"].replace("comment", "different") + " # SKIP reason"),
            ("ok", row["name"][:-1] + " # SKIP reason"),
        )
        for polarity, name in invalid:
            with self.subTest(polarity=polarity, name=name):
                parser = parser_for([row])
                for line in case_lines(row)[:-1]:
                    parser.feed(line)
                with self.assertRaises(HostError):
                    parser.feed(f"  {polarity} 1 {name}")
                self.assertEqual(parser.results, {})
                self.assertEqual(parser.active, 7)

    def test_fatal_message_only_matches_armed_actual_diagnostics(self):
        """验证预期 panic 的消息子串只能在 `armed` 且 `fatal` 事件发生之后的诊断行中匹配，此前的日志不计入匹配。"""
        row = case_record(expected_fatal="panic", fatal_message="unique actual panic")
        parser = parser_for([row])
        for line in case_lines(row)[:3]:
            parser.feed(line)
        parser.feed("  # unique actual panic")
        self.assertFalse(parser.fatal_message_seen)
        parser.feed("  # KTEST armed 0 panic")
        parser.feed("  # KTEST fatal 0 panic")
        self.assertFalse(parser.fatal_message_seen)
        parser.feed("  # panic: unique actual panic")
        self.assertTrue(parser.fatal_message_seen)

    def test_phase_regression_and_outside_fixture_uart_are_errors(self):
        """验证用例执行阶段（phase）不允许倒退，且 `KTEST io` 串口流量声明只能出现在进入有效阶段之后。"""
        row = case_record()
        parser = parser_for([row])
        parser.feed("  # KTEST case-begin 0")
        with self.assertRaises(HostError):
            parser.feed("  # KTEST io 0 tx 9 e3069283 0")
        parser.feed("  # KTEST phase 0 setup")
        parser.feed("  # KTEST phase 0 body")
        with self.assertRaises(HostError):
            parser.feed("  # KTEST phase 0 setup")

    def test_empty_pause_barrier_does_not_allow_empty_transfers(self):
        """验证仅 `pause` 屏障允许声明 0 字节与全零 CRC，而 `tx`/`rx`/`slow` 传输或超限延迟均会被拒绝。"""
        parser = parser_for([case_record()])
        for line in ("  # KTEST case-begin 0", "  # KTEST phase 0 setup", "  # KTEST phase 0 body"):
            parser.feed(line)
        self.assertEqual(parser.feed("  # KTEST io 0 pause 0 00000000 3000"),
                         dict(direction="pause", length=0, digest="00000000", delay_ms=3000, case=0))
        for declaration in ("tx 0 00000000 0", "rx 0 00000000 0", "slow 0 00000000 1",
                            "pause 0 00000001 3000", "pause 0 00000000 60001"):
            with self.subTest(declaration=declaration), self.assertRaises(HostError):
                parser.feed("  # KTEST io 0 " + declaration)

    def test_suite_setup_error_has_no_started_case(self):
        """验证套件级 `setup` 阶段报错时只记录独立的 `suite_error`，不会将任何用例标记为已启动。"""
        parser = parser_for([case_record(0), case_record(1)])
        parser.feed("  # KTEST suite-error setup allocation failed\\nrollback complete")
        parser.feed("not ok 1 os::tests0")
        parser.feed("# KTEST complete")
        self.assertEqual(parser.started, set())
        self.assertEqual(parser.results, {})
        self.assertEqual(parser.suite_error,
                         dict(phase="setup", reason="allocation failed\\nrollback complete"))
        self.assertTrue(parser.complete)

    def test_suite_cleanup_error_preserves_all_case_passes(self):
        """验证即使所有用例均已通过，套件级 `cleanup` 阶段报错后顶层汇总行仍必须为 `not ok`，且结束标记后禁止再出现控制行。"""
        rows = [case_record(0), case_record(1)]
        parser = parser_for(rows)
        for position, row in enumerate(rows, 1):
            for line in case_lines(row, position):
                parser.feed(line)
        parser.feed("  # KTEST suite-error cleanup resource release failed")
        with self.assertRaises(HostError):
            parser.feed("ok 1 os::tests0")
        parser.feed("not ok 1 os::tests0")
        parser.feed("# KTEST complete")
        parser.feed("ordinary business diagnostic")
        self.assertEqual([r["status"] for r in parser.results.values()], ["Pass", "Pass"])
        self.assertEqual(parser.suite_error["phase"], "cleanup")
        for late in ("  # KTEST suite-error cleanup duplicate",
                     "  # KTEST suite-begin setup", case_lines(rows[0])[-1],
                     "not ok 1 os::tests0", "# KTEST complete"):
            with self.subTest(late=late), self.assertRaises(HostError):
                parser.feed(late)

    def test_missing_cases_need_explicit_valid_suite_error(self):
        """验证存在未启动用例时必须有合法的 `suite-error execution` 说明原因，且当前正在执行的用例不允许无结果直接报错退出。"""
        rows = [case_record(0), case_record(1)]
        parser = parser_for(rows)
        for line in case_lines(rows[0]):
            parser.feed(line)
        with self.assertRaises(HostError):
            parser.feed("not ok 1 os::tests0")
        for invalid in ("  # KTEST suite-error setup failed",
                        "  # KTEST suite-error cleanup failed"):
            with self.subTest(invalid=invalid), self.assertRaises(HostError):
                parser.feed(invalid)
        parser.feed("  # KTEST suite-error execution dispatcher failed")
        for invalid in ("  # KTEST case-begin 1", "ok 1 os::tests0", "not ok 1 wrong"):
            with self.subTest(invalid=invalid), self.assertRaises(HostError):
                parser.feed(invalid)
        parser.feed("not ok 1 os::tests0")
        parser.feed("# KTEST complete")
        active = parser_for(rows)
        for line in case_lines(rows[0])[:3]:
            active.feed(line)
        with self.assertRaises(HostError):
            active.feed("  # KTEST suite-error execution missing active result")

    def test_case_abort_requires_current_unfinished_identity_and_actual_cleanup_phase(self):
        """验证 `case-abort` 只能终结当前正在执行且处于清理阶段（`quiesce`/`join`/`cleanup`）的用例，且触发后禁止后续用例继续运行。"""
        rows = [case_record(0), case_record(1)]
        ledger = dict(primary="body failed", child=None, cleanup="unsafe to release",
                      expectations=2, reason="cleanup did not converge")
        payload = json.dumps(ledger)
        for phase in ("quiesce", "join", "cleanup"):
            parser = parser_for(rows)
            for line in case_lines(rows[0])[:3]:
                parser.feed(line)
            parser.feed(f"  # KTEST phase 0 {phase}")
            self.assertEqual(parser.feed(f"  # KTEST case-abort 0 {phase} {payload}"), "case-abort")
            self.assertEqual(parser.results, {}, "abort must not pretend normal cleanup finished")
            self.assertEqual(parser.case_abort, dict(phase=phase, **ledger))
            invalid = (f"  # KTEST case-abort 0 {phase} {payload}",
                       "  # KTEST case-begin 1", "  # KTEST phase 0 cleanup",
                       "  # KTEST status 0 Error", f"  not ok 1 {rows[0]['name']}",
                       "  # KTEST suite-error execution failed", "ok 1 os::tests0",
                       "not ok 1 wrong")
            for line in invalid:
                with self.subTest(phase=phase, line=line), self.assertRaises(HostError):
                    parser.feed(line)
            parser.feed("not ok 1 os::tests0")
            parser.feed("# KTEST complete")
            with self.assertRaises(HostError):
                parser.feed(f"  # KTEST case-abort 0 {phase} {payload}")
        for prefix, identity, phase in (([], 0, "cleanup"),
                                       (case_lines(rows[0])[:2], 0, "setup"),
                                       (case_lines(rows[0])[:3], 0, "body"),
                                       (case_lines(rows[0])[:4], 1, "cleanup"),
                                       (case_lines(rows[0])[:4], 0, "join"),
                                       (case_lines(rows[0]), 0, "cleanup")):
            parser = parser_for(rows)
            for line in prefix:
                parser.feed(line)
            with self.subTest(prefix=prefix, identity=identity, phase=phase), self.assertRaises(HostError):
                parser.feed(f"  # KTEST case-abort {identity} {phase} {payload}")

    def test_case_abort_rejects_malformed_duplicate_or_wrong_typed_json_fields(self):
        """验证 `case-abort` 携带的 JSON 载荷必须严格包含规定的 5 个独立字段且类型合法，拒绝缺失、多余或重复键。"""
        ledger = dict(primary=None, child=None, cleanup=None, expectations=0, reason="stop")
        invalid = ["{", "null", "[]", json.dumps(list(ledger.items())),
                   json.dumps({key: value for key, value in ledger.items() if key != "child"}),
                   json.dumps(dict(ledger, extra="unsupported")),
                   json.dumps(ledger)[:-1] + ', "reason":"overwritten"}']
        for key, value in (("primary", 0), ("child", []), ("cleanup", {}),
                           ("expectations", True), ("expectations", -1),
                           ("expectations", 1.5), ("expectations", "0"), ("reason", None)):
            invalid.append(json.dumps(dict(ledger, **{key: value})))
        for payload in invalid:
            parser = parser_for([case_record()])
            for line in case_lines(case_record())[:4]:
                parser.feed(line)
            with self.subTest(payload=payload), self.assertRaises(HostError):
                parser.feed(f"  # KTEST case-abort 0 cleanup {payload}")
            self.assertIsNone(parser.case_abort)
        parser.feed("  # KTEST case-abort 0 cleanup " + json.dumps(ledger))
        self.assertEqual(parser.case_abort, dict(phase="cleanup", **ledger))

    def test_duplex_declaration_validates_both_lengths_crc_and_delays(self):
        """验证 `duplex` 双向串口传输声明会分别校验 RX/TX 的长度上限、非零载荷 CRC 以及每段延迟范围。"""
        invalid = ("duplex 0 00000000 0 0 00000000 0",
                   "duplex 1 01234567 0 0 00000001 0",
                   "duplex 0 00000001 0 1 00000000 0",
                   "duplex 0 00000000 60001 1 00000000 0",
                   "duplex 0 00000000 0 1 00000000 60001",
                   "duplex 33554433 01234567 0 33554432 89abcdef 0",
                   "duplex 1 00000000 0 1 00000000")
        for declaration in invalid:
            case = case_record(resources=["uart"])
            parser = parser_for([case])
            for line in case_lines(case)[:3]:
                parser.feed(line)
            with self.subTest(declaration=declaration), self.assertRaises(HostError):
                parser.feed(f"  # KTEST io 0 {declaration}")

        case = case_record(resources=["uart"])
        parser = parser_for([case])
        for line in case_lines(case)[:3]:
            parser.feed(line)
        self.assertEqual(parser.feed("  # KTEST io 0 duplex 1 01234567 10 2 89abcdef 20"),
                         dict(direction="duplex", rx_length=1, rx_digest="01234567",
                              rx_delay_ms=10, tx_length=2, tx_digest="89abcdef",
                              tx_delay_ms=20, case=0))


if __name__ == "__main__":
    unittest.main()
