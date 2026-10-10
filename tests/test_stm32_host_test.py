"""No physical serial ports: exercise host lifecycle against a simulated STM32."""
import contextlib
import importlib.util
import io
from pathlib import Path
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    "stm32_host_test", Path(__file__).resolve().parents[1] / "stm32_host_test.py")
host = importlib.util.module_from_spec(spec)
spec.loader.exec_module(host)


class Clock:
    now = 0.0

    def monotonic(self):
        return self.now


class SimulatedSTM32:
    """Replies are deliberately fragmented, including heartbeat acknowledgements."""

    def __init__(self, clock):
        self.clock = clock
        self.rx = bytearray()
        self.sent = []
        self.states = {addr: {"state": 1, "rpm": 0, "flags": 0}
                       for addr in (1, 2)}
        self.pending_enable = {}
        self.enable_delay = 0.0
        self.position_done = None
        self.fault = 0
        self.age = 10
        self.reject_enable = False
        self.drop_motion_ack = False
        self.motion_fault = False
        self.interrupt_motion = False
        self.position_never_done = False
        self.motion_started = False
        self.silent = False

    @property
    def in_waiting(self):
        return len(self.rx)

    def reply(self, text):
        if not self.silent:
            self.rx.extend((text + "\r\n").encode("ascii"))

    def write(self, packet):
        command = packet.decode("ascii").strip()
        self.sent.append((self.clock.now, command))
        parts = command.split()
        if command == "PING":
            self.reply("OK PONG")
        elif command == "HEARTBEAT":
            self.reply("OK HEARTBEAT")
        elif command == "STOP":
            self.position_done = None
            for state in self.states.values():
                state.update(state=2 if state["flags"] & 1 else 1, rpm=0)
            self.reply("OK STOP")
        elif command == "CLEAR_FAULT":
            self.fault = 0
            for state in self.states.values():
                state.update(state=1, rpm=0, flags=0)
            self.reply("OK RECOVERY")
        elif parts[:2] == ["MOTOR", "ENABLE"]:
            addr, enabled = map(int, parts[2:])
            if enabled and self.reject_enable:
                self.reply("ERR NOT_READY")
            else:
                self.pending_enable[addr] = (self.clock.now + self.enable_delay, enabled)
                self.reply("OK QUEUED")
        elif parts[:2] == ["MOTOR", "STATUS"]:
            addr = int(parts[2])
            s = self.states[addr]
            self.reply(f"DATA MOTOR {addr} STATE {s['state']} RPM {s['rpm']} "
                       f"FLAGS {s['flags']} FAULT {self.fault}")
            self.reply(f"DATA FEEDBACK {addr} ONLINE 1 CONFIG 1 AGE {self.age}")
        elif parts[:2] == ["MOTOR", "DIAG"]:
            addr = int(parts[2])
            self.reply(f"DATA DIAG {addr} INIT 5 TIMEOUT 0x35 REJECT 0xF3")
            self.reply(f"DATA TIMEOUT {addr} RX NONE")
            self.reply(f"DATA REJECT {addr} RX {addr:02X} F3 E2 6B")
        elif parts[:2] in (["MOTOR", "VEL"], ["MOTOR", "POS"]) or parts[0] == "WHEELS":
            self.motion_started = True
            if parts[0] == "WHEELS":
                for addr, rpm in zip((1, 2), map(int, parts[1:3])):
                    self.states[addr].update(state=3, rpm=rpm)
            else:
                addr = int(parts[2])
                self.states[addr].update(state=4 if parts[1] == "POS" else 3,
                                         rpm=int(parts[4] if parts[1] == "POS" else parts[3]))
                if parts[1] == "POS" and not self.position_never_done:
                    self.position_done = (self.clock.now + 0.4, addr)
            if self.motion_fault:
                self.fault = 4
                self.reply("EVENT FAULT 4")
            if not self.drop_motion_ack:
                self.reply("OK QUEUED")
        return len(packet)

    def read(self, size):
        self.clock.now += 0.01 if self.rx else 0.02
        for addr, (deadline, enabled) in list(self.pending_enable.items()):
            if self.clock.now >= deadline:
                self.states[addr].update(state=2 if enabled else 1, flags=enabled, rpm=0)
                del self.pending_enable[addr]
        if self.position_done and self.clock.now >= self.position_done[0]:
            self.states[self.position_done[1]].update(state=2, flags=3, rpm=0)
            self.position_done = None
        if self.interrupt_motion and self.motion_started:
            self.interrupt_motion = False
            raise KeyboardInterrupt
        count = min(size, 7, len(self.rx))
        data = bytes(self.rx[:count])
        del self.rx[:count]
        return data


class HostTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.port = SimulatedSTM32(self.clock)
        self.link = host.Link(self.port, reply_timeout=0.4)
        self.time_patch = patch.object(host.time, "monotonic", self.clock.monotonic)
        self.time_patch.start()
        self.output = contextlib.ExitStack()
        self.output.enter_context(contextlib.redirect_stdout(io.StringIO()))
        self.output.enter_context(contextlib.redirect_stderr(io.StringIO()))

    def tearDown(self):
        self.output.close()
        self.time_patch.stop()

    def args(self, *arguments):
        return host.parse_args(["--port", "MOCK", "--ready-timeout", "1", *arguments])

    def commands(self):
        return [command for _, command in self.port.sent]

    def assert_shutdown(self):
        self.assertIn("STOP", self.commands())
        for addr in (1, 2):
            self.assertIn(f"MOTOR ENABLE {addr} 0", self.commands())
            self.assertEqual(self.port.states[addr]["rpm"], 0)
            self.assertFalse(self.port.states[addr]["flags"] & 1)
        self.assertFalse(self.link.heartbeat)

    def test_fragmented_status_does_not_enable_or_heartbeat(self):
        host.execute(self.link, self.args("status"))
        self.assertEqual(self.commands(), ["MOTOR STATUS 1", "MOTOR STATUS 2"])

    def test_diagnostic_reads_three_fragmented_lines_despite_latched_fault(self):
        self.port.fault = self.link.fault = 68
        self.link.reply_timeout = 0.8
        host.execute(self.link, self.args("--single-right", "diag"))
        self.assertEqual(self.commands(), ["MOTOR STATUS 2", "MOTOR DIAG 2"])

    def test_waits_for_actual_enable_and_heartbeats_during_wait(self):
        self.port.enable_delay = 0.35
        host.execute(self.link, self.args("left", "-20", "--run", "0.4"))
        commands = self.commands()
        enabled = next(t for t, cmd in self.port.sent if cmd == "MOTOR ENABLE 1 1")
        moved = next(t for t, cmd in self.port.sent if cmd == "MOTOR VEL 1 -20 10")
        self.assertGreaterEqual(moved - enabled, 0.35)
        self.assertNotIn("MOTOR ENABLE 2 1", commands)
        beats = [t for t, cmd in self.port.sent if cmd == "HEARTBEAT"]
        self.assertGreater(len(beats), 5)
        self.assertLess(max(b - a for a, b in zip(beats, beats[1:])), 0.14)
        self.assert_shutdown()

    def test_wheels_command_uses_body_direction_without_extra_inversion(self):
        host.execute(self.link, self.args("wheels", "20", "20", "--run", "0.2"))
        self.assertIn("WHEELS 20 20 10", self.commands())
        self.assertIn("MOTOR ENABLE 2 1", self.commands())
        self.assert_shutdown()

    def test_position_is_sent_once_and_waits_for_arrival(self):
        host.execute(self.link, self.args("position", "right", "-800", "--run", "2"))
        self.assertEqual(self.commands().count("MOTOR POS 2 -800 20 10 2"), 1)
        self.assertLess(self.clock.now, 2.5)
        self.assert_shutdown()

    def test_lost_position_ack_stops_without_retrying_position(self):
        self.port.drop_motion_ack = True
        with self.assertRaises(TimeoutError):
            host.execute(self.link, self.args("position", "left", "800"))
        self.assertEqual(self.commands().count("MOTOR POS 1 800 20 10 2"), 1)
        self.assert_shutdown()

    def test_position_timeout_is_not_reported_as_success(self):
        self.port.position_never_done = True
        with self.assertRaisesRegex(TimeoutError, "未确认到位"):
            host.execute(self.link, self.args("position", "left", "800", "--run", "0.3"))
        self.assert_shutdown()

    def test_enable_rejection_prevents_motion_and_attempts_shutdown(self):
        self.port.reject_enable = True
        with self.assertRaisesRegex(RuntimeError, "ERR NOT_READY"):
            host.execute(self.link, self.args("left", "20"))
        self.assertFalse(any(" VEL " in cmd for cmd in self.commands()))
        self.assert_shutdown()

    def test_fault_event_aborts_without_auto_recovery(self):
        self.port.motion_fault = True
        with self.assertRaisesRegex(RuntimeError, "故障掩码=4"):
            host.execute(self.link, self.args("left", "20"))
        self.assertNotIn("CLEAR_FAULT", self.commands())
        self.assert_shutdown()

    def test_ctrl_c_attempts_stop_and_disable(self):
        self.port.interrupt_motion = True
        with self.assertRaises(KeyboardInterrupt):
            host.execute(self.link, self.args("left", "20"))
        self.assert_shutdown()

    def test_unresponsive_stm32_never_gets_motion(self):
        self.port.silent = True
        with self.assertRaises(TimeoutError):
            host.execute(self.link, self.args("left", "20"))
        self.assertFalse(any(" VEL " in cmd for cmd in self.commands()))
        self.assertIn("STOP", self.commands())
        self.assertIn("MOTOR ENABLE 2 0", self.commands())

    def test_recovery_is_explicit_and_does_not_restart(self):
        self.link.fault = self.port.fault = 2
        host.execute(self.link, self.args("recover"))
        self.assertIn("CLEAR_FAULT", self.commands())
        self.assertFalse(any(" VEL " in cmd or cmd.endswith(" 1 1") for cmd in self.commands()))
        self.assertEqual(self.link.fault, 0)
        self.assert_shutdown()

    def test_single_mode_never_addresses_right_motor(self):
        host.execute(self.link, self.args("--single", "left", "20", "--run", "0.2"))
        self.assertFalse(any(cmd.startswith("MOTOR ENABLE 2") or cmd == "MOTOR STATUS 2"
                             for cmd in self.commands()))

    def test_right_only_mode_never_addresses_left_motor(self):
        host.execute(self.link, self.args("--single-right", "right", "20", "--run", "0.2"))
        self.assertIn("MOTOR VEL 2 20 10", self.commands())
        self.assertIn("MOTOR ENABLE 2 0", self.commands())
        self.assertFalse(any(cmd.startswith("MOTOR ENABLE 1") or cmd == "MOTOR STATUS 1"
                             for cmd in self.commands()))

    def test_parameter_bounds_and_single_mode_are_rejected_before_open(self):
        for arguments in [("left", "61"), ("left", "20", "--run", "0"),
                          ("left", "20", "--run", "nan"),
                          ("right", "20", "--acc", "256"),
                          ("--single", "right", "20"),
                          ("--single-right", "left", "20"),
                          ("--single-right", "wheels", "20", "20"),
                          ("position", "left", "2147483648"),
                          ("position", "left", "800", "--rpm", "0")]:
            with self.subTest(arguments=arguments), self.assertRaises(SystemExit) as error:
                self.args(*arguments)
            self.assertEqual(error.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
