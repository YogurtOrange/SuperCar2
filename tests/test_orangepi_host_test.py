"""Exercise Orange Pi dual-wheel control without physical serial hardware."""
import contextlib
import importlib.util
import io
from pathlib import Path
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


host = load_module("orangepi_host_test", ROOT / "orangepi_host_test.py")
fixtures = load_module("stm32_host_fixtures", ROOT / "tests" / "test_stm32_host_test.py")
Clock = fixtures.Clock
SimulatedSTM32 = fixtures.SimulatedSTM32


class PerWheelSTM32(SimulatedSTM32):
    """One address can be offline or report stale feedback while moving."""

    def __init__(self, clock):
        super().__init__(clock)
        self.unavailable_addr = None
        self.stale_moving_addr = None
        self.feedback_samples = []

    def reply(self, text):
        parts = text.split()
        if parts[:2] == ["DATA", "FEEDBACK"]:
            addr = int(parts[2])
            if addr == self.unavailable_addr:
                parts[4] = "0"
            if addr == self.stale_moving_addr and self.states[addr]["state"] == 3:
                parts[8] = str(host.FEEDBACK_MAX_AGE_MS)
            self.feedback_samples.append((addr, int(parts[4]), int(parts[8])))
            text = " ".join(parts)
        super().reply(text)


class OrangePiDualWheelTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.port = PerWheelSTM32(self.clock)
        self.link = host.Link(self.port, reply_timeout=0.4)
        self.context = contextlib.ExitStack()
        self.context.enter_context(patch.object(host.time, "monotonic", self.clock.monotonic))
        self.context.enter_context(contextlib.redirect_stdout(io.StringIO()))
        self.context.enter_context(contextlib.redirect_stderr(io.StringIO()))
        self.addCleanup(self.context.close)

    def args(self, *arguments):
        return host.parse_args(["--ready-timeout", "1", *arguments])

    def commands(self):
        return [command for _, command in self.port.sent]

    def assert_shutdown(self):
        commands = self.commands()
        final_stop = max(i for i, command in enumerate(commands) if command == "STOP")
        for addr in (1, 2):
            self.assertIn(f"MOTOR ENABLE {addr} 0", commands[final_stop + 1:])
            self.assertEqual(self.port.states[addr]["rpm"], 0)
            self.assertFalse(self.port.states[addr]["flags"] & 1)
        self.assertFalse(self.link.heartbeat)

    def test_default_uart_and_status_query_both_wheels(self):
        args = self.args("status")
        self.assertEqual(args.port, "/dev/ttyS0")
        self.assertEqual(host.configured_addresses(args), [1, 2])
        host.execute(self.link, args)
        self.assertEqual(self.commands(), ["MOTOR STATUS 1", "MOTOR STATUS 2"])

    def test_independent_signed_speeds_wait_for_both_enables_and_keep_heartbeat(self):
        self.port.enable_delay = 0.3
        host.execute(self.link, self.args("wheels", "13", "-7", "--acc", "12", "--run", "0.4"))
        commands = self.commands()
        self.assertEqual(commands.count("WHEELS 13 -7 12"), 1)
        motion_index = commands.index("WHEELS 13 -7 12")
        motion_time = self.port.sent[motion_index][0]
        for addr in (1, 2):
            enabled_time = next(t for t, command in self.port.sent
                                if command == f"MOTOR ENABLE {addr} 1")
            self.assertGreaterEqual(motion_time - enabled_time, self.port.enable_delay)
            self.assertIn(f"MOTOR STATUS {addr}", commands[:motion_index])
            self.assertIn(f"MOTOR STATUS {addr}", commands[motion_index + 1:])
        beats = [t for t, command in self.port.sent if command == "HEARTBEAT"]
        self.assertGreater(len(beats), 5)
        self.assertLess(max(b - a for a, b in zip(beats, beats[1:])), 0.14)
        self.assertTrue(any(t > motion_time for t in beats))
        self.assert_shutdown()

    def test_one_unavailable_wheel_prevents_any_motion(self):
        self.port.unavailable_addr = 1
        with self.assertRaisesRegex(TimeoutError, "等待电机状态超时"):
            host.execute(self.link, self.args("wheels", "10", "10", "--run", "0.4"))
        commands = self.commands()
        self.assertFalse(any(command.startswith("WHEELS ") for command in commands))
        self.assertNotIn("MOTOR ENABLE 1 1", commands)
        self.assertNotIn("MOTOR ENABLE 2 1", commands)
        self.assertIn((1, 0, 10), self.port.feedback_samples)
        self.assertIn((2, 1, 10), self.port.feedback_samples)
        self.assert_shutdown()

    def test_one_stale_wheel_aborts_and_stops_disables_both(self):
        self.port.stale_moving_addr = 2
        with self.assertRaisesRegex(RuntimeError, "速度反馈过期"):
            host.execute(self.link, self.args("wheels", "-9", "14", "--run", "5"))
        commands = self.commands()
        self.assertEqual(commands.count("WHEELS -9 14 10"), 1)
        motion_time = next(t for t, command in self.port.sent if command == "WHEELS -9 14 10")
        self.assertLess(self.clock.now - motion_time, 5)
        self.assertIn((1, 1, 10), self.port.feedback_samples)
        self.assertIn((2, 1, host.FEEDBACK_MAX_AGE_MS), self.port.feedback_samples)
        self.assertNotIn("CLEAR_FAULT", commands)
        self.assert_shutdown()


if __name__ == "__main__":
    unittest.main()
