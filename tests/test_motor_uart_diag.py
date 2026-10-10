import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location(
    "motor_uart_diag", Path(__file__).resolve().parents[1] / "motor_uart_diag.py")
diag = importlib.util.module_from_spec(spec)
spec.loader.exec_module(diag)


class DiagnosticTests(unittest.TestCase):
    def test_queries_only_and_known_packets(self):
        packets = [diag.query_frame(2, code) for code, _ in diag.QUERIES]
        self.assertEqual([p.hex() for p in packets],
                         ["021a6b", "02426c6b", "023a6b", "02356b"])
        with self.assertRaises(ValueError):
            diag.query_frame(2, 0xF6)

    def test_current_configuration_fields(self):
        packet = bytearray(33)
        packet[:4] = bytes((2, 0x42, 33, 21))
        packet[6], packet[18], packet[20], packet[22], packet[-1] = 2, 5, 2, 1, 0x6B
        self.assertIn("这些字段通过", diag.describe_reply(bytes(packet), 2, 0x42))
        packet[22] = 0
        self.assertIn("有字段不匹配", diag.describe_reply(bytes(packet), 2, 0x42))

    def test_captured_x42s_options_and_config(self):
        option = bytes.fromhex("02 1A 00 06 6B")
        config = bytes.fromhex("02 42 21 15 19 02 02 02 00 10 01 00 04 B0 0B B8 "
                               "0F A0 05 07 02 00 01 01 00 08 08 98 07 D0 00 08 6B")
        self.assertIn("1A选项=0x0006", diag.describe_reply(option, 2, 0x1A))
        self.assertIn("本次满足", diag.describe_reply(option, 2, 0x1A))
        self.assertIn("这些字段通过", diag.describe_reply(config, 2, 0x42))

    def test_options_keep_both_bytes_and_short_reply_is_rejected(self):
        self.assertIn("0x6B06", diag.describe_reply(bytes.fromhex("02 1A 6B 06 6B"), 2, 0x1A))
        self.assertIn("不符", diag.describe_reply(bytes.fromhex("02 1A 06 6B"), 2, 0x1A))
        self.assertIn("不满足", diag.describe_reply(bytes.fromhex("02 1A 00 86 6B"), 2, 0x1A))

    def test_different_reply_length_is_exposed_not_silently_accepted(self):
        packet = bytes((2, 0x42, 31)) + bytes(27) + b"\x6b"
        self.assertIn("声明长度=31", diag.describe_reply(packet, 2, 0x42))

    def test_payload_6b_does_not_terminate_frame(self):
        self.assertIn("107 RPM", diag.describe_reply(b"\x02\x35\x00\x00\x6b\x6b", 2, 0x35))

    def test_wrong_address_and_truncation_are_not_success(self):
        self.assertIn("未找到", diag.describe_reply(b"\x01\x1a\x06\x6b", 2, 0x1A))
        self.assertIn("不符", diag.describe_reply(b"\x02\x35\x00", 2, 0x35))


if __name__ == "__main__":
    unittest.main()
