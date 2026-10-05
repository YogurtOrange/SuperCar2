import importlib.util
import struct
import unittest
from pathlib import Path

spec = importlib.util.spec_from_file_location(
    'host_link', Path(__file__).resolve().parents[1] / 'tools' / 'host_link.py')
host = importlib.util.module_from_spec(spec)
spec.loader.exec_module(host)

class ProtocolTests(unittest.TestCase):
    def test_known_frame(self):
        self.assertEqual(host.crc16(b'123456789'), 0x4B37)
        self.assertEqual(host.frame(1, 1, struct.pack('<hhB', 30, 30, 10)).hex(),
                         'aa55010105011e001e000ada7e')

    def test_fragment_noise_and_bad_crc(self):
        decoder = host.Decoder()
        good = host.frame(0x83, 255, b'\x01\x00')
        bad = bytearray(good)
        bad[-1] ^= 1
        self.assertFalse(decoder.feed(b'noise\xaa' + bytes(bad) + good[:3]))
        self.assertEqual(decoder.feed(good[3:]), [(0x83, 255, b'\x01\x00')])

    def test_feedback_units_and_width(self):
        payload = struct.pack('<IhqIIhqII', 1000, -30, -4294967295, 990, 980,
                              30, 65536, 995, 985)
        result = host.Decoder().feed(host.frame(0x81, 0, payload))
        self.assertEqual(len(payload), 40)
        self.assertIn('-4294967295', host.describe(result[0]))
        self.assertIn('65536', host.describe(result[0]))

if __name__ == '__main__':
    unittest.main()
