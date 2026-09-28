"""装配配置的 CAN/Flash 黄金向量检查，不访问硬件。"""

import json
import struct
import sys
import time
import unittest
import zlib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from can_parameter_protocol import Client, Frame, decode, encode
from project_paths import ROOT


class MotorLoadProtocolTests(unittest.TestCase):
    """使用独立 CRC 实现及模拟总线检查字节约定。"""

    def test_golden_records_and_wire(self):
        vectors = json.loads(
            (ROOT / "tests/vectors/motor_load_profile_v1.json").read_text(encoding="utf-8")
        )
        for item in vectors["records"]:
            record = bytes.fromhex(item["little_endian_hex"])
            magic, version, flags, crc = struct.unpack("<4I", record)
            self.assertEqual((magic, version, flags), (0x31504D44, 1, item["flags"]))
            self.assertEqual(zlib.crc32(record[:12]), crc)
        for item in vectors["can"]:
            wire = bytes.fromhex(item["big_endian_hex"])
            self.assertEqual(encode(item["parameter"], item["value"]), wire)
            self.assertEqual(decode(item["parameter"], wire), item["value"])

    def test_client_supports_load_profile_pair(self):
        class Bus:
            def __init__(self):
                self.sent = []
                self.pending = []

            def receive(self, channel):
                result, self.pending = self.pending, []
                return result

            def send(self, channel, identifier, data):
                self.sent.append((identifier, data))
                if identifier == 0x369:
                    self.pending = [
                        Frame(
                            channel, identifier, bytes.fromhex("3f800000"), 0, time.perf_counter()
                        )
                    ]

        bus = Bus()
        client = Client(bus)
        client.write(0, 3, 0x68, 1)
        self.assertEqual(client.verify(0, 3, 0x68, 1), 1)
        self.assertEqual(bus.sent, [(0x368, bytes.fromhex("3f800000")), (0x369, bytes(4))])


if __name__ == "__main__":
    unittest.main()
