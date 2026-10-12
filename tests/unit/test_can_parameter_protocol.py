"""Offline wire and client regression tests; never opens a CAN adapter."""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2] / "tools"))
from project_paths import ROOT, NATIVE_INCLUDE_FLAGS

from pathlib import Path
import sys, struct, unittest, math, time
import json
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools'))
from can_parameter_protocol import encode, decode, Client, Frame


class ProtocolTests(unittest.TestCase):
    def test_speed_feedforward_wire_and_client(self):
        """新前馈参数采用独立float32线路编码，主机支持配对查询。"""
        vectors = json.loads((ROOT / 'tests/vectors/speed_feedforward_v1.json').read_text(encoding='utf-8'))
        for vector in vectors['can_vectors']:
            payload = bytes.fromhex(vector['hex'])
            self.assertAlmostEqual(decode(vector['get_param'], payload), vector['value'], places=6)
            if 'set_param' in vector:
                self.assertEqual(encode(vector['set_param'], vector['value']), payload)
                class Bus:
                    def __init__(self):
                        self.pending = []
                        self.sent = []
                    def receive(self, channel):
                        frames, self.pending = self.pending, []
                        return frames
                    def send(self, channel, identifier, data):
                        self.sent.append((identifier, data))
                        self.pending = [Frame(channel, identifier, payload, 1, time.perf_counter())]
                bus = Bus()
                actual, _ = Client(bus).read(0, vector['node'], vector['set_param'])
                self.assertAlmostEqual(actual, vector['value'], places=6)
                self.assertEqual(bus.sent, [((vector['node'] << 8) | vector['get_param'], bytes(4))])

    def test_software_version_golden_vectors(self):
        """版本查询使用无符号整数编码，独立于既有浮点协议版本。"""
        vector = json.loads((ROOT / 'tests/vectors/software_version_v1.json').read_text(encoding='utf-8'))
        for frame in vector['vectors']:
            self.assertEqual(frame['identifier'], (frame['node'] << 8) | vector['parameter_id'])
            self.assertEqual(bytes.fromhex(frame['request_hex']), bytes(4))
            value = decode(vector['parameter_id'], bytes.fromhex(frame['reply_hex']))
            self.assertEqual(value, vector['version_code'])
            self.assertIsInstance(value, int)
        self.assertEqual(decode(0x6f, bytes.fromhex('80000000')), 0x80000000)
        self.assertEqual(decode(0x6f, bytes.fromhex('ffffffff')), 0xffffffff)
        for payload in (bytes(2), bytes(3), bytes(5)):
            with self.assertRaises(ValueError):
                decode(0x6f, payload)

    def test_signed_vectors_and_legacy_configuration(self):
        for param,value,wire in [(2,-1.25,'fb1e'),(6,-1.25,'fffffb1e'),
                                  (4,6.28,'00000274'),(0x64,20,'41a00000')]:
            self.assertEqual(encode(param,value),bytes.fromhex(wire))
            self.assertAlmostEqual(decode(param+1,bytes.fromhex(wire)),value)
        self.assertEqual(encode(6,math.radians(5)),bytes.fromhex('00000057'))

    def test_invalid_and_wrong_length(self):
        for param,data in [(3,bytes.fromhex('8000')),(7,bytes.fromhex('80000000')),
                           (5,bytes.fromhex('80000000')),(1,struct.pack('>f',math.nan)),
                           (3,bytes(4)),(7,bytes(2))]:
            with self.assertRaises(ValueError):decode(param,data)
        for value in (math.nan,math.inf,33):
            with self.assertRaises(ValueError):encode(2,value)

    def test_client_queries_current_format_and_traces_wire(self):
        class Bus:
            def __init__(self):self.sent=[];self.pending=[]
            def receive(self,ch):
                result,self.pending=self.pending,[]
                return result
            def send(self,ch,identifier,data):
                self.sent.append((identifier,data))
                if identifier==0x411:
                    self.pending=[Frame(ch,0x7f4,bytes(48),1,time.perf_counter()),
                                  Frame(ch,identifier,bytes.fromhex('1770'),2,time.perf_counter())]
        bus=Bus();traces=[];c=Client(bus,trace=lambda *x:traces.append(x))
        self.assertEqual(c.verify(0,4,0x10,6),6)
        self.assertEqual(bus.sent,[(0x411,bytes(4))])
        self.assertEqual(traces[-1][3],bytes.fromhex('1770'))
        c.write(0,4,6,math.radians(5))
        self.assertEqual(bus.sent[-1],(0x406,bytes.fromhex('00000057')))
        c.timeout=.002
        with self.assertRaises(TimeoutError):c.read(0,4,0)


if __name__=='__main__':unittest.main()
