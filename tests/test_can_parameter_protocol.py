"""Offline wire and client regression tests; never opens a CAN adapter."""
from pathlib import Path
import sys, struct, unittest, math, time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from can_parameter_protocol import encode, decode, Client, Frame


class ProtocolTests(unittest.TestCase):
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
