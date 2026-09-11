from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from can_motor_status import command,decode

class StatusWireTests(unittest.TestCase):
    def test_command_vectors(self):
        self.assertEqual(command(4,20),(0x464,bytes.fromhex('41 a0 00 00')))
        self.assertEqual(command(3,200),(0x364,bytes.fromhex('43 48 00 00')))
        self.assertEqual(command(3,0),(0x364,bytes(4)))
        self.assertEqual(command(4,1),(0x464,bytes.fromhex('3f 80 00 00')))
    def test_reject_invalid(self):
        for value in (2,9,201,19.5,float('nan'),float('inf')):
            with self.assertRaises(ValueError):command(4,value)
        with self.assertRaises(ValueError):command(8,20)
        for identifier,data in [(0x464,bytes(32)),(0x7f4,bytes(24)),(0x7f8,bytes(32))]:
            with self.assertRaises(ValueError):decode(identifier,data)
    def test_golden_status(self):
        v=decode(0x7f4,bytes.fromhex('00070003 fffffb1e 000005dc 000000fa ffffff38 04e2fa24 000002ee ffffffce 197d130b 00000000 00000000 00000000'))
        self.assertEqual(v['fault'],7);self.assertEqual(v['mode'],3)
        self.assertEqual(v['position_target_rad'],-1.25)
        self.assertEqual(v['position_planned_rad'],.75)
        self.assertEqual(v['speed_target_rad_s'],2.5)
        self.assertEqual(v['speed_planned_rad_s'],-.5)
        self.assertEqual(v['iq_feedback_A'],-1.5)
        self.assertEqual(v['temperature_C'],65.25)
        self.assertEqual(v['bus_voltage_V'],48.75)
        self.assertEqual(v['invalid_fields'],[])
    def test_invalid_value_not_falsely_zero(self):
        data=bytearray(48);data[4]=0x80;data[20]=0x80;data[32]=0x80;data[34]=0x80
        result=decode(0x7f3,data)
        self.assertIsNone(result['position_target_rad'])
        self.assertIsNone(result['iq_reference_A'])
        self.assertEqual(result['invalid_fields'],['position_target_rad','iq_reference_A','temperature_C','bus_voltage_V'])
        with self.assertRaises(ValueError):decode(0x7f3,bytes(32))

if __name__=='__main__':unittest.main()
