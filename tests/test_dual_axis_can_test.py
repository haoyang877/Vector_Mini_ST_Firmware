"""Offline envelope and abort-rule checks; these never open a CAN adapter."""
from pathlib import Path
import math
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from dual_axis_can_test import validate_centers, validate_feedback, targets

class DualAxisSafetyTests(unittest.TestCase):
    def test_center_requires_full_excursion_margin(self):
        validate_centers({'roll':0.,'pitch':4.411})
        for centers in ({'roll':81.,'pitch':0.}, {'roll':0.,'pitch':-8.},
                        {'roll':0.,'pitch':math.nan}, {'roll':0.}):
            with self.assertRaises(ValueError): validate_centers(centers)

    def test_all_paths_fit_known_small_envelope(self):
        for case,duration in [('hold',3),('small',24),('same',48),('opposite',24),('sine01',23),('sine02',13)]:
            for i in range(duration*20):
                _,v = targets(case,i/20)
                self.assertLessEqual(max(abs(x) for x in v.values()),5.)
                self.assertAlmostEqual(v['roll'], -v['pitch'] if case=='opposite' else v['pitch'])
            self.assertIsNone(targets(case,duration))

    def test_sine_reference_speed_below_test_limit(self):
        for case in ('sine01','sine02'):
            prev=0.
            for i in range(1,200):
                value=targets(case,i*.05)[1]['roll']
                self.assertLess(abs(value-prev)/.05,15.)
                prev=value

    def test_stop_on_fault_mode_nan_current_speed_position_or_reference(self):
        good=dict(position=0.,speed=0.,iq=0.,mode=3.,fault=0.,accepted=0.)
        validate_feedback('pitch',good,0.,3)
        for key,value in [('mode',2.),('fault',1.),('iq',4.),('iq',-4.),
                          ('speed',math.radians(46)),('position',math.radians(-16)),
                          ('position',math.nan),('accepted',1/360)]:
            with self.subTest(key=key,value=value):
                with self.assertRaises(RuntimeError):
                    validate_feedback('pitch',dict(good,**{key:value}),0.,3)

    def test_negative_time_rejected(self):
        for value in (-1.,math.nan,math.inf):
            with self.assertRaises(ValueError):targets('same',value)

if __name__ == '__main__': unittest.main()
