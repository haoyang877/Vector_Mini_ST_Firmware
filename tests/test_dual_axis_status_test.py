"""Offline checks of stream guard, sentinel handling and asynchronous target matching."""
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from dual_axis_status_test import check_snapshot, accepted_target, validate_start, validate_waypoints, waypoint_target


class StreamGuardTests(unittest.TestCase):
    def test_full_travel_waypoints_bounds_duration_and_return(self):
        p={'max_speed_deg_s':10.,'points':[{'roll':88.,'pitch':49.5,'duration_s':15.},{'roll':0.,'pitch':0.,'duration_s':15.},{'roll':-88.,'pitch':-15.,'duration_s':15.},{'roll':0.,'pitch':0.,'duration_s':15.}]}
        validate_waypoints(p)
        self.assertEqual(waypoint_target(p,30)[1],{'roll':-88.,'pitch':-15.})
        self.assertIsNone(waypoint_target(p,60))
        import copy
        for key,value in [('roll',89.),('pitch',50.5),('pitch',-16.),('duration_s',4.),('roll',math.nan)]:
            bad=copy.deepcopy(p);bad['points'][0][key]=value
            with self.assertRaises(ValueError):validate_waypoints(bad)
        with self.assertRaises(ValueError):validate_waypoints(dict(p,points=p['points'][:-1]))
        restricted=dict(p,feedback_bounds_deg={'roll':[-89.,89.],'pitch':[-7.,50.5]})
        with self.assertRaises(ValueError):validate_waypoints(restricted)
        restricted['points']=copy.deepcopy(p['points']);restricted['points'][2]['pitch']=-5.
        validate_waypoints(restricted)

    def test_zero_center_requires_bounded_approach_before_normal_cases(self):
        zero={'roll':0.,'pitch':0.};start={'roll':-3.3,'pitch':3.5}
        validate_start(start,zero,True)
        with self.assertRaises(RuntimeError):validate_start(start,zero)
        for invalid in ({'roll':11.,'pitch':0.},{'roll':0.,'pitch':-16.},{'roll':math.nan,'pitch':0.}):
            with self.assertRaises(RuntimeError):validate_start(invalid,zero,True)

    def setUp(self):
        self.good=dict(fault=0,mode=3,position_feedback_rad=.07,speed_feedback_rad_s=0.,
                       iq_feedback_A=.1,iq_reference_A=.1,position_target_rad=.07,
                       position_planned_rad=.07,speed_planned_rad_s=0.)

    def test_motion_stops_on_fault_invalid_mode_measurement_or_limit(self):
        check_snapshot('pitch',self.good,3)
        for key,value in [('fault',1),('mode',2),('position_feedback_rad',None),
                          ('position_planned_rad',None),('speed_planned_rad_s',math.nan),
                          ('iq_reference_A',4),('iq_feedback_A',-4),
                          ('speed_feedback_rad_s',math.radians(46)),('position_feedback_rad',-.3)]:
            with self.subTest(key=key):
                with self.assertRaises(RuntimeError):check_snapshot('pitch',dict(self.good,**{key:value}),3)

    def test_stopped_planner_sentinels_are_expected(self):
        check_snapshot('pitch',dict(self.good,mode=0,position_planned_rad=None,speed_planned_rad_s=None),0)

    def test_target_matches_recent_commands_with_truncation_only(self):
        for sign in (-1,1):
            degrees=sign*5.3;truncated=math.trunc(math.radians(degrees)*1000)/1000
            self.assertTrue(accepted_target(dict(position_target_rad=truncated),[0,degrees]))
            self.assertFalse(accepted_target(dict(position_target_rad=truncated),[0,degrees+1]))
        self.assertFalse(accepted_target(dict(position_target_rad=None),[0]))


if __name__=='__main__':unittest.main()
