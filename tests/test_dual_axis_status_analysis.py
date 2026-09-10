"""Acceptance tests using synthetic fixtures only (not bench measurements)."""
import csv
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from analyze_dual_axis_status import analyze


class AnalysisTests(unittest.TestCase):
    def fixture(self,path,error):
        (path/'run.json').write_text(json.dumps(dict(case='hold',success=True,feedback_hz=20,centers={'roll':0,'pitch':0})))
        with (path/'commands.csv').open('w',newline='') as f:
            w=csv.writer(f);w.writerow(['time_s','case_time_s','cycle','segment','axis','target_deg'])
            for i in range(60):
                for a in ['roll','pitch']:w.writerow([i*.05,i*.05,i,'0',a,0])
        with (path/'feedback.csv').open('w',newline='') as f:
            w=csv.writer(f);w.writerow(['time_s','case_time_s','segment','axis','position_feedback_rad','position_planned_rad','speed_feedback_rad_s','iq_feedback_A','iq_reference_A'])
            for i in range(60):
                for a in ['roll','pitch']:w.writerow([i*.05+.01,i*.05+.01,'0',a,math.radians(error if a=='pitch' else 0),0,0,0,0])

    def test_stationary_trace_passes_and_reports_axis_difference(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);self.fixture(p,.1);r=analyze(p,False)
            self.assertTrue(r['acceptance_pass'])
            self.assertAlmostEqual(r['axis_relative_rmse_deg'],.1)

    def test_one_axis_tail_error_blocks_next_stage(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);self.fixture(p,.5);r=analyze(p,False)
            self.assertFalse(r['acceptance_pass']);self.assertTrue(r['axes']['roll']['pass'])
            self.assertFalse(r['axes']['pitch']['pass'])


if __name__=='__main__':unittest.main()
