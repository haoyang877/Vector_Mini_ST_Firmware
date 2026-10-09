"""Exercise the actual RTT encoders with host values; never connects to hardware."""
import argparse
from pathlib import Path
import subprocess
from run_position_servo_tests import ROOT, NATIVE_INCLUDE_FLAGS, rtt_frame_fixture


def calibration_fixture():
    # 标定与停机模式沿用当前 11 路相电流/电角度布局。
    fixture = rtt_frame_fixture()
    return fixture.replace('MotorControl.ModeNow = 3;', 'MotorControl.ModeNow = 13;').replace(
        'puts("PASS actual 11-channel RTT:',
        'MotorControl.ModeNow = 15; sample(); near_count(8,1000);\n'
        '    MotorControl.ModeNow = 0; sample(); near_count(8,1000);\n'
        '    puts("PASS calibration 11-channel RTT:')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--cc', required=True)
    ap.add_argument('--out', type=Path, default=ROOT/'outputs/rtt_calibration_tests')
    a = ap.parse_args(); out = a.out.resolve(); out.mkdir(parents=True, exist_ok=True)
    cc = [a.cc] + (['cc'] if Path(a.cc).stem == 'zig' else [])
    for name, fixture in [('calibration', calibration_fixture()), ('servo_v2', rtt_frame_fixture())]:
        src = out/(name+'.c'); exe = out/(name+'.exe'); src.write_text(fixture, encoding='utf-8')
        subprocess.run(cc + NATIVE_INCLUDE_FLAGS + ['-std=c99','-O2',str(src),'-o',str(exe),'-lm'], check=True)
        subprocess.run([str(exe)], check=True)
    # Check actual profile defaults and wire descriptors for both Keil targets.
    (out/'main.h').write_text('', encoding='utf-8')
    src=out/'profiles.c'
    src.write_text('#include <assert.h>\n#include <string.h>\n#include "hw_conf.h"\n'
                   'int main(void) { assert(RTT_TELEMETRY_PROFILE==EXPECTED_PROFILE); '
                   'assert(strlen(RTT_JSCOPE_DESCRIPTOR)==7+2*EXPECTED_CHANNELS); return 0; }\n')
    for hil, profile, channels in [(0,2,10),(1,0,12)]:
        exe=out/('profile_'+str(hil)+'.exe')
        subprocess.run(cc+['-I',str(out)]+NATIVE_INCLUDE_FLAGS+
                       ['-std=c99','-DSERVO_HIL_ENABLE='+str(hil),'-DEXPECTED_PROFILE='+str(profile),
                        '-DEXPECTED_CHANNELS='+str(channels),str(src),'-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
    print('PASS normal/HIL profile defaults and descriptor channel counts')


if __name__=='__main__': main()
