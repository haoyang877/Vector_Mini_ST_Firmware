"""Exercise the actual RTT encoders with host values; never connects to hardware."""
import argparse
from pathlib import Path
import subprocess
from run_position_servo_tests import ROOT, NATIVE_INCLUDE_FLAGS, rtt_frame_fixture


def calibration_fixture():
    source = (ROOT / 'firmware/app/foc_task.c').read_text(encoding='utf-8')
    encoding = source[source.index('#define RTT_SPEED_SCALE'):source.index('/**', source.index('static void RTT_Sampling'))]
    return r'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#define _PI 3.14159265358979323846f
#define RTT_SAMPLE_DIVIDER 10U
#define CASCADE_POSITION_LOOP_DIVIDER 10U
#define RTT_TELEMETRY_SERVO 0U
#define RTT_TELEMETRY_CALIBRATION 1U
#define RTT_TELEMETRY_PROFILE RTT_TELEMETRY_CALIBRATION
#define Position_Mode 3
#define Encoder_DidUpdateVelocity(p) velocity_tick
static bool MotorOuterLoop_IsReady(void) { return true; }
static struct { int ModeNow, ErrorNow, motor_pole_pairs; float speedRef, iqRef, idRef; } MotorControl;
static struct { float theta_elec, vel_mech; unsigned calib_flag, reverse; } OnBoard_Encoder;
static struct { float Iq, Id, Vbus_filt; } FOC;
static struct { float theta_e, omega_e; } Fluxobserver;
static struct { float open_loop_theta; unsigned state; } SensorlessStartup;
static unsigned CalibStep, writes;
static bool skip_next, velocity_tick;
static int16_t wire[8];
static unsigned SEGGER_RTT_Write(unsigned channel, const void *data, unsigned length) {
    assert(channel == 1 && length == 16);
    ++writes;
    if (skip_next) { skip_next = false; return 0; }
    memcpy(wire, data, length); return length;
}
''' + encoding + r'''
static void sample(void) { unsigned i; for (i=0; i<10; ++i) RTT_Sampling(false); }
static void near_count(unsigned i, int expected) { assert(abs((int)wire[i]-expected)<=1); }
int main(void) {
    unsigned before;
    MotorControl.ModeNow=13; MotorControl.motor_pole_pairs=21;
    MotorControl.speedRef=15; MotorControl.iqRef=.5f; MotorControl.idRef=.3f;
    OnBoard_Encoder.theta_elec=.5f*_PI; OnBoard_Encoder.vel_mech=-20;
    Fluxobserver.theta_e=1.5f*_PI; Fluxobserver.omega_e=630;
    SensorlessStartup.open_loop_theta=_PI; SensorlessStartup.state=5; CalibStep=24;
    FOC.Iq=.49f; FOC.Id=-.2f; FOC.Vbus_filt=28.13f;
    sample(); assert(writes==1);
    near_count(0,16384); near_count(1,-16384);
    near_count(2,-1909); near_count(3,2864);
    near_count(4,500); near_count(5,490);
    assert(wire[6]==24 && wire[7]==5);
    OnBoard_Encoder.theta_elec=_PI; sample(); near_count(0,-32768);
    assert(OnBoard_Encoder.reverse==0 && OnBoard_Encoder.calib_flag==0);
    OnBoard_Encoder.reverse=1; OnBoard_Encoder.calib_flag=3; MotorControl.ErrorNow=9;
    sample(); near_count(4,500);
    assert(OnBoard_Encoder.reverse==1 && OnBoard_Encoder.calib_flag==3);
    MotorControl.ModeNow=0; sample(); near_count(2,-1909);
    MotorControl.iqRef=50; FOC.Iq=-50; OnBoard_Encoder.vel_mech=10000;
    sample(); assert(wire[4]==32767 && wire[5]==-32768 && wire[2]==32767);
    MotorControl.motor_pole_pairs=0; OnBoard_Encoder.theta_elec=NAN; FOC.Id=INFINITY;
    FOC.Iq=INFINITY;
    sample(); assert(wire[0]==0 && wire[3]==0 && wire[5]==0);
    assert(rtt_calibration_dropped_frames==0);
    skip_next=true; sample(); sample(); assert(rtt_calibration_dropped_frames==1);
    sample(); assert(rtt_calibration_dropped_frames==1);
    before=writes; for(unsigned i=0;i<10;++i) RTT_Sampling(true); assert(writes==before);
    RTT_Sampling(false); assert(writes==before+1);
    before=writes; velocity_tick=true; sample(); assert(writes==before);
    velocity_tick=false; RTT_Sampling(false); assert(writes==before+1);
    puts("PASS calibration RTT: 8 signals/units, angle wrap, clipping, invalid data, idle output, drop counter, deferral, direction unchanged");
}
'''


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
    for hil, profile, channels in [(0,1,8),(1,0,12)]:
        exe=out/('profile_'+str(hil)+'.exe')
        subprocess.run(cc+['-I',str(out)]+NATIVE_INCLUDE_FLAGS+
                       ['-std=c99','-DSERVO_HIL_ENABLE='+str(hil),'-DEXPECTED_PROFILE='+str(profile),
                        '-DEXPECTED_CHANNELS='+str(channels),str(src),'-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
    print('PASS normal/HIL profile defaults and descriptor channel counts')


if __name__=='__main__': main()
