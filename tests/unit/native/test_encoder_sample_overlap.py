"""验证实际快环的编码器重叠采样顺序，以及全部状态分支均完成事务。"""

from _bsp_test_support import ROOT, arguments, run_c
from run_position_servo_tests import function_source


def main() -> None:
    """用边界桩记录真实采样/控制阶段，不重写生产分支。"""
    args = arguments(__doc__)
    production = (ROOT / "firmware/app/motor_axis.c").read_text(encoding="utf-8")
    source = (
        r"""
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motor_state_machine.h"
#include "motor_control.h"
#include "motor_sensing.h"
#include "flux_observer.h"
#include "motor_axis.h"
static MotorAxis axis;
static unsigned steps[32],count;
static void step(unsigned n) {steps[count++]=n;}
#define BspEncoder_Begin() step(1)
#define MotorVoltage_Update(a) ((void)(a),step(2))
#define Foc_UpdateCurrentLimits(a) ((void)(a),step(3))
#define MotorCurrentOffsets_Calibrate(a,b,c) ((void)(a),(void)(b),(void)(c),step(4),true)
#define MotorCurrent_Update(a) ((void)(a),step(5))
#define Clarke_Transform(a,b) ((void)(a),(void)(b),step(6))
#define FluxObserver_Update(a,b,c,d) ((void)(a),(void)(b),(void)(c),(void)(d),step(7))
#define MotorCalibration_PublishLutFast(cal) assert(!(cal)->lut_request_epoch)
#define Encoder_Update(a) ((void)(a),step(8))
#define MotorStateMachine_BeginFastTick(machine) ((machine)->epoch=17U,step(9))
#define MotorStateMachine_IsCalibrationRunnable(machine) ((machine)->state == MOTOR_STATE_CALIBRATION)
static MotorTaskResult result(unsigned n) {++axis.state_machine.epoch;step(n);return (MotorTaskResult){MOTOR_TASK_RUNNING,MOTOR_FAULT_NONE};}
#define MotorCalibration_StepFast(cal) result(11)
#define MotorControl_Step(a,b,c,d,e) ((void)(a),(void)(b),(void)(c),(void)(d),(void)(e),result(12))
static void set_duty(float a,float b,float c) {assert(a==0.5f && b==0.5f && c==0.5f);step(13);}
static const MotorAxisHardware hardware = {.power.set_duty=set_duty};
static void post(MotorTaskResult r,unsigned epoch) {(void)r;assert(epoch==17);step(14);}
#define MotorStateMachine_FinishFastTick(machine,a,b) post(a,b)
"""
        + function_source(production, "MotorAxis_SampleFast")
        + function_source(production, "MotorAxis_ControlFast")
        + r"""
int main(void) {
    axis.hardware=&hardware;
    unsigned cases=0;
    for(unsigned st=MOTOR_STATE_IDLE;st<=MOTOR_STATE_FAULT;st++)
    for(unsigned complete=0;complete<2;complete++)
    for(unsigned mode=0;mode<3;mode++)
    for(unsigned fault=0;fault<2;fault++) {
        axis.state_machine.state=(MotorState)st; axis.current_offsets.complete=complete;
        axis.motor.control_mode=mode==0 ? MOTOR_CONTROL_POSITION : mode==1 ? MOTOR_CONTROL_SPEED : MOTOR_CONTROL_CURRENT;
        axis.motor.fault_code=fault ? MOTOR_FAULT_ENCODER : MOTOR_FAULT_NONE;
        count=0;MotorAxis_SampleFast(&axis);
    MotorAxis_ControlFast(&axis);
        unsigned expected[16]={1,2,3},n=3;
        if(axis.state_machine.state==MOTOR_STATE_IDLE) expected[n++]=4;
        if(complete) expected[n++]=5;
        expected[n++]=6;
        if(complete && mode==2) expected[n++]=7;
        expected[n++]=8;expected[n++]=9;
        if(complete && !fault) expected[n++]=axis.state_machine.state==MOTOR_STATE_CALIBRATION ? 11 : axis.state_machine.state==MOTOR_STATE_RUNNING ? 12 : 13;
        expected[n++]=14;
        assert(count==n && memcmp(steps,expected,n*sizeof(unsigned))==0);
        cases++;
    }
    printf("PASS encoder overlap: %u complete/state/mode/fault branches and state machine ordering\n",cases);
    return 0;
}
"""
    )
    run_c(args, "encoder_overlap", source, 1)


if __name__ == "__main__":
    main()
