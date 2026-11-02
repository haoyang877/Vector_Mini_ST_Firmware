"""验证实际快环的编码器重叠采样顺序，以及全部状态分支均完成事务。"""

from _bsp_test_support import ROOT, arguments, run_c
from run_position_servo_tests import function_source


def main() -> None:
    """用边界桩记录真实MotorAxis_FastTick，不重写生产分支。"""
    args = arguments(__doc__)
    production = (ROOT / "firmware/app/runtime/motor_axis.c").read_text(encoding="utf-8")
    source = (
        r"""
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motor_lifecycle.h"
#include "motor_control.h"
#include "motor_sensing.h"
#include "flux_observer.h"
static MotorConfiguration s_motor_configuration;
static MotorControlState s_motor_state;
static FocState s_foc_state;
static EncoderState s_rotor_encoder;
static FluxObserverState s_flux_observer;
static CurrentOffsetCalibration s_current_offset_calibration;
static MotorState selected_state;
static unsigned steps[32],count;
static void step(unsigned n) {steps[count++]=n;}
#define BspEncoder_Begin() step(1)
#define MotorVoltage_Update(a) ((void)(a),step(2))
#define Foc_UpdateCurrentLimits(a) ((void)(a),step(3))
#define MotorLifecycle_GetState() selected_state
#define MotorCurrentOffsets_Calibrate(a,b,c) ((void)(a),(void)(b),(void)(c),step(4),true)
#define MotorCurrent_Update(a) ((void)(a),step(5))
#define Clarke_Transform(a,b) ((void)(a),(void)(b),step(6))
#define FluxObserver_Update(a,b,c,d) ((void)(a),(void)(b),(void)(c),(void)(d),step(7))
#define Encoder_Update(a) ((void)(a),step(8))
#define MotorLifecycle_BeginFastTick() step(9)
#define MotorLifecycle_WorkEpoch() (step(10),17U)
#define MotorLifecycle_IsCalibrationRunnable() true
static MotorTaskResult result(unsigned n) {step(n);return (MotorTaskResult){MOTOR_TASK_RUNNING,MOTOR_FAULT_NONE};}
#define run_active_calibration_cycle() result(11)
#define MotorControl_Step(a,b,c,d,e) ((void)(a),(void)(b),(void)(c),(void)(d),(void)(e),result(12))
#define BspPwm_SetDuty(a,b,c) ((void)(a),(void)(b),(void)(c),step(13))
static void post(MotorTaskResult r,unsigned epoch) {(void)r;assert(epoch==17);step(14);}
#define MotorLifecycle_PostOutcome(a,b) post(a,b)
"""
        + function_source(production, "MotorAxis_FastTick")
        + r"""
int main(void) {
    unsigned cases=0;
    for(unsigned st=MOTOR_STATE_IDLE;st<=MOTOR_STATE_FAULT;st++)
    for(unsigned complete=0;complete<2;complete++)
    for(unsigned position=0;position<2;position++)
    for(unsigned fault=0;fault<2;fault++) {
        selected_state=(MotorState)st; s_current_offset_calibration.complete=complete;
        s_motor_state.control_mode=position ? MOTOR_CONTROL_POSITION : MOTOR_CONTROL_SPEED;
        s_motor_state.fault_code=fault ? MOTOR_FAULT_ENCODER : MOTOR_FAULT_NONE;
        count=0;MotorAxis_FastTick();
        unsigned expected[16]={1,2,3},n=3;
        if(selected_state==MOTOR_STATE_IDLE) expected[n++]=4;
        if(complete) expected[n++]=5;
        expected[n++]=6;
        if(complete && !position) expected[n++]=7;
        expected[n++]=8;expected[n++]=9;expected[n++]=10;
        if(complete && !fault) expected[n++]=selected_state==MOTOR_STATE_CALIBRATION ? 11 : selected_state==MOTOR_STATE_RUNNING ? 12 : 13;
        expected[n++]=14;
        assert(count==n && memcmp(steps,expected,n*sizeof(unsigned))==0);
        cases++;
    }
    printf("PASS encoder overlap: %u complete/state/mode/fault branches and lifecycle ordering\n",cases);
    return 0;
}
"""
    )
    run_c(args, "encoder_overlap", source, 1)


if __name__ == "__main__":
    main()
