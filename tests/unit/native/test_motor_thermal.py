"""编译真实热模型和组合接口并做无硬件验证，不导入探针库。"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from project_paths import NATIVE_INCLUDE_FLAGS, ROOT
from run_position_servo_tests import function_source


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/motor_thermal_native")
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    (out / "main.h").write_text("#include <stdint.h>\n#include <stddef.h>\n", encoding="utf-8")
    compiler = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    base = (
        compiler
        + ["-std=c99", "-O2", "-UNDEBUG", "-Wall", "-Wextra", "-Werror", "-I", str(out)]
        + NATIVE_INCLUDE_FLAGS
    )
    logs = []
    checks = {}

    def run(name: str, files: list[Path], defines: list[str] | tuple[str, ...] = ()) -> None:
        exe = out / (name + ".exe")
        commands = [
            base + list(defines) + [str(f) for f in files] + ["-lm", "-o", str(exe)],
            [str(exe)],
        ]
        for command in commands:
            result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
            logs.append(result.stdout + result.stderr)
            (out / "tests.log").write_text("".join(logs), encoding="utf-8")
            print(logs[-1], end="", flush=True)
            if result.returncode != 0:
                raise RuntimeError(name + " failed: " + str(result.returncode))
        checks[name] = "passed"

    core = ROOT / "firmware/motor/protection/motor_thermal.c"
    control = ROOT / "firmware/app/motor_thermal_control.c"
    run("thermal_model", [ROOT / "tests/unit/motor_thermal_test.c", core])
    fixture = r"""
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "motor_thermal_control.h"
#include "data_type.h"
static MotorThermalDiagnostics snapshot(void) {
    MotorThermalDiagnostics s;assert(MotorThermalControl_GetDiagnostics(&s));return s;
}
#define g_motor_thermal_diagnostics snapshot()
static uint32_t clock_ms;
static void step(float a, float b, float c) {
    unsigned i; ++clock_ms;
    for(i=0;i<20;i++) MotorThermalControl_Sample(a,b,c,false,clock_ms);
}
int main(void) {
    float old_heat; unsigned i;
    MotorThermalControl_Init(0);
    assert(!MotorThermalControl_CanRunMode(Current_Mode));
    assert(!MotorThermalControl_CanRunMode(Calib_EncoderObserver));
    assert(!MotorThermalControl_CanRunMode(Voltage_OpenLoop));
    assert(MotorThermalControl_CanRunMode(Motor_Disable));
    assert(MotorThermalControl_CanRunMode(Save_Param));
    assert(MotorThermalControl_CurrentLimit(7)==0);
    old_heat=g_motor_thermal_diagnostics.model.max_heat;
    MotorThermalControl_Init(0); assert(g_motor_thermal_diagnostics.model.max_heat==old_heat);
    for(i=0;i<450000 && g_motor_thermal_diagnostics.model.max_heat>.5f;i++) step(0,0,0);
#if MOTOR_THERMAL_PARAMETERS_CONFIRMED == 1
    assert(MotorThermalControl_CanClearFault());
    assert(!MotorThermalControl_CanRunMode(Current_Mode));
    old_heat=g_motor_thermal_diagnostics.model.max_heat;
    assert(MotorThermalControl_ClearFault());
    assert(g_motor_thermal_diagnostics.model.max_heat==old_heat);
    assert(MotorThermalControl_CurrentLimit(9)==7);
    assert(MotorThermalControl_CurrentLimit(2)==2);
    assert(MotorThermalControl_CanRunMode(Current_Mode));
    for(i=0;i<5000 && !g_motor_thermal_diagnostics.model.derating;i++) step(7,-3.5f,-3.5f);
    assert(g_motor_thermal_diagnostics.model.derating);
    assert(MotorThermalControl_CanRunMode(Current_Mode));
    assert(!MotorThermalControl_CanRunMode(Calib_EleAngelOffset));
    assert(!MotorThermalControl_Guard(Calib_EleAngelOffset));
    assert(!MotorThermalControl_CanRunMode(Current_Mode));
    assert(!MotorThermalControl_CanClearFault());
    assert(MotorThermalControl_Guard(Save_Param));
#else
    assert(!MotorThermalControl_CanClearFault());
    assert(!MotorThermalControl_ClearFault());
    assert(MotorThermalControl_CurrentLimit(7)==0);
#endif
    puts("PASS composition: hot boot, idempotent init, parameter gate, cold-clear request, cap, calibration abort, no automatic restart, clock wrap and dropout");
    return 0;
}
"""
    path = out / "control.c"
    path.write_text(fixture, encoding="utf-8")
    for confirmed in (0, 1):
        run(
            "thermal_control_" + str(confirmed),
            [path, core, control],
            ["-DMOTOR_THERMAL_PARAMETERS_CONFIRMED=" + str(confirmed)],
        )

    # 每个新进程从静态零初始化模拟软件复位，生产接口没有重置热历史命令。
    path = out / "clock.c"
    path.write_text(
        r"""
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include "motor_thermal_control.h"
#include "data_type.h"
int main(void){
 MotorThermalDiagnostics d;
 uint32_t now=UINT32_MAX-1;
 MotorThermalControl_Init(now);
 assert(MotorThermalControl_GetDiagnostics(&d) && d.model.max_heat>10);
 assert(!MotorThermalControl_CanRunMode(Current_Mode));
 for(unsigned n=0;n<2;n++){
  ++now;for(unsigned i=0;i<20;i++)MotorThermalControl_Sample(0,0,0,false,now);
 }
 assert(MotorThermalControl_GetDiagnostics(&d) && d.model.valid);
 assert(MotorThermalControl_CheckWatchdog(now+100,true));
 assert(!MotorThermalControl_CheckWatchdog(now+10,false));
 for(unsigned i=0;i<20;i++)MotorThermalControl_Sample(0,0,0,false,now+11);
 assert(MotorThermalControl_GetDiagnostics(&d) && !d.model.valid);
 assert(!MotorThermalControl_CanClearFault());
 assert(d.watchdog_fault_pending);
 puts("PASS new process restart: worst-heat init, uint32 clock wrap, idle maintenance gap and irreversible active ADC-loss event");return 0;
}
""",
        encoding="utf-8",
    )
    run("thermal_clock", [path, core, control], ["-DMOTOR_THERMAL_PARAMETERS_CONFIRMED=1"])
    source = (ROOT / "firmware/app/foc_task.c").read_text(encoding="utf-8")
    clear = function_source(source, "Task_TryClearError")
    path = out / "clear.c"
    path.write_text(
        r"""
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include "data_type.h"
#include <math.h>
static MotorControl_TypeDef MotorControl;
static bool allowed;static unsigned attempts;
static bool MotorThermalControl_ClearFault(void){attempts++;return allowed;}
static void Set_ErrorNow(ErrorNow_TypeDef e){MotorControl.ErrorNow=e;}
static void Set_ModeNow(ModeNow_TypeDef m){MotorControl.ModeNow=m;}
"""
        + clear
        + r"""
int main(void){
 MotorControl.ModeNow=Clear_Error;MotorControl.ErrorNow=MotorThermal_Protection;
 Task_TryClearError();assert(MotorControl.ModeNow==0 && MotorControl.ErrorNow==16 && attempts==1);
 MotorControl.ErrorNow=High_Temprature;Task_TryClearError();assert(MotorControl.ErrorNow==0 && attempts==2);
 allowed=true;MotorControl.ErrorNow=MotorThermal_Protection;Task_TryClearError();
 assert(MotorControl.ErrorNow==0 && MotorControl.ModeNow==0);
 puts("PASS actual clear handler: hot thermal fault cannot clear, other fault clear preserves heat, cold clear stays mode0");return 0;
}
""",
        encoding="utf-8",
    )
    run("thermal_clear", [path])

    # 真实外环限幅：保留Kp/Ki和配置电流，下降时已有Ui立即受新边界约束。
    run_source = (ROOT / "firmware/app/foc_run.c").read_text(encoding="utf-8")
    speed = function_source(run_source, "SpeedMode_UpdateControl")
    path = out / "speed.c"
    path.write_text(
        r"""
#include <assert.h>
#include <stdio.h>
#include <math.h>
#include "data_type.h"
#include "foc_pid.h"
#include "motor_thermal.h"
#define Speed_Ts .0005f
"""
        + speed
        + r"""
int main(void){
 MotorControl_TypeDef m={0};PI_Controller_TypeDef pi={0};
 m.current_limit=6;m.thermal_limit_active=true;m.thermal_current_limit_a=1;
 m.speed_Kp=.06f;m.speed_Ki=.6f;m.speedRef=.2f;
 for(unsigned i=0;i<3000;i++){SpeedMode_UpdateControl(&m,&pi,0);assert(fabsf(m.iqRef)<=1.00001f);}
 assert(pi.Ui>.02f);
 m.thermal_current_limit_a=.2f;SpeedMode_UpdateControl(&m,&pi,0);
 assert(fabsf(m.iqRef)<=.20001f && fabsf(pi.Ui)<=.2f/6+.00001f);
 m.thermal_current_limit_a=0;SpeedMode_UpdateControl(&m,&pi,0);
 assert(m.iqRef==0 && pi.Ui==0);
 assert(m.current_limit==6 && pi.Kp==.06f && pi.Ki==.6f);
 puts("PASS actual speed loop: thermal cap, existing integral tracking, zero cap and unchanged gain/storage scale");return 0;
}
""",
        encoding="utf-8",
    )
    run("thermal_speed", [path, ROOT / "firmware/motor/foc/foc_pid.c"])

    algorithm = (ROOT / "firmware/motor/foc/foc_algorithm.c").read_text(encoding="utf-8")
    current = function_source(algorithm, "FOC_Current")
    reset = function_source(algorithm, "FOC_CurrentController_Reset")
    path = out / "current.c"
    path.write_text(
        r"""
#include <assert.h>
#include <stdio.h>
#include <math.h>
#include "data_type.h"
#include "foc_pid.h"
#include "motor_thermal.h"
#define FAST_PROFILE_BEGIN(x) ((void)0)
#define FAST_PROFILE_END(x) ((void)0)
#define Current_Ts .00005f
#define FOC_MAX_MODULATION .9f
#define UTILS_LP_FAST(x,y,a) ((x)+=(a)*((y)-(x)))
typedef struct {
 float Ia,Ib,Ic,Ialpha,Ibeta,Id,Iq,Id_filt,Iq_filt,Vbus_filt,Vd,Vq;
 float mod_d,mod_q,mod_alpha,mod_beta,duty,Ibus,Ibus_filt,Power_filt;
 float dtc_a,dtc_b,dtc_c;unsigned sector;PI_Controller_TypeDef id_pi,iq_pi;
} FOC_TypeDef;
static unsigned writes;static ErrorNow_TypeDef error;
static void Set_ErrorNow(ErrorNow_TypeDef e){error=e;}
static void Clarke_Transform(float a,float b,float c,float *x,float *y){*x=a;*y=(b-c)*.577350269f;}
static void Park_Transform(float x,float y,float p,float *d,float *q){(void)p;*d=x;*q=y;}
static void Inverse_Park_Transform(float d,float q,float p,float *x,float *y){(void)p;*x=d;*y=q;}
static void FOC_SetVoltageModulation(FOC_TypeDef *f,float d,float q){f->mod_d=d;f->mod_q=q;}
static void SVM_SectorJudge(float x,float y,float *a,float *b,float *c,unsigned *s){(void)x;(void)y;*a=*b=*c=.5f;*s=1;}
static void Set_A_Duty(float a){(void)a;writes++;}
static void Set_B_Duty(float a){(void)a;writes++;}
static void Set_C_Duty(float a){(void)a;writes++;}
"""
        + reset
        + current
        + r"""
int main(void){
 MotorControl_TypeDef m={0};FOC_TypeDef f={0};
 m.current_limit=7;m.thermal_limit_active=true;m.thermal_current_limit_a=5;m.idRef=6;m.iqRef=8;
 f.Vbus_filt=33;m.id_Kp=m.iq_Kp=1;
 FOC_Current(&f,&m,0,0);assert(fabsf(m.idRef-3)<.0001f && fabsf(m.iqRef-4)<.0001f && writes==3);
 assert(m.current_limit==7);
 m.iqRef=NAN;f.iq_pi.Ui=5;
 FOC_Current(&f,&m,0,0);assert(error==16 && writes==3 && m.iqRef==0 && m.idRef==0 && f.iq_pi.Ui==0);
 puts("PASS actual current execution: total Id/Iq vector cap before PI/PWM, preserved config, invalid reference resets PI without another PWM write");return 0;
}
""",
        encoding="utf-8",
    )
    run("thermal_current", [path, core, ROOT / "firmware/motor/foc/foc_pid.c"])
    sense = (ROOT / "firmware/motor/foc/foc_sensing.c").read_text(encoding="utf-8")
    path = out / "sensing.c"
    path.write_text(
        r"""
#include <assert.h>
#include <stdio.h>
#include <math.h>
#include "data_type.h"
typedef struct { float Ia,Ib,Ic; } FOC_TypeDef;
static struct { int a,b,c; } adc;
#define CURRENT_ADC (&adc)
#define IA_ADC_CHANNEL a
#define IB_ADC_CHANNEL b
#define IC_ADC_CHANNEL c
#define SENSING_CURR_FACTOR .01f
#define CURRENT_OVERCURRENT_TRIP_A 18
#define OVERCURRENT_CONFIRM_CYCLES 4
static ErrorNow_TypeDef error;
static unsigned motor_hw_current_sample_sector(void){return 0;}
static float fast_abs(float x){return fabsf(x);}
static void Set_ErrorNow(ErrorNow_TypeDef e){error=e;}
"""
        + function_source(sense, "Current_Reconstruct")
        + function_source(sense, "Current_Cal")
        + r"""
int main(void){
 MotorControl_TypeDef m={0};FOC_TypeDef f={0};
 m.A_Offset=m.B_Offset=m.C_Offset=2048;adc.a=2040;adc.b=2049;adc.c=2047;
 assert(Current_Cal(&f,&m) && error==0);assert(fabsf(f.Ia-.08f)<.000001f);
 m.A_Offset=0;f.Ia=6;assert(!Current_Cal(&f,&m) && error==CurrentOffset_Error && f.Ia==6);
 puts("PASS actual current sensing: invalid offset reports false, stale current cannot be consumed as cooling feedback");return 0;
}
""",
        encoding="utf-8",
    )
    run("thermal_sensing", [path])
    # 真实快环准入及独立监督停机：不用复制实现来验证同拍调用顺序。
    guard = function_source(source, "Task_ApplyThermalGuard")
    supervisor = function_source(source, "FOC1kHzSupervisor")
    before_control = source.index("Task_ApplyThermalGuard();")
    if not before_control < source.index("MotorOuterLoop_FastTick(&MotorControl"):
        raise RuntimeError("Thermal guard must precede control/PWM")
    path = out / "dispatch.c"
    path.write_text(
        r"""
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include "data_type.h"
#include <math.h>
static MotorControl_TypeDef MotorControl;
typedef struct {float Ia,Ib,Ic;} FOC_TypeDef;
static FOC_TypeDef FOC;static bool can_run,confirmed,watchdog_ok;static unsigned stops;
static float MotorThermalControl_CurrentLimit(float configured){return configured;}
static bool MotorThermalControl_Guard(unsigned mode){(void)mode;return can_run;}
static bool MotorThermalControl_ParametersConfirmed(void){return confirmed;}
static bool MotorThermalControl_CheckWatchdog(unsigned ms,bool off){(void)ms;(void)off;return watchdog_ok;}
static bool motor_hw_phase_outputs_disabled(void){return false;}
static unsigned time_hw_now_ms(void){return 123;}
static void Temperature_Update(FOC_TypeDef *p){(void)p;}
static void Stop_PWM_Generate(void){stops++;}
static void Set_ModeNow(ModeNow_TypeDef m){MotorControl.ModeNow=m;}
static void Set_ErrorNow(ErrorNow_TypeDef e){MotorControl.ErrorNow=e;}
"""
        + guard
        + supervisor
        + r"""
int main(void){
 MotorControl.ModeNow=Calib_EncoderObserver;MotorControl.iqRef=6;MotorControl.idRef=.5f;
 Task_ApplyThermalGuard();assert(MotorControl.ModeNow==0 && MotorControl.ErrorNow==MotorParam_Error && MotorControl.iqRef==0 && MotorControl.idRef==0);
 MotorControl.ErrorNow=0;MotorControl.ModeNow=Position_Mode;confirmed=true;
 Task_ApplyThermalGuard();assert(MotorControl.ErrorNow==16 && MotorControl.ModeNow==0);
 MotorControl.ErrorNow=High_Temprature;Task_ApplyThermalGuard();assert(MotorControl.ErrorNow==10);
 can_run=true;MotorControl.ModeNow=Current_Mode;MotorControl.ErrorNow=0;Task_ApplyThermalGuard();assert(MotorControl.ModeNow==Current_Mode);
 MotorControl.current_limit=7;MotorControl.ModeNow=Voltage_OpenLoop;FOC.Ia=8;FOC.Ib=-4;FOC.Ic=-4;
 Task_ApplyThermalGuard();assert(MotorControl.ModeNow==0 && MotorControl.ErrorNow==16);
 MotorControl.ModeNow=Current_Mode;MotorControl.ErrorNow=0;
 FOC1kHzSupervisor();assert(stops==1 && MotorControl.ModeNow==0 && MotorControl.ErrorNow==16);
 MotorControl.ErrorNow=10;FOC1kHzSupervisor();assert(stops==2 && MotorControl.ErrorNow==10);
 watchdog_ok=true;FOC1kHzSupervisor();assert(stops==2);
 puts("PASS actual fast dispatch: same-tick rejection before PWM, preserved first fault, independent ADC-loss shutdown");return 0;
}
""",
        encoding="utf-8",
    )
    run("thermal_dispatch", [path])
    (out / "summary.json").write_text(
        json.dumps({"status": "passed", "checks": checks, "hardware_actions": False}, indent=2),
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
