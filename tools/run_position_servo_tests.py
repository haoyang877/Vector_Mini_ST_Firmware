"""Compile and execute the real C servo; optionally replay recorded sensor inputs.

Example: python tools/run_position_servo_tests.py --cc /path/to/zig.exe
Replay fixes the old measured motion as an input, so it is NOT a plant simulation.
"""
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def function_source(source, name):
    # A call may precede the definition (e.g. the board fast ADC adapter).
    match = re.search(r'^[ \t]*(?:[A-Za-z_]\w*[ \t]+)+\**' + re.escape(name) +
                      r'\s*\([^;{}]*\)\s*\{', source, re.M)
    if match is None:
        raise ValueError('Function definition not found: ' + name)
    start, brace = match.start(), match.end() - 1
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def encoder_startup_fixture():
    source = (ROOT / "Bsp/encoder.c").read_text(encoding="utf-8")
    return r'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "../software/config/motor_axis_profile.h"
#define _2PI 6.2831853072f
#define ENCODER_Q15_CPR 65536UL
#define ENCODER_Q15_HALF_TURN 32768
#define ENC_CALIB_MECHANICAL_ZERO 4U
typedef struct {
    MotorAxisProfile axis_profile;
    bool axis_profile_valid;
    int motor_pole_pairs;
} MotorControl_TypeDef;
typedef struct {
    uint16_t raw_q15, directed_q15, linearized_q15, previous_linearized_q15;
    uint16_t electrical_zero_q15, mechanical_zero_q15;
    uint8_t calib_flag;
    bool has_valid_sample;
    int64_t shadow_q15, mechanical_zero_shadow_q15, velocity_shadow_q15;
    float theta_elec, theta_mech;
} Encoder_TypeDef;
static uint16_t sample;
static bool sample_ok = true;
static bool Encoder_ReadTle5012BFrame(Encoder_TypeDef *e, uint16_t *out, bool started)
{ (void)e; (void)started; *out = sample; return sample_ok; }
static uint16_t Encoder_ApplyDirectionQ15(Encoder_TypeDef *e, uint16_t v)
{ (void)e; return v; }
static uint16_t Encoder_ApplyLinearizationQ15(Encoder_TypeDef *e, uint16_t v)
{ (void)e; return v; }
static void Encoder_ResetVelocity(Encoder_TypeDef *e)
{ e->velocity_shadow_q15 = e->shadow_q15; }
static void Encoder_UpdateVelocity2kHz(Encoder_TypeDef *e, uint32_t poles)
{ (void)e; (void)poles; }
''' + "\n".join(function_source(source, name) for name in
    ["Encoder_UpdateAngles", "Encoder_CompleteSample"]) + r'''
int main(void) {
    MotorControl_TypeDef m = {0};
    Encoder_TypeDef e = {0};
    m.motor_pole_pairs = 7;
    assert(MotorAxisProfile_Create(&m.axis_profile, "roll", -1.57079633f, 1.57079633f, .785398163f));
    m.axis_profile_valid = true;
    e.mechanical_zero_q15 = 65000; e.electrical_zero_q15 = 1234;
    e.calib_flag = ENC_CALIB_MECHANICAL_ZERO;
    sample = 4000;
    sample_ok = false; Encoder_CompleteSample(&m, &e, false);
    assert(!e.has_valid_sample);
    sample_ok = true; Encoder_CompleteSample(&m, &e, false);
    assert(e.shadow_q15 - e.mechanical_zero_shadow_q15 == 4536);
    assert(e.velocity_shadow_q15 == e.shadow_q15);
    assert(e.mechanical_zero_q15 == 65000 && e.electrical_zero_q15 == 1234);
    assert(MotorAxisProfile_AllowsPosition(&m.axis_profile, true, e.theta_mech, e.theta_mech));
    sample = 65530; Encoder_CompleteSample(&m, &e, true);
    assert(e.shadow_q15 == 65530);
    sample = 10; Encoder_CompleteSample(&m, &e, true);
    assert(e.shadow_q15 == 65546); /* continuous across wrap after startup */
    memset(&e, 0, sizeof(e)); e.mechanical_zero_q15 = 500;
    e.calib_flag = ENC_CALIB_MECHANICAL_ZERO; sample = 65000;
    Encoder_CompleteSample(&m, &e, false);
    assert(e.shadow_q15 - e.mechanical_zero_shadow_q15 == -1036);
    /* Invalid/unconfigured axes and uncalibrated encoders keep legacy behavior. */
    e.has_valid_sample = false; m.axis_profile_valid = false;
    Encoder_CompleteSample(&m, &e, false);
    assert(e.shadow_q15 - e.mechanical_zero_shadow_q15 == 64500);
    e.has_valid_sample = false; m.axis_profile_valid = true; e.calib_flag = 0;
    Encoder_CompleteSample(&m, &e, false);
    assert(e.shadow_q15 == 65000 && e.mechanical_zero_shadow_q15 == 0);
    /* Do not map a real out-of-travel position into the allowed range. */
    e.has_valid_sample = false; e.calib_flag = ENC_CALIB_MECHANICAL_ZERO;
    e.mechanical_zero_q15 = 0; sample = 30000;
    Encoder_CompleteSample(&m, &e, false);
    assert(!MotorAxisProfile_AllowsPosition(&m.axis_profile, true, e.theta_mech, 0));
    puts("PASS actual encoder startup: calibrated wrap, continuity, invalid sample/axis, preserved zero and travel limits");
    return 0;
}
'''


def encoder_fixture():
    source = (ROOT / "Bsp/encoder.c").read_text(encoding="utf-8")
    header = (ROOT / "Bsp/encoder.h").read_text(encoding="utf-8")
    macros = []
    for name, text in [("ENCODER_Q15_CPR", header), ("ENCODER_VELOCITY_WINDOW", header),
                       ("ENCODER_VELOCITY_ZERO_THRESHOLD_Q15", source)]:
        macros.append(re.search(r"^#define\s+" + name + r"\s+[^\r\n]+", text, re.M)[0])
    # Only peripheral-independent estimator functions are compiled here. The
    # hardware build separately checks the actual full Encoder_TypeDef layout.
    return """
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#define _2PI 6.2831853072f
#define SPEED_LOOP_DIVIDER 10U
#define Speed_Ts 0.0005f
""" + "\n".join(macros) + """
typedef struct {
    int32_t velocity_delta_history[ENCODER_VELOCITY_WINDOW];
    uint8_t velocity_divider, velocity_history_index, velocity_sample_count;
    uint16_t bad_frame_streak;
    int32_t velocity_delta_sum;
    bool velocity_ready;
    int64_t velocity_shadow_q15, shadow_q15;
    float vel_mech, vel_elec, vel_mech_continuous;
} Encoder_TypeDef;
""" + "\n".join(function_source(source, name) for name in
    ["Encoder_ResetVelocity", "Encoder_UpdateVelocity2kHz", "Encoder_GetMecVel",
     "Encoder_GetMecVelContinuous", "Encoder_DidUpdateVelocity"]) + """
int main(void) {
    Encoder_TypeDef e = {0};
    int k, n;
    Encoder_ResetVelocity(&e);
    assert(!Encoder_DidUpdateVelocity(&e));
    for (k=0; k<400; ++k) {
        if ((k % 4)==0) e.shadow_q15++;
        for(n=0; n<10; ++n) {
            Encoder_UpdateVelocity2kHz(&e, 7);
            assert(Encoder_DidUpdateVelocity(&e) == (n == 9));
        }
    }
    e.bad_frame_streak = 1;
    assert(!Encoder_DidUpdateVelocity(&e));
    e.bad_frame_streak = 0;
    assert(Encoder_GetMecVel(&e)==0);
    assert(fabsf(Encoder_GetMecVelContinuous(&e)-_2PI/(65536.0f*.002f))<.00001f);
    Encoder_ResetVelocity(&e);
    assert(Encoder_GetMecVelContinuous(&e)==0 && !e.velocity_ready);
    for (k=0; k<400; ++k) {
        if ((k % 4)==0) e.shadow_q15--;
        for(n=0; n<10; ++n) Encoder_UpdateVelocity2kHz(&e, 7);
    }
    assert(Encoder_GetMecVel(&e)==0 && Encoder_GetMecVelContinuous(&e)<-.04f);
    for(k=0;k<100;++k) {
        e.shadow_q15+=20;
        for(n=0;n<10;++n) Encoder_UpdateVelocity2kHz(&e, 7);
    }
    assert(Encoder_GetMecVel(&e)==Encoder_GetMecVelContinuous(&e));
    assert(e.vel_elec==e.vel_mech*7);
    puts("PASS actual encoder estimator: low speed, both directions, reset, legacy path");
    return 0;
}
"""


def adc_irq_fixture():
    # Compile the actual board vector with register stubs, without CMSIS or
    # HIL cycle-counter instrumentation. The firmware build checks real types.
    source = (ROOT / 'Core/Src/stm32g4xx_it.c').read_text(encoding='utf-8')
    return r'''
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
typedef struct { uint32_t ISR, IER; } AdcRegisters;
typedef struct { AdcRegisters *Instance; unsigned calls; } ADC_HandleTypeDef;
static AdcRegisters adc1, adc2;
static ADC_HandleTypeDef hadc1 = {&adc1, 0}, hadc2 = {&adc2, 0};
static void HAL_ADC_IRQHandler(ADC_HandleTypeDef *adc) {
    adc->calls++;
    adc->Instance->ISR &= ~adc->Instance->IER;
}
''' + '\n#define USE_HAL_ADC_REGISTER_CALLBACKS 1\n' + function_source(source,
    'Board_ADC2DispatchInterrupt') + '\n' + function_source(source, 'ADC1_2_IRQHandler') + r'''
int main(void) {
    unsigned bit;
    for (bit=0; bit<11; ++bit) {
        uint32_t mask=1U<<bit;
        adc1.ISR=mask; adc1.IER=0; adc2.ISR=0; adc2.IER=mask;
        hadc1.calls=hadc2.calls=0;
        ADC1_2_IRQHandler();
        assert(hadc1.calls==0 && hadc2.calls==0 && adc1.ISR==mask);
        adc1.IER=mask; adc2.ISR=mask;
        ADC1_2_IRQHandler();
        assert(hadc1.calls==1 && hadc2.calls==1 && adc1.ISR==0 && adc2.ISR==0);
        ADC1_2_IRQHandler();
        assert(hadc1.calls==1 && hadc2.calls==1);
    }
    puts("PASS actual STM32G4 ADC vector: idle, disabled flags, both ADCs, all 11 event bits");
    return 0;
}
'''


def adc_sequence_fixture():
    source = (ROOT / 'Core/Src/stm32g4xx_it.c').read_text(encoding='utf-8')
    return r'''
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
enum { ADC_FLAG_JEOC = 1U << 5, ADC_FLAG_JEOS = 1U << 6 };
typedef struct { uint32_t flags; } ADC_HandleTypeDef;
static ADC_HandleTypeDef hadc1, hadc2;
static unsigned control_ticks;
#define __HAL_ADC_GET_FLAG(adc, flag) (((adc)->flags & (flag)) != 0U)
static void FOC20kHzIRQHandler(void) { control_ticks++; }
''' + function_source(source, 'HAL_ADCEx_InjectedConvCpltCallback') + r'''
int main(void) {
    unsigned rank;
    for (rank = 1; rank <= 4; ++rank) {
        hadc2.flags = ADC_FLAG_JEOC | (rank == 4 ? ADC_FLAG_JEOS : 0U);
        HAL_ADCEx_InjectedConvCpltCallback(&hadc2);
        assert(control_ticks == (rank == 4 ? 1U : 0U));
    }
    hadc1.flags = ADC_FLAG_JEOS;
    HAL_ADCEx_InjectedConvCpltCallback(&hadc1);
    assert(control_ticks == 1);
    puts("PASS actual ADC callback: no control on incomplete ranks or temperature ADC");
    return 0;
}
'''


def joint_startup_fixture():
    source = (ROOT / 'Foc/foc_param.c').read_text(encoding='utf-8')
    return r'''
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "../software/config/motor_axis_profile.h"
typedef struct {
    MotorAxisProfile axis_profile;
    bool axis_profile_valid;
    float cascade_pos_Kp, cascade_pos_Kd, speed_Kp, speed_Ki;
    float posAcc, posDec, pos_maxspeed, current_limit, speed_limit;
    unsigned calibration_sentinel;
} MotorControl_TypeDef;
''' + function_source(source, 'Param_ApplyJointProfile') + r'''
int main(void) {
    MotorControl_TypeDef m = {0}, before;
    unsigned id;
    m.axis_profile_valid = true;
    m.calibration_sentinel = 0xdeadbeef;
    m.current_limit = 4; m.speed_limit = .3f;
    m.speed_Ki = .123f;
    assert(MotorAxisProfile_Create(&m.axis_profile, "pitch", -.3f, .9f, .785398163f));
    before = m;
    assert(Param_ApplyJointProfile(&m) && memcmp(&m, &before, sizeof(m)) == 0);
    for (id = 1; id <= 2; ++id) {
        assert(MotorAxisProfile_CreateJoint(&m.axis_profile, id, 1, -.3f, .9f, .785398163f));
        before = m;
        assert(Param_ApplyJointProfile(&m));
        assert(m.speed_Ki == (id == 1 ? 1 : 2));
        assert(m.cascade_pos_Kp == 8 && m.cascade_pos_Kd == 2 && m.speed_Kp == .5f);
        assert(m.current_limit == 4 && m.pos_maxspeed == .3f);
        assert(m.calibration_sentinel == 0xdeadbeef);
        assert(memcmp(&m.axis_profile, &before.axis_profile, sizeof(m.axis_profile)) == 0);
    }
    m.current_limit = NAN;
    before = m;
    assert(!Param_ApplyJointProfile(&m) && memcmp(&m, &before, sizeof(m)) == 0);
    m.current_limit = 8;
    assert(Param_ApplyJointProfile(&m) && m.current_limit == 6);
    for (id = 0; id <= 5; ++id) {
        assert(MotorAxisProfile_CreateJoint(&m.axis_profile, id, 0, 0, 0, 0));
        before = m;
        assert(!Param_ApplyJointProfile(&m) && memcmp(&m, &before, sizeof(m)) == 0);
    }
    assert(!Param_ApplyJointProfile(NULL));
    puts("PASS actual joint startup adapter: selected gains, preserved calibration/limits, legacy and unknown");
    return 0;
}
'''


def joint_board_startup_fixture():
    source = (ROOT / 'System/board_config.c').read_text(encoding='utf-8')
    return r'''
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
enum {ADC_SINGLE_ENDED=0, TIM_CHANNEL_1=1, TIM_CHANNEL_2=2, TIM_CHANNEL_3=3,
      TIM_CHANNEL_4=4, ADC_IT_JEOC=1, ADC_IT_JEOS=2};
static int hadc1, hadc2, htim1, htim7;
static struct { bool axis_profile_valid; } MotorControl;
static bool configured;
static unsigned phases, sampling, communication, adc_started;
static unsigned adc_irq_mask = ADC_IT_JEOC;
static void flash_read_param(void) { MotorControl.axis_profile_valid = configured; }
static void MotorControl_Init(void) {}
static void motor_hw_outer_init(void) {}
static bool MotorControl_IsConfigurationValid(void) { return MotorControl.axis_profile_valid; }
static void delay_init(int clock) { (void)clock; }
static void HAL_ADCEx_Calibration_Start(int *adc, int mode) { (void)adc; (void)mode; }
static void HAL_TIM_PWM_Start(int *timer, int channel) {
    (void)timer; if (channel == TIM_CHANNEL_4) sampling++; else phases++;
}
static void HAL_TIMEx_OCN_Start(int *timer, int channel) { (void)timer; (void)channel; phases++; }
static void HAL_ADCEx_InjectedStart(int *adc) { (void)adc; adc_started++; }
static void __HAL_ADC_ENABLE_IT(int *adc, int mask) { assert(adc == &hadc2); adc_irq_mask |= mask; }
static void __HAL_ADC_DISABLE_IT(int *adc, int mask) { assert(adc == &hadc2); adc_irq_mask &= ~mask; }
static void HAL_TIM_Base_Start_IT(int *timer) { (void)timer; }
static void FDCAN1_Param_Init(void) { communication++; }
''' + function_source(source, 'Board_Init') + r'''
int main(void) {
    Board_Init();
    assert(phases == 0 && sampling == 1 && adc_started == 2 && communication == 1);
    assert(adc_irq_mask == ADC_IT_JEOS);
    configured = true;
    Board_Init();
    assert(phases == 6 && sampling == 2 && adc_started == 4 && communication == 2);
    assert(adc_irq_mask == ADC_IT_JEOS);
    puts("PASS actual board startup: unconfigured phase outputs off, sampling and communication retained");
    return 0;
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default=os.environ.get("SERVO_CC"))
    parser.add_argument("--recording", type=Path)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/mode3_host_tests")
    args = parser.parse_args()
    cc = args.cc or shutil.which("zig") or shutil.which("gcc") or shutil.which("clang")
    if not cc:
        parser.error("Provide --cc with a native C99 compiler or Zig executable")
    # Build commands run from ROOT, even when the caller runs elsewhere.
    args.out = args.out.resolve()
    if args.recording:
        args.recording = args.recording.resolve()
    if Path(cc).is_file():
        cc = str(Path(cc).resolve())
    args.out.mkdir(parents=True, exist_ok=True)
    compiler = [cc] + (["cc"] if Path(cc).stem == "zig" else [])
    logs = []

    def run(command):
        result = subprocess.run([str(x) for x in command], cwd=ROOT, capture_output=True, text=True)
        logs.append(result.stdout + result.stderr)
        print(result.stdout + result.stderr, end="")
        (args.out / "host_tests.log").write_text("\n".join(logs), encoding="utf-8")
        result.check_returncode()

    def build(name, files):
        executable = args.out / (name + (".exe" if os.name == "nt" else ""))
        run(compiler + ["-std=c99", "-O1", "-Wall", "-Wextra", "-Werror", "-I", "Foc"] +
            list(files) + ([] if os.name == "nt" else ["-lm"]) + ["-o", executable])
        run([executable])
        return executable

    exe = build("position_servo_test", ["tests/unit/position_servo_test.c",
                                        "Foc/position_cascade.c", "Foc/position_smooth_trajectory.c", "Foc/foc_pid.c"])
    fixture = args.out / "encoder_estimator_test.c"
    build("position_smooth_trajectory_test", ["tests/unit/position_smooth_trajectory_test.c",
                                              "Foc/position_smooth_trajectory.c"])
    fixture.write_text(encoder_fixture(), encoding="utf-8")
    build("encoder_estimator_test", [fixture])
    startup_fixture = args.out / "encoder_startup_test.c"
    startup_fixture.write_text(encoder_startup_fixture(), encoding="utf-8")
    build("encoder_startup_test", [startup_fixture, "software/config/motor_axis_profile.c"])
    irq_fixture = args.out / "adc_irq_dispatch_test.c"
    irq_fixture.write_text(adc_irq_fixture(), encoding="utf-8")
    build("adc_irq_dispatch_test", [irq_fixture])
    sequence_fixture = args.out / "adc_sequence_test.c"
    sequence_fixture.write_text(adc_sequence_fixture(), encoding="utf-8")
    build("adc_sequence_test", [sequence_fixture])
    build("servo_hil_test", ["tests/unit/servo_hil_test.c"])
    build("motor_axis_profile_test", ["tests/unit/motor_axis_profile_test.c",
                                      "software/config/motor_axis_profile.c"])
    joint_fixture = args.out / "joint_startup_test.c"
    joint_fixture.write_text(joint_startup_fixture(), encoding="utf-8")
    build("joint_startup_test", [joint_fixture, "software/config/motor_axis_profile.c"])
    board_fixture = args.out / "joint_board_startup_test.c"
    board_fixture.write_text(joint_board_startup_fixture(), encoding="utf-8")
    build("joint_board_startup_test", [board_fixture])
    if args.recording:
        import numpy as np
        data = np.loadtxt(args.recording, delimiter="\t", skiprows=1)
        status = data[:, 11].astype(int)
        valid = (status & 128) != 0
        indexes = np.flatnonzero(valid)
        assert len(indexes) and np.all(np.diff(indexes) == 1), "Replay requires one contiguous valid interval"
        first = indexes[0]
        phase = status[indexes] & 3
        starts = np.flatnonzero((phase == 0) & np.r_[True, phase[:-1] != 0])
        position = np.unwrap(data[indexes, 1] * np.pi / 32768)
        reference = np.unwrap(data[indexes, 0] * np.pi / 32768)
        target = np.empty(len(indexes))
        episodes = []
        for n, a in enumerate(starts):
            end = starts[n+1] if n+1 < len(starts) else len(indexes)
            settled = a + np.flatnonzero(phase[a:end] != 0)[0]
            target[a:end] = reference[settled]
            episodes.append((int(a), int(settled), int(end)))
        assert starts[0] == 0
        velocity = np.zeros(len(indexes))
        velocity[16:] = (position[16:] - position[:-16]) / .008
        inputs = args.out / "replay_inputs.csv"
        output = args.out / "replay_outputs.csv"
        np.savetxt(inputs, np.column_stack([indexes / 2000., target, position, velocity]),
                   delimiter=",", fmt="%.9f")
        run([exe, inputs, output])
        replay = np.loadtxt(output, delimiter=",", skiprows=1)
        assert len(replay) == len(indexes) and np.isfinite(replay).all()
        tails = []
        for n, (_, settled, end) in enumerate(episodes[:-1]):
            tail = slice(max(settled, end-4000), end)
            ff_max = float(np.max(np.abs(replay[tail, 4])))
            assert ff_max < .001, "Settled original tail must not receive new breakaway pulses"
            tails.append({"move": n+1, "end_time_s": (end+first)/2000,
                          "max_abs_ff_A": ff_max,
                          "hold_fraction": float(np.mean(replay[tail, 7] != 0))})
        result = {"input_rows": len(indexes), "original_motion_is_fixed": True,
                  "not_a_prediction_of_new_closed_loop_motion": True,
                  "targets_inferred_from_original_trajectory_end": True,
                  "velocity_reconstructed_from_quantized_position_16_sample_window": True,
                  "settled_tails": tails}
        (args.out / "replay_summary.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
