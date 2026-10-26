/* 输入是已记录的反馈；此回放不产生机械运动的预测值。 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "position_cascade.h"
#include "position_cascade_config.h"
#include "config.inc"
#include "runtime_params.h"

int main(int argc, char **argv)
{
    if (argc != 4) { return 2; }
    FILE *input = fopen(argv[1], "r");
    FILE *output = fopen(argv[2], "w");
    if (input == NULL || output == NULL) { return 3; }
    PositionCascadeConfig_TypeDef c = config();
    c.acceleration = REPLAY_ACC;
    c.deceleration = REPLAY_DEC;
    c.maximum_speed = REPLAY_MAX_SPEED;
    c.speed_limit = REPLAY_SPEED_LIMIT;
    c.jerk_limit = fmaxf(c.acceleration,c.deceleration) / POSITION_SERVO_JERK_RAMP_TIME_S;
    c.position_kp = REPLAY_KP;
    c.position_kd = REPLAY_KD;
    c.speed_kp = REPLAY_SPEED_KP;
    c.speed_ki = REPLAY_SPEED_KI;
    c.current_limit = REPLAY_CURRENT;
    c.friction_coulomb_positive = REPLAY_FRICTION_POS;
    c.friction_coulomb_negative = REPLAY_FRICTION_NEG;
    c.friction_attack_slew_rate = REPLAY_ATTACK;
    c.friction_fast_release_slew_rate = c.friction_release_slew_rate = REPLAY_RELEASE;
    c.position_error_window = POSITION_SERVO_DAMPED_ERROR_WINDOW_RAD;
    c.velocity_filter_hz = 10.0f;
    c.acceleration_feedforward_gain = 0.0f;
    PositionCascade_Reset();
    float time, target, position, speed;
    unsigned sequence, received;
    fprintf(output,"time,target,measured,measured_speed,reference,trajectory_speed,trajectory_acceleration,speed_command,iq_command,friction,integral,external_stale,external_rejected_points\n");
    while (fscanf(input,"%f %f %f %f %u %u",&time,&target,&position,&speed,&sequence,&received)==6)
    {
        PositionCascadeOutput_TypeDef o;
        PositionCascadeTelemetry_TypeDef t;
        c.target_position = target;
        if (argv[3][0]=='1')
        {
#if REPLAY_HAS_EXTERNAL
            PositionExternalInput sample={target,received,(uint32_t)(time*1000.0f),sequence,sequence!=0,REPLAY_MIN,REPLAY_MAX};
            PositionCascadeControlOutput_TypeDef compact;
            if (!PositionCascade_UpdateExternalControl(&c,&sample,position,speed,&compact)) { return 4; }
            memset(&o,0,sizeof(o));
            o.position_reference=compact.position_reference;
            o.speed_reference=compact.speed_reference;
            o.iq_reference=compact.iq_reference;
#else
            return 5;
#endif
        }
        else if (!PositionCascade_Update(&c,position,speed,&o)) { return 4; }
        if (!PositionCascade_GetTelemetry(&t)) { return 6; }
        unsigned stale = 0U, rejected = 0U;
#if REPLAY_HAS_EXTERNAL
        /* 记录候选实际拒绝/过期状态，防止回放被错误边界冻结后仍仅凭退出码宣称通过。 */
        stale = t.external_reference_stale ? 1U : 0U;
        rejected = t.external_rejected_points;
#endif
        fprintf(output,"%.6f,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%u,%u\n",
            time,target,position,speed,o.position_reference,t.trajectory_speed_reference,
            t.acceleration_reference,o.speed_reference,o.iq_reference,t.friction_feedforward_current,t.hold_current,stale,rejected);
    }
    fclose(input);fclose(output);return 0;
}
