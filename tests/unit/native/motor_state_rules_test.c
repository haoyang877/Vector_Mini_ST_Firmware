/* ccd8d8a8 真实事实归一化规则黄金摘要：覆盖优先级、身份、取消、提交效果和代际耗尽。
 * 无故障事实由 active_faults 推导，删除旧夹具互相矛盾的故障布尔组合。 */
#include <stdio.h>
#include <stdint.h>
#include "motor_state_internal.h"

static uint32_t seed = UINT32_C(0x91ab32cd);
static uint64_t digest = UINT64_C(14695981039346656037);
static uint32_t random_value(void)
{
    seed ^= seed << 13;
    seed ^= seed >> 17;
    seed ^= seed << 5;
    return seed;
}
static void record(uint32_t value)
{
    for (unsigned i = 0; i < 4; ++i)
    {
        digest = (digest ^ (uint8_t)(value >> (8U * i))) * UINT64_C(1099511628211);
    }
}
int main(void)
{
    /* 空输入的关断语义，以及全部实际维护种类的安全分类，不靠摘要隐含覆盖。 */
    MotorStateRulesContext initial;
    MotorStateRules_Init(NULL);
    MotorStateRules_Init(&initial);
    MotorStateDecision misuse = MotorStateRules_Step(NULL, NULL, NULL);
    if (misuse.rejection != MOTOR_STATE_REJECT_ARGUMENT ||
        misuse.actions != MOTOR_STATE_ACTION_DISABLE_POWER || initial.state != MOTOR_STATE_BOOT ||
        initial.epoch != 1U || initial.tested || initial.maintenance_kind != MOTOR_MAINTENANCE_NONE)
    {
        return 1;
    }
    misuse = MotorStateRules_Step(&initial, NULL, NULL);
    if (initial.state != MOTOR_STATE_FATAL || !(misuse.actions & MOTOR_STATE_ACTION_DISABLE_POWER))
    {
        return 1;
    }
    const MotorMaintenanceClass classes[] = {MOTOR_MAINTENANCE_CLASS_CALIBRATION,
                                             MOTOR_MAINTENANCE_CLASS_CALIBRATION,
                                             MOTOR_MAINTENANCE_CLASS_SAVE,
                                             MOTOR_MAINTENANCE_CLASS_CALIBRATION,
                                             MOTOR_MAINTENANCE_CLASS_CALIBRATION,
                                             MOTOR_MAINTENANCE_CLASS_CALIBRATION,
                                             MOTOR_MAINTENANCE_CLASS_ZERO,
                                             MOTOR_MAINTENANCE_CLASS_DEFAULTS,
                                             MOTOR_MAINTENANCE_CLASS_NONE,
                                             MOTOR_MAINTENANCE_CLASS_INVALID};
    for (unsigned kind = 0U; kind < sizeof(classes) / sizeof(classes[0]); ++kind)
    {
        if (MotorStateRules_Classify((MotorMaintenanceKind)kind) != classes[kind])
        {
            return 1;
        }
    }
    const MotorMaintenanceKind kinds[] = {MOTOR_MAINTENANCE_NONE,
                                          MOTOR_MAINTENANCE_ENCODER_LUT,
                                          MOTOR_MAINTENANCE_SAVE,
                                          MOTOR_MAINTENANCE_DEFAULTS,
                                          MOTOR_MAINTENANCE_MECHANICAL_ZERO,
                                          (MotorMaintenanceKind)9};
    const MotorControlMode modes[] = {MOTOR_CONTROL_NONE,
                                      MOTOR_CONTROL_CURRENT,
                                      MOTOR_CONTROL_SPEED,
                                      MOTOR_CONTROL_POSITION,
                                      MOTOR_CONTROL_VQ,
                                      (MotorControlMode)5};
    const uint32_t event_sets[] = {0U,
                                   MOTOR_STATE_EVENT_BOOT_DONE,
                                   MOTOR_STATE_EVENT_SELF_TEST_DONE,
                                   MOTOR_STATE_EVENT_ENABLE,
                                   MOTOR_STATE_EVENT_START_DONE,
                                   MOTOR_STATE_EVENT_STOP,
                                   MOTOR_STATE_EVENT_FAULT,
                                   MOTOR_STATE_EVENT_FATAL,
                                   MOTOR_STATE_EVENT_CLEAR,
                                   MOTOR_STATE_EVENT_BEGIN_OPERATION,
                                   MOTOR_STATE_EVENT_OPERATION_DONE,
                                   MOTOR_STATE_EVENT_OPERATION_FAILED,
                                   MOTOR_STATE_EVENT_LINK_RECOVERED,
                                   MOTOR_STATE_EVENT_CANCEL,
                                   MOTOR_STATE_EVENT_CANCEL_DONE,
                                   MOTOR_STATE_EVENT_STOP | MOTOR_STATE_EVENT_OPERATION_FAILED,
                                   MOTOR_STATE_EVENT_STOP | MOTOR_STATE_EVENT_ENABLE,
                                   MOTOR_STATE_EVENT_CANCEL | MOTOR_STATE_EVENT_OPERATION_DONE,
                                   MOTOR_STATE_EVENT_CLEAR | MOTOR_STATE_EVENT_ENABLE,
                                   MOTOR_STATE_EVENT_OPERATION_DONE | MOTOR_STATE_EVENT_ENABLE,
                                   MOTOR_STATE_EVENT_CANCEL_DONE | MOTOR_STATE_EVENT_FAULT,
                                   UINT32_C(0x8000),
                                   UINT32_C(0x3fff)};
    for (unsigned i = 0; i < 262144U; ++i)
    {
        MotorStateRulesContext a;
        MotorStateRules_Init(&a);
        a.state = (MotorState)(i % 9U);
        a.tested = (random_value() & 1U) != 0U;
        a.epoch = i % 31U == 0U ? UINT32_MAX : 1U + random_value() % 5U;
        a.maintenance_kind = kinds[random_value() % 5U];
        a.maintenance_active = a.maintenance_kind != MOTOR_MAINTENANCE_NONE;
        a.operation_result = (MotorMaintenanceResult)(random_value() % 5U);
        a.operation_effects = (MotorMaintenanceEffect)(random_value() % 3U);
        a.first_fault = i % 19U == 0U ? 1U : 0U;
        a.latched_faults = i % 19U == 0U ? 3U : 0U;
        uint32_t bits = random_value();
        MotorStateFacts g = {.active_faults = i % 29U == 0U ? 4U : 0U,
                             .phases_disabled = (bits & 1U) != 0U,
                             .control_idle = (bits & 2U) != 0U,
                             .operation_idle = (bits & 4U) != 0U,
                             .maintenance_released = (bits & 8U) != 0U,
                             .inputs_valid = (bits & 16U) != 0U,
                             .configuration_valid = true,
                             .self_test_ready = (bits & 64U) != 0U,
                             .start_permitted = (bits & 128U) != 0U,
                             .operation_permitted = (bits & 256U) != 0U,
                             .startup_confirmed = (bits & 512U) != 0U,
                             .request_fresh = (bits & 1024U) != 0U};
        if (i % 4U == 0U)
        {
            g = (MotorStateFacts){
                0U, true, true, true, true, true, true, true, true, true, true, true};
        }
        MotorStateEvent e = {
            .events = event_sets[(i / 9U) % (sizeof(event_sets) / sizeof(event_sets[0]))],
            .faults = i % 17U == 0U ? 8U : 0U,
            .completion_epoch = i % 5U == 0U ? 0U : a.epoch,
            .request_epoch = i % 7U == 0U ? 0U : a.epoch,
            .control_mode = modes[random_value() % 6U],
            .maintenance_kind = kinds[random_value() % 6U],
            .effects = (MotorMaintenanceEffect)(random_value() % 4U)};
        MotorStateDecision r = MotorStateRules_Step(&a, &e, &g);
        record(a.state);
        record(a.epoch);
        record(a.maintenance_active ? MotorStateRules_Classify(a.maintenance_kind)
                                    : MOTOR_MAINTENANCE_CLASS_NONE);
        record(a.operation_result);
        record(a.operation_effects);
        record(a.first_fault);
        record(a.latched_faults);
        record(a.tested);
        record(r.actions);
        record(r.rejection);
    }
    printf("262144 cases, digest %016llx\n", (unsigned long long)digest);
    if (digest != UINT64_C(0x14acc14ec440764b))
    {
        fprintf(stderr, "FAIL frozen motor-state behavior digest\n");
        return 1;
    }
    return 0;
}
