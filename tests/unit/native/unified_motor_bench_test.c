/* 真实测试模块的内存回归；生命周期与外设仅为替身，不代表台架测量。 */
#include <stdio.h>
#include <string.h>

#include "motor_bench.c"

#define CHECK(condition)                                                                           \
    do                                                                                             \
    {                                                                                              \
        if (!(condition))                                                                          \
        {                                                                                          \
            fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition);                        \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)

static MotorConfiguration s_configuration;
static FocState s_foc;
static MotorControlState s_motor;
static EncoderState s_encoder;
static FluxObserverState s_observer;
static MotorState s_state;
static MotorCalibrationType s_calibration;
static uint32_t s_epoch, s_stops, s_starts;
static bool s_disabled;
static uint16_t s_sequence;
static yg_protocol_service_result_t s_reply;

uint32_t BspSystem_EnterCritical(void)
{
    return 0U;
}
void BspSystem_ExitCritical(uint32_t saved_state)
{
    (void)saved_state;
}
bool BspPwm_ArePhasesDisabled(void)
{
    return s_disabled;
}
float BspAdc_ReadRawChannel(BspAdcChannel channel)
{
    return 1000.0f + (float)channel;
}
float FluxObserver_GetElecPos(FluxObserverState *observer)
{
    (void)observer;
    return 0.0f;
}
EncoderCalibrationPhase EncoderCalibrationTask_GetStep(void)
{
    return CS_NULL;
}
MotorState MotorLifecycle_GetState(void)
{
    return s_state;
}
uint32_t MotorLifecycle_WorkEpoch(void)
{
    return s_epoch;
}
MotorCalibrationType MotorLifecycle_GetCalibrationType(void)
{
    return s_calibration;
}
bool MotorLifecycle_IsStopped(void)
{
    return s_state == MOTOR_STATE_IDLE && s_disabled;
}
bool MotorLifecycle_RequestRemoteStop(void)
{
    ++s_stops;
    return true;
}
MotorControlResult MotorLifecycle_RequestEnable(MotorControlMode mode)
{
    ++s_starts;
    ++s_epoch;
    s_motor.control_mode = mode;
    return MOTOR_CONTROL_ACCEPTED;
}

static void reset_case(void)
{
    memset(&s_configuration, 0, sizeof(s_configuration));
    memset(&s_foc, 0, sizeof(s_foc));
    memset(&s_motor, 0, sizeof(s_motor));
    s_motor.control_mode = MOTOR_CONTROL_NONE;
    memset(&s_encoder, 0, sizeof(s_encoder));
    memset(&s_observer, 0, sizeof(s_observer));
    s_configuration.application.current_limit_a = 4.0f;
    s_configuration.application.speed_limit_rad_s = 20.0f;
    s_configuration.model.motor_pole_pairs = 7;
    s_encoder.has_valid_sample = true;
    s_state = MOTOR_STATE_IDLE;
    s_calibration = MOTOR_CALIBRATION_NONE;
    s_disabled = true;
    s_epoch = 1U;
    s_stops = s_starts = 0U;
    s_sequence = 0U;
    MotorBench_Init(&s_configuration, &s_foc, &s_motor, &s_encoder, &s_observer);
}

static uint16_t
command_at(uint16_t operation, const uint8_t *data, uint32_t length, uint16_t sequence)
{
    uint8_t payload[YG_PROTOCOL_MAX_PAYLOAD_SIZE] = {0};
    YgProtocol_WriteU16Le(payload, operation);
    YgProtocol_WriteU16Le(payload + 2U, 1U);
    if (length != 0U)
    {
        memcpy(payload + 4U, data, length);
    }
    yg_protocol_message_t request = {.version = 1U,
                                     .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                     .source_node = 2U,
                                     .destination_node = 3U,
                                     .message_type = MOTOR_BENCH_MESSAGE_TYPE,
                                     .sequence = sequence,
                                     .payload_length = length + 4U,
                                     .payload = payload};
    memset(&s_reply, 0, sizeof(s_reply));
    if (MotorBench_Handle(NULL, &request, &s_reply) != YG_PROTOCOL_OK)
    {
        return UINT16_MAX;
    }
    return YgProtocol_ReadU16Le(s_reply.response_payload);
}

static uint16_t command(uint16_t operation, const uint8_t *data, uint32_t length)
{
    return command_at(operation, data, length, ++s_sequence);
}

static uint16_t limits(float current, float speed, float travel, uint32_t lease)
{
    uint8_t data[20] = {0};
    write_float(data, current);
    write_float(data + 4U, speed);
    write_float(data + 8U, travel);
    YgProtocol_WriteU32Le(data + 12U, 1000U);
    YgProtocol_WriteU32Le(data + 16U, lease);
    return command(OP_LIMITS, data, sizeof(data));
}

static void run_data(uint8_t data[16], uint16_t kind, uint32_t duration_us)
{
    memset(data, 0, 16U);
    YgProtocol_WriteU16Le(data, kind);
    write_float(data + 4U, 0.2f);
    YgProtocol_WriteU32Le(data + 8U, duration_us);
    YgProtocol_WriteU32Le(data + 12U, 10000U);
}

static uint16_t arm(uint16_t trigger, uint16_t pre, uint32_t delay_ms)
{
    uint8_t data[16] = {0};
    YgProtocol_WriteU16Le(data, GROUP_DQ);
    YgProtocol_WriteU16Le(data + 2U, 1U);
    YgProtocol_WriteU16Le(data + 4U, trigger);
    YgProtocol_WriteU16Le(data + 6U, pre);
    YgProtocol_WriteU32Le(data + 8U, delay_ms);
    return command(OP_CAPTURE, data, sizeof(data));
}

static void ticks(unsigned count)
{
    MotorTaskResult outcome = {MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
    for (unsigned i = 0U; i < count; ++i)
    {
        MotorBench_BeforeControl();
        MotorBench_AfterControl(outcome);
    }
}

static int protocol_and_limits(void)
{
    reset_case();
    CHECK(command(OP_INFO, NULL, 0U) == 0U && s_reply.response_payload_length == 46U);
    CHECK(YgProtocol_ReadU32Le(s_reply.response_payload + 4U) == BENCH_MAGIC);
    CHECK(limits(NAN, 10.0f, 100.0f, 500U) == 11U);
    CHECK(limits(5.0f, 10.0f, 100.0f, 500U) == 11U);
    CHECK(limits(1.0f, 0.0f, 100.0f, 500U) == 11U);
    CHECK(limits(1.0f, 10.0f, 0.0f, 500U) == 11U);
    CHECK(limits(1.0f, 10.0f, 100.0f, 99U) == 11U);
    CHECK(command(65535U, NULL, 0U) == 4U);
    uint8_t data[16];
    run_data(data, RUN_ID, 100000U);
    CHECK(command(OP_RUN, data, sizeof(data)) == 5U && s_starts == 0U);
    CHECK(limits(1.0f, 10.0f, 100.0f, 500U) == 0U);
    CHECK(command(OP_RUN, data, sizeof(data)) == 1U && s_starts == 1U);
    CHECK(limits(1.0f, 10.0f, 100.0f, 500U) == 6U);
    return 0;
}

static int duplicate_and_sequence(void)
{
    reset_case();
    CHECK(limits(1.0f, 10.0f, 100.0f, 500U) == 0U);
    uint8_t data[16];
    run_data(data, RUN_ID, 100000U);
    CHECK(command_at(OP_RUN, data, sizeof(data), 65534U) == 5U);
    CHECK(command_at(OP_RUN, data, sizeof(data), 2U) == 1U);
    CHECK(command_at(OP_RUN, data, sizeof(data), 2U) == 1U && s_starts == 1U);
    write_float(data + 4U, 0.3f);
    CHECK(command_at(OP_RUN, data, sizeof(data), 2U) == 4U && s_starts == 1U);
    CHECK(command_at(OP_RUN, data, sizeof(data), 1U) == 5U && s_starts == 1U);
    reset_case();
    s_sequence = 65533U;
    CHECK(limits(1.0f, 10.0f, 100.0f, 500U) == 0U);
    CHECK(command_at(OP_STOP, NULL, 0U, 1U) == 0U && s_stops == 1U);
    CHECK(command_at(OP_STOP, NULL, 0U, 1U) == 0U && s_stops == 1U);
    return 0;
}

static int capture_and_readback(void)
{
    reset_case();
    CHECK(arm(0U, 0U, 0U) == 1U);
    ticks(128U);
    CHECK(command(OP_INFO, NULL, 0U) == 0U);
    CHECK(YgProtocol_ReadU16Le(s_reply.response_payload + 22U) == CAP_FROZEN);
    CHECK(YgProtocol_ReadU16Le(s_reply.response_payload + 28U) == 128U);
    uint32_t capture = YgProtocol_ReadU32Le(s_reply.response_payload + 16U);
    uint8_t data[8];
    YgProtocol_WriteU32Le(data, capture);
    YgProtocol_WriteU16Le(data + 6U, 0U);
    for (uint16_t index = 0U; index < 128U; ++index)
    {
        YgProtocol_WriteU16Le(data + 4U, index);
        CHECK(command(OP_READ, data, sizeof(data)) == 0U);
        CHECK(s_reply.response_payload_length == 44U);
        CHECK(YgProtocol_ReadU32Le(s_reply.response_payload + 12U) == index + 1U);
    }
    s_disabled = false;
    CHECK(command(OP_READ, data, sizeof(data)) == 6U);
    s_disabled = true;
    YgProtocol_WriteU32Le(data, capture - 1U);
    CHECK(command(OP_READ, data, sizeof(data)) == 6U);
    return 0;
}

static int stale_trigger_and_pretrigger(void)
{
    reset_case();
    CHECK(limits(1.0f, 10.0f, 100.0f, 500U) == 0U);
    /* 建立上一轮遗留时刻，模拟前台ARM到下一RUN启动之间的中断窗口。 */
    s_bench.reference_applied = true;
    s_bench.step_tick = 0U;
    s_bench.tick = 1000U;
    s_bench.run_state = RUN_DONE;
    CHECK(arm(2U, 32U, 2U) == 1U);
    ticks(64U);
    CHECK(s_bench.capture_state == CAP_WAIT);
    uint8_t data[16];
    run_data(data, RUN_ID, 100000U);
    CHECK(command(OP_RUN, data, sizeof(data)) == 1U);
    ticks(64U);
    CHECK(s_bench.capture_state == CAP_WAIT);
    s_state = MOTOR_STATE_RUNNING;
    s_disabled = false;
    ticks(240U);
    CHECK(s_bench.capture_state == CAP_WAIT);
    ticks(1U);
    CHECK(s_bench.capture_state == CAP_RECORDING && s_bench.count == 33U);
    uint32_t trigger_tick = s_bench.tick;
    ticks(95U);
    CHECK(s_bench.capture_state == CAP_FROZEN && s_bench.count == 128U);
    CHECK(s_bench.samples[s_bench.first_index].tick == trigger_tick - 32U);
    return 0;
}

static int stop_guards(void)
{
    const uint16_t expected[] = {WHY_CURRENT,
                                 WHY_SPEED,
                                 WHY_TRAVEL,
                                 WHY_LEASE,
                                 WHY_ENCODER,
                                 WHY_MEASUREMENT,
                                 WHY_STATE,
                                 WHY_FAULT};
    for (unsigned test = 0U; test < sizeof(expected) / sizeof(expected[0]); ++test)
    {
        reset_case();
        CHECK(limits(1.0f, 10.0f, 0.1f, 100U) == 0U);
        uint8_t data[16];
        run_data(data, RUN_ID, 500000U);
        CHECK(command(OP_RUN, data, sizeof(data)) == 1U);
        s_state = MOTOR_STATE_RUNNING;
        s_disabled = false;
        ticks(1U);
        if (test == 0U)
        {
            s_foc.phase_current_a.a = 1.1f;
        }
        if (test == 1U)
        {
            s_encoder.mechanical_velocity_continuous_rad_s = 11.0f;
        }
        if (test == 2U)
        {
            s_encoder.directed_q15 = 2048U;
        }
        if (test == 3U)
        {
            s_bench.tick += milliseconds_to_ticks(100U);
        }
        if (test == 4U)
        {
            s_encoder.has_valid_sample = false;
        }
        if (test == 5U)
        {
            s_foc.phase_current_a.c = NAN;
        }
        if (test == 6U)
        {
            ++s_epoch;
        }
        if (test == 7U)
        {
            s_motor.fault_code = MOTOR_FAULT_ENCODER;
        }
        ticks(1U);
        CHECK(s_stops > 0U && s_bench.run_state == RUN_FAILED);
        CHECK(s_bench.reason == expected[test]);
    }
    return 0;
}

static int pulse_and_duration(void)
{
    reset_case();
    CHECK(limits(1.0f, 10.0f, 100.0f, 500U) == 0U);
    uint8_t data[16];
    run_data(data, RUN_IQ_PULSE, 5001U);
    CHECK(command(OP_RUN, data, sizeof(data)) == 11U);
    run_data(data, RUN_IQ_PULSE, 5000U);
    CHECK(command(OP_RUN, data, sizeof(data)) == 1U);
    s_state = MOTOR_STATE_RUNNING;
    s_disabled = false;
    ticks(201U);
    CHECK(s_motor.iq_target_a == 0.2f);
    ticks(100U);
    CHECK(s_motor.iq_target_a == 0.0f && s_stops == 0U);
    ticks(100U);
    CHECK(s_bench.run_state == RUN_DONE && s_bench.reason == WHY_DURATION && s_stops > 0U);
    return 0;
}

static int calibration_completion(void)
{
    for (unsigned completed = 0U; completed < 2U; ++completed)
    {
        reset_case();
        CHECK(limits(1.0f, 10.0f, 100.0f, 500U) == 0U);
        s_state = MOTOR_STATE_CALIBRATION;
        s_calibration = MOTOR_CALIBRATION_ENCODER_LINEARIZATION;
        s_disabled = false;
        ++s_epoch;
        ticks(1U);
        CHECK(s_bench.run_kind == RUN_LUT && s_bench.run_state == RUN_ACTIVE);
        MotorBench_CalibrationPublished(completed != 0U);
        s_state = MOTOR_STATE_IDLE;
        s_disabled = true;
        ticks(1U);
        CHECK(s_bench.run_state == (completed ? RUN_DONE : RUN_FAILED));
    }
    return 0;
}

static int remote_start_guard(void)
{
    reset_case();
    CHECK(MotorLifecycle_RequestEnable(MOTOR_CONTROL_CURRENT) == MOTOR_CONTROL_ACCEPTED);
    MotorBench_BeforeControl();
    CHECK(s_stops > 0U);
    reset_case();
    CHECK(limits(1.0f, 10.0f, 100.0f, 500U) == 0U);
    CHECK(MotorLifecycle_RequestEnable(MOTOR_CONTROL_CURRENT) == MOTOR_CONTROL_ACCEPTED);
    MotorBench_BeforeControl();
    CHECK(s_bench.run_kind == RUN_REMOTE && s_bench.run_state == RUN_WAIT && s_stops == 0U);
    s_state = MOTOR_STATE_RUNNING;
    s_disabled = false;
    s_motor.iq_target_a = 0.125f;
    MotorBench_BeforeControl();
    CHECK(s_bench.run_state == RUN_ACTIVE && s_motor.iq_target_a == 0.125f);
    s_bench.tick += s_bench.limits.lease_ms * (FOC_FREQ / 1000U);
    MotorBench_BeforeControl();
    CHECK(s_bench.run_state == RUN_FAILED && s_bench.reason == WHY_LEASE && s_stops > 0U);
    return 0;
}

static int id_sine_reference(void)
{
    reset_case();
    CHECK(limits(1.0f, 10.0f, 100.0f, 500U) == 0U);
    uint8_t data[24];
    run_data(data, RUN_ID_SINE, 100000U);
    write_float(data + 4U, 0.1f);
    write_float(data + 16U, 1000.0f);
    write_float(data + 20U, 0.5f);
    CHECK(command(OP_RUN, data, 16U) == 4U);
    write_float(data + 16U, NAN);
    CHECK(command(OP_RUN, data, sizeof(data)) == 11U);
    write_float(data + 16U, 4000.1f);
    CHECK(command(OP_RUN, data, sizeof(data)) == 11U);
    write_float(data + 16U, 0.0f);
    CHECK(command(OP_RUN, data, sizeof(data)) == 11U);
    write_float(data + 16U, 1000.0f);
    write_float(data + 20U, NAN);
    CHECK(command(OP_RUN, data, sizeof(data)) == 11U);
    write_float(data + 20U, 0.05f);
    CHECK(command(OP_RUN, data, sizeof(data)) == 11U);
    write_float(data + 20U, 0.95f);
    CHECK(command(OP_RUN, data, sizeof(data)) == 11U);
    write_float(data + 20U, 0.5f);
    CHECK(s_starts == 0U);
    CHECK(command(OP_RUN, data, sizeof(data)) == 1U);
    CHECK(command_at(OP_RUN, data, sizeof(data), s_sequence) == 1U && s_starts == 1U);
    CHECK(arm(1U, 0U, 0U) == 1U);
    s_state = MOTOR_STATE_RUNNING;
    s_disabled = false;
    ticks(200U);
    CHECK(s_motor.id_target_a == 0.5f && s_motor.iq_target_a == 0.0f);
    CHECK(s_bench.capture_state == CAP_WAIT);
    ticks(1U);
    CHECK(s_bench.capture_state == CAP_RECORDING && s_bench.step_tick == s_bench.tick);
    ticks(5U);
    CHECK(fabsf(s_motor.id_target_a - 0.6f) < 0.00001f);
    ticks(10U);
    CHECK(fabsf(s_motor.id_target_a - 0.4f) < 0.00001f);
    s_encoder.mechanical_velocity_continuous_rad_s = 11.0f;
    ticks(1U);
    CHECK(s_bench.run_state == RUN_FAILED && s_bench.reason == WHY_SPEED && s_stops > 0U);
    CHECK(s_bench.capture_state == CAP_FROZEN);
    return 0;
}

int main(void)
{
    struct
    {
        const char *name;
        int (*run)(void);
    } cases[] = {
        {"protocol_and_limits", protocol_and_limits},
        {"duplicate_and_sequence", duplicate_and_sequence},
        {"capture_and_readback", capture_and_readback},
        {"stale_trigger_and_pretrigger", stale_trigger_and_pretrigger},
        {"stop_guards", stop_guards},
        {"pulse_and_duration", pulse_and_duration},
        {"calibration_completion", calibration_completion},
        {"remote_start_guard", remote_start_guard},
        {"id_sine_reference", id_sine_reference},
    };
    for (unsigned i = 0U; i < sizeof(cases) / sizeof(cases[0]); ++i)
    {
        if (cases[i].run() != 0)
        {
            return 1;
        }
        printf("CASE %s PASS\n", cases[i].name);
    }
    return 0;
}
