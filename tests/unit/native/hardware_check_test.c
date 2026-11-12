#include "hardware_check.h"
#include "board_config.h"
#include "bsp_adc.h"
#include "bsp_encoder.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static MotorAxis s_axis;
static MotorAxisPreset s_preset = {.has_linear_hall = true, .gear_ratio = 40.0f};
static ProtocolHardwareCheck s_check;
static uint32_t s_ticks, s_cycles, s_frame, s_run, s_stops;
static uint16_t s_seq;
static float s_adc[10];
static bool s_off = true;
static const float s_pi2 = 6.28318530718f;

uint32_t BspSystem_CycleCount(void)
{
    return s_cycles;
}
uint32_t BspSystem_TimeMs(void)
{
    return s_ticks / 20U;
}
void BspSystem_MemoryBarrier(void)
{
}
uint32_t BspSystem_EnterControlCritical(void)
{
    return 0U;
}
void BspSystem_ExitControlCritical(uint32_t mask)
{
    (void)mask;
}
uint32_t BspEncoder_ReadCompletedFrame(void)
{
    return s_frame;
}
float BspAdc_ReadRawChannel(BspAdcChannel channel)
{
    return s_adc[channel];
}
static bool phases_disabled(void)
{
    return s_off;
}
bool MotorStateMachine_IsStopped(const MotorStateMachine *machine)
{
    return s_off && !machine->stop_pending && !machine->start_pending &&
           s_axis.motor.control_mode == MOTOR_CONTROL_NONE;
}
void MotorStateMachine_RequestStop(MotorStateMachine *machine)
{
    ++s_stops;
    machine->stop_pending = true;
    s_off = true;
}

static uint32_t frame(int32_t angle, unsigned status)
{
    uint32_t data = (((uint32_t)angle & 0x1fffffU) << 3U) | status;
    uint8_t crc = 0;
    for (int bit = 23; bit >= 0; --bit)
    {
        unsigned top = ((crc >> 7U) ^ (data >> bit)) & 1U;
        crc = (uint8_t)((crc << 1U) ^ (top ? 7U : 0U));
    }
    return data << 8U | crc;
}

static ProtocolHardwareReport report(void)
{
    ProtocolHardwareReport r;
    assert(s_check.handle(&s_axis, 0U, 0U, 0U, &r) == PROTOCOL_RESULT_OK);
    return r;
}

static void begin(unsigned mode, int installation)
{
    static const MotorAxisHardware hardware = {.power = {.phases_disabled = phases_disabled}};
    s_off = true;
    s_stops = 0;
    s_axis = (MotorAxis){.hardware = &hardware};
    s_axis.configuration.preset = installation < 0 ? NULL : &s_preset;
    s_preset.has_linear_hall = installation == 1;
    s_axis.configuration.model.motor_pole_pairs = 14;
    s_axis.configuration.application.speed_limit_rad_s = 20;
    s_axis.state_machine.epoch = 7U;
    s_axis.motor.control_mode = MOTOR_CONTROL_NONE;
    s_axis.motor_temperature.valid = true;
    s_axis.motor_temperature.celsius = 25.0f;
    s_axis.foc.bus_voltage_v = 28.0f;
    s_axis.linear_hall = (LinearHall){.sin_off_counts = 2048,
                                      .cos_off_counts = 2048,
                                      .sin_amp = 1800,
                                      .cos_amp = 1800,
                                      .calibration_success = true,
                                      .lut_status = LINEAR_HALL_LUT_OK};
    for (unsigned i = 0; i < 10; ++i)
    {
        s_adc[i] = 2048;
    }
    s_axis.hall_samples[0].sin_counts = 2048;
    s_axis.hall_samples[0].cos_counts = 3848;
    s_axis.motor_temperature.sample_ms = BspSystem_TimeMs();
    s_frame = frame(789, 0U);
    ProtocolHardwareReport r;
    ++s_seq;
    uint32_t previous = report().run_id;
    assert(s_check.handle(&s_axis, mode, s_seq, previous, &r) == PROTOCOL_RESULT_ACCEPTED);
    s_run = r.run_id;
    assert(s_run == previous + 1U && r.stage == 1U);
    assert(s_check.handle(&s_axis, mode, s_seq, previous, &r) == PROTOCOL_RESULT_ACCEPTED);
    assert(s_check.handle(&s_axis, mode, s_seq + 1U, previous, &r) == PROTOCOL_RESULT_DENIED);
    assert(s_check.handle(&s_axis, mode, s_seq + 1U, s_run, &r) == PROTOCOL_RESULT_BUSY);
}

static void tick(bool fresh)
{
    ++s_ticks;
    s_cycles += AT32_CORE_HZ / FOC_FREQ;
    if (fresh)
    {
        s_axis.motor_temperature.sample_ms = BspSystem_TimeMs();
    }
    if (g_hardware_check_capture)
    {
        HardwareCheck_CaptureFast(&s_axis);
        HardwareCheck_Process(&s_axis);
    }
}

static void window(void)
{
    for (unsigned i = 0; i < 2000U; ++i)
    {
        tick(true);
    }
}

static void static_cases(void)
{
    for (unsigned k = 0; k < 8U; ++k)
    {
        begin(1U, 1);
        for (unsigned i = 0; i < 2000; ++i)
        {
            int sign = i % 2 ? 1 : -1;
            s_adc[BSP_ADC_PHASE_CURRENT_A] = 2048.0f + sign * (k & 1U ? 10.0f : 9.75f);
            s_adc[BSP_ADC_PHASE_CURRENT_B] = k & 2U ? 1748.0f : 1748.25f;
            s_adc[BSP_ADC_PHASE_CURRENT_C] = k & 2U ? 2348.0f : 2347.75f;
            s_adc[BSP_ADC_BUS_CURRENT] = 2048 + sign * (k & 4U ? 10 : 9);
            s_axis.hall_samples[0].sin_counts = (uint16_t)(2048 + sign * (k & 2U ? 10 : 9));
            s_axis.hall_samples[0].cos_counts = (uint16_t)(3600 + sign * (k & 4U ? 10 : 9));
            s_frame = frame(i % 2 ? -1 : (k & 1U ? 58 : 57), 0);
            s_axis.foc.bus_voltage_v = i % 2 ? 27.0f : 29.0f;
            s_axis.motor_temperature.celsius = i % 2 ? 0.0f : 60.0f;
            tick(true);
        }
        ProtocolHardwareReport r = report();
        assert(r.stage == 4 && !g_hardware_check_capture);
        assert(r.items[0] == (k & 1U ? 2 : 1) && r.items[1] == (k & 2U ? 2 : 1));
        assert(r.items[2] == (k & 2U ? 2 : 1) && r.items[3] == (k & 4U ? 2 : 1));
        assert(r.items[4] == 1 && r.items[5] == 1 && r.items[6] == (k & 1U ? 2 : 1));
        assert(r.items[7] == (k & 6U ? 2 : 1) && r.items[8] == 0 && r.items[9] == 0);
    }
    for (unsigned k = 0; k < 9; ++k)
    {
        begin(2U, k == 8 ? -1 : 0);
        if (k == 0)
        {
            s_frame = 0;
        }
        if (k == 1)
        {
            s_frame ^= 1U;
        }
        if (k == 2)
        {
            s_frame = frame(77, 2);
        }
        if (k == 3)
        {
            s_axis.motor_temperature.celsius = -0.01f;
        }
        if (k == 4)
        {
            s_axis.motor_temperature.celsius = 60.01f;
        }
        if (k == 5)
        {
            s_axis.motor_temperature.celsius = NAN;
        }
        for (unsigned i = 0; i < 2000; ++i)
        {
            s_axis.foc.bus_voltage_v = i == 123 ? (k == 7 ? 29.01f : 26.99f) : 28.0f;
            tick(k != 6);
        }
        ProtocolHardwareReport r = report();
        assert(r.stage == 5 && r.items[4] == 2 && r.items[8] == 0);
        assert(r.items[5] == (k == 3 || k == 4 ? 0 : (k == 5 || k == 6 ? 2 : 1)));
        assert(r.items[6] == (k == 0 ? 0 : (k == 1 || k == 2 ? 2 : 1)));
        assert(r.items[7] == (k == 8 ? 0 : 3) && r.items[9] == r.items[7]);
    }
}

static void start_motion(int16_t seq_delta)
{
    ++s_axis.state_machine.epoch;
    s_axis.state_machine.motion_lease_active = true;
    s_axis.motor.control_mode = MOTOR_CONTROL_OPEN_CURRENT;
    s_check.started(&s_axis, 10U, (uint16_t)(s_seq + seq_delta));
    s_off = false;
}

static void motion_sample(int32_t rotor, int32_t actual, int override)
{
    s_frame = frame(rotor, 0U);
    float phase = (float)actual / (2097152.0f * s_preset.gear_ratio) * s_pi2;
    s_axis.hall_samples[0].sin_counts = (uint16_t)lroundf(2048 - 1800 * sinf(phase));
    s_axis.hall_samples[0].cos_counts = (uint16_t)lroundf(2048 + 1800 * cosf(phase));
    if (override >= 0)
    {
        s_axis.hall_samples[0].sin_counts = (uint16_t) override;
    }
    tick(true);
}

static void motion_cases(void)
{
    for (unsigned k = 0; k < 14; ++k)
    {
        begin(2U, k == 4 ? 0 : 1);
        s_axis.output_encoder.params.status = OUTPUT_ENCODER_CAL_OK;
        s_axis.output_encoder.params.direction = k == 1 || k == 11 ? -1 : 1;
        s_axis.configuration.device.encoder.reverse = k == 1;
        /* 正/反边界值在零频预备段测试，以免刻意跳码影响连续性判据。 */
        s_frame = frame(-2000, 0);
        window();
        assert(report().stage == 2);
        start_motion(k == 5 ? 1 : 0);
        int32_t angle = -2000;
        int32_t rotor = angle;
        if (k >= 7 && k <= 10)
        {
            /* 四个原码边界单独注入第一拍，以免与角跳变混淆。 */
            static const int s_edges[] = {94, 4001, 95, 4000};
            motion_sample(rotor, angle, s_edges[k - 7]);
            if (k < 9)
            {
                assert(report().stage == 5 && report().items[9] == 2 && s_stops == 1);
            }
            else
            {
                assert(report().stage == 3 && report().items[9] != 2);
                s_axis.foc.bus_voltage_v = 30.0f;
                tick(true);
            }
            continue;
        }
        for (unsigned leg = 0; leg < 2 && g_hardware_check_capture; ++leg)
        {
            s_axis.motor.open_loop_frequency_hz = leg ? -3 : 3;
            for (unsigned j = 0; j < 4000 && g_hardware_check_capture; ++j)
            {
                if (k != 12 || j < 100U || j > 3550U)
                {
                    angle += leg ? -24 : 24;
                }
                if (k != 2)
                {
                    rotor = angle;
                }
                if (k == 3 && j == 987)
                {
                    rotor += 10000;
                }
                motion_sample(rotor, angle, -1);
            }
            s_axis.motor.open_loop_frequency_hz = 0;
            for (unsigned j = 0; j < 1200 && g_hardware_check_capture; ++j)
            {
                if (k == 13 && j == 15)
                {
                    rotor += 10000;
                }
                motion_sample(rotor, angle, -1);
            }
        }
        if (k == 3 || k == 5 || k == 12 || k == 13)
        {
            ProtocolHardwareReport aborted = report();
            assert(aborted.stage == 5 && s_stops == (k == 5 ? 0U : 1U));
            if (k != 5)
            {
                assert(aborted.items[8] == 2);
            }
            continue;
        }
        if (k == 6)
        {
            s_axis.foc.bus_voltage_v = 29.001f;
            tick(true);
            assert(report().stage == 5 && report().items[4] == 2 && s_stops == 1);
            continue;
        }
        /* 110 先撤销模式，监督稍后才实际关相并更换 epoch。 */
        s_axis.state_machine.stop_pending = true;
        s_axis.motor.control_mode = MOTOR_CONTROL_NONE;
        tick(true);
        s_off = true;
        ++s_axis.state_machine.epoch;
        s_axis.state_machine.stop_pending = false;
        tick(true);
        ProtocolHardwareReport r = report();
        assert(r.stage == 4 && s_stops == 0U);
        assert(r.items[8] == (k == 2 ? 0 : 1) && r.items[9] == (k == 4 ? 3 : (k == 11 ? 2 : 1)));
        assert((r.directions & 15) == (k == 1 ? 6 : (k == 2 ? 15 : 9)));
        assert((r.directions >> 4) == (k == 4 ? 0 : 9));
    }
}

/* 这两帧由实际主控编码器生成，按字节穿过生产 RX IRQ、前台及 TX 再交回主控。 */
extern const uint8_t host_start[32], host_read[20];
static ProtocolEndpoint s_endpoint;
static BspCanFrame s_rx, s_tx;
static bool s_rx_pending, s_tx_sent;

bool BspCan_Receive(BspCanFrame *frame)
{
    if (!s_rx_pending)
    {
        return false;
    }
    s_rx_pending = false;
    *frame = s_rx;
    return true;
}

bool BspCan_TrySend(const BspCanFrame *frame)
{
    s_tx = *frame;
    s_tx_sent = true;
    return true;
}

static void service_frame(const uint8_t *wire, unsigned length, bool corrupt)
{
    s_rx = (BspCanFrame){.identifier = 0x0cef0602U,
                         .length = (uint8_t)length,
                         .extended = true,
                         .fd = true,
                         .bitrate_switch = true};
    memcpy(s_rx.data, wire, length);
    if (corrupt)
    {
        s_rx.data[16] ^= 1U;
    }
    s_rx_pending = true;
    s_tx_sent = false;
    Protocol_RxInterrupt(&s_endpoint);
    Protocol_Process(&s_endpoint, BspSystem_TimeMs());
}

static void can_loopback(void)
{
    begin(1U, 1);
    window();
    assert(report().run_id == 1U && report().stage == 4U);
    s_endpoint.hardware_check = s_check;
    assert(Protocol_Init(&s_endpoint, 6U) && Protocol_Start(&s_endpoint));
    service_frame(host_start, 32U, true);
    assert(!s_tx_sent && report().run_id == 1U);
    service_frame(host_start, 32U, false);
    assert(s_tx_sent && report().run_id == 2U && report().stage == 1U && s_off);
    for (unsigned i = 0; i < 2000U; ++i)
    {
        if (i == 500U)
        {
            service_frame(host_start, 32U, false);
        }
        s_adc[BSP_ADC_PHASE_CURRENT_A] = i % 2U ? 2058.0f : 2038.0f;
        tick(true);
    }
    service_frame(host_read, 20U, false);
    assert(s_tx_sent && s_tx.identifier == 0x0cef0206U && s_tx.length == 48U && s_off &&
           s_stops == 0U);
    printf("CAN_HW_REPORT ");
    for (unsigned i = 0U; i < s_tx.length; ++i)
    {
        printf("%02x", s_tx.data[i]);
    }
    puts("");
}

int main(void)
{
    s_check = HardwareCheck_Bind(&s_axis);
    assert(report().stage == 0 && report().run_id == 0);
    s_cycles = UINT32_MAX - 45000;
    can_loopback();
    static_cases();
    for (unsigned age = 30; age <= 31; ++age)
    {
        begin(1U, 1);
        for (unsigned i = 0; i < 1999; ++i)
        {
            tick(true);
        }
        s_axis.motor_temperature.sample_ms = (s_ticks + 1U) / 20U - age;
        tick(false);
        assert(report().items[5] == (age == 30 ? 1 : 2));
    }
    motion_cases();
    static const unsigned s_cos_edges[] = {94U, 95U, 4000U, 4001U};
    for (unsigned i = 0; i < 4U; ++i)
    {
        begin(2U, 1);
        window();
        start_motion(0);
        s_axis.hall_samples[0].cos_counts = (uint16_t)s_cos_edges[i];
        tick(true);
        assert((report().items[9] == 2) == (i == 0 || i == 3));
        s_axis.foc.bus_voltage_v = 30.0f;
        tick(true);
    }
    begin(1U, 1);
    tick(true);
    s_cycles += AT32_CORE_HZ / FOC_FREQ;
    tick(true);
    assert(report().stage == 5 && report().items[0] == 0);
    begin(1U, 1);
    for (unsigned i = 0; i < 150; ++i)
    {
        HardwareCheck_CaptureFast(&s_axis);
    }
    HardwareCheck_Process(&s_axis);
    assert(report().stage == 5);
    begin(1U, 1);
    s_ticks += 12000;
    HardwareCheck_Process(&s_axis);
    assert(report().stage == 5);
    puts("PASS AT32 hardware check: static edges, full DMA frames, two driven legs, ownership, "
         "loss and STOP");
    return 0;
}
