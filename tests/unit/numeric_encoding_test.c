/* 线路与参数数值编码差分检查；直接编译生产私有转换，不连接设备。 */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <math.h>
#include "../../firmware/communication/protocol/yg_protocol_motor_status.c"
#include "../../firmware/services/parameters/parameter_value_service.c"
static int32_t ref_measurement(float value, double scale, int32_t min, int32_t max)
{
    if (!isfinite(value))
    {
        return INT32_MIN;
    }
    double rounded = round((double)value * scale);
    if (rounded < min || rounded > max)
    {
        return INT32_MIN;
    }
    return (int32_t)rounded;
}
static int ref_parameter(float value, float scale, uint32_t *raw)
{
    if (!isfinite(value))
    {
        return 0;
    }
    double rounded = (double)value * scale;
    rounded += rounded >= 0.0 ? 0.5 : -0.5;
    if (rounded > INT32_MAX || rounded < INT32_MIN)
    {
        return 0;
    }
    *raw = (uint32_t)(int32_t)rounded;
    return 1;
}
/* f0c2517 的双精度参考：独立冻结，用于验证舍入和拒绝边界。 */
static uint64_t cases;
static void check(float value)
{
    const uint32_t scales[] = {100, 1000, 1000000, 1000000000};
    for (unsigned i = 0; i < 4; i++)
    {
        uint32_t a = 0x1234, b = a;
        int ra = ref_parameter(value, (float)scales[i], &a),
            rb = (encode_float(value, scales[i], &b) == PARAMETER_OK);
        if (ra != rb || a != b)
        {
            printf("PARAM %a %u: %d %x vs %d %x\n", value, scales[i], ra, a, rb, b);
            exit(1);
        }
        for (unsigned small = 0; small < 2; small++)
        {
            int32_t limit = small ? INT16_MAX : INT32_MAX;
            int32_t ma = ref_measurement(value, scales[i], -limit, limit),
                    mb = measurement(value, scales[i], -limit, limit);
            if (ma != mb)
            {
                printf("MEAS %a %u: %d vs %d\n", value, scales[i], ma, mb);
                exit(1);
            }
        }
        cases += 3;
    }
}
/* 从真实目录读回所有标量，覆盖描述符缩放和私有结构偏移，而非只测转换函数。 */
static void check_catalog(void)
{
    MotorConfiguration motor = {0};
    const float samples[] = {0.0f,
                             -0.0f,
                             0.0625f,
                             -0.0625f,
                             1.25f,
                             -1.25f,
                             2147483.5f,
                             -2147483.5f,
                             NAN,
                             INFINITY,
                             -INFINITY};
    for (unsigned i = 0; i < sizeof(s_fields) / sizeof(s_fields[0]); ++i)
    {
        for (unsigned j = 0; j < sizeof(samples) / sizeof(samples[0]); ++j)
        {
            memcpy((uint8_t *)&motor + s_fields[i].offset, &samples[j], sizeof(float));
            ParameterValue value;
            uint32_t expected = 0U;
            bool valid = ref_parameter(samples[j], (float)s_fields[i].scale, &expected) != 0;
            ParameterStatus result = ParameterValue_Read(&motor, 42U, s_fields[i].id, &value);
            if ((result == PARAMETER_OK) != valid ||
                (valid &&
                 (value.raw_value != expected || value.revision != 42U ||
                  value.parameter_id != s_fields[i].id || value.value_type != PARAMETER_VALUE_I32)))
            {
                printf("CATALOG mismatch: field %u sample %u\n", s_fields[i].id, j);
                exit(1);
            }
        }
    }
}

int main(void)
{
    uint32_t state = 0x5ad123a;
    float f;
    for (uint32_t i = 0; i < 2000000; i++)
    {
        state = state * 1664525U + 1013904223U;
        memcpy(&f, &state, 4);
        check(f);
    }
    for (unsigned e = 0; e < 256; e++)
    {
        for (unsigned m = 0; m < 1024; m++)
        {
            for (unsigned neg = 0; neg < 2; neg++)
            {
                uint32_t bits = (neg << 31) | (e << 23) | (m * 8192U);
                memcpy(&f, &bits, 4);
                check(f);
            }
        }
    }
    const float scales[] = {100, 1000, 1000000, 1000000000};
    const double bounds[] = {
        0, .5, 1, 32766.5, 32767, 32767.5, 2147483646.5, 2147483647, 2147483647.5, 2147483648.0};
    for (unsigned s = 0; s < 4; s++)
    {
        for (unsigned b = 0; b < 10; b++)
        {
            for (int sign = -1; sign <= 1; sign += 2)
            {
                f = (float)(sign * bounds[b] / scales[s]);
                for (int k = 0; k < 100; k++)
                {
                    f = nextafterf(f, -INFINITY);
                }
                for (int k = 0; k < 201; k++)
                {
                    check(f);
                    f = nextafterf(f, INFINITY);
                }
            }
        }
    }
    for (int i = -65536; i <= 65536; i++)
    {
        for (unsigned s = 0; s < 4; s++)
        {
            f = (float)(((double)i + .5) / scales[s]);
            check(f);
            check(nextafterf(f, -INFINITY));
            check(nextafterf(f, INFINITY));
        }
    }
    check_catalog();
    printf("PASS %llu comparisons\n", (unsigned long long)cases);
    return 0;
}
