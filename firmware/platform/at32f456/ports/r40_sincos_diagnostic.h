#ifndef R40_SINCOS_DIAGNOSTIC_H
#define R40_SINCOS_DIAGNOSTIC_H

#include <stdint.h>

/* 仅台架原码快照，不属于公共BSP或输出轴反馈接口；读取需版本一致。 */
typedef struct
{
    uint32_t begin, sequence, milliseconds, pa5_counts, pa6_counts, end;
} R40SinCosDiagnostic;

extern volatile R40SinCosDiagnostic g_r40_sincos_snapshot;

#endif
