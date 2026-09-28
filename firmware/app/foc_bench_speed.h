#ifndef FOC_BENCH_SPEED_H
#define FOC_BENCH_SPEED_H

#include <stdint.h>

#define FOC_BENCH_SPEED_MAILBOX_VERSION 1U

/** 显式台架命令；START 只申请零速度进入现有速度模式。 */
enum
{
    FOC_BENCH_SPEED_START = 1U,
    FOC_BENCH_SPEED_STOP = 2U
};
/** 请求受理结果；ACCEPTED 不表示已经启相。 */
enum
{
    FOC_BENCH_SPEED_ACCEPTED = 1U,
    FOC_BENCH_SPEED_INVALID,
    FOC_BENCH_SPEED_DENIED,
    FOC_BENCH_SPEED_BUSY,
    FOC_BENCH_SPEED_INTERRUPTED
};
/** 会话状态；STOPPED 需要生命周期退出及数字关相确认。 */
enum
{
    FOC_BENCH_SPEED_IDLE = 0U,
    FOC_BENCH_SPEED_STARTING,
    FOC_BENCH_SPEED_RUNNING,
    FOC_BENCH_SPEED_STOPPING,
    FOC_BENCH_SPEED_STOPPED
};
/** 本次台架结束原因；故障仍通过既有 ErrorNow 锁存。 */
enum
{
    FOC_BENCH_SPEED_NO_STOP = 0U,
    FOC_BENCH_SPEED_MANUAL_STOP,
    FOC_BENCH_SPEED_TIMEOUT,
    FOC_BENCH_SPEED_PHASE_CURRENT,
    FOC_BENCH_SPEED_OVERSPEED,
    FOC_BENCH_SPEED_FAULT
};

/**
 * 专用 SWD 台架邮箱，不是电机内部状态导出。每个字段均为一个对齐的 32 位字。
 * 外部唯一生产者先写奇数 request_seq，随后写 command 至 speed_limit_rad_s，最后
 * 提交下一个偶数 request_seq；序号非零、递增、不回绕。CPU 不等待写入者。
 * version 与 status_seq 后的字段只由固件写；读取回复须两次读 status_seq，相等且
 * 为偶数才接受。上电清空，version=1；不是 CAN 或 Flash ABI。
 * START 的 duration_ms 为 1..5000，iq_limit_a 必须低于 phase_limit_a 且不高于
 * 已有 current_limit；phase_limit_a 最大 6 A，speed_limit_rad_s 最大 300 rpm
 * 对应的 rad/s，亦不得高于已有速度/轴限制。START 不接受非零速度目标。
 * 启动后可经既有 116 更新同模式速度目标，定时不会因此延期；STOP/FAULT 优先撤销。
 */
typedef struct
{
    uint32_t version;
    uint32_t request_seq;
    uint32_t command;
    uint32_t duration_ms;
    float iq_limit_a;
    float phase_limit_a;
    float speed_limit_rad_s;
    uint32_t status_seq;
    uint32_t ack_seq;
    uint32_t ack_result;
    uint32_t session_seq;
    uint32_t state;
    uint32_t stop_reason;
    uint32_t remaining_ms;
} FocBenchSpeedMailbox;

/** 唯一显式调试入口；只能按上述发布协议写请求区，禁止写回复区或电机对象。 */
extern volatile FocBenchSpeedMailbox FocBenchSpeed;

#endif
