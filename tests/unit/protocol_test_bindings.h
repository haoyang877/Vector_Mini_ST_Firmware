#ifndef PROTOCOL_TEST_BINDINGS_H
#define PROTOCOL_TEST_BINDINGS_H

/* 仅测试替换业务结果；生产镜像使用 APP 提供的只读静态绑定。 */
static ProtocolMotor s_test_motor;
static ProtocolJobs s_test_jobs;
static BootEntryService s_test_boot;
static ProtocolMotorStatus s_test_motor_status;
static ProtocolDeviceStatus s_test_device_status = {.role = PROTOCOL_DEVICE_ROLE_APP};
const ProtocolDeviceStatus *const Protocol_DeviceStatusSource = &s_test_device_status;

#define MOTOR_TEST_CALLBACK(name)                                                                  \
    static void test_motor_##name(const ProtocolCommand *request, ProtocolReply *reply)            \
    {                                                                                              \
        if (s_test_motor.name != NULL)                                                             \
        {                                                                                          \
            s_test_motor.name(request, reply);                                                     \
        }                                                                                          \
        else                                                                                       \
        {                                                                                          \
            *reply = (ProtocolReply){.status = PROTOCOL_UNSUPPORTED};                              \
        }                                                                                          \
    }

MOTOR_TEST_CALLBACK(start)
MOTOR_TEST_CALLBACK(target)
MOTOR_TEST_CALLBACK(stop)
MOTOR_TEST_CALLBACK(clear_fault)

static void test_job(void *context, const ProtocolJobRequest *request, ProtocolJobReply *reply)
{
    (void)context;
    if (s_test_jobs.handler != NULL)
    {
        s_test_jobs.handler(s_test_jobs.context, request, reply);
    }
    else
    {
        *reply = (ProtocolJobReply){.status = PROTOCOL_UNSUPPORTED};
    }
}

static void test_boot_request(void *context, const BootEntryRequest *request, BootEntryReply *reply)
{
    (void)context;
    if (s_test_boot.request != NULL)
    {
        s_test_boot.request(s_test_boot.context, request, reply);
    }
    else
    {
        *reply = (BootEntryReply){.result = BOOT_ENTRY_UNAVAILABLE};
    }
}

static void test_boot_complete(void *context, uint64_t ticket, bool success)
{
    (void)context;
    if (s_test_boot.tx_complete != NULL)
    {
        s_test_boot.tx_complete(s_test_boot.context, ticket, success);
    }
}

static bool test_boot_confirm(void *context, uint32_t version, uint32_t crc32)
{
    (void)context;
    return s_test_boot.confirm != NULL && s_test_boot.confirm(s_test_boot.context, version, crc32);
}

const ProtocolMotor Protocol_Motor = {
    .start = test_motor_start,
    .target = test_motor_target,
    .stop = test_motor_stop,
    .clear_fault = test_motor_clear_fault,
    .status = &s_test_motor_status,
};
const ProtocolJobs Protocol_Jobs = {NULL, test_job};
const BootEntryService Protocol_Boot = {
    NULL, test_boot_request, test_boot_complete, test_boot_confirm};

static inline bool test_reset_protocol(uint8_t node_id)
{
    s_ready = s_started = false;
    s_node_id = 0U;
    s_rx_queue = (CanQueue){.storage = s_rx_storage, .capacity = RX_QUEUE_SIZE};
    s_tx_queue = (CanQueue){.storage = s_tx_storage, .capacity = TX_QUEUE_SIZE};
    s_motor = (MotorMessages){0};
    s_device = (DeviceMessages){0};
    s_boot = (BootState){0};
    s_test_motor = (ProtocolMotor){0};
    s_test_jobs = (ProtocolJobs){0};
    s_test_boot = (BootEntryService){0};
    s_test_motor_status = (ProtocolMotorStatus){0};
    s_test_device_status = (ProtocolDeviceStatus){.role = PROTOCOL_DEVICE_ROLE_APP};
    return Protocol_Init(node_id) && Protocol_Start();
}

#endif
