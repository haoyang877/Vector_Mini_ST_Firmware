#ifndef PROTOCOL_TEST_BINDINGS_H
#define PROTOCOL_TEST_BINDINGS_H

/* 每个测试场景重新绑定端点，业务结果和队列均由本场景持有。 */
static ProtocolEndpoint endpoint;
typedef struct
{
    uint32_t faults;
    uint8_t state, mode;
    bool sample_available;
} TestMotorStatus;

static TestMotorStatus s_test_motor_status;
static bool test_read_feedback(void *context, ProtocolFeedback *feedback)
{
    (void)context;
    feedback->motor_state = s_test_motor_status.state;
    feedback->control_mode = s_test_motor_status.mode;
    feedback->fault_code = s_test_motor_status.faults;
    return s_test_motor_status.sample_available;
}

static inline bool test_reset_protocol(uint8_t node_id)
{
    endpoint = (ProtocolEndpoint){0};
    s_test_motor_status = (TestMotorStatus){0};
    endpoint.device_status.role = PROTOCOL_DEVICE_ROLE_APP;
    endpoint.motor.read_feedback = test_read_feedback;
    return Protocol_Init(&endpoint, node_id) && Protocol_Start(&endpoint);
}
#endif
