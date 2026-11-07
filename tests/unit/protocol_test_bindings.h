#ifndef PROTOCOL_TEST_BINDINGS_H
#define PROTOCOL_TEST_BINDINGS_H

/* 每个测试场景重新绑定端点，业务结果和队列均由本场景持有。 */
static ProtocolEndpoint endpoint;
static ProtocolMotorStatus s_test_motor_status;
static inline bool test_reset_protocol(uint8_t node_id)
{
    endpoint = (ProtocolEndpoint){0};
    s_test_motor_status = (ProtocolMotorStatus){0};
    endpoint.device_status.role = PROTOCOL_DEVICE_ROLE_APP;
    endpoint.motor.status = &s_test_motor_status;
    return Protocol_Init(&endpoint, node_id) && Protocol_Start(&endpoint);
}
#endif
