#ifndef COMM_HW_H
#define COMM_HW_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

typedef struct {
    uint32_t identifier;
    uint8_t length;
    bool extended, remote;
    uint8_t data[64];
} CommHwCanFrame;

/** Pop one received frame into a full-size bounded buffer, without waiting. */
bool comm_hw_can_receive(CommHwCanFrame *frame);
/** Submit optional CAN FD telemetry only when no transmission is queued.
 * Never retry/wait; reserves room for subsequent priority replies. */
bool comm_hw_can_try_send_status(uint16_t identifier, const uint8_t *data, size_t length);
#endif
