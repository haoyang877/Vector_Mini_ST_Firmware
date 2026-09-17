#ifndef CAN_PARAMETER_FORMAT_H
#define CAN_PARAMETER_FORMAT_H

/** @brief Read-only parameter 0x67 reports float32 revision 2, big endian.
 * Revision 2 requires int32 milliradian positions, int32 centi-radian
 * velocities/accelerations, int16 milliamp currents, and 48-byte status.
 * Unknown/missing revisions must not trigger automatic legacy fallback.
 */
#define CAN_PARAMETER_FORMAT_REVISION 2U

#endif
