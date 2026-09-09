#include "servo_hil.h"
#include <math.h>

/* Host access is confined to a command mailbox; never to FOC/PI state. */
typedef struct {
    uint32_t magic, sequence, opcode;
    float value;
    uint32_t heartbeat, ack, result, active, tick;
    float position, speed, iq;
    uint32_t mode, error;
} ServoHilMailbox;
static volatile ServoHilMailbox servo_hil_mailbox = {.magic = 0x48494C31U};
static uint32_t last_heartbeat, heartbeat_ticks, pending_sequence;
static ServoHilAction pending_action;
static bool armed;

/* Read-only host diagnostics/contract; no host writes are accepted here. */
typedef struct {
    uint32_t magic;
    float phase_peak_limit_a, exposure_threshold_a;
    uint32_t exposure_limit_us, exposure_ticks;
    float observed_peak_a;
    uint32_t trip, frequency_hz, sample_count;
    float trip_ia, trip_ib, trip_ic;
} ServoHilCurrentGuard;
static ServoHilCurrentGuard phase_guard_state = {0x48494331U, SERVO_HIL_PHASE_PEAK_A,
    SERVO_HIL_PHASE_EXPOSURE_THRESHOLD_A, SERVO_HIL_PHASE_EXPOSURE_LIMIT_US,
    0, 0, 0, 0, 0, 0, 0, 0};

/* Publish on the alternating mailbox tick, away from the position-loop tick. */
static volatile ServoHilCurrentGuard servo_hil_current_guard = {0x48494331U,
    SERVO_HIL_PHASE_PEAK_A, SERVO_HIL_PHASE_EXPOSURE_THRESHOLD_A,
    SERVO_HIL_PHASE_EXPOSURE_LIMIT_US, 0, 0, 0, 0, 0, 0, 0, 0};

void ServoHil_ObservePhaseCurrents(float ia, float ib, float ic)
{
    float peak;
    if (!armed || phase_guard_state.trip != 0U) return;
    if (!isfinite(ia + ib + ic)) {
        phase_guard_state.trip = 5U;
        return;
    }
    peak = fabsf(ia);
    if (fabsf(ib) > peak) peak = fabsf(ib);
    if (fabsf(ic) > peak) peak = fabsf(ic);
    if (peak > phase_guard_state.observed_peak_a)
        phase_guard_state.observed_peak_a = peak;
    if (peak >= SERVO_HIL_PHASE_PEAK_A) {
        phase_guard_state.trip_ia = ia;
        phase_guard_state.trip_ib = ib;
        phase_guard_state.trip_ic = ic;
        phase_guard_state.trip = 5U;
    }
    if (peak > SERVO_HIL_PHASE_EXPOSURE_THRESHOLD_A) {
        if (phase_guard_state.exposure_ticks < UINT32_MAX)
            phase_guard_state.exposure_ticks++;
        else phase_guard_state.trip = 6U;
    }
}

static ServoHilCommand stop(uint32_t result)
{
    ServoHilCommand c = {SERVO_HIL_STOP, 0};
    armed = false;
    servo_hil_mailbox.active = 0;
    servo_hil_mailbox.result = result;
    return c;
}

ServoHilCommand ServoHil_Poll(float position, float speed, float iq,
    uint32_t mode, uint32_t error, uint32_t frequency_hz, uint32_t elapsed_ticks)
{
    ServoHilCommand c = {SERVO_HIL_NONE, 0};
    uint32_t seq, op, heartbeat;
    float value;
    servo_hil_mailbox.tick += elapsed_ticks;
    servo_hil_mailbox.position = position;
    servo_hil_mailbox.speed = speed;
    servo_hil_mailbox.iq = iq;
    servo_hil_mailbox.mode = mode;
    servo_hil_mailbox.error = error;
    if (armed) {
        phase_guard_state.sample_count += elapsed_ticks;
        phase_guard_state.frequency_hz = frequency_hz;
        if (frequency_hz < 2U || frequency_hz > UINT32_MAX / SERVO_HIL_PHASE_EXPOSURE_LIMIT_SECONDS || elapsed_ticks == 0U)
            phase_guard_state.trip = 5U;
        else if (phase_guard_state.exposure_ticks >= frequency_hz * SERVO_HIL_PHASE_EXPOSURE_LIMIT_SECONDS)
            phase_guard_state.trip = 6U;
    }
    servo_hil_current_guard = phase_guard_state;
    if (armed && phase_guard_state.trip != 0U)
        return stop(phase_guard_state.trip);
    heartbeat = servo_hil_mailbox.heartbeat;
    if (heartbeat != last_heartbeat) {
        last_heartbeat = heartbeat;
        heartbeat_ticks = 0;
    } else if (elapsed_ticks <= UINT32_MAX - heartbeat_ticks)
        heartbeat_ticks += elapsed_ticks;
    else heartbeat_ticks = UINT32_MAX;
    /* Targets are <=85 deg; trip at the user-authorized +/-90 deg boundary.
     * This is a debug-run stop, not a substitute for the normal fault manager. */
    if (armed && (!isfinite(position) || fabsf(position) >= SERVO_HIL_TRAVEL_LIMIT_RAD ||
        !isfinite(speed) || fabsf(speed) > 1.57079633f || error != 0 || mode != 3))
        return stop(3);
    if (armed && (frequency_hz == 0 || elapsed_ticks == 0 || heartbeat_ticks >= frequency_hz / 2U))
        return stop(2);
    seq = servo_hil_mailbox.sequence;
    if (seq == servo_hil_mailbox.ack) return c;
    op = servo_hil_mailbox.opcode;
    value = servo_hil_mailbox.value;
    if (seq != servo_hil_mailbox.sequence) return c;
    pending_sequence = seq;
    pending_action = SERVO_HIL_NONE;
    if (op == SERVO_HIL_STOP) {
        servo_hil_mailbox.ack = seq;
        return stop(0);
    }
    if (!isfinite(value) || op < SERVO_HIL_ARM || op > SERVO_HIL_POSITION_KD) {
        servo_hil_mailbox.ack = seq;
        return stop(4);
    }
    if (op == SERVO_HIL_ARM) {
        if (mode != 0 || error != 0 || !isfinite(position) || fabsf(position) > SERVO_HIL_TARGET_LIMIT_RAD ||
            heartbeat_ticks >= frequency_hz / 2U) {
            servo_hil_mailbox.ack = seq;
            return stop(4);
        }
    } else if (op == SERVO_HIL_POSITION) {
        if (!armed || fabsf(value) > SERVO_HIL_TARGET_LIMIT_RAD) {
            servo_hil_mailbox.ack = seq;
            return stop(4);
        }
    } else {
        /* Change gains only while disabled, between trials. */
        float maximum = op == SERVO_HIL_POSITION_KP ? 10.0f :
            op == SERVO_HIL_SPEED_KP ? 0.5f :
            op == SERVO_HIL_MAX_SPEED ? SERVO_HIL_MAX_CRUISE_RAD_S : 2.0f;
        if (mode != 0 || armed || value < 0 || value > maximum ||
            (op == SERVO_HIL_MAX_SPEED && value <= 0)) {
            servo_hil_mailbox.ack = seq;
            return stop(4);
        }
    }
    pending_action = (ServoHilAction)op;
    c.action = pending_action;
    c.value = value;
    return c;
}

void ServoHil_Complete(bool accepted)
{
    if (pending_action == SERVO_HIL_NONE) return;
    if (accepted && pending_action == SERVO_HIL_ARM) {
        phase_guard_state.exposure_ticks = 0U;
        phase_guard_state.observed_peak_a = 0.0f;
        phase_guard_state.trip = 0U;
        phase_guard_state.sample_count = 0U;
        phase_guard_state.trip_ia = 0.0f;
        phase_guard_state.trip_ib = 0.0f;
        phase_guard_state.trip_ic = 0.0f;
        armed = true;
        heartbeat_ticks = 0;
    }
    if (!accepted) armed = false;
    servo_hil_mailbox.active = armed ? 1U : 0U;
    servo_hil_mailbox.result = accepted ? 0U : 4U;
    servo_hil_mailbox.ack = pending_sequence;
    pending_action = SERVO_HIL_NONE;
}
