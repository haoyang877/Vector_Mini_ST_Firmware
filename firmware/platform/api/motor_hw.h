#ifndef MOTOR_HW_H
#define MOTOR_HW_H

/** Initialize the deferred outer-control execution context before sampling starts.
 * The fast motor interrupt must preempt this context. No motor algorithm lives here. */
void motor_hw_outer_init(void);
/** Request one deferred outer-control service. Caller owns the single-job mailbox;
 * it must not schedule a second job before consuming the first completion. */
void motor_hw_outer_schedule(void);
/** Publish/acquire shared motor data across the fast and deferred contexts.
 * This is a compiler and hardware memory barrier, never an interrupt lock. */
void motor_hw_outer_barrier(void);

#endif
