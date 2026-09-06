#ifndef PLATFORM_STM32G431_MOTOR_DRIVE_TIM1_ADC2_STM32G431_H
#define PLATFORM_STM32G431_MOTOR_DRIVE_TIM1_ADC2_STM32G431_H

#include "Bsp/Api/bsp_motor_drive.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Binds the VectorMini fixed low-side three-shunt acquisition implemented by
 * TIM1 CH1..3 and ADC2 injected ranks 1..4. The caller-owned capabilities
 * object must remain alive for the lifetime of the returned port.
 */
bool MotorDriveTim1Adc2Stm32g431_CreatePort(
	const BspMotorDriveEndpointCapabilities *capabilities,
	BspMotorDrivePort *port);

/* Called from the ADC2 injected end-of-sequence callback before the fast
 * loop consumes JDR1..4. It publishes the queued plan as completed only after
 * hardware has reported JEOS. */
void MotorDriveTim1Adc2Stm32g431_OnInjectedSequenceComplete(void);

#ifdef __cplusplus
}
#endif

#endif
