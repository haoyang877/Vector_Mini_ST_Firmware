#ifndef COMPOSITION_FIRMWARE_COMPOSITION_H
#define COMPOSITION_FIRMWARE_COMPOSITION_H

#include <stdint.h>

void FirmwareComposition_Initialize(void);
void FirmwareComposition_ExecuteFastLoop(void);
void FirmwareComposition_ExecuteSupervisor1kHz(void);
void FirmwareComposition_OnCanReceiveInterrupt(void);
void FirmwareComposition_OnUsbReceiveInterrupt(const uint8_t *data,
	uint32_t size_bytes);
void FirmwareComposition_OnUsbTransmitCompleteInterrupt(void);
void FirmwareComposition_RunBackground(void);

#endif
