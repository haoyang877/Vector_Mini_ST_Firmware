#ifndef COMPOSITION_FIRMWARE_COMPOSITION_H
#define COMPOSITION_FIRMWARE_COMPOSITION_H

void FirmwareComposition_Initialize(void);
void FirmwareComposition_ExecuteFastLoop(void);
void FirmwareComposition_ExecuteSupervisor1kHz(void);
void FirmwareComposition_RunBackground(void);

#endif
