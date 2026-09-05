#ifndef RUNTIME_ROTOR_CALIBRATION_PORT_ADAPTER_H
#define RUNTIME_ROTOR_CALIBRATION_PORT_ADAPTER_H

#include "encoder.h"
#include "rotor_calibration_port.h"

RotorCalibrationPort RotorCalibrationAdapter_CreatePort(EncoderContext *encoder);

#endif
