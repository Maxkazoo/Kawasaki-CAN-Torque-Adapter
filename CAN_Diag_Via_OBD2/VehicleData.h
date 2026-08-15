#ifndef VEHICLE_DATA_H
#define VEHICLE_DATA_H

#include <Arduino.h>

// KWP cache access from the ELM327 layer.
// Implemented by the main acquisition engine.
bool GetThrottleRaw(uint16_t &raw, unsigned long &updatedMs);
bool GetIntakePressureRaw(uint16_t &raw, unsigned long &updatedMs);
bool GetIntakeAirTempRaw(uint16_t &raw, unsigned long &updatedMs);
bool GetAtmosphericPressureRaw(uint16_t &raw, unsigned long &updatedMs);

bool KWPDataSessionValid(void);

#endif
