#ifndef DATA_CONVERSION_H
#define DATA_CONVERSION_H

#include <Arduino.h>

//==================================================
// Kawasaki -> OBD2 conversion table
//
// Keep vehicle-specific conversion constants/functions here.
// The acquisition engine and ELM327 emulator should not contain
// hard-coded conversion formulas.
//==================================================

// ---- Normal broadcast CAN ----
#define KDS_CAN_ID_RPM       0x100
#define KDS_CAN_ID_SPEED     0x110
#define KDS_CAN_ID_WATER     0x120
#define KDS_CAN_ID_GEAR      0x121
#define KDS_CAN_ID_FUEL      0x32F

// Current working assumptions from vehicle measurements:
// 0x100: first 2 data bytes, big-endian, raw value = rpm.
// 0x110: first 2 data bytes, big-endian, raw value = 0.1 km/h.
// 0x120: Data1 = coolant temperature + 60 degC.
// 0x121: Data0 = gear, Data1 = neutral flag.
// 0x32F: Data0:Data1 BE16 confirmed as the varying raw candidate;
//        physical L/h coefficient is still under calibration.

uint16_t ReadBE16(const byte *data, byte len, byte index);

// Standard OBD-II Mode 01 byte encoders.
bool ConvertBroadcastRPMToOBD(const byte *data, byte len, byte &A, byte &B);
bool ConvertBroadcastSpeedToOBD(const byte *data, byte len, byte &A);
bool ConvertBroadcastWaterToOBD(const byte *data, byte len, byte &A);

// ---- KWP Local Identifier conversions ----
// PID04: table resolution 0.00488, currently interpreted as throttle
// sensor voltage. Standard OBD throttle % conversion is intentionally
// not enabled yet because closed/WOT calibration should be confirmed.
//
// PID05/08:
//   physical pressure [mmHg abs] = raw * 0.9277344
//   OBD MAP/BARO expects integer kPa.
//
// PID07:
//   physical degC = raw - 40
//   Standard OBD IAT byte = degC + 40, therefore low raw byte directly.

byte ConvertKWPPressureRawToOBDkPa(uint16_t raw);
byte ConvertKWPIATRawToOBD(uint16_t raw);

// Standard OBD PID 11 throttle position.
// Uses service-manual sensor endpoints as the initial calibration:
//   closed/idle approx 0.61 V
//   WOT approx         3.80 V
byte ConvertKWPThrottleRawToOBDPercent(uint16_t raw);

// Human-readable fixed-point helpers for diagnostics.
// No floating-point printf is needed on AVR.
uint16_t ConvertKWPPressureRawToTenthMmHg(uint16_t raw);
uint16_t ConvertKWPThrottleRawToMilliVolt(uint16_t raw);


// Fuel flow scale/unit is not confirmed yet.
bool FuelFlowConversionKnown(void);


#define KDS_FUEL_FLOW_LPH_PER_COUNT 0.003667f
bool ConvertFuelFlowFrameToRaw(const byte *data, byte len, uint16_t &raw);
float ConvertFuelFlowRawToLph(uint16_t raw);
bool ConvertFuelFlowRawToOBD5E(uint16_t raw, byte &A, byte &B);
uint16_t CalculateInstantFuelEconomyTenthKmPerL(uint16_t speedRaw, uint16_t fuelRaw);
uint16_t ConvertFuelFlowRawToHundredthLph(uint16_t raw);

#endif



