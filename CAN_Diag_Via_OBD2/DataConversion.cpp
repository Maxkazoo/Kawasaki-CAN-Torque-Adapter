#include "DataConversion.h"

/*
  WHY THIS FILE EXISTS
  --------------------
  Kawasaki raw-signal interpretation changed several times during reverse
  engineering (notably vehicle speed and fuel flow).  Keeping all physical
  conversions here prevents protocol code from becoming littered with magic
  numbers and makes later calibration a one-file change.

  Current provisional items:
    0x110 speed = 0.1 km/h/count
    0x32F fuel flow = first two bytes, big-endian, coefficient defined in
                      KDS_FUEL_FLOW_LPH_PER_COUNT

  Fuel-flow coefficient is deliberately easy to tune against the motorcycle
  meter's instantaneous km/L display.
*/


// ReadBE16(): extracts one big-endian 16-bit field from a cached CAN frame.
uint16_t ReadBE16(const byte *data, byte len, byte index)
{
    if(!data || index+1>=len) return 0;
    return ((uint16_t)data[index]<<8) | data[index+1];
}

// ConvertBroadcastRPMToOBD(): maps Kawasaki RPM raw data to standard OBD PID 0C encoding.
bool ConvertBroadcastRPMToOBD(const byte *data, byte len, byte &A, byte &B)
{
    if(!data || len<2) return false;

    // Working vehicle mapping:
    // CAN raw = rpm.
    uint32_t rpm=ReadBE16(data,len,0);

    // Standard OBD PID 0C:
    // rpm = ((A*256)+B)/4
    uint32_t obd=rpm*4UL;
    if(obd>65535UL) obd=65535UL;

    A=(byte)(obd>>8);
    B=(byte)(obd&0xFF);
    return true;
}

// ConvertBroadcastSpeedToOBD(): assumes 0x110=0.1 km/h/count and maps it to OBD PID 0D.
bool ConvertBroadcastSpeedToOBD(const byte *data, byte len, byte &A)
{
    if(!data || len<2) return false;
    uint32_t raw=ReadBE16(data,len,0);
    uint32_t kmh=(raw+5UL)/10UL;
    if(kmh>255UL) kmh=255UL;
    A=(byte)kmh;
    return true;
}

// ConvertBroadcastWaterToOBD(): converts 0x120 coolant byte offset to OBD PID 05.
bool ConvertBroadcastWaterToOBD(const byte *data, byte len, byte &A)
{
    if(!data || len<2) return false;

    // Kawasaki broadcast:
    //   coolant degC = Data1 - 60
    //
    // OBD PID 05:
    //   coolant degC = A - 40
    //
    // Therefore:
    //   A = Data1 - 20
    int v=(int)data[1]-20;
    if(v<0) v=0;
    if(v>255) v=255;

    A=(byte)v;
    return true;
}

// ConvertKWPPressureRawToOBDkPa(): converts KDS pressure raw units to integer kPa for MAP/BARO.
byte ConvertKWPPressureRawToOBDkPa(uint16_t raw)
{
    // KDS table:
    // pressure [mmHg abs] = raw * 0.9277344
    //
    // 1 mmHg = 0.133322368 kPa
    // => kPa = raw * 0.123687747...
    //
    // Rounded integer implementation:
    uint32_t kpa=((uint32_t)raw*123688UL + 500000UL)/1000000UL;
    if(kpa>255UL) kpa=255UL;
    return (byte)kpa;
}

// ConvertKWPIATRawToOBD(): maps KDS IAT raw directly into the compatible OBD temperature byte.
byte ConvertKWPIATRawToOBD(uint16_t raw)
{
    // KDS:
    // IAT degC = raw - 40
    // OBD:
    // IAT degC = A - 40
    // So A == KDS raw for the observed range.
    if(raw>255U) raw=255U;
    return (byte)raw;
}


// ConvertKWPThrottleRawToOBDPercent(): maps throttle sensor voltage between calibrated closed/WOT endpoints.
byte ConvertKWPThrottleRawToOBDPercent(uint16_t raw)
{
    // KDS PID04 resolution = 0.00488 V/count.
    //
    // Initial calibration from service manual:
    //   closed/idle ~= 0.61 V
    //   WOT         ~= 3.80 V
    //
    // Work in millivolts:
    uint32_t mv=((uint32_t)raw*488UL + 50UL)/100UL;

    const uint32_t CLOSED_MV=610UL;
    const uint32_t WOT_MV=3800UL;

    if(mv<=CLOSED_MV) return 0;
    if(mv>=WOT_MV) return 255;

    uint32_t a=((mv-CLOSED_MV)*255UL + (WOT_MV-CLOSED_MV)/2UL) /
               (WOT_MV-CLOSED_MV);

    if(a>255UL) a=255UL;
    return (byte)a;
}

// ConvertKWPPressureRawToTenthMmHg(): diagnostic helper retaining 0.1-mmHg resolution.
uint16_t ConvertKWPPressureRawToTenthMmHg(uint16_t raw)
{
    // tenth-mmHg = raw * 9.277344
    return (uint16_t)(((uint32_t)raw*9277344UL + 500000UL)/1000000UL);
}

// ConvertKWPThrottleRawToMilliVolt(): diagnostic helper converting PID04 counts to mV.
uint16_t ConvertKWPThrottleRawToMilliVolt(uint16_t raw)
{
    // KDS table resolution = 0.00488 V/count = 4.88 mV/count.
    return (uint16_t)(((uint32_t)raw*488UL + 50UL)/100UL);
}


// FuelFlowConversionKnown(): remains false while 0x32F coefficient is provisional.
bool FuelFlowConversionKnown(void)
{
    return false;
}


// ConvertFuelFlowFrameToRaw(): provisional 0x32F parser using Data0:Data1 big-endian.
bool ConvertFuelFlowFrameToRaw(const byte *data, byte len, uint16_t &raw)
{
    if(!data || len<2) return false;
    raw=((uint16_t)data[0]<<8)|data[1];
    return true;
}

// ConvertFuelFlowRawToLph(): applies the single tunable fuel-flow coefficient.
float ConvertFuelFlowRawToLph(uint16_t raw)
{
    return (float)raw * KDS_FUEL_FLOW_LPH_PER_COUNT;
}

// ConvertFuelFlowRawToOBD5E(): maps provisional L/h to standard OBD PID 5E (0.05 L/h/bit).
bool ConvertFuelFlowRawToOBD5E(uint16_t raw, byte &A, byte &B)
{
    // First convert Kawasaki raw data using the ONE calibration constant.
    // OBD PID 5E represents Engine Fuel Rate in 0.05 L/h per count.
    const float fuelLph = ConvertFuelFlowRawToLph(raw);
    uint32_t obdRaw = (uint32_t)(fuelLph / 0.05f + 0.5f);

    if(obdRaw > 65535UL) obdRaw = 65535UL;

    A = (byte)(obdRaw >> 8);
    B = (byte)(obdRaw & 0xFF);
    return true;
}

// CalculateInstantFuelEconomyTenthKmPerL(): computes speed/fuel-flow and returns 0.1 km/L units.
uint16_t CalculateInstantFuelEconomyTenthKmPerL(uint16_t speedRaw,
                                               uint16_t fuelRaw)
{
    if(speedRaw == 0 || fuelRaw == 0) return 0;

    // 0x110 working assumption: 0.1 km/h per count.
    const float speedKmh = (float)speedRaw * 0.1f;

    // IMPORTANT:
    // Do not embed 0.01 or any other fuel coefficient here.
    // All fuel-flow users must go through ConvertFuelFlowRawToLph(),
    // which uses KDS_FUEL_FLOW_LPH_PER_COUNT from DataConversion.h.
    const float fuelLph = ConvertFuelFlowRawToLph(fuelRaw);
    if(fuelLph <= 0.0f) return 0;

    const float economyKmPerL = speedKmh / fuelLph;
    uint32_t tenth = (uint32_t)(economyKmPerL * 10.0f + 0.5f);

    if(tenth > 65535UL) tenth = 65535UL;
    return (uint16_t)tenth;
}


// Convert provisional Kawasaki fuel-flow raw value to 0.01 L/h units.
// This is the encoding used by custom Torque PID 22F331.
uint16_t ConvertFuelFlowRawToHundredthLph(uint16_t raw)
{
    float lph = ConvertFuelFlowRawToLph(raw);
    if(lph <= 0.0f) return 0;

    uint32_t v = (uint32_t)(lph * 100.0f + 0.5f);
    if(v > 65535UL) v = 65535UL;
    return (uint16_t)v;
}
