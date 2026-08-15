#ifndef CANBUS_H
#define CANBUS_H

#include <Arduino.h>

struct CachedCANFrame
{
    bool valid;
    byte len;
    byte data[8];
    unsigned long updatedMs;
};

bool InitCAN(void);

// Call as often as possible from loop().
void UpdateCANBus(void);

// KWP2000 / CAN requests. TX ID is fixed at known-working 0x7E4.
bool SendKWPStartDiagnosticSession(void);
bool SendKWPReadLocalIdentifier(byte pid);

// Latest response from FI ECU candidate RX ID 0x746.
// Each successful read clears the "ready" flag.
bool TakeKWPSessionPositive(void);
bool TakeKWPReadPositive(byte expectedPid, uint16_t &rawValue);

// Raw broadcast cache.
bool GetCachedCANFrame(unsigned long id, CachedCANFrame &out);

// CAN hardware/receive activity.
unsigned long LastCANReceiveMs(void);

#endif
