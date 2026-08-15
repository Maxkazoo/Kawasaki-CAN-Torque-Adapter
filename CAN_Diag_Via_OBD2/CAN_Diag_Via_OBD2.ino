#include "Config.h"
#include "Bluetooth.h"
#include "CANBus.h"
#include "LED.h"
#include "Version.h"
#include "Maintenance_Notes.h"
#include "OBD2Emulator.h"
#include "VehicleData.h"

/*
  PROJECT ARCHITECTURE / MAINTENANCE NOTES
  ========================================

  This sketch intentionally separates three layers:

    Kawasaki CAN/KWP acquisition
            |
            v
       RAM latest-value cache
            |
            v
    ELM327 / Torque response layer

  Torque requests are answered from RAM.  We do NOT wait for the motorcycle
  each time Torque asks a PID.  This keeps Torque response latency low and
  prevents smartphone polling speed from directly controlling the ECU.

  LED summary:
    BLUE A0 : Torque application traffic is active
    RED  A5 : Kawasaki CAN/KWP session error
    RED  D7 : Bluetooth physical/radio connection error

  See Maintenance_Notes.h for the development history and a function map.
*/

//==================================================
// Ver2.10
//
// Acquisition engine from Ver2.03 is retained.
//
// Fixed 100 ms KWP slots:
//   04,05,04,05,04,07,05,04,05,08
//
// ELM327/Torque layer:
//   Bluetooth sends NO unsolicited acquisition/debug lines.
//   It replies only to ELM/OBD commands.
//
// Initial standard Mode 01 mappings:
//   05 Coolant temperature
//   0B Intake MAP
//   0C Engine RPM
//   0D Vehicle speed
//   0F Intake air temperature
//   33 Barometric pressure
//
// Vehicle-specific conversions live in:
//   DataConversion.h / DataConversion.cpp
//==================================================

struct Value16
{
    bool valid;
    uint16_t raw;
    unsigned long updatedMs;
};

static Value16 throttle    ={false,0,0};
static Value16 intakePress ={false,0,0};
static Value16 intakeTemp  ={false,0,0};
static Value16 atmoPress   ={false,0,0};

enum DiagState
{
    DIAG_DISCONNECTED=0,
    DIAG_WAIT_SESSION,
    DIAG_CONNECTED,
    DIAG_WAIT_PID
};

static DiagState diagState=DIAG_DISCONNECTED;

static byte pendingPid=0;
static unsigned long requestStartedMs=0;
static unsigned long lastSessionTryMs=0;
static byte consecutiveFailures=0;

static const unsigned long SESSION_RETRY_MS=1000UL;
static const unsigned long SESSION_RESPONSE_TIMEOUT_MS=100UL;
static const unsigned long PID_RESPONSE_TIMEOUT_MS=80UL;
static const byte MAX_CONSECUTIVE_FAILURES=3;

static const byte SLOT_COUNT=10;
static const byte slotPid[SLOT_COUNT] =
{
    0x04,0x05,0x04,0x05,0x04,
    0x07,0x05,0x04,0x05,0x08
};

static byte slotIndex=0;
static unsigned long nextSlotMs=0;
static const unsigned long SLOT_PERIOD_MS=100UL;

static unsigned long lastUsbStatusMs=0;

//--------------------------------------------------
// VehicleData accessors for OBD2Emulator.cpp
//--------------------------------------------------
bool GetThrottleRaw(uint16_t &raw,unsigned long &updatedMs)
{
    if(!throttle.valid) return false;
    raw=throttle.raw;
    updatedMs=throttle.updatedMs;
    return true;
}

bool GetIntakePressureRaw(uint16_t &raw,unsigned long &updatedMs)
{
    if(!intakePress.valid) return false;
    raw=intakePress.raw;
    updatedMs=intakePress.updatedMs;
    return true;
}

bool GetIntakeAirTempRaw(uint16_t &raw,unsigned long &updatedMs)
{
    if(!intakeTemp.valid) return false;
    raw=intakeTemp.raw;
    updatedMs=intakeTemp.updatedMs;
    return true;
}

bool GetAtmosphericPressureRaw(uint16_t &raw,unsigned long &updatedMs)
{
    if(!atmoPress.valid) return false;
    raw=atmoPress.raw;
    updatedMs=atmoPress.updatedMs;
    return true;
}

bool KWPDataSessionValid(void)
{
    return diagState==DIAG_CONNECTED || diagState==DIAG_WAIT_PID;
}

//--------------------------------------------------
// Acquisition
//--------------------------------------------------
static void StorePid(byte pid,uint16_t raw)
{
    Value16 *v=0;

    if(pid==0x04) v=&throttle;
    else if(pid==0x05) v=&intakePress;
    else if(pid==0x07) v=&intakeTemp;
    else if(pid==0x08) v=&atmoPress;

    if(v)
    {
        v->valid=true;
        v->raw=raw;
        v->updatedMs=millis();
    }
}

static void EnterDisconnected()
{
    diagState=DIAG_DISCONNECTED;
    consecutiveFailures=0;
    SetCANError(true);
}

static void StartSessionTry()
{
    unsigned long now=millis();

    if(lastSessionTryMs!=0 &&
       (unsigned long)(now-lastSessionTryMs)<SESSION_RETRY_MS)
        return;

    lastSessionTryMs=now;

    if(!SendKWPStartDiagnosticSession())
    {
        SetCANError(true);
        return;
    }

    requestStartedMs=now;
    diagState=DIAG_WAIT_SESSION;
    SetCANError(true);
}

static void MarkSessionConnected()
{
    diagState=DIAG_CONNECTED;
    consecutiveFailures=0;
    SetCANError(false);

    slotIndex=0;
    nextSlotMs=millis()+100UL;
}

static void RegisterPidFailure()
{
    if(consecutiveFailures<255) consecutiveFailures++;

    if(consecutiveFailures>=MAX_CONSECUTIVE_FAILURES)
    {
        EnterDisconnected();
        lastSessionTryMs=0;
        return;
    }

    diagState=DIAG_CONNECTED;
}

static bool GetDuePid(byte &pid)
{
    unsigned long now=millis();

    if((long)(now-nextSlotMs)<0)
        return false;

    pid=slotPid[slotIndex];

    slotIndex++;
    if(slotIndex>=SLOT_COUNT)
        slotIndex=0;

    nextSlotMs+=SLOT_PERIOD_MS;

    if((long)(now-nextSlotMs)>=0)
        nextSlotMs=now+SLOT_PERIOD_MS;

    return true;
}

static void UpdateDiagnosticEngine()
{
    unsigned long now=millis();

    if(diagState==DIAG_DISCONNECTED)
    {
        StartSessionTry();
        return;
    }

    if(diagState==DIAG_WAIT_SESSION)
    {
        if(TakeKWPSessionPositive())
        {
            MarkSessionConnected();
            return;
        }

        if((unsigned long)(now-requestStartedMs)>=SESSION_RESPONSE_TIMEOUT_MS)
        {
            diagState=DIAG_DISCONNECTED;
            SetCANError(true);
        }
        return;
    }

    if(diagState==DIAG_WAIT_PID)
    {
        uint16_t raw=0;

        if(TakeKWPReadPositive(pendingPid,raw))
        {
            StorePid(pendingPid,raw);
            consecutiveFailures=0;
            diagState=DIAG_CONNECTED;
            return;
        }

        if((unsigned long)(now-requestStartedMs)>=PID_RESPONSE_TIMEOUT_MS)
        {
            RegisterPidFailure();
        }
        return;
    }

    byte pid=0;
    if(GetDuePid(pid))
    {
        if(SendKWPReadLocalIdentifier(pid))
        {
            pendingPid=pid;
            requestStartedMs=now;
            diagState=DIAG_WAIT_PID;
        }
        else
        {
            RegisterPidFailure();
        }
    }
}

static void UpdateBluetoothState()
{
    SetBTError(!BluetoothConnected());
}

//--------------------------------------------------
// UpdateTorqueActivityLED()
//
// BLUE A0 is an application-level indicator.  It stays ON only while
// Torque/ELM commands are actually being received.  A paired HC-06 with
// a stopped Torque app therefore produces BLUE=OFF but BT ERROR=OFF.
//--------------------------------------------------
static void UpdateTorqueActivityLED()
{
    SetTorqueActive(TorqueCommandActive());
}


// USB-only one-line heartbeat for bench debugging.
// It does not consume HC-06 bandwidth and does not disturb Torque.
static void PrintUsbStatus()
{
    unsigned long now=millis();
    if((unsigned long)(now-lastUsbStatusMs)<1000UL) return;
    lastUsbStatusMs=now;

    Serial.print(F("V2.10 KWP="));
    Serial.print(KWPDataSessionValid()?F("OK"):F("ERR"));
    Serial.print(F(" BT="));
    Serial.println(BluetoothConnected()?F("OK"):F("ERR"));
}

void setup()
{
    InitLED();

    pinMode(LED_BT_ERROR,OUTPUT);
    pinMode(LED_CAN_ERROR,OUTPUT);

    Serial.begin(115200);

    Serial.println();
    Serial.println(F("=============================="));
    Serial.println(VERSION_TEXT);
    Serial.println(F("=============================="));

    InitBluetooth();
    InitOBD2Emulator();

    if(!InitCAN())
    {
        Serial.println(F("CAN Init ERROR"));
        SetCANError(true);
        diagState=DIAG_DISCONNECTED;
    }
    else
    {
        Serial.println(F("CAN Init OK"));
        SetCANError(true);
        diagState=DIAG_DISCONNECTED;
    }

    UpdateBluetoothState();
    SetTorqueActive(false);

    Serial.println(F("ELM327/Torque mode active"));
    Serial.println(F("Bluetooth output is request/response only"));
}

void loop()
{
    // Drain CAN first.
    UpdateCANBus();

    // Maintain Kawasaki KWP cache.
    UpdateDiagnosticEngine();

    // Serve Torque/ELM requests without blocking acquisition.
    UpdateOBD2Emulator();
    UpdateTorqueActivityLED();

    UpdateBluetoothState();

    PrintUsbStatus();
    UpdateLED();
}
