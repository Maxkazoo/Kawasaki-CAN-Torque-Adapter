#include "OBD2Emulator.h"
#include "Bluetooth.h"
#include "CANBus.h"
#include "DataConversion.h"
#include "VehicleData.h"

#include <Arduino.h>
#include <ctype.h>
#include <string.h>

/*
  ELM327/Torque compatibility layer
  =================================

  IMPORTANT DEVELOPMENT HISTORY

  1) The hardest Bluetooth bug was caused by TWO functions reading the same
     SoftwareSerial RX stream.  UpdateBluetooth() and UpdateOBD2Emulator()
     each consumed some characters, so commands such as ATE0 / ATL0 / ATSP6
     were split into fragments.  The final rule is strict:
         OBD2Emulator.cpp is the ONLY Bluetooth RX consumer.

  2) Torque does not necessarily request every gauge just because the gauge
     is placed on screen.  It first reads the Mode 01 "supported PID" bitmap.
     An incorrect bitmap caused only coolant and IAT to be polled.  Therefore
     Handle0100/0120/0140 must be kept consistent with the implemented PIDs.

  3) Torque often appends an expected-response-count nibble:
         010C  and 010C1
     are the same PID request for our emulator.  MatchMode01Pid() accepts
     both forms.

  4) Unknown Torque requests are NEVER forwarded blindly to the motorcycle.
     This adapter is a whitelist emulator.  Kawasaki KWP polling is controlled
     internally by the acquisition engine.

  5) Ver2.16 adds application-level connection detection.  The BLUE LED is
     ON only while syntactically valid AT / Mode01 / Mode22 commands arrive.
     HC-06 STATE is handled separately by the Bluetooth error LED.
*/

// Minimal ELM327-compatible command layer for Torque.

// No unsolicited Bluetooth output is generated here.

static char cmd[32];
static char currentCommand[32];
static byte cmdLen=0;

static bool echoEnabled=true;
static bool spacesEnabled=true;
static bool headersEnabled=false;
static bool linefeedEnabled=false;

// Torque application activity timeout.
// Torque normally polls much faster than this; 2 s is intentionally generous
// so normal scheduler jitter does not make the BLUE LED flicker.
static const unsigned long TORQUE_ACTIVE_TIMEOUT_MS = 2000UL;
static unsigned long lastTorqueCommandMs = 0;


// Forward declarations used by the command-history helpers.
static void Prompt();
static void ReplyLine(const char *s);

//--------------------------------------------------
// Compact Torque command history
//--------------------------------------------------
#define CMD_HISTORY_COUNT 6
#define CMD_HISTORY_LEN   14

struct CmdHistoryEntry
{
    char command[CMD_HISTORY_LEN];
    byte result; // 0=OK, 1=NO DATA, 2=?, 3=AT/other OK
};

static CmdHistoryEntry history[CMD_HISTORY_COUNT];
static byte historyHead=0;
static byte historyUsed=0;

//--------------------------------------------------
// Fuel/economy diagnostic snapshot (Ver2.19a)
//
// Nothing is printed spontaneously while Torque is connected.
// The last internal state is retained here and can be queried later with
// ATFUEL from Serial Bluetooth Terminal after Torque is disconnected.
//--------------------------------------------------
static uint16_t dbgFuelRaw=0;
static uint16_t dbgSpeedRaw=0;
static uint16_t dbgFuelHundredth=0;   // 0.01 L/h
static uint16_t dbgEconomyTenth=0;    // 0.1 km/L
static unsigned long dbgSpeedAgeMs=0;
static unsigned long dbgFuelAgeMs=0;
static unsigned long dbgFuelReqCount=0;
static unsigned long dbgEconomyReqCount=0;
static unsigned long dbgEconomyNoDataCount=0;
static byte dbgEconomyStatus=0;
// status: 0=never, 1=OK, 2=no speed frame, 3=no fuel frame,
//         4=bad/stale speed, 5=bad/stale fuel, 6=bad fuel frame


static void AddHistory(const char *command,byte result)
{
    CmdHistoryEntry &e=history[historyHead];

    strncpy(e.command,command,CMD_HISTORY_LEN-1);
    e.command[CMD_HISTORY_LEN-1]=0;
    e.result=result;

    historyHead++;
    if(historyHead>=CMD_HISTORY_COUNT) historyHead=0;
    if(historyUsed<CMD_HISTORY_COUNT) historyUsed++;
}

static const char *HistoryResult(byte r)
{
    if(r==0) return "OK";
    if(r==1) return "NO DATA";
    if(r==2) return "UNSUPPORTED";
    return "AT OK";
}

static void DumpHistory()
{
    ReplyLine("Torque RX history:");

    byte start=(historyHead+CMD_HISTORY_COUNT-historyUsed)%CMD_HISTORY_COUNT;

    for(byte i=0;i<historyUsed;i++)
    {
        byte n=(start+i)%CMD_HISTORY_COUNT;
        WriteBluetooth(history[n].command);
        WriteBluetooth(" -> ");
        ReplyLine(HistoryResult(history[n].result));
    }

    Prompt();
}

static void ClearHistory()
{
    historyHead=0;
    historyUsed=0;
}

static void Prompt()
{
    WriteBluetoothChar('>');
}

static void EndLine()
{
    WriteBluetooth("\r");
    if(linefeedEnabled) WriteBluetooth("\n");
}

static void ReplyLine(const char *s)
{
    WriteBluetooth(s);
    EndLine();
}

static void ReplyOK()
{
    ReplyLine("OK");
    Prompt();
}

static void ReplyNoData()
{
    ReplyLine("NO DATA");
    Prompt();
}

static void ReplyQuestion()
{
    ReplyLine("?");
    Prompt();
}

static void Normalize(char *s)
{
    char *d=s;
    while(*s)
    {
        char c=*s++;
        if(c==' ' || c=='\t') continue;
        *d++=(char)toupper((unsigned char)c);
    }
    *d=0;
}

static void HexByte(byte v,char *out)
{
    static const char hex[]="0123456789ABCDEF";
    out[0]=hex[(v>>4)&0x0F];
    out[1]=hex[v&0x0F];
}

static void ReplyBytes(const byte *data,byte len)
{
    char line[40];
    byte p=0;

    if(headersEnabled)
    {
        // A conventional-looking ECU response header for the emulator.
        // Torque normally disables headers, but this is useful if requested.
        line[p++]='7'; line[p++]='E'; line[p++]='8';
        if(spacesEnabled) line[p++]=' ';
    }

    for(byte i=0;i<len;i++)
    {
        if(p+3>=sizeof(line)) break;
        HexByte(data[i],&line[p]);
        p+=2;
        if(spacesEnabled && i+1<len) line[p++]=' ';
    }
    line[p]=0;

    ReplyLine(line);
    Prompt();
}

static bool Fresh(unsigned long updatedMs,unsigned long maxAge)
{
    return updatedMs!=0 &&
           (unsigned long)(millis()-updatedMs)<=maxAge;
}

static void Handle0100()
{
    // Supported: 01,05,0B,0C,0D,0F,11,20
    byte r[6]={0x41,0x00,0x88,0x3A,0x80,0x01};
    ReplyBytes(r,6);
}

static void Handle0120()
{
    byte r[6]={0x41,0x20,0x00,0x00,0x20,0x01}; // PID33 + PID40
    ReplyBytes(r,6);
}

static void HandlePID05()
{
    CachedCANFrame f;
    byte A=0;

    if(!GetCachedCANFrame(KDS_CAN_ID_WATER,f) ||
       !f.valid || !Fresh(f.updatedMs,1500UL) ||
       !ConvertBroadcastWaterToOBD(f.data,f.len,A))
    {
        ReplyNoData();
        return;
    }

    byte r[3]={0x41,0x05,A};
    ReplyBytes(r,3);
}

static void HandlePID0B()
{
    uint16_t raw=0;
    unsigned long updated=0;

    if(!GetIntakePressureRaw(raw,updated) ||
       !Fresh(updated,600UL))
    {
        ReplyNoData();
        return;
    }

    byte A=ConvertKWPPressureRawToOBDkPa(raw);
    byte r[3]={0x41,0x0B,A};
    ReplyBytes(r,3);
}

static void HandlePID0C()
{
    CachedCANFrame f;
    byte A=0,B=0;

    if(!GetCachedCANFrame(KDS_CAN_ID_RPM,f) ||
       !f.valid || !Fresh(f.updatedMs,1500UL) ||
       !ConvertBroadcastRPMToOBD(f.data,f.len,A,B))
    {
        ReplyNoData();
        return;
    }

    byte r[4]={0x41,0x0C,A,B};
    ReplyBytes(r,4);
}

static void HandlePID0D()
{
    CachedCANFrame f;
    byte A=0;

    if(!GetCachedCANFrame(KDS_CAN_ID_SPEED,f) ||
       !f.valid || !Fresh(f.updatedMs,1500UL) ||
       !ConvertBroadcastSpeedToOBD(f.data,f.len,A))
    {
        ReplyNoData();
        return;
    }

    byte r[3]={0x41,0x0D,A};
    ReplyBytes(r,3);
}

static void HandlePID0F()
{
    uint16_t raw=0;
    unsigned long updated=0;

    if(!GetIntakeAirTempRaw(raw,updated) ||
       !Fresh(updated,2500UL))
    {
        ReplyNoData();
        return;
    }

    byte A=ConvertKWPIATRawToOBD(raw);
    byte r[3]={0x41,0x0F,A};
    ReplyBytes(r,3);
}

static void HandlePID11()
{
    uint16_t raw=0;
    unsigned long updated=0;

    if(!GetThrottleRaw(raw,updated) ||
       !Fresh(updated,600UL))
    {
        ReplyNoData();
        return;
    }

    byte A=ConvertKWPThrottleRawToOBDPercent(raw);
    byte r[3]={0x41,0x11,A};
    ReplyBytes(r,3);
}

static void HandlePID33()
{
    uint16_t raw=0;
    unsigned long updated=0;

    if(!GetAtmosphericPressureRaw(raw,updated) ||
       !Fresh(updated,2500UL))
    {
        ReplyNoData();
        return;
    }

    byte A=ConvertKWPPressureRawToOBDkPa(raw);
    byte r[3]={0x41,0x33,A};
    ReplyBytes(r,3);
}



static void Handle0140()
{
    // Supported PIDs 41..60:
    //   PID5E = Engine Fuel Rate
    //   PID60 = continuation to 61..80
    //
    // PID5E is bit 2 of the fourth bitmap byte (0x04).
    // PID60 is bit 0 of the fourth bitmap byte (0x01).
    byte r[6]={0x41,0x40,0x00,0x00,0x00,0x05};
    ReplyBytes(r,6);
}


//--------------------------------------------------
// Extended Supported-PID chain required to expose standard PID A4.
//
// Torque only discovers higher-numbered standard PIDs when each 32-PID
// block advertises the continuation PID at the end of that block.
// Therefore PID60 -> PID80 -> PIDA0 must be advertised before Torque can
// discover PIDA4.
//--------------------------------------------------
static void Handle0160()
{
    // Supported PIDs 61..80:
    // only PID80 (continuation to 81..A0).
    byte r[6]={0x41,0x60,0x00,0x00,0x00,0x01};
    ReplyBytes(r,6);
}

static void Handle0180()
{
    // Supported PIDs 81..A0:
    // only PIDA0 (continuation to A1..C0).
    byte r[6]={0x41,0x80,0x00,0x00,0x00,0x01};
    ReplyBytes(r,6);
}

static void Handle01A0()
{
    // Supported PIDs A1..C0:
    // PIDA4 Transmission Actual Gear.
    //
    // A1=0x80, A2=0x40, A3=0x20, A4=0x10 in the first bitmap byte.
    byte r[6]={0x41,0xA0,0x10,0x00,0x00,0x00};
    ReplyBytes(r,6);
}

static void HandlePID5E()
{
    CachedCANFrame f; uint16_t raw=0; byte A=0,B=0;
    if(!GetCachedCANFrame(KDS_CAN_ID_FUEL,f) || !f.valid ||
       (unsigned long)(millis()-f.updatedMs)>1500UL ||
       !ConvertFuelFlowFrameToRaw(f.data,f.len,raw) ||
       !ConvertFuelFlowRawToOBD5E(raw,A,B))
    { ReplyNoData(); return; }
    byte r[4]={0x41,0x5E,A,B}; ReplyBytes(r,4);
}

static void HandleInstantEconomyCustom()
{
    CachedCANFrame fs,ff; uint16_t fuelRaw=0;
    if(!GetCachedCANFrame(KDS_CAN_ID_SPEED,fs) ||
       !GetCachedCANFrame(KDS_CAN_ID_FUEL,ff) ||
       !fs.valid || !ff.valid || fs.len<2 ||
       (unsigned long)(millis()-fs.updatedMs)>1500UL ||
       (unsigned long)(millis()-ff.updatedMs)>1500UL ||
       !ConvertFuelFlowFrameToRaw(ff.data,ff.len,fuelRaw))
    { ReplyNoData(); return; }
    uint16_t speedRaw=ReadBE16(fs.data,fs.len,0);
    uint16_t econ=CalculateInstantFuelEconomyTenthKmPerL(speedRaw,fuelRaw);
    byte r[5]={0x62,0xF3,0x30,(byte)(econ>>8),(byte)econ};
    ReplyBytes(r,5);
}

// Kawasaki custom Mode 22 raw access.
// These are separate from standard Mode 01 PIDs so that uncertain
// conversions (notably fuel flow) are not presented with false units.

static void ReplyMode22RawFrame(byte didHi, byte didLo, unsigned long canId)
{
    CachedCANFrame f;
    if(!GetCachedCANFrame(canId,f) || !f.valid)
    {
        ReplyNoData();
        return;
    }

    byte r[12];
    byte p=0;
    r[p++]=0x62;
    r[p++]=didHi;
    r[p++]=didLo;
    r[p++]=f.len;

    for(byte i=0;i<f.len && p<sizeof(r);i++)
        r[p++]=f.data[i];

    ReplyBytes(r,p);
}

static void ReplyMode22KwpRaw(byte didLo,
                              bool (*getter)(uint16_t&,unsigned long&),
                              unsigned long maxAge)
{
    uint16_t raw=0;
    unsigned long updated=0;

    if(!getter(raw,updated) ||
       updated==0 ||
       (unsigned long)(millis()-updated)>maxAge)
    {
        ReplyNoData();
        return;
    }

    byte r[5]={0x62,0xF2,didLo,(byte)(raw>>8),(byte)(raw&0xFF)};
    ReplyBytes(r,5);
}


//--------------------------------------------------
// HandleCustomFuelFlow()
//
// Legacy/fallback Torque Custom PID:
//   Request : 22 F3 31
//   Reply   : 62 F3 31 A B
//
// Ver2.20 primary Torque path is private Mode 01 PID 01F0 because
// Mode 22 was observed to update unreliably in Torque.
//
// A:B is fuel flow in 0.01 L/h units.
// Torque equation:
//   ((A*256)+B)/100
//--------------------------------------------------
static void HandleCustomFuelFlow()
{
    dbgFuelReqCount++;

    CachedCANFrame f;
    uint16_t raw=0;

    if(!GetCachedCANFrame(KDS_CAN_ID_FUEL,f) ||
       !f.valid ||
       (unsigned long)(millis()-f.updatedMs)>1500UL ||
       !ConvertFuelFlowFrameToRaw(f.data,f.len,raw))
    {
        ReplyNoData();
        return;
    }

    uint16_t value=ConvertFuelFlowRawToHundredthLph(raw);

    dbgFuelRaw=raw;
    dbgFuelHundredth=value;
    dbgFuelAgeMs=(unsigned long)(millis()-f.updatedMs);

    byte r[5]={0x62,0xF3,0x31,(byte)(value>>8),(byte)value};
    ReplyBytes(r,5);
}

//--------------------------------------------------
// HandleCustomInstantEconomy()
//
// Legacy/fallback Torque Custom PID:
//   Request : 22 F3 30
//   Reply   : 62 F3 30 A B
//
// Ver2.20 primary Torque path is private Mode 01 PID 01F1.
//
// A:B is instantaneous economy in 0.1 km/L units.
// Torque equation:
//   ((A*256)+B)/10
//--------------------------------------------------
static void HandleCustomInstantEconomy()
{
    dbgEconomyReqCount++;

    CachedCANFrame fs,ff;
    uint16_t fuelRaw=0;

    if(!GetCachedCANFrame(KDS_CAN_ID_SPEED,fs))
    {
        dbgEconomyStatus=2; dbgEconomyNoDataCount++;
        ReplyNoData(); return;
    }

    if(!GetCachedCANFrame(KDS_CAN_ID_FUEL,ff))
    {
        dbgEconomyStatus=3; dbgEconomyNoDataCount++;
        ReplyNoData(); return;
    }

    dbgSpeedAgeMs=(unsigned long)(millis()-fs.updatedMs);
    dbgFuelAgeMs=(unsigned long)(millis()-ff.updatedMs);

    if(!fs.valid || fs.len<2 || dbgSpeedAgeMs>1500UL)
    {
        dbgEconomyStatus=4; dbgEconomyNoDataCount++;
        ReplyNoData(); return;
    }

    if(!ff.valid || dbgFuelAgeMs>1500UL)
    {
        dbgEconomyStatus=5; dbgEconomyNoDataCount++;
        ReplyNoData(); return;
    }

    if(!ConvertFuelFlowFrameToRaw(ff.data,ff.len,fuelRaw))
    {
        dbgEconomyStatus=6; dbgEconomyNoDataCount++;
        ReplyNoData(); return;
    }

    uint16_t speedRaw=ReadBE16(fs.data,fs.len,0);
    uint16_t value=CalculateInstantFuelEconomyTenthKmPerL(speedRaw,fuelRaw);

    dbgSpeedRaw=speedRaw;
    dbgFuelRaw=fuelRaw;
    dbgFuelHundredth=ConvertFuelFlowRawToHundredthLph(fuelRaw);
    dbgEconomyTenth=value;
    dbgEconomyStatus=1;

    byte r[5]={0x62,0xF3,0x30,(byte)(value>>8),(byte)value};
    ReplyBytes(r,5);
}



//--------------------------------------------------
// HandlePrivatePIDF0FuelFlow()
//
// Torque Custom PID using Mode 01 instead of Mode 22.
//
//   Request : 01 F0
//   Reply   : 41 F0 A B
//
// A:B = fuel flow in 0.01 L/h units.
// Torque equation:
//   ((A*256)+B)/100
//
// IMPORTANT:
// PID F0 is used here as a PRIVATE emulator PID.  It is not claimed to be
// a standardized SAE PID.  The purpose is to avoid the unstable Torque
// Mode 22 custom-PID behavior observed during testing.
//--------------------------------------------------
static void HandlePrivatePIDF0FuelFlow()
{
    CachedCANFrame f;
    uint16_t raw=0;

    if(!GetCachedCANFrame(KDS_CAN_ID_FUEL,f) ||
       !f.valid ||
       (unsigned long)(millis()-f.updatedMs)>1500UL ||
       !ConvertFuelFlowFrameToRaw(f.data,f.len,raw))
    {
        ReplyNoData();
        return;
    }

    uint16_t value=ConvertFuelFlowRawToHundredthLph(raw);

    // Keep ATFUEL diagnostics useful even though the primary Torque path
    // has moved from 22F331 to private Mode 01 PID F0.
    dbgFuelRaw=raw;
    dbgFuelHundredth=value;
    dbgFuelAgeMs=(unsigned long)(millis()-f.updatedMs);
    dbgFuelReqCount++;

    byte r[4]={0x41,0xF0,(byte)(value>>8),(byte)value};
    ReplyBytes(r,4);
}


//--------------------------------------------------
// HandlePrivatePIDF1InstantEconomy()
//
// Torque Custom PID:
//
//   Request : 01 F1
//   Reply   : 41 F1 A B
//
// A:B = instantaneous fuel economy in 0.1 km/L units.
// Torque equation:
//   ((A*256)+B)/10
//
// Calculation:
//   vehicle speed [km/h] / fuel flow [L/h]
//
// Zero division is explicitly prevented in
// CalculateInstantFuelEconomyTenthKmPerL().
//--------------------------------------------------
static void HandlePrivatePIDF1InstantEconomy()
{
    CachedCANFrame fs,ff;
    uint16_t fuelRaw=0;

    dbgEconomyReqCount++;

    if(!GetCachedCANFrame(KDS_CAN_ID_SPEED,fs))
    {
        dbgEconomyStatus=2; dbgEconomyNoDataCount++;
        ReplyNoData(); return;
    }

    if(!GetCachedCANFrame(KDS_CAN_ID_FUEL,ff))
    {
        dbgEconomyStatus=3; dbgEconomyNoDataCount++;
        ReplyNoData(); return;
    }

    dbgSpeedAgeMs=(unsigned long)(millis()-fs.updatedMs);
    dbgFuelAgeMs=(unsigned long)(millis()-ff.updatedMs);

    if(!fs.valid || fs.len<2 || dbgSpeedAgeMs>1500UL)
    {
        dbgEconomyStatus=4; dbgEconomyNoDataCount++;
        ReplyNoData(); return;
    }

    if(!ff.valid || dbgFuelAgeMs>1500UL)
    {
        dbgEconomyStatus=5; dbgEconomyNoDataCount++;
        ReplyNoData(); return;
    }

    if(!ConvertFuelFlowFrameToRaw(ff.data,ff.len,fuelRaw))
    {
        dbgEconomyStatus=6; dbgEconomyNoDataCount++;
        ReplyNoData(); return;
    }

    uint16_t speedRaw=ReadBE16(fs.data,fs.len,0);
    uint16_t value=CalculateInstantFuelEconomyTenthKmPerL(speedRaw,fuelRaw);

    dbgSpeedRaw=speedRaw;
    dbgFuelRaw=fuelRaw;
    dbgFuelHundredth=ConvertFuelFlowRawToHundredthLph(fuelRaw);
    dbgEconomyTenth=value;
    dbgEconomyStatus=1;

    byte r[4]={0x41,0xF1,(byte)(value>>8),(byte)value};
    ReplyBytes(r,4);
}


//--------------------------------------------------
// HandlePIDA4() - Standard OBD-II Transmission Actual Gear
//
// Information supplied by TriB on Arduino Forum:
//
//   responseByte = (gear << 4) + 1
//
// The low bit indicates that the "current gear" field is supported.
// This gives:
//   Neutral -> 0x01
//   1st     -> 0x11
//   2nd     -> 0x21
//   ...
//   6th     -> 0x61
//
// Vehicle source remains broadcast CAN 0x121:
//   Data0 = gear position
//   Data1 = Neutral flag
//
// The existing custom PID 22F321 is intentionally retained during the
// evaluation period so standard PIDA4 and the proven custom display can
// be compared side by side.
//--------------------------------------------------
static void HandlePIDA4()
{
    CachedCANFrame f;

    if(!GetCachedCANFrame(KDS_CAN_ID_GEAR,f) ||
       !f.valid ||
       f.len<1 ||
       (unsigned long)(millis()-f.updatedMs)>1500UL)
    {
        ReplyNoData();
        return;
    }

    byte gear=f.data[0];

    // Data1 has already been identified as the Neutral indication.
    if(f.len>=2 && f.data[1]!=0)
        gear=0;

    if(gear>6)
    {
        ReplyNoData();
        return;
    }

    byte gearByte=(byte)((gear<<4) | 0x01);
    byte r[3]={0x41,0xA4,gearByte};
    ReplyBytes(r,3);
}

//--------------------------------------------------
// HandleCustomGear()
//
// Torque Custom PID:
//   Request : 22 F3 21
//   Reply   : 62 F3 21 A
//
// A = 0 Neutral, 1..6 = gear position.
//
// Current reverse-engineered interpretation of CAN 0x121:
//   Data0 = gear position
//   Data1 = neutral flag
//
// For robustness, if Data1 indicates neutral, return 0 regardless of Data0.
//--------------------------------------------------
static void HandleCustomGear()
{
    CachedCANFrame f;

    if(!GetCachedCANFrame(KDS_CAN_ID_GEAR,f) ||
       !f.valid ||
       f.len<1 ||
       (unsigned long)(millis()-f.updatedMs)>1500UL)
    {
        ReplyNoData();
        return;
    }

    byte gear=f.data[0];

    // If a second byte exists and clearly indicates neutral, force 0.
    if(f.len>=2 && f.data[1]!=0)
        gear=0;

    // Reject obviously invalid values rather than showing nonsense.
    if(gear>6)
    {
        ReplyNoData();
        return;
    }

    byte r[4]={0x62,0xF3,0x21,gear};
    ReplyBytes(r,4);
}

static void HandleMode22(const char *c)
{
    // Final Torque custom engineering-value PIDs.
    if(!strcmp(c,"22F331")) { HandleCustomFuelFlow(); return; }
    if(!strcmp(c,"22F330")) { HandleCustomInstantEconomy(); return; }
    if(!strcmp(c,"22F321")) { HandleCustomGear(); return; }

    if(!strcmp(c,"22F100")) { ReplyMode22RawFrame(0xF1,0x00,KDS_CAN_ID_RPM); return; }
    if(!strcmp(c,"22F110")) { ReplyMode22RawFrame(0xF1,0x10,KDS_CAN_ID_SPEED); return; }
    if(!strcmp(c,"22F120")) { ReplyMode22RawFrame(0xF1,0x20,KDS_CAN_ID_WATER); return; }
    if(!strcmp(c,"22F121")) { ReplyMode22RawFrame(0xF1,0x21,KDS_CAN_ID_GEAR); return; }
    if(!strcmp(c,"22F32F")) { ReplyMode22RawFrame(0xF3,0x2F,KDS_CAN_ID_FUEL); return; }

    if(!strcmp(c,"22F204")) { ReplyMode22KwpRaw(0x04,GetThrottleRaw,600UL); return; }
    if(!strcmp(c,"22F205")) { ReplyMode22KwpRaw(0x05,GetIntakePressureRaw,600UL); return; }
    if(!strcmp(c,"22F207")) { ReplyMode22KwpRaw(0x07,GetIntakeAirTempRaw,2500UL); return; }
    if(!strcmp(c,"22F208")) { ReplyMode22KwpRaw(0x08,GetAtmosphericPressureRaw,2500UL); return; }

    ReplyNoData();
}


static void ReplyUIntFlash(const __FlashStringHelper *name,unsigned long value)
{
    char b[11];
    ultoa(value,b,10);
    WriteBluetoothFlash(name);
    ReplyLine(b);
}

static void DumpFuelDiagnostic()
{
    WriteBluetoothFlash(F("Fuel diagnostic snapshot:\r\n"));
    ReplyUIntFlash(F("K x1e5="),1000UL);
    ReplyUIntFlash(F("FUEL_REQ="),dbgFuelReqCount);
    ReplyUIntFlash(F("KFE_REQ="),dbgEconomyReqCount);
    ReplyUIntFlash(F("KFE_NODATA="),dbgEconomyNoDataCount);
    ReplyUIntFlash(F("STATUS="),dbgEconomyStatus);
    ReplyUIntFlash(F("SPD_RAW="),dbgSpeedRaw);
    ReplyUIntFlash(F("FUEL_RAW="),dbgFuelRaw);
    ReplyUIntFlash(F("SPD_AGE_MS="),dbgSpeedAgeMs);
    ReplyUIntFlash(F("FUEL_AGE_MS="),dbgFuelAgeMs);
    ReplyUIntFlash(F("FUEL_x0.01LPH="),dbgFuelHundredth);
    ReplyUIntFlash(F("KFE_x0.1KMPL="),dbgEconomyTenth);
    Prompt();
}

static void HandleAT(const char *c)
{
    if(!strcmp(c,"ATZ") || !strcmp(c,"ATWS"))
    {
        echoEnabled=true;
        spacesEnabled=true;
        headersEnabled=false;
        linefeedEnabled=false;
        ReplyLine("ELM327 v1.5");
        Prompt();
        return;
    }

    if(!strcmp(c,"ATD"))
    {
        echoEnabled=true;
        spacesEnabled=true;
        headersEnabled=false;
        linefeedEnabled=false;
        ReplyOK();
        return;
    }

    if(!strcmp(c,"ATI"))
    {
        ReplyLine("ELM327 v1.5");
        Prompt();
        return;
    }

    if(!strcmp(c,"ATE0")) { echoEnabled=false; ReplyOK(); return; }
    if(!strcmp(c,"ATE1")) { echoEnabled=true;  ReplyOK(); return; }
    if(!strcmp(c,"ATS0")) { spacesEnabled=false; ReplyOK(); return; }
    if(!strcmp(c,"ATS1")) { spacesEnabled=true;  ReplyOK(); return; }
    if(!strcmp(c,"ATH0")) { headersEnabled=false; ReplyOK(); return; }
    if(!strcmp(c,"ATH1")) { headersEnabled=true;  ReplyOK(); return; }
    if(!strcmp(c,"ATL0")) { linefeedEnabled=false; ReplyOK(); return; }
    if(!strcmp(c,"ATL1")) { linefeedEnabled=true;  ReplyOK(); return; }

    if(!strcmp(c,"ATSP0") || !strcmp(c,"ATSP6") ||
       !strcmp(c,"ATAT0") || !strcmp(c,"ATAT1") || !strcmp(c,"ATAT2") ||
       !strcmp(c,"ATAL")  || !strcmp(c,"ATM0")  || !strcmp(c,"ATM1") ||
       !strcmp(c,"ATCAF0")|| !strcmp(c,"ATCAF1") ||
       !strcmp(c,"ATPC")  || !strcmp(c,"ATIGN") ||
       !strcmp(c,"ATCFC0")|| !strcmp(c,"ATCFC1") ||
       !strncmp(c,"ATST",4) || !strncmp(c,"ATSH",4) ||
       !strncmp(c,"ATCRA",5)|| !strncmp(c,"ATCF",4) ||
       !strncmp(c,"ATCM",4) || !strncmp(c,"ATSW",4) ||
       !strncmp(c,"ATIB",4) || !strncmp(c,"ATTA",4) ||
       !strncmp(c,"ATCEA",5)|| !strncmp(c,"ATFC",4))
    {
        ReplyOK();
        return;
    }

    if(!strcmp(c,"ATDP"))
    {
        ReplyLine("ISO 15765-4 (CAN 11/500)");
        Prompt();
        return;
    }

    if(!strcmp(c,"ATDPN"))
    {
        ReplyLine("A6");
        Prompt();
        return;
    }

    if(!strcmp(c,"AT@1"))
    {
        ReplyLine("Kawasaki CAN Adapter");
        Prompt();
        return;
    }

    if(!strcmp(c,"ATKDS"))
    {
        ReplyLine("STD:05,0B,0C,0D,0F,11,33");
        ReplyLine("RAW:22F100,F110,F120,F121,F32F,F204,F205,F207,F208");
        Prompt();
        return;
    }

    if(!strcmp(c,"ATRV"))
    {
        // No ADC battery-voltage measurement is wired yet.
        ReplyLine("12.0V");
        Prompt();
        return;
    }

    ReplyQuestion();
}

static bool MatchMode01Pid(const char *c,const char *base)
{
    // Torque/ELM clients may append an expected-response-count nibble:
    //   010C   or   010C1
    //
    // Accept the exact 4-char request or one extra hex digit.
    if(strncmp(c,base,4)!=0) return false;

    byte n=(byte)strlen(c);
    if(n==4) return true;

    if(n==5 && isxdigit((unsigned char)c[4]))
        return true;

    return false;
}


//--------------------------------------------------
// LooksLikeTorqueCommand()
//
// Distinguishes real ELM/Torque traffic from Bluetooth-module status strings
// such as "+CONNECTING..." or "+DISC:SUCCESS".  Only recognized command
// families refresh the BLUE Torque-active LED timeout.
//--------------------------------------------------
static bool LooksLikeTorqueCommand(const char *c)
{
    if(!c || !c[0]) return false;

    if(c[0]=='A' && c[1]=='T') return true;  // ELM AT command
    if(c[0]=='0' && c[1]=='1') return true;  // OBD Mode 01
    if(c[0]=='2' && c[1]=='2') return true;  // custom Mode 22

    return false;
}

static void ProcessCommand(char *c)
{
    Normalize(c);

    strncpy(currentCommand,c,sizeof(currentCommand)-1);
    currentCommand[sizeof(currentCommand)-1]=0;

    if(c[0]==0)
    {
        Prompt();
        return;
    }

    // A valid-looking command means Torque (or a terminal acting as ELM
    // client) is actively talking to us.  This is the source for A0 BLUE.
    if(LooksLikeTorqueCommand(c))
        lastTorqueCommandMs=millis();

    // Diagnostic command: retrieve the last Torque/ELM commands.
    // Intended to be used after disconnecting Torque and reconnecting
    // with Serial Bluetooth Terminal.
    if(!strcmp(c,"ATLOG"))
    {
        DumpHistory();
        return;
    }

    if(!strcmp(c,"ATLOGCLR"))
    {
        ClearHistory();
        ReplyOK();
        return;
    }

    if(!strcmp(c,"ATFUEL"))
    {
        DumpFuelDiagnostic();
        return;
    }

    if(!strncmp(c,"AT",2))
    {
        AddHistory(c,3);
        HandleAT(c);
        return;
    }

    if(MatchMode01Pid(c,"0100")) { AddHistory(c,0); Handle0100(); return; }
    if(MatchMode01Pid(c,"0120")) { AddHistory(c,0); Handle0120(); return; }
    if(MatchMode01Pid(c,"0140")) { AddHistory(c,0); Handle0140(); return; }
    if(MatchMode01Pid(c,"0160")) { AddHistory(c,0); Handle0160(); return; }
    if(MatchMode01Pid(c,"0180")) { AddHistory(c,0); Handle0180(); return; }
    if(MatchMode01Pid(c,"01A0")) { AddHistory(c,0); Handle01A0(); return; }

    if(MatchMode01Pid(c,"0101"))
    {
        AddHistory(c,0);
        byte r[6]={0x41,0x01,0x00,0x00,0x00,0x00};
        ReplyBytes(r,6);
        return;
    }

    if(MatchMode01Pid(c,"0105")) { AddHistory(c,0); HandlePID05(); return; }
    if(MatchMode01Pid(c,"010B")) { AddHistory(c,0); HandlePID0B(); return; }
    if(MatchMode01Pid(c,"010C")) { AddHistory(c,0); HandlePID0C(); return; }
    if(MatchMode01Pid(c,"010D")) { AddHistory(c,0); HandlePID0D(); return; }
    if(MatchMode01Pid(c,"010F")) { AddHistory(c,0); HandlePID0F(); return; }
    if(MatchMode01Pid(c,"0111")) { AddHistory(c,0); HandlePID11(); return; }
    if(MatchMode01Pid(c,"0133")) { AddHistory(c,0); HandlePID33(); return; }
    if(MatchMode01Pid(c,"015E")) { AddHistory(c,0); HandlePID5E(); return; }

    // Private Mode 01 PIDs used by Torque Custom PID definitions.
    // Bench testing showed this path updates reliably, unlike Mode 22.
    if(MatchMode01Pid(c,"01F0")) { AddHistory(c,0); HandlePrivatePIDF0FuelFlow(); return; }
    if(MatchMode01Pid(c,"01F1")) { AddHistory(c,0); HandlePrivatePIDF1InstantEconomy(); return; }

    // Standard Transmission Actual Gear from CAN 0x121.
    if(MatchMode01Pid(c,"01A4")) { AddHistory(c,0); HandlePIDA4(); return; }

    if(!strncmp(c,"22",2))
    {
        AddHistory(c,0);
        HandleMode22(c);
        return;
    }

    // Unknown Mode 01 / other diagnostic requests are never forwarded
    // to the Kawasaki ECU. This remains a whitelist emulator.
    AddHistory(c,2);
    ReplyNoData();
}

//==================================================
// InitOBD2Emulator()
//
// Resets parser state and the command-history ring buffer.
// lastTorqueCommandMs=0 guarantees BLUE LED stays OFF until a real command
// is received after boot.
//==================================================
void InitOBD2Emulator(void)
{
    cmdLen=0;
    currentCommand[0]=0;
    lastTorqueCommandMs=0;
    ClearHistory();
}

void UpdateOBD2Emulator(void)
{
    while(BluetoothAvailable()>0)
    {
        char c=ReadBluetoothChar();

        if(c=='\r' || c=='\n')
        {
            if(cmdLen==0) continue;

            cmd[cmdLen]=0;

            if(echoEnabled)
            {
                WriteBluetooth(cmd);
                EndLine();
            }

            ProcessCommand(cmd);
            cmdLen=0;
            continue;
        }

        if(c==8 || c==127)
        {
            if(cmdLen>0) cmdLen--;
            continue;
        }

        if(cmdLen<sizeof(cmd)-1)
            cmd[cmdLen++]=c;
    }
}


//==================================================
// TorqueCommandActive()
//
// Returns true only while application-level ELM/OBD traffic is recent.
// This solves an important distinction:
//   HC-06 connected != Torque is actually running.
//
// If Torque stops polling, exits, or the app-to-adapter session stalls,
// the BLUE LED goes out after TORQUE_ACTIVE_TIMEOUT_MS.
//==================================================
bool TorqueCommandActive(void)
{
    if(lastTorqueCommandMs==0) return false;
    return (unsigned long)(millis()-lastTorqueCommandMs)
           <= TORQUE_ACTIVE_TIMEOUT_MS;
}
