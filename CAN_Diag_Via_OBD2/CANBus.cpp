#include "Config.h"
#include "CANBus.h"
#include "LED.h"

#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>

/*
  MCP2515 FILTER HISTORY / IMPORTANT LESSON
  =========================================

  Earlier versions used the mcp_can library's init_Mask()/init_Filt()
  functions and then found, by direct register dump, that RXM/RXF registers
  were still zero or RXBCTRL.RXM was set to "receive any message".
  In other words the intended filters were NOT actually active in our setup.

  The reliable solution used here is:
    1. CAN0.begin() only for controller/baud initialization.
    2. Enter MCP2515 CONFIG mode explicitly.
    3. Write RXM/RXF registers directly over SPI.
    4. Set RXB0CTRL/RXB1CTRL RXM bits explicitly.
    5. Return to NORMAL mode.

  Filter allocation:
    RXB0:
      mask 0x700, filter 0x100
      -> hardware admits standard 0x100..0x1FF.
      Software then keeps only 0x100,0x110,0x120,0x121.

    RXB1:
      mask 0x7FF exact match
      -> 0x746 KWP response and 0x32F fuel-flow candidate.

  Why not filter every broadcast ID exactly?
  MCP2515 has only six acceptance filters.  Grouping 0x1xx frames in RXB0
  preserves scarce exact-match filters for diagnostic/fuel frames.

  CAN bus:
    500 kbit/s, MCP2515 module has 8 MHz crystal.

  Diagnostic pair established experimentally:
    TX 0x7E4
    RX 0x746

  Do not casually replace this direct-register configuration with the old
  library filter calls without re-running a register dump.
*/

MCP_CAN CAN0(CAN_CS_PIN);

static const unsigned long KWP_TX_ID = 0x7E4;
static const unsigned long KWP_RX_ID = 0x746;

// Broadcast IDs retained in RAM.
static const unsigned long ID_RPM   = 0x100;
static const unsigned long ID_SPEED = 0x110;
static const unsigned long ID_WATER = 0x120;
static const unsigned long ID_GEAR  = 0x121;
static const unsigned long ID_FUEL  = 0x32F;

static CachedCANFrame g_rpm   = {false,0,{0},0};
static CachedCANFrame g_speed = {false,0,{0},0};
static CachedCANFrame g_water = {false,0,{0},0};
static CachedCANFrame g_gear  = {false,0,{0},0};
static CachedCANFrame g_fuel  = {false,0,{0},0};

static unsigned long g_lastCANReceiveMs = 0;

// Latest useful KWP response from 0x746.
static bool g_sessionPositive = false;
static bool g_pidPositive = false;
static byte g_pid = 0;
static uint16_t g_pidRaw = 0;

// MCP2515 registers
static const byte REG_RXF0SIDH = 0x00;
static const byte REG_RXF1SIDH = 0x04;
static const byte REG_RXF2SIDH = 0x08;
static const byte REG_RXF3SIDH = 0x10;
static const byte REG_RXF4SIDH = 0x14;
static const byte REG_RXF5SIDH = 0x18;
static const byte REG_CANSTAT  = 0x0E;
static const byte REG_CANCTRL  = 0x0F;
static const byte REG_RXM0SIDH = 0x20;
static const byte REG_RXM1SIDH = 0x24;
static const byte REG_CANINTF  = 0x2C;
static const byte REG_EFLG     = 0x2D;
static const byte REG_RXB0CTRL = 0x60;
static const byte REG_RXB1CTRL = 0x70;

static const byte CMD_READ   = 0x03;
static const byte CMD_WRITE  = 0x02;
static const byte CMD_BITMOD = 0x05;

static const byte DIAG_MODE_MASK   = 0xE0;
static const byte DIAG_MODE_CONFIG = 0x80;
static const byte DIAG_MODE_NORMAL = 0x00;

// SPIBegin(): starts one direct MCP2515 register transaction and asserts CS.
static void SPIBegin()
{
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    digitalWrite(CAN_CS_PIN, LOW);
}
// SPIEnd(): deasserts MCP2515 CS and closes the SPI transaction.
static void SPIEnd()
{
    digitalWrite(CAN_CS_PIN, HIGH);
    SPI.endTransaction();
}
// ReadReg(): reads one MCP2515 register; used to verify mode/filter state.
static byte ReadReg(byte a)
{
    SPIBegin();
    SPI.transfer(CMD_READ);
    SPI.transfer(a);
    byte v=SPI.transfer(0);
    SPIEnd();
    return v;
}
// WriteReg(): writes one MCP2515 register directly over SPI.
static void WriteReg(byte a, byte v)
{
    SPIBegin();
    SPI.transfer(CMD_WRITE);
    SPI.transfer(a);
    SPI.transfer(v);
    SPIEnd();
}
// BitModify(): MCP2515 atomic masked register update command.
static void BitModify(byte a, byte m, byte d)
{
    SPIBegin();
    SPI.transfer(CMD_BITMOD);
    SPI.transfer(a);
    SPI.transfer(m);
    SPI.transfer(d);
    SPIEnd();
}
// SetMode(): requests MCP2515 mode and waits briefly for CANSTAT confirmation.
static bool SetMode(byte mode)
{
    BitModify(REG_CANCTRL,DIAG_MODE_MASK,mode);
    unsigned long t=millis();
    while((ReadReg(REG_CANSTAT)&DIAG_MODE_MASK)!=mode)
    {
        if(millis()-t>20UL) return false;
    }
    return true;
}
// EncodeSID(): converts an 11-bit standard CAN ID to SIDH/SIDL register form.
static void EncodeSID(unsigned int id, byte &h, byte &l)
{
    h=(byte)(id>>3);
    l=(byte)((id&7)<<5);
}
// WriteFilter(): writes one standard-ID acceptance filter register block.
static void WriteFilter(byte base, unsigned int id)
{
    byte h,l;
    EncodeSID(id,h,l);
    WriteReg(base+0,h);
    WriteReg(base+1,l);
    WriteReg(base+2,0);
    WriteReg(base+3,0);
}
// WriteMask(): writes one acceptance mask and sets MIDE to reject extended-ID matches.
static void WriteMask(byte base, unsigned int mask)
{
    byte h,l;
    EncodeSID(mask,h,l);
    // MIDE=1 so extended frames do not match.
    l |= 0x08;
    WriteReg(base+0,h);
    WriteReg(base+1,l);
    WriteReg(base+2,0);
    WriteReg(base+3,0);
}

// ConfigureFilters(): programs the proven direct-register MCP2515 filter layout described above.
static bool ConfigureFilters()
{
    if(!SetMode(DIAG_MODE_CONFIG)) return false;

    // RXB0:
    // Mask 0x700 + filter 0x100 receives standard IDs 0x100..0x1FF.
    // Software keeps only 0x100, 0x110, 0x120 and 0x121.
    WriteMask(REG_RXM0SIDH,0x700);
    WriteFilter(REG_RXF0SIDH,0x100);
    WriteFilter(REG_RXF1SIDH,0x100);

    // RXB1:
    // Exact matches. We need only FI diagnostic RX 0x746 and fuel 0x32F.
    WriteMask(REG_RXM1SIDH,0x7FF);
    WriteFilter(REG_RXF2SIDH,0x746);
    WriteFilter(REG_RXF3SIDH,0x32F);
    WriteFilter(REG_RXF4SIDH,0x746);
    WriteFilter(REG_RXF5SIDH,0x32F);

    BitModify(REG_RXB0CTRL,0x60,0x00);
    BitModify(REG_RXB1CTRL,0x60,0x00);

    BitModify(REG_EFLG,0xC0,0x00);
    BitModify(REG_CANINTF,0x20,0x00);

    return SetMode(DIAG_MODE_NORMAL);
}

// SaveFrame(): replaces a RAM cache entry with the newest CAN frame and timestamp.
static void SaveFrame(CachedCANFrame &dst, byte len, const byte *buf)
{
    dst.valid=true;
    dst.len=(len>8)?8:len;
    for(byte i=0;i<dst.len;i++) dst.data[i]=buf[i];
    for(byte i=dst.len;i<8;i++) dst.data[i]=0;
    dst.updatedMs=millis();
}

// InitCAN(): initializes 500 kbps CAN on the 8 MHz MCP2515 and installs acceptance filters.
bool InitCAN(void)
{
    pinMode(CAN_INT_PIN,INPUT);
    pinMode(CAN_CS_PIN,OUTPUT);
    digitalWrite(CAN_CS_PIN,HIGH);

    if(CAN_OK!=CAN0.begin(MCP_ANY,CAN_500KBPS,MCP_8MHZ))
        return false;

    return ConfigureFilters();
}

// Send8(): sends one 8-byte diagnostic CAN frame to 0x7E4 and pulses CAN-TX LED.
static bool Send8(const byte *data)
{
    byte rc=CAN0.sendMsgBuf(KWP_TX_ID,0,8,(byte*)data);
    if(rc!=CAN_OK) return false;
    BlinkCANTx();
    return true;
}

// SendKWPStartDiagnosticSession(): sends KWP StartDiagnosticSession 10 80.
bool SendKWPStartDiagnosticSession(void)
{
    const byte d[8]={0x02,0x10,0x80,0,0,0,0,0};
    g_sessionPositive=false;
    return Send8(d);
}

// SendKWPReadLocalIdentifier(): sends KWP 21 xx ReadDataByLocalIdentifier.
bool SendKWPReadLocalIdentifier(byte pid)
{
    byte d[8]={0x02,0x21,pid,0,0,0,0,0};
    g_pidPositive=false;
    return Send8(d);
}

// UpdateCANBus(): drains MCP2515 RX buffers, updates broadcast caches, and decodes 0x746 KWP replies.
void UpdateCANBus(void)
{
    byte processed=0;

    while(CAN0.checkReceive()==CAN_MSGAVAIL && processed<12)
    {
        processed++;

        unsigned long rawId=0;
        byte len=0;
        byte buf[8]={0};

        if(CAN0.readMsgBuf(&rawId,&len,buf)!=CAN_OK)
            break;

        // Standard ID only.
        unsigned long id=rawId&0x7FFUL;
        g_lastCANReceiveMs=millis();
        BlinkCANRx();

        if(id==ID_RPM)       { SaveFrame(g_rpm,len,buf);   continue; }
        if(id==ID_SPEED)     { SaveFrame(g_speed,len,buf); continue; }
        if(id==ID_WATER)     { SaveFrame(g_water,len,buf); continue; }
        if(id==ID_GEAR)      { SaveFrame(g_gear,len,buf);  continue; }
        if(id==ID_FUEL)      { SaveFrame(g_fuel,len,buf);  continue; }

        if(id!=KWP_RX_ID)
            continue;

        // ISO-TP single-frame positive session response:
        // 02 50 80
        if(len>=3 && buf[0]>=0x02 && buf[1]==0x50 && buf[2]==0x80)
        {
            g_sessionPositive=true;
            continue;
        }

        // KWP positive ReadDataByLocalIdentifier:
        // 04 61 PID DATA_H DATA_L  (observed)
        if(len>=5 && buf[1]==0x61)
        {
            g_pid=buf[2];
            g_pidRaw=((uint16_t)buf[3]<<8)|buf[4];
            g_pidPositive=true;
            continue;
        }
    }
}

// TakeKWPSessionPositive(): consumes the latched 50 80 positive session response.
bool TakeKWPSessionPositive(void)
{
    if(!g_sessionPositive) return false;
    g_sessionPositive=false;
    return true;
}

// TakeKWPReadPositive(): returns a matching 61 PID positive response and its 16-bit raw value.
bool TakeKWPReadPositive(byte expectedPid, uint16_t &rawValue)
{
    if(!g_pidPositive) return false;
    if(g_pid!=expectedPid) return false;

    rawValue=g_pidRaw;
    g_pidPositive=false;
    return true;
}

// GetCachedCANFrame(): copies the newest cached broadcast frame requested by higher layers.
bool GetCachedCANFrame(unsigned long id, CachedCANFrame &out)
{
    const CachedCANFrame *src=0;

    if(id==ID_RPM) src=&g_rpm;
    else if(id==ID_SPEED) src=&g_speed;
    else if(id==ID_WATER) src=&g_water;
    else if(id==ID_GEAR) src=&g_gear;
    else if(id==ID_FUEL) src=&g_fuel;
    else return false;

    out=*src;
    return src->valid;
}

// LastCANReceiveMs(): exposes last bus activity time for diagnostics/future watchdog use.
unsigned long LastCANReceiveMs(void)
{
    return g_lastCANReceiveMs;
}
