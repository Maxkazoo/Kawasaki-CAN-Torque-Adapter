#ifndef MAINTENANCE_NOTES_H
#define MAINTENANCE_NOTES_H

/*
===============================================================================
CAN_Diag_Via_OBD2 - MAINTENANCE / DEVELOPMENT MEMORY
===============================================================================

PURPOSE
-------
2011 Kawasaki Concours14 / GTR1400 diagnostic data is acquired from the
motorcycle's 500-kbit/s CAN bus and exposed to Android Torque as an
ELM327-like OBD2 adapter.

The final architecture is intentionally asynchronous:

    motorcycle broadcast CAN + internally scheduled Kawasaki KWP
                           |
                           v
                      RAM cache
                           |
                           v
                 ELM327/Torque responses

Torque never directly controls which Kawasaki diagnostic command is sent.

-------------------------------------------------------------------------------
HARDWARE
-------------------------------------------------------------------------------
Arduino Nano ATmega328P, 16 MHz
MCP2515 module, 8 MHz crystal
  CS D10, INT D2, MOSI D11, MISO D12, SCK D13
Bluetooth serial module at 9600 bps
  Arduino RX D4, TX D5, STATE D6

LED:
  A0 BLUE = Torque commands actively arriving
  A1 CAN TX
  A2 CAN RX
  A3 BT TX
  A4 BT RX
  A5 RED CAN/KWP error
  D7 RED Bluetooth STATE error

-------------------------------------------------------------------------------
CAN / KWP FACTS ESTABLISHED DURING DEVELOPMENT
-------------------------------------------------------------------------------
Bus speed: 500 kbit/s.

Useful broadcast frames currently cached:
  0x100 RPM
  0x110 vehicle speed
  0x120 coolant
  0x121 gear / neutral
  0x32F fuel-flow candidate

Kawasaki diagnostic communication established experimentally:
  request ID  0x7E4
  useful FI response ID 0x746

Session start:
  02 10 80 00 00 00 00 00
positive:
  02 50 80 ...

KWP local identifier read:
  02 21 PID 00 00 00 00 00
positive observed:
  04 61 PID DATA_H DATA_L ...

Used identifiers:
  04 throttle sensor
  05 intake-pipe pressure
  07 intake-air temperature
  08 atmospheric pressure

A fixed 100-ms slot scheduler was adopted after an earlier priority
scheduler starved PID07/PID08.  The slot table guarantees the slow values
always get time.

-------------------------------------------------------------------------------
MCP2515 FILTER PROBLEM THAT COST TIME
-------------------------------------------------------------------------------
The mcp_can library reported "Setting Mask Successful!" and
"Setting Filter Successful!", but direct register dumps showed masks and
filters at zero and RXM receive mode incorrect.

The working fix is direct MCP2515 register programming in CANBus.cpp.

DO NOT replace ConfigureFilters() with library init_Mask()/init_Filt()
without verifying the actual MCP2515 registers afterward.

Current filter strategy:
  RXB0 mask 0x700/filter 0x100 => hardware passes standard 0x100..0x1FF;
       software keeps only required 0x100/110/120/121.
  RXB1 exact mask 0x7FF => 0x746 and 0x32F.

This arrangement is necessary because MCP2515 has only six acceptance
filters.

-------------------------------------------------------------------------------
BLUETOOTH / TORQUE PROBLEM THAT COST TIME
-------------------------------------------------------------------------------
At one stage BOTH UpdateBluetooth() and UpdateOBD2Emulator() read the
SoftwareSerial receive buffer.  Commands were divided between readers:
  ATE0 became fragments,
  ATL0 became fragments,
  ATSP6 became fragments.
Torque then failed ECU initialization.

FINAL RULE:
  OBD2Emulator.cpp is the only consumer of BT.read().
  UpdateBluetooth() must never consume RX.

Also, do not print startup banners or debug messages spontaneously over the
Bluetooth ELM stream.  Torque expects request/response traffic.

Torque may append expected-response count:
  010C1 must be accepted as 010C.
This is why MatchMode01Pid() accepts one optional trailing hex digit.

-------------------------------------------------------------------------------
SUPPORTED-PID BITMAP LESSON
-------------------------------------------------------------------------------
Torque uses 0100/0120/0140 capability bitmaps to decide which gauges to poll.
An incorrect bitmap caused only coolant and IAT to update even though other
gauges were placed on screen.

A bench dummy test using:
  41 00 FF FF FF FF
proved that Torque then began polling MAP, RPM, speed, throttle, BARO, etc.

Therefore supported-PID bitmap bytes must always match implemented handlers.

-------------------------------------------------------------------------------
DATA CONVERSION / CALIBRATION
-------------------------------------------------------------------------------
Conversions are deliberately isolated in DataConversion.cpp/.h.

Current speed working assumption:
  0x110 raw = 0.1 km/h

Fuel flow is still a calibration item:
  provisional 0x32F field = Data0:Data1 big-endian
  FuelFlow [L/h] = raw * KDS_FUEL_FLOW_LPH_PER_COUNT

Instantaneous economy:
  km/L = speed[km/h] / fuelFlow[L/h]

Calibration against motorcycle meter:
  NewK = OldK * (TorqueEconomy / MeterEconomy)

Keep this coefficient in DataConversion.h rather than scattering it through
ELM or CAN code.

-------------------------------------------------------------------------------
TORQUE ACTIVE BLUE LED
-------------------------------------------------------------------------------
HC-06 STATE only proves the radio link exists.  It does NOT prove Torque is
running.

Ver2.16 therefore records the time of the latest syntactically valid
AT / Mode01 / Mode22 command.  BLUE A0 stays on while commands have arrived
within the last 2 seconds and turns off after Torque stops polling.

-------------------------------------------------------------------------------
Ver2.18 custom PIDs
-------------------
22F331 = Kawasaki Fuel Flow
         A:B in 0.01 L/h
         Torque: ((A*256)+B)/100

22F330 = Kawasaki Instant Fuel Economy
         A:B in 0.1 km/L
         Torque: ((A*256)+B)/10

22F321 = Kawasaki Gear Position
         A = 0 Neutral, 1..6 gear
         Torque: A

FUNCTION MAP
-------------------------------------------------------------------------------
CANBus.cpp
  SPIBegin/SPIEnd       - direct MCP2515 SPI transaction boundaries
  ReadReg/WriteReg      - direct register access
  BitModify             - masked MCP2515 register update
  SetMode               - enter CONFIG/NORMAL mode and verify CANSTAT
  EncodeSID             - 11-bit CAN ID -> SIDH/SIDL
  WriteFilter/WriteMask - direct acceptance register programming
  ConfigureFilters      - installs the proven RXB0/RXB1 filter strategy
  SaveFrame             - latest-frame RAM cache update
  InitCAN               - 500 kbps MCP2515 init + filters
  Send8                 - diagnostic transmit to 0x7E4
  SendKWPStartDiagnosticSession - 10 80
  SendKWPReadLocalIdentifier    - 21 PID
  UpdateCANBus          - drain frames, cache broadcast, decode 0x746
  TakeKWPSessionPositive - consume latched 50 80
  TakeKWPReadPositive   - consume matching 61 PID raw value
  GetCachedCANFrame     - accessor for ELM layer
  LastCANReceiveMs      - diagnostic timestamp

Bluetooth.cpp
  InitBluetooth         - SoftwareSerial 9600, no unsolicited output
  BluetoothConnected    - HC-06 STATE
  UpdateBluetooth       - deliberately no RX read
  BluetoothAvailable    - parser RX count
  ReadBluetoothChar     - ONLY actual RX consumer path
  WriteBluetooth*       - ELM response output

OBD2Emulator.cpp
  AddHistory/DumpHistory/ClearHistory - ring-buffer diagnostic history
  Prompt/Reply*         - ELM framing
  Normalize             - remove spaces and uppercase command
  ReplyBytes            - OBD response formatting
  Fresh                 - reject stale cached values
  Handle01xx            - supported PID bitmaps
  HandlePIDxx           - standard Mode01 outputs
  HandleMode22          - custom Kawasaki raw/calculated outputs
  HandleAT              - ELM AT compatibility
  MatchMode01Pid        - supports 010C and 010C1 forms
  LooksLikeTorqueCommand- filters module status strings for BLUE LED
  ProcessCommand        - whitelist command dispatcher
  InitOBD2Emulator      - parser reset
  UpdateOBD2Emulator    - receive-byte state machine
  TorqueCommandActive   - 2-second application-traffic watchdog

DataConversion.cpp
  All raw Kawasaki -> engineering/OBD conversions.
  Keep calibration constants here.

Main .ino
  Maintains KWP session/scheduler, Bluetooth error indication, Torque BLUE
  LED, and services ELM requests without blocking CAN reception.

===============================================================================
*/



/*
-------------------------------------------------------------------------------
VER2.19 STANDARD GEAR PID A4
-------------------------------------------------------------------------------
Arduino Forum user TriB reported successful use of standard OBD-II PID A4
(Transmission Actual Gear) with this encoding:

  responseByte = (gear << 4) | 0x01

  Neutral = 0x01
  1st     = 0x11
  2nd     = 0x21
  3rd     = 0x31
  4th     = 0x41
  5th     = 0x51
  6th     = 0x61

Because A4 is in the A1..C0 supported-PID block, the capability chain must
continue through PID60, PID80 and PIDA0.  If any continuation bit is omitted,
Torque may never ask 01A0 and therefore never discover 01A4.

Custom 22F321 remains enabled until Torque's standard A4 display is proven
on the real motorcycle.

*/
#endif
