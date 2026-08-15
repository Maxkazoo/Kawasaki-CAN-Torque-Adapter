#include "Config.h"
#include "Bluetooth.h"
#include "LED.h"
#include <Arduino.h>
#include <SoftwareSerial.h>

/*
  CRITICAL RX OWNERSHIP RULE
  --------------------------
  Only OBD2Emulator.cpp may consume BT.read().

  A previous implementation also read BT.available()/BT.read() inside
  UpdateBluetooth().  That split Torque commands between two readers and
  produced fragments such as "ATL" + "0" and "AE0" instead of "ATE0".
  Torque could not finish ECU initialization.

  Therefore UpdateBluetooth() intentionally does nothing to RX data.
  BluetoothConnected() only reads the HC-06/JDY STATE pin.
*/

SoftwareSerial BT(BT_RX_PIN, BT_TX_PIN);

// InitBluetooth(): starts SoftwareSerial at the fixed 9600-bps module rate; sends no unsolicited text.
void InitBluetooth(void)
{
    pinMode(BT_STATE_PIN, INPUT);
    BT.begin(9600);
}

// BluetoothConnected(): reports physical HC-06/JDY link state from the STATE pin.
bool BluetoothConnected(void)
{
    return digitalRead(BT_STATE_PIN);
}

// SendBluetooth(char*): legacy line-oriented helper; avoid unsolicited use during Torque sessions.
void SendBluetooth(const char *text)
{
    BT.println(text);
    BlinkBTTx();
}

// SendBluetooth(String): legacy line-oriented helper; retained for compatibility/debug use.
void SendBluetooth(const String &text)
{
    BT.println(text);
    BlinkBTTx();
}

// UpdateBluetooth(): intentionally does not read RX; OBD2Emulator owns the receive stream.
void UpdateBluetooth(void)
{
    // OBD2Emulator is the only Bluetooth RX consumer.
}

// BluetoothAvailable(): lets the ELM parser check pending SoftwareSerial bytes.
int BluetoothAvailable(void)
{
    return BT.available();
}

// ReadBluetoothChar(): the single approved BT RX read path; also pulses BT-RX LED.
char ReadBluetoothChar(void)
{
    if(!BT.available()) return 0;
    char c=(char)BT.read();
    BlinkBTRx();
    return c;
}

// WriteBluetooth(): writes response text without adding CR/LF; ELM layer controls framing.
void WriteBluetooth(const char *text)
{
    BT.print(text);
    BlinkBTTx();
}

// WriteBluetoothFlash(): streams an F() string directly from program flash.
// This keeps fixed diagnostic labels out of scarce ATmega328P SRAM.
void WriteBluetoothFlash(const __FlashStringHelper *text)
{
    BT.print(text);
    BlinkBTTx();
}

// WriteBluetoothChar(): writes one response character, including the ELM '>' prompt.
void WriteBluetoothChar(char c)
{
    BT.write(c);
    BlinkBTTx();
}
