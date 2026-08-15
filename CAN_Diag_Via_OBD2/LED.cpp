#include "Config.h"
#include <Arduino.h>

/*
  Ver2.16 LED meaning
  -------------------
  A0 BLUE : Torque communication active.
            It is NOT merely Bluetooth paired status.  It turns on only
            while ELM/OBD commands are actually arriving from Torque.
  A1      : CAN TX pulse
  A2      : CAN RX pulse
  A3      : Bluetooth TX pulse
  A4      : Bluetooth RX pulse
  A5 RED  : CAN/KWP diagnostic-session error
  D7 RED  : Bluetooth radio link error

  Early versions used A0 as a generic power/status LED.  Keeping the pin
  name LED_POWER as an alias avoids needless hardware/config churn, while
  SetTorqueActive() gives the LED its final practical meaning.
*/

// LED pulse time used during normal loop operation
static const unsigned long LED_PULSE_MS = 20;

// CAN RX/TX LED timing state
static unsigned long g_CANRxLED_OnTime = 0;
static unsigned long g_CANTxLED_OnTime = 0;
static bool g_CANRxLED_Active = false;
static bool g_CANTxLED_Active = false;

// Bluetooth RX/TX LED timing state
static unsigned long g_BTRxLED_OnTime = 0;
static unsigned long g_BTTxLED_OnTime = 0;
static bool g_BTRxLED_Active = false;
static bool g_BTTxLED_Active = false;


//==================================================
// LED initialization / startup self-test
//==================================================
void InitLED()
{
    pinMode(LED_POWER, OUTPUT);
    pinMode(LED_CAN_TX, OUTPUT);
    pinMode(LED_CAN_RX, OUTPUT);
    pinMode(LED_BT_TX, OUTPUT);
    pinMode(LED_BT_RX, OUTPUT);
    pinMode(LED_CAN_ERROR, OUTPUT);
    pinMode(LED_BT_ERROR, OUTPUT);

    digitalWrite(LED_POWER, LOW);
    digitalWrite(LED_CAN_TX, LOW);
    digitalWrite(LED_CAN_RX, LOW);
    digitalWrite(LED_BT_TX, LOW);
    digitalWrite(LED_BT_RX, LOW);
    digitalWrite(LED_CAN_ERROR, LOW);
    digitalWrite(LED_BT_ERROR, LOW);

    // Startup LED self-test
    // delay() is intentional here because normal processing has not started.
    digitalWrite(LED_POWER, HIGH);
    delay(100);

    digitalWrite(LED_CAN_TX, HIGH);
    delay(100);

    digitalWrite(LED_CAN_RX, HIGH);
    delay(100);

    digitalWrite(LED_BT_TX, HIGH);
    delay(100);

    digitalWrite(LED_BT_RX, HIGH);
    delay(100);

    digitalWrite(LED_CAN_ERROR, HIGH);
    delay(100);

    digitalWrite(LED_BT_ERROR, HIGH);

    // All LEDs remain ON for 0.5 s
    delay(500);

    // All LEDs OFF
    digitalWrite(LED_POWER, LOW);
    digitalWrite(LED_CAN_TX, LOW);
    digitalWrite(LED_CAN_RX, LOW);
    digitalWrite(LED_BT_TX, LOW);
    digitalWrite(LED_BT_RX, LOW);
    digitalWrite(LED_CAN_ERROR, LOW);
    digitalWrite(LED_BT_ERROR, LOW);

    delay(100);

    // Normal state: Power LED ON
    digitalWrite(LED_POWER, LOW);

    g_CANRxLED_Active = false;
    g_CANTxLED_Active = false;
    g_BTRxLED_Active = false;
    g_BTTxLED_Active = false;
}


//==================================================
// Non-blocking LED timer processing
// Call repeatedly from loop()
//==================================================
void UpdateLED()
{
    unsigned long now = millis();

    if (g_CANRxLED_Active &&
        (unsigned long)(now - g_CANRxLED_OnTime) >= LED_PULSE_MS)
    {
        digitalWrite(LED_CAN_RX, LOW);
        g_CANRxLED_Active = false;
    }

    if (g_CANTxLED_Active &&
        (unsigned long)(now - g_CANTxLED_OnTime) >= LED_PULSE_MS)
    {
        digitalWrite(LED_CAN_TX, LOW);
        g_CANTxLED_Active = false;
    }

    if (g_BTRxLED_Active &&
        (unsigned long)(now - g_BTRxLED_OnTime) >= LED_PULSE_MS)
    {
        digitalWrite(LED_BT_RX, LOW);
        g_BTRxLED_Active = false;
    }

    if (g_BTTxLED_Active &&
        (unsigned long)(now - g_BTTxLED_OnTime) >= LED_PULSE_MS)
    {
        digitalWrite(LED_BT_TX, LOW);
        g_BTTxLED_Active = false;
    }
}


//==================================================
// CAN RX LED pulse start
//==================================================
void BlinkCANRx()
{
    digitalWrite(LED_CAN_RX, HIGH);
    g_CANRxLED_OnTime = millis();
    g_CANRxLED_Active = true;
}


//==================================================
// CAN TX LED pulse start
//==================================================
void BlinkCANTx()
{
    digitalWrite(LED_CAN_TX, HIGH);
    g_CANTxLED_OnTime = millis();
    g_CANTxLED_Active = true;
}


//==================================================
// Bluetooth RX LED pulse start
//==================================================
void BlinkBTRx()
{
    digitalWrite(LED_BT_RX, HIGH);
    g_BTRxLED_OnTime = millis();
    g_BTRxLED_Active = true;
}


//==================================================
// Bluetooth TX LED pulse start
//==================================================
void BlinkBTTx()
{
    digitalWrite(LED_BT_TX, HIGH);
    g_BTTxLED_OnTime = millis();
    g_BTTxLED_Active = true;
}


//==================================================
// CAN error LED control
//==================================================
void SetCANError(bool error)
{
    digitalWrite(LED_CAN_ERROR, error ? HIGH : LOW);
}


void SetBTError(bool error)
{
    digitalWrite(LED_BT_ERROR, error ? HIGH : LOW);
}


//==================================================
// Torque communication activity LED
//
// Called by the main loop using TorqueCommandActive() from the ELM layer.
// This deliberately represents application-level Torque traffic rather
// than the HC-06 STATE pin: Bluetooth can remain connected while Torque
// itself is stopped or no longer polling the adapter.
//==================================================
void SetTorqueActive(bool active)
{
    digitalWrite(LED_TORQUE, active ? HIGH : LOW);
}
