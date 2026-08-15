#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <Arduino.h>

void InitBluetooth(void);
void UpdateBluetooth(void);

void SendBluetooth(const char *text);
void SendBluetooth(const String &text);

bool BluetoothConnected(void);

int BluetoothAvailable(void);
char ReadBluetoothChar(void);
void WriteBluetooth(const char *text);
void WriteBluetoothFlash(const __FlashStringHelper *text);
void WriteBluetoothChar(char c);

#endif
