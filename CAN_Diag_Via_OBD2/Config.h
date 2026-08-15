#ifndef CONFIG_H
#define CONFIG_H

// Bluetooth
#define BT_RX_PIN      4
#define BT_TX_PIN      5
#define BT_STATE_PIN   6

// CAN
#define CAN_CS_PIN     10
#define CAN_INT_PIN    2

// LEDs
// A0 was originally called LED_POWER during early development.
// From Ver2.16 it is the BLUE "Torque active" indicator:
//   ON  = valid ELM/OBD commands are arriving from Torque
//   OFF = no Torque command for TORQUE_ACTIVE_TIMEOUT_MS
#define LED_POWER      A0
#define LED_TORQUE     LED_POWER
#define LED_CAN_TX     A1
#define LED_CAN_RX     A2
#define LED_BT_TX      A3
#define LED_BT_RX      A4
#define LED_CAN_ERROR  A5
#define LED_BT_ERROR   7

// Test Switch
#define TEST_SW        8

#endif