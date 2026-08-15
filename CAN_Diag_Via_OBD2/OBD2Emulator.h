#ifndef OBD2_EMULATOR_H
#define OBD2_EMULATOR_H

#include <Arduino.h>

void InitOBD2Emulator(void);
void UpdateOBD2Emulator(void);

// True while valid Torque/ELM commands have arrived recently.
// Used for the BLUE LED.  Bluetooth pairing alone does not make this true.
bool TorqueCommandActive(void);

#endif
