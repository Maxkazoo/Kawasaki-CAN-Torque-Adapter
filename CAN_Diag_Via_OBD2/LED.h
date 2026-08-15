#ifndef LED_H
#define LED_H

void InitLED();
void UpdateLED();

void BlinkCANRx();
void BlinkCANTx();

void BlinkBTRx();
void BlinkBTTx();

void SetCANError(bool error);
void SetBTError(bool error);
void SetTorqueActive(bool active);

#endif
