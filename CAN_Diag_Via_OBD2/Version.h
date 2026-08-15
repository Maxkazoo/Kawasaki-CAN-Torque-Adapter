#ifndef VERSION_H
#define VERSION_H

#define VERSION_TEXT "CAN_Diag_Via_OBD2 Ver2.20a"

/*
Ver2.20a - Fuel-flow coefficient calibration

Observed on real motorcycle:
  Motorcycle instantaneous fuel economy : 15.0 km/L
  Torque K-FE                           :  5.5 km/L

Ver2.20 coefficient:
  0.01 L/h/count

Because:
  K-FE = vehicle speed / fuel flow
  fuel flow = raw * coefficient

the corrected coefficient is:

  0.01 * (5.5 / 15.0)
  = 0.003666666...
  -> 0.003667 L/h/count

Mode 01 practical mapping retained:
  01F0 = K-Fuel
  01F1 = K-FE
  01A4 = Transmission Actual Gear

Legacy Mode 22 handlers remain available for diagnostics.
*/
#endif
