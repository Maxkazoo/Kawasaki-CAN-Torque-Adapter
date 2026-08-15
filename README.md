# Kawasaki-CAN-Torque-Adapter
An adapter using an Arduino Nano to display CAN data from a Kawasaki Concours 14 (2011 model) in the Torque app.

## Compatible Vehicles
- Kawasaki Concours 14
- 2011 Model

## Components
- Arduino Nano
- CAN communication module
- HC-05 Bluetooth module
- 6-pin diagnostic connector

## Key Features
- Receives ECU data via CAN
- Responds to ELM327-compatible commands
- Transmits data to the Torque app via Bluetooth
- Displays metrics such as instantaneous fuel economy

## Wiring
Please refer to the `Documentation` folder for wiring details.

## Usage
1. Upload the sketch to the Arduino Nano
2. Connect to the diagnostic connector
3. Pair the Bluetooth module with your Android device
4. Connect to the adapter from the Torque app

## Precautions
Use this software at your own risk.
Incorrect wiring to the vehicle's diagnostic port may damage the ECU or connected devices.

## Credits
This project is based in part on KDS2Bluetooth by HerrRiebmann.

Original project:
https://github.com/HerrRiebmann/KDS2Bluetooth

The original project and this modified version are distributed under the GNU General Public License v3.0.

## License
This project is licensed under the GNU General Public License v3.0.
