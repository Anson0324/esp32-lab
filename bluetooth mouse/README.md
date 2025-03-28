# Motion-Controlled Bluetooth Mouse

A Bluetooth HID mouse built with ESP32-C3 and an ICM42670P accelerometer. The device maps physical board tilt to on-screen mouse movement, scaling speed based on tilt intensity.

## Features
- Accelerometer-based tilt detection
- Bluetooth HID profile for mouse movement
- Dynamic cursor speed scaling
- BLE-compatible with desktops and mobile devices

## Tech Stack
- ESP32-C3
- C / FreeRTOS
- I2C (ICM42670P)
- Bluetooth HID (BLE)
