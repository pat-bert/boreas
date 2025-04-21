# Boreas

## Overview

Boreas estimates the orientation of the PCB using the on-board IMU of the
Arduino Nano 33 BLE Sense board.

Currently, there are three Zephyr threads used:

1. **main**: Toggles LED to indicate that the system is running.
2. **environment**: Samples the on-board pressure, temperature, and humidity sensors at 1 Hz.
3. **imu**:
   - Samples the on-board accelerometer and gyrometer at 119 Hz.
   - Estimates the gravity vector based on the accelerometer.
   - Predicts the roll and pitch angles by integrating the gyro rates.
   - Corrects the prediction using the estimate with a Kalman filter.

---

## Building and Running

Build and flash Boreas using the built-in tasks (**Build All**, **Flash All**) in the VS-Code Extension.  
The **Segger J-Link Real-time Terminal (RTT)** output is enabled by default.  
When a terminal session is started, the code will periodically output the sensor readings and the estimated orientation.

---

## Flashing

The buttons of the **nRF Connect SDK** are bound to custom tasks that call `nrfutil` to flash via **Segger J-Link**.  
The binary directory of `nrfutil` must be included in the system path.

---

## Device Firmware Update (DFU)

Boreas uses **MCUBoot** to update the firmware.  
Currently, **DFU over USB via serial recovery** is implemented.

To put the board into **recovery mode**:

- Short **P1.02** to **GND**.
- Press the **reset** button.

Signed firmware binaries can be uploaded using **AuTerm**.

A private key can be generated using the following command:

```sh
openssl ecparam -name prime256v1 -genkey -noout -out priv.pem
```

## Roadmap

- [x] Set initial pressure in altitude estimation
- [ ] Onboard distance sensor vs vl53l0x
- [ ] Extend driver for lsm9ds1 to read magnetometer
- [ ] Extend Kalman filter to estimate yaw angle using magnetometer
- [ ] Research GPS and motor/propeller modules
- [ ] Send orientation data to phone over Bluetooth
- [ ] Enable firmware update over Bluetooth
