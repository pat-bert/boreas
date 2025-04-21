Overview
********

Boreas estimates the orientation of the PCB using the on-board IMU of the 
Arduino Nano 33 BLE Sense board.

Currently, there are three Zephyr threads used:

#. main: Toggles LED to indicate that the system is running
#. environment: Samples the on-board pressure, temperature and humidity sensors at 1 Hz
#. imu: Samples the on-board accelerometer and gyrometer at 119 Hz, 
   estimates the gravity vector based on the accelerometer,
   predicts the roll and pitch angles by integration the gyro rates,
   corrects the prediction using the estimate with a Kalman filter.

Building and Running
********************

Build and flash Boreas using the built-in tasks (Build All, Flash All) in the VS-Code Extension.
The Segger J-Link Real-time terminal (RTT) output is enabled by default.
When a terminal session is started the code will periodically output the sensor readings 
and the estimated orientation.

Flashing
********

The buttons of the nRF connect SDK are bound to custom tasks that call nrfutil to flash via Segger J-Link.
The binary directory of nrfutil has to be on the system path.

Device Firmware Update (DFU)
****************************

Boreas uses MCUBoot to update the firmware. Currently, DFU over USB via serial recovery is implemented.
To put the board into recovery mode short P1.02 to GND and press the reset button.
Signed firmware binaries can be uploaded using AuTerm.

A private key can be generated using the following command:

.. code-block:: console
   openssl ecparam -name prime256v1 -genkey -noout -out priv.pem
