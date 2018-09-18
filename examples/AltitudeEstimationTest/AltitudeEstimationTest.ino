/*
    AltitudeEstimationTest.ino : Arduino sketch to perform altitude estimation using
    the provided library

    Author: Juan Gallostra Acín

    Additional libraries required (under your Arduino/libraries folder):

      https://github.com/simondlevy/VL53L1X
      https://github.com/simondlevy/MS5637
      https://github.com/simondlevy/MPU
      https://github.com/simondlevy/CrossPlatformDataBus


    Copyright (c) 2018 Juan Gallostra

    This file is part of the Arduino Range-Baro-AltitudeEstimation library.

    The Arduino Range-Baro-AltitudeEstimation library is free software:
    you can redistribute it and/or modify it under the terms of the GNU
    General Public License as published by the Free Software Foundation,
    either version 3 of the License, or (at your option) any later version.

    The Arduino Range-Baro-AltitudeEstimation library is distributed in the
    hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
    implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
    See the GNU General Public License for more details.
    <http://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include <Wire.h>
// Assuming the IMU is an MPU9250 and thr baro a MS5637
#include <MPU9250_Passthru.h>
#include <MS5637.h>
#include <VL53L1X.h>

#include "estimator.h"

// helper variables and functions for obtaining IMU data
// Sensor full-scale settings
static const MPUIMU::Ascale_t ASCALE = MPUIMU::AFS_2G;
static const MPUIMU::Gscale_t GSCALE = MPUIMU::GFS_2000DPS;
static const MPU9250::Mscale_t MSCALE = MPU9250::MFS_16BITS;
static const MPU9250::Mmode_t MMODE = MPU9250::M_100Hz;
// SAMPLE_RATE_DIVISOR: (1 + SAMPLE_RATE_DIVISOR) is a simple divisor of the fundamental 1000 kHz rate of the gyro and accel, so
// SAMPLE_RATE_DIVISOR = 0 means 1 kHz sample rate for both accel and gyro, 4 means 200 Hz, etc.
static const uint8_t SAMPLE_RATE_DIVISOR = 0;
// MPU9250 add-on board has interrupt on Butterfly pin 8
static const uint8_t INTERRUPT_PIN = 8;

// Use the MPU9250 in pass-through mode
static MPU9250_Passthru imu(ASCALE, GSCALE, MSCALE, MMODE, SAMPLE_RATE_DIVISOR);
// flag for when new data is received
static bool gotNewData = false;

static void interruptHandler()
{
    gotNewData = true;
}

static void getGyrometerAndAccelerometer(float gyro[3], float accel[3])
{
    if (gotNewData) {

        gotNewData = false;

        if (imu.checkNewAccelGyroData()) {
            float ax, ay, az, gx, gy, gz;
            imu.readAccelerometer(ax, ay, az);
            imu.readGyrometer(gx, gy, gz);
            // Negate to support board orientation
            ax = -ax;
            gz = -gz;
            // Copy gyro values back out in rad/sec
            gyro[0] = gx * M_PI / 180.0f;
            gyro[1] = gy * M_PI / 180.0f;
            gyro[2] = gz * M_PI / 180.0f;
            // and acceleration values
            accel[0] = ax;
            accel[1] = ay;
            accel[2] = az;
        } // if (imu.checkNewAccelGyroData())
    } // if gotNewData
}

// Pressure and temperature oversample rate
static MS5637::Rate_t OSR = MS5637::ADC_8192;
static MS5637 barometer = MS5637(OSR);
// RangeFinder
static VL53L1X distanceSensor;

// Altitude estimator
static cp::AltitudeEstimator altitude = cp::AltitudeEstimator(5.0);

void setup(void)
{
    // Start I^2C
    Wire.begin();
    Wire.setClock(400000); // I2C frequency at 400 kHz
    delay(1000);

    // initialize sensors
    imu.begin();
    barometer.begin();
    if (distanceSensor.begin() == false) {
        while (true) {
            Serial.println("Sensor offline!");
            delay(200);
        }
    }
    // initialize the estimator
    altitude.init();

    // Begin serial comms
    Serial.begin(115200);
    // Set up the interrupt pin, it's set as active high, push-pull
    pinMode(INTERRUPT_PIN, INPUT);
    attachInterrupt(INTERRUPT_PIN, interruptHandler, RISING);

}

void loop(void)
{
      float pressure = 0, temperature = 0;
      barometer.readData(temperature, pressure);
      float rangeHeight = (float)distanceSensor.getDistance() / 1000.0f;
      float accelData[3];
      float gyroData[3];
      getGyrometerAndAccelerometer(gyroData, accelData);
      altitude.estimate(accelData, gyroData, rangeHeight, pressure);
      Serial.print(altitude.range.getAltitude());
      Serial.print(",");
      Serial.print(altitude.baro.getAltitude());
      Serial.print(",");
      Serial.println(altitude.getAltitude());
}
