/*
   AltitudeEstimationTest.ino : Arduino sketch to perform altitude estimation using
   the provided library
 */

#include <Arduino.h>
#include <Wire.h>
// Assuming the IMU is an MPU9250 and thr baro a MS5637
#include <MPU9250_Passthru.h>
#include <MS5637.h>

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
            imu.readAccelerometer(ay, ax, az);
            imu.readGyrometer(gy, gx, gz);
            gx = -gx;
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

// Altitude estimator
static cp::AltitudeEstimator altitude;

void setup(void)
{
    // Start I^2C
    Wire.begin();
    Wire.setClock(400000); // I2C frequency at 400 kHz
    delay(1000);

    // initialize the MPU9250
    imu.begin();
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

}
