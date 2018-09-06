/*
    altitude.cpp: Altitude estimation via barometer/rangefinder fusion
*/

#include "estimator.h"
#include <cmath>

namespace cp {

  AltitudeEstimator::AltitudeEstimator(float gain)
  {
    this->gain = gain;
  }

  void AltitudeEstimator::init(void)
  {
      estimatedAltitude = 0;
      baro.init();
      range.init();
  }
  
  void AltitudeEstimator::estimate(float accel[3], float gyro[3], float rangeData, float baroData)
  {
    // first update each sensor's estimation
    bool isCalibrating = baro.update(baroData);
    if (isCalibrating) {
      estimatedAltitude = 0;
    }
    range.update(accel, gyro, rangeData);
    // obtain the altitude deltas
    float baroDelta = baro.getDeltaAltitude();
    float rangeDelta = range.getDeltaAltitude();
    // Combine them with a complementary filter
    float alpha = exp(-fabs(baroDelta - rangeDelta)*gain);
    estimatedAltitude += alpha * rangeDelta;
    estimatedAltitude += (1 - alpha) * baroDelta; 
  }
  
  float AltitudeEstimator::getAltitude(void)
  {
     return estimatedAltitude;
  }

} // namespace cp
