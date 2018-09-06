/*
    altitude.cpp: Altitude estimation via barometer/rangefinder fusion
*/

#include "estimator.h"
#include <cmath>

namespace cp {

  AltitudeEstimator::AltitudeEstimator(float gain)
  {
    _gain = gain;
  }

  void AltitudeEstimator::init(void)
  {
      _estimatedAltitude = 0;
      baro.init();
      range.init();
  }
  
  void AltitudeEstimator::estimate(float accel[3], float gyro[3], float rangeData, float baroData)
  {
    // first update each sensor's estimation
    bool isCalibrating = baro.update(baroData);
    if (isCalibrating) {
      _estimatedAltitude = 0;
    }
    range.update(accel, gyro, rangeData);
    // obtain the altitude deltas
    float baroDelta = baro.getDeltaAltitude();
    float rangeDelta = range.getDeltaAltitude();
    // Combine them with a complementary filter
    if (range.isAtGround)
    {
      _estimatedAltitude = 0;
      return;
    }
    
    float alpha = exp(-fabs(baroDelta - rangeDelta)*_gain);
    _estimatedAltitude += alpha * rangeDelta;
    _estimatedAltitude += (1 - alpha) * baroDelta; 
  }
  
  float AltitudeEstimator::getAltitude(void)
  {
     return _estimatedAltitude;
  }

} // namespace cp
