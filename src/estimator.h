/*
    altitude.h: Altitude estimation via barometer/rangefinder fusion
*/

# pragma once

#include "barometer.h"
#include "rangefinder.h"

#include <Arduino.h>

namespace cp {

  class AltitudeEstimator {

    private:
    
      float _estimatedAltitude;
      
      float _gain;
        
    public:
    
      Barometer baro;
      Rangefinder range;
    
      AltitudeEstimator(float gain);
    
      void init(void);
      
      void estimate(float accel[3], float gyro[3], float rangeData, float baroData);
      
      float getAltitude(void);

  }; // class ComplementaryAltitude

} // namespace cp
