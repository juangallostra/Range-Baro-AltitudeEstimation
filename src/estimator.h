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
    
      Barometer baro;
      Rangefinder range;
    
    public:
    
      void init(void);

  }; // class ComplementaryAltitude

} // namespace cp
