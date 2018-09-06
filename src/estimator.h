/* 
   estimator.h: Declarations of the estimator class and its methods

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
