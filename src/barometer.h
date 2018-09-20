/* 
   barometer.h: Barometer abstraction with the required methods for using
   its data in the estimation

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
#include "filters.h"

namespace cp {

  class Barometer {

      private:
          
          uint8_t _iters;
          uint8_t _calibrationIters             = 100;

          const float NOISE_LPF                 = 0.5f;

          float   _alt;

          float   _groundAltitude;
          float   _previousAlt;
          float   _pressure;
          
          // Use a history of 10 readings for the low pass filter
          cp::LowPassFilter _lpf = cp::LowPassFilter(10);

          // Pressure in millibars to altitude in meters
          float millibarsToMeters(float mbar);

        public:

          void init(void);

          void calibrate(void);

          bool update(float pressure);

          float getAltitude(void);
        
          float getDeltaAltitude(void);

  }; // class Barometer

} // namespace cp
