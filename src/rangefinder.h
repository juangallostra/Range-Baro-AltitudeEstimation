/* 
   rangefinder.h: Rangefinder abstraction with the required methods for using
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
 
#include "quaternion.h"
#include <Arduino.h>

namespace cp {

  class Rangefinder {
  
    private:
  
      float _alt = 0;
      float _previousAlt = 0;
      
      // Zero Height update
      static const uint8_t ZH_SIZE = 10;
      float _ZH[ZH_SIZE];
      uint8_t _ZHIdx = 0;
      float _heightThreshold = 0.04f;
      
      // Global constants for 6 DoF quaternion filter
      const float GYRO_MEAS_ERROR = M_PI * (40.0f / 180.0f);
      const float GYRO_MEAS_DRIFT = M_PI * (0.0f  / 180.0f);
      const float BETA = sqrtf(3.0f / 4.0f) * GYRO_MEAS_ERROR; 
      const float ZETA = sqrt(3.0f / 4.0f) * GYRO_MEAS_DRIFT;

      uint32_t _time;
      
      cp::MadgwickQuaternionFilter6DOF _quaternionFilter = cp::MadgwickQuaternionFilter6DOF(BETA, ZETA);
  
      float altitudeCompensation(float accel[3], float gyro[3], float altitude);
      
      float altitudeCompensation(float q[4], float altitude);
      
      float altitudeCompensation(float altitude, float euler[3]);
      
      float ZHUpdate(float compensatedAltitude);
      
    public:
    
      bool isAtGround;
    
      void init(void);
    
      void update(float accel[3], float gyro[3], float altitude);
      
      void update(float quat[4], float altitude);
      
      void update(float altitude,float euler[3]);
    
      float getAltitude(void);
    
      float getDeltaAltitude(void);
  
  }; //class Rangefinder

} // namespace cp
