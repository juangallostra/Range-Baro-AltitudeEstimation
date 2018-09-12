/* 
   rangefinder.cpp: Required methods to process the data obtained from the
   rangefinder

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
 
#include <cmath>

#include "quaternion.h"
#include "rangefinder.h"

namespace cp {

  void Rangefinder::init(void)
  {
      _alt = 0;
      _previousAlt = 0;
      // Apply Zero-height update
      for (uint8_t k = 0; k < ZH_SIZE; ++k) {
          _ZH[k] = 0;
      }
  }

  float Rangefinder::altitudeCompensation(float accel[3], float gyro[3], float altitude)
  {
      // Compensate for effect of pitch, roll on rangefinder reading
      float q[4];
      // Run the quaternion on the IMU values
      // Set integration time by time elapsed since last filter update
      uint32_t time = micros();
      float deltat = (time - _time) / 1.e6f;
      _time = time;
      
      _quaternionFilter.update(accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], deltat);
      // get quaternion updated values back
      q[0] = _quaternionFilter.q1;
      q[1] = _quaternionFilter.q2;
      q[2] = _quaternionFilter.q3;
      q[3] = _quaternionFilter.q4;
      
      float euler0 = atan2(2.0f*(q[0]*q[1]+q[2]*q[3]),q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3]);
      float euler1 = asin(2.0f*(q[1]*q[3]-q[0]*q[2]));
      
      float compensatedAltitude = altitude * cos(euler0) * cos(euler1);
      
      return ZHUpdate(compensatedAltitude);
  }
  
  float Rangefinder::altitudeCompensation(float q[4], float altitude)
  {    
      float euler0 = atan2(2.0f*(q[0]*q[1]+q[2]*q[3]),q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3]);
      float euler1 = asin(2.0f*(q[1]*q[3]-q[0]*q[2]));
      
      float compensatedAltitude = altitude * cos(euler0) * cos(euler1);
      
      return ZHUpdate(compensatedAltitude);
  }

  float Rangefinder::ZHUpdate(float compensatedAltitude)
  {
      isAtGround = false;
      // first update ZH array with latest estimation
      _ZH[_ZHIdx] = compensatedAltitude;
      // and move index to next slot
      uint8_t nextIndex = (_ZHIdx + 1) % ZH_SIZE;
      _ZHIdx = nextIndex;
      // Apply Zero-height update
      for (uint8_t k = 0; k < ZH_SIZE; ++k) {
          if (fabs(_ZH[k]) > _heightThreshold) return compensatedAltitude;
      }
      isAtGround = true;
      return compensatedAltitude;
  }

  void Rangefinder::update(float accel[3], float gyro[3], float altitude)
  {
      _previousAlt = _alt;
      _alt = Rangefinder::altitudeCompensation(accel, gyro, altitude);
  }
  
  void Rangefinder::update(float quat[4], float altitude)
  {
      _previousAlt = _alt;
      _alt = Rangefinder::altitudeCompensation(quat, altitude);
  }

  float Rangefinder::getAltitude(void)
  {
      return _alt;
  }

  float Rangefinder::getDeltaAltitude(void)
  {
      return _alt - _previousAlt;
  }

} // namespace cp