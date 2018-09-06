
#include <cmath>

#include "MadgwickAHRS.h"
#include "rangefinder.h"

namespace cp {

  void Rangefinder::init(void)
  {
      alt = 0;
      previousAlt = 0;
      // Apply Zero-height update
      for (uint8_t k = 0; k < ZH_SIZE; ++k) {
          ZH[k] = 0;
      }
  }

  float Rangefinder::altitudeCompensation(float accel[3], float gyro[3], float altitude)
  {
      // Compensate for effect of pitch, roll on rangefinder reading
      float q[4]; 
      MadgwickAHRSupdateIMU(q, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
  
      float euler0 = atan2(2.0f*(q[0]*q[1]+q[2]*q[3]),q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3]);
      float euler1 = asin(2.0f*(q[1]*q[3]-q[0]*q[2]));
  
      float compensatedAltitude = altitude * cos(euler0) * cos(euler1);
      return ZHUpdate(compensatedAltitude);
  }

  float Rangefinder::ZHUpdate(float compensatedAltitude)
  {
      isAtGround = false;
      // first update ZH array with latest estimation
      ZH[ZHIdx] = compensatedAltitude;
      // and move index to next slot
      uint8_t nextIndex = (ZHIdx + 1) % ZH_SIZE;
      ZHIdx = nextIndex;
      // Apply Zero-height update
      for (uint8_t k = 0; k < ZH_SIZE; ++k) {
          if (fabs(ZH[k]) > heightThreshold) return compensatedAltitude;
      }
      isAtGround = true;
      return compensatedAltitude;
  }

  void Rangefinder::update(float accel[3], float gyro[3], float altitude)
  {
      previousAlt = alt;
      alt = Rangefinder::altitudeCompensation(accel, gyro, altitude);
  }

  float Rangefinder::getAltitude(void)
  {
      return alt;
  }

  float Rangefinder::getDeltaAltitude(void)
  {
      return alt - previousAlt;
  }

} // namespace cp