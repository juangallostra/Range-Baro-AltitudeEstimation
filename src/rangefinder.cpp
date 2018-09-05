
#include <cmath>

#include "MadgwickAHRS.h"
#include "rangefinder.h"

namespace cp {

  void Rangefinder::init(void)
  {
      alt = 0;
      previousAlt = 0;
  }

  float Rangefinder::altitudeCompensation(float accel[3], float gyro[3], float altitude)
  {
      // Compensate for effect of pitch, roll on rangefinder reading
      float q[4]; 
      MadgwickAHRSupdateIMU(q, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
  
      float euler0 = atan2(2.0f*(q[0]*q[1]+q[2]*q[3]),q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3]);
      float euler1 = asin(2.0f*(q[1]*q[3]-q[0]*q[2]));
  
      return  altitude * cos(euler0) * cos(euler1);
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