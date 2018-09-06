
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
      
      float ZHUpdate(float compensatedAltitude);
      
    public:
    
      bool isAtGround;
    
      void init(void);
    
      void update(float accel[3], float gyro[3], float altitude);
    
      float getAltitude(void);
    
      float getDeltaAltitude(void);
  
  }; //class Rangefinder

} // namespace cp
