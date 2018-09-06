
#include "MadgwickAHRS.h"
#include <Arduino.h>

namespace cp {

  class Rangefinder {
  
    private:
  
      float alt = 0;
      float previousAlt = 0;
      float eulerAngles[3];
      
      // Zero Height update
      static const uint8_t ZH_SIZE = 10;
      float ZH[ZH_SIZE];
      uint8_t ZHIdx = 0;
      float heightThreshold = 0.04f;
  
      float altitudeCompensation(float accel[3], float gyro[3], float altitude);
      
      float ZHUpdate(float compensatedAltitude);
      
    public:
    
      void init(void);
    
      void update(float accel[3], float gyro[3], float altitude);
    
      float getAltitude(void);
    
      float getDeltaAltitude(void);
  
  }; //class Rangefinder

} // namespace cp
