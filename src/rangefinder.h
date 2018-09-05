
#include "MadgwickAHRS.h"

namespace cp {

  class Rangefinder {
  
    private:
  
      float alt = 0;
      float previousAlt = 0;
      float eulerAngles[3];
  
      float altitudeCompensation(float accel[3], float gyro[3], float altitude);
      
    public:
    
      void init(void);
    
      void update(float accel[3], float gyro[3], float altitude);
    
      float getAltitude(void);
    
      float getDeltaAltitude(void);
  
  }; //class Rangefinder

} // namespace cp
