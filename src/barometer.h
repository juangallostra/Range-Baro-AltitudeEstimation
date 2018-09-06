
#include <Arduino.h>

namespace cp {

  class Barometer {

      private:
          
          float iters;

          const float NOISE_LPF                 = 0.5f;
          static const uint8_t HISTORY_SIZE     = 10;
          static const uint8_t calibrationIters = 100;

          float   alt;
        
          float   history[HISTORY_SIZE];
          uint8_t historyIdx;
          float   groundAltitude;
          float   previousAlt;
          float   pressureSum;

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
