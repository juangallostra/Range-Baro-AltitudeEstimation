
#include "barometer.h"
#include <Arduino.h>

namespace cp {

  // Pressure in millibars to altitude in meters. We assume
  // millibars are the units of the pressure readings from the sensor
  float Barometer::millibarsToMeters(float mbar)
  {
      // see: https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf
      return (1.0f - powf(mbar / 1013.25f, 0.190295f)) * 44330.0f;
  }

  void Barometer::init(void)
  {
      pressureSum = 0;
      historyIdx = 0;
      groundAltitude = 0;
      alt = 0;
      previousAlt = 0;

      for (uint8_t k=0; k<HISTORY_SIZE; ++k) {
            history[k] = 0;
      }
  }

  void Barometer::calibrate(void)
  {
      static float   groundPressure;

      groundPressure -= groundPressure / 8;
      groundPressure += pressureSum / (HISTORY_SIZE - 1);
      groundAltitude = millibarsToMeters(groundPressure/8);
  }

  void Barometer::update(float pressure)
  {
      // update pressure history
      uint8_t indexplus1 = (historyIdx + 1) % HISTORY_SIZE;
      history[historyIdx] = pressure;
      pressureSum += history[historyIdx];
      pressureSum -= history[indexplus1];
      historyIdx = indexplus1;
      // update altitude estimation
      previousAlt = alt;
      float alt_tmp = millibarsToMeters(pressureSum/(HISTORY_SIZE-1)) - groundAltitude;
      alt = alt*NOISE_LPF + (1-NOISE_LPF)*alt_tmp;
  }

  float Barometer::getAltitude(void)
  {
      return alt;
  }

  float Barometer::getDeltaAltitude(void)
  {
      return alt - previousAlt;
  }

} // namespace cp
