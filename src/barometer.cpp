/* 
   barometer.cpp: Required methods to process the data obtained from the
   barometer

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
      _iters = 0;
    
      _pressureSum = 0;
      _historyIdx = 0;
      _groundAltitude = 0;
      _alt = 0;
      _previousAlt = 0;

      for (uint8_t k = 0; k < HISTORY_SIZE; ++k) {
            _history[k] = 0;
      }
  }

  void Barometer::calibrate(void)
  {
      static float _groundPressure;

      _groundPressure -= _groundPressure / 8;
      _groundPressure += _pressureSum / (HISTORY_SIZE - 1);
      _groundAltitude = millibarsToMeters(_groundPressure / 8);
  }

  bool Barometer::update(float pressure)
  {
      bool calibrating = true;
      // update pressure history
      uint8_t indexplus1 = (_historyIdx + 1) % HISTORY_SIZE;
      _history[_historyIdx] = pressure;
      _pressureSum += _history[_historyIdx];
      _pressureSum -= _history[indexplus1];
      _historyIdx = indexplus1;
      // if required, calibrate baro
      if (_iters < _calibrationIters)
      {
        Barometer::calibrate();
        _iters += 1;
      }
      else {
        calibrating = false;
      }
      // update altitude estimation
      _previousAlt = _alt;
      float alt_tmp = millibarsToMeters(_pressureSum / (HISTORY_SIZE - 1)) - _groundAltitude;
      _alt = _alt * NOISE_LPF + (1 - NOISE_LPF) * alt_tmp;
      return calibrating;
  }

  float Barometer::getAltitude(void)
  {
      return _alt;
  }

  float Barometer::getDeltaAltitude(void)
  {
      return _alt - _previousAlt;
  }

} // namespace cp
