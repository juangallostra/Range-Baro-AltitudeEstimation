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
  float Barometer::decibarsToMeters(float dbar)
  {
      // see: https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf
      return (1.0f - powf(dbar / 101325.00f, 0.190295f)) * 44330.0f;
  }

  void Barometer::init(void)
  {
      _iters = 0;

      _lpf.init();

      _pressure = 0;
      _groundAltitude = 0;
      _alt = 0;
      _previousAlt = 0;
  }

  void Barometer::calibrate(int pressure)
  {
      static float _groundPressure;
      _groundPressure += (int)((float)pressure / _calibrationIters);
      _groundAltitude = decibarsToMeters(_groundPressure);
  }

  bool Barometer::update(float pressure)
  {
      bool calibrating = true;
      // Round pressure to 2 decimal places and
      // work with it as an int. We assume the rest
      // is noise
      pressure  = (int)(pressure * 100); 
      _pressure = _lpf.update(pressure);
      // if required, calibrate baro
      if (_iters < _calibrationIters)
      {
        Barometer::calibrate(pressure);
        _iters += 1;
      }
      else {
        calibrating = false;
        // update altitude estimation
        _previousAlt = _alt;
        float alt_tmp = decibarsToMeters(_pressure) - _groundAltitude;
        _alt = _alt * NOISE_LPF + (1 - NOISE_LPF) * alt_tmp;
      }
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
