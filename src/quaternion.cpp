/* 
   quaternion.cpp: Quaternion filter classes and methods

   Copyright (c) 2018 Simon D. Levy

   This file is a partial copy of Hackflight's filters.hpp file.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with EM7180.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <cmath>
#include <math.h>

#include "quaternion.h"

namespace cp {


  QuaternionFilter::QuaternionFilter(void)
  {
      q1 = 1;
      q2 = 0;
      q3 = 0;
      q4 = 0;
  }


  MadgwickQuaternionFilter::MadgwickQuaternionFilter(float beta) : QuaternionFilter::QuaternionFilter()
  {
      _beta = beta;
  }


  MadgwickQuaternionFilter6DOF::MadgwickQuaternionFilter6DOF(float beta, float zeta) :
  MadgwickQuaternionFilter::MadgwickQuaternionFilter(beta) 
  { 
      _zeta = zeta;
  }

  // Adapted from https://github.com/kriswiner/MPU6050/blob/master/quaternionFilter.ino
  void MadgwickQuaternionFilter6DOF::update(float ax, float ay, float az,
                                            float gx, float gy, float gz,
                                            float deltat)
  {
      static float gbiasx, gbiasy, gbiasz; // gyro bias error

      // Auxiliary variables to avoid repeated arithmetic
      float _halfq1 = 0.5f * q1;
      float _halfq2 = 0.5f * q2;
      float _halfq3 = 0.5f * q3;
      float _halfq4 = 0.5f * q4;
      float _2q1 = 2.0f * q1;
      float _2q2 = 2.0f * q2;
      float _2q3 = 2.0f * q3;
      float _2q4 = 2.0f * q4;
      //float _2q1q3 = 2.0f * q1 * q3;
      //float _2q3q4 = 2.0f * q3 * q4;

      // Normalise accelerometer measurement
      float norm = sqrt(ax * ax + ay * ay + az * az);
      if (norm == 0.0f) return; // handle NaN
      norm = 1.0f/norm;
      ax *= norm;
      ay *= norm;
      az *= norm;

      // Compute the objective function and Jacobian
      float f1 = _2q2 * q4 - _2q1 * q3 - ax;
      float f2 = _2q1 * q2 + _2q3 * q4 - ay;
      float f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
      float J_11or24 = _2q3;
      float J_12or23 = _2q4;
      float J_13or22 = _2q1;
      float J_14or21 = _2q2;
      float J_32 = 2.0f * J_14or21;
      float J_33 = 2.0f * J_11or24;
      
      // Compute the gradient (matrix multiplication)
      float hatDot1 = J_14or21 * f2 - J_11or24 * f1;
      float hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
      float hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
      float hatDot4 = J_14or21 * f1 + J_11or24 * f2;
      
      // Normalize the gradient
      norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
      hatDot1 /= norm;
      hatDot2 /= norm;
      hatDot3 /= norm;
      hatDot4 /= norm;
      
      // Compute estimated gyroscope biases
      float gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
      float gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
      float gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;
      
      // Compute and remove gyroscope biases
      gbiasx += gerrx * deltat * _zeta;
      gbiasy += gerry * deltat * _zeta;
      gbiasz += gerrz * deltat * _zeta;
      gx -= gbiasx;
      gy -= gbiasy;
      gz -= gbiasz;
      
      // Compute the quaternion derivative
      float qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
      float qDot2 =  _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
      float qDot3 =  _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
      float qDot4 =  _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;
      
      // Compute then integrate estimated quaternion derivative
      q1 += (qDot1 -(_beta * hatDot1)) * deltat;
      q2 += (qDot2 -(_beta * hatDot2)) * deltat;
      q3 += (qDot3 -(_beta * hatDot3)) * deltat;
      q4 += (qDot4 -(_beta * hatDot4)) * deltat;
      
      // Normalize the quaternion
      norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
      norm = 1.0f/norm;
      q1 *= norm;
      q2 *= norm;
      q3 *= norm;
      q4 *= norm;
  }
  
} // namespace cp
