/*
   filters.h: Quaternion and low pass filter classes and methods

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

#pragma once

#include <cmath>
#include <math.h>
#include <Arduino.h>

namespace cp {

    class LowPassFilter {

        private:

            float _history[256];
            uint8_t _historySize;
            uint8_t _historyIdx;
            float _sum;

        public:

            LowPassFilter(uint16_t historySize);

            void init(void);

            float update(float value);

    }; // class LowPassFilter

    class LowPassFilterInt {

        private:

            int _history[256];
            uint8_t _historySize;
            uint8_t _historyIdx;
            int _sum;

        public:

            LowPassFilterInt(uint16_t historySize);

            void init(void);

            int update(int value);

    }; // class LowPassFilter


    class QuaternionFilter {

        public:

            float q1;
            float q2;
            float q3;
            float q4;

        protected:

            QuaternionFilter(void);

    }; // class QuaternionFilter


    class MadgwickQuaternionFilter : public QuaternionFilter {

        protected:

            float _beta;

            MadgwickQuaternionFilter(float beta);

    }; // Class MadgwickQuaternionFilter


    class MadgwickQuaternionFilter6DOF : public MadgwickQuaternionFilter {

        private:

            float _zeta;

        public:

            MadgwickQuaternionFilter6DOF(float beta, float zeta);

            // Adapted by Simon D. Levy from
            // https://github.com/kriswiner/MPU6050/blob/master/quaternionFilter.ino
            void update(float ax, float ay, float az, float gx, float gy, float gz, float deltat);

    }; // class MadgwickQuaternionFilter6DOF

} // namespace cp
