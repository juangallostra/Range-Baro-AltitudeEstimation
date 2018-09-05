/*
    altitude.cpp: Altitude estimation via barometer/rangefinder fusion
*/

#include "estimator.h"

namespace cp {

  void AltitudeEstimator::init(void)
  {
      baro.init();
      range.init();
  }

} // namespace cp
