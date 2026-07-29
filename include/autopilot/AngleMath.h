#ifndef APPS_AUTOPILOT_INCLUDE_AUTOPILOT_ANGLEMATH_H_
#define APPS_AUTOPILOT_INCLUDE_AUTOPILOT_ANGLEMATH_H_

#include <cmath>

namespace arlcore::autopilot {

//! \brief Wrap an angle (radians) into (-pi, pi]. The autopilot keeps its own angle math so
//! it has no dependency on the SDK's legacy guidance utilities.
inline double wrapPi(double angleRad) {
  double a = std::fmod(angleRad, 2.0 * M_PI);
  if (a > M_PI) {
    a -= 2.0 * M_PI;
  } else if (a <= -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

}  // namespace arlcore::autopilot
#endif  // APPS_AUTOPILOT_INCLUDE_AUTOPILOT_ANGLEMATH_H_
