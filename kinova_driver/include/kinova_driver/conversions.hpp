#ifndef KINOVA_DRIVER__CONVERSIONS_HPP_
#define KINOVA_DRIVER__CONVERSIONS_HPP_

#include <cmath>

namespace kinova_driver
{

/// Finger "turns" (0...6800) to degrees. This is the ROS 1 `finger_conv_ratio_`.
constexpr double kFingerTurnsToDeg = 80.0 / 6800.0;

inline double degToRad(double deg)
{
  return deg * M_PI / 180.0;
}

inline double radToDeg(double rad)
{
  return rad * 180.0 / M_PI;
}

/// Joint velocity as reported by the Kinova API (deg/s) to rad/s. Negative velocities can
/// arrive encoded modulo 360, so values above 180 have 360 subtracted first.
inline double kinovaVelocityToRad(double deg_per_s)
{
  if (deg_per_s > 180.0) {
    deg_per_s -= 360.0;
  }
  return degToRad(deg_per_s);
}

inline double fingerTurnsToRad(double turns)
{
  return degToRad(turns * kFingerTurnsToDeg);
}

inline double fingerRadToTurns(double rad)
{
  return radToDeg(rad) / kFingerTurnsToDeg;
}

/// Returns angle + 2*pi*k for the integer k that puts the result closest to reference.
inline double unwrapNear(double angle, double reference)
{
  return angle + 2.0 * M_PI * std::round((reference - angle) / (2.0 * M_PI));
}

}  // namespace kinova_driver

#endif  // KINOVA_DRIVER__CONVERSIONS_HPP_
