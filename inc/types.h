#ifndef CABLE_ROBOT_TYPES_H
#define CABLE_ROBOT_TYPES_H

#include <cmath>

struct PulleyParams
{
  static const double kPulleyResolution;
  static const double kPulleyAngleFactorRad;
  static const double kPulleyAngleFactorDeg;
};
// 18 bit, change if the encoder is different
const double PulleyParams::kPulleyResolution = pow(2.0, 18.0);
const double PulleyParams::kPulleyAngleFactorRad =
  2.0 * M_PI / PulleyParams::kPulleyResolution;
const double PulleyParams::kPulleyAngleFactorDeg = 360.0 / PulleyParams::kPulleyResolution;

/**
 * @brief The WinchParams struct
 */
struct WinchParams
{
  static constexpr double kDrumPitch = 0.007;  // [m]
  static constexpr double kDrumDiameter = 0.1; // [m]
  static constexpr double kGearRatio = 5.0;
  static const double kShaftResolution;
  static const double kCountsToLengthFactor;
};
// 20 bit, change if the encoder is different
const double WinchParams::kShaftResolution = pow(2.0, 20.0);
const double WinchParams::kCountsToLengthFactor =
  sqrt(pow(M_PI * WinchParams::kDrumDiameter, 2.0) + pow(WinchParams::kDrumPitch, 2.0)) /
  (WinchParams::kShaftResolution * WinchParams::kGearRatio); // from turns to cable meters

struct ActuatorParams
{
  WinchParams winch;
  PulleyParams pulley;
};

#endif // CABLE_ROBOT_TYPES_H
