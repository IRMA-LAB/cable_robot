#include "utils/types.h"

/* Pulley params */
// 18 bit, change if the encoder is different
const double PulleyParams::kPulleyResolution = pow(2.0, 18.0);
const double PulleyParams::kPulleyAngleFactorRad =
  2.0 * M_PI / PulleyParams::kPulleyResolution;
const double PulleyParams::kPulleyAngleFactorDeg = 360.0 / PulleyParams::kPulleyResolution;

/* WinchParams */
// 20 bit, change if the encoder is different
const double WinchParams::kShaftResolution = pow(2.0, 20.0);
const double WinchParams::kCountsToLengthFactor =
  sqrt(pow(M_PI * WinchParams::kDrumDiameter, 2.0) + pow(WinchParams::kDrumPitch, 2.0)) /
  (WinchParams::kShaftResolution * WinchParams::kGearRatio); // from turns to cable meters
