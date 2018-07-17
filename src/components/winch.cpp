#include "winch.h"

// 20 bit, change if the encoder is different
const double Winch::shaftResolution = pow(2.0, 20.0);
// 18 bit, change if the encoder is different
const double Winch::pulleyResolution = pow(2.0, 18.0);
const double Winch::countsToLengthFactor =
  sqrt(pow(M_PI * drumDiameter, 2.0) + pow(drumPitch, 2.0)) /
  (shaftResolution * gearRatio); // from turns to cable meters
const double Winch::pulleyAngleFactorRad = 2.0 * M_PI / Winch::pulleyResolution;
const double Winch::pulleyAngleFactorDeg = 360.0 / Winch::pulleyResolution;
