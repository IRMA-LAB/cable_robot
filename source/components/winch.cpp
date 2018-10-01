#include "winch.h"

// 20 bit, change if the encoder is different
const double Winch::kShaftResolution_ = pow(2.0, 20.0);
// 18 bit, change if the encoder is different
const double Winch::kPulleyResolution_ = pow(2.0, 18.0);
const double Winch::kCountsToLengthFactor_ =
  sqrt(pow(M_PI * kDrumDiameter_, 2.0) + pow(kDrumPitch_, 2.0)) /
  (kShaftResolution_ * kGearRatio_); // from turns to cable meters
const double Winch::kPulleyAngleFactorRad_ = 2.0 * M_PI / Winch::kPulleyResolution_;
const double Winch::kPulleyAngleFactorDeg_ = 360.0 / Winch::kPulleyResolution_;
