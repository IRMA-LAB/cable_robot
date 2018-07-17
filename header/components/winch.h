#ifndef WINCH_H
#define WINCH_H

#include <cmath>

class Winch
{
public:
  Winch() {}

  constexpr static double kDrumPitch_ = 0.007;  // [m]
  constexpr static double kDrumDiameter_ = 0.1; // [m]
  static const double kShaftResolution_;
  static const double kPulleyResolution_;
  constexpr static double kGearRatio_ = 5.0;
  static const double kCountsToLengthFactor_;
  static const double kPulleyAngleFactorRad_;
  static const double kPulleyAngleFactorDeg_;

  inline double FromCountsToLength(int counts)
  {
    return counts * kCountsToLengthFactor_;
  }

  inline int FromLengthToCounts(double length)
  {
    return static_cast<int>(length / kCountsToLengthFactor_);
  }

  inline double FromCountsToPulleyAngleDeg(int counts)
  {
    return counts * kPulleyAngleFactorDeg_;
  }

  inline double FromCountsToPulleyAngleRad(int counts)
  {
    return counts * kPulleyAngleFactorRad_;
  }
};

#endif // WINCH_H
