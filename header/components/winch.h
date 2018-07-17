#ifndef WINCH_H
#define WINCH_H

#include <cmath>

class Winch
{
public:
  Winch() {}

  constexpr static double drumPitch = 0.007;  // in m
  constexpr static double drumDiameter = 0.1; // in m
  static const double shaftResolution;
  static const double pulleyResolution;
  constexpr static double gearRatio = 5.0;
  static const double countsToLengthFactor;
  static const double pulleyAngleFactorRad;
  static const double pulleyAngleFactorDeg;

  inline double FromCountsToLength(int counts) { return counts * countsToLengthFactor; }
  inline int FromLengthToCounts(double length)
  {
    return static_cast<int>(length / countsToLengthFactor);
  }
  inline double FromCountsToPulleyAngleDeg(int counts)
  {
    return counts * pulleyAngleFactorDeg;
  }
  inline double FromCountsToPulleyAngleRad(int counts)
  {
    return counts * pulleyAngleFactorRad;
  }
};

#endif // WINCH_H
