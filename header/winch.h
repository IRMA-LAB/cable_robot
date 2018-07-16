#ifndef WINCH_H
#define WINCH_H

#include <cmath>

class Winch
{
public:
  Winch() {}

  constexpr static double drumPitch = 0.007;  // in m
  constexpr static double drumDiameter = 0.1; // in m
  constexpr static double shaftResolution =
    pow(2.0, 20.0); // 20 bit, change if the encoder is different
  constexpr static double pulleyResolution =
    pow(2.0, 18.0); // 20 bit, change if the encoder is different
  constexpr static double gearRatio = 5.0;
  constexpr static double countsToLengthFactor =
    sqrt(pow(M_PI * drumDiameter, 2.0) + pow(drumPitch, 2.0)) /
    (shaftResolution * gearRatio); // from turns to cable meters
  constexpr static double pulleyAngleFactorRad = 2.0 * M_PI / pulleyResolution;
  constexpr static double pulleyAngleFactorDeg = 360.0 / pulleyResolution;
  inline double FromCountsToLength(int counts)
  {
    return counts * countsToLengthFactor;
  }
  inline int FromLengthToCounts(double length)
  {
    return (int)(length / countsToLengthFactor);
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
