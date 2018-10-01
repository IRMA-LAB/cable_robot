#ifndef TYPES_H
#define TYPES_H

#include <cmath>

struct Pulley
{
  int home_counts = 0;
  double home_angle = 0.0;
  double angle = 0.0;
};

/**
 * @brief The WinchParams struct
 */
struct WinchParams
{
  static constexpr double kDrumPitch = 0.007;  // [m]
  static constexpr double kDrumDiameter = 0.1; // [m]
  static constexpr double kGearRatio = 5.0;
  static const double kShaftResolution;
  static const double kPulleyResolution;
  static const double kCountsToLengthFactor;
  static const double kPulleyAngleFactorRad;
  static const double kPulleyAngleFactorDeg;

  /**
   * @brief FromCountsToLength
   * @param counts
   * @return
   */
  inline double CountsToLength(const int counts) const
  {
    return counts * kCountsToLengthFactor;
  }

  /**
   * @brief FromLengthToCounts
   * @param length
   * @return
   */
  inline int LengthToCounts(const double length) const
  {
    return static_cast<int>(length / kCountsToLengthFactor);
  }

  /**
   * @brief FromCountsToPulleyAngleDeg
   * @param counts
   * @return
   */
  inline double CountsToPulleyAngleDeg(const int counts) const
  {
    return counts * kPulleyAngleFactorDeg;
  }

  /**
   * @brief FromCountsToPulleyAngleRad
   * @param counts
   * @return
   */
  inline double CountsToPulleyAngleRad(const int counts) const
  {
    return counts * kPulleyAngleFactorRad;
  }
};

// 20 bit, change if the encoder is different
const double WinchParams::kShaftResolution = pow(2.0, 20.0);
// 18 bit, change if the encoder is different
const double WinchParams::kPulleyResolution = pow(2.0, 18.0);
const double WinchParams::kCountsToLengthFactor =
  sqrt(pow(M_PI * WinchParams::kDrumDiameter, 2.0) + pow(WinchParams::kDrumPitch, 2.0)) /
  (WinchParams::kShaftResolution * WinchParams::kGearRatio); // from turns to cable meters
const double WinchParams::kPulleyAngleFactorRad =
  2.0 * M_PI / WinchParams::kPulleyResolution;
const double WinchParams::kPulleyAngleFactorDeg = 360.0 / WinchParams::kPulleyResolution;

#endif // TYPES_H
