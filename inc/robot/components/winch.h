#ifndef CABLE_ROBOT_WINCH_H
#define CABLE_ROBOT_WINCH_H

#include "slaves/goldsolowhistledrive.h"
#include "types.h"

/**
 * @brief The Cable class
 */
class Cable
{
public:
  /**
   * @brief SetHomeLength
   * @param length
   */
  void SetHomeLength(const double length) { home_length_ = length; }
  /**
   * @brief SetHomeLengthTrue
   * @param length
   */
  void SetHomeLengthTrue(const double length) { home_length_true_ = length; }

  /**
   * @brief UpdateCableLen
   * @param delta_length
   */
  void UpdateCableLen(const double delta_length);

private:
  double home_length_ = 0.0;
  double length_ = 0.0;
  double home_length_true_ = 0.0;
};

/**
 * @brief The Winch class
 */
class Winch
{
public:
  /**
   * @brief Winch
   * @param slave_position
   * @param params
   */
  Winch(const uint8_t slave_position, WinchParams* const params);

  /**
   * @brief GetServo
   * @return
   */
  const grabec::GoldSoloWhistleDrive* GetServo() const { return &servo_; }
  /**
   * @brief GetServo
   * @return
   */
  grabec::GoldSoloWhistleDrive* GetServo() { return &servo_; }
  /**
   * @brief GetCable
   * @return
   */
  const Cable* GetCable() const { return &cable_; }

  /**
   * @brief SetServoPosByCableLen
   * @param target_length
   */
  void SetServoPosByCableLen(const double target_length);
  /**
   * @brief SetServoSpeed
   * @param target_speed
   */
  void SetServoSpeed(const int32_t target_speed);
  /**
   * @brief SetServoTorque
   * @param target_torque
   */
  void SetServoTorque(const int16_t target_torque);

  /**
   * @brief UpdateHomeConfig
   * @param cable_len
   * @param cable_len_true
   */
  void UpdateHomeConfig(const double cable_len, const double cable_len_true);
  /**
   * @brief UpdateConfig
   */
  void UpdateConfig();

private:
  WinchParams* params_;
  Cable cable_;
  grabec::GoldSoloWhistleDrive servo_;
  int servo_home_pos_ = 0;
  int servo_start_pos_ = 0;

  inline double CountsToLength(const int counts) const
  {
    return counts * params_->kCountsToLengthFactor;
  }

  inline int LengthToCounts(const double length) const
  {
    return static_cast<int>(length / params_->kCountsToLengthFactor);
  }
};

#endif // CABLE_ROBOT_WINCH_H
