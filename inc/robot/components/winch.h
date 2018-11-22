#ifndef CABLE_ROBOT_WINCH_H
#define CABLE_ROBOT_WINCH_H

#include "slaves/goldsolowhistledrive.h"
#include "utils/types.h"

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
   * @brief GetHomeLength
   * @return
   */
  double GetHomeLength() const { return home_length_; }
  /**
   * @brief GetLength
   * @return
   */
  double GetLength() const { return length_; }
  /**
   * @brief GetLength
   * @param delta_length
   * @return
   */
  double GetLength(const double delta_length);

  /**
   * @brief UpdateCableLen
   * @param delta_length
   */
  void UpdateCableLen(const double delta_length);

private:
  double home_length_ = 0.0;
  double length_ = 0.0;
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
  Winch(const uint8_t slave_position, const WinchParams& params);

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
   * @brief GetServoStatus
   * @return
   */
  WinchStatus GetStatus();
  /**
   * @brief GetServoHomePos
   * @return
   */
  int32_t GetServoHomePos() const { return servo_home_pos_; }

  /**
   * @brief SetServoPos
   * @param target_pos
   */
  void SetServoPos(const int32_t target_pos) { servo_.ChangePosition(target_pos); }
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
   * @brief SetServoOpMode
   * @param op_mode
   */
  void SetServoOpMode(const int8_t op_mode);

  /**
   * @brief UpdateHomeConfig
   * @param cable_len
   * @param cable_len_true
   */
  void UpdateHomeConfig(const double cable_len);
  /**
   * @brief UpdateConfig
   */
  void UpdateConfig() { UpdateConfig(servo_.GetPosition()); }
  /**
   * @brief UpdateConfig
   * @param servo_pos
   */
  void UpdateConfig(const int32_t servo_pos);

private:
  WinchParams params_;
  Cable cable_;
  grabec::GoldSoloWhistleDrive servo_;
  int32_t servo_home_pos_ = 0;

  inline double CountsToLength(const int counts) const
  {
    return counts * params_.kCountsToLengthFactor;
  }

  inline int LengthToCounts(const double length) const
  {
    return static_cast<int>(length / params_.kCountsToLengthFactor);
  }
};

#endif // CABLE_ROBOT_WINCH_H
