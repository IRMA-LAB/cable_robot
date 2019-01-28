#ifndef CONTROLLER_BASIC_H
#define CONTROLLER_BASIC_H

#include "grabcommon.h"
#include "ctrl/controller_base.h"

enum class Sign : int16_t
{
  POS = 1,
  NEG = -1
};

template <typename T> T operator*(const Sign& sign, const T value)
{
  return static_cast<T>(sign) * value;
}

class ControllerSingleDriveNaive : public ControllerBase
{
public:
  ControllerSingleDriveNaive(): ControllerBase() { Clear(); }
  ControllerSingleDriveNaive(const id_t motor_id, const uint32_t period_nsec);

  void SetCableLenTarget(const double target);
  void SetMotorPosTarget(const int32_t target);
  void SetMotorSpeedTarget(const int32_t target);
  void SetMotorTorqueTarget(const int16_t target);

  double GetCableLenTarget() const { return length_target_; }
  int32_t GetMotorPosTarget() const { return pos_target_true_; }
  int32_t GetMotorSpeedTarget() const { return speed_target_true_; }
  int16_t GetMotorTorqueTarget() const { return torque_target_true_; }

  void CableLenIncrement(const bool active, const Sign sign = Sign::POS,
                         const bool micromove = true);
  void ScaleMotorSpeed(const double scale);
  void MotorTorqueIncrement(const bool active, const Sign sign = Sign::POS);

  bool CableLenTargetReached(const double target);
  bool MotorPosTargetReached(const int32_t target);
  bool MotorSpeedTargetReached(const int32_t target);
  bool MotorTorqueTargetReached(const int16_t target);

  vect<ControlAction> CalcCableSetPoint(const grabcdpr::Vars& robot_status) override;

private:
  static constexpr double kAbsDeltaLengthMicroPerSec_ = 0.005; // [m/s]
  static constexpr double kAbsDeltaLengthPerSec_ = 0.02;  // [m/s]
  static constexpr int32_t kAbsMaxSpeed_ = 800000;  // [counts/s]
  static constexpr int16_t kAbsDeltaTorquePerSec_ = 20;  // [nominal points]
  static constexpr int16_t kAbsMaxTorque_ = 600; // [nominal points]

  enum BitPosition
  {
    LENGTH,
    POSITION,
    SPEED,
    TORQUE
  };

  double period_sec_;
  Bitfield8 target_flags_;

  double length_target_;
  int32_t pos_target_true_;
  int32_t speed_target_true_;
  int16_t torque_target_true_;
  double pos_target_;
  double speed_target_;
  double torque_target_;

  bool change_length_target_;
  bool change_speed_target_;
  bool change_torque_target_;

  double abs_delta_length_;
  double delta_length_;
  double abs_delta_speed_;
  double delta_speed_;
  double abs_delta_torque_;
  double delta_torque_;

  void Clear();
};

#endif // CONTROLLER_BASIC_H
