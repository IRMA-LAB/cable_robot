#ifndef CONTROLLER_BASIC_H
#define CONTROLLER_BASIC_H

#include "grabcommon.h"
#include "ctrl/controller_base.h"

enum class Sign : int8_t
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
  explicit ControllerSingleDriveNaive(const uint8_t motor_id);

  void SetCableLenTarget(const double target);
  void SetMotorPosTarget(const int32_t target);
  void SetMotorSpeedTarget(const int32_t target);
  void SetMotorTorqueTarget(const int16_t target);

  double GetCableLenTarget() const { return length_target_; }
  int32_t GetMotorPosTarget() const { return pos_target_; }
  int32_t GetMotorSpeedTarget() const { return speed_target_; }
  int16_t GetMotorTorqueTarget() const { return torque_target_; }

  void CableLenIncrement(const bool active, const Sign sign = Sign::POS,
                         const bool micromove = true);
  void MotorSpeedIncrement(const Sign sign);
  void MotorTorqueIncrement(const Sign sign);

  bool CableLenTargetReached(const double target);
  bool MotorPosTargetReached(const int32_t target);
  bool MotorSpeedTargetReached(const int32_t target);
  bool MotorTorqueTargetReached(const int16_t target);

  vect<ControlAction> CalcCableSetPoint(const grabcdpr::Vars& robot_status) override;

private:
  static constexpr double kDeltaLengthMicro = 0.001;
  static constexpr double kDeltaLength = 0.01;
  static constexpr int32_t kDeltaSpeed = 1;
  static constexpr int16_t kDeltaTorque = 1;

  enum BitPosition
  {
    LENGTH,
    POSITION,
    SPEED,
    TORQUE
  };

  Bitfield8 target_flags_;

  double length_target_;
  int32_t pos_target_;
  int32_t speed_target_;
  int16_t torque_target_;

  bool change_length_target_;
  double delta_length_;

  void Clear();
};

#endif // CONTROLLER_BASIC_H
