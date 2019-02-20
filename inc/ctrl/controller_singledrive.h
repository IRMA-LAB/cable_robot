#ifndef CONTROLLER_SINGLEDRIVE_H
#define CONTROLLER_SINGLEDRIVE_H

#include "lib/grab_common/pid/pid.h"

#include "ctrl/controller_base.h"
#include "grabcommon.h"

enum class Sign : int16_t
{
  POS = 1,
  NEG = -1
};

template <typename T> T operator*(const Sign& sign, const T value)
{
  return static_cast<T>(sign) * value;
}

class ControllerSingleDrive: public ControllerBase
{
 public:
  ControllerSingleDrive(const uint32_t period_nsec);
  ControllerSingleDrive(const id_t motor_id, const uint32_t period_nsec);

  void SetCableLenTarget(const double target);
  void SetMotorPosTarget(const int32_t target);
  void SetMotorSpeedTarget(const int32_t target);
  void SetMotorTorqueTarget(const int16_t target);

  void SetMotorTorqueSsErrTol(const int16_t tol) { torque_ss_err_tol_ = tol; }

  double GetCableLenTarget() const { return length_target_; }
  int32_t GetMotorPosTarget() const { return pos_target_true_; }
  int32_t GetMotorSpeedTarget() const { return speed_target_true_; }
  int16_t GetMotorTorqueTarget() const { return torque_target_true_; }

  void CableLenIncrement(const bool active, const Sign sign = Sign::POS,
                         const bool micromove = true);
  void ScaleMotorSpeed(const double scale);
  void MotorTorqueIncrement(const bool active, const Sign sign = Sign::POS);

  bool TargetReached() const override { return modes_.empty() ? false : on_target_; }
  bool CableLenTargetReached(const double current_value);
  bool MotorPosTargetReached(const int32_t current_value);
  bool MotorSpeedTargetReached(const int32_t current_value);
  bool MotorTorqueTargetReached() const { return on_target_; }

  vect<ControlAction>
  CalcCtrlActions(const grabcdpr::Vars& robot_status,
                  const vect<ActuatorStatus>& actuators_status) override;

 private:
  static constexpr double kAbsDeltaLengthMicroPerSec_ = 0.005;  // [m/s]
  static constexpr double kAbsDeltaLengthPerSec_      = 0.02;   // [m/s]
  static constexpr int32_t kDefaultPosSsErrTol_       = 5;      // [counts]
  static constexpr int32_t kAbsMaxSpeed_              = 800000; // [counts/s]
  static constexpr int16_t kAbsDeltaTorquePerSec_     = 20;     // [nominal points]
  static constexpr int16_t kAbsMaxTorque_             = 600;    // [nominal points]
  static constexpr int16_t kDefaultTorqueSsErrTol_    = 5;      // [nominal points]

  enum BitPosition
  {
    LENGTH,
    POSITION,
    SPEED,
    TORQUE
  };

  double period_sec_;
  Bitfield8 target_flags_;
  bool on_target_;

  double length_target_;
  int32_t pos_target_true_;
  int32_t speed_target_true_;
  int16_t torque_target_true_;
  double pos_target_;
  double speed_target_;
  double torque_target_;

  int32_t pos_ss_err_tol_;
  int16_t torque_ss_err_tol_;

  bool change_length_target_;
  bool change_speed_target_;
  bool change_torque_target_;

  double abs_delta_length_;
  double delta_length_;
  double abs_delta_speed_;
  double delta_speed_;
  double abs_delta_torque_;
  double delta_torque_;

  PID pos_pid_;
  ParamsPID pos_pid_params_ = {0.0263, 0., 15.847, 0., 50.0, -50.0};
  PID torque_pid_;
  ParamsPID torque_pid_params_ = {0.0263, 0., 15.847, 0., kAbsMaxTorque_, -kAbsMaxTorque_};

  int32_t CalcMotorPos(const vect<ActuatorStatus>& actuators_status);
  int16_t CalcMotorTorque(const vect<ActuatorStatus>& actuators_status);

  void Clear();
};

#endif // CONTROLLER_SINGLEDRIVE_H
