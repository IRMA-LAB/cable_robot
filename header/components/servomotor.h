#ifndef SERVOMOTOR_H
#define SERVOMOTOR_H

#include "goldsolowhistledrive.h"
#include "winch.h"
#include "cmath"

class ServoMotor
{
private:
  GoldSoloWhistleDrive* drive_ = NULL;
  Winch winch_;

  constexpr static uint8_t kSet_ = 1;
  constexpr static uint8_t kReset_ = 0;
  constexpr static short kMaxTorque_ = 400;

  double stop_counts_ = 0.0;
  double start_counts_ = 0.0;
  double end_time_ = 1.0;
  int home_motor_counts_ = 0;
  int home_pulley_counts_ = 0;
  int start_motor_counts_ = 0;
  int start_pulley_counts_ = 0;
  uint8_t poly7_flag_ = 0;
  double poly7_coeff_[4] = {35.0, -84.0, 70.0, -20.0};

public:
  enum TypeOfMotions
  {
    POS_MOVE_UP,
    POS_MOVE_DOWN,
    POS_MICRO_MOVE_UP,
    POS_MICRO_MOVE_DOWN,
    TORQUE_UP,
    TORQUE_DOWN,
    SPEED_UP,
    SPEED_DOWN
  } type_of_motion_;
  uint8_t motion_status_ = kReset_;
  GoldSoloWhistleDrive::InputPdos* servo_motor_input_pdos_ = NULL;
  GoldSoloWhistleDrive::GoldSoloWhistleDriveState servo_motor_state_;
  GoldSoloWhistleDrive::GoldSoloWhistleOperationState servo_motor_operation_state_;

  ServoMotor() {}
  int update_idx_ = 0;
  int up_counter_ = 0;
  int down_counter_ = 10000;
  double cable_home_len_ = 0.0;
  double cable_len_ = 0.0;
  double cable_effective_home_len_ = 0.0;
  double pulley_home_angle_ = 0.0;
  double pulley_angle_ = 0.0;
  double motor_torque_ = 0.0;

  void AssignDrive(GoldSoloWhistleDrive* new_drive);
  void UpdateState();
  void LoopFunction();
  void Enable();
  void Disable();
  void FaultReset();
  void ChangeOperationMode(int target_op_mode);
  void SetCommand(int cmd, int state);

  inline int HasEnableRequestBeenProcessed()
  {
    if (drive_->state_ == GoldSoloWhistleDrive::operationEnabled &&
        servo_motor_state_ == GoldSoloWhistleDrive::switchOn)
    {
      return 1;
    }
    else if (drive_->state_ == GoldSoloWhistleDrive::switchOnDisabled &&
             servo_motor_state_ == GoldSoloWhistleDrive::operationEnabled)
    {
      return 0;
    }
    else
      return 2;
  }
  inline int FaultPresent()
  {
    if (drive_->state_ == GoldSoloWhistleDrive::fault &&
        servo_motor_state_ != GoldSoloWhistleDrive::fault)
    {
      return 1;
    }
    else
      return 0;
  }
  inline int HasClearFaultRequestBeenProcessed()
  {
    if (drive_->state_ == GoldSoloWhistleDrive::switchOnDisabled &&
        servo_motor_state_ == GoldSoloWhistleDrive::fault)
    {
      return 1;
    }
    else
      return 0;
  }
  inline int HasOperationModeChangeRequestBeenProcessed()
  {
    if (drive_->operationState != servo_motor_operation_state_ &&
        drive_->operationState != GoldSoloWhistleDrive::nullOperation)
    {
      servo_motor_operation_state_ = drive_->operationState;
      return 1;
    }
    else
      return 0;
  }

  void SetTargetDefaults();
  void SetStartingWinchParameter();
  void SetHomeWinchParameters(double cable_len, double pulley_angle, double cable_len_true);
  void SetMaxTorque();
  void SetTorque(short torque);
  void SetPosition(double position);
  void SetSpeed(int velocity);
  void SetPoly7IncrementalParameters(double end_length, double end_t);
  void SetPoly7GoHomeParameters(double end_t);
  void SetPoly7GoStartParameters(double end_t);
  void MovePoly7Incremental(double t);
};

#endif // SERVOMOTOR_H
