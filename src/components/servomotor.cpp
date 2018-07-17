#include "servomotor.h"

void ServoMotor::AssignDrive(GoldSoloWhistleDrive* new_drive)
{
  drive_ = new_drive;
  servo_motor_input_pdos_ = &drive_->inputPdos;
  servo_motor_state_ = drive_->state_;
  servo_motor_operation_state_ = drive_->operationState;
  home_motor_counts_ = 0;
  home_pulley_counts_ = 0;
  cable_home_len_ = 0.0;
  pulley_home_angle_ = 0.0;
  cable_effective_home_len_ = 0.0;
}

void ServoMotor::UpdateState()
{
  servo_motor_state_ = drive_->state_;
  servo_motor_operation_state_ = drive_->operationState;
  cable_len_ =
    cable_home_len_ +
    winch_.FromCountsToLength(drive_->inputPdos.positionActualValue - home_motor_counts_);
  pulley_angle_ = pulley_home_angle_ +
                  winch_.FromCountsToPulleyAngleRad(
                    drive_->inputPdos.auxiliaryPositionActualValue - home_pulley_counts_);
}

void ServoMotor::LoopFunction()
{
  if (servo_motor_state_ == GoldSoloWhistleDrive::operationEnabled)
  {
    switch (servo_motor_operation_state_)
    {
    case GoldSoloWhistleDrive::cyclicPosition:
    {
      if (motion_status_ == kSet_)
      {
        if (up_counter_ >= 10000)
          up_counter_ = 10000;
        if (down_counter_ <= 0)
          down_counter_ = 0;
        switch (type_of_motion_)
        {
        case POS_MOVE_UP:
        {
          drive_->output_pdos_.target_position +=
            winch_.FromLengthToCounts(static_cast<double>(up_counter_) * 0.0000001);
          break;
        }
        case POS_MOVE_DOWN:
        {
          drive_->output_pdos_.target_position -=
            winch_.FromLengthToCounts(static_cast<double>(up_counter_) * 0.0000001);
          break;
        }
        case POS_MICRO_MOVE_UP:
        {
          drive_->output_pdos_.target_position +=
            winch_.FromLengthToCounts(static_cast<double>(up_counter_) * 0.000000001);
          break;
        }
        case POS_MICRO_MOVE_DOWN:
        {
          drive_->output_pdos_.target_position -=
            winch_.FromLengthToCounts(static_cast<double>(up_counter_) * 0.000000001);
          break;
        }
        default:
          break;
        }
      }
      else
      {
        if (drive_->inputPdos.torqueActualValue > 10)
          drive_->output_pdos_.target_position -= static_cast<int>(ceil(
            abs(static_cast<double>(drive_->inputPdos.velocityActualValue)) * 0.00005));
        if (drive_->inputPdos.torqueActualValue < -10)
          drive_->output_pdos_.target_position += static_cast<int>(ceil(
            abs(static_cast<double>(drive_->inputPdos.velocityActualValue)) * 0.00005));
      }
      break;
    }
    case GoldSoloWhistleDrive::cyclicVelocity:
    {
      if (motion_status_ == kSet_)
      {
        switch (type_of_motion_)
        {
        case SPEED_UP:
        {
          drive_->output_pdos_.target_velocity += 1000;
          break;
        }
        case SPEED_DOWN:
        {
          drive_->output_pdos_.target_velocity -= 1000;
          break;
        }
        default:
          break;
        }
      }
      else
      {
        if (drive_->inputPdos.velocityActualValue > 2000)
          drive_->output_pdos_.target_velocity -= drive_->output_pdos_.target_velocity / 100;
        else if (drive_->inputPdos.velocityActualValue < -2000)
          drive_->output_pdos_.target_velocity += -drive_->output_pdos_.target_velocity / 100;
      }
      break;
    }
    case GoldSoloWhistleDrive::cyclicTorque:
    {
      if (up_counter_ > 30)
      {
        up_counter_ = 0;
        if (motion_status_ == kSet_)
        {
          switch (type_of_motion_)
          {
          case TORQUE_UP:
          {
            drive_->output_pdos_.target_torque += 1;
            break;
          }
          case TORQUE_DOWN:
          {
            drive_->output_pdos_.target_torque -= 1;
            break;
          }
          default:
            break;
          }
        }
        else
        {
          if (drive_->inputPdos.torqueActualValue > 10)
            drive_->output_pdos_.target_torque -= drive_->output_pdos_.target_torque / 10;
          else if (drive_->inputPdos.torqueActualValue < -10)
            drive_->output_pdos_.target_torque += -drive_->output_pdos_.target_torque / 10;
        }
      }
      break;
    }
    default:
      break;
    }
  }
  up_counter_++;
  down_counter_--;
}

void ServoMotor::Enable() { drive_->state_flags_ = GoldSoloWhistleDrive::readyToSwitchOn; }

void ServoMotor::Disable()
{
  drive_->state_flags_ = GoldSoloWhistleDrive::switchOnDisabled;
}

void ServoMotor::FaultReset()
{
  drive_->state_flags_ = GoldSoloWhistleDrive::switchOnDisabled;
}

void ServoMotor::ChangeOperationMode(int target_op_mode)
{
  up_counter_ = 0;
  down_counter_ = 10000;
  drive_->operationStateFlags =
    static_cast<GoldSoloWhistleDrive::GoldSoloWhistleOperationState>(target_op_mode);
}

void ServoMotor::SetCommand(int cmd, int state)
{
  up_counter_ = 0;
  down_counter_ = 10000;
  type_of_motion_ = static_cast<TypeOfMotions>(cmd);
  motion_status_ = static_cast<uint8_t>(state);
}

void ServoMotor::SetTargetDefaults()
{
  drive_->output_pdos_.target_torque = drive_->inputPdos.torqueActualValue;
  drive_->output_pdos_.target_position = drive_->inputPdos.positionActualValue;
  drive_->output_pdos_.target_velocity = drive_->inputPdos.velocityActualValue;
}

void ServoMotor::SetStartingWinchParameter()
{
  home_motor_counts_ = drive_->inputPdos.positionActualValue;
  home_pulley_counts_ = drive_->inputPdos.auxiliaryPositionActualValue;
  cable_home_len_ = 0.0;
  pulley_home_angle_ = 0.0;
  cable_len_ = 0.0;
  pulley_angle_ = 0.0;
}

void ServoMotor::SetHomeWinchParameters(double cable_len, double pulley_angle,
                                        double cable_len_true)
{
  cable_home_len_ = cable_len;
  pulley_home_angle_ = pulley_angle;
  cable_effective_home_len_ = cable_len_true;
  start_motor_counts_ =
    home_motor_counts_ +
    winch_.FromLengthToCounts(cable_effective_home_len_ - cable_home_len_);
}

void ServoMotor::SetMaxTorque()
{
  if (abs(drive_->inputPdos.torqueActualValue) < kMaxTorque_)
    drive_->output_pdos_.target_torque--;
}

void ServoMotor::SetTorque(short torque)
{
  drive_->output_pdos_.target_torque = torque;
}

void ServoMotor::SetPosition(double position)
{
  drive_->output_pdos_.target_position =
    start_motor_counts_ + winch_.FromLengthToCounts(position);
  // thisDrive->outputPdos.TargetPosition = startMotorCounts;
}

void ServoMotor::SetSpeed(int velocity)
{
  drive_->output_pdos_.target_velocity = velocity;
}

void ServoMotor::SetPoly7IncrementalParameters(double end_length, double end_t)
{
  if (end_t >= 0.0)
  {
    start_counts_ = static_cast<double>(drive_->inputPdos.positionActualValue);
    stop_counts_ =
      start_counts_ + static_cast<double>(winch_.FromLengthToCounts(end_length));
    end_time_ = end_t;
    poly7_flag_ = 1;
  }
}

void ServoMotor::SetPoly7GoHomeParameters(double end_t)
{
  if (end_t >= 0.0)
  {
    start_counts_ = static_cast<double>(drive_->inputPdos.positionActualValue);
    stop_counts_ = static_cast<double>(home_motor_counts_);
    end_time_ = end_t;
    poly7_flag_ = 1;
  }
}

void ServoMotor::SetPoly7GoStartParameters(double end_t)
{
  if (end_t >= 0.0)
  {
    start_counts_ = static_cast<double>(drive_->inputPdos.positionActualValue);
    stop_counts_ = static_cast<double>(start_motor_counts_);
    end_time_ = end_t;
    poly7_flag_ = 1;
  }
}

void ServoMotor::MovePoly7Incremental(double t)
{
  double normalizedTime = t / end_time_;
  if (normalizedTime <= 1.0 && poly7_flag_)
  {
    drive_->output_pdos_.target_position = static_cast<int>(
      start_counts_ +
      (stop_counts_ - start_counts_) * (poly7_coeff_[0] * pow(normalizedTime, 4.0) +
                                        poly7_coeff_[1] * pow(normalizedTime, 5.0) +
                                        poly7_coeff_[2] * pow(normalizedTime, 6.0) +
                                        poly7_coeff_[3] * pow(normalizedTime, 7.0)));
  }
  else
  {
    poly7_flag_ = 0;
  }
}
