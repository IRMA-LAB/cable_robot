#include "cablerobot.h"

void CableRobot::IdleFun() {}

void CableRobot::CalibrationFun() {}

void CableRobot::HomingFun()
{

  for (uint8_t i = 0; i < kNumActuators; i++)
  {
    int flag = servo_motor_[i].HasEnableRequestBeenProcessed();
    if (flag < 2)
    {
      emit SendEnableRequestProcessed(flag, i);
      if (flag == 1)
      {
        robot_globally_enabled_ += 1;
        std::cout << static_cast<int>(robot_globally_enabled_) << std::endl;
      }
      else
      {
      }
    }
    else if (servo_motor_[i].HasClearFaultRequestBeenProcessed())
    {
      servo_motor_[i].Enable();
      emit SendClearFaultRequestProcessed(i);
    }
    else if (servo_motor_[i].FaultPresent())
    {
      servo_motor_[i].FaultReset();
      robot_globally_enabled_ -= 1;
      emit SendFaultPresentAdvice(i);
    }
    servo_motor_[i].UpdateState();
  }
  if (robot_globally_enabled_ < kNumActuators)
  {
    homing_state_ = IDLE_HOMING;
    homing_state_flags_ = NULL_STATE_HOMING;
    internal_delay_counter_ = 0;
    homing_process_finished_ = 0;
    homing_stage_ = 0;
  }
  (this->*homing_state_manager_[homing_state_])();
  (this->*homing_state_machine_[homing_state_])();
}

void CableRobot::Robot66ManualFun() {}

void CableRobot::Robot66DemoFun() {}

void CableRobot::Robot33ActuatorPvtFun()
{
  for (uint8_t i = 0; i < kNumActuators; i++)
  {
    int flag = servo_motor_[i].HasEnableRequestBeenProcessed();
    if (flag < 2)
    {
      emit SendEnableRequestProcessed(flag, i);
      if (flag == 1)
      {
        robot_globally_enabled_ += 1;
        std::cout << static_cast<int>(robot_globally_enabled_) << std::endl;
      }
      else
      {
      }
    }
    else if (servo_motor_[i].HasClearFaultRequestBeenProcessed())
    {
      servo_motor_[i].Enable();
      emit SendClearFaultRequestProcessed(i);
    }
    else if (servo_motor_[i].FaultPresent())
    {
      servo_motor_[i].FaultReset();
      robot_globally_enabled_ -= 1;
      emit SendFaultPresentAdvice(i);
    }
    servo_motor_[i].UpdateState();
  }
  if (robot_globally_enabled_ < kNumActuators)
  {
    actuator_pvt33state_ = IDLE_ACTUATOR_PVT33;
    actuator_pvt33state_flags_ = NULL_STATE_ACTUATOR_PVT33;
    internal_delay_counter_ = 0;
  }
  (this->*actuator_pvt33state_manager_[actuator_pvt33state_])();
  (this->*actuator_pvt33state_machine_[actuator_pvt33state_])();
}

void CableRobot::Robot33AutomaticFun() {}

void CableRobot::Robot33ManualFun() {}

void CableRobot::IdleTransition()
{
  if (state_flags_ == IDLE)
  {
    state_flags_ = NULL_STATE;
    emit SendRobotRequestProcessed(state_);
    std::cout << "Changing robot state to " << state_ << std::endl;
  }
  else if (state_flags_ != NULL_STATE)
  {
    state_ = state_flags_;
    homing_state_ = IDLE_HOMING;
    homing_state_flags_ = IDLE_HOMING;
    actuator_pvt33state_ = IDLE_ACTUATOR_PVT33;
    actuator_pvt33state_flags_ = IDLE_ACTUATOR_PVT33;
    actuator_pvt33process_finished_ = 0;
    internal_delay_counter_ = 0;
    enable_pvt_ = 0;
    pvt_counter_ = 0;
  }
}

void CableRobot::CalibrationTransition()
{
  if (state_flags_ == CALIBRATION)
  {
    state_flags_ = NULL_STATE;
    emit SendRobotRequestProcessed(state_);
    std::cout << "Changing robot state to " << state_ << std::endl;
  }
  else if (state_flags_ != NULL_STATE)
  {
    state_ = state_flags_;
  }
}

void CableRobot::HomingTransition()
{
  if (state_flags_ == HOMING)
  {
    state_flags_ = NULL_STATE;
    internal_delay_counter_ = 0;
    emit SendRobotRequestProcessed(state_);
    std::cout << "Changing robot state to " << state_ << std::endl;
  }
  else if (state_flags_ != NULL_STATE)
  {
    state_ = state_flags_;
    robot_globally_enabled_ = 0;
  }
}

void CableRobot::Robot66ManualTransition()
{
  if (state_flags_ == ROBOT66MANUAL)
  {
    state_flags_ = NULL_STATE;
    emit SendRobotRequestProcessed(state_);
    std::cout << "Changing robot state to " << state_ << std::endl;
  }
  else if (state_flags_ != NULL_STATE)
  {
    state_ = state_flags_;
  }
}

void CableRobot::Robot66DemoTransition()
{
  if (state_flags_ == ROBOT66DEMO)
  {
    state_flags_ = NULL_STATE;
    emit SendRobotRequestProcessed(state_);
    std::cout << "Changing robot state to " << state_ << std::endl;
  }
  else if (state_flags_ != NULL_STATE)
  {
    state_ = state_flags_;
  }
}

void CableRobot::Robot33ActuatorPvtTransition()
{
  if (state_flags_ == ROBOT33ACTUATOR_PVT)
  {
    state_flags_ = NULL_STATE;
    internal_delay_counter_ = 0;
    emit SendRobotRequestProcessed(state_);
    std::cout << "Changing robot state to " << state_ << std::endl;
  }
  else if (state_flags_ != NULL_STATE)
  {
    state_ = state_flags_;
    robot_globally_enabled_ = 0;
  }
}

void CableRobot::Robot33AutomaticTransition()
{
  if (state_flags_ == ROBOT33AUTOMATIC)
  {
    state_flags_ = NULL_STATE;
    emit SendRobotRequestProcessed(state_);
    std::cout << "Changing robot state to " << state_ << std::endl;
  }
  else if (state_flags_ != NULL_STATE)
  {
    state_ = state_flags_;
  }
}

void CableRobot::Robot33ManualTransition()
{
  if (state_flags_ == ROBOT33MANUAL)
  {
    state_flags_ = NULL_STATE;
    emit SendRobotRequestProcessed(state_);
    std::cout << "Changing robot state to " << state_ << std::endl;
  }
  else if (state_flags_ != NULL_STATE)
  {
    state_ = state_flags_;
  }
}

void CableRobot::IdleHomingFun()
{
  if (robot_globally_enabled_ >= kNumActuators && !homing_process_finished_)
  {
    if (internal_delay_counter_ < kHomingDelay)
    {
      internal_delay_counter_++;
    }
    else
    {
      homing_state_flags_ = SWITCH2TENSION_MODE;
      internal_delay_counter_ = 0;
    }
  }
}

void CableRobot::SwitchToTensionMode()
{
  if (robot_globally_enabled_ >= kNumActuators)
  {
    uint8_t flag_completed = 0;
    if (homing_flag_ == 0)
    {
      for (int i = 0; i < kNumActuators; i++)
      {
        if (servo_motor_[i].servo_motor_operation_state_ !=
            GoldSoloWhistleDrive::CYCLIC_TORQUE)
        {
          servo_motor_[i].SetTargetDefaults();
          servo_motor_[i].ChangeOperationMode(GoldSoloWhistleDrive::CYCLIC_TORQUE);
        }
        else
          flag_completed++;
      }
      if (flag_completed == kNumActuators)
      {
        homing_state_flags_ = GO2CENTER;
        emit SendHomingControl(1);
        std::cout << "here" << std::endl;
        internal_delay_counter_ = 0;
        homing_stage_ = 0;
        homing_actuator_ = 0;
      }
    }
    else
    {
      for (int i = 0; i < kNumActuators; i++)
      {
        if (servo_motor_[i].servo_motor_operation_state_ !=
            GoldSoloWhistleDrive::CYCLIC_POSITION)
        {
          servo_motor_[i].SetTargetDefaults();
          servo_motor_[i].ChangeOperationMode(GoldSoloWhistleDrive::CYCLIC_POSITION);
        }
        else
          flag_completed++;
      }
      if (flag_completed == kNumActuators)
      {
        homing_state_flags_ = GOING_HOME;
        for (int i = 0; i < kNumActuators; i += 2)
        {
          servo_motor_[i].SetPoly7GoHomeParameters(kGoHomeTime);
        }
        emit SendHomingControl(0);
        std::cout << "here going home" << std::endl;
        internal_delay_counter_ = 0;
        homing_actuator_ = 0;
      }
    }
  }
}

void CableRobot::GoToCenter()
{
  internal_delay_counter_++;
  if (internal_delay_counter_ > 30)
  {
    internal_delay_counter_ = 0;
    for (int i = 0; i < kNumActuators; i += 2)
      servo_motor_[i].SetMaxTorque();
  }
  if (homing_flag_)
  {
    std::cout << "here2" << std::endl;
    homing_state_flags_ = SWITCH_ACTUATED_CABLE;
    for (int i = 0; i < kNumActuators; i += 2)
      servo_motor_[i].SetStartingWinchParameter();
    internal_delay_counter_ = 0;
    homing_actuator_ -= 2;
    // Send Start Values
    homing_data_[0] = servo_motor_[0].cable_len_;
    homing_data_[1] = servo_motor_[2].cable_len_;
    homing_data_[2] = servo_motor_[4].cable_len_;
    homing_data_[3] = servo_motor_[0].pulley_angle_;
    homing_data_[4] = servo_motor_[2].pulley_angle_;
    homing_data_[5] = servo_motor_[4].pulley_angle_;
    SendMeasurement(homing_data_);
  }
}

void CableRobot::SwitchActuatedCable()
{
  static uint8_t mod_index = 99;

  if (homing_flag_ && internal_delay_counter_ == 0)
  {
    homing_actuator_ += 2;
    internal_delay_counter_++;
    if (homing_actuator_ < kNumActuators)
    {
      servo_motor_[homing_actuator_].SetTargetDefaults();
      for (int i = 0; i < kNumActuators; i += 2)
      {
        if (i != homing_actuator_)
        {
          if (servo_motor_[i].servo_motor_operation_state_ !=
              GoldSoloWhistleDrive::CYCLIC_TORQUE)
          {
            servo_motor_[i].ChangeOperationMode(GoldSoloWhistleDrive::CYCLIC_TORQUE);
            mod_index = static_cast<uint8_t>(i);
          }
        }
        else
        {
          servo_motor_[i].ChangeOperationMode(GoldSoloWhistleDrive::CYCLIC_POSITION);
        }
        servo_motor_[i].SetTargetDefaults();
      }
    }
    else
    {
      homing_state_flags_ = SWITCH2TENSION_MODE;
      homing_actuator_ = 0;
      std::cout << "here" << std::endl;
      internal_delay_counter_ = 0;
    }
  }
  else if (homing_flag_ && homing_actuator_ <= kNumActuators)
  {
    uint8_t flag = 1;
    for (int i = 0; i < kNumActuators; i += 2)
    {
      if (i == homing_actuator_ &&
          servo_motor_[i].servo_motor_operation_state_ ==
            GoldSoloWhistleDrive::CYCLIC_POSITION)
        flag *= 1;
      else if (i != homing_actuator_ &&
               servo_motor_[i].servo_motor_operation_state_ ==
                 GoldSoloWhistleDrive::CYCLIC_TORQUE)
        flag *= 1;
      else
        flag *= 0;
    }
    if (flag)
    {
      homing_state_flags_ = MOVE_AWAY;
      servo_motor_[homing_actuator_].SetPoly7IncrementalParameters(
        -kIncrementalHomingLength, kTransitionTime);
      internal_delay_counter_ = 0;
      meas_stage_ = 0;
    }
  }
}

void CableRobot::MoveAway()
{
  if (homing_flag_)
  {
    internal_delay_counter_++;
    double time_sec = (static_cast<double>(internal_delay_counter_)) / 1000.0;
    if (time_sec <= kTransitionTime)
      servo_motor_[homing_actuator_].MovePoly7Incremental(time_sec);
    else
    {
      homing_state_flags_ = WAIT_FOR_MEAS;
    }
  }
}

void CableRobot::WaitForMeasurement()
{
  std::cout << servo_motor_[0].pulley_angle_ << '\t' << servo_motor_[2].pulley_angle_
            << '\t' << servo_motor_[4].pulley_angle_ << std::endl;
  if (homing_flag_ && meas_flag_)
  {
    // Send Measurement To non RT Thread
    homing_data_[0] = servo_motor_[0].cable_len_;
    homing_data_[1] = servo_motor_[2].cable_len_;
    homing_data_[2] = servo_motor_[4].cable_len_;
    homing_data_[3] = servo_motor_[0].pulley_angle_;
    homing_data_[4] = servo_motor_[2].pulley_angle_;
    homing_data_[5] = servo_motor_[4].pulley_angle_;
    SendMeasurement(homing_data_);
    meas_flag_ = 0;
    meas_stage_++;
    if (meas_stage_ < 1)
    {
      servo_motor_[homing_actuator_].SetPoly7IncrementalParameters(
        -kIncrementalHomingLength, kTransitionTime);
      internal_delay_counter_ = 0;
      homing_state_flags_ = MOVE_AWAY;
    }
    else if (meas_stage_ < 2)
    {
      homing_state_flags_ = MOVE_CENTRAL;
      servo_motor_[homing_actuator_].SetPoly7IncrementalParameters(
        kIncrementalHomingLength, kTransitionTime);
      internal_delay_counter_ = 0;
    }
    else
    {
      homing_state_flags_ = SWITCH_ACTUATED_CABLE;
      internal_delay_counter_ = 0;
    }
  }
}

void CableRobot::MoveCentral()
{
  if (homing_flag_)
  {
    internal_delay_counter_++;
    double time_sec = (static_cast<double>(internal_delay_counter_)) / 1000.0;
    if (time_sec <= kTransitionTime)
      servo_motor_[homing_actuator_].MovePoly7Incremental(time_sec);
    else
    {
      homing_state_flags_ = WAIT_FOR_MEAS;
    }
  }
}

void CableRobot::GoingHome()
{
  internal_delay_counter_++;
  homing_flag_ = 0;
  double time_sec = (static_cast<double>(internal_delay_counter_)) / 1000.0;
  if (time_sec <= kGoHomeTime)
    for (int i = 0; i < kNumActuators; i += 2)
      servo_motor_[i].MovePoly7Incremental(time_sec);
  else
  {
    homing_state_flags_ = IDLE_HOMING;
    homing_process_finished_ = 1;
  }
}

void CableRobot::SwitchToTensionModeTransition()
{
  if (homing_state_flags_ == SWITCH2TENSION_MODE)
  {
    homing_state_flags_ = NULL_STATE_HOMING;
    std::cout << "Changing homing state to " << homing_state_ << std::endl;
  }
  else if (homing_state_flags_ != NULL_STATE_HOMING)
  {
    homing_state_ = homing_state_flags_;
    internal_delay_counter_ = 0;
  }
}

void CableRobot::GoToCenterTransition()
{
  if (homing_state_flags_ == GO2CENTER)
  {
    homing_state_flags_ = NULL_STATE_HOMING;
    std::cout << "Changing homing state to " << homing_state_ << std::endl;
  }
  else if (homing_state_flags_ != NULL_STATE_HOMING)
  {
    homing_state_ = homing_state_flags_;
    internal_delay_counter_ = 0;
  }
}

void CableRobot::MoveAwayTransition()
{
  if (homing_state_flags_ == MOVE_AWAY)
  {
    homing_state_flags_ = NULL_STATE_HOMING;
    std::cout << "Changing homing state to " << homing_state_ << std::endl;
  }
  else if (homing_state_flags_ != NULL_STATE_HOMING)
  {
    homing_state_ = homing_state_flags_;
    internal_delay_counter_ = 0;
  }
}

void CableRobot::WaitForMeasurementTransition()
{
  if (homing_state_flags_ == WAIT_FOR_MEAS)
  {
    homing_state_flags_ = NULL_STATE_HOMING;
    std::cout << "Changing homing state to " << homing_state_ << std::endl;
  }
  else if (homing_state_flags_ != NULL_STATE_HOMING)
  {
    homing_state_ = homing_state_flags_;
    internal_delay_counter_ = 0;
  }
}

void CableRobot::MoveCentralTransition()
{
  if (homing_state_flags_ == MOVE_CENTRAL)
  {
    homing_state_flags_ = NULL_STATE_HOMING;
    std::cout << "Changing homing state to " << homing_state_ << std::endl;
  }
  else if (homing_state_flags_ != NULL_STATE_HOMING)
  {
    homing_state_ = homing_state_flags_;
    internal_delay_counter_ = 0;
  }
}

void CableRobot::SwitchActuatedCableTransition()
{
  if (homing_state_flags_ == SWITCH_ACTUATED_CABLE)
  {
    homing_state_flags_ = NULL_STATE_HOMING;
    std::cout << "Changing homing state to " << homing_state_ << std::endl;
  }
  else if (homing_state_flags_ != NULL_STATE_HOMING)
  {
    homing_state_ = homing_state_flags_;
    internal_delay_counter_ = 0;
  }
}

void CableRobot::IdleFunHomingTransition()
{
  if (homing_state_flags_ == IDLE_HOMING)
  {
    homing_state_flags_ = NULL_STATE_HOMING;
    std::cout << "Changing homing state to " << homing_state_ << std::endl;
  }
  else if (homing_state_flags_ != NULL_STATE_HOMING)
  {
    homing_state_ = homing_state_flags_;
  }
}

void CableRobot::GoingHomeTransition()
{
  if (homing_state_flags_ == GOING_HOME)
  {
    homing_state_flags_ = NULL_STATE_HOMING;
    std::cout << "Changing homing state to " << homing_state_ << std::endl;
  }
  else if (homing_state_flags_ != NULL_STATE_HOMING)
  {
    homing_state_ = homing_state_flags_;
    internal_delay_counter_ = 0;
  }
}

void CableRobot::IdleActuatorPvt33Fun()
{
  if (robot_globally_enabled_ >= kNumActuators && !actuator_pvt33process_finished_)
  {
    if (internal_delay_counter_ < kHomingDelay)
    {
      internal_delay_counter_++;
    }
    else
    {
      actuator_pvt33state_flags_ = SWITCH2POSITION_MODE;
      internal_delay_counter_ = 0;
    }
  }
  if (actuator_pvt33process_finished_)
  {
    // emit
    // SendData(servoMotor[0].pulleyAngle,servoMotor[1].pulleyAngle,servoMotor[2].pulleyAngle);
  }
}

void CableRobot::SwitchToPositionMode()
{
  if (robot_globally_enabled_ >= kNumActuators)
  {
    uint8_t flag_completed = 0;
    for (int i = 0; i < kNumActuators; i++)
    {
      if (servo_motor_[i].servo_motor_operation_state_ !=
          GoldSoloWhistleDrive::CYCLIC_POSITION)
      {
        servo_motor_[i].SetTargetDefaults();
        servo_motor_[i].ChangeOperationMode(GoldSoloWhistleDrive::CYCLIC_POSITION);
      }
      else
        flag_completed++;
    }
    if (flag_completed == kNumActuators)
    {
      enable_pvt_ = 1;
      actuator_pvt33state_flags_ = MOVE_ACTUATOR_PVT33;
      emit SendActuatorPvt33Control(1);
      std::cout << "going to move..." << std::endl;
    }
  }
}

void CableRobot::MoveActuatorPvt33()
{
  if (enable_pvt_ && robot_globally_enabled_ >= kNumActuators)
  {
    if (pvt_counter_ < num_pvt33data_)
    {
      pvt_counter_++;
      emit SendData(servo_motor_[0].pulley_angle_, servo_motor_[2].pulley_angle_,
                    servo_motor_[4].pulley_angle_);
      for (int i = 0; i < kNumActuators; i += 2)
      {
        servo_motor_[i].SetPosition(*ptr2pvt33data_[i / 2]);
        ptr2pvt33data_[i / 2]++;
      }
    }
    else
    {
      emit SendActuatorPvt33Control(2);
      actuator_pvt33state_flags_ = GOING_HOME_PVT33;
      for (int i = 0; i < kNumActuators; i += 2)
        servo_motor_[i].SetPoly7GoStartParameters(kGoHomeTime);
    }
  }
  else
  {
    // emit SendActuatorPvt33Control(0);
  }
}

void CableRobot::GoingHomePvt33()
{
  if (robot_globally_enabled_ >= kNumActuators)
  {
    internal_delay_counter_++;
    double time_sec = (static_cast<double>(internal_delay_counter_)) / 1000.0;
    if (time_sec <= kGoHomeTime)
      for (int i = 0; i < kNumActuators; i += 2)
        servo_motor_[i].MovePoly7Incremental(time_sec);
    else
    {
      actuator_pvt33state_flags_ = IDLE_ACTUATOR_PVT33;
      actuator_pvt33process_finished_ = 1;
      enable_pvt_ = 0;
    }
  }
}

void CableRobot::IdleActuatorPvt33Transition()
{
  if (actuator_pvt33state_flags_ == IDLE_ACTUATOR_PVT33)
  {
    actuator_pvt33state_flags_ = NULL_STATE_ACTUATOR_PVT33;
    std::cout << "Changing actuatorPvt33 state to " << actuator_pvt33state_ << std::endl;
  }
  else if (actuator_pvt33state_flags_ != NULL_STATE_ACTUATOR_PVT33)
  {
    actuator_pvt33state_ = actuator_pvt33state_flags_;
    internal_delay_counter_ = 0;
  }
}

void CableRobot::SwitchToPositionModeTransition()
{
  if (actuator_pvt33state_flags_ == SWITCH2POSITION_MODE)
  {
    actuator_pvt33state_flags_ = NULL_STATE_ACTUATOR_PVT33;
    std::cout << "Changing actuatorPvt33 state to " << actuator_pvt33state_ << std::endl;
  }
  else if (actuator_pvt33state_flags_ != NULL_STATE_ACTUATOR_PVT33)
  {
    actuator_pvt33state_ = actuator_pvt33state_flags_;
    internal_delay_counter_ = 0;
  }
}

void CableRobot::MoveActuatorPvt33Transition()
{
  if (actuator_pvt33state_flags_ == MOVE_ACTUATOR_PVT33)
  {
    actuator_pvt33state_flags_ = NULL_STATE_ACTUATOR_PVT33;
    std::cout << "Changing actuatorPvt33 state to " << actuator_pvt33state_ << std::endl;
  }
  else if (actuator_pvt33state_flags_ != NULL_STATE_ACTUATOR_PVT33)
  {
    actuator_pvt33state_ = actuator_pvt33state_flags_;
    internal_delay_counter_ = 0;
  }
}

void CableRobot::GoingHomePvt33Transition()
{
  if (actuator_pvt33state_flags_ == GOING_HOME_PVT33)
  {
    actuator_pvt33state_flags_ = NULL_STATE_ACTUATOR_PVT33;
    std::cout << "Changing actuatorPvt33 state to " << actuator_pvt33state_ << std::endl;
  }
  else if (actuator_pvt33state_flags_ != NULL_STATE_ACTUATOR_PVT33)
  {
    actuator_pvt33state_ = actuator_pvt33state_flags_;
    internal_delay_counter_ = 0;
  }
}

CableRobot::CableRobot(QObject* parent, GoldSoloWhistleDrive* drives) : QObject(parent)
{
  for (int i = 0; i < kNumActuators; i++)
  {
    servo_motor_[i].AssignDrive(&drives[i]);
  }

  state_ = IDLE;
  state_flags_ = NULL_STATE;
  homing_data_.resize(6);
}

void CableRobot::StandardLoopFunction()
{
  (this->*state_manager_[state_])();
  (this->*state_machine_[state_])();
}

void CableRobot::UserLoopFunction()
{
  (this->*state_manager_[state_])();
  (this->*state_machine_[state_])();
}

void CableRobot::CollectRobotRequest(int state)
{
  state_flags_ = CableRobot::RobotState(state);
}

void CableRobot::CollectEnableRequest(int enable)
{
  robot_globally_enabled_ = 0;
  for (uint8_t i = 0; i < kNumActuators; i++)
  {
    if (enable)
    {
      if (servo_motor_[i].servo_motor_state_ != GoldSoloWhistleDrive::OPERATION_ENABLED)
        servo_motor_[i].Enable();
    }
    else
    {
      servo_motor_[i].Disable();
    }
  }
}

void CableRobot::CollectClearFaultRequest()
{
  for (uint8_t i = 0; i < kNumActuators; i++)
  {
    if (servo_motor_[i].servo_motor_state_ == GoldSoloWhistleDrive::FAULT)
      servo_motor_[i].FaultReset();
  }
}

void CableRobot::CollectHomingProcessControl(int state)
{
  homing_flag_ = static_cast<uint8_t>(state);
}

void CableRobot::CollectMeasurementRequest() { meas_flag_ = 1; }

void CableRobot::CollectHomingData(QVector<double> data)
{
  for (uint8_t i = 0; i < kNumActuators; i += 2)
  {
    servo_motor_[i].SetHomeWinchParameters(data[i / 2], data[i / 2 + kNumActuators / 2],
                                           data[i / 2 + kNumActuators]);
    servo_motor_[i].SetPoly7GoStartParameters(kGoHomeTime);
  }
  homing_state_flags_ = GOING_HOME;
  std::cout << "going to start position" << std::endl;
  internal_delay_counter_ = 0;
}

void CableRobot::CollectDataPointers(int n, double* p1, double* p2, double* p3)
{
  num_pvt33data_ = n;
  ptr2pvt33data_[0] = p1;
  ptr2pvt33data_[1] = p2;
  ptr2pvt33data_[2] = p3;
  actuator_pvt33process_finished_ = 0;
  internal_delay_counter_ = 0;
  enable_pvt_ = 0;
  pvt_counter_ = 0;
}

void CableRobot::CollectActuatorPvt33Control(int /*state*/)
{
  enable_pvt_ = 1;
  pvt_counter_ = 0;
}

void CableRobot::CollectStartRequest() {}
