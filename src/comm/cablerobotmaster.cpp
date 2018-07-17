#include "cablerobotmaster.h"

void CableRobotMaster::IdleFun() {}

void CableRobotMaster::ActuatorControlFun()
{
  if (servo_motor_ != NULL)
  {
    int flag = servo_motor_->HasEnableRequestBeenProcessed();
    if (flag < 2)
    {
      emit SendEnableRequestProcessed(flag);
    }
    else if (servo_motor_->HasClearFaultRequestBeenProcessed())
    {
      emit SendClearFaultRequestProcessed();
    }
    else if (servo_motor_->HasOperationModeChangeRequestBeenProcessed())
    {
      emit SendOperationModeChangeRequestProcessed(
        static_cast<int>(servo_motor_->servo_motor_operation_state_));
    }
    else if (servo_motor_->FaultPresent())
    {
      emit SendFaultPresentAdvice();
    }
    servo_motor_->UpdateState();
    servo_motor_->LoopFunction();
    if (servo_motor_->update_idx_ ==
        static_cast<int>(GoldSoloWhistleDrive::kGoldSoloWhistleDomainInputs_))
      servo_motor_->update_idx_ = 0;
    emit SendGuiData(servo_motor_->servo_motor_input_pdos_, servo_motor_->update_idx_);
    servo_motor_->update_idx_++;
  }
}

void CableRobotMaster::EasyCatControlFun()
{
  for (int i = 0; i < kEasyCatNum_; i++)
    easycat_slave_[i].LoopFunction();
}

void CableRobotMaster::StandardRobotOperationFun() { cable_robot_.StandardLoopFunction(); }

void CableRobotMaster::UserRobotOperationFun() { cable_robot_.UserLoopFunction(); }

void CableRobotMaster::IdleTransition()
{
  if (state_flags_ == IDLE)
  {
    state_flags_ = NULL_STATE;
    emit SendMasterRequestProcessed(state_);
    std::cout << "Changing state to " << state_ << std::endl;
  }
  else if (state_flags_ != NULL_STATE)
  {
    state_ = state_flags_;
    for (int i = 0; i < kGoldSoloWhistleNum_; i++)
      gold_solo_whistle_slave_[i].SetTargetDefaults();
  }
}

void CableRobotMaster::ActuatorControlTransition()
{
  if (state_flags_ == ACTUATOR_CTRL)
  {
    state_flags_ = NULL_STATE;
    emit SendMasterRequestProcessed(state_);
    std::cout << "Changing state to " << state_ << std::endl;
  }
  else if (state_flags_ != NULL_STATE)
  {
    state_ = state_flags_;
    servo_motor_ = NULL;
  }
}

void CableRobotMaster::EasyCatControlTransition()
{
  if (state_flags_ == EASYCAT_CTRL)
  {
    state_flags_ = NULL_STATE;
    emit SendMasterRequestProcessed(state_);
    std::cout << "Changing state to " << state_ << std::endl;
  }
  else if (state_flags_ != NULL_STATE)
  {
    state_ = state_flags_;
  }
}

void CableRobotMaster::StandardRobotOperationTransition()
{
  if (state_flags_ == STD_ROBOT_OPERATION)
  {
    state_flags_ = NULL_STATE;
    emit SendMasterRequestProcessed(state_);
    std::cout << "Changing state to " << state_ << std::endl;
  }
  else if (state_flags_ != NULL_STATE)
  {
    state_ = state_flags_;
  }
}

void CableRobotMaster::UserRobotOperationTransition()
{
  if (state_flags_ == USER_ROBOT_OPERATION)
  {
    state_flags_ = NULL_STATE;
    emit SendMasterRequestProcessed(state_);
    std::cout << "Changing state to " << state_ << std::endl;
  }
  else if (state_flags_ != NULL_STATE)
  {
    state_ = state_flags_;
  }
}

CableRobotMaster::CableRobotMaster(QObject* parent)
  : QObject(parent), cable_robot_(nullptr, gold_solo_whistle_slave_)
{
  /* It is FUNDAMENTAL that the constructor has this form. There are several
   * other ways we can program
   * the slave interface so that we do not have to deal with the assignment of
   * this variables in the constructor,
   * but they will require the user to initialize some of the variables in the
   * main file and then pass the as
   * input variable in the constructor. I find that this way it is easy for "not
   * experienced user" to write all their code
   * in just two files, the .h and .cpp of their actual implementation of the
   * ethercat master
  */

  num_slaves_ = kSlavesNum_;
  slave_ = &etherCatSlave_[0];
  for (int i = 0; i < num_slaves_; i++)
    num_domain_elements_ += etherCatSlave_[i]->num_domain_entries_;

  state_ = IDLE;
  state_flags_ = NULL_STATE;
}

void CableRobotMaster::StartUpFunction() { emit SendStartUp(); }

void CableRobotMaster::LoopFunction()
{
  for (int i = 0; i < kSlavesNum_; i++)
    etherCatSlave_[i]->ReadInputs(); // Read pdos, act accordingly
  (this->*state_manager_[state_])();   // check if the state has to be changed
  (this->*state_machine_[state_])();   // do your job
  for (int i = 0; i < kSlavesNum_; i++)
    etherCatSlave_[i]->WriteOutputs(); // Write all the necessary pdos
}

void CableRobotMaster::CollectMasterRequest(int state)
{
  state_flags_ = CableRobotMaster::MasterState(state);
}

void CableRobotMaster::CollectMotorNumber(int motor_num)
{
  servo_motor_ = &cable_robot_.servo_motor_[motor_num];
}

void CableRobotMaster::CollectEnableRequest(int enable)
{
  if (enable)
    servo_motor_->Enable();
  else
    servo_motor_->Disable();
}

void CableRobotMaster::CollectClearFaultRequest() { servo_motor_->FaultReset(); }

void CableRobotMaster::CollectOperationModeChangeRequest(int target_op_mode)
{
  servo_motor_->ChangeOperationMode(target_op_mode);
}

void CableRobotMaster::CollectCommandUpdateRequest(int cmd, int state)
{
  servo_motor_->SetCommand(cmd, state);
  emit SendCommandUpdateRequestProcessed(cmd, state);
}
