#include "goldsolowhistledrive.h"

void GoldSoloWhistleDrive::DetermineState()
{
  if (input_pdos_.status_word[offStatusBit] == SET)
  { // drive idle
    state_ = SWITCH_ON_DISABLED;
  }
  else
  {
    if (input_pdos_.status_word[onStatusBit] == SET)
    { // drive operational or in operational progress
      if (input_pdos_.status_word[switchOnStatusBit] == RESET)
      {
        state_ = READY2SWITCH_ON;
      }
      else
      {
        if (input_pdos_.status_word[enabledStatusBit] == RESET)
        {
          state_ = SWITCH_ON;
        }
        else
        {
          state_ = OPERATION_ENABLED;
          DetermineOperationState();
        }
      }
    }
    else
    { // drive in a stop or fault
      if (input_pdos_.status_word[faultStatusBit] == RESET)
      { // drive in quick stop
        state_ = QUICK_STOP_ACTIVE;
      }
      else
      { // drive in fault
        if (input_pdos_.status_word[enabledStatusBit] == SET)
        {
          state_ = FAULT_REACTION_ACTIVE;
        }
        else
        {
          state_ = FAULT;
        }
      }
    }
  }
}

void GoldSoloWhistleDrive::DetermineOperationState()
{
  switch (input_pdos_.modes_of_operation_display)
  {
  case CYCLIC_POSITION:
  {
    operation_state_ = CYCLIC_POSITION;
    break;
  }
  case CYCLIC_VELOCITY:
  {
    operation_state_ = CYCLIC_VELOCITY;
    break;
  }
  case CYCLIC_TORQUE:
  {
    operation_state_ = CYCLIC_TORQUE;
    break;
  }
  default:
    break;
  }
}

void GoldSoloWhistleDrive::SwitchOnDisabledFun() {}

void GoldSoloWhistleDrive::ReadyToSwitchOnFun() {}

void GoldSoloWhistleDrive::SwitchOnFun() {}

void GoldSoloWhistleDrive::OperationEnabledFun() {}

void GoldSoloWhistleDrive::QuickStopActiveFun() {}

void GoldSoloWhistleDrive::FaultReactionActiveFun() {}

void GoldSoloWhistleDrive::FaultFun() {}

void GoldSoloWhistleDrive::SwitchOnDisabledTransitions()
{
  switch (state_flags_)
  {
  case SWITCH_ON_DISABLED:
  { // We previously asked for a state change: it occurred
    std::cout << "Drive " << position_ << " Status Word: " << input_pdos_.status_word
              << std::endl;
    std::cout << "Drive " << position_ << " Idle." << std::endl;
    state_flags_ = NULL_STATE;
    break;
  }
  case READY2SWITCH_ON:
  { // We are starting the enabling sequence, transition 2
    std::cout << "Drive " << position_ << " Status Word: " << input_pdos_.status_word
              << std::endl;
    std::cout << "Drive " << position_ << " requesting Ready To Switch On." << std::endl;
    output_pdos_.control_word[switchOnControlBit] = RESET;
    output_pdos_.control_word[enableVoltageControlBit] = SET;
    output_pdos_.control_word[quickStopControlBit] = SET;
    output_pdos_.control_word[enableControlBit] = RESET;
    output_pdos_.control_word[faultResetControlBit] = RESET;
    break;
  }
  default:
    break;
  }
}

void GoldSoloWhistleDrive::ReadyToSwitchOnTransitions()
{
  switch (state_flags_)
  {
  case READY2SWITCH_ON:
  { // We previously asked for a feasible state change, now we ask for another
    std::cout << "Drive " << position_ << " Status Word: " << input_pdos_.status_word
              << std::endl;
    std::cout << "Drive " << position_ << " Ready To Switch On." << std::endl;
    output_pdos_.control_word[switchOnControlBit] = SET;
    state_flags_ = SWITCH_ON;
    output_pdos_.modes_of_operation = CYCLIC_POSITION;
    output_pdos_.target_position = input_pdos_.position_actual_value;
    break;
  }
  default:
    break;
  }
}

void GoldSoloWhistleDrive::SwitchOnTransitions()
{
  switch (state_flags_)
  {
  case SWITCH_ON:
  { // We previously asked for a feasible state change, now we ask for another
    std::cout << "Drive " << position_ << " Status Word: " << input_pdos_.status_word
              << std::endl;
    std::cout << "Drive " << position_ << " Switch On." << std::endl;
    // outputPdos.TargetPosition = inputPdos.positionActualValue;
    // outputPdos.TargetTorque = inputPdos.torqueActualValue;
    // outputPdos.TargetVelocity = inputPdos.velocityActualValue;
    output_pdos_.control_word[enableControlBit] = SET;
    state_flags_ = OPERATION_ENABLED;
    break;
  }
  default:
    break;
  }
}

void GoldSoloWhistleDrive::OperationEnabledTransitions()
{
  switch (state_flags_)
  {
  case OPERATION_ENABLED:
  { // We previously asked for a state change: it occurred
    std::cout << "Drive " << position_ << " Status Word: " << input_pdos_.status_word
              << std::endl;
    std::cout << "Drive " << position_ << "  Enabled." << std::endl;
    state_flags_ = NULL_STATE;
    (this->*operation_state_manager_[operation_state_ - operation_offset_])();
    break;
  }
  case SWITCH_ON_DISABLED:
  { // We want to disable the drive
    std::cout << "Drive " << position_ << " Status Word: " << input_pdos_.status_word
              << std::endl;
    std::cout << "Drive " << position_ << " going Idle" << std::endl;
    output_pdos_.control_word.reset();
    break;
  }
  default:
  {
    (this->*operation_state_manager_[operation_state_ - operation_offset_])();
    break;
  }
  }
}

void GoldSoloWhistleDrive::QuickStopActiveTransitions()
{
  state_flags_ = NULL_STATE; // We shouldn't be here..
}

void GoldSoloWhistleDrive::FaultReactionActiveTransitions()
{
  state_flags_ = NULL_STATE; // We shouldn't be here..
}

void GoldSoloWhistleDrive::FaultTransitions()
{
  switch (state_flags_)
  {
  case SWITCH_ON_DISABLED:
  { // we are requesting a fault reset
    std::cout << "Drive " << position_ << " Status Word: " << input_pdos_.status_word
              << std::endl;
    std::cout << "Drive " << position_ << " requesting Fault Reset: going Idle"
              << std::endl;
    output_pdos_.control_word.reset();
    output_pdos_.control_word[faultResetControlBit] = SET;
    break;
  }
  case NULL_STATE:
    break;
  default:
  {
    std::cout << "Drive " << position_ << " Status Word: " << input_pdos_.status_word
              << std::endl;
    std::cout << "Drive " << position_
              << " encountered an error asking for state change code: " << state_flags_
              << std::endl;
    state_flags_ = NULL_STATE;
    break;
  }
  }
}

void GoldSoloWhistleDrive::CyclicPositionFun() {}

void GoldSoloWhistleDrive::CyclicVelocityFun() {}

void GoldSoloWhistleDrive::CyclicTorqueFun() {}

void GoldSoloWhistleDrive::CyclicPositionTransition()
{
  switch (operation_state_flags_)
  {
  case CYCLIC_POSITION:
  {
    operation_state_flags_ = NULL_OPERATION;
    break;
  }
  case CYCLIC_VELOCITY:
  {
    output_pdos_.modes_of_operation = CYCLIC_VELOCITY;
    output_pdos_.target_velocity = input_pdos_.velocity_actual_value;
    break;
  }
  case CYCLIC_TORQUE:
  {
    output_pdos_.modes_of_operation = CYCLIC_TORQUE;
    output_pdos_.target_torque = input_pdos_.torque_actual_value;
    break;
  }
  default:
    break;
  }
}

void GoldSoloWhistleDrive::CyclicVelocityTransition()
{
  switch (operation_state_flags_)
  {
  case CYCLIC_POSITION:
  {
    output_pdos_.modes_of_operation = CYCLIC_POSITION;
    output_pdos_.target_position = input_pdos_.position_actual_value;
    break;
  }
  case CYCLIC_VELOCITY:
  {
    operation_state_flags_ = NULL_OPERATION;
    break;
  }
  case CYCLIC_TORQUE:
  {
    output_pdos_.modes_of_operation = CYCLIC_TORQUE;
    output_pdos_.target_torque = input_pdos_.torque_actual_value;
    break;
  }
  default:
    break;
  }
}

void GoldSoloWhistleDrive::CyclicTorqueTransition()
{
  switch (operation_state_flags_)
  {
  case CYCLIC_POSITION:
  {
    output_pdos_.modes_of_operation = CYCLIC_POSITION;
    output_pdos_.target_position = input_pdos_.position_actual_value;
    break;
  }
  case CYCLIC_VELOCITY:
  {
    output_pdos_.modes_of_operation = CYCLIC_VELOCITY;
    output_pdos_.target_velocity = input_pdos_.velocity_actual_value;
    break;
  }
  case CYCLIC_TORQUE:
  {
    operation_state_flags_ = NULL_OPERATION;
    break;
  }
  default:
    break;
  }
}

GoldSoloWhistleDrive::GoldSoloWhistleDrive(uint8_t slave_position)
{
  /* It is FUNDAMENTAL that the constructor has this form. There are several
   * other ways we can program
   * the slave interface so that we do not have to deal with the assignment of
   * this variables in the constructor,
   * but they will require the user to initialize some of the variables in the
   * main file and then pass the as
   * input variable in the constructor. I find that this way it is easy for "not
   * experienced user" to write all their code
   * in just two files, th .h and .cpp of their actual implementation of the
   * ethercat slave
  */

  // From here, it must be edited by the user
  alias_ = kGoldSoloWhistleAlias_;
  position_ = slave_position;
  vendor_id_ = kGoldSoloWhistleVendorID_;
  product_code_ = kGoldSoloWhistleProductCode_;
  num_domain_entries_ = kGoldSoloWhistleDomainEntries_;

  domain_registers_[0] = {alias_, position_, vendor_id_, product_code_, kControlWordIndex_,
                        kControlWordSubIndex_, &offset_out_.control_word, NULL};
  domain_registers_[1] = {alias_, position_, vendor_id_, product_code_, kModesOfOperationIndex_,
                        kModesOfOperationSubIndex_, &offset_out_.modes_of_operation, NULL};
  domain_registers_[2] = {alias_, position_, vendor_id_, product_code_, kTargetTorqueIndex_,
                        kTargetTorqueSubIndex_, &offset_out_.target_torque, NULL};
  domain_registers_[3] = {alias_, position_, vendor_id_, product_code_, kTargetPositionIndex_,
                        kTargetPositionSubIndex_, &offset_out_.target_position, NULL};
  domain_registers_[4] = {alias_, position_, vendor_id_, product_code_, kTargetVelocityIndex_,
                        kTargetVelocitySubIndex_, &offset_out_.target_velocity, NULL};
  domain_registers_[5] = {alias_, position_, vendor_id_, product_code_, kStatusWordIndex_,
                        kStatusWordSubIndex_, &offset_in_.status_word, NULL};
  domain_registers_[6] = {alias_, position_, vendor_id_, product_code_,
                        kModesOfOperationDisplayIndex_, kModesOfOperationDisplaySubIndex_,
                        &offset_in_.modes_of_operation_display, NULL};
  domain_registers_[7] = {alias_, position_, vendor_id_, product_code_,
                        kPositionActualValueIndex_, kPositionActualValueSubIndex_,
                        &offset_in_.position_actual_value, NULL};
  domain_registers_[8] = {alias_, position_, vendor_id_, product_code_,
                        kVelocityActualValueIndex_, kVelocityActualValueSubIndex_,
                        &offset_in_.velocity_actual_value, NULL};
  domain_registers_[9] = {alias_, position_, vendor_id_, product_code_, kTorqueActualValueIndex_,
                        kTorqueActualValueSubIndex_, &offset_in_.torque_actual_value, NULL};
  domain_registers_[10] = {alias_, position_, vendor_id_, product_code_, kDigitalInputsIndex_,
                         kDigitalInputsSubIndex_, &offset_in_.digital_inputs, NULL};
  domain_registers_[11] = {
    alias_, position_, vendor_id_, product_code_, kAuxiliaryPositionActualValueIndex_,
    kAuxiliaryPositionActualValueSubIndex_, &offset_in_.auxiliary_position_actual_value, NULL};

  domain_registers_ptr_ = domain_registers_;
  slave_pdo_entries_ptr_ = slave_pdo_entries_;
  slave_pdos_ptr_ = slave_pdos_;
  slave_sync_ptr_ = slave_syncs_;
  // and stop here, the rest is additional

  state_flags_ = NULL_STATE;
  state_ = SWITCH_ON_DISABLED;
  operation_state_flags_ = NULL_OPERATION;
  operation_state_ = CYCLIC_POSITION;
}

void GoldSoloWhistleDrive::SetTargetDefaults()
{
  switch (input_pdos_.modes_of_operation_display)
  {
  case CYCLIC_POSITION:
  {
    output_pdos_.target_position = input_pdos_.position_actual_value;
    break;
  }
  case CYCLIC_VELOCITY:
  {
    output_pdos_.target_velocity = input_pdos_.velocity_actual_value;
    break;
  }
  case CYCLIC_TORQUE:
  {
    output_pdos_.target_torque = input_pdos_.torque_actual_value;
    break;
  }
  default:
    break;
  }
}

int GoldSoloWhistleDrive::SdoRequests(ec_sdo_request_t* sdo_ptr,
                                      ec_slave_config_t* config_ptr)
{
  if (!(sdo_ptr =
          ecrt_slave_config_create_sdo_request(config_ptr, kModesOfOperationIndex_,
                                               kModesOfOperationSubIndex_, CYCLIC_POSITION)))
  {
    std::cout << "Failed to create SDO request." << std::endl;
    return 1;
  }
  ecrt_sdo_request_timeout(sdo_ptr, 500);
  ecrt_slave_config_sdo8(config_ptr, kModesOfOperationIndex_, kModesOfOperationSubIndex_,
                         CYCLIC_POSITION);
  if (!(sdo_ptr = ecrt_slave_config_create_sdo_request(
          config_ptr, kHomingMethodIndex_, kHomingMethodSubIndex_,
          kHomingOnPositionMethod_)))
  {
    std::cout << "Failed to create SDO request." << std::endl;
    return 1;
  }
  ecrt_sdo_request_timeout(sdo_ptr, 500);
  ecrt_slave_config_sdo8(config_ptr, kHomingMethodIndex_, kHomingMethodSubIndex_,
                         kHomingOnPositionMethod_);
  return 0;
}

void GoldSoloWhistleDrive::LoopFunction() { (this->*state_machine_[state_])(); }

void GoldSoloWhistleDrive::ReadInputs()
{
  input_pdos_.status_word = EC_READ_U16(domain_data_ptr_ + offset_in_.status_word);
  input_pdos_.modes_of_operation_display =
    EC_READ_S8(domain_data_ptr_ + offset_in_.modes_of_operation_display);
  DetermineState();
  input_pdos_.position_actual_value =
    EC_READ_S32(domain_data_ptr_ + offset_in_.position_actual_value);
  input_pdos_.velocity_actual_value =
    EC_READ_S32(domain_data_ptr_ + offset_in_.velocity_actual_value);
  input_pdos_.torque_actual_value =
    EC_READ_S16(domain_data_ptr_ + offset_in_.torque_actual_value);
  input_pdos_.digital_inputs = EC_READ_U32(domain_data_ptr_ + offset_in_.digital_inputs);
  input_pdos_.auxiliary_position_actual_value =
    EC_READ_S32(domain_data_ptr_ + offset_in_.auxiliary_position_actual_value);
  (this->*state_manager_[state_])();
}

void GoldSoloWhistleDrive::WriteOutputs()
{
  EC_WRITE_U16(domain_data_ptr_ + offset_out_.control_word,
               static_cast<unsigned short>(output_pdos_.control_word.to_ulong()));
  EC_WRITE_S8(domain_data_ptr_ + offset_out_.modes_of_operation,
              output_pdos_.modes_of_operation);
  if (state_ == OPERATION_ENABLED || state_ == SWITCH_ON)
  {
    EC_WRITE_S32(domain_data_ptr_ + offset_out_.target_position, output_pdos_.target_position);
    EC_WRITE_S32(domain_data_ptr_ + offset_out_.target_velocity, output_pdos_.target_velocity);
    EC_WRITE_S16(domain_data_ptr_ + offset_out_.target_torque, output_pdos_.target_torque);
  }
}
