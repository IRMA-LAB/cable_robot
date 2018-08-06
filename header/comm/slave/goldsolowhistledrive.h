#ifndef GOLDSOLOWHISTLEDRIVE_H
#define GOLDSOLOWHISTLEDRIVE_H

#include <bitset>
#include <iostream>
#include "ethercatslave.h"
#include "common.h"

class GoldSoloWhistleDrive : public EthercatSlave
{
private:
  constexpr static uint8_t kGoldSoloWhistleDomainEntries_ = 12;
  constexpr static uint8_t kGoldSoloWhistleAlias_ = 0;
  constexpr static uint32_t kGoldSoloWhistleVendorID_ = 0x0000009a;
  constexpr static uint32_t kGoldSoloWhistleProductCode_ = 0x00030924;
  constexpr static uint8_t kNumGoldSoloWhistleStates_ = 7;
  constexpr static uint16_t kControlWordIndex_ = 0x6040;
  constexpr static uint8_t kControlWordSubIndex_ = 0x00;
  constexpr static uint16_t kHomingMethodIndex_ = 0x6098;
  constexpr static uint8_t kHomingMethodSubIndex_ = 0x00;
  constexpr static uint16_t kModesOfOperationIndex_ = 0x6060;
  constexpr static uint8_t kModesOfOperationSubIndex_ = 0x00;
  constexpr static uint16_t kTargetTorqueIndex_ = 0x6071;
  constexpr static uint8_t kTargetTorqueSubIndex_ = 0x00;
  constexpr static uint16_t kTargetPositionIndex_ = 0x607a;
  constexpr static uint8_t kTargetPositionSubIndex_ = 0x00;
  constexpr static uint16_t kTargetVelocityIndex_ = 0x60FF;
  constexpr static uint8_t kTargetVelocitySubIndex_ = 0x00;
  constexpr static uint16_t kStatusWordIndex_ = 0x6041;
  constexpr static uint8_t kStatusWordSubIndex_ = 0x00;
  constexpr static uint16_t kModesOfOperationDisplayIndex_ = 0x6061;
  constexpr static uint8_t kModesOfOperationDisplaySubIndex_ = 0x00;
  constexpr static uint16_t kPositionActualValueIndex_ = 0x6064;
  constexpr static uint8_t kPositionActualValueSubIndex_ = 0x00;
  constexpr static uint16_t kVelocityActualValueIndex_ = 0x606C;
  constexpr static uint8_t kVelocityActualValueSubIndex_ = 0x00;
  constexpr static uint16_t kTorqueActualValueIndex_ = 0x6077;
  constexpr static uint8_t kTorqueActualValueSubIndex_ = 0x00;
  constexpr static uint16_t kDigitalInputsIndex_ = 0x60FD;
  constexpr static uint8_t kDigitalInputsSubIndex_ = 0x00;
  constexpr static uint16_t kAuxiliaryPositionActualValueIndex_ = 0x20A0;
  constexpr static uint8_t kAuxiliaryPositionActualValueSubIndex_ = 0x00;
  constexpr static uint8_t kHomingOnPositionMethod_ = 35;
  constexpr static uint8_t kNumSupportedOperations_ = 3;

  struct OffsetOut
  { // Useful ethercat struct
    unsigned int control_word;
    unsigned int modes_of_operation;
    unsigned int target_torque;
    unsigned int target_position;
    unsigned int target_velocity;
  } offset_out_;

  struct OffsetIn
  { // Useful ethercat struct
    unsigned int status_word;
    unsigned int modes_of_operation_display;
    unsigned int position_actual_value;
    unsigned int velocity_actual_value;
    unsigned int torque_actual_value;
    unsigned int digital_inputs;
    unsigned int auxiliary_position_actual_value;
  } offset_in_;

  typedef void (
    GoldSoloWhistleDrive::*StateFunction)(); // Easyway to implement state machine

  void DetermineState();
  void DetermineOperationState();

  void SwitchOnDisabledFun();
  void ReadyToSwitchOnFun();
  void SwitchOnFun();
  void OperationEnabledFun();
  void QuickStopActiveFun();
  void FaultReactionActiveFun();
  void FaultFun();

  void SwitchOnDisabledTransitions();
  void ReadyToSwitchOnTransitions();
  void SwitchOnTransitions();
  void OperationEnabledTransitions();
  void QuickStopActiveTransitions();
  void FaultReactionActiveTransitions();
  void FaultTransitions();

  void CyclicPositionFun();
  void CyclicVelocityFun();
  void CyclicTorqueFun();

  void CyclicPositionTransition();
  void CyclicVelocityTransition();
  void CyclicTorqueTransition();

public:
  GoldSoloWhistleDrive(uint8_t slave_position);
  ~GoldSoloWhistleDrive() {}

  constexpr static uint8_t kGoldSoloWhistleDomainInputs_ = 7;
  constexpr static uint8_t kGoldSoloWhistleDomainOutputs_ = 5;

  enum GoldSoloWhistleDriveState
  {
    SWITCH_ON_DISABLED,
    READY2SWITCH_ON,
    SWITCH_ON,
    OPERATION_ENABLED,
    QUICK_STOP_ACTIVE,     // To be implemented, fast stop during operation
    FAULT_REACTION_ACTIVE, // To be implemented, reaction to a specific fault
    FAULT,
    NULL_STATE // usefull null flag
  } state_,
    state_flags_; // state machine utilities
  // State machine function array
  StateFunction state_machine_[kNumGoldSoloWhistleStates_] = {
    &GoldSoloWhistleDrive::SwitchOnDisabledFun, &GoldSoloWhistleDrive::ReadyToSwitchOnFun,
    &GoldSoloWhistleDrive::SwitchOnFun, &GoldSoloWhistleDrive::OperationEnabledFun,
    &GoldSoloWhistleDrive::QuickStopActiveFun,
    &GoldSoloWhistleDrive::FaultReactionActiveFun, &GoldSoloWhistleDrive::FaultFun};
  // State machine transition function array
  StateFunction state_manager_[kNumGoldSoloWhistleStates_] = {
    &GoldSoloWhistleDrive::SwitchOnDisabledTransitions,
    &GoldSoloWhistleDrive::ReadyToSwitchOnTransitions,
    &GoldSoloWhistleDrive::SwitchOnTransitions,
    &GoldSoloWhistleDrive::OperationEnabledTransitions,
    &GoldSoloWhistleDrive::QuickStopActiveTransitions,
    &GoldSoloWhistleDrive::FaultReactionActiveTransitions,
    &GoldSoloWhistleDrive::FaultTransitions};

  ec_pdo_entry_reg_t
    domain_registers_[kGoldSoloWhistleDomainEntries_]; // ethercat utilities
  ec_pdo_entry_info_t slave_pdo_entries_[kGoldSoloWhistleDomainEntries_] =
    { // ethercat utilities, can be retrieved in the xml config file provided by the vendor
     {kControlWordIndex_, kControlWordSubIndex_, 16}, // Start of RxPdo mapping (Outputs)
     {kModesOfOperationIndex_, kModesOfOperationSubIndex_, 8},
     {kTargetTorqueIndex_, kTargetTorqueSubIndex_, 16},
     {kTargetPositionIndex_, kTargetPositionSubIndex_, 32},
     {kTargetVelocityIndex_, kTargetVelocitySubIndex_, 32},
     {kStatusWordIndex_, kStatusWordSubIndex_, 16}, // Start of TxPdo mapping (Inputs)
     {kModesOfOperationDisplayIndex_, kModesOfOperationDisplaySubIndex_, 8},
     {kPositionActualValueIndex_, kPositionActualValueSubIndex_, 32},
     {kVelocityActualValueIndex_, kVelocityActualValueSubIndex_, 32},
     {kTorqueActualValueIndex_, kTorqueActualValueSubIndex_, 16},
     {kDigitalInputsIndex_, kDigitalInputsSubIndex_, 32},
     {kAuxiliaryPositionActualValueIndex_, kAuxiliaryPositionActualValueSubIndex_, 32}};

  ec_pdo_info_t slave_pdos_[2] = {
    // ethercat utilities, can be retrieved in the xml config file provided by
    // the vendor
    {0x1607, 5, slave_pdo_entries_ + 0}, /* Outputs */
    {0x1a07, 7, slave_pdo_entries_ + 5}, /* Inputs */
  };

  ec_sync_info_t slave_syncs_[5] = {{0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
                                    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
                                    {2, EC_DIR_OUTPUT, 1, slave_pdos_ + 0, EC_WD_ENABLE},
                                    {3, EC_DIR_INPUT, 1, slave_pdos_ + 1, EC_WD_DISABLE},
                                    {0xff, EC_DIR_INVALID, 0, 0x00, EC_WD_DEFAULT}};

  struct OutputPdos
  { // this is a simple way to store the pdos output values
    std::bitset<16> control_word;
    signed char modes_of_operation;
    short target_torque;
    int target_position;
    int target_velocity;
  } output_pdos_;

  enum InputPdosElements
  {
    kStatusWordElement,
    kModesOfOperationElement,
    kPositionActualvalueElement,
    kVelocityActualvalueElement,
    kTorqueActualValueElement,
    kDigitalInputsElement,
    kAuxiliaryPositionActualValueElement
  };

  struct InputPdos
  { // this is a simple way to store the pdos input values
    std::bitset<16> status_word;
    signed char modes_of_operation_display;
    int position_actual_value;
    int velocity_actual_value;
    short torque_actual_value;
    unsigned int digital_inputs;
    int auxiliary_position_actual_value;
  } input_pdos_;

  int operation_offset_ = 8;

  enum GoldSoloWhistleOperationState
  {
    NULL_OPERATION = 0,
    CYCLIC_POSITION = 8,
    CYCLIC_VELOCITY = 9,
    CYCLIC_TORQUE = 10,
  } operation_state_,
    operation_state_flags_;

  StateFunction operation_state_machine_[kNumSupportedOperations_] = {
    &GoldSoloWhistleDrive::CyclicPositionFun, &GoldSoloWhistleDrive::CyclicVelocityFun,
    &GoldSoloWhistleDrive::CyclicTorqueFun};
  // State machine transition function array
  StateFunction operation_state_manager_[kNumSupportedOperations_] = {
    &GoldSoloWhistleDrive::CyclicPositionTransition,
    &GoldSoloWhistleDrive::CyclicVelocityTransition,
    &GoldSoloWhistleDrive::CyclicTorqueTransition};

  void SetTargetDefaults();
  virtual int SdoRequests(ec_sdo_request_t* sdo_ptr, ec_slave_config_t* config_ptr);
  virtual void LoopFunction(); // The function we are overloading from the base class
  virtual void ReadInputs();
  virtual void WriteOutputs();
};

#endif // GOLDSOLOWHISTLEDRIVE_H
