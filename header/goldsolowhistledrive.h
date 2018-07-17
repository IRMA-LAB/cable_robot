#ifndef GOLDSOLOWHISTLEDRIVE_H
#define GOLDSOLOWHISTLEDRIVE_H

#include <bitset>
#include <iostream>
#include "ethercatslave.h"

class GoldSoloWhistleDrive : public EthercatSlave
{
private:
  constexpr static uint8_t GoldSoloWhistleDomainEntries = 12;
  constexpr static uint8_t GoldSoloWhistleAlias = 0;
  constexpr static uint32_t GoldSoloWhistleVendor_id = 0x0000009a;
  constexpr static uint32_t GoldSoloWhistleProduct_code = 0x00030924;
  constexpr static uint8_t numberOfGoldSoloWhistleStates = 7;
  constexpr static uint16_t controlWordIndex = 0x6040;
  constexpr static uint8_t controlWordSubIndex = 0x00;
  constexpr static uint16_t homingMethodIndex = 0x6098;
  constexpr static uint8_t homingMethodSubIndex = 0x00;
  constexpr static uint16_t modesOfOperationIndex = 0x6060;
  constexpr static uint8_t modesOfOperationSubIndex = 0x00;
  constexpr static uint16_t targetTorqueIndex = 0x6071;
  constexpr static uint8_t targetTorqueSubIndex = 0x00;
  constexpr static uint16_t targetPositionIndex = 0x607a;
  constexpr static uint8_t targetPositionSubIndex = 0x00;
  constexpr static uint16_t targetVelocityIndex = 0x60FF;
  constexpr static uint8_t targetVelocitySubIndex = 0x00;
  constexpr static uint16_t statusWordIndex = 0x6041;
  constexpr static uint8_t statusWordSubIndex = 0x00;
  constexpr static uint16_t modesOfOperationDisplayIndex = 0x6061;
  constexpr static uint8_t modesOfOperationDisplaySubIndex = 0x00;
  constexpr static uint16_t positionActualValueIndex = 0x6064;
  constexpr static uint8_t positionActualValueSubIndex = 0x00;
  constexpr static uint16_t velocityActualValueIndex = 0x606C;
  constexpr static uint8_t velocityActualValueSubIndex = 0x00;
  constexpr static uint16_t torqueActualValueIndex = 0x6077;
  constexpr static uint8_t torqueActualValueSubIndex = 0x00;
  constexpr static uint16_t digitalInputsIndex = 0x60FD;
  constexpr static uint8_t digitalInputsSubIndex = 0x00;
  constexpr static uint16_t auxiliaryPositionActualValueIndex = 0x20A0;
  constexpr static uint8_t auxiliaryPositionActualValueSubIndex = 0x00;
  constexpr static uint8_t set = 1;
  constexpr static uint8_t reset = 0;
  constexpr static uint8_t offStatusBit = 6;
  constexpr static uint8_t onStatusBit = 5;
  constexpr static uint8_t readyToSwitchOnStatusBit = 0;
  constexpr static uint8_t switchOnStatusBit = 1;
  constexpr static uint8_t enabledStatusBit = 2;
  constexpr static uint8_t faultStatusBit = 3;
  constexpr static uint8_t switchOnControlBit = 0;
  constexpr static uint8_t enableVoltageControlBit = 1;
  constexpr static uint8_t quickStopControlBit = 2;
  constexpr static uint8_t enableControlBit = 3;
  constexpr static uint8_t faultResetControlBit = 7;
  constexpr static uint8_t homingOnPositionMethod = 35;
  constexpr static uint8_t numberOfSupportedOperations = 3;

  struct OffsetOut
  { // Useful ethercat struct
    unsigned int controlWord;
    unsigned int modesOfOperation;
    unsigned int targetTorque;
    unsigned int targetPosition;
    unsigned int targetVelocity;
  } offsetOut;

  struct OffsetIn
  { // Useful ethercat struct
    unsigned int statusWord;
    unsigned int modesOfOperationDisplay;
    unsigned int positionActualValue;
    unsigned int velocityActualValue;
    unsigned int torqueActualValue;
    unsigned int digitalInputs;
    unsigned int auxiliaryPositionActualValue;
  } offsetIn;

  typedef void (GoldSoloWhistleDrive::*
                  StateFunction)(); // Easyway to implement state machine

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
  GoldSoloWhistleDrive(uint8_t thisSlavePosition);
  ~GoldSoloWhistleDrive() {}
  constexpr static uint8_t GoldSoloWhistleDomainInputs = 7;
  constexpr static uint8_t GoldSoloWhistleDomainOutputs = 5;
  enum GoldSoloWhistleDriveState
  {
    switchOnDisabled,
    readyToSwitchOn,
    switchOn,
    operationEnabled,
    quickStopActive,     // To be implemented, fast stop during operation
    faultReactionActive, // To be implemented, reaction to a specific fault
    fault,
    nullState // usefull null flag
  } state,
    stateFlags; // state machine utilities
  // State machine function array
  StateFunction stateMachine[numberOfGoldSoloWhistleStates] = {
    &GoldSoloWhistleDrive::SwitchOnDisabledFun,
    &GoldSoloWhistleDrive::ReadyToSwitchOnFun,
    &GoldSoloWhistleDrive::SwitchOnFun,
    &GoldSoloWhistleDrive::OperationEnabledFun,
    &GoldSoloWhistleDrive::QuickStopActiveFun,
    &GoldSoloWhistleDrive::FaultReactionActiveFun,
    &GoldSoloWhistleDrive::FaultFun};
  // State machine transition function array
  StateFunction stateManager[numberOfGoldSoloWhistleStates] = {
    &GoldSoloWhistleDrive::SwitchOnDisabledTransitions,
    &GoldSoloWhistleDrive::ReadyToSwitchOnTransitions,
    &GoldSoloWhistleDrive::SwitchOnTransitions,
    &GoldSoloWhistleDrive::OperationEnabledTransitions,
    &GoldSoloWhistleDrive::QuickStopActiveTransitions,
    &GoldSoloWhistleDrive::FaultReactionActiveTransitions,
    &GoldSoloWhistleDrive::FaultTransitions};

  ec_pdo_entry_reg_t
    domainRegisters[GoldSoloWhistleDomainEntries]; // ethercat utilities
  ec_pdo_entry_info_t slavePdoEntries[GoldSoloWhistleDomainEntries] =
    { // ethercat utilities, can be retrieved in the xml config file provided by
      // the vendor
     {controlWordIndex, controlWordSubIndex,
      16}, // Start of RxPdo mapping (Outputs)
     {modesOfOperationIndex, modesOfOperationSubIndex, 8},
     {targetTorqueIndex, targetTorqueSubIndex, 16},
     {targetPositionIndex, targetPositionSubIndex, 32},
     {targetVelocityIndex, targetVelocitySubIndex, 32},
     {statusWordIndex, statusWordSubIndex,
      16}, // Start of TxPdo mapping (Inputs)
     {modesOfOperationDisplayIndex, modesOfOperationDisplaySubIndex, 8},
     {positionActualValueIndex, positionActualValueSubIndex, 32},
     {velocityActualValueIndex, velocityActualValueSubIndex, 32},
     {torqueActualValueIndex, torqueActualValueSubIndex, 16},
     {digitalInputsIndex, digitalInputsSubIndex, 32},
     {auxiliaryPositionActualValueIndex, auxiliaryPositionActualValueSubIndex,
      32}};

  ec_pdo_info_t slavePdos[2] = {
    // ethercat utilities, can be retrieved in the xml config file provided by
    // the vendor
    {0x1607, 5, slavePdoEntries + 0}, /* Outputs */
    {0x1a07, 7, slavePdoEntries + 5}, /* Inputs */
  };

  ec_sync_info_t slaveSyncs[5] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slavePdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slavePdos + 1, EC_WD_DISABLE},
    {0xff, EC_DIR_INVALID, 0, 0x00, EC_WD_DEFAULT}};

  struct OutputPdos
  { // this is a simple way to store the pdos output values
    std::bitset<16> controlWord;
    signed char modesOfOperation;
    short TargetTorque;
    int TargetPosition;
    int TargetVelocity;
  } outputPdos;

  enum InputPdosElements
  {
    statusWordElement,
    modesOfOperationElement,
    positionActualvalueElement,
    velocityActualvalueElement,
    torqueActualValueElement,
    digitalInputsElement,
    auxiliaryPositionActualValueElement
  };
  struct InputPdos
  { // this is a simple way to store the pdos input values
    std::bitset<16> statusWord;
    signed char modesOfOperationDisplay;
    int positionActualValue;
    int velocityActualValue;
    short torqueActualValue;
    unsigned int digitalInputs;
    int auxiliaryPositionActualValue;
  } inputPdos;
  int operationOffset = 8;
  enum GoldSoloWhistleOperationState
  {
    nullOperation = 0,
    cyclicPosition = 8,
    cyclicVelocity = 9,
    cyclicTorque = 10,
  } operationState,
    operationStateFlags;

  StateFunction operationStateMachine[numberOfSupportedOperations] = {
    &GoldSoloWhistleDrive::CyclicPositionFun,
    &GoldSoloWhistleDrive::CyclicVelocityFun,
    &GoldSoloWhistleDrive::CyclicTorqueFun};
  // State machine transition function array
  StateFunction operationStateManager[numberOfSupportedOperations] = {
    &GoldSoloWhistleDrive::CyclicPositionTransition,
    &GoldSoloWhistleDrive::CyclicVelocityTransition,
    &GoldSoloWhistleDrive::CyclicTorqueTransition};

  void SetTargetDefaults();
  virtual int SdoRequests(ec_sdo_request_t* sdoPointer,
                          ec_slave_config_t* configPointer);
  virtual void
  LoopFunction(); // The function we are overloading from the base class
  virtual void ReadInputs();
  virtual void WriteOutputs();
};

#endif // GOLDSOLOWHISTLEDRIVE_H
