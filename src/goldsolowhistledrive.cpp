#include "goldsolowhistledrive.h"

void GoldSoloWhistleDrive::DetermineState()
{
  if (inputPdos.statusWord[offStatusBit] == set)
  { // drive idle
    state = switchOnDisabled;
  }
  else
  {
    if (inputPdos.statusWord[onStatusBit] == set)
    { // drive operational or in operational progress
      if (inputPdos.statusWord[switchOnStatusBit] == reset)
      {
        state = readyToSwitchOn;
      }
      else
      {
        if (inputPdos.statusWord[enabledStatusBit] == reset)
        {
          state = switchOn;
        }
        else
        {
          state = operationEnabled;
          DetermineOperationState();
        }
      }
    }
    else
    { // drive in a stop or fault
      if (inputPdos.statusWord[faultStatusBit] == reset)
      { // drive in quick stop
        state = quickStopActive;
      }
      else
      { // drive in fault
        if (inputPdos.statusWord[enabledStatusBit] == set)
        {
          state = faultReactionActive;
        }
        else
        {
          state = fault;
        }
      }
    }
  }
}

void GoldSoloWhistleDrive::DetermineOperationState()
{
  switch (inputPdos.modesOfOperationDisplay)
  {
  case cyclicPosition:
  {
    operationState = cyclicPosition;
    break;
  }
  case cyclicVelocity:
  {
    operationState = cyclicVelocity;
    break;
  }
  case cyclicTorque:
  {
    operationState = cyclicTorque;
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
  switch (stateFlags)
  {
  case switchOnDisabled:
  { // We previously asked for a state change: it occurred
    cout << "Drive " << position << " Status Word: " << inputPdos.statusWord
         << endl;
    cout << "Drive " << position << " Idle." << endl;
    stateFlags = nullState;
    break;
  }
  case readyToSwitchOn:
  { // We are starting the enabling sequence, transition 2
    cout << "Drive " << position << " Status Word: " << inputPdos.statusWord
         << endl;
    cout << "Drive " << position << " requesting Ready To Switch On." << endl;
    outputPdos.controlWord[switchOnControlBit] = reset;
    outputPdos.controlWord[enableVoltageControlBit] = set;
    outputPdos.controlWord[quickStopControlBit] = set;
    outputPdos.controlWord[enableControlBit] = reset;
    outputPdos.controlWord[faultResetControlBit] = reset;
    break;
  }
  default:
    break;
  }
}

void GoldSoloWhistleDrive::ReadyToSwitchOnTransitions()
{
  switch (stateFlags)
  {
  case readyToSwitchOn:
  { // We previously asked for a feasible state change, now we ask for another
    cout << "Drive " << position << " Status Word: " << inputPdos.statusWord
         << endl;
    cout << "Drive " << position << " Ready To Switch On." << endl;
    outputPdos.controlWord[switchOnControlBit] = set;
    stateFlags = switchOn;
    outputPdos.modesOfOperation = cyclicPosition;
    outputPdos.TargetPosition = inputPdos.positionActualValue;
    break;
  }
  default:
    break;
  }
}

void GoldSoloWhistleDrive::SwitchOnTransitions()
{
  switch (stateFlags)
  {
  case switchOn:
  { // We previously asked for a feasible state change, now we ask for another
    cout << "Drive " << position << " Status Word: " << inputPdos.statusWord
         << endl;
    cout << "Drive " << position << " Switch On." << endl;
    // outputPdos.TargetPosition = inputPdos.positionActualValue;
    // outputPdos.TargetTorque = inputPdos.torqueActualValue;
    // outputPdos.TargetVelocity = inputPdos.velocityActualValue;
    outputPdos.controlWord[enableControlBit] = set;
    stateFlags = operationEnabled;
    break;
  }
  default:
    break;
  }
}

void GoldSoloWhistleDrive::OperationEnabledTransitions()
{
  switch (stateFlags)
  {
  case operationEnabled:
  { // We previously asked for a state change: it occurred
    cout << "Drive " << position << " Status Word: " << inputPdos.statusWord
         << endl;
    cout << "Drive " << position << "  Enabled." << endl;
    stateFlags = nullState;
    (this->*operationStateManager[operationState - operationOffset])();
    break;
  }
  case switchOnDisabled:
  { // We want to disable the drive
    cout << "Drive " << position << " Status Word: " << inputPdos.statusWord
         << endl;
    cout << "Drive " << position << " going Idle" << endl;
    outputPdos.controlWord.reset();
  }
  default:
  {
    (this->*operationStateManager[operationState - operationOffset])();
    break;
  }
  }
}

void GoldSoloWhistleDrive::QuickStopActiveTransitions()
{
  stateFlags = nullState; // We shouldn't be here..
}

void GoldSoloWhistleDrive::FaultReactionActiveTransitions()
{
  stateFlags = nullState; // We shouldn't be here..
}

void GoldSoloWhistleDrive::FaultTransitions()
{
  switch (stateFlags)
  {
  case switchOnDisabled:
  { // we are requesting a fault reset
    cout << "Drive " << position << " Status Word: " << inputPdos.statusWord
         << endl;
    cout << "Drive " << position << " requesting Fault Reset: going Idle"
         << endl;
    outputPdos.controlWord.reset();
    outputPdos.controlWord[faultResetControlBit] = set;
    break;
  }
  case nullState:
    break;
  default:
  {
    cout << "Drive " << position << " Status Word: " << inputPdos.statusWord
         << endl;
    cout << "Drive " << position
         << " encountered an error asking for state change code: " << stateFlags
         << endl;
    stateFlags = nullState;
    break;
  }
  }
}

void GoldSoloWhistleDrive::CyclicPositionFun() {}

void GoldSoloWhistleDrive::CyclicVelocityFun() {}

void GoldSoloWhistleDrive::CyclicTorqueFun() {}

void GoldSoloWhistleDrive::CyclicPositionTransition()
{
  switch (operationStateFlags)
  {
  case cyclicPosition:
  {
    operationStateFlags = nullOperation;
    break;
  }
  case cyclicVelocity:
  {
    outputPdos.modesOfOperation = cyclicVelocity;
    outputPdos.TargetVelocity = inputPdos.velocityActualValue;
    break;
  }
  case cyclicTorque:
  {
    outputPdos.modesOfOperation = cyclicTorque;
    outputPdos.TargetTorque = inputPdos.torqueActualValue;
    break;
  }
  default:
    break;
  }
}

void GoldSoloWhistleDrive::CyclicVelocityTransition()
{
  switch (operationStateFlags)
  {
  case cyclicPosition:
  {
    outputPdos.modesOfOperation = cyclicPosition;
    outputPdos.TargetPosition = inputPdos.positionActualValue;
    break;
  }
  case cyclicVelocity:
  {
    operationStateFlags = nullOperation;
    break;
  }
  case cyclicTorque:
  {
    outputPdos.modesOfOperation = cyclicTorque;
    outputPdos.TargetTorque = inputPdos.torqueActualValue;
    break;
  }
  default:
    break;
  }
}

void GoldSoloWhistleDrive::CyclicTorqueTransition()
{
  switch (operationStateFlags)
  {
  case cyclicPosition:
  {
    outputPdos.modesOfOperation = cyclicPosition;
    outputPdos.TargetPosition = inputPdos.positionActualValue;
    break;
  }
  case cyclicVelocity:
  {
    outputPdos.modesOfOperation = cyclicVelocity;
    outputPdos.TargetVelocity = inputPdos.velocityActualValue;
    break;
  }
  case cyclicTorque:
  {
    operationStateFlags = nullOperation;
    break;
  }
  default:
    break;
  }
}

GoldSoloWhistleDrive::GoldSoloWhistleDrive(uint8_t thisSlavePosition)
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
  alias = GoldSoloWhistleAlias;
  position = thisSlavePosition;
  vendor_id = GoldSoloWhistleVendor_id;
  product_code = GoldSoloWhistleProduct_code;
  numberOfDomainEntries = GoldSoloWhistleDomainEntries;

  domainRegisters[0] = {alias, position, vendor_id, product_code,
                        controlWordIndex, controlWordSubIndex,
                        &offsetOut.controlWord, NULL};
  domainRegisters[1] = {alias, position, vendor_id, product_code,
                        modesOfOperationIndex, modesOfOperationSubIndex,
                        &offsetOut.modesOfOperation, NULL};
  domainRegisters[2] = {alias, position, vendor_id, product_code,
                        targetTorqueIndex, targetTorqueSubIndex,
                        &offsetOut.targetTorque, NULL};
  domainRegisters[3] = {alias, position, vendor_id, product_code,
                        targetPositionIndex, targetPositionSubIndex,
                        &offsetOut.targetPosition, NULL};
  domainRegisters[4] = {alias, position, vendor_id, product_code,
                        targetVelocityIndex, targetVelocitySubIndex,
                        &offsetOut.targetVelocity, NULL};
  domainRegisters[5] = {alias, position, vendor_id, product_code,
                        statusWordIndex, statusWordSubIndex,
                        &offsetIn.statusWord, NULL};
  domainRegisters[6] = {
    alias, position, vendor_id, product_code, modesOfOperationDisplayIndex,
    modesOfOperationDisplaySubIndex, &offsetIn.modesOfOperationDisplay, NULL};
  domainRegisters[7] = {alias, position, vendor_id, product_code,
                        positionActualValueIndex, positionActualValueSubIndex,
                        &offsetIn.positionActualValue, NULL};
  domainRegisters[8] = {alias, position, vendor_id, product_code,
                        velocityActualValueIndex, velocityActualValueSubIndex,
                        &offsetIn.velocityActualValue, NULL};
  domainRegisters[9] = {alias, position, vendor_id, product_code,
                        torqueActualValueIndex, torqueActualValueSubIndex,
                        &offsetIn.torqueActualValue, NULL};
  domainRegisters[10] = {alias, position, vendor_id, product_code,
                         digitalInputsIndex, digitalInputsSubIndex,
                         &offsetIn.digitalInputs, NULL};
  domainRegisters[11] = {alias, position, vendor_id, product_code,
                         auxiliaryPositionActualValueIndex,
                         auxiliaryPositionActualValueSubIndex,
                         &offsetIn.auxiliaryPositionActualValue, NULL};

  domainRegistersPointer = domainRegisters;
  slavePdoEntriesPointer = slavePdoEntries;
  slavePdosPointer = slavePdos;
  slaveSyncsPointer = slaveSyncs;
  // and stop here, the rest is additional

  stateFlags = nullState;
  state = switchOnDisabled;
  operationStateFlags = nullOperation;
  operationState = cyclicPosition;
}

void GoldSoloWhistleDrive::SetTargetDefaults()
{
  switch (inputPdos.modesOfOperationDisplay)
  {
  case cyclicPosition:
  {
    outputPdos.TargetPosition = inputPdos.positionActualValue;
    break;
  }
  case cyclicVelocity:
  {
    outputPdos.TargetVelocity = inputPdos.velocityActualValue;
    break;
  }
  case cyclicTorque:
  {
    outputPdos.TargetTorque = inputPdos.torqueActualValue;
    break;
  }
  default:
    break;
  }
}

int GoldSoloWhistleDrive::SdoRequests(ec_sdo_request_t* sdoPointer,
                                      ec_slave_config_t* configPointer)
{
  if (!(sdoPointer = ecrt_slave_config_create_sdo_request(
          configPointer, modesOfOperationIndex, modesOfOperationSubIndex,
          cyclicPosition)))
  {
    cout << "Failed to create SDO request." << endl;
    return 1;
  }
  ecrt_sdo_request_timeout(sdoPointer, 500);
  ecrt_slave_config_sdo8(configPointer, modesOfOperationIndex,
                         modesOfOperationSubIndex, cyclicPosition);
  if (!(sdoPointer = ecrt_slave_config_create_sdo_request(
          configPointer, homingMethodIndex, homingMethodSubIndex,
          homingOnPositionMethod)))
  {
    cout << "Failed to create SDO request." << endl;
    return 1;
  }
  ecrt_sdo_request_timeout(sdoPointer, 500);
  ecrt_slave_config_sdo8(configPointer, homingMethodIndex, homingMethodSubIndex,
                         homingOnPositionMethod);
  return 0;
}

void GoldSoloWhistleDrive::LoopFunction() { (this->*stateMachine[state])(); }

void GoldSoloWhistleDrive::ReadInputs()
{
  inputPdos.statusWord = EC_READ_U16(domainDataPointer + offsetIn.statusWord);
  inputPdos.modesOfOperationDisplay =
    EC_READ_S8(domainDataPointer + offsetIn.modesOfOperationDisplay);
  DetermineState();
  inputPdos.positionActualValue =
    EC_READ_S32(domainDataPointer + offsetIn.positionActualValue);
  inputPdos.velocityActualValue =
    EC_READ_S32(domainDataPointer + offsetIn.velocityActualValue);
  inputPdos.torqueActualValue =
    EC_READ_S16(domainDataPointer + offsetIn.torqueActualValue);
  inputPdos.digitalInputs =
    EC_READ_U32(domainDataPointer + offsetIn.digitalInputs);
  inputPdos.auxiliaryPositionActualValue =
    EC_READ_S32(domainDataPointer + offsetIn.auxiliaryPositionActualValue);
  (this->*stateManager[state])();
}

void GoldSoloWhistleDrive::WriteOutputs()
{
  EC_WRITE_U16(domainDataPointer + offsetOut.controlWord,
               static_cast<unsigned short>(outputPdos.controlWord.to_ulong()));
  EC_WRITE_S8(domainDataPointer + offsetOut.modesOfOperation,
              outputPdos.modesOfOperation);
  if (state == operationEnabled || state == switchOn)
  {
    EC_WRITE_S32(domainDataPointer + offsetOut.targetPosition,
                 outputPdos.TargetPosition);
    EC_WRITE_S32(domainDataPointer + offsetOut.targetVelocity,
                 outputPdos.TargetVelocity);
    EC_WRITE_S16(domainDataPointer + offsetOut.targetTorque,
                 outputPdos.TargetTorque);
  }
}
