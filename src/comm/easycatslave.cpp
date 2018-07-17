#include "easycatslave.h"

EasyCatSlave::EasyCatSlave(uint8_t thisSlavePosition)
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
  alias = EasyCatAlias;
  position = thisSlavePosition;
  vendor_id = EasyCatVendor_id;
  product_code = EasyCatProduct_code;
  numberOfDomainEntries = EasyCatDomainEntries;

  domainRegisters[0] = {alias, position, vendor_id, product_code, 0x0005, 0x01,
                        &offsetOut.slaveStatus, NULL};
  domainRegisters[1] = {alias, position, vendor_id, product_code, 0x0005, 0x02,
                        &offsetOut.controlWord, NULL};
  domainRegisters[2] = {alias, position, vendor_id, product_code, 0x0005, 0x03,
                        &offsetOut.ledFrequency, NULL};
  domainRegisters[3] = {alias, position, vendor_id, product_code, 0x0006, 0x01,
                        &offsetIn.slaveState, NULL};
  domainRegisters[4] = {alias, position, vendor_id, product_code, 0x0006, 0x02,
                        &offsetIn.numberOfCalls, NULL};
  domainRegisters[5] = {alias, position, vendor_id, product_code, 0x0006, 0x03,
                        &offsetIn.cycleCounter, NULL};

  domainRegistersPointer = domainRegisters;
  slavePdoEntriesPointer = slavePdoEntries;
  slavePdosPointer = slavePdos;
  slaveSyncsPointer = slaveSyncs;
  // and stop here, the rest is additional

  internalState = idle;
  slaveFlags = idle;
  outputPdos.slaveStatus = operational;
}

EasyCatSlave::~EasyCatSlave()
{
  outputPdos.slaveStatus = notOperational;
  EC_WRITE_U8(domainDataPointer + offsetOut.slaveStatus, outputPdos.slaveStatus);
}

void EasyCatSlave::IdleFun() { outputPdos.controlWord = idle; }

void EasyCatSlave::UpdateSlaveFun()
{
  outputPdos.controlWord = updateSlave;
  temp = inputPdos.numberOfCalls;
}

void EasyCatSlave::IdleTransition()
{
  if (slaveFlags == updateSlave && inputPdos.slaveState == idle)
  {
    slaveFlags = idle;
    internalState = updateSlave;
  }
}

void EasyCatSlave::UpdateSlaveTransition()
{
  if (inputPdos.numberOfCalls > temp)
  {
    internalState = idle;
  }
}

void EasyCatSlave::LoopFunction()
{
  /* This is a simple way to make a state machine work.
   * State manager deals with state changes: something set
   * a change flag (in this case, slaveFlags) and state manager
   * check if the state change is feasible. If so, it changes the state
   * State Machine executes the code associated with the current state
  */
  (this->*stateManager[internalState])();
  (this->*stateMachine[internalState])();
}

void EasyCatSlave::ReadInputs()
{
  // This is the way we can read the Pdos, according to ecrt.h
  inputPdos.slaveState = EC_READ_U8(domainDataPointer + offsetIn.slaveState);
  inputPdos.numberOfCalls = EC_READ_U8(domainDataPointer + offsetIn.numberOfCalls);
  inputPdos.cycleCounter = EC_READ_U8(domainDataPointer + offsetIn.cycleCounter);
}

void EasyCatSlave::WriteOutputs()
{
  // This is the way we can write the Pdos, according to ecrt.h
  EC_WRITE_U8(domainDataPointer + offsetOut.slaveStatus, outputPdos.slaveStatus);
  EC_WRITE_U8(domainDataPointer + offsetOut.controlWord, outputPdos.controlWord);
  EC_WRITE_U8(domainDataPointer + offsetOut.ledFrequency, outputPdos.ledFrequency);
}
