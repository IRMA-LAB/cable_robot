#ifndef EASYCATSLAVE_H
#define EASYCATSLAVE_H

// EasyCAT Slave class. Inherits Ethercat Slave base class

#include "ethercatslave.h"

using namespace std;

class EasyCatSlave : public EthercatSlave
{
private:
  constexpr static uint8_t EasyCatDomainEntries =
    6; // Easycat Slave specific info.
  constexpr static uint8_t EasyCatAlias = 0;
  constexpr static uint32_t EasyCatVendor_id = 0x0000079a;
  constexpr static uint32_t EasyCatProduct_code = 0x00defede;
  constexpr static uint8_t numberOfEasyCatStates =
    2; // Number of state of easycat state machine, according to how we
       // programmed it
  constexpr static uint8_t operational = 1;
  constexpr static uint8_t notOperational = 0;

  uint8_t temp;

  struct OffsetIn
  { // Useful ethercat struct
    unsigned int slaveState;
    unsigned int numberOfCalls;
    unsigned int cycleCounter;
  } offsetIn;

  struct OffsetOut
  { // Useful ethercat struct
    unsigned int slaveStatus;
    unsigned int controlWord;
    unsigned int ledFrequency;
  } offsetOut;

  typedef void (
    EasyCatSlave::*StateFunction)(); // Easyway to implement state machine

  void IdleFun(); // State functions
  void UpdateSlaveFun();
  void IdleTransition(); // State transition functions
  void UpdateSlaveTransition();

public:
  EasyCatSlave(uint8_t thisSlavePosition);
  ~EasyCatSlave();
  enum EasyCatState
  {
    idle = 0,
    updateSlave = 1,
  } internalState,
    slaveFlags; // state machine utilities
  // State machine function array
  StateFunction stateMachine[numberOfEasyCatStates] = {
    &EasyCatSlave::IdleFun, &EasyCatSlave::UpdateSlaveFun};
  // State machine transition function array
  StateFunction stateManager[numberOfEasyCatStates] = {
    &EasyCatSlave::IdleTransition, &EasyCatSlave::UpdateSlaveTransition};

  ec_pdo_entry_reg_t
    domainRegisters[EasyCatDomainEntries]; // ethercat utilities
  ec_pdo_entry_info_t slavePdoEntries[6] = {
    // ethercat utilities, can be retrieved in the xml config file provided by
    // the vendor
    {0x0005, 0x01, 8}, /* Byte0 */
    {0x0005, 0x02, 8}, /* Byte1 */
    {0x0005, 0x03, 8}, /* Byte2 */
    {0x0006, 0x01, 8}, /* Byte0 */
    {0x0006, 0x02, 8}, /* Byte1 */
    {0x0006, 0x03, 8}, /* Byte2 */
  };

  ec_pdo_info_t slavePdos[2] = {
    // ethercat utilities, can be retrieved in the xml config file provided by
    // the vendor
    {0x1600, 3, slavePdoEntries + 0}, /* Outputs */
    {0x1a00, 3, slavePdoEntries + 3}, /* Inputs */
  };

  ec_sync_info_t slaveSyncs[3] = {
    // ethercat utilities, can be retrieved in the xml config file provided by
    // the vendor
    {0, EC_DIR_OUTPUT, 1, slavePdos + 0, EC_WD_ENABLE},
    {1, EC_DIR_INPUT, 1, slavePdos + 1, EC_WD_DISABLE},
    {0xff, (ec_direction_t)0, 0, NULL,
     (ec_watchdog_mode_t)0} // this line has been modified so that the compiler
                            // won't get mad
  };

  // The last 3 array can be obtained by command line, executing sudo
  // /opt/etherlab/bin/./ethercat cstruct

  struct InputPdos
  { // this is a simple way to store the pdos input values
    uint8_t slaveState;
    uint8_t numberOfCalls;
    uint8_t cycleCounter;
  } inputPdos;

  struct OutputPdos
  { // this is a simple way to store the pdos output values
    uint8_t slaveStatus;
    uint8_t controlWord;
    uint8_t ledFrequency;
  } outputPdos;

  virtual void
  LoopFunction(); // The function we are overloading from the base class
  virtual void ReadInputs();
  virtual void WriteOutputs();
};

#endif // EASYCATSLAVE_H
