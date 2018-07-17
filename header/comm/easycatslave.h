#ifndef EASYCATSLAVE_H
#define EASYCATSLAVE_H

// EasyCAT Slave class. Inherits Ethercat Slave base class

#include "ethercatslave.h"

class EasyCatSlave : public EthercatSlave
{
private:
  constexpr static uint8_t kEasyCatDomainEntries_ = 6; // Easycat Slave specific info.
  constexpr static uint8_t kEasyCatAlias_ = 0;
  constexpr static uint32_t kEasyCatVendorID_ = 0x0000079a;
  constexpr static uint32_t kEasyCatProductCode_ = 0x00defede;
  constexpr static uint8_t kNumEasyCatStates_ = 2;
  constexpr static uint8_t kOperational_ = 1;
  constexpr static uint8_t kNotOperational_ = 0;

  uint8_t temp_;

  struct OffsetIn
  { // Useful ethercat struct
    unsigned int slave_state;
    unsigned int num_calls;
    unsigned int cycle_counter;
  } offset_in_;

  struct OffsetOut
  { // Useful ethercat struct
    unsigned int slave_status;
    unsigned int control_word;
    unsigned int led_frequency;
  } offset_out_;

  typedef void (EasyCatSlave::*StateFunction)(); // Easyway to implement state machine

  void IdleFun(); // State functions
  void UpdateSlaveFun();
  void IdleTransition(); // State transition functions
  void UpdateSlaveTransition();

public:
  EasyCatSlave(uint8_t slave_position);
  ~EasyCatSlave();
  enum EasyCatState
  {
    idle = 0,
    updateSlave = 1,
  } internal_state_,
    slave_flags_; // state machine utilities
  // State machine function array
  StateFunction state_machine_[kNumEasyCatStates_] = {&EasyCatSlave::IdleFun,
                                                       &EasyCatSlave::UpdateSlaveFun};
  // State machine transition function array
  StateFunction state_manager_[kNumEasyCatStates_] = {
    &EasyCatSlave::IdleTransition, &EasyCatSlave::UpdateSlaveTransition};

  ec_pdo_entry_reg_t domain_registers_[kEasyCatDomainEntries_]; // ethercat utilities
  ec_pdo_entry_info_t slave_pdo_entries_[6] = {
    // ethercat utilities, can be retrieved in the xml config file provided by the vendor
    {0x0005, 0x01, 8}, /* Byte0 */
    {0x0005, 0x02, 8}, /* Byte1 */
    {0x0005, 0x03, 8}, /* Byte2 */
    {0x0006, 0x01, 8}, /* Byte0 */
    {0x0006, 0x02, 8}, /* Byte1 */
    {0x0006, 0x03, 8}, /* Byte2 */
  };

  ec_pdo_info_t slave_pdos_[2] = {
    // ethercat utilities, can be retrieved in the xml config file provided by the vendor
    {0x1600, 3, slave_pdo_entries_ + 0}, /* Outputs */
    {0x1a00, 3, slave_pdo_entries_ + 3}, /* Inputs */
  };

  ec_sync_info_t slave_syncs_[3] = {
    // ethercat utilities, can be retrieved in the xml config file provided by
    // the vendor
    {0, EC_DIR_OUTPUT, 1, slave_pdos_ + 0, EC_WD_ENABLE},
    {1, EC_DIR_INPUT, 1, slave_pdos_ + 1, EC_WD_DISABLE},
    {0xff, static_cast<ec_direction_t>(0), 0, NULL,
     static_cast<ec_watchdog_mode_t>(
       0)} // this line has been modified so that the compiler
           // won't get mad
  };

  // The last 3 array can be obtained by command line, executing sudo
  // /opt/etherlab/bin/./ethercat cstruct

  struct InputPdos
  { // this is a simple way to store the pdos input values
    uint8_t slave_state;
    uint8_t num_calls;
    uint8_t cycle_counter;
  } input_pdos_;

  struct OutputPdos
  { // this is a simple way to store the pdos output values
    uint8_t slave_status;
    uint8_t control_word;
    uint8_t led_frequency;
  } output_pdos_;

  virtual void LoopFunction(); // The function we are overloading from the base class
  virtual void ReadInputs();
  virtual void WriteOutputs();
};

#endif // EASYCATSLAVE_H
