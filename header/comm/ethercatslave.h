#ifndef ETHERCATSLAVE_H
#define ETHERCATSLAVE_H

/* Ethercat Slave interface. This class is used as base class  for any slave. All the
   effort can be put in the design of our specific slave, with in mind that the ethercat
   slave interface requires to overload some function. These functions are the ones marked
   as virtual.
*/

#include "ecrt.h"

class EthercatSlave
{
public:
  EthercatSlave() {}
  virtual ~EthercatSlave() = 0;

  uint8_t num_domain_entries_; // Ethercat utility
  uint16_t alias_;                // Ethercat utility
  uint16_t position_;             // Ethercat utility
  uint32_t vendor_id_;            // Ethercat utility
  uint32_t product_code_;         // Ethercat utility
  uint8_t id_;                    // Ethercat utility

  ec_pdo_entry_reg_t* domain_registers_ptr_;  // Ethercat utility
  ec_pdo_entry_info_t* slave_pdo_entries_ptr_; // Ethercat utility
  ec_pdo_info_t* slave_pdos_ptr_;             // Ethercat utility
  ec_sync_info_t* slave_sync_ptr_;           // Ethercat utility

  uint8_t* domain_data_ptr_; // Ethercat utility

  // Function to overload in the slave, it gets called before the cyclical task
  // begins
  virtual void Init(uint8_t* domain_data_ptr_ext);
  virtual void LoopFunction() = 0; // slave main function
  virtual void ReadInputs() = 0;   // to specify what to read
  virtual void WriteOutputs() = 0; // to specify what to write

  virtual int SdoRequests(ec_sdo_request_t* sdo_ptr, ec_slave_config_t* config_ptr);
};

#endif // ETHERCATSLAVE_H
