#include "easycatslave.h"

EasyCatSlave::EasyCatSlave(uint8_t slave_position)
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
  alias_ = kEasyCatAlias_;
  position_ = slave_position;
  vendor_id_ = kEasyCatVendorID_;
  product_code_ = kEasyCatProductCode_;
  num_domain_entries_ = kEasyCatDomainEntries_;

  domain_registers_[0] = {alias_, position_, vendor_id_, product_code_, 0x0005, 0x01,
                          &offset_out_.slave_status, NULL};
  domain_registers_[1] = {alias_, position_, vendor_id_, product_code_, 0x0005, 0x02,
                          &offset_out_.control_word, NULL};
  domain_registers_[2] = {alias_, position_, vendor_id_, product_code_, 0x0005, 0x03,
                          &offset_out_.led_frequency, NULL};
  domain_registers_[3] = {alias_, position_, vendor_id_, product_code_, 0x0006, 0x01,
                          &offset_in_.slave_state, NULL};
  domain_registers_[4] = {alias_, position_, vendor_id_, product_code_, 0x0006, 0x02,
                          &offset_in_.num_calls, NULL};
  domain_registers_[5] = {alias_, position_, vendor_id_, product_code_, 0x0006, 0x03,
                          &offset_in_.cycle_counter, NULL};

  domain_registers_ptr_ = domain_registers_;
  slave_pdo_entries_ptr_ = slave_pdo_entries_;
  slave_pdos_ptr_ = slave_pdos_;
  slave_sync_ptr_ = slave_syncs_;
  // and stop here, the rest is additional

  internal_state_ = idle;
  slave_flags_ = idle;
  output_pdos_.slave_status = kOperational_;
}

EasyCatSlave::~EasyCatSlave()
{
  output_pdos_.slave_status = kNotOperational_;
  EC_WRITE_U8(domain_data_ptr_ + offset_out_.slave_status, output_pdos_.slave_status);
}

void EasyCatSlave::IdleFun() { output_pdos_.control_word = idle; }

void EasyCatSlave::UpdateSlaveFun()
{
  output_pdos_.control_word = updateSlave;
  temp_ = input_pdos_.num_calls;
}

void EasyCatSlave::IdleTransition()
{
  if (slave_flags_ == updateSlave && input_pdos_.slave_state == idle)
  {
    slave_flags_ = idle;
    internal_state_ = updateSlave;
  }
}

void EasyCatSlave::UpdateSlaveTransition()
{
  if (input_pdos_.num_calls > temp_)
  {
    internal_state_ = idle;
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
  (this->*state_manager_[internal_state_])();
  (this->*state_machine_[internal_state_])();
}

void EasyCatSlave::ReadInputs()
{
  // This is the way we can read the Pdos, according to ecrt.h
  input_pdos_.slave_state = EC_READ_U8(domain_data_ptr_ + offset_in_.slave_state);
  input_pdos_.num_calls = EC_READ_U8(domain_data_ptr_ + offset_in_.num_calls);
  input_pdos_.cycle_counter = EC_READ_U8(domain_data_ptr_ + offset_in_.cycle_counter);
}

void EasyCatSlave::WriteOutputs()
{
  // This is the way we can write the Pdos, according to ecrt.h
  EC_WRITE_U8(domain_data_ptr_ + offset_out_.slave_status, output_pdos_.slave_status);
  EC_WRITE_U8(domain_data_ptr_ + offset_out_.control_word, output_pdos_.control_word);
  EC_WRITE_U8(domain_data_ptr_ + offset_out_.led_frequency, output_pdos_.led_frequency);
}
