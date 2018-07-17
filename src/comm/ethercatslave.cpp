#include "ethercatslave.h"

EthercatSlave::~EthercatSlave() {}

void EthercatSlave::Init(uint8_t* domain_data_ptr_ext)
{
  domain_data_ptr_ = domain_data_ptr_ext;
}

int EthercatSlave::SdoRequests(ec_sdo_request_t* sdo_ptr, ec_slave_config_t* config_ptr)
{
  if (sdo_ptr != NULL)
    return 1;
  if (config_ptr == NULL)
    return 1;
  return 0;
}
