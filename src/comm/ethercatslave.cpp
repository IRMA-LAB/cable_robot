#include "ethercatslave.h"

EthercatSlave::~EthercatSlave() {}

void EthercatSlave::Init(uint8_t* domainDataPointerExt)
{
  domainDataPointer = domainDataPointerExt;
}

int EthercatSlave::SdoRequests(ec_sdo_request_t* sdoPointer,
                               ec_slave_config_t* configPointer)
{
  if (sdoPointer != NULL)
    return 1;
  if (configPointer == NULL)
    return 1;
  return 0;
}
