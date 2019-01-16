#ifndef COMMON_H
#define COMMON_H

#include <iostream>

enum Status: uint8_t
{
  SET = 1,
  RESET = 0,
  OPERATIONAL = 1,
  NOT_OPERATIONAL = 0,
};

enum StatusBit: uint8_t
{
  readyToSwitchOnStatusBit = 0,
  switchOnStatusBit = 1,
  enabledStatusBit = 2,
  faultStatusBit = 3,
  onStatusBit = 5,
  offStatusBit = 6
};

enum ControlBit: uint8_t
{
  switchOnControlBit = 0,
  enableVoltageControlBit = 1,
  quickStopControlBit = 2,
  enableControlBit = 3,
  faultResetControlBit = 7,
};

#endif // COMMON_H
