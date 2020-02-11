/**
 * @file winch_torque_controller.cpp
 * @author Simone Comari
 * @date 03 Jul 2019
 * @brief This file includes definitions of classes present in winch_torque_controller.h.
 */

#include "ctrl/winch_torque_controller.h"

WinchTorqueControl::WinchTorqueControl(const id_t id,
                                       const grabcdpr::ActuatorParams& /*params*/)
  : id_(id)
{}

short WinchTorqueControl::calcServoTorqueSetpoint(const ActuatorStatus& /*status*/,
                                                  const short target)
{
  return target; // dummy
}

WinchesTorqueControl::WinchesTorqueControl(const vect<grabcdpr::ActuatorParams>& params)
{
  for (id_t i = 0; i < params.size(); i++)
  {
    if (!params[i].active)
      continue;
    controllers_.push_back(WinchTorqueControl(i, params[i]));
  }
}

WinchTorqueControl& WinchesTorqueControl::operator[](const id_t id)
{
  for (WinchTorqueControl& controller : controllers_)
    if (controller.id() == id)
      return controller;
  throw std::out_of_range("Index out of range");
}
