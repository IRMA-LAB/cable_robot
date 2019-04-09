#ifndef WINCH_TORQUE_CONTROLLER_H
#define WINCH_TORQUE_CONTROLLER_H

#include "libcdpr/inc/types.h"

#include "utils/types.h"

class WinchTorqueControl
{
 public:
  explicit WinchTorqueControl(const id_t id, const grabcdpr::ActuatorParams& params);

  short calcServoTorqueSetpoint(const ActuatorStatus& status, const short target);

  id_t id() const { return id_; }

 private:
  id_t id_;
};

class WinchesTorqueControl
{
 public:
  WinchesTorqueControl(const vect<grabcdpr::ActuatorParams>& params);

  WinchTorqueControl& operator[](const id_t id);

 private:
  vect<WinchTorqueControl> controllers_;
};

#endif // WINCH_TORQUE_CONTROLLER_H
