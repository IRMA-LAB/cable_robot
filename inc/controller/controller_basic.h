#ifndef CONTROLLER_BASIC_H
#define CONTROLLER_BASIC_H

#include "grabcommon.h"
#include "controller/controller_base.h"

class ControllerBasic : public ControllerBase
{
public:
  explicit ControllerBasic(const uint8_t motor_id);

  void SetCableLenTarget(const double target);
  void SetMotorSpeedTarget(const int32_t target);
  void SetMotorTorqueTarget(const int16_t target);

  vect<MotorStatus> CalcCableSetPoint(const grabcdpr::Vars& robot_status) override;

private:
  enum BitPosition
  {
    LENGTH,
    SPEED,
    TORQUE
  };

  Bitfield8 target_flags_;

  double length_target_;
  int32_t speed_target_;
  int16_t torque_target_;
};

#endif // CONTROLLER_BASIC_H
