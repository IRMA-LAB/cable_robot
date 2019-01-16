#ifndef CONTROLLER_BASE_H
#define CONTROLLER_BASE_H

#include <vector>
#include <stdint.h>

#include "libcdpr/inc/types.h"
#include "libgrabec/inc/slaves/goldsolowhistledrive.h"

#include "utils/types.h"

template <typename T> using vect = std::vector<T>;

enum ControlMode: uint8_t
{
  MOTOR_POSITION,
  MOTOR_SPEED,
  MOTOR_TORQUE,
  CABLE_LENGTH,
  NONE
};

struct ControlAction
{
    // Set points
    double cable_length;
    int32_t motor_position;
    int32_t motor_speed;
    int16_t motor_torque;
    // Characteristic
    ID_t motor_id;
    ControlMode ctrl_mode;
};

class ControllerBase
{
public:
  ControllerBase() {}
  explicit ControllerBase(const ID_t motor_id);
  explicit ControllerBase(const vect<ID_t>& motors_id);
  virtual ~ControllerBase();

  void SetMotorID(const ID_t motor_id);
  void SetMotorsID(const vect<ID_t>& motors_id);
  void SetMode(const ControlMode mode);
  void SetMode(const ID_t motor_id, const ControlMode mode);

  vect<ID_t> GetMotorsID() const { return motors_id_; }
  ControlMode GetMode(const ID_t motor_id) const;
  vect<ControlMode> GetModes() const { return modes_; }

  virtual vect<ControlAction> CalcCableSetPoint(const grabcdpr::Vars& robot_status) = 0;

protected:
  vect<ID_t> motors_id_;
  vect<ControlMode> modes_;
};

#endif // CONTROLLER_BASE_H
