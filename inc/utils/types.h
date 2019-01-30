#ifndef CABLE_ROBOT_TYPES_H
#define CABLE_ROBOT_TYPES_H

#include <cmath>
#include <stdint.h>
#include <stdlib.h>

/**
 * @brief The MotorStatus struct
 */
struct MotorStatus
{
  MotorStatus() : id(0), motor_position(0), motor_speed(0), motor_torque(0), op_mode(0) {}
  MotorStatus(const id_t _id, const int8_t _op_mode, const int32_t motor_pos,
              const int32_t _motor_speed, const int16_t _motor_torque)
    : id(_id), motor_position(motor_pos), motor_speed(_motor_speed),
      motor_torque(_motor_torque), op_mode(_op_mode)
  {
  }

  id_t id;
  int32_t motor_position;
  int32_t motor_speed;
  int16_t motor_torque;
  int8_t op_mode;
};

/**
 * @brief The WinchStatus struct
 */
struct WinchStatus : MotorStatus
{
  WinchStatus() : aux_position(0), cable_length(0.0) {}
  WinchStatus(const id_t _id, const int8_t _op_mode, const int32_t motor_pos,
              const int32_t _motor_speed, const int16_t _motor_torque,
              const double cable_len, const int aux_pos)
    : MotorStatus(_id, _op_mode, motor_pos, _motor_speed, _motor_torque),
      aux_position(aux_pos), cable_length(cable_len)
  {
  }

  int aux_position;
  double cable_length;
};

/**
 * @brief The ActuatorStatus struct
 */
struct ActuatorStatus : WinchStatus
{
  ActuatorStatus() : pulley_angle(0.0) {}
  ActuatorStatus(const id_t _id, const int8_t _op_mode, const int32_t motor_pos,
                 const int32_t _motor_speed, const int16_t _motor_torque,
                 const double cable_len, const int aux_pos, const double pulley_ang)
    : WinchStatus(_id, _op_mode, motor_pos, _motor_speed, _motor_torque, cable_len,
                  aux_pos),
      pulley_angle(pulley_ang)
  {
  }

  uint8_t state;  /**< see Actuator::States */
  double pulley_angle;
};

#endif // CABLE_ROBOT_TYPES_H
