/**
 * @file controller_base.h
 * @author Simone Comari
 * @date 22 Mar 2019
 * @brief File containing the base class of cable robot controller.
 */

#ifndef CABLE_ROBOT_CONTROLLER_BASE_H
#define CABLE_ROBOT_CONTROLLER_BASE_H

#include <stdint.h>

#include "libcdpr/inc/types.h"
#include "libgrabec/inc/slaves/goldsolowhistledrive.h"

#include "utils/types.h"


/**
 * @brief The possible control modes of any controller derived from ControllerBase.
 *
 * It reflects the possible control modes of the physical drive. Note that both
 * MOTOR_POSITION and CABLE_LENGTH in fact end up in drive position control, but in the
 * latter case the position target depends on the cable-to-counts conversion and homing
 * results.
 */
enum ControlMode : uint8_t
{
  MOTOR_POSITION,
  MOTOR_SPEED,
  MOTOR_TORQUE,
  CABLE_LENGTH,
  NONE
};

/**
 * @brief The control action structure, including target drive information and set point.
 */
struct ControlAction
{
  // Set points
  double cable_length; /**< cable length set point, used if ctrl_mode = CABLE_LENGTH */
  int32_t
    motor_position; /**< motor position set point, used if ctrl_mode = MOTOR_POSITION */
  int32_t motor_speed;  /**< motor speed set point, used if ctrl_mode = MOTOR_SPEED */
  int16_t motor_torque; /**< motor torque set point, used if ctrl_mode = MOTOR_TORQUE */
  // Characteristic
  id_t motor_id; /**< The ID of the target motor of this control action */
  ControlMode ctrl_mode = ControlMode::NONE; /**< The control mode for the target motor */
};

/**
 * @brief The abstract base class for any cable robot controller.
 *
 * The controller is called at every cycle of the real time thread of cable robot and
 * provides commands to the motors, if present and its output is valid. Any controller is
 * a derived class of this abstract class, which provides the virtual API used and called
 * there.
 * Make sure that the computational time of your new controller stays largely within 1ms
 * to have some margin for other cyclic operations. Because the controller is a shared
 * pointer between threads, be sure to lock the robot mutex when accessing it from
 * outside. Any controller is characterized by a set of targeted motors id and their
 * relative control mode at drive level.
 */
class ControllerBase
{
 public:
  /**
   * @brief ControllerBase default constructor.
   */
  ControllerBase() {}
  /**
   * @brief ControllerBase constructor.
   * @param[in] motor_id ID of the motor to be controlled, used for single drive control.
   */
  explicit ControllerBase(const id_t motor_id);
  /**
   * @brief ControllerBase constructor.
   * @param[in] motors_id IDs of the motors to be controlled, used for multi drive
   * control.
   */
  explicit ControllerBase(const vect<id_t>& motors_id);
  virtual ~ControllerBase();

  /**
   * @brief Set the ID of the motor to be controlled in single drive control.
   * @param[in] motor_id ID of the motor to be controlled, used for single drive control.
   * @note This operation clears and resets the controller mode, which is initialized by
   * default to _ControlMode::NONE_.
   */
  void SetMotorID(const id_t motor_id);
  /**
   * @brief Set the IDs of the motors to be controlled in multi drive control.
   * @param[in] motors_id IDs of the motors to be controlled, used for multi drive
   * control.
   * @note This operation clears and resets any controller modes, which are initialized by
   * default to _ControlMode::NONE_.
   */
  void SetMotorsID(const vect<id_t>& motors_id);
  /**
   * @brief Set the control mode of all targeted motors.
   * @param[in] mode The new control mode to be applied.
   */
  void SetMode(const ControlMode mode);
  /**
   * @brief Set the control mode of a single motor.
   * @param[in] motor_id The ID of the motor whose control mode is to be changed.
   * @param[in] mode The new control mode to be applied.
   */
  void SetMode(const id_t motor_id, const ControlMode mode);

  /**
   * @brief Get IDs of currently controlled motors.
   * @return IDs of currently controlled motors.
   */
  vect<id_t> GetMotorsID() const { return motors_id_; }
  /**
   * @brief Get control mode of inquired motor.
   * @param[in] motor_id The ID of the inquired motor.
   * @return The control mode of the inquired motor.
   */
  ControlMode GetMode(const id_t motor_id) const;
  /**
   * @brief Get control mode of all currently controlled motors.
   * @return The control mode of all currently controlled motors.
   */
  vect<ControlMode> GetModes() const { return modes_; }

  /**
   * @brief Calculate control actions depending on current robot status.
   *
   * This is the main method of this class, which is called at every cycle of the real
   * time thread and needs to be overridden by any derived class.
   * Given current robot configuration, possibly resulting from a state estimator, and
   * actuators status provided by proprioceptive sensors, control actions for each
   * targeted motor are computed.
   * @param[in] robot_status Cable robot status, in terms of platform configuration.
   * @param[in] actuators_status Actuators status, in terms of drives, winches, pulleys
   * and cables configuration.
   * @return Control actions for each targeted motor.
   */
  virtual vect<ControlAction>
  CalcCtrlActions(const grabcdpr::Vars& robot_status,
                  const vect<ActuatorStatus>& actuators_status) = 0;

  /**
   * @brief Check if control target was reached.
   * @return _True_ if control target was reached, _false_ otherwise.
   */
  virtual bool TargetReached() const = 0;

 protected:
  vect<id_t> motors_id_;    /**< IDs of the motors to be controlled. */
  vect<ControlMode> modes_; /**< Control modes of each motor. */
};

#endif // CABLE_ROBOT_CONTROLLER_BASE_H
