/**
 * @file joints_pvt_app.h
 * @author Simone Comari
 * @date 10 Jan 2020
 * @brief This file includes the implementation of winch controllers.
 */

#ifndef CABLE_ROBOT_WINCH_TORQUE_CONTROLLER_H
#define CABLE_ROBOT_WINCH_TORQUE_CONTROLLER_H

#include "libcdpr/inc/cdpr_types.h"

#include "utils/types.h"

/**
 * @brief The controller of a single winch.
 *
 * This controller takes into account the friction given by the pulleys systems and
 * regulate the torque consequently.
 */
class WinchTorqueControl
{
 public:
  /**
   * @brief Constructor.
   * @param id Winch ID.
   * @param params Parameters of the actuator to which this winch belongs.
   */
  explicit WinchTorqueControl(const id_t id, const grabcdpr::ActuatorParams& params);

  /**
   * @brief Calculate servo torque setpoint given current actuator status and desired
   * target.
   * @param status Current actuator status.
   * @param target Desired torque target in N/m.
   * @return The adjusted torque setpoint in nominal points to achieve desired target.
   * @warning Currently this function is dummy, so output = target in.
   * @todo this
   */
  short calcServoTorqueSetpoint(const ActuatorStatus& status, const short target);

  /**
   * @brief Returns winch ID.
   * @return Winch ID.
   */
  id_t id() const { return id_; }

 private:
  id_t id_;
};


/**
 * @brief The controller of multiple winches.
 *
 * This controller takes into account the frictions given by each pulleys systems and
 * regulate the torques consequently.
 */
class WinchesTorqueControl
{
 public:
  /**
   * @brief Constructor.
   * @param params Parameters set for each actuator to which the winches belongs.
   */
  WinchesTorqueControl(const vect<grabcdpr::ActuatorParams>& params);

  /**
   * @brief operator []
   * @param id Winch ID
   * @return The corresponding single winch controller.
   */
  WinchTorqueControl& operator[](const id_t id);

 private:
  vect<WinchTorqueControl> controllers_;
};

#endif // CABLE_ROBOT_WINCH_TORQUE_CONTROLLER_H
