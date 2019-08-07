/**
 * @file controller_singledrive.h
 * @author Simone Comari
 * @date 08 Jul 2019
 * @brief File containing a simple single drive controller class for cable robot.
 */

#ifndef CABLE_ROBOT_CONTROLLER_SINGLEDRIVE_H
#define CABLE_ROBOT_CONTROLLER_SINGLEDRIVE_H

#include <bitset>

#include "libs/grab_common/pid/pid.h"

#include "ctrl/controller_base.h"

/**
 * @brief A simple single drive control class for cable robot.
 *
 * This simple controller provides control action for a single drive at a time and is used
 * by both the direct drive control panel and simple operations in single cable control,
 * such as during the homing procedure.
 */
class ControllerSingleDrive: public ControllerBase
{
 public:
  /**
   * @brief ControllerSingleDrive targetless constructor.
   * @param[in] period_nsec Controller sample period in nanoseconds.
   */
  ControllerSingleDrive(const uint32_t period_nsec);
  /**
   * @brief ControllerSingleDrive full constructor.
   * @param[in] motor_id ID of single motor to be controlled.
   * @param[in] period_nsec Controller sample period in nanoseconds.
   */
  ControllerSingleDrive(const id_t motor_id, const uint32_t period_nsec);

  /**
   * @brief Set cable length target in meters.
   * @param[in] target Cable length target in meters.
   * @note This target is effective if and only if ControlMode::CABLE_LENGTH is set.
   * Moreover all previous targets are cleared when calling this functions.
   */
  void SetCableLenTarget(const double target);
  /**
   * @brief Set motor position target, in encoder absolute counts.
   * @param[in] target Motor position target, in encoder absolute counts.
   * @param[in] apply_traj If _true_, a polynomial trajectory of 5th order is applied to
   * move from current position to target one with zero initial and final velocity and
   * acceleration.
   * @param[in] time Time of trajectory execution. If not given, this value is computed
   * according to the maximum delta counts that the motor can safely handle in a single
   * cycle.
   * @note This target is effective if and only if ControlMode::MOTOR_POSITION is set.
   * Moreover all previous targets are cleared when calling this functions.
   */
  void SetMotorPosTarget(const int32_t target, const bool apply_traj = true,
                         const double time = -1.0);
  /**
   * @brief Set motor speed target, in encoder counts/second.
   * @param[in] target Motor speed target, in encoder counts/second.
   * @note This target is effective if and only if ControlMode::MOTOR_SPEED is set.
   * Moreover all previous targets are cleared when calling this functions.
   */
  void SetMotorSpeedTarget(const int32_t target);
  /**
   * @brief Set motor torque target, in per thousand nominal points.
   * @param[in] target Motor torque target, in per thousand nominal points.
   * @note This target is effective if and only if ControlMode::MOTOR_TORQUE is set.
   * Moreover all previous targets are cleared when calling this functions.
   */
  void SetMotorTorqueTarget(const int16_t target);

  /**
   * @brief Set motor torque steady-state error tolerance, in per thousand nominal points.
   *
   * Motor torque steady-state error tolerance is the accepted margin to consider a torque
   * target reached, by taking into account the white noise on the measured values.
   * @param[in] tol Motor torque steady-state error tolerance, in per thousand points.
   */
  void SetMotorTorqueSsErrTol(const int16_t tol) { torque_ss_err_tol_ = tol; }

  /**
   * @brief Get cable length target in meters.
   * @return Cable length target in meters.
   */
  double GetCableLenTarget() const { return length_target_; }
  /**
   * @brief Get motor position target, in encoder absolute counts.
   * @return Motor position target, in encoder absolute counts.
   */
  int32_t GetMotorPosTarget() const { return pos_target_true_; }
  /**
   * @brief Get motor speed target, in encoder counts/second.
   * @return Motor speed target, in encoder counts/second.
   */
  int32_t GetMotorSpeedTarget() const { return speed_target_true_; }
  /**
   * @brief Get motor torque target, in per thousand points.
   * @return Motor torque target, in per thousand points.
   */
  int16_t GetMotorTorqueTarget() const { return torque_target_true_; }

  /**
   * @brief Set cable length trajectory.
   * @param trajectory A trajectory vector where each waypoint is equally spaced at 1ms
   * from the next one.
   */
  void SetCableLenTrajectory(const std::vector<double>& trajectory);
  /**
   * @brief Change cable length increment.
   *
   * Cable length increment is used in direct drive manual control from the main GUI of
   * cable robot app.
   * The increment is applied as long as the plus/minus button is
   * pressed, so that at every cycle of the controller the winch rolls/unrolls the cable
   * by a small amount.
   * @param[in] active When _true_, a small increment to the targeted drive position is
   * applied at every cycle.
   * @param[in] sign The direction of the increment. POS unrolls the cable, NEG rolls it.
   * @param[in] micromove If _true_ a "micro" increment is applied (5 mm/s), a larger one
   * (2 cm/s) otherwise.
   */
  void CableLenIncrement(const bool active, const Sign sign = Sign::POS,
                         const bool micromove = true);
  /**
   * @brief Scale motor speed.
   *
   * Speed scale is directly linked to the slider present in direct drive manual control
   * on the main GUI of cable robot app.
   * This scaling factor set motor's speed in a range from -800000 to +800000 counts per
   * seconds.
   * @param[in] scale A scaling factor, from -1.0 to 1.0.
   */
  void ScaleMotorSpeed(const double scale);
  /**
   * @brief Change motor torque increment.
   *
   * Motor torque increment is used in direct drive manual control from the main GUI of
   * cable robot app.
   * The increment is applied as long as the plus/minus button is pressed, so that at
   * every cycle of the controller the winch pulls/pushes the cable by a small amount.
   * @param[in] active When _true_, a small increment to the targeted drive torque is
   * applied at every cycle.
   * @param[in] sign The direction of the increment. POS pushes the cable, NEG pulls it.
   */
  void MotorTorqueIncrement(const bool active, const Sign sign = Sign::POS);

  /**
   * @brief Check if active target is reached, independently from the control mode.
   * @return _True_ if target is reached, _false_ otherwise.
   */
  bool targetReached() const override { return modes_.empty() ? false : on_target_; }

  /**
   * @brief Calculate control actions depending on current robot status.
   *
   * This is the main method of this class, which is called at every cycle of the real
   * time thread.
   * Given current robot configuration, possibly resulting from a state estimator, and
   * actuators status provided by proprioceptive sensors, the control action for the
   * single targeted motor is computed.
   * In particular:
   * - cable length control is applied directly with a continuous synchronous increment
   * according to the user inputs in the direct drive control panels;
   * - motor position control follows a smooth 5th order polynomial trajectory;
   * - motor speed is applied directly once scaled by a factor specified by the user in
   * the same panel;
   * - motor torque target is filtered through a PI controller before being assigned to
   * the end drive to avoid aggressive, possibly unfeasible deltas.
   * @param[in] robot_status Cable robot status, in terms of platform configuration.
   * @param[in] actuators_status Actuators status, in terms of drives, winches, pulleys
   * and cables configuration.
   * @return Control actions for each targeted motor.
   */
  vect<ControlAction>
  calcCtrlActions(const grabcdpr::RobotVars& robot_status,
                  const vect<ActuatorStatus>& actuators_status) override final;

 private:
  static constexpr double kAbsDeltaLengthMicroPerSec_ = 0.005;     // [m/s]
  static constexpr double kAbsDeltaLengthPerSec_      = 0.02;      // [m/s]
  static constexpr int32_t kMaxPos_                   = 12000000;  // [counts]
  static constexpr int32_t kMinPos_                   = -16500000; // [counts]
  static constexpr int32_t kDefaultPosSsErrTol_       = 5;         // [counts]
  static constexpr int32_t kAbsMaxSpeed_              = 4000000;   // [counts/s]
  static constexpr int16_t kAbsDeltaTorquePerSec_     = 20;        // [nominal points]
  static constexpr int16_t kAbsMaxTorque_             = 800;       // [nominal points]
  static constexpr int16_t kDefaultTorqueSsErrTol_    = 5;         // [nominal points]

  enum BitPosition
  {
    LENGTH,
    POSITION,
    SPEED,
    TORQUE
  };

  double period_sec_;
  std::bitset<4> target_flags_;
  bool on_target_;

  double length_target_;
  int32_t pos_target_true_;
  int32_t speed_target_true_;
  int16_t torque_target_true_;
  double pos_target_;
  double speed_target_;
  double torque_target_;

  int32_t pos_ss_err_tol_;
  int16_t torque_ss_err_tol_;

  bool change_length_target_;
  bool change_speed_target_;
  bool change_torque_target_;

  double abs_delta_length_;
  double delta_length_;
  double abs_delta_speed_;
  double delta_speed_;
  double abs_delta_torque_;
  double delta_torque_;

  PID torque_pid_;
  const ParamsPID torque_pid_params_ = {0.0263, 15.847,         0.,
                                        0.,     kAbsMaxTorque_, -kAbsMaxTorque_};

  double traj_time_; /**< [sec] */
  bool new_trajectory_ = false;
  bool apply_trajectory_;

  vect<double> cable_len_traj_;

  int32_t CalcMotorPos(const vect<ActuatorStatus>& actuators_status);
  int16_t CalcMotorTorque(const vect<ActuatorStatus>& actuators_status);

  int32_t CalcPoly5Waypoint(const int32_t q, const int32_t q_final, const int32_t max_dq);

  double GetTrajectoryPoint();

  void Clear();
};

#endif // CABLE_ROBOT_CONTROLLER_SINGLEDRIVE_H
