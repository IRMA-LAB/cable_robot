/**
 * @file controller_joints_pvt.h
 * @author Simone Comari
 * @date 27 Aug 2019
 * @brief This file includes the implementation of the controller for the joints pvt app.
 */

#ifndef CABLE_ROBOT_CONTROLLER_JOINTS_PVT_H
#define CABLE_ROBOT_CONTROLLER_JOINTS_PVT_H

#include "easylogging++.h"

#include "ctrl/controller_base.h"
#include "ctrl/winch_torque_controller.h"

/**
 * @brief The controller for joints pvt app.
 *
 * This controller can take four different types of trajectories as input:
 * 0. Cable lenghts [m]
 * 1. Motor positions [counts]
 * 2. Motor velocities [counts/s]
 * 3. Motor torques [nominal points]
 * Disregarding the type, each given set must include one trajectory per active motor.
 * If trajectories are valid, upon each call of CalcCtrlActions() the point next in line
 * in the trajectory is read and used as next setpoint for the motor.
 *
 * The trajectory following can be pause, resumed and stopped at any time. When stopping
 * or resuming time is warped to smooth out the arrest/start up phase and avoid abrubt
 * accelerations at motors level.
 *
 * While executing a trajectory, the progress status is emitted at a constant rate (5Hz).
 * Once trajectory is completed a trajectoryCompleted() signal is emitted.
 */
class ControllerJointsPVT: public QObject, public ControllerBase
{
  Q_OBJECT

 public:
  /**
   * @brief Full constructor.
   * @param[in] params A vector of actuator parameters, with as many elements as the
   * active motors.
   * @param cycle_t_nsec Real-time thread cycle time in nanoseconds.
   * @param parent The optional parent QObject.
   */
  explicit ControllerJointsPVT(const vect<grabcdpr::ActuatorParams>& params,
                               const uint32_t cycle_t_nsec, QObject* parent = nullptr);
  ~ControllerJointsPVT() override {}

  /**
   * @brief Set trajectories of cable length type.
   * @param trajectories Trajectories of cable length type [m].
   * @return _True_ if trajectories are valid, _False_ otherwise.
   */
  bool setCablesLenTrajectories(const vect<TrajectoryD>& trajectories);
  /**
   * @brief Set trajectories of motor position type.
   * @param trajectories Trajectories of motor position type [counts].
   * @return _True_ if trajectories are valid, _False_ otherwise.
   */
  bool setMotorsPosTrajectories(const vect<TrajectoryI>& trajectories);
  /**
   * @brief Set trajectories of motor velocity type.
   * @param trajectories Trajectories of motor velocity type [counts/s].
   * @return _True_ if trajectories are valid, _False_ otherwise.
   */
  bool setMotorsVelTrajectories(const vect<TrajectoryI>& trajectories);
  /**
   * @brief Set trajectories of motor torque type.
   * @param trajectories Trajectories of motor torque type [nominal points].
   * @return _True_ if trajectories are valid, _False_ otherwise.
   */
  bool setMotorsTorqueTrajectories(const vect<TrajectoryS>& trajectories);

  /**
   * @brief Pause trajectory following with a smooth arrest.
   */
  void pauseTrajectoryFollowing();
  /**
   * @brief Resume trajectory following with a smooth start up.
   */
  void resumeTrajectoryFollowing();
  /**
   * @brief Stop trajectory following with a smooth arrest.
   */
  void stopTrajectoryFollowing();

  /**
   * @brief Check if a stop or resume request is pending.
   * @return _True_ if a request is pending, _False_ otherwise.
   */
  bool requestPending() const { return stop_request_ || resume_request_; }
  /**
   * @brief Check if trajectory following is paused.
   * @return _True_ if trajectory following, _False_ otherwise.
   */
  bool isPaused() const { return (stop_ || stop_request_) && !resume_request_; }
  /**
   * @brief Check if active target is reached, independently from the control mode.
   * @return _True_ if target is reached, _false_ otherwise.
   */
  bool TargetReached() const override final { return stop_; }

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
  CalcCtrlActions(const grabcdpr::Vars& robot_status,
                  const vect<ActuatorStatus>& actuators_status) override final;

 signals:
  /**
   * @brief Signal to notice that the trajectory has benn completed.
   */
  void trajectoryCompleted() const;
  /**
   * @brief Signal carrying trajectory progress status.
   */
  void trajectoryProgressStatus(const int, const double) const;

 private:
  static constexpr double kMinArrestTime_       = 1.0;     // [sec]
  static constexpr double kVel2ArrestTimeRatio_ = 1500000; // [counts/sec^2]
  static constexpr short kTorqueStopValue_ = -300; // [nominal points]

  enum BitPosition
  {
    LENGTH,
    POSITION,
    SPEED,
    TORQUE
  };

  std::bitset<4> target_flags_;

  double cycle_time_;     // [sec]
  double traj_time_;      // [sec]
  double true_traj_time_; // [sec]
  ulong progress_counter_;
  bool new_trajectory_;

  double stop_request_time_; // [sec]
  bool stop_;
  bool stop_request_;
  bool resume_request_;

  vectI motors_vel_;
  double slowing_exp_;
  double arrest_time_; // [sec]
  double time_since_stop_request_; // [sec]

  WinchesTorqueControl winches_controller_;

  vect<TrajectoryD> traj_cables_len_;
  vect<TrajectoryI> traj_motors_pos_;
  vect<TrajectoryI> traj_motors_vel_;
  vect<TrajectoryS> traj_motors_torque_;

  void processTrajTime();

  template <typename T>
  T getTrajectoryPointValue(const id_t id, const vect<Trajectory<T>>& trajectories, const ControlMode mode);

  void reset();
  void resetTime();

  template <typename T>
  bool areTrajectoriesValid(const vect<Trajectory<T>>& trajectories);
};

#endif // CABLE_ROBOT_CONTROLLER_JOINTS_PVT_H
