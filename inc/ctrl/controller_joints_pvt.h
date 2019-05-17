#ifndef CABLE_ROBOT_CONTROLLER_JOINTS_PVT_H
#define CABLE_ROBOT_CONTROLLER_JOINTS_PVT_H

#include "ctrl/controller_base.h"
#include "ctrl/winch_torque_controller.h"

class ControllerJointsPVT: public QObject, public ControllerBase
{
  Q_OBJECT

 public:
  explicit ControllerJointsPVT(const vect<grabcdpr::ActuatorParams>& params,
                               const uint32_t cycle_t_nsec, QObject* parent = NULL);
  ~ControllerJointsPVT() override {}

  bool SetCablesLenTrajectories(const vect<TrajectoryD>& trajectories);
  bool SetMotorsPosTrajectories(const vect<TrajectoryI>& trajectories);
  bool SetMotorsVelTrajectories(const vect<TrajectoryI>& trajectories);
  bool SetMotorsTorqueTrajectories(const vect<TrajectoryS>& trajectories);

  void PauseTrajectoryFollowing(const bool value);
  void StopTrajectoryFollowing();

  bool IsPaused() const { return stop_; }
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
  void trajectoryCompleted() const;
  void trajectoryProgressStatus(const int) const;

 private:
  static constexpr double kArrestTime_ = 1.0; // sec

  enum BitPosition
  {
    LENGTH,
    POSITION,
    SPEED,
    TORQUE
  };

  std::bitset<4> target_flags_;

  grabrt::Clock clock_;
  double cycle_time_;
  double traj_time_;
  double true_traj_time_;
  bool stop_;
  double stop_time_;
  bool stop_request_;
  double stop_request_time_;
  bool new_trajectory_;

  double paused_time_;
  double resume_request_time_;
  bool resume_request_;


  WinchesTorqueControl winches_controller_;

  vect<TrajectoryD> traj_cables_len_;
  vect<TrajectoryI> traj_motors_pos_;
  vect<TrajectoryI> traj_motors_vel_;
  vect<TrajectoryS> traj_motors_torque_;

  double GetProcessedTrajTime();

  template <typename T>
  T GetTrajectoryPointValue(const id_t id, const vect<Trajectory<T>>& trajectories);

  void Reset();

  template <typename T>
  bool AreTrajectoriesValid(const vect<Trajectory<T>>& trajectories);
};

#endif // CABLE_ROBOT_CONTROLLER_JOINTS_PVT_H
