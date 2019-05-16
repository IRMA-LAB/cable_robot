#include "ctrl/controller_joints_pvt.h"

ControllerJointsPVT::ControllerJointsPVT(const vect<grabcdpr::ActuatorParams>& params,
                                         QObject* parent)
  : QObject(parent), ControllerBase(), winches_controller_(params)
{
  Reset();
}

bool ControllerJointsPVT::SetCablesLenTrajectories(const vect<TrajectoryD>& trajectories)
{
  if (!AreTrajectoriesValid(trajectories))
    return false;
  Reset();
  traj_cables_len_ = trajectories;
  SetMode(ControlMode::CABLE_LENGTH);
  target_flags_.set(LENGTH);
  return true;
}

bool ControllerJointsPVT::SetMotorsPosTrajectories(const vect<TrajectoryI>& trajectories)
{
  if (!AreTrajectoriesValid(trajectories))
    return false;
  Reset();
  traj_motors_pos_ = trajectories;
  SetMode(ControlMode::MOTOR_POSITION);
  target_flags_.set(POSITION);
  return true;
}

bool ControllerJointsPVT::SetMotorsVelTrajectories(const vect<TrajectoryI>& trajectories)
{
  if (!AreTrajectoriesValid(trajectories))
    return false;
  Reset();
  traj_motors_vel_ = trajectories;
  SetMode(ControlMode::MOTOR_SPEED);
  target_flags_.set(SPEED);
  return true;
}

bool ControllerJointsPVT::SetMotorsTorqueTrajectories(
  const vect<TrajectoryS>& trajectories)
{
  if (!AreTrajectoriesValid(trajectories))
    return false;
  Reset();
  traj_motors_torque_ = trajectories;
  SetMode(ControlMode::MOTOR_TORQUE);
  target_flags_.set(TORQUE);
  return true;
}

void ControllerJointsPVT::StopTrajectoryFollowing()
{
  stop_request_ = true;
  stop_time_    = true_traj_time_;
  traj_time_    = true_traj_time_;
}

void ControllerJointsPVT::PauseTrajectoryFollowing(const bool value)
{
  static timespec start_pause_time;
  if (value)
    start_pause_time = clock_.GetCurrentTime();
  else
    pause_time_ += clock_.Elapsed(start_pause_time);
  pause_ = value;
}

vect<ControlAction>
ControllerJointsPVT::CalcCtrlActions(const grabcdpr::Vars&,
                                     const vect<ActuatorStatus>& actuators_status)
{
  true_traj_time_ = clock_.Elapsed() - pause_time_;
  vect<ControlAction> actions(modes_.size());
  for (size_t i = 0; i < actions.size(); i++)
  {
    actions[i].motor_id  = motors_id_[i];
    actions[i].ctrl_mode = (stop_ || pause_) ? NONE : modes_[i];
    switch (actions[i].ctrl_mode)
    {
      case CABLE_LENGTH:
        if (target_flags_.test(LENGTH))
          actions[i].cable_length =
            GetTrajectoryPointValue(actions[i].motor_id, traj_cables_len_);
        else
          actions[i].ctrl_mode = NONE;
        break;
      case MOTOR_POSITION:
        if (target_flags_.test(POSITION))
          actions[i].motor_position =
            GetTrajectoryPointValue(actions[i].motor_id, traj_motors_pos_);
        else
          actions[i].ctrl_mode = NONE;
        break;
      case MOTOR_SPEED:
        if (target_flags_.test(SPEED))
          actions[i].motor_speed =
            GetTrajectoryPointValue(actions[i].motor_id, traj_motors_vel_);
        else
          actions[i].ctrl_mode = NONE;
        break;
      case MOTOR_TORQUE:
        if (target_flags_.test(TORQUE))
          actions[i].motor_torque =
            winches_controller_[actions[i].motor_id].calcServoTorqueSetpoint(
              actuators_status[i],
              GetTrajectoryPointValue(actions[i].motor_id, traj_motors_torque_));
        else
          actions[i].ctrl_mode = NONE;
        break;
      case NONE:
        break;
    }
  }
  return actions;
}

double ControllerJointsPVT::GetProcessedTrajTime()
{
  static constexpr double kSlowingExp = -4.0;

  if (stop_request_)
  {
    double time_since_stop = true_traj_time_ - stop_time_;
    if (time_since_stop < kArrestTime_)
      traj_time_ += (true_traj_time_ - traj_time_) * exp(kSlowingExp * time_since_stop);
    else
      stop_ = true;
  }
  else
    traj_time_ = true_traj_time_;
  return traj_time_;
}

template <typename T>
T ControllerJointsPVT::GetTrajectoryPointValue(const id_t id,
                                               const vect<Trajectory<T>>& trajectories)
{
  static constexpr uint kProgressTriggerCounts = 10;
  static ulong progress_counter                = 0;

  if (new_trajectory_)
  {
    new_trajectory_ = false;
    clock_.Reset();
    traj_time_       = 0.0;
    true_traj_time_  = 0.0;
    progress_counter = 0;
  }

  WayPoint<T> waypoint;
  double progress = -1;
  bool stop       = true;
  for (const Trajectory<T>& traj : trajectories)
  {
    if (traj.id != id)
      continue;
    waypoint = traj.waypointFromRelTime(GetProcessedTrajTime());
    progress = waypoint.ts / traj.timestamps.back();
    stop &= progress >= 1.0;
    break;
  }

  if (progress > 0 && (progress_counter++ % kProgressTriggerCounts == 0))
    emit trajectoryProgressStatus(qRound(progress * 100.));
  if (stop)
  {
    stop_ = true;
    emit trajectoryCompleted();
  }

  return waypoint.value;
}

void ControllerJointsPVT::Reset()
{
  target_flags_.reset();
  stop_           = false;
  stop_request_   = false;
  new_trajectory_ = true;
  pause_          = false;
  pause_time_     = 0.0;
  stop_time_      = 0.0;
}

template <typename T>
bool ControllerJointsPVT::AreTrajectoriesValid(const vect<Trajectory<T>>& trajectories)
{
  // Safety check: all motors must have a trajectory
  for (const id_t& id : motors_id_)
  {
    bool id_found = false;
    for (const Trajectory<T>& traj : trajectories)
      if (traj.id == id)
      {
        id_found = true;
        break;
      }
    if (!id_found)
      return false;
  }
  return true;
}
