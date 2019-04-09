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
  target_flags_.Set(LENGTH);
  return true;
}

bool ControllerJointsPVT::SetMotorsPosTrajectories(const vect<TrajectoryI>& trajectories)
{
  if (!AreTrajectoriesValid(trajectories))
    return false;
  Reset();
  traj_motors_pos_ = trajectories;
  target_flags_.Set(POSITION);
  return true;
}

bool ControllerJointsPVT::SetMotorsVelTrajectories(const vect<TrajectoryI>& trajectories)
{
  if (!AreTrajectoriesValid(trajectories))
    return false;
  Reset();
  traj_motors_vel_ = trajectories;
  target_flags_.Set(SPEED);
  return true;
}

bool ControllerJointsPVT::SetMotorsTorqueTrajectories(
  const vect<TrajectoryS>& trajectories)
{
  if (!AreTrajectoriesValid(trajectories))
    return false;
  Reset();
  traj_motors_torque_ = trajectories;
  target_flags_.Set(TORQUE);
  return true;
}

void ControllerJointsPVT::PauseTrajectoryFollowing(const bool value)
{
  static timespec start_pause_time_;
  if (value)
    start_pause_time_ = clock_.GetCurrentTime();
  else
    pause_time_ += clock_.Elapsed(start_pause_time_);
  pause_ = value;
}

vect<ControlAction>
ControllerJointsPVT::CalcCtrlActions(const grabcdpr::Vars&,
                                     const vect<ActuatorStatus>& actuators_status)
{
  traj_time_ = clock_.Elapsed() - pause_time_;
  vect<ControlAction> actions(modes_.size());
  for (size_t i = 0; i < actions.size(); i++)
  {
    actions[i].motor_id  = motors_id_[i];
    actions[i].ctrl_mode = (stop_ || pause_) ? NONE : modes_[i];
    switch (actions[i].ctrl_mode)
    {
      case CABLE_LENGTH:
        if (target_flags_.CheckBit(LENGTH))
          actions[i].cable_length =
            GetTrajectoryPointValue(actions[i].motor_id, traj_cables_len_);
        else
          actions[i].ctrl_mode = NONE;
        break;
      case MOTOR_POSITION:
        if (target_flags_.CheckBit(POSITION))
          actions[i].motor_position =
            GetTrajectoryPointValue(actions[i].motor_id, traj_motors_pos_);
        else
          actions[i].ctrl_mode = NONE;
        break;
      case MOTOR_SPEED:
        if (target_flags_.CheckBit(SPEED))
          actions[i].motor_speed =
            GetTrajectoryPointValue(actions[i].motor_id, traj_motors_vel_);
        else
          actions[i].ctrl_mode = NONE;
        break;
      case MOTOR_TORQUE:
        if (target_flags_.CheckBit(TORQUE))
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

template <typename T>
T ControllerJointsPVT::GetTrajectoryPointValue(const id_t id,
                                               const vect<Trajectory<T>>& trajectories)
{
  if (new_trajectory_)
  {
    new_trajectory_ = false;
    clock_.Reset();
    traj_time_ = 0.0;
  }

  WayPoint<T> waypoint;
  for (const Trajectory<T>& traj : trajectories)
  {
    if (traj.id != id)
      continue;
    waypoint = traj.waypointFromRelTime(traj_time_);
    stop_    = waypoint.ts >= traj.timestamps.back();
    break;
  }
  if (stop_)
    emit trajectoryCompleted();

  return waypoint.value;
}

void ControllerJointsPVT::Reset()
{
  target_flags_.ClearAll();
  stop_           = false;
  new_trajectory_ = true;
  pause_          = false;
  pause_time_     = 0.0;
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
