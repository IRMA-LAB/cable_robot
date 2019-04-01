#include "ctrl/controller_joints_pvt.h"

ControllerJointsPVT::ControllerJointsPVT(QObject* parent)
  : QObject(parent), ControllerBase()
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

vect<ControlAction> ControllerJointsPVT::CalcCtrlActions(const grabcdpr::Vars&,
                                                         const vect<ActuatorStatus>&)
{
  traj_time_ = clock_.Elapsed();
  vect<ControlAction> actions(modes_.size());
  for (size_t i = 0; i < actions.size(); i++)
  {
    actions[i].motor_id  = motors_id_[i];
    actions[i].ctrl_mode = traj_completed_ ? NONE : modes_[i];
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
            GetTrajectoryPointValue(actions[i].motor_id, traj_motors_torque_);
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
    waypoint        = traj.waypointFromRelTime(traj_time_);
    traj_completed_ = waypoint.ts >= traj.timestamps.back();
    break;
  }
  if (traj_completed_)
    emit trajectoryCompleted();

  return waypoint.value;
}

void ControllerJointsPVT::Reset()
{
  target_flags_.ClearAll();
  traj_completed_ = false;
  new_trajectory_ = true;
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
