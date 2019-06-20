#include "ctrl/controller_joints_pvt.h"

constexpr double ControllerJointsPVT::kMinArrestTime_;

ControllerJointsPVT::ControllerJointsPVT(const vect<grabcdpr::ActuatorParams>& params,
                                         const uint32_t cycle_t_nsec, QObject* parent)
  : QObject(parent), ControllerBase(), winches_controller_(params)
{
  motors_vel_.resize(params.size());
  cycle_time_ = grabrt::NanoSec2Sec(cycle_t_nsec);
  Reset();

  traj_time_         = 0.0;
  true_traj_time_    = 0.0;
  stop_request_time_ = 0.0;
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

void ControllerJointsPVT::stopTrajectoryFollowing()
{
  stop_request_      = true;
  stop_request_time_ = true_traj_time_;
  traj_time_         = true_traj_time_;

  int max_abs_motor_speed = 0;
  for (const int vel : motors_vel_)
    if (abs(vel) > max_abs_motor_speed)
      max_abs_motor_speed = abs(vel);
  arrest_time_ = std::max(kMinArrestTime_, max_abs_motor_speed / kVel2ArrestTimeRatio_);
  slowing_exp_ = -5. / arrest_time_;
}

void ControllerJointsPVT::pauseTrajectoryFollowing() { stopTrajectoryFollowing(); }

void ControllerJointsPVT::resumeTrajectoryFollowing()
{
  stop_           = false;
  resume_request_ = true;
  true_traj_time_ = 0.0;
}

vect<ControlAction>
ControllerJointsPVT::CalcCtrlActions(const grabcdpr::Vars&,
                                     const vect<ActuatorStatus>& actuators_status)
{
  // Possibly apply smooth resume/stop
  processTrajTime();
  // Collect motors speed for possible arrest/resume time computation
  for (ulong i = 0; i < actuators_status.size(); i++)
    motors_vel_[i] = actuators_status[i].motor_speed;
  vect<ControlAction> actions(modes_.size());
  for (size_t i = 0; i < actions.size(); i++)
  {
    actions[i].motor_id  = motors_id_[i];
    actions[i].ctrl_mode = stop_ ? NONE : modes_[i];
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

void ControllerJointsPVT::processTrajTime()
{
  if (stop_)
    return;

  if (new_trajectory_)
  {
    new_trajectory_ = false;
    ResetTime();
    return;
  }

  true_traj_time_ += cycle_time_;

  if (stop_request_)
  {
    double time_since_stop_request = true_traj_time_ - stop_request_time_;
    if (time_since_stop_request <= arrest_time_)
      traj_time_ += cycle_time_ * exp(slowing_exp_ * time_since_stop_request);
    else
    {
      stop_         = true;
      stop_request_ = false;
    }
  }
  else if (resume_request_)
  {
    if (true_traj_time_ <= arrest_time_)
      traj_time_ += cycle_time_ * exp(slowing_exp_ * (arrest_time_ - true_traj_time_));
    else
    {
      resume_request_ = false;
      true_traj_time_ = traj_time_;
    }
  }
  else
    traj_time_ = true_traj_time_;
}

template <typename T>
T ControllerJointsPVT::GetTrajectoryPointValue(const id_t id,
                                               const vect<Trajectory<T>>& trajectories)
{
  static const ulong kProgressTriggerCounts = 200 * motors_id_.size();

  WayPoint<T> waypoint;
  double progress = -1;
  bool stop       = true;
  for (const Trajectory<T>& traj : trajectories)
  {
    if (traj.id != id)
      continue;
    if (resume_request_ || stop_request_)
      waypoint = traj.waypointFromRelTime(traj_time_);
    else
      waypoint = traj.waypointFromRelTime(traj_time_, cycle_time_);
    progress = waypoint.ts / traj.timestamps.back();
    stop &= progress >= 1.0;
    break;
  }

  if (stop)
  {
    stop_ = true;
    emit trajectoryCompleted();
  }
  if (progress > 0 && (progress_counter_++ % kProgressTriggerCounts == 0) &&
      !stop_request_)
    emit trajectoryProgressStatus(qRound(progress * 100.), waypoint.ts);
  return waypoint.value;
}

void ControllerJointsPVT::Reset()
{
  target_flags_.reset();
  stop_             = false;
  stop_request_     = false;
  resume_request_   = false;
  new_trajectory_   = true;
  progress_counter_ = 0;
}

void ControllerJointsPVT::ResetTime()
{
  traj_time_         = 0.0;
  true_traj_time_    = 0.0;
  stop_request_time_ = 0.0;
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
    {
      CLOG(WARNING, "event") << "Invalid trajectories!";
      return false;
    }
  }
  return true;
}
