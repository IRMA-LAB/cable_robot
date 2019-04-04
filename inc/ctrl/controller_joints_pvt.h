#ifndef CABLE_ROBOT_CONTROLLER_JOINTS_PVT_H
#define CABLE_ROBOT_CONTROLLER_JOINTS_PVT_H

#include "ctrl/controller_base.h"

template <typename T> struct WayPoint
{
  double ts = -1.0;
  T value;

  WayPoint() {}
  WayPoint(const double time, const T& _value) : ts(time), value(_value) {}
};

using WayPointD = WayPoint<double>;
using WayPointI = WayPoint<int>;
using WayPointS = WayPoint<short>;

template <typename T> struct Trajectory
{
  id_t id = 0;
  vectD timestamps;
  vect<T> values;

  Trajectory() {}
  Trajectory(const id_t _id) : id(_id) {}
  Trajectory(const id_t _id, const vect<T>& _values) : id(_id), values(_values)
  {
    timestamps = vectD(values.size(), -1.0);
  }
  Trajectory(const id_t _id, const vect<T>& _values, const vectD& times)
    : id(_id), values(_values), timestamps(times)
  {
    assert(times.size() == _values.size());
  }

  WayPoint<T> waypointFromIndex(const size_t index) const
  {
    assert(index < timestamps.size() && index < values.size());
    return WayPoint<T>(timestamps[index], values[index]);
  }

  WayPoint<T> waypointFromAbsTime(const double time) const
  {
    assert(timestamps.front() >= 0.0);

    if (time <= timestamps.front())
      return WayPoint<T>(timestamps.front(), values.front());
    if (time >= timestamps.back())
      return WayPoint<T>(timestamps.back(), values.back());
    // Find nearest neighbor
    vectD::const_iterator low =
      std::lower_bound(timestamps.begin(), timestamps.end(), time);
    if (low == timestamps.end())
      low -= 1; // no bigger value than val in vector
    const ulong lower_idx = low - timestamps.begin();
    const ulong upper_idx = lower_idx + 1;
    // Interpolate
    const double slope = (values[upper_idx] - values[lower_idx]) /
                         (timestamps[upper_idx] - timestamps[lower_idx]);
    const double offset = values[upper_idx] - slope * timestamps[upper_idx];
    const double value  = slope * time + offset;

    return WayPoint<T>(time, static_cast<T>(value));
  }

  WayPoint<T> waypointFromRelTime(const double time) const
  {
    assert(timestamps.front() >= 0.0);

    return waypointFromAbsTime(time + timestamps.front());
  }
};

using TrajectoryD = Trajectory<double>;
using TrajectoryI = Trajectory<int>;
using TrajectoryS = Trajectory<short>;

class ControllerJointsPVT: public QObject, public ControllerBase
{
  Q_OBJECT

 public:
  explicit ControllerJointsPVT(QObject* parent = NULL);
  ~ControllerJointsPVT() {}

  bool SetCablesLenTrajectories(const vect<TrajectoryD>& trajectories);
  bool SetMotorsPosTrajectories(const vect<TrajectoryI>& trajectories);
  bool SetMotorsVelTrajectories(const vect<TrajectoryI>& trajectories);
  bool SetMotorsTorqueTrajectories(const vect<TrajectoryS>& trajectories);

  void PauseTrajectoryFollowing(const bool value) { pause_ = value; }
  void StopTrajectoryFollowing() { traj_completed_ = true; }

  bool IsPaused() const { return pause_; }
  /**
   * @brief Check if active target is reached, independently from the control mode.
   * @return _True_ if target is reached, _false_ otherwise.
   */
  bool TargetReached() const override final { return traj_completed_; }

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
  vect<ControlAction> CalcCtrlActions(const grabcdpr::Vars& robot_status,
                                      const vect<ActuatorStatus>&) override final;

 signals:
  void trajectoryCompleted() const;

 private:
  enum BitPosition
  {
    LENGTH,
    POSITION,
    SPEED,
    TORQUE
  };

  Bitfield8 target_flags_;

  grabrt::Clock clock_;
  double traj_time_;
  bool traj_completed_;
  bool new_trajectory_;
  bool pause_;

  vect<TrajectoryD> traj_cables_len_;
  vect<TrajectoryI> traj_motors_pos_;
  vect<TrajectoryI> traj_motors_vel_;
  vect<TrajectoryS> traj_motors_torque_;

  template <typename T>
  T GetTrajectoryPointValue(const id_t id, const vect<Trajectory<T>>& trajectories);

  void Reset();

  template <typename T>
  bool AreTrajectoriesValid(const vect<Trajectory<T>>& trajectories);
};

#endif // CABLE_ROBOT_CONTROLLER_JOINTS_PVT_H
