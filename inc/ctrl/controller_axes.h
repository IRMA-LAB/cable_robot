#ifndef CABLE_ROBOT_CONTROLLER_AXES_H
#define CABLE_ROBOT_CONTROLLER_AXES_H

#include "kinematics.h"
#include "statics.h"
#include "under_actuated_utils.h"

#include "controller_base.h"

enum Axis : uchar
{
  X = 1,
  Y,
  Z
};

class ControllerAxes: public ControllerBase
{
 public:
  explicit ControllerAxes(const vect<id_t>& motors_id, const uint32_t period_nsec,
                          const grabcdpr::RobotParams& params);
  ~ControllerAxes() override {}

  void setTargetPosition(const Vector3d& target_pos);
  void singleAxisIncrement(const Axis axis, const bool active,
                           const Sign sign = Sign::POS);

  void stop();
  bool targetReached() const override final { return !target_set_; }

  vect<ControlAction>
  calcCtrlActions(const grabcdpr::RobotVars& vars,
                  const vect<ActuatorStatus>& robot_status) override final;

 private:
  static constexpr double kAbsDeltaMovePerSec_ = 0.01; // [m/s]
  static constexpr double kMinCableTension_    = 2.0;  // [N]
  static constexpr double kMaxCableTension_    = 10.0; // [N]

  grabcdpr::RobotParams params_;
  double abs_delta_move_per_cycle_;
  bool change_target_;
  bool target_set_;
  grabnum::VectorXd<POSE_DIM> delta_move_;
  grabnum::VectorXd<POSE_DIM> target_pose_;
  grabnum::Vector3d target_position_;

  void interpPosTarget(const grabnum::Vector3d& current_pos);

  bool calcRealTargetPose(grabnum::Vector3d& position,
                          grabnum::Vector3d& orientation) const;

  bool isPoseReachable(grabcdpr::RobotVars& vars) const;
};

#endif // CABLE_ROBOT_CONTROLLER_AXES_H
