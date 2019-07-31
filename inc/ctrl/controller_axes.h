#ifndef CABLE_ROBOT_CONTROLLER_AXES_H
#define CABLE_ROBOT_CONTROLLER_AXES_H

#include "kinematics.h"
#include "statics.h"

#include "controller_base.h"

enum Axis: uchar
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

  void setAxesTarget(const Vector3d &target_pos);
  void SingleAxisIncrement(const Axis axis, const bool active,
                           const Sign sign = Sign::POS);

  void stop();
  bool TargetReached() const override final { return !target_set_; }

  vect<ControlAction>
  CalcCtrlActions(const grabcdpr::RobotVars& vars,
                  const vect<ActuatorStatus>& robot_status) override final;

 private:
  static constexpr double kAbsDeltaMovePerSec_ = 0.01; // [m/s]

  grabcdpr::RobotParams params_;
  double period_sec_;
  bool change_target_;
  bool target_set_;
  grabnum::VectorXd<POSE_DIM> delta_move_;
  grabnum::VectorXd<POSE_DIM> target_pose_;

  grabnum::Vector3d
  NonLinsolveJacGeomStatic(const grabnum::VectorXd<POSE_DIM>& init_guess,
                           const uint8_t nmax = 100) const;

  bool isPoseReachable(const grabcdpr::RobotVars& vars) const;
};

#endif // CABLE_ROBOT_CONTROLLER_AXES_H
