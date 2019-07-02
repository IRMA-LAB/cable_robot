#include "apps/manual_control_app.h"

ManualControlApp::ManualControlApp(QObject* parent, CableRobot* robot)
  : QObject(parent), robot_ptr_(robot)
{
  // TODO:
  // 1. Initialize target position with current platform position
  // 2. Setup axes controller with initial target and assign it to robot object
}

ManualControlApp::~ManualControlApp() { robot_ptr_->SetController(nullptr); }

//--------- Public functions ---------------------------------------------------------//

const grabnum::Vector3d& ManualControlApp::getActualPos() const
{
  // TODO: get actual platform pose from controller
  return actual_pos_;
}

void ManualControlApp::setTarget(const Coordinates coord, const double value)
{
  target_pos_(coord) = value;
  // TODO: set new updated controller target
}

void ManualControlApp::resetTarget()
{
  // TODO: set controller target position with current platform position
}
