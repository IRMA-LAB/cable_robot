/**
 * @file manual_control_app.cpp
 * @author Simone Comari
 * @date 02 Mar 2020
 * @brief This file includes definitions of class present in manual_control_app.h.
 */

#include "apps/manual_control_app.h"

ManualControlApp::ManualControlApp(QObject* parent, CableRobot* robot)
  : QObject(parent), robot_ptr_(robot),
    controller_(robot->getActiveMotorsID(), robot->GetRtCycleTimeNsec(),
                robot->getActiveComponentsParams())
{
  target_pos_ = getActualPos();
//  robot_ptr_->SetController(&controller_);
}

ManualControlApp::~ManualControlApp() { robot_ptr_->setController(nullptr); }

//--------- Public functions ---------------------------------------------------------//

const grabnum::Vector3d& ManualControlApp::getActualPos()
{
  pthread_mutex_lock(&robot_ptr_->Mutex());
  actual_pos_ = robot_ptr_->getCdprStatus().platform.position;
  pthread_mutex_unlock(&robot_ptr_->Mutex());
  return actual_pos_;
}

void ManualControlApp::setTarget(const Axis coord, const double value)
{
  target_pos_(coord) = value;
  pthread_mutex_lock(&robot_ptr_->Mutex());
  controller_.setTargetPosition(target_pos_);
  pthread_mutex_unlock(&robot_ptr_->Mutex());
}

void ManualControlApp::resetTarget()
{
  pthread_mutex_lock(&robot_ptr_->Mutex());
  controller_.stop();
  pthread_mutex_unlock(&robot_ptr_->Mutex());
  target_pos_ = getActualPos();
}
