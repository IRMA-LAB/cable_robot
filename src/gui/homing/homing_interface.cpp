/**
 * @file homing_interface.cpp
 * @author Simone Comari
 * @date 11 Mar 2019
 * @brief This file includes definitions of classes present in homing_interface.h.
 */

#include "gui/homing/homing_interface.h"

HomingInterface::HomingInterface(QWidget* parent, CableRobot* robot)
  : QDialog(parent), robot_ptr_(robot)
{}

HomingInterface::~HomingInterface() {}
