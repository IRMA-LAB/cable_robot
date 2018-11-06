#include "gui/homing/homing_interface.h"

HomingInterface::HomingInterface(QWidget* parent, CableRobot* robot)
  : QDialog(parent), robot_ptr_(robot)
{
}

HomingInterface::~HomingInterface() {}
