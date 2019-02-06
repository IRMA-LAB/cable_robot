#ifndef CABLE_ROBOT_HOMING_INTERFACE_H
#define CABLE_ROBOT_HOMING_INTERFACE_H

#include <QDialog>

#include "easylogging++.h"

#include "robot/cablerobot.h"

class HomingInterface: public QDialog
{
  Q_OBJECT

 public:
  HomingInterface(QWidget* parent, CableRobot* robot);
  virtual ~HomingInterface() = 0;

  virtual void Close() { close(); }

 signals:
  void homingFailed() const;
  void homingSuccess() const;

 protected:
  CableRobot* robot_ptr_ = NULL;
};

#endif // CABLE_ROBOT_HOMING_INTERFACE_H
