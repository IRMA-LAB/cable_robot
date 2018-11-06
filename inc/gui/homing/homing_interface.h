#ifndef HOMING_INTERFACE_H
#define HOMING_INTERFACE_H

#include <QDialog>

#include "robot/cablerobot.h"

class HomingInterface : public QDialog
{
  Q_OBJECT

public:
  explicit HomingInterface(QWidget* parent, CableRobot* robot);
  virtual ~HomingInterface() = 0;

signals:
  void homingFailed() const;
  void homingSuccess() const;

protected:
  CableRobot* robot_ptr_ = NULL;
};

#endif // HOMING_INTERFACE_H
