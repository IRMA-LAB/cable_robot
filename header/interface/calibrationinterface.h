#ifndef CALIBRATIONINTERFACE_H
#define CALIBRATIONINTERFACE_H

#include <QWidget>
#include <QCloseEvent>

#include "cablerobotmaster.h" //User declaration

namespace Ui
{
class CalibrationInterface;
}

class CalibrationInterface : public QWidget
{
  Q_OBJECT

public:
  explicit CalibrationInterface(QWidget* parent = 0, CableRobotMaster* master = 0);
  ~CalibrationInterface();

signals:
  void GoBackIdle(int);

private:
  CableRobotMaster* cable_robot_master_;
  Ui::CalibrationInterface* ui;

  virtual void closeEvent(QCloseEvent* event);
};

#endif // CALIBRATIONINTERFACE_H
