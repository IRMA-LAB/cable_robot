#ifndef DEMOINTERFACE33_H
#define DEMOINTERFACE33_H

#include <QWidget>
#include <QCloseEvent>

#include "cablerobotmaster.h" //User declaration

namespace Ui
{
class DemoInterface33;
}

class DemoInterface33 : public QWidget
{
  Q_OBJECT

public:
  explicit DemoInterface33(QWidget* parent = 0, CableRobotMaster* master = 0);
  ~DemoInterface33();

signals:
  void GoBackIdle(int);

private:
  CableRobotMaster* cable_robot_master_;
  Ui::DemoInterface33* ui;

  virtual void closeEvent(QCloseEvent* event);
};

#endif // DEMOINTERFACE33_H
