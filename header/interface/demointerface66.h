#ifndef DEMOINTERFACE66_H
#define DEMOINTERFACE66_H

#include <QWidget>
#include <QCloseEvent>

#include "cablerobotmaster.h" //User declaration

namespace Ui
{
class DemoInterface66;
}

class DemoInterface66 : public QWidget
{
  Q_OBJECT

public:
  explicit DemoInterface66(QWidget* parent = 0, CableRobotMaster* master = 0);
  ~DemoInterface66();

signals:
  void GoBackIdle(int);

private:
  CableRobotMaster* cable_robot_master_;
  Ui::DemoInterface66* ui;

  virtual void closeEvent(QCloseEvent* event);
};

#endif // DEMOINTERFACE66_H
