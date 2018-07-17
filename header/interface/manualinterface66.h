#ifndef MANUALINTERFACE66_H
#define MANUALINTERFACE66_H

#include <QWidget>
#include <QCloseEvent>

#include "cablerobotmaster.h" //User declaration

namespace Ui
{
class ManualInterface66;
}

class ManualInterface66 : public QWidget
{
  Q_OBJECT

public:
  explicit ManualInterface66(QWidget* parent = 0, CableRobotMaster* theMaster = 0);
  ~ManualInterface66();

signals:
  void GoBackIdle(int);

private:
  CableRobotMaster* cableRobotMaster;
  Ui::ManualInterface66* ui;

  virtual void closeEvent(QCloseEvent* event);
};

#endif // MANUALINTERFACE66_H
