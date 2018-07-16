#ifndef MANUALINTERFACE33_H
#define MANUALINTERFACE33_H

#include <QWidget>
#include <QCloseEvent>

#include "cablerobotmaster.h" //User declaration

namespace Ui
{
class ManualInterface33;
}

class ManualInterface33 : public QWidget
{
  Q_OBJECT

public:
  explicit ManualInterface33(QWidget* parent = 0,
                             CableRobotMaster* theMaster = 0);
  ~ManualInterface33();

signals:
  void GoBackIdle(int);

private:
  CableRobotMaster* cableRobotMaster;
  Ui::ManualInterface33* ui;

  virtual void closeEvent(QCloseEvent* event);
};

#endif // MANUALINTERFACE33_H
