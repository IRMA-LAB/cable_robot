#ifndef CABLEROBOTINTERFACE_H
#define CABLEROBOTINTERFACE_H

#include <QWidget>

#include "cablerobotmaster.h" //User declaration
#include "actuatorinterface.h"
#include "calibrationinterface.h"
#include "hominginterface.h"
#include "actuatorpvtinterface33.h"
#include "demointerface33.h"
#include "demointerface66.h"
#include "manualinterface33.h"
#include "manualinterface66.h"

namespace Ui
{
class CableRobotInterface;
}

class CableRobotInterface : public QWidget
{
  Q_OBJECT

private:
public:
  explicit CableRobotInterface(QWidget* parent = 0, CableRobotMaster* theMaster = 0);
  ~CableRobotInterface();

private slots:
  void on_ActuatorControlButton_clicked();
  void on_CalibrationButton_clicked();
  void on_HomingButton_clicked();
  void on_Robot66ManualButton_clicked();
  void on_Robot66DemoButton_clicked();
  void on_Robot33ActuatorPvtButton_clicked();
  void on_Robot33AutomaticButton_clicked();
  void on_Robot33ManualButton_clicked();
  void on_EasyCatButton_clicked();
  void on_StandardRobotButton_toggled(bool checked);
  void on_UserRobotButton_toggled(bool checked);

signals:
  void SendMotorNumber(int);
  void SendMasterRequest(int);
  void SendRobotRequest(int);
public slots:
  void CollectStartUp();
  void CollectMasterRequestProcessed(int state);
  void CollectRobotRequestProcessed(int state);

private:
  CableRobotMaster* cableRobotMaster;
  Ui::CableRobotInterface* ui;
};

#endif // CABLEROBOTINTERFACE_H
