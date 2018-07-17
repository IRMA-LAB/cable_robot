#ifndef ACTUATORINTERFACE_H
#define ACTUATORINTERFACE_H

#include <QWidget>
#include <QCloseEvent>
#include <QTableWidgetItem>

#include "cablerobotmaster.h" //User declaration
//#include "servomotor.h"
#include "common.h"

namespace Ui
{
class ActuatorInterface;
}

class ActuatorInterface : public QWidget
{
  Q_OBJECT

public:
  explicit ActuatorInterface(QWidget* parent = 0, CableRobotMaster* master = 0);
  ~ActuatorInterface();

signals:
  void GoBackIdle(int);
  void SendEnableRequest(int);
  void SendClearFaultRequest();
  void SendOperationModeChangeRequest(int);
  void SendCommandUpdateRequest(int, int);

private slots:
  void on_EnableButton_toggled(bool checked);
  void on_FaultResetButton_clicked();
  void on_PositionEnable_toggled(bool checked);
  void on_TorqueEnable_toggled(bool checked);
  void on_VelocityEnable_toggled(bool checked);
  void on_MovePlusBut_pressed();
  void on_MovePlusBut_released();
  void on_MoveMinusButton_pressed();
  void on_MoveMinusButton_released();
  void on_MicroMovePlusButton_pressed();
  void on_MicroMovePlusButton_released();
  void on_MicroMoveMinusButton_pressed();
  void on_MicroMoveMinusButton_released();
  void on_TorquePlusButton_pressed();
  void on_TorquePlusButton_released();
  void on_TorqueMinusButton_pressed();
  void on_TorqueMinusButton_released();
  void on_SpeedPlusButton_pressed();
  void on_SpeedPlusButton_released();
  void on_SpeedMinusButton_pressed();
  void on_SpeedMinusButton_released();

public slots:
  void CollectFaultPresentAdvice();
  void CollectEnableCommandProcessed(int status);
  void CollectClearFaultRequestProcessed();
  void CollectOperationModeChangeRequestProcessed(int op_mode);
  void CollectCommandUpdateRequestProcessed(int command, int status);
  void CollectGuiData(GoldSoloWhistleDrive::InputPdos* pdos, int n);

private:
  CableRobotMaster* cable_robot_master_;
  Ui::ActuatorInterface* ui;
  std::array<QTableWidgetItem, GoldSoloWhistleDrive::kGoldSoloWhistleDomainInputs_> input_items_;

  virtual void closeEvent(QCloseEvent* event);
};

#endif // ACTUATORINTERFACE_H
