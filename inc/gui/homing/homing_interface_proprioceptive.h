#ifndef CABLE_ROBOT_HOMING_INTERFACE_PROPRIOCEPTIVE_H
#define CABLE_ROBOT_HOMING_INTERFACE_PROPRIOCEPTIVE_H

#include <QFileDialog>
#include <QMessageBox>

#include "gui/homing/init_torque_form.h"
#include "gui/homing/homing_interface.h"
#include "homing/homing_proprioceptive.h"

namespace Ui
{
class HomingInterfaceProprioceptive;
}

class HomingInterfaceProprioceptive : public HomingInterface
{
  Q_OBJECT

public:
  explicit HomingInterfaceProprioceptive(QWidget* parent, CableRobot* robot);
  ~HomingInterfaceProprioceptive();

public slots:
  void FaultPresent(const bool);
  void AcquisitionCompleteCb();
  void HomingCompleteCb();

private slots:
  void closeEvent(QCloseEvent *);

  void on_pushButton_enable_clicked();
  void on_pushButton_clearFaults_clicked();
  void on_checkBox_useCurrentTorque_stateChanged(int);
  void on_checkBox_maxTorque_stateChanged(int);
  void on_pushButton_start_clicked();
  void on_pushButton_acquire_clicked();

  void on_radioButton_internal_clicked();
  void on_radioButton_external_clicked();
  void on_pushButton_extFile_clicked();
  void on_pushButton_ok_clicked();

  void on_pushButton_cancel_clicked();
  void on_pushButton_done_clicked();

private slots:
  void AppendText2Browser(const QString& text);

private:
  Ui::HomingInterfaceProprioceptive* ui;
  QVector<InitTorqueForm*> init_torque_forms_;

  HomingProprioceptive app_;

  bool ParseExtFile(HomingProprioceptiveHomeData* res);
};

#endif // CABLE_ROBOT_HOMING_INTERFACE_PROPRIOCEPTIVE_H
