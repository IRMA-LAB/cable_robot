#ifndef CABLE_ROBOT_HOMING_INTERFACE_PROPRIOCEPTIVE_H
#define CABLE_ROBOT_HOMING_INTERFACE_PROPRIOCEPTIVE_H

#include <QCloseEvent>
#include <QFileDialog>
#include <QMessageBox>

#include "gui/homing/homing_interface.h"
#include "gui/homing/init_torque_form.h"
#include "homing/homing_proprioceptive.h"


namespace Ui {
class HomingInterfaceProprioceptive;
}

class HomingInterfaceProprioceptive: public HomingInterface
{
  Q_OBJECT

 public:
  explicit HomingInterfaceProprioceptive(QWidget* parent, CableRobot* robot);
  ~HomingInterfaceProprioceptive() override final;

 private slots:
  void closeEvent(QCloseEvent* event) override final;

  void on_pushButton_enable_clicked();
  void on_pushButton_clearFaults_clicked();
  void on_checkBox_useCurrentTorque_stateChanged(int);
  void on_checkBox_maxTorque_stateChanged(int);
  void on_spinBox_maxTorque_valueChanged(int value);
  void on_pushButton_start_clicked();

  void on_radioButton_internal_clicked();
  void on_radioButton_external_clicked();
  void on_pushButton_extFile_clicked();
  void on_pushButton_ok_clicked();

  void on_pushButton_cancel_clicked();
  void on_pushButton_done_clicked();

 private slots:
  void appendText2Browser(const QString& text);
  void updateAcquisitionProgress(const int value);
  void updateOptimizationProgress(const int value);

  void handleAcquisitionComplete();
  void handleHomingComplete();
  void handleStateChanged(const quint8& state);

 private:
  Ui::HomingInterfaceProprioceptive* ui;
  QVector<InitTorqueForm*> init_torque_forms_;

  HomingProprioceptive app_;
  bool acquisition_complete_;
  bool ext_close_cmd_;

  void UpdateTorquesLimits();
  bool ParseExtFile(HomingProprioceptiveHomeData*);

  void Close() override final;
};

#endif // CABLE_ROBOT_HOMING_INTERFACE_PROPRIOCEPTIVE_H
