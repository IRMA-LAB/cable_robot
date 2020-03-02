/**
 * @file homing_interface_proprioceptive.h
 * @author Simone Comari
 * @date 02 Mar 2020
 * @brief This file takes care of the functionalities of the proprioceptive homing
 * interface of cable robot app.
 *
 * The functionalities of the proprioceptive homing interface include buttons management
 * and signaling with the proprioceptive homing app, where the actual algorithm is
 * implemented.
 */

#ifndef CABLE_ROBOT_HOMING_INTERFACE_PROPRIOCEPTIVE_H
#define CABLE_ROBOT_HOMING_INTERFACE_PROPRIOCEPTIVE_H

#include <QCloseEvent>
#include <QFileDialog>
#include <QMessageBox>

#include "gui/homing/homing_interface.h"
#include "gui/homing/init_torque_form.h"
#include "homing/homing_proprioceptive_app.h"


namespace Ui {
class HomingInterfaceProprioceptiveWidget;
}

/**
 * @brief This class implements the controls of the proprioceptive homing process and
 * allows the user to triggers events, set important parameters and interact with the
 * homing applications.
 */
class HomingInterfaceProprioceptiveWidget: public QWidget
{
  Q_OBJECT

 public:
  /**
   * @brief HomingInterfaceProprioceptive constructor.
   * @param parent The parent Qt object, in our case the homing dialog.
   * @param robot Pointer to the cable robot instance, to be passed to the inner app.
   */
  explicit HomingInterfaceProprioceptiveWidget(QWidget* parent, CableRobot* robot);
  ~HomingInterfaceProprioceptiveWidget() override final;

  HomingProprioceptiveApp app;

 private slots:

  void on_pushButton_enable_clicked();
  void on_pushButton_clearFaults_clicked();
  void on_checkBox_useCurrentTorque_stateChanged(int);
  void on_checkBox_initTorque_stateChanged(int);
  void on_spinBox_initTorque_valueChanged(int value);
  void on_checkBox_maxTorque_stateChanged(int);
  void on_spinBox_maxTorque_valueChanged(int value);
  void on_pushButton_start_clicked();

  void on_radioButton_internal_clicked();
  void on_radioButton_external_clicked();
  void on_pushButton_extFile_clicked();
  void on_pushButton_ok_clicked();

 private slots:
  void appendText2Browser(const QString& text);
  void updateAcquisitionProgress(const int value);
  void updateOptimizationProgress(const int value);

  void handleAcquisitionComplete();
  void handleStateChanged(const quint8& state);
  void handleHomingComplete();

 private:
  Ui::HomingInterfaceProprioceptiveWidget* ui;
  QVector<InitTorqueForm*> init_torque_forms_;

  CableRobot* robot_ptr_ = nullptr;
  bool acquisition_complete_;
  bool ext_close_cmd_;

  void updateTorquesLimits();
};

class HomingInterfaceProprioceptive: public HomingInterface
{
 public:
  explicit HomingInterfaceProprioceptive(QWidget* parent, CableRobot* robot);
  ~HomingInterfaceProprioceptive() override;

 private:
  HomingInterfaceProprioceptiveWidget widget_;

  bool rejectedExitRoutine(const bool force_exit = false) override final;
};

#endif // CABLE_ROBOT_HOMING_INTERFACE_PROPRIOCEPTIVE_H
