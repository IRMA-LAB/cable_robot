/**
 * @file main_gui.h
 * @author Simone Comari
 * @date 16 May 2019
 * @brief This file takes care of the functionalities of the main GUI of cable robot app.
 *
 * The functionalities of the main GUI include buttons management, signaling with children
 * widgets and, more importantly, direct drive manual control.
 */

#ifndef MAIN_GUI_H
#define MAIN_GUI_H

#include <QDialog>
#include <QSpacerItem>

#include "easylogging++.h"
#include "libcdpr/inc/types.h"

#include "ctrl/controller_singledrive.h"
#include "gui/apps/joints_pvt_dialog.h"
#include "gui/apps/manual_control_dialog.h"
#include "gui/calib/calibration_dialog.h"
#include "gui/homing/homing_dialog.h"
#include "robot/cablerobot.h"

#define DEBUG_GUI 1

using GSWDOpModes = grabec::GoldSoloWhistleOperationModes; /**< Shortcut for op modes. */

namespace Ui {
class MainGUI;
}

/**
 * @brief This class implements the control logic of the main GUI of cable robot app.
 *
 * MainGUI class represents the central control panel of cable robot app.
 * From here calibration, homing and operational procedure can be launched, moreover a
 * feedback about the status of the network and the real time thread is provided by means
 * of colored LEDs.
 * A text browser displays important informations about recent events, warnings and
 * errors. At the bottom the direct drive control panel allows the user to interact with
 * each available drive and move it in either position, velocity or torque mode. Please
 * note that this control mode is enabled only before robot completed the homing
 * procedure, i.e. when robot is in ENABLED state.
 */
class MainGUI: public QDialog
{
  Q_OBJECT

 public:
  /**
   * @brief MainGUI constructor.
   * @param[in] parent The parent Qt object.
   * @param[in] config The configuration parameters of the cable robot.
   */
  MainGUI(QWidget* parent, const grabcdpr::Params& config);
  ~MainGUI();

 private slots:
  void on_pushButton_reset_clicked();

  void on_pushButton_calib_clicked();

  void on_pushButton_homing_clicked();

  void on_pushButton_startApp_clicked();

#if DEBUG_GUI
  void pushButton_debug_clicked();
  void handleDebugCompleted();
#endif

 private slots:
  //--------- Direct drive control panel buttons -------------------------------------//

  void on_pushButton_enable_clicked();
  void on_pushButton_faultReset_clicked();
  void on_pushButton_exitReady_clicked();

  void on_radioButton_posMode_clicked();
  void on_radioButton_velMode_clicked();
  void on_radioButton_torqueMode_clicked();

  void on_pushButton_posPlus_pressed();
  void on_pushButton_posPlus_released();
  void on_pushButton_posMinus_pressed();
  void on_pushButton_posMinus_released();
  void on_pushButton_posMicroPlus_pressed();
  void on_pushButton_posMicroPlus_released();
  void on_pushButton_posMicroMinus_pressed();
  void on_pushButton_posMicroMinus_released();

  void on_horizontalSlider_speed_ctrl_sliderPressed();
  void on_horizontalSlider_speed_ctrl_sliderMoved(int position);
  void on_horizontalSlider_speed_ctrl_sliderReleased();

  void on_pushButton_torqueMinus_pressed();
  void on_pushButton_torqueMinus_released();
  void on_pushButton_torquePlus_pressed();
  void on_pushButton_torquePlus_released();

 private slots:
  void enableInterface(const bool op_outcome = true);
  void appendText2Browser(const QString& text);
  void updateEcStatusLED(const std::bitset<3>& ec_status_flags);
  void updateRtThreadStatusLED(const bool active);
  void handleMotorStatusUpdate(const id_t&, const grabec::GSWDriveInPdos& motor_status);

 private:
  bool ec_network_valid_  = false;
  bool rt_thread_running_ = false;

  Ui::MainGUI* ui                  = nullptr;
  CalibrationDialog* calib_dialog_ = nullptr;
  HomingDialog* homing_dialog_     = nullptr;

  JointsPVTDialog* joints_pvt_dialog_ = nullptr;
  ManualControlDialog* man_ctrl_dialog_ = nullptr;

  grabcdpr::Params config_params_;
  CableRobot* robot_ptr_ = nullptr;

  void StartRobot();
  void DeleteRobot();
  bool ExitReadyStateRequest();
  void CloseAllApps();

#if DEBUG_GUI
  QPushButton* pushButton_debug;
  QSpacerItem* verticalSpacer_5;
  // Insert your debug object here..
//  MyDebugClass* temp_app = NULL;
#endif

 private:
  //--------- Direct drive control stuff ---------------------------------------------//

  static constexpr int16_t kTorqueSsErrTol_ = 5;

  bool manual_ctrl_enabled_ = false;
  std::bitset<3> waiting_for_response_;
  std::bitset<5> desired_ctrl_mode_;
  id_t motor_id_;
  ControllerSingleDrive* man_ctrl_ptr_;

  void DisablePosCtrlButtons(const bool value);
  void DisableVelCtrlButtons(const bool value);
  void DisableTorqueCtrlButtons(const bool value);

  void UpdateDriveStatusTable(const grabec::GSWDriveInPdos& status);
  void UpdateDriveCtrlPanel(const Actuator::States state);
  void UpdateDriveCtrlButtons(const ControlMode ctrl_mode);

  void SetupDirectMotorCtrl(const bool enable);
  static ControlMode DriveOpMode2CtrlMode(const int8_t drive_op_mode);
};

#endif // MAIN_GUI_H
