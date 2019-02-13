#ifndef MAIN_GUI_H
#define MAIN_GUI_H

#include <QDialog>

#include "libcdpr/inc/types.h"
#include "easylogging++.h"

#include "gui/calib/calibration_dialog.h"
#include "gui/homing/homing_dialog.h"
#include "robot/cablerobot.h"
#include "ctrl/controller_singledrive.h"

using GSWDOpModes = grabec::GoldSoloWhistleOperationModes;

namespace Ui
{
class MainGUI;
}

class MainGUI : public QDialog
{
  Q_OBJECT

public:
  MainGUI(QWidget* parent, const grabcdpr::Params& config);
  ~MainGUI();

private slots:
  void on_pushButton_reset_clicked();

  void on_pushButton_calib_clicked();

  void on_pushButton_homing_clicked();

  void on_pushButton_startApp_clicked();

  void on_pushButton_enable_clicked();
  void on_pushButton_faultReset_clicked();

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
  void enableInterface(const bool op_outcome = false);
  void appendText2Browser(const QString& text);
  void updateEcStatusLED(const Bitfield8& ec_status_flags);
  void updateRtThreadStatusLED(const bool active);
  void handleMotorStatusUpdate(const id_t&, const grabec::GSWDriveInPdos& motor_status);

private:
  bool ec_network_valid_ = false;
  bool rt_thread_running_ = false;

  Ui::MainGUI* ui = NULL;
  CalibrationDialog* calib_dialog_ = NULL;
  HomingDialog* homing_dialog_ = NULL;

  grabcdpr::Params config_params_;
  CableRobot* robot_ptr_ = NULL;

  void StartRobot();
  void DeleteRobot();
  bool ExitReadyStateRequest();
  void CloseAllApps();

private:
  /*--------- Direct drive control stuff --------*/

  bool manual_ctrl_enabled_ = false;
  Bitfield8 waiting_for_response_;
  Bitfield8 desired_ctrl_mode_;
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
