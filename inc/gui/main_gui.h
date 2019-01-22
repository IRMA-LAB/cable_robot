#ifndef MAIN_GUI_H
#define MAIN_GUI_H

#include <QDialog>

#include "libcdpr/inc/types.h"
#include "easylogging++.h"

#include "gui/calib/calibration_dialog.h"
#include "gui/homing/homing_dialog.h"
#include "robot/cablerobot.h"
#include "ctrl/controller_singledrive_naive.h"

using GSWDStates = grabec::GoldSoloWhistleDriveStates;

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
  void on_pushButton_speedPlus_clicked();
  void on_pushButton_speedMinus_clicked();
  void on_pushButton_torquePlus_clicked();
  void on_pushButton_torqueMinus_clicked();

private slots:
  void enableInterface(const bool op_outcome = false);
  void appendText2Browser(const QString& text);
  void updateEcStatusLED(const Bitfield8& ec_status_flags);
  void handleMotorStatusUpdate(const ID_t&, const grabec::GSWDriveInPdos&motor_status);

private:
  bool ec_network_valid_ = false;

  Ui::MainGUI* ui = NULL;
  CalibrationDialog* calib_dialog_ = NULL;
  HomingDialog* homing_dialog_ = NULL;

  CableRobot robot_;

  bool manual_ctrl_enabled_ = false;
  bool waiting_for_response_ = false;
  ID_t motor_id_;
  ControllerSingleDriveNaive* man_ctrl_ptr_;

  void DisablePosCtrlButtons(const bool value);
  void DisableVelCtrlButtons(const bool value);
  void DisableTorqueCtrlButtons(const bool value);

  void UpdateDriveStatusTable(const grabec::GSWDriveInPdos& status);
  void UpdateDriveCtrlPanel(const Actuator::States state);

  bool ExitReadyStateRequest();

private:
  void SetupDirectMotorCtrl(const bool enable);
  Actuator::States DriveStateToActuatorState(const GSWDStates drive_state);
};

#endif // MAIN_GUI_H
