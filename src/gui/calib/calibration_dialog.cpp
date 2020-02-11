/**
 * @file calibration_dialog.cpp
 * @author Simone Comari
 * @date 10 Jan 2020
 * @brief This file includes definitions of class present in calibration_dialog.h.
 */

#include "gui/calib/calibration_dialog.h"
#include "ui_calibration_dialog.h"


CalibrationDialog::CalibrationDialog(QWidget* parent, CableRobot* robot,
                                     const grabcdpr::RobotParams& params)
  : QDialog(parent), ui(new Ui::CalibrationDialog), robot_ptr_(robot), params_(params)
{
  ui->setupUi(this);
}

CalibrationDialog::~CalibrationDialog()
{
  if (interface_ != nullptr)
    interface_->close();
  delete ui;
  CLOG(INFO, "event") << "Calibration dialog closed";
}

//--------- Private GUI slots -------------------------------------------------------//

void CalibrationDialog::on_buttonBox_accepted()
{
  interface_ =
    new CalibInterfaceExcitation(parentWidget(), robot_ptr_, params_.actuators);
  connect(interface_, SIGNAL(destroyed()), this, SLOT(fwdCalibFinished()));
  interface_->show();
  CLOG(INFO, "event") << "Prompt " << ui->comboBox_calibMethod->currentText()
                      << " calibration interface";
  hide();
  CLOG(INFO, "event") << "Hide calibration dialog";
}

void CalibrationDialog::on_buttonBox_rejected() { fwdCalibFinished(); }

//--------- Private slots -----------------------------------------------------------//

void CalibrationDialog::fwdCalibFinished()
{
  CLOG(INFO, "event") << ui->comboBox_calibMethod->currentText()
                      << " calibration finished";
  emit calibrationEnd();
  emit enableMainGUI(false);
  interface_ = nullptr;
  close();
}
