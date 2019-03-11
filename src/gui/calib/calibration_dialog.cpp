/**
 * @file calibration_dialog.cpp
 * @author Simone Comari
 * @date 11 Mar 2019
 * @brief This file includes definitions of class present in calibration_dialog.h.
 */

#include "gui/calib/calibration_dialog.h"
#include "ui_calibration_dialog.h"


CalibrationDialog::CalibrationDialog(QWidget* parent, CableRobot* robot)
  : QDialog(parent), ui(new Ui::CalibrationDialog), robot_(robot)
{
  ui->setupUi(this);
}

CalibrationDialog::~CalibrationDialog()
{
  delete ui;
  CLOG(INFO, "event") << "Calibration dialog closed";
}

//--------- Private GUI slots -------------------------------------------------------//

void CalibrationDialog::on_buttonBox_accepted()
{
  emit calibrationEnd();
  emit enableMainGUI();
  close();
}

void CalibrationDialog::on_buttonBox_rejected()
{
  emit calibrationEnd();
  emit enableMainGUI();
  close();
}
