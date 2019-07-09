/**
 * @file camera_calib_settings_dialog.cpp
 * @author Simone Comari
 * @date 09 Jul 2019
 * @brief This file includes definitions of class present in
 * camera_calib_settings_dialog.h.
 */

#include "gui/camera/camera_calib_settings_dialog.h"
#include "ui_camera_calib_settings_dialog.h"

CameraCalibSettingsDialog::CameraCalibSettingsDialog(QWidget* parent)
  : QDialog(parent), ui(new Ui::CameraCalibSettingsDialog)
{
  ui->setupUi(this);
}

CameraCalibSettingsDialog::~CameraCalibSettingsDialog() { delete ui; }

void CameraCalibSettingsDialog::on_pushButton_start_clicked()
{
  settings_.num_frames = static_cast<size_t>(ui->spinBox->value());
  //  settings_.use_tangential_dist = ui->checkBox->isChecked();

  emit cameraCalibSettings(settings_);
  close();
}

void CameraCalibSettingsDialog::on_pushButton_cancel_clicked() { close(); }
