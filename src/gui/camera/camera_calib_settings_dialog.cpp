/**
 * @file camera_calib_settings_dialog.cpp
 * @author Simone Comari
 * @date 16 Jul 2019
 * @brief This file includes definitions of class present in
 * camera_calib_settings_dialog.h.
 */

#include "gui/camera/camera_calib_settings_dialog.h"
#include "ui_camera_calib_settings_dialog.h"

CameraCalibSettingsDialog::CameraCalibSettingsDialog(QWidget* parent)
  : QDialog(parent), ui(new Ui::CameraCalibSettingsDialog)
{
  ui->setupUi(this);
  // Setup default values for advanced settings
  ui->checkBox_tanDist->setChecked(!settings_.calib_zero_tan_dist);
  ui->spinBox_zeroZone->setValue(settings_.zero_zone);
  ui->spinBox_corSpSize->setValue(settings_.cor_sp_size);
  ui->spinBox_boardSizeW->setValue(settings_.board_size.width);
  ui->spinBox_boardSizeH->setValue(settings_.board_size.height);
  ui->spinBox_maxCounter->setValue(settings_.max_counter);
  ui->doubleSpinBox_delay->setValue(settings_.delay);
  ui->doubleSpinBox_squareSize->setValue(settings_.square_size);
  ui->doubleSpinBox_maxPrecision->setValue(settings_.max_precision);
  ui->widget_advancedSettings->hide();
}

CameraCalibSettingsDialog::~CameraCalibSettingsDialog() { delete ui; }

void CameraCalibSettingsDialog::on_pushButton_advancedSettings_clicked()
{
  if (ui->widget_advancedSettings->isHidden())
  {
    ui->widget_advancedSettings->show();
    ui->pushButton_advancedSettings->setText("Hide Advanced Settings...");
  }
  else
  {
    ui->widget_advancedSettings->hide();
    ui->pushButton_advancedSettings->setText("Show Advanced Settings...");
  }
}

void CameraCalibSettingsDialog::on_pushButton_start_clicked()
{
  settings_.num_frames          = static_cast<size_t>(ui->spinBox_numFrames->value());
  settings_.use_fisheye         = ui->checkBox_fisheye->isChecked();
  settings_.calib_zero_tan_dist = !ui->checkBox_tanDist->isChecked();
  settings_.delay               = ui->doubleSpinBox_delay->value();
  settings_.square_size       = static_cast<float>(ui->doubleSpinBox_squareSize->value());
  settings_.max_precision     = ui->doubleSpinBox_maxPrecision->value();
  settings_.zero_zone         = ui->spinBox_zeroZone->value();
  settings_.cor_sp_size       = ui->spinBox_corSpSize->value();
  settings_.board_size.width  = ui->spinBox_boardSizeW->value();
  settings_.board_size.height = ui->spinBox_boardSizeH->value();
  settings_.max_counter       = ui->spinBox_maxCounter->value();

  emit cameraCalibSettings(settings_);
  close();
}

void CameraCalibSettingsDialog::on_pushButton_cancel_clicked()
{
  emit cancelClicked();
  close();
}
