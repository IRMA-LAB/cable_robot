#ifndef CAMERA_CALIB_WINDOW_H
#define CAMERA_CALIB_WINDOW_H

#include <QDialog>

#include "utils/types.h"

namespace Ui {
class CameraCalibSettingsDialog;
}

class CameraCalibSettingsDialog: public QDialog
{
  Q_OBJECT

 public:
  explicit CameraCalibSettingsDialog(QWidget* parent = nullptr);
  ~CameraCalibSettingsDialog();

 signals:
  void cameraCalibSettings(const CameraCalibSettings&);

 private slots:
  void on_pushButton_advancedSettings_clicked();

  void on_pushButton_start_clicked();
  void on_pushButton_cancel_clicked();

 private:
  Ui::CameraCalibSettingsDialog* ui;

  CameraCalibSettings settings_;
};

#endif // CAMERA_CALIB_WINDOW_H
