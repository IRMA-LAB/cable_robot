/**
 * @file camera_calib_dialog.h
 * @author Simone Comari
 * @date 02 Apr 2019
 * @brief This file includes the camera calibration dialog.
 */

#ifndef CABLE_ROBOT_HOMING_CAMERA_CALIB_DIALOG_H
#define CABLE_ROBOT_HOMING_CAMERA_CALIB_DIALOG_H

#include <QDialog>
#include <QFileDialog>
#include <QMessageBox>
#include <QMutex>

#include "opencv2/opencv.hpp"

#include "easylogging++.h"
#include "json.hpp"

#include "utils/types.h"

#include "gui/homing/camera_calib_app.h"
#include "gui/homing/camera_calib_settings_dialog.h"

using json = nlohmann::json; /**< Alias for JSON library support. */

namespace Ui {
class CameraCalibDialog;
}

/**
 * @brief The CameraCalibDialog class
 */
class CameraCalibDialog: public QDialog
{
  Q_OBJECT

 public:
  /**
   * @brief CameraCalibDialog
   * @param parent
   */
  explicit CameraCalibDialog(QWidget* parent = 0);
  ~CameraCalibDialog();

 signals:
  /**
   * @brief calibParamsReady
   */
  void cameraParamsReady(const CameraParams&) const;
  /**
   * @brief printToQConsole
   */
  void printToQConsole(const QString&) const;

 public slots:
  void getNewVideoFrame(const cv::Mat&);

 private slots:
  void startCalibration(const CameraCalibSettings& settings);
  void handleCalibrationFailure();
  void handleCalibrationSuccess(const CameraParams& params);

 private slots:
  void on_pushButton_newCalib_clicked();
  void on_pushButton_load_clicked();
  void on_pushButton_loadDefault_clicked();

 private:
  static const QString kDefaultCalibFile_;

  Ui::CameraCalibDialog* ui;
  CameraCalibSettingsDialog settings_win_;
  CameraCalibApp app_;

  CameraParams camera_params_;

  bool parseCalibFile(
    const QString& filepath); // dummy, implement once we know calib params and file
};

#endif // CABLE_ROBOT_HOMING_CAMERA_CALIB_DIALOG_H
