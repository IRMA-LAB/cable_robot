/**
 * @file camera_calib_dialog.h
 * @author Simone Comari
 * @date 12 Jul 2019
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

#include "gui/camera/camera_calib_app.h"
#include "gui/camera/camera_calib_settings_dialog.h"

using json = nlohmann::json; /**< Alias for JSON library support. */

namespace Ui {
class CameraCalibDialog;
}

enum CalibrationStatus : uchar
{
  OFF = 0,
  ON = 1
};

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
  explicit CameraCalibDialog(QWidget* parent, const CameraParams default_params);
  ~CameraCalibDialog() override;

 signals:
  /**
   * @brief calibParamsReady
   */
  void cameraParamsReady(const CameraParams&) const;
  /**
   * @brief printToQConsole
   */
  void printToQConsole(const QString&) const;
  /**
   * @brief augmentedFrameAvailable
   * @param augmented_frame
   */
  void augmentedFrameAvailable(const cv::Mat& augmented_frame) const;
  /**
   * @brief calibrationStatusChanged
   * @param status
   */
  void calibrationStatusChanged(const CalibrationStatus status);

 public slots:
  void setNewVideoFrame(const cv::Mat&);

 private slots:
  void startCalibration(const CameraCalibSettings& settings);
  void handleCalibrationFailure();
  void handleCalibrationSuccess(const CameraParams& params);
  void frwAugmentedFrame(const cv::Mat& augmented_frame) const;
  void frwPrintToQConsole(const QString& msg) const;

 private slots:
  void closeEvent(QCloseEvent* event) override final;

  void on_pushButton_newCalib_clicked();
  void on_pushButton_load_clicked();
  void on_pushButton_loadDefault_clicked();

 private:
  static const QString kDefaultCalibFile_;

  Ui::CameraCalibDialog* ui;
  CameraCalibSettingsDialog settings_win_;
  CameraCalibApp app_;

  CameraParams default_camera_params_;
  CameraParams camera_params_;

  bool parseCalibFile(
    const QString& filepath); // dummy, implement once we know calib params and file
};

#endif // CABLE_ROBOT_HOMING_CAMERA_CALIB_DIALOG_H
