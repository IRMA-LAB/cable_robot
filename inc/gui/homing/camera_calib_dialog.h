/**
 * @file camera_calib_dialog.h
 * @author Simone Comari
 * @date 22 Mar 2019
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
  void calibParamsReady(const CalibParams&) const;
  /**
   * @brief printToQConsole
   */
  void printToQConsole(const QString&) const;

 public slots:
  /**
   * @brief getNewVideoFrame
   * @param frame
   */
  void getNewVideoFrame(const cv::Mat& frame);

 private slots:
  void on_pushButton_newCalib_clicked();

  void on_pushButton_load_clicked();

  void on_pushButton_loadDefault_clicked();

 private:
  static const QString kDefaultCalibFile_;

  Ui::CameraCalibDialog* ui;
  CalibParams calib_params_;

  QMutex mutex_;
  cv::Mat frame_;
  bool new_frame_available_;

  bool parseCalibFile(
    const QString& filepath); // dummy, implement once we know calib params and file
};

#endif // CABLE_ROBOT_HOMING_CAMERA_CALIB_DIALOG_H
