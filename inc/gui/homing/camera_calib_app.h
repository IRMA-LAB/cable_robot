/**
 * @file camera_calib_app.h
 * @author Marco Caselli
 * @date 02 Jul 2019
 * @brief File containing camera calibration class to found intrinsic parameter and
 * distorsion coefficients of camera
 */

#ifndef CABLE_ROBOT_HOMING_CAMERA_CALIB_APP_H
#define CABLE_ROBOT_HOMING_CAMERA_CALIB_APP_H

#include <QDialog>
#include <QFileDialog>
#include <QMessageBox>
#include <QMutex>
#include <QThread>
#include <QWaitCondition>
#include <QWidget>

#include "clocks.h"
#include "opencv2/opencv.hpp"

#include "utils/cameraparamsjsonparser.h"
#include "utils/types.h"

/**
 * @brief compute camera intrinsic matrix and distorsion coefficients
 */
class WorkerThread: public QThread
{
  Q_OBJECT
 public:
  /**
   * @brief WorkerThread constructor
   * @param settings
   */
  WorkerThread(const CameraCalibSettings& settings);

  void setNewFrame(const cv::Mat& frame);
  void stop();

 signals:
  void calibStateUpdate(const QString&) const;
  void calibFrameCaptured(const int&, const int&) const;
  void resultReady(const CameraParams&) const;
  void processFrameError() const;

 private:
  enum CalibMode
  {
    DETECTION  = 0,
    CAPTURING  = 1,
    CALIBRATED = 2
  };

  static constexpr uint kErrorX_ =
    180U; // sum min difference of x coordinate of 2 frames considering only first row
  static constexpr uint kErrorY_ =
    30U; // sum min difference of y coordinate of 2 frames considering only first col

  CameraCalibSettings settings_;
  cv::Mat latest_frame_;

  QMutex mutex_;
  QWaitCondition new_frame_available_;

  std::vector<cv::Point2f> point_buf_;
  std::vector<std::vector<cv::Point2f>> image_points_;
  CameraParams camera_params_;
  cv::Size image_size_;
  CalibMode calib_mode_;
  bool stop_;
  bool new_frame_pending_;

  void run();
  bool processFrame(const cv::Mat& frame);

  bool storeValidFrame(const cv::Mat& view);
  bool findChessboard(const cv::Mat& image);
  bool catchCalibrationSample(const cv::Mat& image);
  bool compareFrameAgainstPrev();

  bool runCalibration();
  bool computeCameraParams();
  double
  computeCalibReprojectionErr(const std::vector<std::vector<cv::Point3f>>& object_points,
                              const std::vector<cv::Mat>& rvecs,
                              const std::vector<cv::Mat>& tvecs,
                              std::vector<float>& per_view_errors) const;

  // TODO: move somewhere else
  cv::Mat getUndistortedImage(const cv::Mat& image);
};


namespace Ui {
class CameraCalibApp;
}

class CameraCalibApp: public QDialog
{
  Q_OBJECT

 public:
  explicit CameraCalibApp(QWidget* parent = nullptr);
  ~CameraCalibApp();

  void start(const CameraCalibSettings& settings);
  void setNewFrame(const cv::Mat& frame);

 signals:
  void calibrationFailed() const;
  void calibrationSuccess(const CameraParams& params) const;

 private slots:
  void on_pushButton_stop_clicked();

 private slots:
  void updateProgressBar(const uint& current, const size_t& total);
  void setProgressLabel(const QString& text);
  void handleResults(const CameraParams& params);
  void handleErrors();
  void workFinished();

 private:
  Ui::CameraCalibApp* ui;
  WorkerThread* worker_thread_ = nullptr;

  void saveCameraParams(const CameraParams& params);
};


#endif // CABLE_ROBOT_HOMING_CAMERA_CALIB_APP_H
