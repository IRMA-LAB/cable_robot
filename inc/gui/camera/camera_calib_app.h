/**
 * @file camera_calib_app.h
 * @author Simone Comari, Marco Caselli
 * @date 08 Jul 2019
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
 * @brief A thread that takes care of heavy computational loads from CameraCalibApp.
 *
 * This parallel thread continuosly reads latest frame captured by a camera, process it
 * and, if valid, adds it to its calibration samples stack. Once the stack is full, camera
 * intrinsic matrix and distorsion coefficients are evaluated and emitted and the thread
 * terminates.
 */
class WorkerThread: public QThread
{
  Q_OBJECT
 public:
  /**
   * @brief Constructor.
   * @param[in] settings Calibration settings that affects the calibration process and
   * type.
   */
  WorkerThread(const CameraCalibSettings& settings);

  /**
   * @brief Set the latest frame captured by the camera.
   * @param[in] frame The latest frame captured by the camera.
   * @note This function is called by CameraCalibApp every time a new frame is available.
   * The new frame overwrites the previous one, resembling a LIFO 1-element stack.
   */
  void setNewFrame(const cv::Mat& frame);
  /**
   * @brief User stop command, to interrupt the calibration process before completion.
   */
  void stop();

 signals:
  /**
   * @brief Signal notifying a state transition in calibration procedure.
   */
  void calibStateUpdate(const QString&) const;
  /**
   * @brief Signal carrying the current number of collected image samples and the total
   * target one.
   * @param[in] samples_collected Current number of image samples collected so far.
   * @param[in] tot_samples Total number of samples to collect to perform the calibration.
   */
  void calibFrameCaptured(const quint64 samples_collected,
                          const quint64 tot_samples) const;
  /**
   * @brief Signal carrying the newly computed calibration results.
   * @param[in] params The newly computed calibration results, i.e. the camera parameters.
   */
  void resultReady(const CameraParams& params) const;
  /**
   * @brief Signal notifying an error in the frame processing. This is also emitted upon
   * user's stop request.
   */
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

/**
 * @brief The implementation of the calibration procedure.
 *
 * This class takes incoming frames captured by a camera and process them until enough
 * samples are collected to perform a calibration.
 * The heavy computational loads are left to a parellel worker thread that start its cycle
 * until task is complete, that is calibration is finished.
 * This class mainly takes care of forwarding information in and out and controlling the
 * task execution.
 */
class CameraCalibApp: public QDialog
{
  Q_OBJECT

 public:
  /**
   * @brief Constructor.
   * @param parent The parent object, in this case the corresponding widget class.
   */
  explicit CameraCalibApp(QWidget* parent = nullptr);
  ~CameraCalibApp();

  /**
   * @brief Start a parallel thread that execute the calibration process.
   * @param[in] settings The calibration settings that affect the calibration process and
   * type.
   */
  void start(const CameraCalibSettings& settings);
  /**
   * @brief Set the latest frame captured by a camera.
   * @param[in] frame The latest frame captured by a camera.
   * @note This function is called every time a new frame is available. The new frame
   * overwrites the previous one, resembling a LIFO 1-element stack.
   */
  void setNewFrame(const cv::Mat& frame);

 signals:
  /**
   * @brief Signal notifying a failure in the camera calibration process. This can also be
   * the result of a user's stop request.
   */
  void calibrationFailed() const;
  /**
   * @brief Signal carrying the newly computed calibration results, i.e. the camera
   * parameters.
   * @param[in] params The newly computed calibration results, i.e. the camera parameters.
   */
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
