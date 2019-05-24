#ifndef CABLE_ROBOT_HOMING_CAMERA_CALIB_APP_H
#define CABLE_ROBOT_HOMING_CAMERA_CALIB_APP_H

#include <QDialog>
#include <QFileDialog>
#include <QMessageBox>
#include <QMutex>
#include <QThread>
#include <QWaitCondition>
#include <QWidget>

#include "opencv2/opencv.hpp"

#include "utils/types.h"

class WorkerThread: public QThread
{
  Q_OBJECT
 public:
  WorkerThread(const CameraCalibSettings& settings);

  void setNewFrame(const cv::Mat& frame);
  void stop();

 signals:
  void calibStateUpdate(const QString&) const;
  void calibFrameCaptured(const int&, const int&) const;
  void resultReady(const CameraParams&) const;
  void processFrameError() const;

 private:
  CameraCalibSettings settings_;
  cv::Mat latest_frame_;
  bool new_frame_pending_;
  bool stop_;

  QMutex mutex_;
  QWaitCondition new_frame_available_;

  void run();
  bool processFrame(const cv::Mat& frame);


  enum CalibMode
  {
    DETECTION  = 0,
    CAPTURING  = 1,
    CALIBRATED = 2
  };
  std::vector<cv::Point2f> point_buf_;
  std::vector<std::vector<cv::Point2f>> image_points_;
  CameraParams camera_params_;
  cv::Size image_size_;

  // Mettere qua probabilmente mode = CAPTURING
  CalibMode calib_mode_ = CAPTURING;

  clock_t prev_timestamp_ = 0;

  double error_x_ = 180;
  double error_y_ = 30;
  uint counter_x_ = 0;
  uint counter_y_ = 0;
  uint delta_x_   = 0;
  uint delta_y_   = 0;
  uint counter_   = 0;

  bool checkFrameAgainstPrev();

  bool runCalibrationAndSave(CameraParams & params);

  bool runCalibration(std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs,
                      std::vector<float>& reproj_errs, double& total_avg_err,
                      std::vector<cv::Point3f>& new_obj_points);

  double
  computeCalibReprojectionErr(const std::vector<std::vector<cv::Point3f>>& object_points,
                              const std::vector<cv::Mat>& rvecs,
                              const std::vector<cv::Mat>& tvecs,
                              std::vector<float>& per_view_errors) const;

  bool isCalibrated() const { return calib_mode_ == CalibMode::CALIBRATED; }

  cv::Mat getUndistortedImage(const cv::Mat &image);

  bool findChessboard(const cv::Mat &image);

  bool catchCalibrationSample(const cv::Mat &image);

  int getCurrentSamplesNum() const {
    return static_cast<int>(image_points_.size());
  }

  bool storeValidFrame(const cv::Mat & view);

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
