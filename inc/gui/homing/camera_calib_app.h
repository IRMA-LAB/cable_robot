#ifndef CABLE_ROBOT_HOMING_CAMERA_CALIB_APP_H
#define CABLE_ROBOT_HOMING_CAMERA_CALIB_APP_H

#include <QMutex>
#include <QThread>
#include <QWaitCondition>
#include <QWidget>
#include <QDialog>
#include <QMessageBox>
#include <QFileDialog>

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
