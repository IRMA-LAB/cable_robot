#ifndef CABLE_ROBOT_HOMING_VISION_APP_H
#define CABLE_ROBOT_HOMING_VISION_APP_H

#include <QObject>

#include "opencv2/opencv.hpp"

#include "robot/cablerobot.h"

class HomingVisionApp: public QObject
{
  Q_OBJECT
 public:
  explicit HomingVisionApp(QObject* parent, CableRobot* robot);

  void setCameraParams(const CameraParams& params) { calib_params_ = params; }

 signals:
  void printToQConsole(const QString&) const;

 public slots:
  void getNewVideoFrame(const cv::Mat& frame);

 private:
  CableRobot* robot_ptr_ = NULL;
  CameraParams calib_params_;

  QMutex mutex_;
  cv::Mat frame_;
  bool new_frame_available_;
};

#endif // CABLE_ROBOT_HOMING_VISION_APP_H
