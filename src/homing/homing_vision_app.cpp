#include "homing/homing_vision_app.h"

HomingVisionApp::HomingVisionApp(QObject* parent, CableRobot* robot) : QObject(parent), robot_ptr_(robot), new_frame_available_(false) {}

//--------- Public slots  ------------------------------------------------------------//

void HomingVisionApp::getNewVideoFrame(const cv::Mat& frame)
{
  mutex_.lock();
  frame_ = frame;
  new_frame_available_ = true;
  mutex_.unlock();
}
