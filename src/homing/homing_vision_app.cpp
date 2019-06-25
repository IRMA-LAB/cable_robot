#include "homing/homing_vision_app.h"

HomingVisionApp::HomingVisionApp(QObject* parent, CableRobot* robot)
  : QObject(parent), robot_ptr_(robot), new_frame_available_(false)
{}

//--------- Public slots  ------------------------------------------------------------//

void HomingVisionApp::getNewVideoFrame(const cv::Mat& frame)
{
  if (frame.empty())
    return;

  mutex_.lock();
  frame_               = frame;
  new_frame_available_ = true;
  mutex_.unlock();
}

// aggiungo
void HomingVisionApp::elaborate()
{
  mutex_.lock();
  if (!new_frame_available_)
  {
    mutex_.unlock();
    return;
  }
  processed_frame_     = frame_;
  new_frame_available_ = false;
  mutex_.unlock();

  if (poseEstimationFromCoplanarPoints())
  {
    showAugmentedFrame();
    emit poseReady(R_, tvec_);
  }
}
// chiudere la connessione con la telecamera... probabilmente è gia implementata
// void HomingVisionApp::stopShow() { video_capture_.release(); }

void HomingVisionApp::calcChessboardCorners(std::vector<cv::Point3f>& corners)
{
  for (int i = 0; i < settings_.pattern_size.height; i++)
    for (int j = 0; j < settings_.pattern_size.width; j++)
      corners.push_back(
        cv::Point3f(j * settings_.square_size, i * settings_.square_size, 0));
}

bool HomingVisionApp::calcImageCorner()
{
  std::vector<cv::Point2f> corners;
  object_points_.clear();
  object_points_planar_.clear();
  image_points_.clear();

  bool found = cv::findChessboardCorners(processed_frame_, settings_.pattern_size,
                                         corners, settings_.chess_board_flags);
  if (!found)
  {
    emit printToQConsole("WARNING: Could not find chessboard");
    return false;
  }

  cv::Mat view_gray;
  cv::cvtColor(processed_frame_, view_gray, cv::COLOR_BGR2GRAY);
  cornerSubPix(view_gray, corners, cv::Size(settings_.cor_sp_size, settings_.cor_sp_size),
               cv::Size(settings_.zero_zone, settings_.zero_zone),
               cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
                                settings_.max_counter, settings_.max_precision));

  calcChessboardCorners(object_points_);

  for (size_t i = 0; i < object_points_.size(); i++)
  {
    object_points_planar_.push_back(
      cv::Point2f(object_points_[i].x, object_points_[i].y));
  }

  if (static_cast<uint>(camera_params_.dist_coeff.rows) == 4)
    cv::fisheye::undistortPoints(corners, image_points_, camera_params_.camera_matrix,
                                 camera_params_.dist_coeff);
  else
    cv::undistortPoints(corners, image_points_, camera_params_.camera_matrix,
                        camera_params_.dist_coeff);
  return true;
}

bool HomingVisionApp::poseEstimationFromCoplanarPoints()
{
  if (!calcImageCorner())
    return false;

  cv::Mat H = cv::findHomography(object_points_planar_, image_points_);

  // Normalization to ensure that ||c1|| = 1
  double norm = sqrt(H.at<double>(0, 0) * H.at<double>(0, 0) +
                     H.at<double>(1, 0) * H.at<double>(1, 0) +
                     H.at<double>(2, 0) * H.at<double>(2, 0));
  H /= norm;
  cv::Mat c1 = H.col(0);
  cv::Mat c2 = H.col(1);
  cv::Mat c3 = c1.cross(c2);
  R_         = cv::Mat(3, 3, CV_64F);
  tvec_      = H.col(2);

  for (int i = 0; i < 3; i++)
  {
    R_.at<double>(i, 0) = c1.at<double>(i, 0);
    R_.at<double>(i, 1) = c2.at<double>(i, 0);
    R_.at<double>(i, 2) = c3.at<double>(i, 0);
  }

  cv::Mat W, U, Vt;
  cv::SVDecomp(R_, W, U, Vt);
  R_              = U * Vt;
  QString message = QString("Homing vision done \n matrix rotation: %1  %2  %3 \n %4  %5 "
                            " %6 \n %7  %8  %9 \n translation vector: %10  %11  %12")
                      .arg(R_.at<double>(0, 0))
                      .arg(R_.at<double>(0, 1))
                      .arg(R_.at<double>(0, 2))
                      .arg(R_.at<double>(1, 0))
                      .arg(R_.at<double>(1, 1))
                      .arg(R_.at<double>(1, 2))
                      .arg(R_.at<double>(2, 0))
                      .arg(R_.at<double>(2, 1))
                      .arg(R_.at<double>(2, 2))
                      .arg(tvec_.at<double>(0, 0))
                      .arg(tvec_.at<double>(1, 0))
                      .arg(tvec_.at<double>(2, 0));
  emit printToQConsole(message);
  return true;
}

void HomingVisionApp::showAugmentedFrame()
{
  cv::Mat rvec;
  cv::Rodrigues(R_, rvec);
  cv::drawFrameAxes(processed_frame_, camera_params_.camera_matrix,
                    camera_params_.dist_coeff, rvec, tvec_, 2 * settings_.square_size);
  emit frameReadyToShow(processed_frame_);
}

void HomingVisionApp::setCameraParams(const CameraParams& params)
{
  camera_params_ = params;
}
