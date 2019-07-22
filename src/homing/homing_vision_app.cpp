/**
 * @file homing_vision_app.cpp
 * @author Marco Caselli, Simone Comari
 * @date 22 Jul 2019
 * @brief This file includes definitions of classes present in homing_vision_app.h.
 */

#include "homing/homing_vision_app.h"
#include "kinematics.h"

HomingVisionApp::HomingVisionApp(QObject* parent, CableRobot* robot,
                                 const VisionParams params)
  : QThread(parent), robot_ptr_(robot), new_frame_pending_(false), stop_(false)
{
  H_b2p = params.H_b2p;
  H_c2w = params.H_c2w;
}

//--------- Public functions ---------------------------------------------------------//

void HomingVisionApp::stop()
{
  mutex_.lock();
  stop_ = true;
  mutex_.unlock();
}

void HomingVisionApp::applyPoseEstimate()
{
  // Calculate platform global pose from camera relative pose wrt to chessboard.
  grabnum::Vector3d position;
  grabnum::Vector3d orientation;
  calcPlatformGlobalPose(position, orientation);
  // Calculate inverse kinematics to get cables lenght and swivel angles from given pose.
  grabcdpr::RobotParams params = robot_ptr_->GetActiveComponentsParams();
  grabcdpr::RobotVars cdpr_vars; // empty container
  cdpr_vars.cables.resize(params.actuators.size());
  // debug
  return;
  grabcdpr::UpdateIK0<grabnum::Vector3d, grabcdpr::RobotVars>(position, orientation,
                                                              &params, &cdpr_vars);
  // Update homing configuration for each cable/pulley.
  vect<id_t> actuators_id = robot_ptr_->GetActiveMotorsID();
  for (uint i = 0; i < actuators_id.size(); i++)
    robot_ptr_->UpdateHomeConfig(actuators_id[i], cdpr_vars.cables[i].length,
                                 cdpr_vars.cables[i].swivel_ang);
}

bool HomingVisionApp::isPoseReady()
{
  mutex_.lock();
  bool pose_ready = pose_ready_;
  mutex_.unlock();
  return pose_ready;
}

//--------- Public slots  ------------------------------------------------------------//

void HomingVisionApp::setNewFrame(const cv::Mat& frame)
{
  if (frame.empty())
    return;

  mutex_.lock();
  latest_frame_      = frame;
  new_frame_pending_ = true;
  new_frame_available_.wakeAll();
  mutex_.unlock();
}

//--------- Private functions --------------------------------------------------------//

void HomingVisionApp::run()
{
  pose_ready_ = false;
  while (1)
  {
    mutex_.lock();
    // Check if stop command was sent
    if (stop_)
    {
      stop_ = false; // reset
      mutex_.unlock();
      break;
    }

    // Grab new frame
    if (!new_frame_pending_)
      new_frame_available_.wait(&mutex_);
    latest_frame_.copyTo(frame_);
    new_frame_pending_ = false;
    mutex_.unlock();

    // Try to estimate platform pose from chessboard
    pose_ready_ = poseEstimationFromCoplanarPoints();
    showAugmentedFrame();
  }
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
  R_b2c      = cv::Mat(3, 3, CV_64F);
  t_b2c      = H.col(2) * 0.001; // convert [mm] to [m]

  for (int i = 0; i < 3; i++)
  {
    R_b2c.at<double>(i, 0) = c1.at<double>(i, 0);
    R_b2c.at<double>(i, 1) = c2.at<double>(i, 0);
    R_b2c.at<double>(i, 2) = c3.at<double>(i, 0);
  }

  cv::Mat W, U, Vt;
  cv::SVDecomp(R_b2c, W, U, Vt);
  R_b2c = U * Vt;
  return true;
}

bool HomingVisionApp::calcImageCorner()
{
  static constexpr int kFisheyeDistCoeffNum = 4;
  static constexpr double kWarningMsgPeriod = 1.0; // [sec]
  static grabrt::Clock clock;

  std::vector<cv::Point2f> corners;
  object_points_.clear();
  object_points_planar_.clear();
  image_points_.clear();

  bool found = cv::findChessboardCorners(frame_, settings_.pattern_size, corners,
                                         settings_.chess_board_flags);
  if (!found)
  {
    if (clock.Elapsed() > kWarningMsgPeriod)
    {
      emit printToQConsole("WARNING: Chessboard not found");
      clock.Reset();
    }
    return false;
  }

  cv::Mat view_gray;
  cv::cvtColor(frame_, view_gray, cv::COLOR_BGR2GRAY);
  cv::cornerSubPix(view_gray, corners,
                   cv::Size(settings_.cor_sp_size, settings_.cor_sp_size),
                   cv::Size(settings_.zero_zone, settings_.zero_zone),
                   cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
                                    settings_.max_counter, settings_.max_precision));

  calcChessboardCorners(object_points_);

  for (size_t i = 0; i < object_points_.size(); i++)
    object_points_planar_.push_back(
      cv::Point2f(object_points_[i].x, object_points_[i].y));

  if (camera_params_.dist_coeff.rows == kFisheyeDistCoeffNum)
    cv::fisheye::undistortPoints(corners, image_points_, camera_params_.camera_matrix,
                                 camera_params_.dist_coeff);
  else
    cv::undistortPoints(corners, image_points_, camera_params_.camera_matrix,
                        camera_params_.dist_coeff);
  return true;
}

void HomingVisionApp::calcChessboardCorners(std::vector<cv::Point3f>& corners)
{
  for (int i = 0; i < settings_.pattern_size.height; i++)
    for (int j = 0; j < settings_.pattern_size.width; j++)
      corners.push_back(
        cv::Point3f(j * settings_.square_size, i * settings_.square_size, 0));
}

void HomingVisionApp::showAugmentedFrame()
{
  cv::Mat augm_frame = frame_.clone();
  if (pose_ready_)
  {
    cv::Mat rvec;
    cv::Rodrigues(R_b2c, rvec);
    cv::drawFrameAxes(augm_frame, camera_params_.camera_matrix, camera_params_.dist_coeff,
                      rvec, t_b2c * 1000, 2 * settings_.square_size);
  }
  emit frameReadyToShow(augm_frame);
}

void HomingVisionApp::calcPlatformGlobalPose(grabnum::Vector3d& position,
                                             grabnum::Vector3d& orientation)
{
  // Frame subscripts: b = chessboard, c = camera, p = platform, w = world
  // 1. Build H_b2c from R_b2c and t_b2c
  grabnum::Matrix4d H_b2c =
    grabgeom::BuildHomogeneousTransf(cv2grabnum<3, 3>(R_b2c), cv2grabnum<3, 1>(t_b2c));
  // 2. Calculate H_p2w = H_p2b * H_b2c * H_c2w
  grabnum::Matrix4d H_p2w = grabgeom::InverseTransformation(H_b2p) * H_b2c * H_c2w;
  // 3. Extract platform pose from H_p2w
  position    = grabgeom::GetHomgTransfTransl(H_p2w); // [m]
  orientation = grabgeom::Rot2EulerXYZ(grabgeom::GetHomgTransfRot(H_p2w)); // [rad]

  emit printToQConsole(QString("Estimated platform position:\n [ %1  %2  %3 ] m\n"
                               "Estimated platform orientation:\n [ %4  %5  %6 ] deg")
                         .arg(position(1))
                         .arg(position(2))
                         .arg(position(3))
                         .arg(orientation(1) * 180.0 / M_PI)
                         .arg(orientation(2) * 180.0 / M_PI)
                         .arg(orientation(3) * 180.0 / M_PI));
}
