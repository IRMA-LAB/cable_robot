/**
 * @file homing_vision_app.h
 * @author Marco Caselli, Simone Comari
 * @date 17 Jul 2019
 * @brief File containing homing vision structure and class to find rotation matrix and
 * traslation vector between camera and chessboard reference systems.
 */

#ifndef CABLE_ROBOT_HOMING_VISION_APP_H
#define CABLE_ROBOT_HOMING_VISION_APP_H

#include <QObject>

#include "opencv2/opencv.hpp"

#include "homogeneous_transf.h"

#include "robot/cablerobot.h"

/**
 * @brief Structure including HomingVisionApp constant parameters.
 */
struct HomingVisionParams
{
  cv::Size pattern_size = cv::Size(9, 6); /**< Corner chessboard dimension. */
  double max_precision =
    0.0001;                 /**< Max precision of corner position in sub pixel detection;
                             * when the corner position moves by less than max_precision.
                             * This value must be less or equal to max_precision in
                             * CameraCalibSettings defined in types.h */
  float square_size = 26.f; /**< Chessboard squares dimension in [mm] */
  int chess_board_flags =
    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE |
    cv::CALIB_CB_FAST_CHECK; /**< Flags usefull to findChessboard function */
  int cor_sp_size = 11;      /**< Windows dimension for sub pixel accurate location */
  int zero_zone   = -1;      /**< Half diemsnion of zero-zone */
  int max_counter = 30; /**< Number of max iteration to computer corner position in sub
                         * pixel detection.  This value must be less or equal to
                         * max_counter in CameraCalibSettings defined in types.h */
};

/**
 * @brief This class implements the homing vision algorithm.
 *
 * This class follows a state machine structure and works jointly with its parent, the
 * corresponding widget HomingProprioceptiveInterface, which triggers all transitions,
 * both manually (from user end) and automatically, like in the acquisition phase.
 *
 * More details about the states and transition events and policy can be found in the
 * relative documentation in a more friendly and schematic format.
 * @todo finish this class
 */
class HomingVisionApp: public QThread
{
  Q_OBJECT

 public:
  /**
   * @brief HomingVisionApp explicit constructor.
   * @param parent The parent Qt object, in this case the corresponding interface.
   * @param robot Pointer to cable robot instance, to access robot features and commands.
   */
  explicit HomingVisionApp(QObject* parent, CableRobot* robot, const VisionParams params);

  /**
   * @brief User stop command, to interrupt the calibration process before completion.
   */
  void stop();
  /**
   * @brief applyPoseEstimate
   */
  void applyPoseEstimate();

  /**
   * @brief isCameraCalibrated
   * @return
   */
  bool isCameraCalibrated() const { return !camera_params_.isEmpty(); }
  /**
   * @brief isPoseReady
   * @return
   */
  bool isPoseReady();

 signals:
  /**
   * @brief Signal including a message to any QConsole, for instance a QTextBrowser.
   */
  void printToQConsole(const QString&) const;
  /**
   * @brief Signal carrying an augmented frame with cartesian axis on the chessboard
   * origin.
   * @param[in] augm_frame An augmented frame with cartesian axis on the chessboard
   * origin.
   */
  void frameReadyToShow(const cv::Mat& augm_frame);

 public slots:
  /**
   * @brief getNewVideoFrame
   * @param frame The latest frame captured by the camera.
   */
  void setNewFrame(const cv::Mat& frame);
  /**
   * @brief Set camera parameters.
   * @param params Camera parameters.
   */
  void setCameraParams(const CameraParams& params) { camera_params_ = params; }

 private:
  CableRobot* robot_ptr_ = nullptr;
  CameraParams camera_params_;
  const HomingVisionParams settings_;

  QMutex mutex_;
  QWaitCondition new_frame_available_;
  cv::Mat latest_frame_;
  cv::Mat frame_;
  bool new_frame_pending_;
  bool stop_;
  bool pose_ready_;

  cv::Mat R_b2c;
  cv::Mat t_b2c;
  std::vector<cv::Point3f> object_points_;
  std::vector<cv::Point2f> object_points_planar_;
  std::vector<cv::Point2f> image_points_;

  grabnum::Matrix4d H_b2p;
  grabnum::Matrix4d H_c2w;

  void run();
  bool poseEstimationFromCoplanarPoints();
  void calcChessboardCorners(std::vector<cv::Point3f>& corners);
  bool calcImageCorner();
  void showAugmentedFrame();

  void calcPlatformGlobalPose(grabnum::Vector3d& position,
                              grabnum::Vector3d& orientation);
};

template <uint rows, uint cols>
/**
 * @brief cv2grabnum
 * @param cv_mat
 * @return
 */
grabnum::MatrixXd<rows, cols> cv2grabnum(cv::Mat& cv_mat)
{
  assert(cv_mat.rows == rows && cv_mat.cols == cols);
  grabnum::MatrixXd<rows, cols> mat;
  for (int i = 0; i < cv_mat.rows; i++)
    for (int j = 0; i < cv_mat.cols; j++)
      mat(static_cast<uint>(i + 1), static_cast<uint>(j + 1)) = cv_mat.at<double>(i, j);
  return mat;
}

#endif // CABLE_ROBOT_HOMING_VISION_APP_H
