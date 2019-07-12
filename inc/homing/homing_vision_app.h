/**
 * @file homing_vision_app.h
 * @author Marco Caselli, Simone Comari
 * @date 08 Jul 2019
 * @brief File containing homing vision structure and class to find rotation matrix and
 * traslation vector between camera and chessboard reference systems.
 */

#ifndef CABLE_ROBOT_HOMING_VISION_APP_H
#define CABLE_ROBOT_HOMING_VISION_APP_H

#include <QObject>

#include "opencv2/opencv.hpp"

#include "robot/cablerobot.h"

/**
 * @brief Structure including HomingVisionApp constant parameters.
 */
struct HomingVisionParams
{
  cv::Size pattern_size = cv::Size(9, 6); /**< Corner chessboard dimension. */
  /**
   * @brief Max precision of corner position
   * Max precision of corner position in sub pixel detection;
   * when the corner position moves by less than max_precision. This value must be less or
   * equal to max_precision in CameraCalibSettings defined in types.h
   */
  double max_precision = 0.0001;
  float square_size    = 26.f; /**< Square chessboard dimension in [mm] */
  int chess_board_flags =
    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE |
    cv::CALIB_CB_FAST_CHECK; /**< Flags usefull to findChessboard function */
  int cor_sp_size = 11;      /**< Windows dimension for sub pixel accurate location  */
  int zero_zone   = -1;      /**< Half diemsnion of zero-zone */
  /**
   * @brief Number of max iteration
   * Number of max iteration to computer corner position in sub pixel detection.  This
   * value must be less or equal to max_counter in CameraCalibSettings defined in
   * types.h
   */
  int max_counter = 30;
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
class HomingVisionApp: public QObject
{
  Q_OBJECT

 public:
  /**
   * @brief HomingVisionApp explicit constructor.
   * @param parent The parent Qt object, in this case the corresponding interface.
   * @param robot Pointer to cable robot instance, to access robot features and commands.
   */
  explicit HomingVisionApp(QObject* parent, CableRobot* robot);

  /**
   * @brief Get rotation matrix between camera and chessboard reference systems.
   * @return Latest rotation matrix between camera frame and chessboard frame.
   */
  cv::Mat getRotationMatrix() const { return R_; }
  /**
   * @brief Get translation vector between camera and chessboard reference systems.
   * @return Latest translation vector between camera and chessboard reference systems.
   */
  cv::Mat getTranslationVector() const { return tvec_; }

  /**
   * @brief Compute rotation matrix and translation vector between camera and chessboard
   * reference systems from a captured image framing a chessboard of known size and
   * format.
   */
  void elaborate();

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
  /**
   * @brief Signal carrying a rotation matrix and a translation vector between camera and
   * chessboard reference systems.
   * @param rot_mat The rotation matrix between camera and chessboard reference systems.
   * @param transl_vect The translation vector between camera and chessboard reference
   * systems.
   */
  void poseReady(const cv::Mat& rot_mat, const cv::Mat& transl_vect);

 public slots:
  /**
   * @brief getNewVideoFrame
   * @param frame The latest frame captured by the camera.
   */
  void getNewVideoFrame(const cv::Mat& frame);
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
  cv::Mat frame_;
  cv::Mat processed_frame_;
  bool new_frame_available_;

  cv::Mat R_;
  cv::Mat tvec_;
  std::vector<cv::Point3f> object_points_;
  std::vector<cv::Point2f> object_points_planar_;
  std::vector<cv::Point2f> image_points_;

  bool poseEstimationFromCoplanarPoints();
  void calcChessboardCorners(std::vector<cv::Point3f>& corners);
  bool calcImageCorner();
  void showAugmentedFrame();
};

#endif // CABLE_ROBOT_HOMING_VISION_APP_H
