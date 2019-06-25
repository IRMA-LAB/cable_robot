#ifndef CABLE_ROBOT_HOMING_VISION_APP_H
#define CABLE_ROBOT_HOMING_VISION_APP_H

#include <QObject>

#include "opencv2/opencv.hpp"

#include "robot/cablerobot.h"

/**
 * @brief Struct with HomingVisionApp constant parameters.
 */
struct HomingVisionParams
{
  cv::Size pattern_size = cv::Size(9, 6); /*<< ... corner chessboard dimension*/
  /**
   * @brief square chessboard dimension
   */
  float square_size = 0.026f;
  /**
   * @brief flags usefull to findchessboard function
   */
  int chess_board_flags =
    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK;
  int cor_sp_size = 11; /*<< ... windows dimension for sub pixel accurate location  */
  int zero_zone   = -1; /*<< ... half diemsnion of zero-zone */
  int max_counter = 30; /*<< ... number of max iteration*/
  /**
   * @brief number of max precision. thet value have to be less than value in camera calibration
   */
  double max_precision = 0.0001;

};


class HomingVisionApp: public QObject
{
  Q_OBJECT

 public:
  explicit HomingVisionApp(QObject* parent, CableRobot* robot);

  cv::Mat getRotationMatrix() const { return R_; }
  cv::Mat getTranslationVector() const { return tvec_; }

  void elaborate();
  void stopShow();

 signals:
  void printToQConsole(const QString&) const;
  void frameReadyToShow(const cv::Mat&);
  void poseReady(const cv::Mat&, const cv::Mat&);

 public slots:
  void getNewVideoFrame(const cv::Mat& frame);
  void setCameraParams(const CameraParams& params);

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

  void calcChessboardCorners(std::vector<cv::Point3f>& corners);
  bool poseEstimationFromCoplanarPoints();
  void showAugmentedFrame();
  bool calcImageCorner();
};

#endif // CABLE_ROBOT_HOMING_VISION_APP_H
