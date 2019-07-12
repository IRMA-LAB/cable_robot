/**
 * @file utils/types.h
 * @author Simone Comari
 * @date 22 Mar 2019
 * @brief File containing the implementation of a custom wrapper to log cable robot data
 * employing easylogging++ package.
 */

#ifndef CABLE_ROBOT_TYPES_H
#define CABLE_ROBOT_TYPES_H

#include <cmath>
#include <stdint.h>
#include <stdlib.h>

#include "opencv2/calib3d.hpp"
#include "opencv2/core.hpp"

#include "libgrabec/inc/slaves/goldsolowhistledrive.h"

/**
 * @brief StringBuf typedef.
 */
typedef std::basic_stringbuf<char16_t> StringBuf;

/**
 * @brief A structure including motor status information.
 */
struct MotorStatus
{
  /**
   * @brief MotorStatus default constructor.
   */
  MotorStatus() : id(0), motor_position(0), motor_speed(0), motor_torque(0), op_mode(0) {}
  /**
   * @brief MotorStatus full implicit constructor.
   * @param[in] _id Motor ID.
   * @param[in] drive_pdos GoldSoloWhistle drive input PDOs from which to extract motor
   * status information.
   */
  MotorStatus(const id_t _id, const grabec::GSWDriveInPdos& drive_pdos)
    : id(_id), motor_position(drive_pdos.pos_actual_value),
      motor_speed(drive_pdos.vel_actual_value),
      motor_torque(drive_pdos.torque_actual_value), op_mode(drive_pdos.display_op_mode)
  {}
  /**
   * @brief MotorStatus full explicit constructor.
   * @param[in] _id Motor ID.
   * @param[in] _op_mode Motor operational mode
   * @param[in] motor_pos Motor position in encoder counts.
   * @param[in] _motor_speed Motor velocity in encoder counts/second.
   * @param[in] _motor_torque Motor torque in per thousand nominal points.
   */
  MotorStatus(const id_t _id, const int8_t _op_mode, const int32_t motor_pos,
              const int32_t _motor_speed, const int16_t _motor_torque)
    : id(_id), motor_position(motor_pos), motor_speed(_motor_speed),
      motor_torque(_motor_torque), op_mode(_op_mode)
  {}

  id_t id;                /**< Motor ID. */
  int32_t motor_position; /**< Motor position in encoder counts.*/
  int32_t motor_speed;    /**< Motor velocity in encoder counts/second.*/
  int16_t motor_torque;   /**< Motor torque in per thousand nominal points.*/
  int8_t op_mode;         /**< Motor operational mode*/
};

/**
 * @brief A structure including winch status information.
 *
 * Winch status is an extension of motor status, with the additional information about
 * cable length.
 */
struct WinchStatus: MotorStatus
{
  /**
   * @brief WinchStatus default constructor.
   */
  WinchStatus() : aux_position(0), cable_length(0.0) {}
  /**
   * @brief WinchStatus full implicit constructor.
   * @param[in] _id Motor ID.
   * @param[in] drive_pdos GoldSoloWhistle drive input PDOs from which to extract motor
   * status information.
   * @param[in] cable_len Cable length in meters.
   */
  WinchStatus(const id_t _id, const grabec::GSWDriveInPdos& drive_pdos,
              const double cable_len = 0.0)
    : MotorStatus(_id, drive_pdos), aux_position(drive_pdos.aux_pos_actual_value),
      cable_length(cable_len)
  {}
  /**
   * @brief WinchStatus full explicit constructor.
   * @param[in] _id Motor ID.
   * @param[in] _op_mode Motor operational mode.
   * @param[in] motor_pos Motor position in encoder counts.
   * @param[in] _motor_speed Motor velocity in encoder counts/second.
   * @param[in] _motor_torque Motor torque in per thousand nominal points.
   * @param[in] cable_len Cable length in meters.
   * @param[in] aux_pos Auxiliary encoder position. This corresponds to swivel pulley
   * encoder value for our case.
   */
  WinchStatus(const id_t _id, const int8_t _op_mode, const int32_t motor_pos,
              const int32_t _motor_speed, const int16_t _motor_torque,
              const double cable_len, const int aux_pos)
    : MotorStatus(_id, _op_mode, motor_pos, _motor_speed, _motor_torque),
      aux_position(aux_pos), cable_length(cable_len)
  {}

  int aux_position;    /**< Cable length in meters.*/
  double cable_length; /**< Auxiliary encoder position. */
};

/**
 * @brief A structure including actuator status information.
 *
 * Actuator status is an extension of winch status, with the additional information about
 * swivel pulleys system.
 */
struct ActuatorStatus: WinchStatus
{
  /**
   * @brief ActuatorStatus default constructor.
   */
  ActuatorStatus() : pulley_angle(0.0) {}
  /**
   * @brief ActuatorStatus full implicit constructor.
   * @param[in] _id Motor ID.
   * @param[in] drive_pdos GoldSoloWhistle drive input PDOs from which to extract motor
   * status information.
   * @param[in] cable_len Cable length in meters.
   * @param[in] pulley_ang Swivel pulley angle in radians.
   */
  ActuatorStatus(const id_t _id, const grabec::GSWDriveInPdos& drive_pdos,
                 const double cable_len = 0.0, const double pulley_ang = 0.0)
    : WinchStatus(_id, drive_pdos, cable_len), pulley_angle(pulley_ang)
  {}
  /**
   * @brief ActuatorStatus full explicit constructor.
   * @param[in] _id Motor ID.
   * @param[in] _op_mode Motor operational mode.
   * @param[in] motor_pos Motor position in encoder counts.
   * @param[in] _motor_speed Motor velocity in encoder counts/second.
   * @param[in] _motor_torque Motor torque in per thousand nominal points.
   * @param[in] cable_len Cable length in meters.
   * @param[in] aux_pos Auxiliary encoder position. This corresponds to swivel pulley
   * encoder value for our case.
   * @param[in] pulley_ang Swivel pulley angle in radians.
   */
  ActuatorStatus(const id_t _id, const int8_t _op_mode, const int32_t motor_pos,
                 const int32_t _motor_speed, const int16_t _motor_torque,
                 const double cable_len, const int aux_pos, const double pulley_ang)
    : WinchStatus(_id, _op_mode, motor_pos, _motor_speed, _motor_torque, cable_len,
                  aux_pos),
      pulley_angle(pulley_ang)
  {}

  uint8_t state;       /**< see Actuator::States */
  double pulley_angle; /**< [rad] */
};

/**
 * @brief A structure including camera calibrations output parameters.
 *
 * The camera parameters consist of a intrinsic parameter matrix, which incorporates
 * perspective projection and pixelization, and a vector made of the coefficients used to
 * model camera lens distortion.
 */
struct CameraParams
{
  cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F); /**< The camera matrix. */
  cv::Mat dist_coeff =
    cv::Mat::zeros(4, 1, CV_64F); /**< The distortion coefficients vector. */

  /**
   * @brief CameraParams default constructor.
   */
  CameraParams() {}
  /**
   * @brief CameraParams full constructor.
   * @param[in] cam_mat Camera matrix.
   * @param[in] dist Distortion coefficients vector.
   */
  CameraParams(const cv::Mat& cam_mat, const cv::Mat& dist)
  {
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        camera_matrix.at<double>(i, j) = cam_mat.at<double>(i, j);

    for (int i = 0; i < 4; i++)
      dist_coeff.at<double>(i, 0) = dist.at<double>(i, 0);
  }
};

/**
 * @brief A structure including all necessary camera calibration settings.
 */
struct CameraCalibSettings
{
  // Basic settings
  size_t num_frames =
    0; /*<< The number of frames to use from the input for calibration. */
  bool use_fisheye = false; /*<< use fisheye camera model for calibration. */

  // Advanced settings
  cv::Size board_size = cv::Size(9, 6); /**< Chessboard corners size. */
  float square_size   = 26.f;           /**< The size of a chessboard square in [mm] */
  double delay        = 0.5; /**< [s] minimun delay value between 2 frame processing */
  /**
   * @brief Max precision of corner position
   * Max precision of corner position in sub pixel detection;
   * when the corner position moves by less than max_precision
   */
  double max_precision = 0.00001;
  int chess_board_flags =
    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE |
    cv::CALIB_CB_FAST_CHECK; /**< Set chessboard flag to findChessboard. */
  std::string ofilepath = SRCDIR "/output_camera_calibration.json";
  int cor_sp_size       = 11;   /**< Windows dimension for sub-pixel accurate location. */
  int zero_zone         = -1;   /**< Half dimension of zero-zone. */
  /**
   * @brief Number of max iteration
   * Number of max iteration to computer corner position in sub pixel detection
   */
  int max_counter       = 50;
  bool write_points     = true; /**< Write detected feature points. */
  bool write_extrinsics = true; /**< Write extrinsic parameters. */
  bool write_grid       = true; /**< Write refined 3D target grid points. */
  bool calib_zero_tan_dist = false; /**< Set 0 tangential distortion coefficients. */

  /**
   * @brief Default constructor.
   *
   * By default the number of frames to be collected is set to zero and the distortion
   * model does not use fisheye.
   */
  CameraCalibSettings() {}
  /**
   * @brief Full constructor.
   * @param[in] number_frames set number of frame to compute camera calibration.
   * @param[in] _use_fisheye Flag to indicate whether to use fisheye model or not.
   */
  CameraCalibSettings(const size_t number_frames, const bool _use_fisheye = false)
    : num_frames(number_frames), use_fisheye(_use_fisheye)
  {}

  /**
   * @brief Returns calibration flags according to current selected camera distorsion
   * model.
   * @return Calibration flags for current selected camera distorsion model.
   */
  int calibFlags() const
  {
    if (use_fisheye)
      return cv::fisheye::CALIB_FIX_SKEW | cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC |
             cv::fisheye::CALIB_FIX_PRINCIPAL_POINT | cv::fisheye::CALIB_CHECK_COND;
    // Else
    int calib_flags = cv::CALIB_FIX_PRINCIPAL_POINT | cv::CALIB_USE_LU;
    if (calib_zero_tan_dist)
      calib_flags |= cv::CALIB_ZERO_TANGENT_DIST;
    return calib_flags;
  }

  /**
   * @brief Returns chessboard grid width in mm.
   * @return Chessboard grid width in mm.
   */
  inline float gridWidth() const { return square_size * (board_size.width - 1); }
};

#endif // CABLE_ROBOT_TYPES_H
