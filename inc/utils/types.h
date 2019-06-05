/**
 * @file utils/types.h
 * @author Simone Comari
 * @date 09 Apr 2019
 * @brief File containing the implementation of a custom wrapper to log cable robot data
 * employing easylogging++ package.
 */

#ifndef CABLE_ROBOT_TYPES_H
#define CABLE_ROBOT_TYPES_H

#include <cmath>
#include <stdint.h>
#include <stdlib.h>
#include <vector>

#include "libgrabec/inc/slaves/goldsolowhistledrive.h"


/**
 * @brief StringBuf typedef.
 */
typedef std::basic_stringbuf<char16_t> StringBuf;

template <typename T> using vect = std::vector<T>; /**< Alias for vector type. */
using vectD                      = vect<double>; /**< Alias for vector of double type. */
using vectI                      = vect<int>;    /**< Alias for vector of int type. */
using vectS                      = vect<short>;  /**< Alias for vector of short type. */

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


template <typename T>
/**
 * @brief The WayPoint struct
 */
struct WayPoint
{
  double ts = -1.0; /**< .. */
  T value;          /**< .. */

  /**
   * @brief WayPoint
   */
  WayPoint() {}
  /**
   * @brief WayPoint
   * @param time
   * @param _value
   */
  WayPoint(const double time, const T& _value) : ts(time), value(_value) {}
};

using WayPointD = WayPoint<double>; /**< .. */
using WayPointI = WayPoint<int>;    /**< .. */
using WayPointS = WayPoint<short>;  /**< .. */

template <typename T>
/**
 * @brief The Trajectory struct
 */
struct Trajectory
{
  id_t id = 0;      /**< .. */
  vectD timestamps; /**< .. */
  vect<T> values;   /**< .. */

  /**
   * @brief Trajectory
   */
  Trajectory() {}
  /**
   * @brief Trajectory
   * @param _id
   */
  Trajectory(const id_t _id) : id(_id) {}
  /**
   * @brief Trajectory
   * @param _id
   * @param _values
   */
  Trajectory(const id_t _id, const vect<T>& _values) : id(_id), values(_values)
  {
    timestamps = vectD(values.size(), -1.0);
  }
  /**
   * @brief Trajectory
   * @param _id
   * @param _values
   * @param times
   */
  Trajectory(const id_t _id, const vect<T>& _values, const vectD& times)
    : id(_id), values(_values), timestamps(times)
  {
    assert(times.size() == _values.size());
  }

  /**
   * @brief waypointFromIndex
   * @param index
   * @return
   */
  WayPoint<T> waypointFromIndex(const size_t index) const
  {
    assert(index < timestamps.size() && index < values.size());
    return WayPoint<T>(timestamps[index], values[index]);
  }

  /**
   * @brief waypointFromAbsTime
   * @param time
   * @return
   */
  WayPoint<T> waypointFromAbsTime(const double time) const
  {
    assert(timestamps.front() >= 0.0);

    if (time <= timestamps.front())
      return WayPoint<T>(timestamps.front(), values.front());
    if (time >= timestamps.back())
      return WayPoint<T>(timestamps.back(), values.back());
    // Find nearest neighbor
    vectD::const_iterator low =
      std::lower_bound(timestamps.begin(), timestamps.end(), time);
    ulong lower_idx;
    if (low == timestamps.end())
      // no bigger value than val in vector --> this shouldn't happen
      return WayPoint<T>(timestamps.back(), values.back());
    else if (low == timestamps.begin())
      lower_idx = 0;
    else
      lower_idx = low - timestamps.begin() - 1;
    const ulong upper_idx = lower_idx + 1;
    // Interpolate
    const double slope = (values[upper_idx] - values[lower_idx]) /
                         (timestamps[upper_idx] - timestamps[lower_idx]);
    const double offset = values[upper_idx] - slope * timestamps[upper_idx];
    const double value  = slope * time + offset;

    return WayPoint<T>(time, static_cast<T>(value));
  }

  /**
   * @brief waypointFromRelTime
   * @param time
   * @return
   */
  WayPoint<T> waypointFromRelTime(const double time) const
  {
    assert(timestamps.front() >= 0.0);

    return waypointFromAbsTime(time + timestamps.front());
  }
};

using TrajectoryD = Trajectory<double>; /**< .. */
using TrajectoryI = Trajectory<int>;    /**< .. */
using TrajectoryS = Trajectory<short>;  /**< .. */

#endif // CABLE_ROBOT_TYPES_H
