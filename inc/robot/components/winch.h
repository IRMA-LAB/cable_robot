/**
 * @file winch.h
 * @author Simone Comari, Edoardo Idà
 * @date 02 Mar 2020
 * @brief File containing the virtualization of a winch system of the cable robot.
 */

#ifndef CABLE_ROBOT_WINCH_H
#define CABLE_ROBOT_WINCH_H

#include "libcdpr/inc/cdpr_types.h"
#include "libgrabec/inc/slaves/goldsolowhistledrive.h"

#include "utils/types.h"

/**
 * @brief The cable class, component of Winch object.
 *
 * This class yields all information specific to a single cable, in terms of properties
 * and configuration.
 */
class Cable
{
 public:
  /**
   * @brief Set cable length at home position.
   * @param[in] length _[m]_ Cable length at home position.
   * @see GetHomeLength()
   */
  void setHomeLength(const double length) { home_length_ = length; }

  /**
   * @brief Get cable length at home position.
   * @return Cable length at home position in meters.
   * @see SetHomeLength()
   */
  double getHomeLength() const { return home_length_; }
  /**
   * @brief Get most recente cable length.
   * @return Most recente cable length in meters.
   * @see GetUpdatedLength()
   */
  double getLength() const { return length_; }
  /**
   * @brief Update and get newly computed cable length.
   * @param[in] delta_length Cable increment in meters wrt home reference value.
   * @return Updated cable length in meters.
   * @see GetLength() UpdateCableLen()
   */
  double getUpdatedLength(const double delta_length);

  /**
   * @brief Update cable length by adding given value to home reference value.
   * @param[in] delta_length Cable increment in meters wrt home reference value.
   */
  inline void updateCableLen(const double delta_length);

 private:
  double home_length_ = 0.0;
  double length_      = 0.0;
};

/**
 * @brief The winch class, component of Actuator object.
 *
 * Each winch is composed of two separate components:
 * - the cable, which is a completely passive element;
 * - the servo motor, or drive, which is an _Elmo_ GoldSoloWhistle drive, which instead is
 * an active element, and the motion source of the whole robot.
 *
 * This class mainly serves as bridge between the parent actuator and the interaction with
 * these two sub-components, especially when information from both parts are needed to do
 * some evaluations.
 */
class Winch
{
 public:
  /**
   * @brief Winch constructor.
   * @param[in] id Motor ID.
   * @param[in] slave_position Position of this ethercat slave in the network, more
   * precisely of the GoldSoloWhistle drive, which is one of its components.
   * @param[in] params Configuration parameters describing assembly details and technical
   * information about its components.
   */
  Winch(const id_t id, const uint8_t slave_position, const grabcdpr::WinchParams& params);

  /**
   * @brief Get a constant pointer to the servo motor object.
   * @return A constant pointer to the servo motor object.
   */
  const grabec::GoldSoloWhistleDrive* getServo() const { return &servo_; }
  /**
   * @brief Get a pointer to the servo motor object.
   * @return A pointer to the servo motor object.
   */
  grabec::GoldSoloWhistleDrive* getServo() { return &servo_; }
  /**
   * @brief Get a constant pointer to the cable object.
   * @return A constant pointer to the cable object.
   */
  const Cable* getCable() const { return &cable_; }
  /**
   * @brief Get winch most recent status.
   * @return Winch most recent status.
   */
  WinchStatus getStatus();
  /**
   * @brief Get servo motor home position.
   * @return servo motor home position in encoder counts.
   */
  int32_t getServoHomePos() const { return servo_home_pos_; }

  /**
   * @brief Set motor position target and change operational mode to CYCLIC_POSITION.
   * @param[in] target_pos Servo motor position target in encoder counts.
   */
  void setServoPos(const int32_t target_pos) { servo_.ChangePosition(target_pos); }
  /**
   * @brief Set motor position target by desired cable length and change operational mode
   * to CYCLIC_POSITION.
   * @param[in] target_length Desired cable length in meters.
   */
  void setServoPosByCableLen(const double target_length);
  /**
   * @brief Set motor velocity target and change operational mode to CYCLIC_VELOCITY.
   * @param[in] target_speed Motor velocity target in counts/seconds.
   */
  void setServoSpeed(const int32_t target_speed);
  /**
   * @brief Set motor torque target and change operational mode to CYCLIC_TORQUE.
   * @param[in] target_torque Motor torque target in per thousand nominal points.
   * @note A negative value corresponds to a _pulling_ torque.
   */
  void setServoTorque(const int16_t target_torque);
  /**
   * @brief Set motor operational mode.
   * @param[in] op_mode Desired motor operational mode.
   */
  void setServoOpMode(const int8_t op_mode);

  /**
   * @brief Update winch configuration at home position.
   *
   * Home configuration is nothing but the definition of actuator configuration in terms
   * of cable length and swivel pulley angle for a specific set of encoder values of motor
   * and pulley encoders respectively.
   * For the winch, this unique correspondence defines a fixed reference for the movements
   * of the cable with respect to the drive motion.
   * @param[in] cable_len _[m]_ Cable length corresponding to the recorded home position
   * in motor counts.
   */
  void updateHomeConfig(const double cable_len);
  /**
   * @brief Update winch configuration using internal most recent measurements.
   *
   * Actuator configuration consists in particular of cable length.
   */
  void updateConfig() { updateConfig(servo_.GetPosition()); }
  /**
   * @brief Update winch configuration using given servo motor position.
   *
   * Actuator configuration consists in particular of cable length.
   * @param[in] servo_pos Servo motor position in encoder counts.
   */
  void updateConfig(const int32_t servo_pos);

  /**
   * @brief Convert motor counts to cable length.
   * @param counts Motor (encoder) counts.
   * @return Corresponding cable length in meters.
   */
  double countsToLength(const int counts) const;
  /**
   * @brief Convert cable length to motor counts.
   * @param length Cable length in meters.
   * @return Corresponding motor (encoder) counts.
   */
  int lengthToCounts(const double length) const;

 private:
  grabcdpr::WinchParams params_;
  Cable cable_;
  grabec::GoldSoloWhistleDrive servo_;
  int32_t servo_home_pos_ = 0;
  id_t id_;
};

#endif // CABLE_ROBOT_WINCH_H
