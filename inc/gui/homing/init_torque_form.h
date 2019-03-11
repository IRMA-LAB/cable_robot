/**
 * @file init_torque_form.h
 * @author Simone Comari
 * @date 11 Mar 2019
 * @brief This file includes the class interface to the widget used by the user to define
 * the torque targets to be applied during the homing procedure for a single motor.
 */

#ifndef CABLE_ROBOT_HOMING_INIT_TORQUE_FORM_H
#define CABLE_ROBOT_HOMING_INIT_TORQUE_FORM_H

#include <QWidget>

namespace Ui {
class InitTorqueForm;
}

/**
 * @brief The interface to access and modify the elements of a single form for target
 * torques of a single motor for the homing procedure.
 *
 * An instance of this object is created together with its relative form on the
 * proprioceptive homing interface for each active motor present in the robot
 * configuration.
 * It allows the user to set the initial and final torque value for each actuator during
 * the proprioceptive homing procedure.
 */
class InitTorqueForm: public QWidget
{
  Q_OBJECT

 public:
  /**
   * @brief InitTorqueForm constructor.
   * @param[in] motor_id The ID of the motor it represents.
   * @param[in] parent The parent QWidget, in this case the HomingInterfaceProprioceptive.
   */
  InitTorqueForm(const id_t motor_id, QWidget* parent = NULL);
  ~InitTorqueForm();

  /**
   * @brief Set initial torque value in per thousand nominal points.
   * @param[in] value Initial torque value in per thousand nominal points.
   */
  void SetInitTorque(const int value);
  /**
   * @brief Set maximum torque value in per thousand nominal points.
   *
   * The maximum torque is the value the actuator reaches at the end of the coiling phase
   * before moving to the uncoiling phase.
   * @param[in] value Maximum torque value in per thousand nominal points.
   */
  void SetMaxTorque(const int value);

  /**
   * @brief Get initial torque value in per thousand nominal points.
   * @return Initial torque value in per thousand nominal points.
   */
  qint16 GetInitTorque() const;
  /**
   * @brief Get maximum torque value in per thousand nominal points.
   *
   * The maximum torque is the value the actuator reaches at the end of the coiling phase
   * before moving to the uncoiling phase.
   * @return Maximum torque value in per thousand nominal points.
   */
  qint16 GetMaxTorque() const;
  /**
   * @brief Get maximum/final torque minimum value in per thousand nominal points.
   * @return The maximum/final torque minimum value in per thousand nominal points.
   * @note The pulling torque in the GoldSoloWhistle drive has negative value, which means
   * the smaller the value, the strongest the torque. Therefore the minimum value the
   * final torque can have is in fact the closest number to zero or, in other words, the
   * biggest number.
   */
  qint16 GetMaxTorqueMinumum() const;

  /**
   * @brief Enable/disable initial torque spinbox.
   * @param[in] value _True_ to enable, _false_to disable.
   */
  void EnableInitTorque(const bool value);
  /**
   * @brief Enable/disable maximum torque spinbox.
   * @param[in] value _True_ to enable, _false_to disable.
   */
  void EnableMaxTorque(const bool value);

 private:
  static constexpr int kTorqueMeasTol_ = 10;

  Ui::InitTorqueForm* ui;
};

#endif // CABLE_ROBOT_HOMING_INIT_TORQUE_FORM_H
