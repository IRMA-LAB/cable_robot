/**
 * @file calib_interface_excitation.h
 * @author Simone Comari
 * @date 02 Jul 2019
 * @brief This file include the interface of CalibExcitation app.
 */

#ifndef CABLE_ROBOT_CALIB_INTERFACE_EXCITATION_H
#define CABLE_ROBOT_CALIB_INTERFACE_EXCITATION_H

#include <QDialog>

#include "calib/calib_excitation.h"

namespace Ui {
class CalibInterfaceExcitation;
}

/**
 * @brief The implementation of the interface of the calibration support app in case of
 * external tracking system.
 *
 * This interface allows the user to:
 * - Enable/disable all motors at once;
 * - Switch between position and torque control, being the former used to fix the cables
 * length and the latter to manually move the platform (freedrive mode);
 * - Once in position control, start the logging phase, while a trajectory is excecuted to
 * excite most platform dynamics.
 */
class CalibInterfaceExcitation: public QDialog
{
  Q_OBJECT

 public:
  /**
   * @brief Full constructor.
   * @param parent The parent Qt object, in our case the main GUI.
   * @param robot Pointer to the cable robot instance.
   * @param[in] params params A vector of actuator parameters, with as many elements as
   * the active motors.
   */
  explicit CalibInterfaceExcitation(QWidget* parent, CableRobot* robot,
                                    const vect<grabcdpr::ActuatorParams>& params);
  ~CalibInterfaceExcitation();

 private slots:
  void appendText2Browser(const QString& text);
  void handleStateChanged(const quint8& state);

 private slots:
  void on_pushButton_enable_clicked();
  void on_radioButton_torque_clicked();
  void on_radioButton_position_clicked();
  void on_pushButton_return_clicked();

  void on_pushButton_logging_clicked();

 private:
  Ui::CalibInterfaceExcitation* ui;

  CableRobot* robot_ptr_ = nullptr;
  CalibExcitation app_;
};

#endif // CABLE_ROBOT_CALIB_INTERFACE_EXCITATION_H
