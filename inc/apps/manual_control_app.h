/**
 * @file manual_control_app.h
 * @author Simone Comari
 * @date 02 Mar 2020
 * @brief This file includes the implementation of the axes manual control application.
 */

#ifndef CABLE_ROBOT_MANUAL_CONTROL_APP_H
#define CABLE_ROBOT_MANUAL_CONTROL_APP_H

#include <QObject>
#include <QTextStream>

#include "matrix.h"

#include "ctrl/controller_axes.h"
#include "robot/cablerobot.h"

/**
 * @brief This class takes care of controlling the robot when axes manual control app is
 * selected.
 *
 * This application uses cable robot kinematics to move the platform along the three
 * cartesian axes. The new targets comes directly from the user via the corresponding
 * interface.
 * @todo Implement controller for this app and complete.
 */
class ManualControlApp: public QObject
{
  Q_OBJECT

 public:
  /**
   * @brief Full constructor.
   * @param parent The parent QObject, in this case the corresponding interface.
   * @param robot A pointer to the cable robot object.
   */
  explicit ManualControlApp(QObject* parent, CableRobot* robot = nullptr);
  ~ManualControlApp();

  /**
   * @brief Get actual platform 3D global position.
   * @return The actual platform 3D global position in meters.
   */
  const grabnum::Vector3d& getActualPos();

  /**
   * @brief Set platform new global position target.
   * @param coord The coordinate to be updated.
   * @param value The new value of the specified coordinate to be updated.
   */
  void setTarget(const Axis coord, const double value);
  /**
   * @brief Set target global position of the platform to its current value.
   */
  void resetTarget();

 signals:
  /**
   * @brief Signal including a message to any QConsole, for instance a QTextBrowser.
   */
  void printToQConsole(const QString&) const;

 private:
  CableRobot* robot_ptr_ = nullptr;
  ControllerAxes controller_;
  grabnum::Vector3d target_pos_;
  grabnum::Vector3d actual_pos_;
};

#endif // CABLE_ROBOT_MANUAL_CONTROL_APP_H
