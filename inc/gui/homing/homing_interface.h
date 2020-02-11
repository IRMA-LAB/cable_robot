/**
 * @file homing_interface.h
 * @author Simone Comari
 * @date 02 Jul 2019
 * @brief This file includes the abstract class for any homing interface of cable robot
 * app.
 */

#ifndef CABLE_ROBOT_HOMING_INTERFACE_H
#define CABLE_ROBOT_HOMING_INTERFACE_H

#include <QDialog>

#include "easylogging++.h"

#include "robot/cablerobot.h"

/**
 * @brief The abstract base class for any homing interface of cable robot app.
 *
 * This abstract class takes care of the basic functionalities of any homing interface of
 * cable robot app: signaling, necessary attributes and closing behaviour.
 */
class HomingInterface: public QDialog
{
  Q_OBJECT

 public:
  /**
   * @brief HomingInterface constructor.
   * @param parent The parent QWidget, in this case the HomingDialog.
   * @param robot Pointer to the cable robot instance, to be possibly used withing the
   * derived class.
   */
  HomingInterface(QWidget* parent, CableRobot* robot);
  virtual ~HomingInterface() = 0;

  /**
   * @brief Close command.
   */
  virtual void Close() { close(); }

 signals:
  /**
   * @brief Homing failure notice.
   */
  void homingFailed() const;
  /**
   * @brief Homing success notice.
   */
  void homingSuccess() const;

 protected:
  CableRobot* robot_ptr_ = nullptr; /**< Pointer to the cable robot instance. */
};

#endif // CABLE_ROBOT_HOMING_INTERFACE_H
