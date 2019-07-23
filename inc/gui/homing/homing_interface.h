/**
 * @file homing_interface.h
 * @author Simone Comari
 * @date 23 Jul 2019
 * @brief This file includes the base class for any homing interface of cable robot app.
 */

#ifndef CABLE_ROBOT_HOMING_INTERFACE_H
#define CABLE_ROBOT_HOMING_INTERFACE_H

#include <QDialog>

#include "easylogging++.h"

#include "robot/cablerobot.h"

namespace Ui {
class HomingInterface;
}

/**
 * @brief The base class for any homing interface of cable robot app.
 *
 * This base class takes care of the basic functionalities of any homing interface of
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
  virtual ~HomingInterface() override;

  /**
   * @brief External close command.
   */
  void extClose();

 signals:
  /**
   * @brief Homing failure notice.
   */
  void homingFailed() const;
  /**
   * @brief Homing success notice.
   */
  void homingSuccess() const;

 protected slots:
  void enableOkButton();

 protected:
  Ui::HomingInterface* ui;
  CableRobot* robot_ptr_ = nullptr; /**< Pointer to the cable robot instance. */
  bool ext_close_cmd_;

  virtual bool acceptedExitRoutine() { return true; }
  virtual bool rejectedExitRoutine(const bool force_exit = false) { return true; }

 private slots:
  void closeEvent(QCloseEvent* event) override final;

  void on_buttonBox_accepted();
  void on_buttonBox_rejected();
};

#endif // CABLE_ROBOT_HOMING_INTERFACE_H
