/**
 * @file homing_dialog.h
 * @author Simone Comari
 * @date 10 Jan 2020
 * @brief This file include the homing dialog class.
 */

#ifndef CABLE_ROBOT_HOMING_DIALOG_H
#define CABLE_ROBOT_HOMING_DIALOG_H

#include <QDialog>

#include "easylogging++.h"
#include "libcdpr/inc/cdpr_types.h"

#include "gui/homing/homing_interface_proprioceptive.h"


namespace Ui {
class HomingDialog;
}

/**
 * @brief A dialog to select homing procedure type and bridge communication between
 * selected procedure and main GUI.
 */
class HomingDialog: public QDialog
{
  Q_OBJECT

 public:
  /**
   * @brief HomingDialog constructor.
   * @param parent The parent Qt object, in our case the main GUI.
   * @param robot Pointer to the cable robot instance, to be passed to the selected homing
   * procedure interface.
   */
  HomingDialog(QWidget* parent, CableRobot* robot);
  ~HomingDialog();

 signals:
  /**
   * @brief Enable(_true_)/disable(_false_) main GUI command.
   */
  void enableMainGUI(bool);
  /**
   * @brief Homing failure notice.
   */
  void homingFailed();
  /**
   * @brief Homing success notice.
   */
  void homingSuccess();

 private slots:
  void on_buttonBox_accepted();
  void on_buttonBox_rejected();

 private slots:
  void fwdHomingFailed();
  void fwdHomingSuccess();

 private:
  Ui::HomingDialog* ui;

  enum HomingMethod
  {
    PROPRIOCEPTIVE,
    VISION,
    FUSION,
    NONE
  };

  HomingInterface* interface_ = nullptr;
  CableRobot* robot_ptr_      = nullptr;
  int homing_method_;

  void DeleteInterface();
};

#endif // CABLE_ROBOT_HOMING_DIALOG_H
