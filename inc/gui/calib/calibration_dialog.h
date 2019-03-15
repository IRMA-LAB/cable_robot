/**
 * @file calibration_dialog.h
 * @author Simone Comari
 * @date 11 Mar 2019
 * @brief This file include the calibration dialog class.
 */

#ifndef CALIBRATION_DIALOG_H
#define CALIBRATION_DIALOG_H

#include <QDialog>

#include "easylogging++.h"
#include "libcdpr/inc/types.h"

#include "robot/cablerobot.h"

namespace Ui {
class CalibrationDialog;
}

/**
 * @brief A dialog to select calibration procedure type and bridge communication between
 * selected procedure and main GUI.
 */
class CalibrationDialog: public QDialog
{
  Q_OBJECT

 public:
  /**
   * @brief CalibrationDialog constructor.
   * @param parent The parent Qt object, in our case the main GUI.
   * @param robot Pointer to the cable robot instance, to be passed to the selected
   * calibration procedure interface.
   */
  CalibrationDialog(QWidget* parent, CableRobot* robot);
  ~CalibrationDialog();

 signals:
  /**
   * @brief Enable main GUI command.
   */
  void enableMainGUI();
  /**
   * @brief Calibration end notice.
   */
  void calibrationEnd();

 private slots:
  void on_buttonBox_accepted();

  void on_buttonBox_rejected();

 private:
  Ui::CalibrationDialog* ui;

  CableRobot* robot_;
};

#endif // CALIBRATION_DIALOG_H
