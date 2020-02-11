/**
 * @file manual_control_dialog.h
 * @author Simone Comari
 * @date 03 Jul 2019
 * @brief This file include the interface of axes manual control app.
 */

#ifndef CABLE_ROBOT_MANUAL_CONTROL_DIALOG_H
#define CABLE_ROBOT_MANUAL_CONTROL_DIALOG_H

#include <QDialog>
#include <QTimer>

#include "apps/manual_control_app.h"
#include "gui/apps/scatter3d_widget.h"

namespace Ui {
class ManualControlDialog;
}

/**
 * @brief The implementation of the interface of the axes manual control app.
 *
 * This interface allows the user to move the robot platform along the three cartesian
 * axes X,Y,Z. This motion requires the knowledge of the kinematics of the robot as well
 * as the current homing position.
 * It also display a schematic 3D view of the current platform position wrt global frame.
 * @todo display part
 */
class ManualControlDialog: public QDialog
{
  Q_OBJECT

 public:
  /**
   * @brief Full constructor.
   * @param parent The parent Qt object, in our case the main GUI.
   * @param robot Pointer to the cable robot instance.
   */
  explicit ManualControlDialog(QWidget* parent, CableRobot* robot);
  ~ManualControlDialog();

 private slots:
  void appendText2Browser(const QString& text);
  void updateActualXYZ();

 private slots:
  void on_pushButton_reset_clicked();

  void on_pushButton_return_clicked();

 private:
  static constexpr uint kTimerPeriodMsec_ = 100;

  Ui::ManualControlDialog* ui;
  Scatter3DWidget traj_display_;

  CableRobot* robot_ptr_ = nullptr;
  ManualControlApp app_;
  QTimer actual_pos_timer_;

  void resetTargetXYZ();
};

#endif // CABLE_ROBOT_MANUAL_CONTROL_DIALOG_H
