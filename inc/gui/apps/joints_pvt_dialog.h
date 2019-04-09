#ifndef CABLE_ROBOT_JOINTS_PVT_DIALOG_H
#define CABLE_ROBOT_JOINTS_PVT_DIALOG_H

#include <QDialog>
#include <QFileDialog>
#include <QGridLayout>
#include <QLineSeries>
#include <QMessageBox>
#include <QtMath>

#include "easylogging++.h"

#include "ctrl/controller_joints_pvt.h"
#include "gui/apps/chartview.h"
#include "gui/apps/my3dscatterwidget.h"
#include "robot/cablerobot.h"

namespace Ui {
class JointsPVTDialog;
}

class JointsPVTDialog: public QDialog
{
  Q_OBJECT

 public:
  explicit JointsPVTDialog(QWidget* parent, CableRobot* robot,
                           const vect<grabcdpr::ActuatorParams>& params);
  ~JointsPVTDialog();

 signals:

 private slots:
  void handleTrajectoryCompleted();
  void progressUpdate(const int progress_value);

 private slots:
  void on_pushButton_fileSelection_clicked();
  void on_pushButton_read_clicked();

  void on_checkBox_toggled(bool checked);

  void on_pushButton_start_clicked();
  void on_pushButton_pause_clicked();
  void on_pushButton_stop_clicked();

  void on_pushButton_return_clicked();

 private:
  Ui::JointsPVTDialog* ui;
  My3DScatterWidget traj_display_;
  QGridLayout* grid_layout_ = NULL;

  CableRobot* robot_ptr_;
  ControllerJointsPVT controller_;

  ControlMode traj_type_;
  vect<TrajectoryD> traj_platform_;
  vect<TrajectoryD> traj_cables_len_;
  vect<TrajectoryI> traj_motors_pos_;
  vect<TrajectoryI> traj_motors_vel_;
  vect<TrajectoryS> traj_motors_torque_;

  bool readTrajectories(const QString& ifilepath);

  void updatePlots();
};

#endif // CABLE_ROBOT_JOINTS_PVT_DIALOG_H
