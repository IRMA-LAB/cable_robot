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
#include "gui/apps/input_form.h"
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
  void on_pushButton_addTraj_clicked();
  void on_pushButton_removeTraj_clicked();

  void on_pushButton_read_clicked();

  void on_checkBox_toggled(bool checked);

  void on_pushButton_start_clicked();
  void on_pushButton_pause_clicked();
  void on_pushButton_stop_clicked();

  void on_pushButton_return_clicked();


 private:
  static const quint8 kInputFormPos0_ = 2;
  Ui::JointsPVTDialog* ui;
  My3DScatterWidget traj_display_;
  QGridLayout* grid_layout_ = NULL;
  QVector<InputForm*> line_edits_;
  quint8 input_form_pos_;

  CableRobot* robot_ptr_;
  ControllerJointsPVT controller_;

  struct TrajectorySet
  {
    ControlMode traj_type;
    vect<TrajectoryD> traj_platform;
    vect<TrajectoryD> traj_cables_len;
    vect<TrajectoryI> traj_motors_pos;
    vect<TrajectoryI> traj_motors_vel;
    vect<TrajectoryS> traj_motors_torque;
  };

  QVector<TrajectorySet> traj_sets_;

  bool readTrajectories(const QString& ifilepath);

  void setCablesLenTraj(const bool relative, const vect<id_t>& motors_id, QTextStream& s,
                        TrajectorySet& traj_set);
  void setMotorPosTraj(const bool relative, const vect<id_t>& motors_id, QTextStream& s,
                       TrajectorySet& traj_set);
  void setMotorVelTraj(const vect<id_t>& motors_id, QTextStream& s,
                       TrajectorySet& traj_set);
  void setMotorTorqueTraj(const bool relative, const vect<id_t>& motors_id,
                          QTextStream& s, TrajectorySet& traj_set);

  void updatePlots(const TrajectorySet& traj_set);

  void runTransition(const TrajectorySet& traj_set);

  void sendTrajectories(const TrajectorySet& traj_set);
};

#endif // CABLE_ROBOT_JOINTS_PVT_DIALOG_H
