#ifndef CABLE_ROBOT_JOINTS_PVT_DIALOG_H
#define CABLE_ROBOT_JOINTS_PVT_DIALOG_H

#include <QDialog>
#include <QGridLayout>
#include <QLineSeries>
#include <QtMath>

#include "gui/apps/chartview.h"
#include "gui/apps/input_form.h"
#include "gui/apps/my3dscatterwidget.h"
#include "apps/joints_pvt_app.h"

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
  void progressUpdateTrigger(const int progress_value, const double timestamp);

 private slots:
  void handleTransitionCompleted();
  void handleTrajectoryCompleted();
  void progressUpdateCallback(const int progress_value, const double timestamp);
  void progressUpdate(const int progress_value, const double timestamp);

 private slots:
  void on_pushButton_addTraj_clicked();
  void on_pushButton_removeTraj_clicked();

  void on_pushButton_read_clicked();

  void on_checkBox_infLoop_toggled(bool checked);

  void on_pushButton_start_clicked();
  void on_pushButton_pause_clicked();
  void on_pushButton_stop_clicked();

  void on_pushButton_return_clicked();

 private:
  static const quint8 kInputFormPosInit_ = 1;
  Ui::JointsPVTDialog* ui;
  My3DScatterWidget traj_display_;
  QGridLayout* grid_layout_ = nullptr;
  QList<QSharedPointer<ChartView>> chart_views_;
  QVector<InputForm*> line_edits_;
  quint8 input_form_pos_;

  JointsPVTApp app_;
  int traj_counter_;
  int num_traj_;

  void updatePlots(const TrajectorySet& traj_set);

  void stop();
};

#endif // CABLE_ROBOT_JOINTS_PVT_DIALOG_H
