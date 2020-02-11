/**
 * @file joints_pvt_dialog.h
 * @author Simone Comari
 * @date 09 Jul 2019
 * @brief This file include the interface of joints pvt control app.
 */

#ifndef CABLE_ROBOT_JOINTS_PVT_DIALOG_H
#define CABLE_ROBOT_JOINTS_PVT_DIALOG_H

#include <QDialog>
#include <QGridLayout>
#include <QLineSeries>
#include <QtMath>

#include "apps/joints_pvt_app.h"
#include "gui/misc/chartview.h"
#include "gui/misc/file_selection_form.h"
#include "gui/misc/scatter3d_widget.h"

namespace Ui {
class JointsPVTDialog;
}

/**
 * @brief The implementation of the interface of the joints pvt control app.
 *
 * This interface allows the user to:
 * - Add an arbitrary number of trajectories to be excecuted one after the other;
 * - Read them and display the current one on a side plot (one for each active motor
 * involved);
 * - Start/pause/stop the set of trajectories;
 * - Run all loaded trajectories in loop forever;
 *
 * Besides the trajectory plots, a 3D plot shows the global position of the platform as a
 * result of the trajectory followed (TODO).
 * A progress bar, moreover, displays the current progress over the current trajectory
 * excecuted.
 *
 * Before the execution of each trajectory, the robot will slowly transition towards the
 * first point in the trajectory and wait undefinitely for the platform to be steady.
 * The operation can be paused/stopped at any time once started.
 *
 * When pausing and resuming the robot will smoothly stop/start yet following the exact
 * same trajectory, from a setpoint point of view.
 *
 * @todo implementation of 3D trajectory display
 */
class JointsPVTDialog: public QDialog
{
  Q_OBJECT

 public:
  /**
   * @brief Full constructor.
   * @param parent The parent QObject, in this case the corresponding interface.
   * @param robot A pointer to the cable robot object.
   * @param[in] params A vector of actuator parameters, with as many elements as the
   * active motors.
   */
  explicit JointsPVTDialog(QWidget* parent, CableRobot* robot,
                           const vect<grabcdpr::ActuatorParams>& params);
  ~JointsPVTDialog();

 signals:
  /**
   * @brief Signal including the trajectory/transition progress and latest point-value
   * timestamp.
   * @param progress_value The latest progress of the trajectory currently executed.
   * @param timestamp The timestamp of the latest point-value attained.
   */
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
  Scatter3DWidget traj_display_;
  QGridLayout* grid_layout_ = nullptr;
  QList<QSharedPointer<ChartView>> chart_views_;
  QVector<FileSelectionForm*> line_edits_;
  quint8 input_form_pos_;
  int screen_width_;

  JointsPVTApp app_;
  int traj_counter_;
  int num_traj_;

  void updatePlots(const TrajectorySet& traj_set);

  void stop();
};

#endif // CABLE_ROBOT_JOINTS_PVT_DIALOG_H
