#include "gui/apps/joints_pvt_dialog.h"
#include "ui_joints_pvt_dialog.h"

JointsPVTDialog::JointsPVTDialog(QWidget* parent, CableRobot* robot,
                                 const vect<grabcdpr::ActuatorParams>& params)
  : QDialog(parent), ui(new Ui::JointsPVTDialog), traj_display_(this),
    input_form_pos_(kInputFormPosInit_), app_(this, robot, params), traj_counter_(0),
    num_traj_(0)
{
  ui->setupUi(this);
  ui->horizontalLayout_display->addWidget(&traj_display_, 1);
  setAttribute(Qt::WA_DeleteOnClose);

  line_edits_.append(new InputForm(this));
  ui->verticalLayout_inputSource->insertWidget(kInputFormPosInit_ - 1,
                                               line_edits_.last());

  connect(&app_, SIGNAL(transitionComplete()), this, SLOT(handleTransitionCompleted()));
  connect(&app_, SIGNAL(trajectoryComplete()), this, SLOT(handleTrajectoryCompleted()));
  connect(&app_, SIGNAL(trajectoryProgress(int, double)), this,
          SLOT(progressUpdateCallback(int, double)));
}

JointsPVTDialog::~JointsPVTDialog()
{
  app_.stop(); // in case of external close call

  disconnect(&app_, SIGNAL(transitionComplete()), this,
             SLOT(handleTransitionCompleted()));
  disconnect(&app_, SIGNAL(trajectoryComplete()), this,
             SLOT(handleTrajectoryCompleted()));
  disconnect(&app_, SIGNAL(trajectoryProgress(int, double)), this,
             SLOT(progressUpdateCallback(int, double)));

  while (!line_edits_.empty())
  {
    ui->verticalLayout_inputSource->removeWidget(line_edits_.last());
    delete line_edits_.last();
    line_edits_.pop_back();
  }
  delete ui;
  CLOG(INFO, "event") << "Joints PVT dialog closed";
}

//--------- Private slots -----------------------------------------------------------//

void JointsPVTDialog::handleTransitionCompleted()
{
  CLOG(TRACE, "event");
  disconnect(this, SIGNAL(progressUpdateTrigger(int, double)), this,
             SLOT(progressUpdate(int, double)));
  ui->progressBar->setFormat(
    QString("Trajectory %1 in progress... %p%").arg(traj_counter_));
  ui->progressBar->setValue(0);
  app_.sendTrajectories(traj_counter_);
  connect(this, SIGNAL(progressUpdateTrigger(int, double)), this,
          SLOT(progressUpdate(int, double)), Qt::QueuedConnection);
}

void JointsPVTDialog::handleTrajectoryCompleted()
{
  CLOG(TRACE, "event");
  disconnect(this, SIGNAL(progressUpdateTrigger(int, double)), this,
             SLOT(progressUpdate(int, double)));
  if (++traj_counter_ >= num_traj_)
  {
    if (!ui->checkBox_infLoop->isChecked())
    {
      // All trajectories completed.
      app_.stop(); // transition to READY
      stop();      // update GUI
      return;
    }
    traj_counter_ = 0; // infinite loop --> start over
  }

  // Run next trajectory.
  updatePlots(app_.getTrajectorySet(traj_counter_));
  ui->progressBar->setFormat(
    QString("Transition %1 in progress... %p%").arg(traj_counter_));
  ui->progressBar->setValue(0);
  for (const auto& chart_view : chart_views_)
    chart_view->removeHighlight();
  app_.runTransition(traj_counter_);
  connect(this, SIGNAL(progressUpdateTrigger(int, double)), this,
          SLOT(progressUpdate(int, double)), Qt::QueuedConnection);
}

void JointsPVTDialog::progressUpdateCallback(const int progress_value,
                                             const double timestamp)
{
  emit progressUpdateTrigger(progress_value, timestamp);
}

void JointsPVTDialog::progressUpdate(const int progress_value, const double timestamp)
{
  ui->progressBar->setValue(progress_value);
  if (app_.GetCurrentState() == JointsPVTApp::ST_TRANSITION || progress_value >= 100)
    return;

  const TrajectorySet traj_set = app_.getTrajectorySet(traj_counter_);
  switch (traj_set.traj_type)
  {
    case CABLE_LENGTH:
      for (uint i = 0; i < traj_set.traj_cables_len.size(); i++)
      {
        WayPointD waypoint =
          traj_set.traj_cables_len[i].waypointFromAbsTime(timestamp, 0.01);
        chart_views_[i]->highlightCurrentPoint(QPointF(waypoint.ts, waypoint.value));
      }
      break;
    case MOTOR_POSITION:
      for (uint i = 0; i < traj_set.traj_cables_len.size(); i++)
      {
        WayPointI waypoint =
          traj_set.traj_motors_pos[i].waypointFromAbsTime(timestamp, 0.01);
        chart_views_[i]->highlightCurrentPoint(QPointF(waypoint.ts, waypoint.value));
      }
      break;
    case MOTOR_SPEED:
      for (uint i = 0; i < traj_set.traj_cables_len.size(); i++)
      {
        WayPointI waypoint =
          traj_set.traj_motors_vel[i].waypointFromAbsTime(timestamp, 0.01);
        chart_views_[i]->highlightCurrentPoint(QPointF(waypoint.ts, waypoint.value));
      }
      break;
    case MOTOR_TORQUE:
      for (uint i = 0; i < traj_set.traj_cables_len.size(); i++)
      {
        WayPointS waypoint =
          traj_set.traj_motors_torque[i].waypointFromAbsTime(timestamp, 0.01);
        chart_views_[i]->highlightCurrentPoint(QPointF(waypoint.ts, waypoint.value));
      }
      break;
    case NONE:
      break;
  }
}

//--------- Private GUI slots -------------------------------------------------------//

void JointsPVTDialog::on_pushButton_addTraj_clicked()
{
  CLOG(TRACE, "event");
  line_edits_.append(new InputForm(this));
  ui->verticalLayout_inputSource->insertWidget(input_form_pos_++, line_edits_.last());
  ui->pushButton_removeTraj->setEnabled(true);
}

void JointsPVTDialog::on_pushButton_removeTraj_clicked()
{
  CLOG(TRACE, "event");
  ui->verticalLayout_inputSource->removeWidget(line_edits_.last());
  delete line_edits_.last();
  line_edits_.pop_back();
  if (--input_form_pos_ <= kInputFormPosInit_)
    ui->pushButton_removeTraj->setDisabled(true);
}

void JointsPVTDialog::on_pushButton_read_clicked()
{
  CLOG(TRACE, "event");
  // Collect all given filepaths.
  QVector<QString> input_filenames;
  for (InputForm* form : line_edits_)
    input_filenames.append(form->getFilepath());
  // Check if at least one file was provided.
  if (input_filenames.isEmpty())
  {
    CLOG(WARNING, "event") << "Trajectory files empty";
    QMessageBox::warning(
      this, "File Error",
      "Trajectory files are missing!\nPlease select at least one file containing "
      "joints trajectory first.");
    return;
  }

  // Clear all trajectories.
  app_.clearAllTrajectories();
  num_traj_ = 0; // reset

  // Read trajectories from each file.
  for (const QString& input_filename : input_filenames)
  {
    if (!app_.readTrajectories(input_filename))
    {
      CLOG(WARNING, "event") << "Trajectory file is not valid";
      QMessageBox::warning(this, "File Error",
                           "Trajectory file '" + input_filename + "' is not valid");
      return;
    }
    num_traj_++;
  }

  updatePlots(app_.getTrajectorySet(0)); // display first trajectory in queue
  ui->pushButton_start->setEnabled(true);
}

void JointsPVTDialog::on_checkBox_infLoop_toggled(bool checked)
{
  CLOG(TRACE, "event") << checked;
}

void JointsPVTDialog::on_pushButton_start_clicked()
{
  CLOG(TRACE, "event");

  ui->pushButton_start->setDisabled(true);
#if DEBUG_GUI
  ui->pushButton_pause->setEnabled(true);
#endif
  ui->pushButton_stop->setEnabled(true);
  ui->pushButton_return->setDisabled(true);
  ui->groupBox_inputs->setDisabled(true);

  traj_counter_ = 0; // reset
  ui->progressBar->setFormat(
    QString("Transition %1 in progress... %p%").arg(traj_counter_));
  ui->progressBar->setValue(0);
  app_.runTransition(traj_counter_);
  connect(this, SIGNAL(progressUpdateTrigger(int, double)), this,
          SLOT(progressUpdate(int, double)), Qt::QueuedConnection);
}

void JointsPVTDialog::on_pushButton_pause_clicked()
{
  CLOG(TRACE, "event") << !app_.isPaused();
  app_.pause();
  ui->pushButton_pause->setText(app_.isPaused() ? "Resume" : "Pause");
}

void JointsPVTDialog::on_pushButton_stop_clicked()
{
  CLOG(TRACE, "event");
  app_.stop(); // stop trajectory/transition in progress
  stop();      // update gui
}

void JointsPVTDialog::on_pushButton_return_clicked()
{
  CLOG(TRACE, "event");
  close();
}

//--------- Private functions ------------------------------------------------------//

void JointsPVTDialog::updatePlots(const TrajectorySet& traj_set)
{
  // Set 3D platform trajectory (fill scatter plot)
//  traj_display_.setTrajectory(traj_set.traj_platform);

  // Add a 2D line plot for each active actuator.
  size_t num_plots = 0;
  if (!traj_set.traj_cables_len.empty())
    num_plots = traj_set.traj_cables_len.size();
  else if (!traj_set.traj_motors_pos.empty())
    num_plots = traj_set.traj_motors_pos.size();
  else if (!traj_set.traj_motors_vel.empty())
    num_plots = traj_set.traj_motors_vel.size();
  else if (!traj_set.traj_motors_torque.empty())
    num_plots = traj_set.traj_motors_torque.size();

  if (grid_layout_ == nullptr)
  {
    auto vLine = new QFrame;
    vLine->setFrameShape(QFrame::VLine);
    vLine->setFrameShadow(QFrame::Sunken);
    ui->horizontalLayout_plots->addWidget(vLine);
  }
  else
  {
    ui->horizontalLayout_plots->removeItem(grid_layout_);
    chart_views_.clear();
    delete grid_layout_;
  }

  grid_layout_ = new QGridLayout;
  for (size_t i = 0; i < num_plots; i++)
  {
    QChart* chart         = new QChart();
    ChartView* chart_view = new ChartView(chart);
    switch (traj_set.traj_type)
    {
      case CABLE_LENGTH:
        chart_view->setCableTrajectory(traj_set.traj_cables_len[i]);
        break;
      case MOTOR_POSITION:
        chart_view->setMotorPosTrajectory(traj_set.traj_motors_pos[i]);
        break;
      case MOTOR_SPEED:
        chart_view->setMotorVelTrajectory(traj_set.traj_motors_vel[i]);
        break;
      case MOTOR_TORQUE:
        chart_view->setMotorTorqueTrajectory(traj_set.traj_motors_torque[i]);
        break;
      case NONE:
        break;
    }
    chart_view->setMinimumWidth(traj_display_.width() / 2);
    chart_views_.append(QSharedPointer<ChartView>(chart_view));

    int row = static_cast<int>(i) / 2;
    int col = i % 2;
    grid_layout_->addWidget(chart_view, row, col);
    grid_layout_->setColumnStretch(col, 1);
    grid_layout_->setRowStretch(row, 1);
  }

  ui->horizontalLayout_plots->addLayout(grid_layout_, 1);
  ui->horizontalLayout_plots->setStretch(0, 1);
  ui->horizontalLayout_plots->setSpacing(12);
  CLOG(INFO, "event") << "Joints PVT plots update";
}

void JointsPVTDialog::stop()
{
  ui->pushButton_start->setEnabled(true);
  ui->pushButton_pause->setText("Pause");
  ui->pushButton_pause->setDisabled(true);
  ui->pushButton_stop->setDisabled(true);
  ui->pushButton_return->setEnabled(true);
  ui->groupBox_inputs->setEnabled(true);
  ui->progressBar->setFormat("%p%");
  ui->progressBar->setValue(0);
  for (const auto& chart_view : chart_views_)
    chart_view->removeHighlight();
}
