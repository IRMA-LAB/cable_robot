#include "gui/apps/joints_pvt_dialog.h"
#include "ui_joints_pvt_dialog.h"

JointsPVTDialog::JointsPVTDialog(QWidget* parent, CableRobot* robot,
                                 const vect<grabcdpr::ActuatorParams>& params)
  : QDialog(parent), ui(new Ui::JointsPVTDialog), traj_display_(this),
    input_form_pos_(kInputFormPosInit_), robot_ptr_(robot), controller_(params, this),
    traj_counter_(0), transition_in_progress_(false)
{
  ui->setupUi(this);
  ui->horizontalLayout_display->addWidget(&traj_display_, 1);
  setAttribute(Qt::WA_DeleteOnClose);

  line_edits_.append(new InputForm(this));
  ui->verticalLayout_inputSource->insertWidget(kInputFormPosInit_ - 1, line_edits_.last());

  connect(&controller_, SIGNAL(trajectoryProgressStatus(int)), this,
          SLOT(progressUpdate(int)), Qt::ConnectionType::QueuedConnection);
  connect(&controller_, SIGNAL(trajectoryCompleted()), this,
          SLOT(handleTrajectoryCompleted()), Qt::ConnectionType::QueuedConnection);
  robot->SetController(&controller_);
}

JointsPVTDialog::~JointsPVTDialog()
{
  disconnect(&controller_, SIGNAL(trajectoryProgressStatus(int)), this,
             SLOT(progressUpdate(int)));
  disconnect(&controller_, SIGNAL(trajectoryCompleted()), this,
             SLOT(handleTrajectoryCompleted()));
  robot_ptr_->SetController(NULL);
  delete ui;
}

//--------- Private slots -----------------------------------------------------------//

void JointsPVTDialog::handleTrajectoryCompleted()
{
  if (transition_in_progress_)
  {
    transition_in_progress_ = false;
    ui->progressBar->setFormat(
      QString("Trajectory %1 in progress... %p%").arg(traj_counter_));
    ui->progressBar->setValue(0);
    sendTrajectories(traj_sets_[traj_counter_]);
    return;
  }

  if (++traj_counter_ >= traj_sets_.size())
  {
    if (!ui->checkBox_infLoop->isChecked())
    {
      stop(); // all trajectories completed
      return;
    }
    traj_counter_ = 0; // infinite loop --> start over
  }

  // Run next trajectory.
  updatePlots(traj_sets_[traj_counter_]);
  ui->progressBar->setFormat(
    QString("Transition %1 in progress... %p%").arg(traj_counter_));
  ui->progressBar->setValue(0);
  runTransition(traj_sets_[traj_counter_]);
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
}

void JointsPVTDialog::progressUpdate(const int progress_value)
{
  ui->progressBar->setValue(progress_value);
}

//--------- Private GUI slots -------------------------------------------------------//

void JointsPVTDialog::on_pushButton_addTraj_clicked()
{
  line_edits_.append(new InputForm(this));
  ui->verticalLayout_inputSource->insertWidget(input_form_pos_++, line_edits_.last());
  ui->pushButton_removeTraj->setEnabled(true);
}

void JointsPVTDialog::on_pushButton_removeTraj_clicked()
{
  ui->verticalLayout_inputSource->removeWidget(line_edits_.last());
  delete line_edits_.last();
  line_edits_.pop_back();
  if (--input_form_pos_ <= kInputFormPosInit_)
    ui->pushButton_removeTraj->setDisabled(true);
}

void JointsPVTDialog::on_pushButton_read_clicked()
{
  CLOG(TRACE, "event");
  QVector<QString> input_filenames;
  for (InputForm* form : line_edits_)
  {
    input_filenames.append(form->getFilepath());
  }
  if (input_filenames.isEmpty())
  {
    CLOG(WARNING, "event") << "Trajectory files empty";
    QMessageBox::warning(
      this, "File Error",
      "Trajectory files are missing!\nPlease select at least one file containing "
      "joints trajectory first.");
    return;
  }

  // Clear all trajectories
  traj_sets_.clear();

  for (const QString& input_filename : input_filenames)
  {
    if (!readTrajectories(input_filename))
    {
      CLOG(WARNING, "event") << "Trajectory file is not valid";
      QMessageBox::warning(this, "File Error",
                           "Trajectory file '" + input_filename + "' is not valid");
      return;
    }
    CLOG(INFO, "event") << "Read trajectories from '" << input_filename << "'";
  }

  updatePlots(traj_sets_.first());
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
  ui->pushButton_pause->setEnabled(true);
  ui->pushButton_stop->setEnabled(true);
  ui->pushButton_return->setDisabled(true);
  ui->groupBox_inputs->setDisabled(true);

  traj_counter_ = 0; // reset
  ui->progressBar->setFormat(
    QString("Transition %1 in progress... %p%").arg(traj_counter_));
  ui->progressBar->setValue(0);
  runTransition(traj_sets_.first());
}

void JointsPVTDialog::on_pushButton_pause_clicked()
{
  CLOG(TRACE, "event") << !controller_.IsPaused();
  pthread_mutex_lock(&robot_ptr_->Mutex());
  bool is_paused = controller_.IsPaused();
  controller_.PauseTrajectoryFollowing(!is_paused);
  pthread_mutex_unlock(&robot_ptr_->Mutex());

  ui->pushButton_pause->setText(controller_.IsPaused() ? "Resume" : "Pause");
}

void JointsPVTDialog::on_pushButton_stop_clicked()
{
  CLOG(TRACE, "event");
  pthread_mutex_lock(&robot_ptr_->Mutex());
  controller_.StopTrajectoryFollowing();
  pthread_mutex_unlock(&robot_ptr_->Mutex());

  stop(); // update gui
}

void JointsPVTDialog::on_pushButton_return_clicked()
{
  CLOG(TRACE, "event");
  if (!controller_.TargetReached())
    on_pushButton_stop_clicked();
  close();
}

//--------- Private functions ------------------------------------------------------//

bool JointsPVTDialog::readTrajectories(const QString& ifilepath)
{
  QFile ifile(ifilepath);
  if (!ifile.open(QIODevice::ReadOnly | QIODevice::Text))
    return false;
  TrajectorySet traj_set;

  // Read header yielding information about trajectory type and involved motors
  QTextStream s(&ifile);
  QStringList header = s.readLine().split(" ");
  traj_set.traj_type = static_cast<ControlMode>(header[0].toUShort());
  bool relative      = static_cast<bool>(header[1].toUShort());
  vect<id_t> motors_id;
  for (int i = 2; i < header.size(); i++)
    motors_id.push_back(header[i].toUInt());

  // Fill trajectories accordingly reading text body line-by-line
  switch (traj_set.traj_type)
  {
    case CABLE_LENGTH:
      setCablesLenTraj(relative, motors_id, s, traj_set);
      break;
    case MOTOR_POSITION:
      setMotorPosTraj(relative, motors_id, s, traj_set);
      break;
    case MOTOR_SPEED:
      setMotorVelTraj(motors_id, s, traj_set);
      break;
    case MOTOR_TORQUE:
      setMotorTorqueTraj(relative, motors_id, s, traj_set);
      break;
    case NONE:
      return false;
  }
  traj_sets_.append(traj_set);
  return true;
}

void JointsPVTDialog::setCablesLenTraj(const bool relative, const vect<id_t>& motors_id,
                                       QTextStream& s, TrajectorySet& traj_set)
{
  traj_set.traj_cables_len.resize(motors_id.size());
  vectD current_cables_len(traj_set.traj_cables_len.size());
  for (size_t i = 0; i < motors_id.size(); i++)
  {
    traj_set.traj_cables_len[i].id = motors_id[i];
    current_cables_len[i] = robot_ptr_->GetActuatorStatus(motors_id[i]).cable_length;
  }
  while (!s.atEnd())
  {
    QStringList line = s.readLine().split(" ");
    for (auto& traj : traj_set.traj_cables_len)
      traj.timestamps.push_back(line[0].toDouble());
    for (int i = 1; i < line.size(); i++)
    {
      traj_set.traj_cables_len[static_cast<size_t>(i) - 1].values.push_back(
        line[i].toDouble());
      if (relative)
        traj_set.traj_cables_len[static_cast<size_t>(i) - 1].values.back() +=
          current_cables_len[static_cast<size_t>(i) - 1];
    }
  }
}

void JointsPVTDialog::setMotorPosTraj(const bool relative, const vect<id_t>& motors_id,
                                      QTextStream& s, TrajectorySet& traj_set)
{
  traj_set.traj_motors_pos.resize(motors_id.size());
  vectI current_motors_pos(traj_set.traj_cables_len.size());
  for (size_t i = 0; i < motors_id.size(); i++)
  {
    traj_set.traj_motors_pos[i].id = motors_id[i];
    current_motors_pos[i] = robot_ptr_->GetActuatorStatus(motors_id[i]).motor_position;
  }
  while (!s.atEnd())
  {
    QStringList line = s.readLine().split(" ");
    for (auto& traj : traj_set.traj_motors_pos)
      traj.timestamps.push_back(line[0].toDouble());
    for (int i = 1; i < line.size(); i++)
    {
      traj_set.traj_motors_pos[static_cast<size_t>(i) - 1].values.push_back(
        line[i].toInt());
      if (relative)
        traj_set.traj_cables_len[static_cast<size_t>(i) - 1].values.back() +=
          current_motors_pos[static_cast<size_t>(i) - 1];
    }
  }
}

void JointsPVTDialog::setMotorVelTraj(const vect<id_t>& motors_id, QTextStream& s,
                                      TrajectorySet& traj_set)
{
  traj_set.traj_motors_vel.resize(motors_id.size());
  for (size_t i = 0; i < motors_id.size(); i++)
    traj_set.traj_motors_vel[i].id = motors_id[i];
  while (!s.atEnd())
  {
    QStringList line = s.readLine().split(" ");
    for (auto& traj : traj_set.traj_motors_vel)
      traj.timestamps.push_back(line[0].toDouble());
    for (int i = 1; i < line.size(); i++)
      traj_set.traj_motors_vel[static_cast<size_t>(i) - 1].values.push_back(
        line[i].toInt());
  }
}

void JointsPVTDialog::setMotorTorqueTraj(const bool relative, const vect<id_t>& motors_id,
                                         QTextStream& s, TrajectorySet& traj_set)
{
  traj_set.traj_motors_torque.resize(motors_id.size());
  vectS current_motors_torque(traj_set.traj_cables_len.size());
  for (size_t i = 0; i < motors_id.size(); i++)
  {
    traj_set.traj_motors_torque[i].id = motors_id[i];
    current_motors_torque[i] = robot_ptr_->GetActuatorStatus(motors_id[i]).motor_torque;
  }
  while (!s.atEnd())
  {
    QStringList line = s.readLine().split(" ");
    for (auto& traj : traj_set.traj_motors_torque)
      traj.timestamps.push_back(line[0].toDouble());
    for (int i = 1; i < line.size(); i++)
    {
      traj_set.traj_motors_torque[static_cast<size_t>(i) - 1].values.push_back(
        line[i].toShort());
      if (relative)
        traj_set.traj_cables_len[static_cast<size_t>(i) - 1].values.back() +=
          current_motors_torque[static_cast<size_t>(i) - 1];
    }
  }
}

void JointsPVTDialog::updatePlots(const TrajectorySet& traj_set)
{
  // Set 3D platform trajectory (fill scatter plot)
  traj_display_.setTrajectory(traj_set.traj_platform);

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

  if (grid_layout_ == NULL)
  {
    auto vLine = new QFrame;
    vLine->setFrameShape(QFrame::VLine);
    vLine->setFrameShadow(QFrame::Sunken);
    ui->horizontalLayout_plots->addWidget(vLine);
  }
  else
  {
    ui->horizontalLayout_plots->removeItem(grid_layout_);
    delete grid_layout_;
  }

  grid_layout_ = new QGridLayout;
  for (size_t i = 0; i < num_plots; i++)
  {
    QChart* chart        = new QChart();
    ChartView* chartView = new ChartView(chart);
    switch (traj_set.traj_type)
    {
      case CABLE_LENGTH:
        chartView->setCableTrajectory(traj_set.traj_cables_len[i]);
        break;
      case MOTOR_POSITION:
        chartView->setMotorPosTrajectory(traj_set.traj_motors_pos[i]);
        break;
      case MOTOR_SPEED:
        chartView->setMotorVelTrajectory(traj_set.traj_motors_vel[i]);
        break;
      case MOTOR_TORQUE:
        chartView->setMotorTorqueTrajectory(traj_set.traj_motors_torque[i]);
        break;
      case NONE:
        break;
    }
    chartView->setMinimumWidth(traj_display_.width() / 2);
    int row = static_cast<int>(i) / 2;
    int col = i % 2;
    grid_layout_->addWidget(chartView, row, col);
    grid_layout_->setColumnStretch(col, 1);
    grid_layout_->setRowStretch(row, 1);
  }

  ui->horizontalLayout_plots->addLayout(grid_layout_, 1);
  ui->horizontalLayout_plots->setStretch(0, 1);
  ui->horizontalLayout_plots->setSpacing(12);
}

void JointsPVTDialog::runTransition(const TrajectorySet& traj_set)
{
  static constexpr double kMaxCableSpeed = 0.001; // [m/s]

  if (traj_set.traj_type == CABLE_LENGTH)
  {
    transition_in_progress_                   = true;
    vect<TrajectoryD> transition_trajectories = traj_set.traj_cables_len;
    for (size_t i = 0; i < transition_trajectories.size(); i++)
    {
      // First waypoint of next trajectory becomes end point of transition.
      double target_cable_len = transition_trajectories[i].values.front();
      transition_trajectories[i].timestamps.clear();
      transition_trajectories[i].values.clear();
      // Current cable length becomes start point of transition.
      double current_cable_len =
        robot_ptr_->GetActuatorStatus(transition_trajectories[i].id).cable_length;
      // Calculate necessary time to move from A to B with fixed constant velocity.
      double t = std::abs(target_cable_len - current_cable_len) / kMaxCableSpeed;
      // Set a simple trajectory composed by two waypoints (begin, end).
      transition_trajectories[i].timestamps = {0, t};
      transition_trajectories[i].values     = {current_cable_len, target_cable_len};
    }
    // Send trajectories
    pthread_mutex_lock(&robot_ptr_->Mutex());
    controller_.SetCablesLenTrajectories(transition_trajectories);
    pthread_mutex_unlock(&robot_ptr_->Mutex());
  }
  else if (traj_set.traj_type == MOTOR_POSITION)
  {
    transition_in_progress_                   = true;
    vect<TrajectoryI> transition_trajectories = traj_set.traj_motors_pos;
    for (size_t i = 0; i < transition_trajectories.size(); i++)
    {
      // First waypoint of next trajectory becomes end point of transition.
      int target_motor_pos = transition_trajectories[i].values.front();
      transition_trajectories[i].timestamps.clear();
      transition_trajectories[i].values.clear();
      double max_motor_speed = robot_ptr_->GetActuator(transition_trajectories[i].id)
                                 ->GetWinch()
                                 .LengthToCounts(kMaxCableSpeed); // [counts/s]
      // Current motor position becomes start point of transition.
      int current_motor_pos =
        robot_ptr_->GetActuatorStatus(transition_trajectories[i].id).motor_position;
      // Calculate necessary time to move from A to B with fixed constant velocity.
      double t = std::abs(target_motor_pos - current_motor_pos) / max_motor_speed;
      // Set a simple trajectory composed by two waypoints (begin, end).
      transition_trajectories[i].timestamps = {0, t};
      transition_trajectories[i].values     = {current_motor_pos, target_motor_pos};
    }
    // Send trajectories
    pthread_mutex_lock(&robot_ptr_->Mutex());
    controller_.SetMotorsPosTrajectories(transition_trajectories);
    pthread_mutex_unlock(&robot_ptr_->Mutex());
  }
}

void JointsPVTDialog::sendTrajectories(const TrajectorySet& traj_set)
{
  switch (traj_set.traj_type)
  {
    case CABLE_LENGTH:
      pthread_mutex_lock(&robot_ptr_->Mutex());
      controller_.SetCablesLenTrajectories(traj_set.traj_cables_len);
      pthread_mutex_unlock(&robot_ptr_->Mutex());
      break;
    case MOTOR_POSITION:
      pthread_mutex_lock(&robot_ptr_->Mutex());
      controller_.SetMotorsPosTrajectories(traj_set.traj_motors_pos);
      pthread_mutex_unlock(&robot_ptr_->Mutex());
      break;
    case MOTOR_SPEED:
      pthread_mutex_lock(&robot_ptr_->Mutex());
      controller_.SetMotorsVelTrajectories(traj_set.traj_motors_vel);
      pthread_mutex_unlock(&robot_ptr_->Mutex());
      break;
    case MOTOR_TORQUE:
      pthread_mutex_lock(&robot_ptr_->Mutex());
      controller_.SetMotorsTorqueTrajectories(traj_set.traj_motors_torque);
      pthread_mutex_unlock(&robot_ptr_->Mutex());
      break;
    case NONE:
      return;
  }
}
