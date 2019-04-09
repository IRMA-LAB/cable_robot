#include "gui/apps/joints_pvt_dialog.h"
#include "ui_joints_pvt_dialog.h"

JointsPVTDialog::JointsPVTDialog(QWidget* parent, CableRobot* robot,
                                 const vect<grabcdpr::ActuatorParams>& params)
  : QDialog(parent), ui(new Ui::JointsPVTDialog), traj_display_(this), robot_ptr_(robot),
    controller_(params, this)
{
  ui->setupUi(this);
  ui->horizontalLayout_display->addWidget(&traj_display_, 1);
  setAttribute(Qt::WA_DeleteOnClose);

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
  ui->checkBox->setEnabled(true);
  ui->pushButton_start->setEnabled(true);
  ui->pushButton_pause->setText("Pause");
  ui->pushButton_pause->setDisabled(true);
  ui->pushButton_stop->setDisabled(true);
  ui->groupBox->setEnabled(true);
}

void JointsPVTDialog::progressUpdate(const int progress_value)
{
  ui->progressBar->setValue(progress_value);
}

//--------- Private GUI slots -------------------------------------------------------//

void JointsPVTDialog::on_pushButton_fileSelection_clicked()
{
  CLOG(TRACE, "event");
  QString config_filename = QFileDialog::getOpenFileName(
    this, tr("Load Trajectory"), tr("../.."), tr("Trajectory file (*.txt)"));
  if (config_filename.isEmpty())
  {
    QMessageBox::warning(this, "File Error", "File name is empty!");
    return;
  }
  ui->lineEdit_inputFile->setText(config_filename);
}

void JointsPVTDialog::on_pushButton_read_clicked()
{
  CLOG(TRACE, "event");
  QString input_filename = ui->lineEdit_inputFile->text();
  if (input_filename.isEmpty())
  {
    CLOG(WARNING, "event") << "Trajectory file is empty";
    QMessageBox::warning(this, "File Error",
                         "Trajectory file is missing!\nPlease select a file containing "
                         "joints trajectory first.");
    return;
  }
  if (!readTrajectories(input_filename))
  {
    CLOG(WARNING, "event") << "Trajectory file is not valid";
    QMessageBox::warning(this, "File Error", "Trajectory file is not valid");
    return;
  }
  CLOG(INFO, "event") << "Read trajectories from '" << input_filename << "'";
  updatePlots();
  ui->pushButton_start->setEnabled(true);
}

void JointsPVTDialog::on_checkBox_toggled(bool checked)
{
  CLOG(TRACE, "event") << checked;
  ui->progressBar->setDisabled(checked);
}

void JointsPVTDialog::on_pushButton_start_clicked()
{
  CLOG(TRACE, "event");
  switch (traj_type_)
  {
    case CABLE_LENGTH:
      pthread_mutex_lock(&robot_ptr_->Mutex());
      controller_.SetCablesLenTrajectories(traj_cables_len_);
      pthread_mutex_unlock(&robot_ptr_->Mutex());
      break;
    case MOTOR_POSITION:
      pthread_mutex_lock(&robot_ptr_->Mutex());
      controller_.SetMotorsPosTrajectories(traj_motors_pos_);
      pthread_mutex_unlock(&robot_ptr_->Mutex());
      break;
    case MOTOR_SPEED:
      pthread_mutex_lock(&robot_ptr_->Mutex());
      controller_.SetMotorsVelTrajectories(traj_motors_vel_);
      pthread_mutex_unlock(&robot_ptr_->Mutex());
      break;
    case MOTOR_TORQUE:
      pthread_mutex_lock(&robot_ptr_->Mutex());
      controller_.SetMotorsTorqueTrajectories(traj_motors_torque_);
      pthread_mutex_unlock(&robot_ptr_->Mutex());
      break;
    case NONE:
      return;
  }

  ui->checkBox->setDisabled(true);
  ui->pushButton_start->setDisabled(true);
  ui->pushButton_pause->setEnabled(true);
  ui->pushButton_stop->setEnabled(true);
  ui->groupBox->setDisabled(true);
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

  handleTrajectoryCompleted(); // update gui
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

  traj_cables_len_.clear();
  traj_motors_pos_.clear();
  traj_motors_vel_.clear();
  traj_motors_torque_.clear();

  // Read header yielding information about trajectory type and involved motors
  QTextStream s(&ifile);
  QStringList header = s.readLine().split(" ");
  traj_type_         = static_cast<ControlMode>(header[0].toUShort());
  bool relative      = static_cast<bool>(header[1].toUShort());
  vect<id_t> motors_id;
  for (int i = 2; i < header.size(); i++)
    motors_id.push_back(header[i].toUInt());

  // Fill trajectories accordingly reading text body line-by-line
  switch (traj_type_)
  {
    case CABLE_LENGTH:
    {
      traj_cables_len_.resize(motors_id.size());
      vectD current_cables_len(traj_cables_len_.size());
      for (size_t i = 0; i < motors_id.size(); i++)
      {
        traj_cables_len_[i].id = motors_id[i];
        current_cables_len[i]  = robot_ptr_->GetActuatorStatus(motors_id[i]).cable_length;
      }
      while (!s.atEnd())
      {
        QStringList line = s.readLine().split(" ");
        for (auto& traj : traj_cables_len_)
          traj.timestamps.push_back(line[0].toDouble());
        for (int i = 1; i < line.size(); i++)
        {
          traj_cables_len_[static_cast<size_t>(i) - 1].values.push_back(
            line[i].toDouble());
          if (relative)
            traj_cables_len_[static_cast<size_t>(i) - 1].values.back() +=
              current_cables_len[static_cast<size_t>(i) - 1];
        }
      }
      break;
    }
    case MOTOR_POSITION:
    {
      traj_motors_pos_.resize(motors_id.size());
      vectI current_motors_pos(traj_cables_len_.size());
      for (size_t i = 0; i < motors_id.size(); i++)
      {
        traj_motors_pos_[i].id = motors_id[i];
        current_motors_pos[i] =
          robot_ptr_->GetActuatorStatus(motors_id[i]).motor_position;
      }
      while (!s.atEnd())
      {
        QStringList line = s.readLine().split(" ");
        for (auto& traj : traj_motors_pos_)
          traj.timestamps.push_back(line[0].toDouble());
        for (int i = 1; i < line.size(); i++)
        {
          traj_motors_pos_[static_cast<size_t>(i) - 1].values.push_back(line[i].toInt());
          if (relative)
            traj_cables_len_[static_cast<size_t>(i) - 1].values.back() +=
              current_motors_pos[static_cast<size_t>(i) - 1];
        }
      }
      break;
    }
    case MOTOR_SPEED:
      traj_motors_vel_.resize(motors_id.size());
      for (size_t i = 0; i < motors_id.size(); i++)
        traj_motors_vel_[i].id = motors_id[i];
      while (!s.atEnd())
      {
        QStringList line = s.readLine().split(" ");
        for (auto& traj : traj_motors_vel_)
          traj.timestamps.push_back(line[0].toDouble());
        for (int i = 1; i < line.size(); i++)
          traj_motors_vel_[static_cast<size_t>(i) - 1].values.push_back(line[i].toInt());
      }
      break;
    case MOTOR_TORQUE:
    {
      traj_motors_torque_.resize(motors_id.size());
      vectS current_motors_torque(traj_cables_len_.size());
      for (size_t i = 0; i < motors_id.size(); i++)
      {
        traj_motors_torque_[i].id = motors_id[i];
        current_motors_torque[i] =
          robot_ptr_->GetActuatorStatus(motors_id[i]).motor_torque;
      }
      while (!s.atEnd())
      {
        QStringList line = s.readLine().split(" ");
        for (auto& traj : traj_motors_torque_)
          traj.timestamps.push_back(line[0].toDouble());
        for (int i = 1; i < line.size(); i++)
        {
          traj_motors_torque_[static_cast<size_t>(i) - 1].values.push_back(
            line[i].toShort());
          if (relative)
            traj_cables_len_[static_cast<size_t>(i) - 1].values.back() +=
              current_motors_torque[static_cast<size_t>(i) - 1];
        }
      }
      break;
    }
    case NONE:
      return false;
  }
  return true;
}

void JointsPVTDialog::updatePlots()
{
  traj_display_.setTrajectory(traj_platform_);

  size_t num_plots = 0;
  if (!traj_cables_len_.empty())
    num_plots = traj_cables_len_.size();
  else if (!traj_motors_pos_.empty())
    num_plots = traj_motors_pos_.size();
  else if (!traj_motors_vel_.empty())
    num_plots = traj_motors_vel_.size();
  else if (!traj_motors_torque_.empty())
    num_plots = traj_motors_torque_.size();

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
    switch (traj_type_)
    {
      case CABLE_LENGTH:
        chartView->setCableTrajectory(traj_cables_len_[i]);
        break;
      case MOTOR_POSITION:
        chartView->setMotorPosTrajectory(traj_motors_pos_[i]);
        break;
      case MOTOR_SPEED:
        chartView->setMotorVelTrajectory(traj_motors_vel_[i]);
        break;
      case MOTOR_TORQUE:
        chartView->setMotorTorqueTrajectory(traj_motors_torque_[i]);
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
