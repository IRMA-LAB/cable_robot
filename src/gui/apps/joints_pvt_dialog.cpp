#include "gui/apps/joints_pvt_dialog.h"
#include "ui_joints_pvt_dialog.h"

JointsPVTDialog::JointsPVTDialog(QWidget* parent, CableRobot* robot)
  : QDialog(parent), ui(new Ui::JointsPVTDialog), robot_ptr_(robot), controller_(this)
{
  ui->setupUi(this);

  connect(&controller_, SIGNAL(trajectoryCompleted()), this,
          SLOT(setTrajectoryCompleted()), Qt::ConnectionType::QueuedConnection);
  robot->SetController(&controller_);
}

JointsPVTDialog::~JointsPVTDialog()
{
  disconnect(&controller_, SIGNAL(trajectoryCompleted()), this,
             SLOT(setTrajectoryCompleted()));
  robot_ptr_->SetController(NULL);
  delete ui;
}

//--------- Private slots -----------------------------------------------------------//

void JointsPVTDialog::setTrajectoryCompleted()
{
  ui->pushButton_start->setEnabled(true);
  ui->pushButton_pause->setDisabled(true);
  ui->pushButton_stop->setDisabled(true);
  ui->groupBox->setEnabled(true);
}

//--------- Private GUI slots -------------------------------------------------------//

void JointsPVTDialog::on_pushButton_fileSelection_clicked()
{
  CLOG(TRACE, "event");
  QString config_filename =
    QFileDialog::getOpenFileName(this, tr("Load Optimization Results"), tr("../.."),
                                 tr("Optimization results (*.json)"));
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
  ui->pushButton_start->setEnabled(true);
}

void JointsPVTDialog::on_checkBox_toggled(bool checked) {}

void JointsPVTDialog::on_pushButton_start_clicked()
{
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

  ui->pushButton_start->setDisabled(true);
  ui->pushButton_pause->setEnabled(true);
  ui->pushButton_stop->setEnabled(true);
  ui->groupBox->setDisabled(true);
}

void JointsPVTDialog::on_pushButton_pause_clicked()
{
  pthread_mutex_lock(&robot_ptr_->Mutex());
  controller_.PauseTrajectoryFollowing(!controller_.IsPaused());
  pthread_mutex_unlock(&robot_ptr_->Mutex());

  ui->pushButton_pause->setText(controller_.IsPaused() ? "Resume" : "Pause");
}

void JointsPVTDialog::on_pushButton_stop_clicked()
{
  pthread_mutex_lock(&robot_ptr_->Mutex());
  controller_.StopTrajectoryFollowing();
  pthread_mutex_unlock(&robot_ptr_->Mutex());

  ui->pushButton_start->setEnabled(true);
  ui->pushButton_pause->setDisabled(true);
  ui->pushButton_stop->setDisabled(true);
  ui->groupBox->setEnabled(true);
}

void JointsPVTDialog::on_pushButton_return_clicked()
{
  if (!controller_.TargetReached())
    on_pushButton_stop_clicked();
  close();
}

//--------- Private functions ------------------------------------------------------//

bool JointsPVTDialog::readTrajectories(const QString& ifilepath) { return true; }
