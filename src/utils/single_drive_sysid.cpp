#include "utils/single_drive_sysid.h"
#include <QDebug>

SingleDriveSysID::SingleDriveSysID(QObject* parent, CableRobot* robot,
                                   ControllerSingleDrive* controller)
  : QObject(parent), robot_(robot), controller_(controller)
{
  log_timer_ = new QTimer(this);
  connect(log_timer_, SIGNAL(timeout()), this, SLOT(logData()));
}

SingleDriveSysID::~SingleDriveSysID()
{
  if (log_timer_->isActive())
    log_timer_->stop();
  disconnect(log_timer_, SIGNAL(timeout()), this, SLOT(logData()));
  delete log_timer_;
}

void SingleDriveSysID::start(const double cable_len)
{
  std::vector<double> traj = ComputeTrajectory(cable_len);

  log_timer_->start(kLogIntervalMsec_);

  pthread_mutex_lock(&robot_->Mutex());
  controller_->SetCableLenTrajectory(traj);
  pthread_mutex_unlock(&robot_->Mutex());

  QTimer::singleShot(kTrajLength_, this, SLOT(stopLogging(())));
}

void SingleDriveSysID::logData()
{
  robot_->CollectMeas();
  robot_->DumpMeas();
}

void SingleDriveSysID::stopLogging()
{
  log_timer_->stop();

  id_t motor_id = controller_->GetMotorsID()[0];
  controller_->SetCableLenTarget(robot_->GetActuatorStatus(motor_id).cable_length);
  pthread_mutex_lock(&robot_->Mutex());
  controller_->SetMode(ControlMode::CABLE_LENGTH);
  pthread_mutex_unlock(&robot_->Mutex());
}

std::vector<double> SingleDriveSysID::ComputeTrajectory(const double cable_len)
{
  std::vector<double> traj(kTrajLength_, cable_len);

  std::ifstream traj_file("/tmp/trajectory.txt");

  double traj_pt;
  uint i = 0;
  while (traj_file >> traj_pt)
  {
    traj[i++] += traj_pt;
    //    qDebug() << i-1 << traj[i-1];
  }

  return traj;
}
