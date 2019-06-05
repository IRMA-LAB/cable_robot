#include "apps/joints_pvt_app.h"

//------------------------------------------------------------------------------------//
//--------- Joints PVT App Data class ------------------------------------------------//
//------------------------------------------------------------------------------------//

JointsPVTAppData::JointsPVTAppData() {}

JointsPVTAppData::JointsPVTAppData(const int _traj_idx) : traj_idx(_traj_idx) {}

//------------------------------------------------------------------------------------//
//--------- Joints PVT App class -----------------------------------------------------//
//------------------------------------------------------------------------------------//

// For static constexpr passed by reference we need a dummy definition no matter what
constexpr char* JointsPVTApp::kStatesStr[];

JointsPVTApp::JointsPVTApp(QObject* parent, CableRobot* robot,
                           const vect<grabcdpr::ActuatorParams>& params)
  : QObject(parent), StateMachine(ST_MAX_STATES), robot_ptr_(robot),
    controller_(params, robot->GetRtCycleTimeNsec(), this)
{
  connect(&controller_, SIGNAL(trajectoryProgressStatus(int)), this,
          SLOT(progressUpdate(int)), Qt::ConnectionType::QueuedConnection);
  connect(&controller_, SIGNAL(trajectoryCompleted()), this,
          SLOT(handleTrajectoryCompleted()), Qt::ConnectionType::QueuedConnection);
  connect(this, SIGNAL(printToQConsole(QString)), this, SLOT(logInfo(QString)),
          Qt::ConnectionType::DirectConnection);
  connect(this, SIGNAL(stopWaitingCmd()), robot_ptr_, SLOT(stopWaiting()));

  controller_.SetMotorsID(robot_ptr_->GetActiveMotorsID());
  robot->SetController(&controller_);

  prev_state_ = ST_MAX_STATES;
  ExternalEvent(ST_IDLE);
}

JointsPVTApp::~JointsPVTApp()
{
  clearAllTrajectories();

  disconnect(&controller_, SIGNAL(trajectoryProgressStatus(int)), this,
             SLOT(progressUpdate(int)));
  disconnect(&controller_, SIGNAL(trajectoryCompleted()), this,
             SLOT(handleTrajectoryCompleted()));
  disconnect(this, SIGNAL(printToQConsole(QString)), this, SLOT(logInfo(QString)));
  disconnect(this, SIGNAL(stopWaitingCmd()), robot_ptr_, SLOT(stopWaiting()));

  robot_ptr_->SetController(nullptr);
}

//--------- Public functions --------------------------------------------------------//

void JointsPVTApp::pause()
{
  pthread_mutex_lock(&robot_ptr_->Mutex());
  if (!controller_.requestPending())
  {
    if (controller_.IsPaused())
      controller_.resumeTrajectoryFollowing();
    else
      controller_.pauseTrajectoryFollowing();
  }
  pthread_mutex_unlock(&robot_ptr_->Mutex());
}

const TrajectorySet& JointsPVTApp::getTrajectorySet(const int traj_idx) const
{
  return traj_sets_[traj_idx];
}

//--------- External Events ---------------------------------------------------------//

void JointsPVTApp::clearAllTrajectories()
{
  CLOG(TRACE, "event");
  // clang-format off
  BEGIN_TRANSITION_MAP                         // - Current State -
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)     // ST_IDLE
      TRANSITION_MAP_ENTRY (ST_IDLE)           // ST_READY
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)     // ST_TRANSITION
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)     // ST_TRAJECTORY_FOLLOW
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

bool JointsPVTApp::readTrajectories(const QString& ifilepath)
{
  CLOG(TRACE, "event") << "from '" << ifilepath << "'";
  QFile ifile(ifilepath);
  if (!ifile.open(QIODevice::ReadOnly | QIODevice::Text))
  {
    ExternalEvent(ST_IDLE);
    return false;
  }

  // Read header yielding information about trajectory type and involved motors
  QTextStream s(&ifile);
  QStringList header = s.readLine().split(" ");
  TrajectorySet traj_set;
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
  CLOG(INFO, "event") << "Trajectory parsed";
  ExternalEvent(ST_READY);
  return true;
}

void JointsPVTApp::runTransition(const int traj_idx)
{
  CLOG(TRACE, "event") << "of trajectory set #" << traj_idx;
  JointsPVTAppData* data = new JointsPVTAppData(traj_idx);

  // clang-format off
  BEGIN_TRANSITION_MAP                          // - Current State -
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)      // ST_IDLE
      TRANSITION_MAP_ENTRY (ST_TRANSITION)      // ST_READY
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)      // ST_TRANSITION
      TRANSITION_MAP_ENTRY (ST_TRANSITION)      // ST_TRAJECTORY_FOLLOW
  END_TRANSITION_MAP(data)
  // clang-format on
}

void JointsPVTApp::sendTrajectories(const int traj_idx)
{
  CLOG(TRACE, "event") << "of trajectory set #" << traj_idx;
  JointsPVTAppData* data = new JointsPVTAppData(traj_idx);

  // clang-format off
  BEGIN_TRANSITION_MAP                               // - Current State -
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)           // ST_IDLE
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)           // ST_READY
      TRANSITION_MAP_ENTRY (ST_TRAJECTORY_FOLLOW)    // ST_TRANSITION
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)           // ST_TRAJECTORY_FOLLOW
  END_TRANSITION_MAP(data)
  // clang-format on
}

void JointsPVTApp::stop()
{
  if (robot_ptr_->isWaiting())
    emit stopWaitingCmd();

  // clang-format off
  BEGIN_TRANSITION_MAP                         // - Current State -
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)     // ST_IDLE
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)     // ST_READY
      TRANSITION_MAP_ENTRY (ST_READY)          // ST_TRANSITION
      TRANSITION_MAP_ENTRY (ST_READY)          // ST_TRAJECTORY_FOLLOW
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

//--------- Private slots -----------------------------------------------------------//

void JointsPVTApp::handleTrajectoryCompleted()
{
  if (GetCurrentState() == ST_TRANSITION)
    emit transitionComplete();
  else if (GetCurrentState() == ST_TRAJECTORY_FOLLOW)
    emit trajectoryComplete();
}

void JointsPVTApp::progressUpdate(const int progress_value)
{
  emit trajectoryProgress(progress_value);
}

void JointsPVTApp::logInfo(const QString& text) const
{
  if (text.contains("warning", Qt::CaseSensitivity::CaseInsensitive))
    CLOG(WARNING, "event") << text;
  else if (text.contains("error", Qt::CaseSensitivity::CaseInsensitive))
    CLOG(ERROR, "event") << text;
  else
    CLOG(INFO, "event") << text;
}

//--------- States actions -----------------------------------------------------------//

STATE_DEFINE(JointsPVTApp, Idle, NoEventData)
{
  PrintStateTransition(prev_state_, ST_IDLE);
  prev_state_ = ST_IDLE;

  traj_sets_.clear();
}

STATE_DEFINE(JointsPVTApp, Ready, NoEventData)
{
  PrintStateTransition(prev_state_, ST_READY);
  prev_state_ = ST_READY;

  pthread_mutex_lock(&robot_ptr_->Mutex());
  controller_.stopTrajectoryFollowing();
  pthread_mutex_unlock(&robot_ptr_->Mutex());
}

STATE_DEFINE(JointsPVTApp, Transition, JointsPVTAppData)
{
  PrintStateTransition(prev_state_, ST_TRANSITION);
  prev_state_ = ST_TRANSITION;

  static constexpr double kMaxCableSpeed = 0.002; // [m/s]

  if (traj_sets_[data->traj_idx].traj_type == CABLE_LENGTH)
  {
    vect<TrajectoryD> transition_trajectories =
      traj_sets_[data->traj_idx].traj_cables_len;
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
      t        = std::max(t, grabrt::NanoSec2Sec(robot_ptr_->GetRtCycleTimeNsec()));
      // Set a simple trajectory composed by two waypoints (begin, end).
      transition_trajectories[i].timestamps = {0.0, t};
      transition_trajectories[i].values     = {current_cable_len, target_cable_len};
      CLOG(INFO, "event") << QString(
                               "Cable #%1 transitioning from %2 m to %3 m in %4 sec")
                               .arg(transition_trajectories[i].id)
                               .arg(current_cable_len)
                               .arg(target_cable_len)
                               .arg(t);
    }
    // Send trajectories
    pthread_mutex_lock(&robot_ptr_->Mutex());
    controller_.SetCablesLenTrajectories(transition_trajectories);
    pthread_mutex_unlock(&robot_ptr_->Mutex());
  }
  else if (traj_sets_[data->traj_idx].traj_type == MOTOR_POSITION)
  {
    vect<TrajectoryI> transition_trajectories =
      traj_sets_[data->traj_idx].traj_motors_pos;
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
      CLOG(INFO, "event") << QString("Motor #%1 transitioning from %2 to %3")
                               .arg(transition_trajectories[i].id)
                               .arg(current_motor_pos)
                               .arg(target_motor_pos);
    }
    // Send trajectories
    pthread_mutex_lock(&robot_ptr_->Mutex());
    controller_.SetMotorsPosTrajectories(transition_trajectories);
    pthread_mutex_unlock(&robot_ptr_->Mutex());
  }
}

STATE_DEFINE(JointsPVTApp, TrajectoryFollow, JointsPVTAppData)
{
  PrintStateTransition(prev_state_, ST_TRAJECTORY_FOLLOW);
  prev_state_ = ST_TRAJECTORY_FOLLOW;

  if (robot_ptr_->WaitUntilPlatformSteady(-1.) != RetVal::OK)
  {
    InternalEvent(ST_READY);
    return;
  }

  switch (traj_sets_[data->traj_idx].traj_type)
  {
    case CABLE_LENGTH:
      pthread_mutex_lock(&robot_ptr_->Mutex());
      controller_.SetCablesLenTrajectories(traj_sets_[data->traj_idx].traj_cables_len);
      pthread_mutex_unlock(&robot_ptr_->Mutex());
      break;
    case MOTOR_POSITION:
      pthread_mutex_lock(&robot_ptr_->Mutex());
      controller_.SetMotorsPosTrajectories(traj_sets_[data->traj_idx].traj_motors_pos);
      pthread_mutex_unlock(&robot_ptr_->Mutex());
      break;
    case MOTOR_SPEED:
      pthread_mutex_lock(&robot_ptr_->Mutex());
      controller_.SetMotorsVelTrajectories(traj_sets_[data->traj_idx].traj_motors_vel);
      pthread_mutex_unlock(&robot_ptr_->Mutex());
      break;
    case MOTOR_TORQUE:
      pthread_mutex_lock(&robot_ptr_->Mutex());
      controller_.SetMotorsTorqueTrajectories(
        traj_sets_[data->traj_idx].traj_motors_torque);
      pthread_mutex_unlock(&robot_ptr_->Mutex());
      break;
    case NONE:
      return;
  }
}

//--------- Private functions -------------------------------------------------------//

void JointsPVTApp::setCablesLenTraj(const bool relative, const vect<id_t>& motors_id,
                                    QTextStream& s, TrajectorySet& traj_set)
{
  CLOG(INFO, "event") << QString("File contains %1 cables length trajectories")
                           .arg(relative ? "relative" : "absolute");
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

void JointsPVTApp::setMotorPosTraj(const bool relative, const vect<id_t>& motors_id,
                                   QTextStream& s, TrajectorySet& traj_set)
{
  CLOG(INFO, "event") << QString("File contains %1 motors position trajectories")
                           .arg(relative ? "relative" : "absolute");
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

void JointsPVTApp::setMotorVelTraj(const vect<id_t>& motors_id, QTextStream& s,
                                   TrajectorySet& traj_set)
{
  CLOG(INFO, "event") << "File contains motors velocity trajectories";
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

void JointsPVTApp::setMotorTorqueTraj(const bool relative, const vect<id_t>& motors_id,
                                      QTextStream& s, TrajectorySet& traj_set)
{
  CLOG(INFO, "event") << QString("File contains %1 motors torque trajectories")
                           .arg(relative ? "relative" : "absolute");
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

void JointsPVTApp::PrintStateTransition(const States current_state,
                                        const States new_state) const
{
  if (current_state == new_state)
    return;
  QString msg;
  if (current_state != ST_MAX_STATES)
    msg = QString("Joints PVT app state transition: %1 --> %2")
            .arg(kStatesStr[current_state], kStatesStr[new_state]);
  else
    msg = QString("Joints PVT app initial state: %1").arg(kStatesStr[new_state]);
  emit printToQConsole(msg);
}
