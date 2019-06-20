#include "apps/manual_control_app.h"

MyData::MyData() {}

MyData::MyData(const qint16 _torque) { torque = _torque; }

std::ostream& operator<<(std::ostream& stream, const MyData& data)
{
  stream << "torque = " << data.torque;
  return stream;
}


// For static constexpr passed by reference we need a dummy definition no matter what
constexpr char* ManualControlApp::kStatesStr[];
const QString ManualControlApp::kExcitationTrajFilepath_ =
  SRCDIR "resources/trajectories/excitation_traj.txt";

ManualControlApp::ManualControlApp(QObject* parent, CableRobot* robot)
  : QObject(parent), StateMachine(ST_MAX_STATES), robot_ptr_(robot),
    controller_single_drive_(robot->GetRtCycleTimeNsec())
{
  prev_state_ = ST_MAX_STATES;
  ExternalEvent(ST_IDLE);

  controller_single_drive_.SetMotorTorqueSsErrTol(kTorqueSsErrTol_);
  active_actuators_id_ = robot_ptr_->GetActiveMotorsID();
  connect(this, SIGNAL(stopWaitingCmd()), robot_ptr_, SLOT(stopWaiting()));

  traj_cables_len_.clear();
  vect<grabcdpr::ActuatorParams> dummy_params;
  controller_joints_ptv_ =
  new ControllerJointsPVT(dummy_params, robot->GetRtCycleTimeNsec(), this);
  controller_joints_ptv_->SetMotorsID(active_actuators_id_);
  connect(controller_joints_ptv_, SIGNAL(trajectoryCompleted()), this,
          SLOT(stopLogging()), Qt::ConnectionType::QueuedConnection);
}

ManualControlApp::~ManualControlApp()
{
  disconnect(this, SIGNAL(stopWaitingCmd()), robot_ptr_, SLOT(stopWaiting()));
  if (controller_joints_ptv_ != nullptr)
    delete controller_joints_ptv_;
}

//--------- External events ----------------------------------------------------------//

void ManualControlApp::enable(MyData* data /*= nullptr*/)
{
  if (data == nullptr)
    CLOG(TRACE, "event") << "with NULL";
  else
    CLOG(TRACE, "event") << "with " << *data;

  // clang-format off
  BEGIN_TRANSITION_MAP                      // - Current State -
      TRANSITION_MAP_ENTRY (ST_ENABLED)     // ST_IDLE
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)  // ST_ENABLED
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)  // ST_POS_CONTROL
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)  // ST_TORQUE_CONTROL
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)  // ST_LOGGING
  END_TRANSITION_MAP(data)
  // clang-format on
}

void ManualControlApp::changeControlMode()
{
  CLOG(TRACE, "event") << "with NULL";

  // clang-format off
  BEGIN_TRANSITION_MAP                          // - Current State -
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)      // ST_IDLE
      TRANSITION_MAP_ENTRY (ST_POS_CONTROL)     // ST_ENABLED
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)      // ST_POS_CONTROL
      TRANSITION_MAP_ENTRY (ST_POS_CONTROL)     // ST_TORQUE_CONTROL
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)      // ST_LOGGING
  END_TRANSITION_MAP(nullptr)
  // clang-format on
}

void ManualControlApp::changeControlMode(MyData* data)
{
  CLOG(TRACE, "event") << "with " << *data;

  // clang-format off
  BEGIN_TRANSITION_MAP                          // - Current State -
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)      // ST_IDLE
      TRANSITION_MAP_ENTRY (ST_TORQUE_CONTROL)  // ST_ENABLED
      TRANSITION_MAP_ENTRY (ST_TORQUE_CONTROL)  // ST_POS_CONTROL
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)      // ST_TORQUE_CONTROL
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)      // ST_LOGGING
  END_TRANSITION_MAP(data)
  // clang-format on
}

void ManualControlApp::exciteAndLog()
{
  CLOG(TRACE, "event");

  // clang-format off
  BEGIN_TRANSITION_MAP                          // - Current State -
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)      // ST_IDLE
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)      // ST_ENABLED
      TRANSITION_MAP_ENTRY (ST_LOGGING)         // ST_POS_CONTROL
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)      // ST_TORQUE_CONTROL
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)      // ST_LOGGING
  END_TRANSITION_MAP(nullptr)
  // clang-format on
}

void ManualControlApp::stopLogging()
{
  CLOG(TRACE, "event");

  // clang-format off
  BEGIN_TRANSITION_MAP                          // - Current State -
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)      // ST_IDLE
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)      // ST_ENABLED
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)      // ST_POS_CONTROL
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)      // ST_TORQUE_CONTROL
      TRANSITION_MAP_ENTRY (ST_POS_CONTROL)     // ST_LOGGING
  END_TRANSITION_MAP(nullptr)
  // clang-format on
}

void ManualControlApp::disable()
{
  CLOG(TRACE, "event");

  qmutex_.lock();
  disable_cmd_recv_ = true;
  qmutex_.unlock();
  if (robot_ptr_->isWaiting())
    emit stopWaitingCmd();

  // clang-format off
  BEGIN_TRANSITION_MAP                      // - Current State -
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)  // ST_IDLE
      TRANSITION_MAP_ENTRY (ST_IDLE)        // ST_ENABLED
      TRANSITION_MAP_ENTRY (ST_IDLE)        // ST_POS_CONTROL
      TRANSITION_MAP_ENTRY (ST_IDLE)        // ST_TORQUE_CONTROL
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)  // ST_LOGGING
  END_TRANSITION_MAP(nullptr)
  // clang-format on
}

//--------- States actions -----------------------------------------------------------//

STATE_DEFINE(ManualControlApp, Idle, NoEventData)
{
  PrintStateTransition(prev_state_, ST_IDLE);
  prev_state_ = ST_IDLE;
  emit stateChanged(ST_IDLE);

  robot_ptr_->SetController(nullptr);
  if (robot_ptr_->AnyMotorEnabled())
    robot_ptr_->DisableMotors();

  qmutex_.lock();
  disable_cmd_recv_ = false; // reset
  qmutex_.unlock();
}

GUARD_DEFINE(ManualControlApp, GuardEnabled, NoEventData)
{
  robot_ptr_->EnableMotors();

  grabrt::ThreadClock clock(grabrt::Sec2NanoSec(CableRobot::kCycleWaitTimeSec));
  while (1)
  {
    if (robot_ptr_->MotorsEnabled())
      return true;
    if (clock.ElapsedFromStart() > CableRobot::kMaxWaitTimeSec)
      break;
    clock.WaitUntilNext();
  }
  emit printToQConsole("WARNING: State transition FAILED. Taking too long to "
                       "enable drives.");
  return false;
}

STATE_DEFINE(ManualControlApp, Enabled, NoEventData)
{
  PrintStateTransition(prev_state_, ST_ENABLED);
  prev_state_ = ST_ENABLED;

  qmutex_.lock();
  if (disable_cmd_recv_)
    InternalEvent(ST_IDLE);
  qmutex_.unlock();

  robot_ptr_->SetController(&controller_single_drive_);
  emit stateChanged(ST_ENABLED);
}

STATE_DEFINE(ManualControlApp, PosControl, NoEventData)
{
  PrintStateTransition(prev_state_, ST_POS_CONTROL);
  prev_state_ = ST_POS_CONTROL;

  RetVal ret = RetVal::OK;
  for (const id_t id : active_actuators_id_)
  {
    int motor_pos = robot_ptr_->GetActuatorStatus(id).motor_position;
    pthread_mutex_lock(&robot_ptr_->Mutex());
    controller_single_drive_.SetMotorID(id);
    controller_single_drive_.SetMode(ControlMode::MOTOR_POSITION);
    controller_single_drive_.SetMotorPosTarget(motor_pos, false);
    pthread_mutex_unlock(&robot_ptr_->Mutex());
    // Wait until each motor reached user-given initial torque setpoint
    ret = robot_ptr_->WaitUntilTargetReached();
    if (ret != RetVal::OK)
    {
      emit printToQConsole(
        QString("WARNING: Could not switch motor %1 to position control mode").arg(id));
      break;
    }
  }

  emit stateChanged(ST_POS_CONTROL);
}

STATE_DEFINE(ManualControlApp, TorqueControl, MyData)
{
  PrintStateTransition(prev_state_, ST_TORQUE_CONTROL);
  prev_state_ = ST_TORQUE_CONTROL;

  RetVal ret = RetVal::OK;
  for (const id_t id : active_actuators_id_)
  {
    pthread_mutex_lock(&robot_ptr_->Mutex());
    controller_single_drive_.SetMotorID(id);
    controller_single_drive_.SetMode(ControlMode::MOTOR_TORQUE);
    controller_single_drive_.SetMotorTorqueTarget(data->torque);
    pthread_mutex_unlock(&robot_ptr_->Mutex());
    // Wait until each motor reached user-given initial torque setpoint
    ret = robot_ptr_->WaitUntilTargetReached();
    if (ret != RetVal::OK)
    {
      emit printToQConsole(
        QString("WARNING: Could not switch motor %1 to torque control mode").arg(id));
      break;
    }
  }

  emit stateChanged(ST_TORQUE_CONTROL);
}

STATE_DEFINE(ManualControlApp, Logging, NoEventData)
{
  PrintStateTransition(prev_state_, ST_LOGGING);
  prev_state_ = ST_LOGGING;
  emit stateChanged(ST_LOGGING);

  traj_cables_len_.clear();
  if (!readTrajectories(kExcitationTrajFilepath_))
  {
    printToQConsole("WARNING: Could not read trjectories file");
    InternalEvent(ST_POS_CONTROL);
    return;
  }

  controller_joints_ptv_->SetCablesLenTrajectories(traj_cables_len_);
  robot_ptr_->StartRtLogging(kRtCycleMultiplier_);
  robot_ptr_->SetController(controller_joints_ptv_);
  emit printToQConsole("Start logging...");
}

EXIT_DEFINE(ManualControlApp, ExitLogging)
{
  robot_ptr_->StopRtLogging();
  robot_ptr_->SetController(&controller_single_drive_);
  emit printToQConsole("Logging stopped");
}

//--------- Private functions --------------------------------------------------------//

bool ManualControlApp::readTrajectories(const QString& ifilepath)
{
  CLOG(TRACE, "event") << "from '" << ifilepath << "'";
  QFile ifile(ifilepath);
  if (!ifile.open(QIODevice::ReadOnly | QIODevice::Text))
  {
    CLOG(ERROR, "event") << "Could not open trajectory file";
    return false;
  }

  // Read header yielding information about trajectory type and involved motors
  QTextStream s(&ifile);
  QStringList header = s.readLine().split(" ");
  bool relative      = static_cast<bool>(header[1].toUShort());
  vect<id_t> motors_id;
  for (int i = 2; i < header.size(); i++)
    motors_id.push_back(header[i].toUInt());

  // Fill trajectories accordingly reading text body line-by-line
  setCablesLenTraj(relative, motors_id, s);
  CLOG(INFO, "event") << "Trajectory parsed";
  return true;
}

void ManualControlApp::setCablesLenTraj(const bool relative, const vect<id_t>& motors_id,
                                        QTextStream& s)
{
  CLOG(INFO, "event") << QString("File contains %1 cables length trajectories")
                           .arg(relative ? "relative" : "absolute");
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
      traj_cables_len_[static_cast<size_t>(i) - 1].values.push_back(line[i].toDouble());
      if (relative)
        traj_cables_len_[static_cast<size_t>(i) - 1].values.back() +=
          current_cables_len[static_cast<size_t>(i) - 1];
    }
  }
}

void ManualControlApp::PrintStateTransition(const States current_state,
                                            const States new_state) const
{
  if (current_state == new_state)
    return;
  QString msg;
  if (current_state != ST_MAX_STATES)
    msg = QString("Manual control app state transition: %1 --> %2")
            .arg(kStatesStr[current_state], kStatesStr[new_state]);
  else
    msg = QString("Manual control app initial state: %1").arg(kStatesStr[new_state]);
  emit printToQConsole(msg);
}
