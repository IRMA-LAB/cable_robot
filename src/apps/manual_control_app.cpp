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

ManualControlApp::ManualControlApp(QObject* parent, CableRobot* robot)
  : QObject(parent), StateMachine(ST_MAX_STATES), robot_ptr_(robot),
    controller_(robot->GetRtCycleTimeNsec())
{
  prev_state_ = ST_MAX_STATES;
  ExternalEvent(ST_IDLE);

  controller_.SetMotorTorqueSsErrTol(kTorqueSsErrTol_);
  active_actuators_id_ = robot_ptr_->GetActiveMotorsID();
  connect(this, SIGNAL(stopWaitingCmd()), robot_ptr_, SLOT(stopWaiting()));
}

ManualControlApp::~ManualControlApp()
{
  disconnect(this, SIGNAL(stopWaitingCmd()), robot_ptr_, SLOT(stopWaiting()));
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
  END_TRANSITION_MAP(data)
  // clang-format on
}

void ManualControlApp::disable()
{
  CLOG(TRACE, "event");

  qmutex_.lock();
  disable_cmd_recv_ = true;
  qmutex_.unlock();
  emit stopWaitingCmd();

  // clang-format off
  BEGIN_TRANSITION_MAP                      // - Current State -
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)  // ST_IDLE
      TRANSITION_MAP_ENTRY (ST_IDLE)        // ST_ENABLED
      TRANSITION_MAP_ENTRY (ST_IDLE)        // ST_POS_CONTROL
      TRANSITION_MAP_ENTRY (ST_IDLE)        // ST_TORQUE_CONTROL
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

  robot_ptr_->SetController(&controller_);
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
    controller_.SetMotorID(id);
    controller_.SetMode(ControlMode::MOTOR_POSITION);
    controller_.SetMotorPosTarget(motor_pos, false);
    pthread_mutex_unlock(&robot_ptr_->Mutex());
    // Wait until each motor reached user-given initial torque setpoint
    ret = robot_ptr_->WaitUntilTargetReached();
    if (ret != RetVal::OK)
      break;
  }

  emit printToQConsole(QString("Robot is now in position control mode"));
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
    controller_.SetMotorID(id);
    controller_.SetMode(ControlMode::MOTOR_TORQUE);
    controller_.SetMotorTorqueTarget(data->torque);
    pthread_mutex_unlock(&robot_ptr_->Mutex());
    // Wait until each motor reached user-given initial torque setpoint
    ret = robot_ptr_->WaitUntilTargetReached();
    if (ret != RetVal::OK)
      break;
  }

  emit printToQConsole(QString("Robot is now in torque control mode"));
  emit stateChanged(ST_TORQUE_CONTROL);
}

//--------- Private functions --------------------------------------------------------//

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
