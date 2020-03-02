/**
 * @file actuator.cpp
 * @author Simone Comari, Edoardo Idà
 * @date 02 Mar 2020
 * @brief This file includes definitions of class declared in actuator.h.
 */

#include "robot/components/actuator.h"

// Must provide redundant definition of the static member as well as the declaration.
constexpr char* Actuator::kStatesStr_[];

Actuator::Actuator(const id_t id, const uint8_t slave_position,
                   const grabcdpr::ActuatorParams& params, QObject* parent /* = NULL*/)
  : QObject(parent), StateMachine(ST_MAX_STATES), id_(id),
    slave_position_(slave_position), winch_(id, slave_position, params.winch),
    pulley_(id, params.pulley)
{
  active_ = params.active;
  clock_.SetCycleTime(kWaitCycleTimeNsec_);
  prev_state_ = ST_IDLE;

  winch_.getServo()->setParent(this);
  connect(winch_.getServo(), SIGNAL(driveFaulted()), this, SLOT(faultTrigger()));
  connect(winch_.getServo(), SIGNAL(logMessage(QString)), this,
          SLOT(logServoMsg(QString)));
  connect(winch_.getServo(), SIGNAL(printMessage(QString)), this,
          SLOT(forwardServoPrintMsg(QString)));
}

Actuator::~Actuator()
{
  disconnect(winch_.getServo(), SIGNAL(driveFaulted()), this, SLOT(faultTrigger()));
  disconnect(winch_.getServo(), SIGNAL(logMessage(QString)), this,
             SLOT(logServoMsg(QString)));
  disconnect(winch_.getServo(), SIGNAL(printMessage(QString)), this,
             SLOT(forwardServoPrintMsg(QString)));
}

//--------- External Events Public (slots) -------------------------------------------//

void Actuator::enable()
{
  CLOG(TRACE, "event");
  if (!active_)
    return;
  // clang-format off
  BEGIN_TRANSITION_MAP                              // - Current State -
    TRANSITION_MAP_ENTRY(ST_ENABLED)                // ST_IDLE
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)             // ST_ENABLED
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)             // ST_FAULT
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

void Actuator::disable()
{
  CLOG(TRACE, "event");
  if (!active_)
    return;
  // clang-format off
  BEGIN_TRANSITION_MAP                              // - Current State -
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)             // ST_IDLE
    TRANSITION_MAP_ENTRY(ST_IDLE)                   // ST_ENABLED
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)             // ST_FAULT
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

void Actuator::faultTrigger()
{
  CLOG(TRACE, "event");
  if (!active_)
    return;
  // clang-format off
  BEGIN_TRANSITION_MAP                              // - Current State -
    TRANSITION_MAP_ENTRY(ST_FAULT)                  // ST_IDLE
    TRANSITION_MAP_ENTRY(ST_FAULT)                  // ST_ENABLED
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)             // ST_FAULT
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

void Actuator::faultReset()
{
  CLOG(TRACE, "event");
  if (!active_)
    return;
  // clang-format off
  BEGIN_TRANSITION_MAP                              // - Current State -
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)             // ST_IDLE
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)             // ST_ENABLED
    TRANSITION_MAP_ENTRY(ST_IDLE)                   // ST_FAULT
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

//--------- Public Functions --------------------------------------------------//

const ActuatorStatus Actuator::getStatus()
{
  WinchStatus winch_status = winch_.getStatus();
  ActuatorStatus status;
  status.op_mode        = winch_status.op_mode;
  status.motor_position = winch_status.motor_position;
  status.motor_speed    = winch_status.motor_speed;
  status.motor_torque   = winch_status.motor_torque;
  status.cable_length   = winch_status.cable_length;
  status.aux_position   = winch_status.aux_position;
  status.id             = id_;
  status.state          = driveState2ActuatorState(
    static_cast<GSWDStates>(winch_.getServo()->GetCurrentState()));
  status.pulley_angle = pulley_.GetAngleRad(status.aux_position);
  return status;
}

void Actuator::setCableLength(const double target_length)
{
  if (active_)
    winch_.setServoPosByCableLen(target_length);
}

void Actuator::setMotorPos(const int32_t target_pos)
{
  if (active_)
    winch_.setServoPos(target_pos);
}

void Actuator::setMotorSpeed(const int32_t target_speed)
{
  if (active_)
    winch_.setServoSpeed(target_speed);
}

void Actuator::setMotorTorque(const int16_t target_torque)
{
  if (active_)
    winch_.setServoTorque(target_torque);
}

void Actuator::setMotorOpMode(const int8_t op_mode)
{
  if (active_)
    winch_.setServoOpMode(op_mode);
}

void Actuator::updateHomeConfig(const double cable_len, const double pulley_angle)
{
  if (!active_)
    return;
  pulley_.UpdateHomeConfig(winch_.getServo()->GetAuxPosition(), pulley_angle);
  winch_.updateHomeConfig(cable_len);
}

void Actuator::updateConfig()
{
  if (!active_)
    return;
  winch_.updateConfig();
  pulley_.UpdateConfig(winch_.getServo()->GetAuxPosition());
}

Actuator::States Actuator::driveState2ActuatorState(const GSWDStates drive_state)
{
  switch (drive_state)
  {
    case GSWDStates::ST_OPERATION_ENABLED:
      return Actuator::States::ST_ENABLED;
    case GSWDStates::ST_FAULT:
      return Actuator::States::ST_FAULT;
    default:
      return Actuator::States::ST_IDLE;
  }
}

//--------- States Actions Private --------------------------------------------------//

// Guard condition to detemine whether Idle state is executed.
GUARD_DEFINE(Actuator, GuardIdle, NoEventData)
{
  if (prev_state_ == ST_FAULT)
    winch_.getServo()->FaultReset(); // clear fault and disable drive completely
  else
    winch_.getServo()->DisableVoltage(); // disable drive completely

  clock_.Reset();
  timespec t0 = clock_.GetCurrentTime();
  while (1)
  {
    if (winch_.getServo()->GetCurrentState() == GSWDStates::ST_SWITCH_ON_DISABLED)
      return true; // drive is disabled
    if (clock_.Elapsed(t0) > kMaxTransitionTimeSec_)
    {
      emit printToQConsole(
        QString("WARNING: Actuator state transition FAILED. Taking too long to disable "
                "drive %1.")
          .arg(id_));
      return false;
    }
    clock_.WaitUntilNext();
  }
}

STATE_DEFINE(Actuator, Idle, NoEventData)
{
  printStateTransition(ST_IDLE);
  prev_state_ = ST_IDLE;
}

// Guard condition to detemine whether Enable state is executed.
GUARD_DEFINE(Actuator, GuardEnabled, NoEventData)
{
  winch_.getServo()->Shutdown(); // prepare to switch on
  clock_.Reset();
  timespec t0 = clock_.GetCurrentTime();
  while (1)
  {
    if (winch_.getServo()->GetCurrentState() == GSWDStates::ST_READY_TO_SWITCH_ON)
      break;
    if (clock_.Elapsed(t0) > kMaxTransitionTimeSec_)
    {
      emit printToQConsole(QString("WARNING: Actuator state transition FAILED. Taking "
                                   "too long to prepare to switch on drive %1.")
                             .arg(id_));
      return false;
    }
    clock_.WaitUntilNext();
  }

  winch_.getServo()->SwitchOn(); // switch on voltage
  clock_.Reset();
  t0 = clock_.GetCurrentTime();
  while (1)
  {
    if (winch_.getServo()->GetCurrentState() == GSWDStates::ST_SWITCHED_ON)
      break;
    if (clock_.Elapsed(t0) > kMaxTransitionTimeSec_)
    {
      emit printToQConsole(QString("WARNING: Actuator state transition FAILED. Taking "
                                   "too long to switch on voltage of drive %1.")
                             .arg(id_));
      return false;
    }
    clock_.WaitUntilNext();
  }

  winch_.getServo()->EnableOperation(); // enable drive
  clock_.Reset();
  t0 = clock_.GetCurrentTime();
  while (1)
  {
    if (winch_.getServo()->GetCurrentState() == GSWDStates::ST_OPERATION_ENABLED)
      return true; // drive is enabled
    if (clock_.Elapsed(t0) > kMaxTransitionTimeSec_)
    {
      emit printToQConsole(
        QString("WARNING: Actuator state transition FAILED. Taking too long to enable "
                "drive %1.")
          .arg(id_));
      return false;
    }
    clock_.WaitUntilNext();
  }
}

STATE_DEFINE(Actuator, Enabled, NoEventData)
{
  printStateTransition(ST_ENABLED);
  prev_state_ = ST_ENABLED;
}

// Guard condition to detemine whether Fault state is executed.
GUARD_DEFINE(Actuator, GuardFault, NoEventData)
{
  winch_.getServo()->FaultReset(); // clear fault and disable drive completely
  clock_.Reset();
  timespec t0 = clock_.GetCurrentTime();
  while (1)
  {
    // Try to clear faults automatically
    if (winch_.getServo()->GetCurrentState() == GSWDStates::ST_SWITCH_ON_DISABLED)
    {
      InternalEvent(ST_IDLE);
      return false;
    }
    if (clock_.Elapsed(t0) > kMaxTransitionTimeSec_)
    {
      emit printToQConsole(
        QString("WARNING: Attempt to automatically reset fault FAILED on drive %1.")
          .arg(id_));
      return true; // taking too long to disable drive. Something's wrong.
    }
    clock_.WaitUntilNext();
  }
}

STATE_DEFINE(Actuator, Fault, NoEventData)
{
  printStateTransition(ST_FAULT);
  prev_state_ = ST_FAULT;
}

//--------- Miscellaneous private --------------------------------------------------//

void Actuator::printStateTransition(const States current_state) const
{
  if (current_state == prev_state_)
    return;
  QString msg;
  msg = QString("Actuator %1 state transition: %2 --> %3")
          .arg(id_)
          .arg(kStatesStr_[prev_state_], kStatesStr_[current_state]);
  emit printToQConsole(msg);
  emit stateChanged(id_, current_state);
}
