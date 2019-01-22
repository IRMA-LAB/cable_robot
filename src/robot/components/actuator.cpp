#include "robot/components/actuator.h"

// Must provide redundant definition of the static member as well as the declaration.
constexpr char* Actuator::kStatesStr_[];

Actuator::Actuator(const ID_t id, const uint8_t slave_position,
                   const grabcdpr::ActuatorParams& params, QObject* parent /* = NULL*/)
  : QObject(parent), StateMachine(ST_MAX_STATES), id_(id),
    slave_position_(slave_position), winch_(id, slave_position, params.winch),
    pulley_(id, params.pulley)
{
  active_ = params.active;

  winch_.GetServo()->setParent(this);
  connect(winch_.GetServo(), SIGNAL(driveFaulted()), this, SLOT(FaultTrigger()));
}

Actuator::~Actuator()
{
  disconnect(winch_.GetServo(), SIGNAL(driveFaulted()), this, SLOT(FaultTrigger()));
}

//--------- External Events Public --------------------------------------------------//

void Actuator::Enable()
{
  CLOG(TRACE, "event");
  if (!active_)
    return;
  // clang-format off
  BEGIN_TRANSITION_MAP                                       // - Current State -
    TRANSITION_MAP_ENTRY(ST_ENABLED)              // ST_IDLE
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)       // ST_ENABLED
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)       // ST_FAULT
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

void Actuator::Disable()
{
  CLOG(TRACE, "event");
  if (!active_)
    return;
  if (GetCurrentState() == ST_ENABLED)
  {
    winch_.GetServo()->DisableVoltage(); // disable drive completely
    ExternalEvent(ST_IDLE);
  }
}

void Actuator::FaultTrigger()
{
  CLOG(TRACE, "event");
  if (!active_)
    return;
  // clang-format off
  BEGIN_TRANSITION_MAP                                       // - Current State -
    TRANSITION_MAP_ENTRY(ST_FAULT)                  // ST_IDLE
    TRANSITION_MAP_ENTRY(ST_FAULT)                  // ST_ENABLED
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)       // ST_FAULT
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

void Actuator::FaultReset()
{
  CLOG(TRACE, "event");
  if (!active_)
    return;
  if (GetCurrentState() == ST_FAULT)
  {
    winch_.GetServo()->FaultReset(); // clear fault and disable drive completely
    ExternalEvent(ST_IDLE);
  }
}

//--------- Public Functions --------------------------------------------------//

const ActuatorStatus Actuator::GetStatus()
{
  WinchStatus winch_status = winch_.GetStatus();
  ActuatorStatus status;
  status.op_mode = winch_status.op_mode;
  status.motor_position = winch_status.motor_position;
  status.motor_speed = winch_status.motor_speed;
  status.motor_torque = winch_status.motor_torque;
  status.cable_length = winch_status.cable_length;
  status.aux_position = winch_status.aux_position;
  status.id = id_;
  status.pulley_angle = pulley_.GetAngleRad(status.aux_position);
  return status;
}

void Actuator::SetCableLength(const double target_length)
{
  if (active_)
    winch_.SetServoPosByCableLen(target_length);
}

void Actuator::SetMotorPos(const int32_t target_pos)
{
  if (active_)
    winch_.SetServoPos(target_pos);
}

void Actuator::SetMotorSpeed(const int32_t target_speed)
{
  if (active_)
    winch_.SetServoSpeed(target_speed);
}

void Actuator::SetMotorTorque(const int16_t target_torque)
{
  if (active_)
    winch_.SetServoTorque(target_torque);
}

void Actuator::SetMotorOpMode(const int8_t op_mode)
{
  if (active_)
    winch_.SetServoOpMode(op_mode);
}

void Actuator::UpdateHomeConfig(const double cable_len, const double pulley_angle)
{
  if (!active_)
    return;
  pulley_.UpdateHomeConfig(winch_.GetServo()->GetAuxPosition(), pulley_angle);
  winch_.UpdateHomeConfig(cable_len);
}

void Actuator::UpdateConfig()
{
  if (!active_)
    return;
  winch_.UpdateConfig();
  pulley_.UpdateConfig(winch_.GetServo()->GetAuxPosition());
}

//--------- States Actions Private --------------------------------------------------//

// Guard condition to detemine whether Idle state is executed.
GUARD_DEFINE(Actuator, GuardIdle, NoEventData)
{
  clock_.Reset();
  while (1)
  {
    if (winch_.GetServo()->GetCurrentState() == grabec::ST_SWITCH_ON_DISABLED)
      return TRUE; // drive is disabled
    if (clock_.Elapsed() > kMaxTransitionTimeSec_)
    {
      emit printToQConsole(
        QString("[WARNING] Actuator state transition FAILED. Taking too long to disable "
                "drive %1.").arg(id_));
      return FALSE;
    }
    clock_.Reset();
  }
}

STATE_DEFINE(Actuator, Idle, NoEventData)
{
  PrintStateTransition(ST_IDLE);
  prev_state_ = ST_IDLE;
}

// Guard condition to detemine whether Enable state is executed.
GUARD_DEFINE(Actuator, GuardEnabled, NoEventData)
{
  winch_.GetServo()->Shutdown(); // prepare to switch on
  clock_.Reset();
  while (1)
  {
    if (winch_.GetServo()->GetCurrentState() == grabec::ST_READY_TO_SWITCH_ON)
      break;
    if (clock_.Elapsed() > kMaxTransitionTimeSec_)
    {
      emit printToQConsole(
        QString("[WARNING] Actuator state transition FAILED. Taking "
                "too long to prepare to switch on drive %1.").arg(id_));
      return FALSE;
    }
    usleep(100000);
  }

  winch_.GetServo()->SwitchOn(); // switch on voltage
  clock_.Reset();
  while (1)
  {
    if (winch_.GetServo()->GetCurrentState() == grabec::ST_SWITCHED_ON)
      break;
    if (clock_.Elapsed() > kMaxTransitionTimeSec_)
    {
      emit printToQConsole(
        QString("[WARNING] Actuator state transition FAILED. Taking "
                "too long to switch on voltage of drive %1.").arg(id_));
      return FALSE;
    }
    usleep(100000);
  }

  winch_.GetServo()->EnableOperation(); // enable drive
  clock_.Reset();
  while (1)
  {
    if (winch_.GetServo()->GetCurrentState() == grabec::ST_OPERATION_ENABLED)
      return TRUE; // drive is enabled
    if (clock_.Elapsed() > kMaxTransitionTimeSec_)
    {
      emit printToQConsole(
        QString("[WARNING] Actuator state transition FAILED. Taking too long to enable "
                "drive %1.").arg(id_));
      return FALSE;
    }
    usleep(100000);
  }
}

STATE_DEFINE(Actuator, Enabled, NoEventData)
{
  PrintStateTransition(ST_ENABLED);
  prev_state_ = ST_ENABLED;
}

// Guard condition to detemine whether Fault state is executed.
GUARD_DEFINE(Actuator, GuardFault, NoEventData)
{
  winch_.GetServo()->FaultReset(); // clear fault and disable drive completely
  clock_.Reset();
  while (1)
  {
    // Try to clear faults automatically
    if (winch_.GetServo()->GetCurrentState() == grabec::ST_SWITCH_ON_DISABLED)
    {
      InternalEvent(ST_IDLE);
      return FALSE;
    }
    if (clock_.Elapsed() > kMaxTransitionTimeSec_)
    {
      emit printToQConsole(
        QString("[WARNING] Attempt to automatically reset fault FAILED on drive %1.")
          .arg(id_));
      return TRUE; // taking too long to disable drive. Something's wrong.
    }
    usleep(100000);
  }
}

STATE_DEFINE(Actuator, Fault, NoEventData)
{
  PrintStateTransition(ST_FAULT);
  prev_state_ = ST_FAULT;
}

//--------- Miscellaneous private --------------------------------------------------//

void Actuator::PrintStateTransition(const States current_state) const
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
