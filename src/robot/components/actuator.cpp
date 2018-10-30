#include "robot/components/actuator.h"

////////////////////////////////////////////////////////////////
/// Actuator Class
////////////////////////////////////////////////////////////////

// Must provide redundant definition of the static member as well as the declaration.
constexpr char* Actuator::kStatesStr_[];

Actuator::Actuator(const uint8_t slave_position, const grabcdpr::CableParams &params)
  : StateMachine(ST_MAX_STATES), slave_position_(slave_position)
{
  // todo: distribute params to components
  WinchParams wp;
  winch_ = new Winch(slave_position, wp);
  PulleyParams pp;
  pulley_ = new PulleysSystem(pp);
}

void Actuator::Enable()
{
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
  // clang-format off
  BEGIN_TRANSITION_MAP                                       // - Current State -
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)       // ST_IDLE
    TRANSITION_MAP_ENTRY(ST_IDLE)                     // ST_ENABLED
    TRANSITION_MAP_ENTRY(ST_IDLE)                     // ST_FAULT
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

void Actuator::FaultReset()
{
  // clang-format off
  BEGIN_TRANSITION_MAP                                       // - Current State -
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)       // ST_IDLE
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)       // ST_ENABLED
    TRANSITION_MAP_ENTRY(ST_IDLE)                     // ST_FAULT
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

void Actuator::SetCableLength(const double target_length)
{
  winch_->SetServoPosByCableLen(target_length);
}

void Actuator::SetCableSpeed(const int32_t target_speed)
{
  winch_->SetServoSpeed(target_speed);
}

void Actuator::SetCableTorque(const int16_t target_torque)
{
  winch_->SetServoTorque(target_torque);
}

void Actuator::UpdateHomeConfig(const double cable_len, const double cable_len_true,
                                const double pulley_angle)
{
  pulley_->UpdateHomeConfig(winch_->GetServo()->GetAuxPosition(), pulley_angle);
  winch_->UpdateHomeConfig(cable_len, cable_len_true);
}

void Actuator::UpdateStartConfig() {}

void Actuator::UpdateConfig()
{
  winch_->UpdateConfig();
  pulley_->UpdateConfig(winch_->GetServo()->GetAuxPosition());
}

// Guard condition to detemine whether Idle state is executed.
GUARD_DEFINE(Actuator, GuardIdle, NoEventData)
{
  if (prev_state_ == ST_ENABLED)
    winch_->GetServo()->Shutdown();   // disable drive completely
  else                               // ST_FAULT
    winch_->GetServo()->FaultReset(); // clear fault and disable drive completely
  clock_.Reset();
  while (1)
  {
    if (winch_->GetServo()->GetCurrentState() == grabec::ST_SWITCH_ON_DISABLED)
      return TRUE; // drive is disabled
    if (clock_.Elapsed() > kMaxTransitionTimeSec_)
      return FALSE; // taking too long to disable drive. Something's wrong.
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
  winch_->GetServo()->Shutdown(); // prepare to switch on
  clock_.Reset();
  while (1)
  {
    if (winch_->GetServo()->GetCurrentState() == grabec::ST_READY_TO_SWITCH_ON)
      break;
    if (clock_.Elapsed() > kMaxTransitionTimeSec_)
      return FALSE; // taking too long to disable drive. Something's wrong.
    clock_.Reset();
  }

  winch_->GetServo()->SwitchOn(); // switch on voltage
  clock_.Reset();
  while (1)
  {
    if (winch_->GetServo()->GetCurrentState() == grabec::ST_SWITCHED_ON)
      break;
    if (clock_.Elapsed() > kMaxTransitionTimeSec_)
      return FALSE; // taking too long to disable drive. Something's wrong.
    clock_.Reset();
  }

  winch_->GetServo()->EnableOperation(); // enable drive
  clock_.Reset();
  while (1)
  {
    if (winch_->GetServo()->GetCurrentState() == grabec::ST_OPERATION_ENABLED)
      return TRUE; // drive is enabled
    if (clock_.Elapsed() > kMaxTransitionTimeSec_)
      return FALSE; // taking too long to enable drive. Something's wrong.
    clock_.Reset();
  }
}

STATE_DEFINE(Actuator, Enabled, NoEventData)
{
  PrintStateTransition(ST_ENABLED);
  prev_state_ = ST_ENABLED;
}

// Guard condition to detemine whether Idle state is executed.
GUARD_DEFINE(Actuator, GuardFault, NoEventData)
{
  winch_->GetServo()->FaultReset(); // clear fault and disable drive completely
  clock_.Reset();
  while (1)
  {
    if (winch_->GetServo()->GetCurrentState() == grabec::ST_SWITCH_ON_DISABLED)
      return TRUE; // drive is disabled
    if (clock_.Elapsed() > kMaxTransitionTimeSec_)
      return FALSE; // taking too long to disable drive. Something's wrong.
    clock_.Reset();
  }
}

STATE_DEFINE(Actuator, Fault, NoEventData)
{
  PrintStateTransition(ST_FAULT);
  prev_state_ = ST_FAULT;
}

void Actuator::PrintStateTransition(const States current_state) const
{
  if (current_state == prev_state_)
    return;
  printf("Actuator %u state transition: %s --> %s\n", slave_position_,
         kStatesStr_[prev_state_], kStatesStr_[current_state]);
}
