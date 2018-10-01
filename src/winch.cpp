#include "winch.h"

////////////////////////////////////////////////////////////////
/// Cable Class
////////////////////////////////////////////////////////////////

void Cable::UpdateCableLen(const double delta_length)
{
  length_ = home_length_ + delta_length;
}

////////////////////////////////////////////////////////////////
/// Winch Class
////////////////////////////////////////////////////////////////

Winch::Winch(const uint8_t slave_position)
  : StateMachine(ST_MAX_STATES), slave_position_(slave_position), servo_(slave_position)
{
}

void Winch::Enable()
{
  // clang-format off
  BEGIN_TRANSITION_MAP                                       // - Current State -
    TRANSITION_MAP_ENTRY(ST_ENABLED)              // ST_IDLE
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)       // ST_ENABLED
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)       // ST_FAULT
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

void Winch::Disable()
{
  // clang-format off
  BEGIN_TRANSITION_MAP                                       // - Current State -
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)       // ST_IDLE
    TRANSITION_MAP_ENTRY(ST_IDLE)                     // ST_ENABLED
    TRANSITION_MAP_ENTRY(ST_IDLE)                     // ST_FAULT
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

void Winch::FaultReset()
{
  // clang-format off
  BEGIN_TRANSITION_MAP                                       // - Current State -
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)       // ST_IDLE
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)       // ST_ENABLED
    TRANSITION_MAP_ENTRY(ST_IDLE)                     // ST_FAULT
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

void Winch::SetServoPosByCableLen(const double target_length)
{
  servo_.ChangePosition(servo_start_pos_ + params_.LengthToCounts(target_length));
}

void Winch::SetServoSpeed(const int32_t target_speed)
{
  servo_.ChangeVelocity(target_speed);
}

void Winch::SetServoTorque(const int16_t target_torque)
{
  servo_.ChangeTorque(target_torque);
}

void Winch::UpdateHomeConfig(const double cable_len, const double pulley_angle,
                             const double cable_len_true)
{
  pulley_.home_counts = servo_.GetAuxPosition();
  pulley_.home_angle = pulley_angle;

  cable_.SetHomeLength(cable_len);
  cable_.SetHomeLengthTrue(cable_len_true);

  servo_home_pos_ = servo_.GetPosition();
}

void Winch::UpdateStartConfig() {}

void Winch::UpdateConfig()
{
  cable_.UpdateCableLen(params_.CountsToLength(servo_.GetPosition()));
  pulley_.angle =
    pulley_.home_angle +
    params_.CountsToPulleyAngleRad(servo_.GetAuxPosition() - pulley_.home_counts);
}

// Guard condition to detemine whether Idle state is executed.
GUARD_DEFINE(Winch, GuardIdle, NoEventData)
{
  if (prev_state_ == ST_ENABLED)
    servo_.Shutdown();   // disable drive completely
  else                   // ST_FAULT
    servo_.FaultReset(); // clear fault and disable drive completely
  uint8_t counter = 0;
  while (1)
  {
    if (servo_.GetCurrentState() == grabec::ST_SWITCH_ON_DISABLED)
      return TRUE; // drive is disabled
    if (counter > kMaxTransitionCounter_)
      return FALSE; // taking too long to disable drive. Something's wrong.
    counter++;
  }
}

STATE_DEFINE(Winch, Idle, NoEventData)
{
  PrintStateTransition(ST_IDLE);
  prev_state_ = ST_IDLE;
}

// Guard condition to detemine whether Enable state is executed.
GUARD_DEFINE(Winch, GuardEnabled, NoEventData)
{
  servo_.Shutdown(); // prepare to switch on
  uint8_t counter = 0;
  while (1)
  {
    if (servo_.GetCurrentState() == grabec::ST_READY_TO_SWITCH_ON)
      break;
    if (counter > kMaxTransitionCounter_)
      return FALSE; // taking too long to disable drive. Something's wrong.
    counter++;
  }

  servo_.SwitchOn(); // switch on voltage
  counter = 0;
  while (1)
  {
    if (servo_.GetCurrentState() == grabec::ST_SWITCHED_ON)
      break;
    if (counter > kMaxTransitionCounter_)
      return FALSE; // taking too long to disable drive. Something's wrong.
    counter++;
  }

  servo_.EnableOperation(); // enable drive
  counter = 0;
  while (1)
  {
    if (servo_.GetCurrentState() == grabec::ST_OPERATION_ENABLED)
      return TRUE; // drive is enabled
    if (counter > kMaxTransitionCounter_)
      return FALSE; // taking too long to enable drive. Something's wrong.
    counter++;
  }
}

STATE_DEFINE(Winch, Enabled, NoEventData)
{
  PrintStateTransition(ST_ENABLED);
  prev_state_ = ST_ENABLED;
}

// Guard condition to detemine whether Idle state is executed.
GUARD_DEFINE(Winch, GuardFault, NoEventData)
{
  servo_.FaultReset(); // clear fault and disable drive completely
  uint8_t counter = 0;
  while (1)
  {
    if (servo_.GetCurrentState() == grabec::ST_SWITCH_ON_DISABLED)
      return TRUE; // drive is disabled
    if (counter > kMaxTransitionCounter_)
      return FALSE; // taking too long to disable drive. Something's wrong.
    counter++;
  }
}

STATE_DEFINE(Winch, Fault, NoEventData)
{
  PrintStateTransition(ST_FAULT);
  prev_state_ = ST_FAULT;
}

void Winch::PrintStateTransition(const States current_state) const
{
  if (current_state == prev_state_)
    return;
  printf("Winch %u state transition: %s --> %s\n", slave_position_,
         kStatesStr[prev_state_], kStatesStr[current_state]);
}
