#include "homing/homing_proprioceptive.h"

constexpr char* HomingProprioceptive::kStatesStr[];

HomingProprioceptive::HomingProprioceptive(QObject* parent,
                                           const grabcdpr::Params* config)
  : QObject(parent), StateMachine(ST_MAX_STATES), config_ptr_(config)
{
}

HomingProprioceptive::~HomingProprioceptive() {}

////////////////////////////////////////////////////////////////////////////
//// External Events
////////////////////////////////////////////////////////////////////////////

void HomingProprioceptive::Start()
{
  // clang-format off
  BEGIN_TRANSITION_MAP			              			// - Current State -
      TRANSITION_MAP_ENTRY (ST_START_UP)                         // ST_IDLE
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)                    // ST_START_UP
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)			// ST_COILING
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)			// ST_UNCOILING
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)			// ST_SWITCH_CABLE
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_OPTIMIZING
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_GO_HOME
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_FAULT
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

void HomingProprioceptive::Stop()
{
  // clang-format off
  BEGIN_TRANSITION_MAP			              			// - Current State -
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)                    // ST_IDLE
      TRANSITION_MAP_ENTRY (ST_IDLE)                                  // ST_START_UP
      TRANSITION_MAP_ENTRY (ST_IDLE)                                  // ST_COILING
      TRANSITION_MAP_ENTRY (ST_IDLE)                                  // ST_UNCOILING
      TRANSITION_MAP_ENTRY (ST_IDLE)                                  // ST_SWITCH_CABLE
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_OPTIMIZING
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)                   // ST_GO_HOME
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_FAULT
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

void HomingProprioceptive::Next()
{
  // clang-format off
  BEGIN_TRANSITION_MAP			              			// - Current State -
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)                   // ST_IDLE
      TRANSITION_MAP_ENTRY (ST_COILING)                            // ST_START_UP
      TRANSITION_MAP_ENTRY (ST_UNCOILING)			// ST_COILING
      TRANSITION_MAP_ENTRY (ST_SWITCH_CABLE)			// ST_UNCOILING
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)                    // ST_SWITCH_CABLE
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_OPTIMIZING
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_GO_HOME
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_FAULT
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

void HomingProprioceptive::Optimize()
{
  // clang-format off
  BEGIN_TRANSITION_MAP			              			// - Current State -
      TRANSITION_MAP_ENTRY (ST_OPTIMIZING)                      // ST_IDLE
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)                    // ST_START_UP
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)			// ST_COILING
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)			// ST_UNCOILING
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)                    // ST_SWITCH_CABLE
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)			// ST_OPTIMIZING
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)			// ST_GO_HOME
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_FAULT
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

void HomingProprioceptive::FaultTrigger()
{
  // clang-format off
  BEGIN_TRANSITION_MAP			              			// - Current State -
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)                   // ST_IDLE
      TRANSITION_MAP_ENTRY (ST_FAULT)                               // ST_START_UP
      TRANSITION_MAP_ENTRY (ST_FAULT)                               // ST_COILING
      TRANSITION_MAP_ENTRY (ST_FAULT)                               // ST_UNCOILING
      TRANSITION_MAP_ENTRY (ST_FAULT)                               // ST_SWITCH_CABLE
      TRANSITION_MAP_ENTRY (ST_FAULT)                               // ST_OPTIMIZING
      TRANSITION_MAP_ENTRY (ST_FAULT)                               // ST_GO_HOME
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)			// ST_FAULT
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

void HomingProprioceptive::FaultReset()
{
  // clang-format off
  BEGIN_TRANSITION_MAP			              			// - Current State -
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)                   // ST_IDLE
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)                   // ST_START_UP
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_COILING
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_UNCOILING
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)                   // ST_SWITCH_CABLE
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_OPTIMIZING
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_GO_HOME
      TRANSITION_MAP_ENTRY (ST_IDLE)                                  // ST_FAULT
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

bool HomingProprioceptive::IsCollectingData()
{
  States current_state = static_cast<States>(GetCurrentState());
  switch (current_state)
  {
  case ST_IDLE:
    return false;
  case ST_FAULT:
    return false;
  case ST_OPTIMIZING:
    return false;
  case ST_GO_HOME:
    return false;
  default:
    return true;
  }
}

////////////////////////////////////////////////////////////////////////////
//// States actions
////////////////////////////////////////////////////////////////////////////

STATE_DEFINE(HomingProprioceptive, Idle, NoEventData)
{
  PrintStateTransition(prev_state_, ST_IDLE);
  prev_state_ = ST_IDLE;
}

STATE_DEFINE(HomingProprioceptive, StartUp, NoEventData)
{
  PrintStateTransition(prev_state_, ST_START_UP);
  prev_state_ = ST_START_UP;
}

GUARD_DEFINE(HomingProprioceptive, GuardCoiling, NoEventData) { return true; }

STATE_DEFINE(HomingProprioceptive, Coiling, NoEventData)
{
  PrintStateTransition(prev_state_, ST_COILING);
  prev_state_ = ST_COILING;
}

EXIT_DEFINE(HomingProprioceptive, CollectMeas) {}

GUARD_DEFINE(HomingProprioceptive, GuardUncoiling, NoEventData) { return true; }

STATE_DEFINE(HomingProprioceptive, Uncoiling, NoEventData)
{
  PrintStateTransition(prev_state_, ST_UNCOILING);
  prev_state_ = ST_UNCOILING;
}

GUARD_DEFINE(HomingProprioceptive, GuardSwitch, NoEventData) { return true; }

STATE_DEFINE(HomingProprioceptive, SwitchCable, NoEventData)
{
  PrintStateTransition(prev_state_, ST_SWITCH_CABLE);
  prev_state_ = ST_SWITCH_CABLE;
}

STATE_DEFINE(HomingProprioceptive, Optimizing, NoEventData)
{
  PrintStateTransition(prev_state_, ST_OPTIMIZING);
  prev_state_ = ST_OPTIMIZING;
}

STATE_DEFINE(HomingProprioceptive, GoHome, NoEventData)
{
  PrintStateTransition(prev_state_, ST_GO_HOME);
  prev_state_ = ST_GO_HOME;
}

STATE_DEFINE(HomingProprioceptive, Fault, NoEventData)
{
  PrintStateTransition(prev_state_, ST_FAULT);
  prev_state_ = ST_FAULT;
}

////////////////////////////////////////////////////////////////////////////
//// Miscellaneous
////////////////////////////////////////////////////////////////////////////

void HomingProprioceptive::PrintStateTransition(const States current_state,
                                                const States new_state) const
{
  if (current_state == new_state)
    return;
  QString msg;
  if (current_state != ST_MAX_STATES)
    msg = QString("State transition: %1 --> %2")
            .arg(kStatesStr[current_state], kStatesStr[new_state]);
  else
    msg = QString("Intial state: %1").arg(kStatesStr[new_state]);
  printf("%s\n", msg.toStdString().c_str());
  emit printToQConsole(msg);
}
