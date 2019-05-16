#include "apps/manual_control_app.h"

// For static constexpr passed by reference we need a dummy definition no matter what
constexpr char* ManualControlApp::kStatesStr[];

ManualControlApp::ManualControlApp(QObject* parent, CableRobot* robot)
  : QObject(parent), StateMachine(ST_MAX_STATES), robot_ptr_(robot)
{
  prev_state_ = ST_MAX_STATES;
  ExternalEvent(ST_IDLE);
}

void ManualControlApp::next() {
  // clang-format off
  BEGIN_TRANSITION_MAP                    // - Current State -
      TRANSITION_MAP_ENTRY (ST_READY)     // ST_IDLE
      TRANSITION_MAP_ENTRY (ST_IDLE)      // ST_READY
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

STATE_DEFINE(ManualControlApp, Idle, NoEventData)
{
  PrintStateTransition(prev_state_, ST_IDLE);
  prev_state_ = ST_IDLE;
}

STATE_DEFINE(ManualControlApp, Ready, NoEventData)
{
  PrintStateTransition(prev_state_, ST_READY);
  prev_state_ = ST_READY;
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
