#include "robot/cablerobot.h"

constexpr char* CableRobot::kStatesStr[];

CableRobot::CableRobot(QObject* parent) : QObject(parent), StateMachine(ST_MAX_STATES)
{
  InternalEvent(ST_IDLE);
  prev_state_ = ST_MAX_STATES;
}

////////////////////////////////////////////////////////////////////////////
//// States actions
////////////////////////////////////////////////////////////////////////////

STATE_DEFINE(CableRobot, Idle, NoEventData)
{
  PrintStateTransition(prev_state_, ST_IDLE);
  prev_state_ = ST_IDLE;
}

STATE_DEFINE(CableRobot, Enabled, NoEventData)
{
  PrintStateTransition(prev_state_, ST_ENABLED);
  prev_state_ = ST_ENABLED;
}

STATE_DEFINE(CableRobot, Calibration, NoEventData)
{
  PrintStateTransition(prev_state_, ST_CALIBRATION);
  prev_state_ = ST_CALIBRATION;
}

STATE_DEFINE(CableRobot, Homing, NoEventData)
{
  PrintStateTransition(prev_state_, ST_HOMING);
  prev_state_ = ST_HOMING;
}

STATE_DEFINE(CableRobot, Ready, NoEventData)
{
  PrintStateTransition(prev_state_, ST_READY);
  prev_state_ = ST_READY;
}

STATE_DEFINE(CableRobot, Operational, NoEventData)
{
  PrintStateTransition(prev_state_, ST_OPERATIONAL);
  prev_state_ = ST_OPERATIONAL;
}

STATE_DEFINE(CableRobot, Error, NoEventData)
{
  PrintStateTransition(prev_state_, ST_ERROR);
  prev_state_ = ST_ERROR;
}

////////////////////////////////////////////////////////////////////////////
//// Miscellaneous
////////////////////////////////////////////////////////////////////////////

void CableRobot::PrintStateTransition(const States current_state,
                                      const States new_state) const
{
  if (current_state == new_state)
    return;
  if (current_state != ST_MAX_STATES)
  {
    printf("CableRobot state transition: %s --> %s\n", kStatesStr[current_state],
           kStatesStr[new_state]);
    return;
  }
  printf("CableRobot intial state: %s\n", kStatesStr[new_state]);
}
