#include "robot/cablerobot.h"

constexpr char* CableRobot::kStatesStr[];

CableRobot::CableRobot(QObject* parent, const grabcdpr::Params& /*config*/)
  : QObject(parent), StateMachine(ST_MAX_STATES), prev_state_(ST_MAX_STATES)
{
  PrintStateTransition(prev_state_, ST_IDLE);
  prev_state_ = ST_IDLE;
}

void CableRobot::EnterCalibrationMode()
{
  // clang-format off
  BEGIN_TRANSITION_MAP			              			// - Current State -
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_IDLE
      TRANSITION_MAP_ENTRY (ST_CALIBRATION)                    // ST_ENABLED
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)			// ST_CALIBRATION
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_HOMING
      TRANSITION_MAP_ENTRY (ST_CALIBRATION)                    // ST_READY
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_OPERATIONAL
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_ERROR
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

void CableRobot::EnterHomingMode()
{
  // clang-format off
  BEGIN_TRANSITION_MAP			              			// - Current State -
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_IDLE
      TRANSITION_MAP_ENTRY (ST_HOMING)                            // ST_ENABLED
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_CALIBRATION
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)			// ST_HOMING
      TRANSITION_MAP_ENTRY (ST_HOMING)                            // ST_READY
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_OPERATIONAL
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_ERROR
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

////////////////////////////////////////////////////////////////////////////
//// External events
////////////////////////////////////////////////////////////////////////////

void CableRobot::EventSuccess()
{
  // clang-format off
  BEGIN_TRANSITION_MAP			              			// - Current State -
      TRANSITION_MAP_ENTRY (ST_ENABLED)				// ST_IDLE
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)                   // ST_ENABLED
      TRANSITION_MAP_ENTRY (ST_ENABLED)				// ST_CALIBRATION
      TRANSITION_MAP_ENTRY (ST_READY)				// ST_HOMING
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)                   // ST_READY
      TRANSITION_MAP_ENTRY (ST_READY)				// ST_OPERATIONAL
      TRANSITION_MAP_ENTRY (ST_ENABLED)				// ST_ERROR
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

void CableRobot::EventFailure()
{
  // clang-format off
  BEGIN_TRANSITION_MAP			              			// - Current State -
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_IDLE
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)                   // ST_ENABLED
      TRANSITION_MAP_ENTRY (ST_ENABLED)				// ST_CALIBRATION
      TRANSITION_MAP_ENTRY (ST_ENABLED)				// ST_HOMING
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)                   // ST_READY
      TRANSITION_MAP_ENTRY (ST_ERROR)				// ST_OPERATIONAL
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)			// ST_ERROR
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

void CableRobot::Stop()
{
  // clang-format off
  BEGIN_TRANSITION_MAP			              			// - Current State -
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_IDLE
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)                    // ST_ENABLED
      TRANSITION_MAP_ENTRY (ST_ENABLED)				// ST_CALIBRATION
      TRANSITION_MAP_ENTRY (ST_ENABLED)				// ST_HOMING
      TRANSITION_MAP_ENTRY (ST_ENABLED)                           // ST_READY
      TRANSITION_MAP_ENTRY (ST_READY)				// ST_OPERATIONAL
      TRANSITION_MAP_ENTRY (ST_ENABLED)   			// ST_ERROR
  END_TRANSITION_MAP(NULL)
  // clang-format on
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
  QString msg;
  if (current_state != ST_MAX_STATES)
    msg = QString("CableRobot state transition: %1 --> %2")
            .arg(kStatesStr[current_state], kStatesStr[new_state]);
  else
    msg = QString("CableRobot intial state: %1").arg(kStatesStr[new_state]);
  printf("%s\n", msg.toStdString().c_str());
  emit printToQConsole(msg);
}
