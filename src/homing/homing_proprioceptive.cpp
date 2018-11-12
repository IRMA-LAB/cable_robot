#include "homing/homing_proprioceptive.h"

////////////////////////////////////////////////////////////////////////////
//// Homing proprioceptive data class
////////////////////////////////////////////////////////////////////////////

HomingProprioceptiveStartData::HomingProprioceptiveStartData() {}

HomingProprioceptiveStartData::HomingProprioceptiveStartData(
  const vect<qint16>& _init_torques, const quint8 _num_meas)
  : init_torques(_init_torques), num_meas(_num_meas)
{
}

////////////////////////////////////////////////////////////////////////////
//// Homing proprioceptive class
////////////////////////////////////////////////////////////////////////////

constexpr char* HomingProprioceptive::kStatesStr[];

HomingProprioceptive::HomingProprioceptive(QObject* parent, CableRobot* robot)
  : QObject(parent), StateMachine(ST_MAX_STATES), robot_(robot)
{
  // init with default values
  num_meas_ = kNumMeasMin;
  prev_state_ = ST_MAX_STATES;
}

////////////////////////////////////////////////////////////////////////////
//// External Events
////////////////////////////////////////////////////////////////////////////

void HomingProprioceptive::Start(HomingProprioceptiveStartData* data)
{
  // clang-format off
  BEGIN_TRANSITION_MAP			              			// - Current State -
      TRANSITION_MAP_ENTRY (ST_ENABLED)                          // ST_IDLE
      TRANSITION_MAP_ENTRY (ST_START_UP)                         // ST_ENABLED
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)                    // ST_START_UP
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)			// ST_COILING
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)			// ST_UNCOILING
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)			// ST_SWITCH_CABLE
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_OPTIMIZING
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_GO_HOME
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_FAULT
  END_TRANSITION_MAP(data)
  // clang-format on
}

void HomingProprioceptive::Stop()
{
  // clang-format off
  BEGIN_TRANSITION_MAP			              			// - Current State -
      TRANSITION_MAP_ENTRY (ST_IDLE)                                  // ST_IDLE
      TRANSITION_MAP_ENTRY (ST_ENABLED)                          // ST_ENABLED
      TRANSITION_MAP_ENTRY (ST_ENABLED)                          // ST_START_UP
      TRANSITION_MAP_ENTRY (ST_ENABLED)                          // ST_COILING
      TRANSITION_MAP_ENTRY (ST_ENABLED)                          // ST_UNCOILING
      TRANSITION_MAP_ENTRY (ST_ENABLED)                          // ST_SWITCH_CABLE
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
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)                   // ST_ENABLED
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
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)                    // ST_ENABLED
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
      TRANSITION_MAP_ENTRY (ST_FAULT)                               // ST_ENABLED
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
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)                   // ST_ENABLED
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
  case ST_ENABLED:
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

  if (robot_->AnyMotorEnabled())
    robot_->DisableMotors();
}

GUARD_DEFINE(HomingProprioceptive, GuardEnabled, NoEventData)
{
  if (robot_->EnableMotors())
    return true;
  return false;
}

STATE_DEFINE(HomingProprioceptive, Enabled, NoEventData)
{
  PrintStateTransition(prev_state_, ST_ENABLED);
  prev_state_ = ST_ENABLED;

  robot_->SetMotorsOpMode(grabec::CYCLIC_TORQUE);
}

STATE_DEFINE(HomingProprioceptive, StartUp, HomingProprioceptiveStartData)
{
  PrintStateTransition(prev_state_, ST_START_UP);
  prev_state_ = ST_START_UP;

  QString msg("Start up phase complete\nRobot in predefined configuration\nInitial "
              "torque values:");

  vect<quint8> motors_id = robot_->GetMotorsID();
  for (quint8 i = 0; i < motors_id.size(); ++i)
  {
    // Use current torque values if not given
    if (!data->init_torques.empty())
    {
      controller_.SetMotorID(motors_id[i]);
      controller_.SetMotorTorqueTarget(data->init_torques[i]);
      while (controller_.GetMotorTorqueTarget() !=
             robot_->GetMotorStatus(motors_id[i]).torque_target)
        continue; // inserisci un tempo di attesa qui magari
    }
    msg.append(
      QString("\n\t%1 [Nm]").arg(robot_->GetMotorStatus(motors_id[i]).torque_target));
    num_meas_ = data->num_meas;
  }
  emit printToQConsole(msg);
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
    msg = QString("Homing state transition: %1 --> %2")
            .arg(kStatesStr[current_state], kStatesStr[new_state]);
  else
    msg = QString("Homing initial state: %1").arg(kStatesStr[new_state]);
  printf("%s\n", msg.toStdString().c_str());
  emit printToQConsole(msg);
}
