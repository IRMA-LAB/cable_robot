#include "homing/homing_proprioceptive.h"

////////////////////////////////////////////////////////////////////////////
//// Homing proprioceptive data class
////////////////////////////////////////////////////////////////////////////

HomingProprioceptiveStartData::HomingProprioceptiveStartData() {}

HomingProprioceptiveStartData::HomingProprioceptiveStartData(
  const vect<qint16>& _init_torques, const vect<qint16>& _max_torques,
  const quint8 _num_meas)
  : init_torques(_init_torques), max_torques(_max_torques), num_meas(_num_meas)
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
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)			// ST_SWITCH_CABLE
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)			// ST_COILING
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)			// ST_UNCOILING
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
      TRANSITION_MAP_ENTRY (ST_ENABLED)                          // ST_SWITCH_CABLE
      TRANSITION_MAP_ENTRY (ST_ENABLED)                          // ST_START_UP
      TRANSITION_MAP_ENTRY (ST_ENABLED)                          // ST_COILING
      TRANSITION_MAP_ENTRY (ST_ENABLED)                          // ST_UNCOILING
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
      TRANSITION_MAP_ENTRY (ST_SWITCH_CABLE)                 // ST_START_UP
      TRANSITION_MAP_ENTRY (ST_COILING)                            // ST_SWITCH_CABLE
      TRANSITION_MAP_ENTRY (ST_COILING)                            // ST_COILING
      TRANSITION_MAP_ENTRY (ST_UNCOILING)			// ST_UNCOILING
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
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)                    // ST_SWITCH_CABLE
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)			// ST_COILING
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)			// ST_UNCOILING
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
      TRANSITION_MAP_ENTRY (ST_FAULT)                               // ST_SWITCH_CABLE
      TRANSITION_MAP_ENTRY (ST_FAULT)                               // ST_COILING
      TRANSITION_MAP_ENTRY (ST_FAULT)                               // ST_UNCOILING
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
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)                   // ST_SWITCH_CABLE
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_COILING
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_UNCOILING
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

  num_meas_ = data->num_meas;
  init_torques_.clear();
  max_torques_ = data->max_torques;
  torques_.reserve(num_meas_);

  vect<quint8> motors_id = robot_->GetMotorsID();
  for (quint8 i = 0; i < motors_id.size(); ++i)
  {
    qint16 current_torque = robot_->GetActuatorStatus(motors_id[i]).motor_torque;
    // Use current torque values if not given
    if (!data->init_torques.empty())
    {
      init_torques_.push_back(data->init_torques[i]);
      // Wait until all motors reached user-given initial torque setpoint
      controller_.SetMotorID(motors_id[i]);
      controller_.SetMotorTorqueTarget(init_torques_.back());
      while (1)
      {
        if (controller_.MotorTorqueTargetReached(current_torque))
          break;
        current_torque = robot_->GetActuatorStatus(motors_id[i]).motor_torque;
        // todo: inserisci un tempo di attesa qui magari
      }
    }
    else
      init_torques_.push_back(current_torque);
    msg.append(QString("\n\t%1 %").arg(current_torque)); // todo: unità di misura??
  }
  emit printToQConsole(msg);
}

GUARD_DEFINE(HomingProprioceptive, GuardSwitch, NoEventData)
{
  if (prev_state_ == ST_START_UP)
    return true;

  if (controller_.GetMotorsID().front() !=
      robot_->GetMotorsID().back()) // we are not done ==> move to next cable
    return true;

  emit acquisitionComplete();
  InternalEvent(ST_ENABLED); // todo: funzionerà?
  return false;
}

STATE_DEFINE(HomingProprioceptive, SwitchCable, NoEventData)
{
  static quint8 motor_id = 0;

  PrintStateTransition(prev_state_, ST_SWITCH_CABLE);
  prev_state_ = ST_SWITCH_CABLE;

  qint16 delta_torque =
    (max_torques_[motor_id] - init_torques_[motor_id]) / (num_meas_ - 1);
  for (quint8 i = 0; i < num_meas_ - 1; ++i)
    torques_[i] = init_torques_[motor_id] + i * delta_torque;
  torques_.back() = max_torques_[motor_id]; // last element is forced to be = max torque

  controller_.SetMotorID(motor_id);
  controller_.SetMotorTorqueTarget(torques_.front());

  emit printToQConsole(
    QString("Switched to actuator #%1.\nInitial torque setpoint = %2 ‰")
      .arg(motor_id, torques_.front()));
  motor_id++;
}

GUARD_DEFINE(HomingProprioceptive, GuardCoiling, NoEventData)
{
  if (controller_.MotorTorqueTargetReached(
        robot_->GetActuatorStatus(controller_.GetMotorsID().front()).motor_torque))
  {
    robot_->CollectMeas();
    robot_->DumpMeas();
    return true;
  }

  emit printToQConsole("Torque target not reached yet! Please wait...");
  return false;
}

STATE_DEFINE(HomingProprioceptive, Coiling, NoEventData)
{
  static quint8 counter = 0;

  PrintStateTransition(prev_state_, ST_COILING);
  prev_state_ = ST_COILING;

  quint8 meas_step = ++counter % num_meas_;
  if (meas_step == 0)
  {
    InternalEvent(ST_UNCOILING);
    return;
  }

  controller_.SetMotorTorqueTarget(torques_[meas_step]);
  emit printToQConsole(QString("Next torque setpoint = %1 ‰").arg(torques_[meas_step]));
}

GUARD_DEFINE(HomingProprioceptive, GuardUncoiling, NoEventData)
{
  if (prev_state_ == ST_COILING)
    return true;

  if (controller_.MotorTorqueTargetReached(
        robot_->GetActuatorStatus(controller_.GetMotorsID().front()).motor_torque))
  {
    robot_->CollectMeas();
    robot_->DumpMeas();
    return true;
  }

  emit printToQConsole("Torque target not reached yet! Please wait...");
  return false;
}

STATE_DEFINE(HomingProprioceptive, Uncoiling, NoEventData)
{
  static qint16 meas_step = num_meas_ - 2;

  PrintStateTransition(prev_state_, ST_UNCOILING);
  prev_state_ = ST_UNCOILING;

  if (meas_step < 0)
  {
    InternalEvent(ST_SWITCH_CABLE);
    meas_step = num_meas_ - 2;  // reset if transition is successful
    return;
  }

  controller_.SetMotorTorqueTarget(torques_[static_cast<quint8>(meas_step)]);
  emit printToQConsole(
    QString("Next torque setpoint = %1 ‰").arg(controller_.GetMotorTorqueTarget()));
  meas_step--;
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
