#include "homing/homing_proprioceptive.h"

//----------------------------------------------------------------------------------------------------//
//--------- Homing proprioceptive data classes ----------------------------------------//
//----------------------------------------------------------------------------------------------------//

HomingProprioceptiveStartData::HomingProprioceptiveStartData() {}

HomingProprioceptiveStartData::HomingProprioceptiveStartData(
  const vect<qint16>& _init_torques, const vect<qint16>& _max_torques,
  const quint8 _num_meas)
  : init_torques(_init_torques), max_torques(_max_torques), num_meas(_num_meas)
{
}

std::ostream& operator<<(std::ostream& stream, const HomingProprioceptiveStartData& data)
{
  stream << "initial torques=[ ";
  for (const qint16 value : data.init_torques)
    stream << value << " ";
  stream << " ]\tmaximum torques=[ ";
  for (const qint16 value : data.max_torques)
    stream << value << " ";
  stream << " ]\tnumber of measurements=" << data.num_meas;
  return stream;
}

std::ostream& operator<<(std::ostream& stream, const HomingProprioceptiveHomeData& data)
{
  stream << "initial cable lengths=[ ";
  for (const double& value : data.init_lengths)
    stream << value << " ";
  stream << " ]\tinitial pulley angles=[ ";
  for (const double& value : data.init_angles)
    stream << value << " ";
  stream << " ]";
  return stream;
}

//--------------------------------------------------------------------------------------------------//
//--------- Homing proprioceptive class ------------------------------------------------//
//--------------------------------------------------------------------------------------------------//

constexpr char* HomingProprioceptive::kStatesStr[];

HomingProprioceptive::HomingProprioceptive(QObject* parent, CableRobot* robot)
  : QObject(parent), StateMachine(ST_MAX_STATES), robot_(robot)
{
  // init with default values
  num_meas_ = kNumMeasMin;
  prev_state_ = ST_MAX_STATES;
}

//--------- External Events ---------------------------------------------------------//

void HomingProprioceptive::Start(HomingProprioceptiveStartData* data)
{
  CLOG(TRACE, "event") << "with " << *data;
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
  CLOG(TRACE, "event");
  // clang-format off
  BEGIN_TRANSITION_MAP			              			// - Current State -
      TRANSITION_MAP_ENTRY (ST_IDLE)                                  // ST_IDLE
      TRANSITION_MAP_ENTRY (ST_IDLE)                                  // ST_ENABLED
      TRANSITION_MAP_ENTRY (ST_ENABLED)                          // ST_START_UP
      TRANSITION_MAP_ENTRY (ST_ENABLED)                          // ST_SWITCH_CABLE
      TRANSITION_MAP_ENTRY (ST_ENABLED)                          // ST_COILING
      TRANSITION_MAP_ENTRY (ST_ENABLED)                          // ST_UNCOILING
      TRANSITION_MAP_ENTRY (ST_ENABLED)                          // ST_OPTIMIZING
      TRANSITION_MAP_ENTRY (ST_ENABLED)                          // ST_GO_HOME
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_FAULT
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

void HomingProprioceptive::Next()
{
  CLOG(TRACE, "event");
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
  CLOG(TRACE, "event");
  // clang-format off
  BEGIN_TRANSITION_MAP			              			// - Current State -
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)                   // ST_IDLE
      TRANSITION_MAP_ENTRY (ST_OPTIMIZING)                      // ST_ENABLED
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

void HomingProprioceptive::GoHome(HomingProprioceptiveHomeData* data)
{
  CLOG(TRACE, "event") << "with " << *data;
  // clang-format off
  BEGIN_TRANSITION_MAP			              			// - Current State -
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)                   // ST_IDLE
      TRANSITION_MAP_ENTRY (ST_HOME)                               // ST_ENABLED
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)                   // ST_START_UP
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)                   // ST_SWITCH_CABLE
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)                   // ST_COILING
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_UNCOILING
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_OPTIMIZING
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)			// ST_GO_HOME
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_FAULT
  END_TRANSITION_MAP(data)
  // clang-format on
}

void HomingProprioceptive::FaultTrigger()
{
  CLOG(TRACE, "event");
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
  CLOG(TRACE, "event");
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

//--------- States actions -----------------------------------------------------------//

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
  torques_.resize(num_meas_);

  vect<ID_t> motors_id = robot_->GetMotorsID();
  for (size_t i = 0; i < motors_id.size(); ++i)
  {
    qint16 current_torque = robot_->GetActuatorStatus(motors_id[i]).motor_torque;
    // Use current torque values if not given
    if (data->init_torques.empty())
      init_torques_.push_back(current_torque);
    else
    {
      init_torques_.push_back(data->init_torques[i]);
      // Wait until all motors reached user-given initial torque setpoint
      controller_.SetMotorID(motors_id[i]);
      controller_.SetMotorTorqueTarget(init_torques_.back()); //  = data->init_torques[i]
      while (1)
      {
        if (controller_.MotorTorqueTargetReached(current_torque))
          break;
        current_torque = robot_->GetActuatorStatus(motors_id[i]).motor_torque;
        // todo: inserisci un tempo di attesa qui magari
      }
    }
    msg.append(QString("\n\t%1 ‰").arg(current_torque));
  }
  // At the beginning we don't know where we are, neither we care.
  // Just update encoder home position to be used as reference to compute deltas.
  robot_->UpdateHomeConfig(0.0, 0.0);

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
  meas_step_ = 0; // reset
}

GUARD_DEFINE(HomingProprioceptive, GuardCoiling, NoEventData)
{
  // Note: there is only one motor ID at the time in this controller
  if (controller_.MotorTorqueTargetReached(
        robot_->GetActuatorStatus(controller_.GetMotorsID().front()).motor_torque))
  {
    robot_->CollectMeas();
    robot_->DumpMeas();
    meas_step_++;
    return true;
  }

  emit printToQConsole("Torque target not reached yet! Please wait...");
  return false;
}

STATE_DEFINE(HomingProprioceptive, Coiling, NoEventData)
{
  PrintStateTransition(prev_state_, ST_COILING);
  prev_state_ = ST_COILING;

  if (meas_step_ == num_meas_)
  {
    InternalEvent(ST_UNCOILING);
    return;
  }

  controller_.SetMotorTorqueTarget(torques_[meas_step_]);
  emit printToQConsole(
    QString("Next torque setpoint = %1 ‰").arg(controller_.GetMotorTorqueTarget()));
}

GUARD_DEFINE(HomingProprioceptive, GuardUncoiling, NoEventData)
{
  if (prev_state_ == ST_COILING)
  {
    meas_step_++;
    return true;
  }

  if (controller_.MotorTorqueTargetReached(
        robot_->GetActuatorStatus(controller_.GetMotorsID().front()).motor_torque))
  {
    robot_->CollectMeas();
    robot_->DumpMeas();
    meas_step_++;
    return true;
  }

  emit printToQConsole("Torque target not reached yet! Please wait...");
  return false;
}

STATE_DEFINE(HomingProprioceptive, Uncoiling, NoEventData)
{
  static const quint16 kOffset = 2 * num_meas_ - 1;

  PrintStateTransition(prev_state_, ST_UNCOILING);
  prev_state_ = ST_UNCOILING;

  if (meas_step_ == 2 * num_meas_)
  {
    InternalEvent(ST_SWITCH_CABLE);
    return;
  }

  controller_.SetMotorTorqueTarget(torques_[kOffset - meas_step_]);
  emit printToQConsole(
    QString("Next torque setpoint = %1 ‰").arg(controller_.GetMotorTorqueTarget()));
}

STATE_DEFINE(HomingProprioceptive, Optimizing, NoEventData)
{
  PrintStateTransition(prev_state_, ST_OPTIMIZING);
  prev_state_ = ST_OPTIMIZING;

  HomingProprioceptiveHomeData* home_data = new HomingProprioceptiveHomeData;
  // do optimization here..
  bool optimization_success = true; // dummy

  if (optimization_success)
  {
    emit printToQConsole("Optimization complete");
    InternalEvent(ST_HOME, home_data);
  }
  else
  {
    emit printToQConsole("Optimization failed");
    InternalEvent(ST_ENABLED);
  }
}

STATE_DEFINE(HomingProprioceptive, Home, HomingProprioceptiveHomeData)
{
  PrintStateTransition(prev_state_, ST_HOME);
  prev_state_ = ST_HOME;

  // Current home corresponds to robot configuration at the beginning of homing procedure.
  // Remind that motors home count corresponds to null cable length, which needs to be
  // updated...
  if (robot_->GoHome()) // (position control)
  {
    // ...which is done here.
    for (ID_t motor_id : robot_->GetMotorsID())
      robot_->UpdateHomeConfig(motor_id, data->init_lengths[motor_id],
                               data->init_angles[motor_id]);
    emit homingComplete();
  }
  else
  {
    emit printToQConsole("Something went unexpectedly wrong, please start over");
    InternalEvent(ST_ENABLED);
  }
}

STATE_DEFINE(HomingProprioceptive, Fault, NoEventData)
{
  PrintStateTransition(prev_state_, ST_FAULT);
  prev_state_ = ST_FAULT;
}

//--------- Miscellaneous -----------------------------------------------------------//

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
  case ST_HOME:
    return false;
  default:
    return true;
  }
}

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
  emit printToQConsole(msg);
}
