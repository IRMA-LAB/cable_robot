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

// For static constexpr passed by reference we need a dummy definition no matter what
constexpr double HomingProprioceptive::kCycleWaitTimeSec_;
constexpr double HomingProprioceptive::kCutoffFreq_;
constexpr char* HomingProprioceptive::kStatesStr[];

HomingProprioceptive::HomingProprioceptive(QObject* parent, CableRobot* robot)
  : QObject(parent), StateMachine(ST_MAX_STATES), robot_(robot)
{
  // init with default values
  num_meas_ = kNumMeasMin_;
  prev_state_ = ST_MAX_STATES;
}

//--------- Public functions -----------------------------------------------------------//

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
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_HOME
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
      TRANSITION_MAP_ENTRY (ST_ENABLED)                          // ST_HOME
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
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)			// ST_HOME
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
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)			// ST_HOME
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
      TRANSITION_MAP_ENTRY (ST_FAULT)                               // ST_HOME
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
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)			// ST_HOME
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
  robot_->EnableMotors();
  if (robot_->MotorsEnabled())
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

  pthread_mutex_lock(&robot_->Mutex());
  robot_->SetController(&controller_);
  pthread_mutex_unlock(&robot_->Mutex());
  vect<id_t> motors_id = robot_->GetActiveMotorsID();
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
      pthread_mutex_lock(&robot_->Mutex());
      controller_.SetMotorID(motors_id[i]);
      controller_.SetMotorTorqueTarget(init_torques_.back()); //  = data->init_torques[i]
      pthread_mutex_unlock(&robot_->Mutex());
      while (1)
      {
        pthread_mutex_lock(&robot_->Mutex());
        if (controller_.MotorTorqueTargetReached(current_torque))
          break;
        pthread_mutex_unlock(&robot_->Mutex());
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
  InternalEvent(ST_SWITCH_CABLE);
}

GUARD_DEFINE(HomingProprioceptive, GuardSwitch, NoEventData)
{
  if (prev_state_ == ST_START_UP)
    return true;

  pthread_mutex_lock(&robot_->Mutex());
  if (controller_.GetMotorsID().front() != robot_->GetActiveMotorsID().back())
  {
    // We are not done ==> move to next cable
    pthread_mutex_unlock(&robot_->Mutex());
    return true;
  }
  pthread_mutex_unlock(&robot_->Mutex());

  InternalEvent(ST_ENABLED);
  emit acquisitionComplete();
  return false;
}

STATE_DEFINE(HomingProprioceptive, SwitchCable, NoEventData)
{
  static quint8 motor_id_idx = 0;
  static vect<id_t> motors_id = robot_->GetActiveMotorsID();

  PrintStateTransition(prev_state_, ST_SWITCH_CABLE);
  prev_state_ = ST_SWITCH_CABLE;

  qint16 delta_torque =
    (max_torques_[motor_id_idx] - init_torques_[motor_id_idx]) / (num_meas_ - 1);
  for (quint8 i = 0; i < num_meas_ - 1; ++i)
    torques_[i] = init_torques_[motor_id_idx] + i * delta_torque;
  torques_.back() =
    max_torques_[motor_id_idx]; // last element is forced to be = max torque

  pthread_mutex_lock(&robot_->Mutex());
  controller_.SetMotorID(motors_id[motor_id_idx]);
  controller_.SetMotorTorqueTarget(torques_.front());
  pthread_mutex_unlock(&robot_->Mutex());

  emit printToQConsole(
    QString("Switched to actuator #%1.\nInitial torque setpoint = %2 ‰")
      .arg(motors_id[motor_id_idx])
      .arg(torques_.front()));
  motor_id_idx++;
  meas_step_ = 0; // reset

  WaitUntilPlatformSteady();
  InternalEvent(ST_COILING);
}

ENTRY_DEFINE(HomingProprioceptive, EntryCoiling, NoEventData) { DumpMeasAndMoveNext(); }

STATE_DEFINE(HomingProprioceptive, Coiling, NoEventData)
{
  PrintStateTransition(prev_state_, ST_COILING);
  prev_state_ = ST_COILING;

  if (meas_step_ == num_meas_)
  {
    InternalEvent(ST_UNCOILING);
    return;
  }

  pthread_mutex_lock(&robot_->Mutex());
  controller_.SetMotorTorqueTarget(torques_[meas_step_]);
  emit printToQConsole(
    QString("Next torque setpoint = %1 ‰").arg(controller_.GetMotorTorqueTarget()));
  pthread_mutex_unlock(&robot_->Mutex());

  WaitUntilPlatformSteady();
  DumpMeasAndMoveNext();
  InternalEvent(ST_COILING);
}

ENTRY_DEFINE(HomingProprioceptive, EntryUncoiling, NoEventData) { meas_step_++; }

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

  pthread_mutex_lock(&robot_->Mutex());
  controller_.SetMotorTorqueTarget(torques_[kOffset - meas_step_]);
  emit printToQConsole(
    QString("Next torque setpoint = %1 ‰").arg(controller_.GetMotorTorqueTarget()));
  pthread_mutex_unlock(&robot_->Mutex());

  WaitUntilPlatformSteady();
  DumpMeasAndMoveNext();
  InternalEvent(ST_UNCOILING);
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
    for (id_t motor_id : robot_->GetActiveMotorsID())
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

//--------- Private functions ---------------------------------------------------------//

void HomingProprioceptive::WaitUntilPlatformSteady()
{
  // Compute these once for all
  static constexpr size_t kBuffSize =
    static_cast<size_t>(kBufferingTimeSec_ / kCycleWaitTimeSec_);
  static const std::vector<id_t> motors_id = robot_->GetActiveMotorsID();

  // LP filters setup
  static std::vector<grabnum::LowPassFilter> lp_filters(
    motors_id.size(), grabnum::LowPassFilter(kCutoffFreq_, kCycleWaitTimeSec_));
  for (size_t i = 0; i < motors_id.size(); i++)
    lp_filters[i].Reset();

  // Init
  ActuatorStatus status;
  bool swinging = true;
  std::vector<RingBufferD> pulleys_angles(motors_id.size(), RingBufferD(kBuffSize));
  grabrt::ThreadClock clock(grabrt::Sec2NanoSec(kCycleWaitTimeSec_));
  // Start waiting
  while (swinging)
  {
    for (size_t i = 0; i < motors_id.size(); i++)
    {
      status = robot_->GetActuatorStatus(motors_id[i]);
      pulleys_angles[i].Add(
        lp_filters[i].Filter(status.pulley_angle)); // add filtered angle
      if (!pulleys_angles[i].IsFull())              // wait at least until buffer is full
        continue;
      // Condition to detect steadyness
      swinging = grabnum::Std(pulleys_angles[i].Data()) > kMaxAngleDeviation_;
    }
    clock.WaitUntilNext();
  }
}

void HomingProprioceptive::DumpMeasAndMoveNext()
{
  robot_->CollectMeas();
  robot_->DumpMeas();
  meas_step_++;
}

void HomingProprioceptive::PrintStateTransition(const States current_state,
                                                const States new_state) const
{
  if (current_state == new_state)
    return;
  emit stateChanged(new_state);
  QString msg;
  if (current_state != ST_MAX_STATES)
    msg = QString("Homing state transition: %1 --> %2")
            .arg(kStatesStr[current_state], kStatesStr[new_state]);
  else
    msg = QString("Homing initial state: %1").arg(kStatesStr[new_state]);
  emit printToQConsole(msg);
}
