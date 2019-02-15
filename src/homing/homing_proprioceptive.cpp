#include "homing/homing_proprioceptive.h"

//------------------------------------------------------------------------------------//
//--------- Homing Proprioceptive Data classes ---------------------------------------//
//------------------------------------------------------------------------------------//

HomingProprioceptiveStartData::HomingProprioceptiveStartData() {}

HomingProprioceptiveStartData::HomingProprioceptiveStartData(
  const vect<qint16>& _init_torques, const vect<qint16>& _max_torques,
  const quint8 _num_meas)
  : init_torques(_init_torques), max_torques(_max_torques), num_meas(_num_meas)
{}

std::ostream& operator<<(std::ostream& stream, const HomingProprioceptiveStartData& data)
{
  stream << "initial torques = [ ";
  if (data.init_torques.empty())
    stream << "default";
  else
    for (const qint16 value : data.init_torques)
      stream << value << " ";
  stream << " ], maximum torques = [ ";
  for (const qint16 value : data.max_torques)
    stream << value << " ";
  stream << " ], number of measurements = " << static_cast<int>(data.num_meas);
  return stream;
}

std::ostream& operator<<(std::ostream& stream, const HomingProprioceptiveHomeData& data)
{
  stream << "initial cable lengths = [ ";
  for (const double& value : data.init_lengths)
    stream << value << " ";
  stream << " ], initial pulley angles = [ ";
  for (const double& value : data.init_angles)
    stream << value << " ";
  stream << " ]";
  return stream;
}

//------------------------------------------------------------------------------------//
//--------- Homing Proprioceptive class ----------------------------------------------//
//------------------------------------------------------------------------------------//

// For static constexpr passed by reference we need a dummy definition no matter what
constexpr double HomingProprioceptive::kCycleWaitTimeSec_;
constexpr double HomingProprioceptive::kCutoffFreq_;
constexpr char* HomingProprioceptive::kStatesStr[];

HomingProprioceptive::HomingProprioceptive(QObject* parent, CableRobot* robot)
  : QObject(parent), StateMachine(ST_MAX_STATES), robot_ptr_(robot),
    controller_(robot->GetRtCycleTimeNsec())
{
  // Initialize with default values
  num_meas_   = kNumMeasMin_;
  prev_state_ = ST_MAX_STATES;
  ExternalEvent(ST_IDLE);
  prev_state_ = ST_IDLE;
  controller_.SetMotorTorqueSsErrTol(kTorqueSsErrTol_);

  // Setup connection to track robot status
  active_actuators_id_ = robot_ptr_->GetActiveMotorsID();
  actuators_status_.resize(active_actuators_id_.size());
  connect(robot_ptr_, SIGNAL(actuatorStatus(ActuatorStatus)), this,
          SLOT(handleActuatorStatusUpdate(ActuatorStatus)));
}

HomingProprioceptive::~HomingProprioceptive()
{
  disconnect(robot_ptr_, SIGNAL(actuatorStatus(ActuatorStatus)), this,
             SLOT(handleActuatorStatusUpdate(ActuatorStatus)));
}

//--------- Public functions ---------------------------------------------------------//

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

ActuatorStatus HomingProprioceptive::GetActuatorStatus(const id_t id)
{
  ActuatorStatus status;
  for (size_t i = 0; i < active_actuators_id_.size(); i++)
  {
    if (active_actuators_id_[i] != id)
      continue;
    qmutex_.lock();
    status = actuators_status_[i];
    qmutex_.unlock();
    break;
  }
  return status;
}

//--------- External events ----------------------------------------------------------//

void HomingProprioceptive::Start(HomingProprioceptiveStartData* data)
{
  if (data == NULL)
    CLOG(TRACE, "event") << "with NULL";
  else
    CLOG(TRACE, "event") << "with " << *data;

  // clang-format off
  BEGIN_TRANSITION_MAP                          // - Current State -
      TRANSITION_MAP_ENTRY (ST_ENABLED)         // ST_IDLE
      TRANSITION_MAP_ENTRY (ST_START_UP)        // ST_ENABLED
      TRANSITION_MAP_ENTRY (ST_SWITCH_CABLE)    // ST_START_UP
      TRANSITION_MAP_ENTRY (ST_COILING)         // ST_SWITCH_CABLE
      TRANSITION_MAP_ENTRY (ST_COILING)         // ST_COILING
      TRANSITION_MAP_ENTRY (ST_UNCOILING)       // ST_UNCOILING
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)      // ST_OPTIMIZING
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)      // ST_HOME
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)      // ST_FAULT
  END_TRANSITION_MAP(data)
  // clang-format on
}

void HomingProprioceptive::Stop()
{
  CLOG(TRACE, "event");

  qmutex_.lock();
  stop_cmd_recv_ = true;
  qmutex_.unlock();
}

void HomingProprioceptive::Disable()
{
  CLOG(TRACE, "event");

  qmutex_.lock();
  disable_cmd_recv_ = true;
  qmutex_.unlock();

  // clang-format off
  BEGIN_TRANSITION_MAP                          // - Current State -
      TRANSITION_MAP_ENTRY (ST_IDLE)            // ST_IDLE
      TRANSITION_MAP_ENTRY (ST_IDLE)            // ST_ENABLED
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)      // ST_START_UP
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)      // ST_SWITCH_CABLE
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)      // ST_COILING
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)      // ST_UNCOILING
      TRANSITION_MAP_ENTRY (ST_IDLE)            // ST_OPTIMIZING
      TRANSITION_MAP_ENTRY (ST_IDLE)            // ST_HOME
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)      // ST_FAULT
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

void HomingProprioceptive::Optimize()
{
  CLOG(TRACE, "event");
  // clang-format off
  BEGIN_TRANSITION_MAP                          // - Current State -
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)      // ST_IDLE
      TRANSITION_MAP_ENTRY (ST_OPTIMIZING)      // ST_ENABLED
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)      // ST_START_UP
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)      // ST_SWITCH_CABLE
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)      // ST_COILING
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)      // ST_UNCOILING
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)      // ST_OPTIMIZING
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)      // ST_HOME
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)      // ST_FAULT
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

void HomingProprioceptive::GoHome(HomingProprioceptiveHomeData* data)
{
  CLOG(TRACE, "event") << "with " << *data;
  // clang-format off
  BEGIN_TRANSITION_MAP                          // - Current State -
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)      // ST_IDLE
      TRANSITION_MAP_ENTRY (ST_HOME)            // ST_ENABLED
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)      // ST_START_UP
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)      // ST_SWITCH_CABLE
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)      // ST_COILING
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)      // ST_UNCOILING
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)      // ST_OPTIMIZING
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)      // ST_HOME
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)      // ST_FAULT
  END_TRANSITION_MAP(data)
  // clang-format on
}

void HomingProprioceptive::FaultTrigger()
{
  CLOG(TRACE, "event");
  // clang-format off
  BEGIN_TRANSITION_MAP                          // - Current State -
      TRANSITION_MAP_ENTRY (ST_FAULT)           // ST_IDLE
      TRANSITION_MAP_ENTRY (ST_FAULT)           // ST_ENABLED
      TRANSITION_MAP_ENTRY (ST_FAULT)           // ST_START_UP
      TRANSITION_MAP_ENTRY (ST_FAULT)           // ST_SWITCH_CABLE
      TRANSITION_MAP_ENTRY (ST_FAULT)           // ST_COILING
      TRANSITION_MAP_ENTRY (ST_FAULT)           // ST_UNCOILING
      TRANSITION_MAP_ENTRY (ST_FAULT)           // ST_OPTIMIZING
      TRANSITION_MAP_ENTRY (ST_FAULT)           // ST_HOME
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)      // ST_FAULT
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

void HomingProprioceptive::FaultReset()
{
  CLOG(TRACE, "event");
  ExternalEvent(ST_IDLE);
}

//--------- Private slots  -----------------------------------------------------------//

void HomingProprioceptive::handleActuatorStatusUpdate(
  const ActuatorStatus& actuator_status)
{
  for (size_t i = 0; i < active_actuators_id_.size(); i++)
  {
    if (active_actuators_id_[i] != actuator_status.id)
      continue;
    if (actuator_status.state == Actuator::ST_FAULT)
    {
      FaultTrigger();
      return;
    }
    qmutex_.lock();
    actuators_status_[i] = actuator_status;
    qmutex_.unlock();
    break;
  }
}

//--------- States actions -----------------------------------------------------------//

GUARD_DEFINE(HomingProprioceptive, GuardIdle, NoEventData)
{
  if (prev_state_ != ST_FAULT)
    return true;

  robot_ptr_->ClearFaults();

  grabrt::ThreadClock clock(grabrt::Sec2NanoSec(kCycleWaitTimeSec_));
  bool faults_cleared = false;
  while (!faults_cleared)
  {
    if (clock.ElapsedFromStart() > kMaxWaitTimeSec_)
    {
      emit printToQConsole("WARNING: Homing state transition FAILED. Taking too long to "
                           "clear faults.");
      return false;
    }
    faults_cleared = true;
    qmutex_.lock();
    for (ActuatorStatus& actuator_status : actuators_status_)
      if (actuator_status.state == ST_FAULT)
      {
        faults_cleared = false;
        break;
      }
    qmutex_.unlock();
    clock.WaitUntilNext();
  }
  return true;
}

STATE_DEFINE(HomingProprioceptive, Idle, NoEventData)
{
  PrintStateTransition(prev_state_, ST_IDLE);
  prev_state_ = ST_IDLE;
  emit stateChanged(ST_IDLE);

  if (robot_ptr_->AnyMotorEnabled())
    robot_ptr_->DisableMotors();

  qmutex_.lock();
  disable_cmd_recv_ = false; // reset
  qmutex_.unlock();
}

GUARD_DEFINE(HomingProprioceptive, GuardEnabled, NoEventData)
{
  robot_ptr_->SetController(NULL);
  robot_ptr_->EnableMotors();

  grabrt::ThreadClock clock(grabrt::Sec2NanoSec(kCycleWaitTimeSec_));
  while (1)
  {
    if (robot_ptr_->MotorsEnabled())
      return true;
    if (clock.ElapsedFromStart() > kMaxWaitTimeSec_)
      break;
    clock.WaitUntilNext();
  }
  emit printToQConsole("WARNING: Homing state transition FAILED. Taking too long to "
                       "enable drives.");
  return false;
}

STATE_DEFINE(HomingProprioceptive, Enabled, NoEventData)
{
  PrintStateTransition(prev_state_, ST_ENABLED);
  prev_state_ = ST_ENABLED;
  emit stateChanged(ST_ENABLED);

  robot_ptr_->SetMotorsOpMode(grabec::CYCLIC_TORQUE);
  qmutex_.lock();
  stop_cmd_recv_ = false; // reset
  if (disable_cmd_recv_)
    InternalEvent(ST_IDLE);
  qmutex_.unlock();
}

STATE_DEFINE(HomingProprioceptive, StartUp, HomingProprioceptiveStartData)
{
  PrintStateTransition(prev_state_, ST_START_UP);
  prev_state_ = ST_START_UP;

  QString msg("Start up phase complete\nRobot in predefined configuration\nInitial "
              "torque values:");

  working_actuator_idx_ = 0;
  num_meas_             = data->num_meas;
  num_tot_meas_         = (2 * num_meas_ - 1) * active_actuators_id_.size();
  init_torques_.clear();
  max_torques_ = data->max_torques;
  torques_.resize(num_meas_);

  robot_ptr_->SetController(&controller_);
  for (size_t i = 0; i < active_actuators_id_.size(); ++i)
  {
    // Setup initial target torque for each motor
    init_torques_.push_back(data->init_torques[i]);
    pthread_mutex_lock(&robot_ptr_->Mutex());
    controller_.SetMotorID(active_actuators_id_[i]);
    controller_.SetMode(ControlMode::MOTOR_TORQUE);
    controller_.SetMotorTorqueTarget(init_torques_.back()); // = data->init_torques[i]
    pthread_mutex_unlock(&robot_ptr_->Mutex());
    // Wait until each motor reached user-given initial torque setpoint
    RetVal ret = WaitUntilTargetReached();
    if (ret != RetVal::OK)
    {
      if (ret == RetVal::ETIMEOUT)
        emit printToQConsole(
          "WARNING: Start up phase is taking too long: operation aborted");
      robot_ptr_->SetController(NULL);
      InternalEvent(ST_ENABLED);
      return;
    }
    msg.append(QString("\n\t%1±%2 ‰").arg(init_torques_.back()).arg(kTorqueSsErrTol_));
  }
  // At the beginning we don't know where we are, neither we care.
  // Just update encoder home position to be used as reference to compute deltas.
  robot_ptr_->UpdateHomeConfig(0.0, 0.0);

  emit printToQConsole(msg);
  emit stateChanged(ST_START_UP);
}

GUARD_DEFINE(HomingProprioceptive, GuardSwitch, NoEventData)
{
  if (prev_state_ == ST_START_UP)
    return true;

  pthread_mutex_lock(&robot_ptr_->Mutex());
  if (working_actuator_idx_ < active_actuators_id_.size())
  {
    // We are not done ==> move to next cable
    pthread_mutex_unlock(&robot_ptr_->Mutex());
    return true;
  }
  pthread_mutex_unlock(&robot_ptr_->Mutex());

  InternalEvent(ST_ENABLED);
  emit acquisitionComplete();
  return false;
}

STATE_DEFINE(HomingProprioceptive, SwitchCable, NoEventData)
{
  static vect<id_t> motors_id = robot_ptr_->GetActiveMotorsID();

  PrintStateTransition(prev_state_, ST_SWITCH_CABLE);
  prev_state_ = ST_SWITCH_CABLE;

  // Compute sequence of torque setpoints for i-th actuator
  qint16 delta_torque =
    (max_torques_[working_actuator_idx_] - init_torques_[working_actuator_idx_]) /
    (static_cast<qint16>(num_meas_) - 1);
  for (quint8 i = 0; i < num_meas_ - 1; ++i)
    torques_[i] = init_torques_[working_actuator_idx_] + i * delta_torque;
  torques_.back() =
    max_torques_[working_actuator_idx_]; // last element is forced to be = max torque

  // Setup first setpoint of the sequence
  pthread_mutex_lock(&robot_ptr_->Mutex());
  controller_.SetMotorID(motors_id[working_actuator_idx_]);
  controller_.SetMode(ControlMode::MOTOR_TORQUE);
  controller_.SetMotorTorqueTarget(torques_.front());
  pthread_mutex_unlock(&robot_ptr_->Mutex());

  emit printToQConsole(
    QString("Switched to actuator #%1.\nInitial torque setpoint = %2 ‰")
      .arg(motors_id[working_actuator_idx_])
      .arg(torques_.front()));
  working_actuator_idx_++;
  meas_step_ = 0; // reset

  if (WaitUntilTargetReached() == RetVal::OK && WaitUntilPlatformSteady() == RetVal::OK)
  {
    emit stateChanged(ST_SWITCH_CABLE);
    return;
  }
  robot_ptr_->SetController(NULL);
  InternalEvent(ST_ENABLED);
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

  pthread_mutex_lock(&robot_ptr_->Mutex());
  controller_.SetMotorTorqueTarget(torques_[meas_step_]);
  pthread_mutex_unlock(&robot_ptr_->Mutex());
  emit printToQConsole(QString("Next torque setpoint = %1 ‰").arg(torques_[meas_step_]));

  if (WaitUntilTargetReached() == RetVal::OK && WaitUntilPlatformSteady() == RetVal::OK)
  {
    DumpMeasAndMoveNext();
    emit stateChanged(ST_COILING);
    return;
  }
  robot_ptr_->SetController(NULL);
  InternalEvent(ST_ENABLED);
}

ENTRY_DEFINE(HomingProprioceptive, EntryUncoiling, NoEventData) { meas_step_++; }

STATE_DEFINE(HomingProprioceptive, Uncoiling, NoEventData)
{
  PrintStateTransition(prev_state_, ST_UNCOILING);
  prev_state_ = ST_UNCOILING;

  if (meas_step_ == 2 * num_meas_)
  {
    InternalEvent(ST_SWITCH_CABLE);
    return;
  }

  const ulong kOffset = num_tot_meas_ / active_actuators_id_.size();
  pthread_mutex_lock(&robot_ptr_->Mutex());
  controller_.SetMotorTorqueTarget(torques_[kOffset - meas_step_]);
  pthread_mutex_unlock(&robot_ptr_->Mutex());
  emit printToQConsole(
    QString("Next torque setpoint = %1 ‰").arg(torques_[kOffset - meas_step_]));

  if (WaitUntilTargetReached() == RetVal::OK && WaitUntilPlatformSteady() == RetVal::OK)
  {
    DumpMeasAndMoveNext();
    emit stateChanged(ST_UNCOILING);
    return;
  }
  robot_ptr_->SetController(NULL);
  InternalEvent(ST_ENABLED);
}

STATE_DEFINE(HomingProprioceptive, Optimizing, NoEventData)
{
  PrintStateTransition(prev_state_, ST_OPTIMIZING);
  prev_state_ = ST_OPTIMIZING;
  emit stateChanged(ST_OPTIMIZING);

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
  emit stateChanged(ST_HOME);

  // Current home corresponds to robot configuration at the beginning of homing procedure.
  // Remind that motors home count corresponds to null cable length, which needs to be
  // updated...
  if (robot_ptr_->GoHome()) // (position control)
  {
    // ...which is done here.
    for (id_t motor_id : robot_ptr_->GetActiveMotorsID())
      robot_ptr_->UpdateHomeConfig(motor_id, data->init_lengths[motor_id],
                                   data->init_angles[motor_id]);
    emit homingComplete();
  }
  else
  {
    emit printToQConsole("WARNING: Something went unexpectedly wrong, please start over");
    InternalEvent(ST_ENABLED);
  }
}

STATE_DEFINE(HomingProprioceptive, Fault, NoEventData)
{
  PrintStateTransition(prev_state_, ST_FAULT);
  prev_state_ = ST_FAULT;
  emit stateChanged(ST_FAULT);
}

//--------- Private functions --------------------------------------------------------//

RetVal HomingProprioceptive::WaitUntilTargetReached()
{
  grabrt::ThreadClock clock(grabrt::Sec2NanoSec(kCycleWaitTimeSec_));
  while (1)
  {
    // Check if target is reached
    pthread_mutex_lock(&robot_ptr_->Mutex());
    if (controller_.MotorTorqueTargetReached())
    {
      pthread_mutex_unlock(&robot_ptr_->Mutex());
      return RetVal::OK;
    }
    pthread_mutex_unlock(&robot_ptr_->Mutex());
    // Check if external abort signal is received
    qmutex_.lock();
    if (stop_cmd_recv_ || disable_cmd_recv_)
    {
      qmutex_.unlock();
      return RetVal::EINT;
    }
    qmutex_.unlock();
    // Check if timeout expired (safety feature to prevent hanging in forever)
    if (clock.ElapsedFromStart() > kMaxWaitTimeSec_)
    {
      emit printToQConsole(
        "WARNING: Actuator is taking too long to reach target: operation aborted");
      return RetVal::ETIMEOUT;
    }
    clock.WaitUntilNext();
  }
}

RetVal HomingProprioceptive::WaitUntilPlatformSteady()
{
  // Compute these once for all
  static constexpr size_t kBuffSize =
    static_cast<size_t>(kBufferingTimeSec_ / kCycleWaitTimeSec_);
  // LP filters setup
  static std::vector<grabnum::LowPassFilter> lp_filters(
    active_actuators_id_.size(),
    grabnum::LowPassFilter(kCutoffFreq_, kCycleWaitTimeSec_));
  for (size_t i = 0; i < active_actuators_id_.size(); i++)
    lp_filters[i].Reset();

  // debug
  ulong step = (working_actuator_idx_ - 1) * (2 * num_meas_ - 1) + meas_step_;
  std::ofstream dbg_log_file("/home/labpc/MATLAB-Drive/cable_robot/pulley_angles/pulley_angles_" + std::to_string(step) + ".txt");
  // Init
  bool swinging = true;
  std::vector<RingBufferD> pulleys_angles(active_actuators_id_.size(),
                                          RingBufferD(kBuffSize));
  grabrt::ThreadClock clock(grabrt::Sec2NanoSec(kCycleWaitTimeSec_));
  // Start waiting
  while (swinging)
  {
    for (size_t i = 0; i < active_actuators_id_.size(); i++)
    {
      QCoreApplication::processEvents();
      qmutex_.lock();
      // Check if external abort signal is received
      if (stop_cmd_recv_ || disable_cmd_recv_)
      {
        qmutex_.unlock();
        dbg_log_file.close(); // debug
        return RetVal::EINT;
      }
      pulleys_angles[i].Add(
        lp_filters[i].Filter(actuators_status_[i].pulley_angle)); // add filtered angle
      // debug
      dbg_log_file << pulleys_angles[i].Tail() << "," << actuators_status_[i].pulley_angle
                   << ",";
      qmutex_.unlock();
      if (!pulleys_angles[i].IsFull()) // wait at least until buffer is full
        continue;
      // Condition to detect steadyness
      swinging = grabnum::Std(pulleys_angles[i].Data()) > kMaxAngleDeviation_;
      if (swinging)
        break;
    }
    // debug
    dbg_log_file << "\n";
    // Check if timeout expired (safety feature to prevent hanging in forever)
    if (clock.ElapsedFromStart() > kMaxWaitTimeSec_)
    {
      dbg_log_file.close(); // debug
      emit printToQConsole(
        "WARNING: Platform is taking too long to stabilize: operation aborted");
      return RetVal::ETIMEOUT;
    }
    clock.WaitUntilNext();
  }
  dbg_log_file.close();
  return RetVal::OK;
}

void HomingProprioceptive::DumpMeasAndMoveNext()
{
  robot_ptr_->CollectMeas();
  emit printToQConsole("Measurements collected");
  robot_ptr_->DumpMeas();
  emit printToQConsole("Measurements dumped onto log file");
  meas_step_++;
  double normalized_value =
    round(100. * ((working_actuator_idx_ - 1.0) / active_actuators_id_.size() +
                  static_cast<double>(meas_step_) / num_tot_meas_));
  emit progressValue(static_cast<int>(normalized_value));
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
