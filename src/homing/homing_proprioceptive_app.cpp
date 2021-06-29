/**
 * @file homing_proprioceptive.cpp
 * @author Simone Comari
 * @date 13 Jul 2020
 * @brief This file includes definitions of classes present in homing_proprioceptive.h.
 */

#include "homing/homing_proprioceptive_app.h"

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
  stream << "initial robot pose =\n" << data.init_pose;
  return stream;
}

//------------------------------------------------------------------------------------//
//--------- Homing Proprioceptive class ----------------------------------------------//
//------------------------------------------------------------------------------------//

// For static constexpr passed by reference we need a dummy definition no matter what
constexpr char* HomingProprioceptiveApp::kStatesStr[];
const QString HomingProprioceptiveApp::kMatlabOptimizationResultsLoc =
  QString(SRCDIR) + "libs/grab_common/libcdpr/cdpr_matlab/data/homing_results.json";

HomingProprioceptiveApp::HomingProprioceptiveApp(QObject* parent, CableRobot* robot)
  : QObject(parent), StateMachine(ST_MAX_STATES), robot_ptr_(robot),
    controller_(robot->GetRtCycleTimeNsec()), optimization_progess_timer_(this)
{
  // Initialize with default values
  num_meas_   = kNumMeasMin_;
  prev_state_ = ST_MAX_STATES;
  ExternalEvent(ST_IDLE);
  prev_state_ = ST_IDLE;
  controller_.SetMotorTorqueSsErrTol(kTorqueSsErrTol_);

  // Setup connection to track robot status
  active_actuators_id_ = robot_ptr_->getActiveMotorsID();
  actuators_status_.resize(active_actuators_id_.size());
  connect(robot_ptr_, SIGNAL(actuatorStatus(ActuatorStatus)), this,
          SLOT(handleActuatorStatusUpdate(ActuatorStatus)));
  connect(this, SIGNAL(stopWaitingCmd()), robot_ptr_, SLOT(stopWaiting()));

  // Setup timer for optimization progress
  connect(&optimization_progess_timer_, SIGNAL(timeout()), this,
          SLOT(updateOptimizationProgress()));
}

HomingProprioceptiveApp::~HomingProprioceptiveApp()
{
  disconnect(robot_ptr_, SIGNAL(actuatorStatus(ActuatorStatus)), this,
             SLOT(handleActuatorStatusUpdate(ActuatorStatus)));
  disconnect(this, SIGNAL(stopWaitingCmd()), robot_ptr_, SLOT(stopWaiting()));
  disconnect(&optimization_progess_timer_, SIGNAL(timeout()), this,
             SLOT(updateOptimizationProgress()));
}

//--------- Public functions ---------------------------------------------------------//

bool HomingProprioceptiveApp::isCollectingData()
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

ActuatorStatus HomingProprioceptiveApp::getActuatorStatus(const id_t id)
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

void HomingProprioceptiveApp::start(HomingProprioceptiveStartData* data)
{
  if (data == nullptr)
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
      TRANSITION_MAP_ENTRY (ST_START_UP)        // ST_HOME
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)      // ST_FAULT
  END_TRANSITION_MAP(data)
  // clang-format on
}

void HomingProprioceptiveApp::stop()
{
  CLOG(TRACE, "event");

  if (robot_ptr_->isWaiting())
    emit stopWaitingCmd();
}

void HomingProprioceptiveApp::disable()
{
  CLOG(TRACE, "event");

  qmutex_.lock();
  disable_cmd_recv_ = true;
  qmutex_.unlock();
  if (robot_ptr_->isWaiting())
    emit stopWaitingCmd();

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
  END_TRANSITION_MAP(nullptr)
  // clang-format on
}

void HomingProprioceptiveApp::optimize()
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
      TRANSITION_MAP_ENTRY (ST_OPTIMIZING)      // ST_HOME
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)      // ST_FAULT
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

void HomingProprioceptiveApp::goHome(HomingProprioceptiveHomeData* data)
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

void HomingProprioceptiveApp::faultTrigger()
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

void HomingProprioceptiveApp::faultReset()
{
  CLOG(TRACE, "event");
  ExternalEvent(ST_IDLE);
}

//--------- Private slots  -----------------------------------------------------------//

void HomingProprioceptiveApp::handleActuatorStatusUpdate(
  const ActuatorStatus& actuator_status)
{
  for (size_t i = 0; i < active_actuators_id_.size(); i++)
  {
    if (active_actuators_id_[i] != actuator_status.id)
      continue;
    if (actuator_status.state == Actuator::ST_FAULT)
    {
      faultTrigger();
      return;
    }
    qmutex_.lock();
    actuators_status_[i] = actuator_status;
    qmutex_.unlock();
    break;
  }
}

void HomingProprioceptiveApp::handleMatlabResultsReady()
{
  optimization_progess_timer_.stop();
  // Read results from json file generated by matlab script
  HomingProprioceptiveHomeData* home_data = new HomingProprioceptiveHomeData;
  if (parseExtFile(kMatlabOptimizationResultsLoc, home_data))
  {
    emit progressValue(100);
    emit printToQConsole("Optimization complete");
    ExternalEvent(ST_HOME, home_data);
    return;
  }
  delete home_data;
  emit printToQConsole("WARNING: Optimization failed");
  ExternalEvent(ST_ENABLED);
}

void HomingProprioceptiveApp::updateOptimizationProgress()
{
  optimization_progress_counter_ = std::min(optimization_progress_counter_ + 1, 95);
  emit progressValue(optimization_progress_counter_);
}

//--------- States actions -----------------------------------------------------------//

GUARD_DEFINE(HomingProprioceptiveApp, GuardIdle, NoEventData)
{
  if (prev_state_ != ST_FAULT)
    return true;

  robot_ptr_->clearFaults();

  grabrt::ThreadClock clock(grabrt::Sec2NanoSec(CableRobot::kCycleWaitTimeSec));
  bool faults_cleared = false;
  while (!faults_cleared)
  {
    if (clock.ElapsedFromStart() > CableRobot::kMaxWaitTimeSec)
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

STATE_DEFINE(HomingProprioceptiveApp, Idle, NoEventData)
{
  printStateTransition(prev_state_, ST_IDLE);
  prev_state_ = ST_IDLE;
  emit stateChanged(ST_IDLE);

  if (robot_ptr_->anyMotorEnabled())
    robot_ptr_->disableMotors();

  qmutex_.lock();
  disable_cmd_recv_ = false; // reset
  qmutex_.unlock();
}

GUARD_DEFINE(HomingProprioceptiveApp, GuardEnabled, NoEventData)
{
  robot_ptr_->setController(nullptr);
  robot_ptr_->enableMotors();

  grabrt::ThreadClock clock(grabrt::Sec2NanoSec(CableRobot::kCycleWaitTimeSec));
  while (1)
  {
    if (robot_ptr_->motorsEnabled())
      return true;
    if (clock.ElapsedFromStart() > CableRobot::kMaxWaitTimeSec)
      break;
    clock.WaitUntilNext();
  }
  emit printToQConsole("WARNING: Homing state transition FAILED. Taking too long to "
                       "enable drives.");
  return false;
}

STATE_DEFINE(HomingProprioceptiveApp, Enabled, NoEventData)
{
  printStateTransition(prev_state_, ST_ENABLED);
  prev_state_ = ST_ENABLED;
  emit stateChanged(ST_ENABLED);

  robot_ptr_->setMotorsOpMode(grabec::CYCLIC_TORQUE);
  qmutex_.lock();
  if (disable_cmd_recv_)
    InternalEvent(ST_IDLE);
  qmutex_.unlock();
}

STATE_DEFINE(HomingProprioceptiveApp, StartUp, HomingProprioceptiveStartData)
{
  printStateTransition(prev_state_, ST_START_UP);
  prev_state_ = ST_START_UP;

  QString msg("Start up phase complete\nRobot in predefined configuration\nInitial "
              "torque values:");

  working_actuator_idx_ = 0;
  num_meas_             = data->num_meas;
  num_tot_meas_         = (2 * num_meas_ - 1) * active_actuators_id_.size();
  init_torques_.clear();
  max_torques_ = data->max_torques;
#if HOMING_ACK
  positions_.resize(num_meas_);
#else
  torques_.resize(num_meas_);
  reg_pos_.resize(num_meas_);
#endif

  RetVal ret = RetVal::OK;
  robot_ptr_->setController(&controller_);
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
    ret = robot_ptr_->waitUntilTargetReached();
    if (ret != RetVal::OK)
      break;
    msg.append(QString("\n\t%1±%2 ‰").arg(init_torques_.back()).arg(kTorqueSsErrTol_));
  }
  if (ret != RetVal::OK || robot_ptr_->waitUntilPlatformSteady(-1.) != RetVal::OK)
  {
    emit printToQConsole("WARNING: Start up phase failed");
    InternalEvent(ST_ENABLED);
    return;
  }
  // At the beginning we don't know where we are, neither we care.
  // Just update encoder home position to be used as reference to compute deltas.
  robot_ptr_->updateHomeConfig(0.0, 0.0);

  // Flush previous data logs if any.
  robot_ptr_->flushDataLogs();

  emit printToQConsole(msg);
  emit stateChanged(ST_START_UP);
}

GUARD_DEFINE(HomingProprioceptiveApp, GuardSwitch, NoEventData)
{
  if (prev_state_ == ST_START_UP)
    return true;

  if (working_actuator_idx_ < active_actuators_id_.size())
    // We are not done ==> move to next cable
    return true;

  InternalEvent(ST_ENABLED);
  emit acquisitionComplete();
  return false;
}

STATE_DEFINE(HomingProprioceptiveApp, SwitchCable, NoEventData)
{
  printStateTransition(prev_state_, ST_SWITCH_CABLE);
  prev_state_ = ST_SWITCH_CABLE;

#if HOMING_ACK
  static constexpr double kCableStroke = -0.4; // [m] cable stroke starting from init pos
  const double kDeltaLen = kCableStroke / (num_meas_ - 1); // [m]
  const qint32 kDeltaPos =
    robot_ptr_->getActuator(active_actuators_id_[working_actuator_idx_])->getWinch()
      .lengthToCounts(kDeltaLen);
  const qint32 init_pos =
    robot_ptr_->getActuatorStatus(active_actuators_id_[working_actuator_idx_])
      .motor_position;
  // Compute sequence of position setpoints for i-th actuator, given the fact that cable
  for (quint8 i = 0; i < num_meas_; ++i)
    positions_[i] = init_pos + i * kDeltaPos;

  // Setup first setpoint of the sequence
  pthread_mutex_lock(&robot_ptr_->Mutex());
  controller_.SetMotorID(active_actuators_id_[working_actuator_idx_]);
  controller_.SetMode(ControlMode::MOTOR_POSITION);
  controller_.SetMotorPosTarget(positions_.front());
  pthread_mutex_unlock(&robot_ptr_->Mutex());

  emit printToQConsole(
    QString("Switched to actuator #%1.\nInitial position setpoint = %2")
      .arg(active_actuators_id_[working_actuator_idx_])
      .arg(positions_.front()));
#else
  // Compute sequence of torque setpoints for i-th actuator
  qint16 delta_torque =
    (max_torques_[working_actuator_idx_] - init_torques_[working_actuator_idx_]) /
    (static_cast<qint16>(num_meas_) - 1);
  for (quint8 i = 0; i < num_meas_ - 1; ++i)
    torques_[i] = init_torques_[working_actuator_idx_] + i * delta_torque;
  torques_.back() = max_torques_[working_actuator_idx_]; // last element = max torque

  // Setup first setpoint of the sequence
  pthread_mutex_lock(&robot_ptr_->Mutex());
  controller_.SetMotorID(active_actuators_id_[working_actuator_idx_]);
  controller_.SetMode(ControlMode::MOTOR_TORQUE);
  controller_.SetMotorTorqueTarget(torques_.front());
  pthread_mutex_unlock(&robot_ptr_->Mutex());

  emit printToQConsole(
    QString("Switched to actuator #%1.\nInitial torque setpoint = %2 ‰")
      .arg(active_actuators_id_[working_actuator_idx_])
      .arg(torques_.front()));
#endif
  meas_step_ = 0; // reset

  if (robot_ptr_->waitUntilTargetReached() == RetVal::OK &&
      robot_ptr_->waitUntilPlatformSteady(-1.) == RetVal::OK)
  {
    emit stateChanged(ST_SWITCH_CABLE);
    return;
  }
  InternalEvent(ST_ENABLED);
}

ENTRY_DEFINE(HomingProprioceptiveApp, EntryCoiling, NoEventData)
{
#if !HOMING_ACK
  // Record initial motor position for future uncoiling phase
  reg_pos_[0] = robot_ptr_->getActuatorStatus(active_actuators_id_[working_actuator_idx_])
                  .motor_position;
#endif

  dumpMeasAndMoveNext();
}

STATE_DEFINE(HomingProprioceptiveApp, Coiling, NoEventData)
{
  printStateTransition(prev_state_, ST_COILING);
  prev_state_ = ST_COILING;

  if (meas_step_ == num_meas_)
  {
    InternalEvent(ST_UNCOILING);
    return;
  }

#if HOMING_ACK
  pthread_mutex_lock(&robot_ptr_->Mutex());
  controller_.SetMotorPosTarget(positions_[meas_step_], true, kPositionStepTransTime_);
  pthread_mutex_unlock(&robot_ptr_->Mutex());
  emit printToQConsole(
    QString("Next position setpoint = %1").arg(positions_[meas_step_]));
#else
  pthread_mutex_lock(&robot_ptr_->Mutex());
  controller_.SetMotorTorqueTarget(torques_[meas_step_]);
  pthread_mutex_unlock(&robot_ptr_->Mutex());
  emit printToQConsole(QString("Next torque setpoint = %1 ‰").arg(torques_[meas_step_]));
#endif

  if (robot_ptr_->waitUntilTargetReached() == RetVal::OK &&
      robot_ptr_->waitUntilPlatformSteady(-1.) == RetVal::OK)
  {
#if !HOMING_ACK
    // Record motor position for future uncoiling phase
    reg_pos_[meas_step_] =
      robot_ptr_->getActuatorStatus(active_actuators_id_[working_actuator_idx_])
        .motor_position;
    emit printToQConsole(QString("Torque setpoint reached with motor position = %1")
                           .arg(reg_pos_[meas_step_]));
#endif

    dumpMeasAndMoveNext();
    emit stateChanged(ST_COILING);
    return;
  }
  InternalEvent(ST_ENABLED);
}

STATE_DEFINE(HomingProprioceptiveApp, Uncoiling, NoEventData)
{
  printStateTransition(prev_state_, ST_UNCOILING);
  prev_state_ = ST_UNCOILING;

  if (meas_step_ == (2 * num_meas_ - 1))
  {
    // At the end of uncoiling phase, move back to initial position...
    robot_ptr_->goHome();
    // ...and restore torque control for all cables before moving to next one
    RetVal ret = RetVal::OK;
    for (size_t i = 0; i < active_actuators_id_.size(); ++i)
    {
      // Setup initial target torque for each motor
      pthread_mutex_lock(&robot_ptr_->Mutex());
      controller_.SetMotorID(active_actuators_id_[i]);
      controller_.SetMode(ControlMode::MOTOR_TORQUE);
      controller_.SetMotorTorqueTarget(init_torques_[i]);
      pthread_mutex_unlock(&robot_ptr_->Mutex());
      // Wait until each motor reached user-given initial torque setpoint
      ret = robot_ptr_->waitUntilTargetReached();
      if (ret != RetVal::OK)
        break;
    }
    if (ret == RetVal::OK && robot_ptr_->waitUntilPlatformSteady(-1.) == RetVal::OK)
    {
      working_actuator_idx_++;
      InternalEvent(ST_SWITCH_CABLE);
    }
    else
      InternalEvent(ST_ENABLED);
    return;
  }

  const ulong kOffset = num_tot_meas_ / active_actuators_id_.size() - 1;
  // Uncoiling done in position control to return to previous steps. In torque control
  // this wouldn't happen due to friction.
  pthread_mutex_lock(&robot_ptr_->Mutex());
  controller_.SetMode(ControlMode::MOTOR_POSITION);
#if HOMING_ACK
  controller_.SetMotorPosTarget(positions_[kOffset - meas_step_], true,
                                kPositionStepTransTime_);
  pthread_mutex_unlock(&robot_ptr_->Mutex());
  emit printToQConsole(
    QString("Next position setpoint = %1").arg(positions_[kOffset - meas_step_]));
#else
  controller_.SetMotorPosTarget(reg_pos_[kOffset - meas_step_], true,
                                kPositionStepTransTime_);
  pthread_mutex_unlock(&robot_ptr_->Mutex());
  emit printToQConsole(
    QString("Next position setpoint = %1").arg(reg_pos_[kOffset - meas_step_]));
#endif

  if (robot_ptr_->waitUntilTargetReached() == RetVal::OK &&
      robot_ptr_->waitUntilPlatformSteady(-1.) == RetVal::OK)
  {
#if !HOMING_ACK
    int16_t actual_torque =
      robot_ptr_->getActuatorStatus(active_actuators_id_[working_actuator_idx_])
        .motor_torque;
    emit printToQConsole(
      QString("Position setpoint reached with torque = %1 ‰ (original was %2 ‰)")
        .arg(actual_torque)
        .arg(torques_[kOffset - meas_step_]));
#endif

    dumpMeasAndMoveNext();
    emit stateChanged(ST_UNCOILING);
    return;
  }
  InternalEvent(ST_ENABLED);
}

STATE_DEFINE(HomingProprioceptiveApp, Optimizing, NoEventData)
{
  printStateTransition(prev_state_, ST_OPTIMIZING);
  prev_state_ = ST_OPTIMIZING;
  emit stateChanged(ST_OPTIMIZING);
  // Run matlab optimization script from shell command
  const QString script_loc =
    QString(SRCDIR) +
    "libs/grab_common/libcdpr/cdpr_matlab/apps/homing/ExternalHomingScript.m";
  MatlabThread* matlab_thread = new MatlabThread(this, script_loc);
  connect(matlab_thread, SIGNAL(resultsReady()), this, SLOT(handleMatlabResultsReady()));
  connect(matlab_thread, SIGNAL(printToQConsole(QString)), this->parent(),
          SLOT(appendText2Browser(QString)));
  connect(matlab_thread, &MatlabThread::finished, matlab_thread, &QObject::deleteLater);
  matlab_thread->start();
  optimization_progress_counter_ = 0;
  optimization_progess_timer_.start(kOptProgressIntervalMsec_);
}

STATE_DEFINE(HomingProprioceptiveApp, Home, HomingProprioceptiveHomeData)
{
  printStateTransition(prev_state_, ST_HOME);
  prev_state_ = ST_HOME;
  emit stateChanged(ST_HOME);

  // Current home corresponds to robot configuration at the beginning of homing procedure.
  // Remind that motors home count corresponds to null cable length, which needs to be
  // updated...
  if (robot_ptr_->goHome()) // (position control)
  {
    // ...which is done here.
    robot_ptr_->updateHomeConfig(data->init_pose);
    grabcdpr::RobotVars robot_vars = robot_ptr_->getRobotVars();
    for (uint i = 0; i < active_actuators_id_.size(); i++)
      emit printToQConsole(QString("Homing results for drive #%1:\n\tcable length = %2 "
                                   "[m]\n\tpulley angle = %3 [deg]")
                             .arg(active_actuators_id_[i])
                             .arg(robot_vars.cables[i].length)
                             .arg(robot_vars.cables[i].swivel_ang * 180. / M_PI));
    emit homingComplete();
  }
  else
  {
    emit printToQConsole("WARNING: Something went unexpectedly wrong, please start over");
    InternalEvent(ST_ENABLED);
  }
}

STATE_DEFINE(HomingProprioceptiveApp, Fault, NoEventData)
{
  printStateTransition(prev_state_, ST_FAULT);
  prev_state_ = ST_FAULT;
  emit stateChanged(ST_FAULT);
}

//--------- Private functions --------------------------------------------------------//

void HomingProprioceptiveApp::dumpMeasAndMoveNext()
{
  robot_ptr_->collectAndDumpMeas();
  emit printToQConsole("Measurements collected and dumped onto log file");
  meas_step_++;
  double normalized_value = round(
    100. * (static_cast<double>(working_actuator_idx_) / active_actuators_id_.size() +
            static_cast<double>(meas_step_) / num_tot_meas_));
  emit progressValue(static_cast<int>(normalized_value));
}

bool HomingProprioceptiveApp::parseExtFile(const QString& filepath,
                                           HomingProprioceptiveHomeData* home_data)
{
  // Open file
  emit printToQConsole("Parsing file '" + filepath + "'...");
  std::ifstream ifile(filepath.toStdString());
  if (!ifile.is_open())
  {
    emit printToQConsole("ERROR: Could not open file '" + filepath + "'");
    return false;
  }

  // Parse JSON (generic) data
  json optimization_results;
  ifile >> optimization_results;
  ifile.close();

  // Fill home_data structure
  QString field;
  try
  {
    std::vector<double> init_pose = optimization_results["init_pose"];
    home_data->init_pose.Fill(init_pose);
  }
  catch (json::type_error)
  {
    emit printToQConsole("ERROR: Missing, invalid or incomplete optimization result: " +
                         field);
    return false;
  }
  return true;
}

void HomingProprioceptiveApp::printStateTransition(const States current_state,
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
