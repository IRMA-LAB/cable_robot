/**
 * @file cablerobot.cpp
 * @author Simone Comari, Edoardo Idà
 * @date 10 JuL 2019
 * @brief File containing definitions of functions and class declared in cablerobot.h.
 */

#include "robot/cablerobot.h"

//------------------------------------------------------------------------------------//
//--------- CableRobotLoopThread class -----------------------------------------------//
//------------------------------------------------------------------------------------//


CableRobotLoopThread::CableRobotLoopThread(QObject* parent, LockFreeMeasBuff* meas_buffer,
                                           LockFreeCtrlBuff* ctrl_actions_buffer,
                                           const vect<id_t>& active_actuators_id)
  : QThread(parent), meas_buffer_(meas_buffer), ctrl_actions_buffer_(ctrl_actions_buffer),
    stop_request_(false)
{
  for (uint i = 0; i < active_actuators_id.size(); i++)
  {
    grabcdpr::CableVars cable;
    robot_vars_.cables.push_back(cable);
  }

  active_actuators_status_.resize(active_actuators_id.size());
}

//--------- Public functions --------------------------------------------------------//

void CableRobotLoopThread::reset()
{
  mutex_.lock();
  if (state_estimator_ != nullptr)
  {
    delete state_estimator_;
    state_estimator_ = nullptr;
  }
  if (controller_ != nullptr)
  {
    delete controller_;
    controller_ = nullptr;
  }
  mutex_.unlock();
}

void CableRobotLoopThread::setController(ControllerBase* controller)
{
  mutex_.lock();
  controller_   = controller;
  stop_request_ = false;
  mutex_.unlock();
}

void CableRobotLoopThread::setStateEstimator(StateEstimatorBase* state_estimator)
{
  mutex_.lock();
  state_estimator_ = state_estimator;
  stop_request_    = false;
  mutex_.unlock();
}

void CableRobotLoopThread::initCdprStatus(const grabnum::VectorXd<POSE_DIM>& init_pose)
{
  mutex_.lock();
  robot_vars_.platform.SetPose(init_pose);
  mutex_.unlock();
}

//--------- Public Slots ------------------------------------------------------------//

void CableRobotLoopThread::stop()
{
  mutex_.lock();
  stop_request_    = true;
  controller_      = nullptr;
  state_estimator_ = nullptr;
  mutex_.unlock();
}

grabcdpr::RobotVars CableRobotLoopThread::getRobotVars()
{
  mutex_.lock();
  grabcdpr::RobotVars copy = robot_vars_;
  mutex_.unlock();
  return copy;
}

vect<ActuatorStatus> CableRobotLoopThread::getActiveActuatorsStatus()
{
  mutex_.lock();
  vect<ActuatorStatus> copy = active_actuators_status_;
  mutex_.unlock();
  return copy;
}

//--------- Private functions -------------------------------------------------------//

void CableRobotLoopThread::run()
{
  static CdprMeasurement meas;
  while (true)
  {
    // Read latest measurement from lock-free buffer
    if (!meas_buffer_->pop(meas))
      continue;

    mutex_.lock();
    if (stop_request_)
      break;

    for (uint i = 0; i < active_actuators_status_.size(); i++)
      active_actuators_status_[i] = meas[i].body;

    if (state_estimator_ != nullptr)
      state_estimator_->EstimatePlatformPose(active_actuators_status_, robot_vars_);

    if (controller_ != nullptr)
      controlStep(); // unlock mutex here
    else
      mutex_.unlock();
  }
}

void CableRobotLoopThread::controlStep()
{
  vect<ControlAction> ctrl_actions =
    controller_->calcCtrlActions(robot_vars_, active_actuators_status_);
  mutex_.unlock();

  static std::array<ControlAction, MAX_CABLES_NUM> full_ctrl_actions;
  for (uint i = 0; i < ctrl_actions.size(); i++)
    full_ctrl_actions[i] = ctrl_actions[i];

  // Push on lock-free buffer latest control actions
  static std::array<ControlAction, MAX_CABLES_NUM> trash;
  while (!ctrl_actions_buffer_->push(full_ctrl_actions))
    ctrl_actions_buffer_->pop(trash);
}

//------------------------------------------------------------------------------------//
//--------- CableRobot class ---------------------------------------------------------//
//------------------------------------------------------------------------------------//

constexpr double CableRobot::kMaxWaitTimeSec;
constexpr double CableRobot::kCycleWaitTimeSec;
constexpr char* CableRobot::kStatesStr_[];
constexpr double CableRobot::kCutoffFreq_;

CableRobot::CableRobot(QObject* parent, const grabcdpr::RobotParams& config)
  : QObject(parent), StateMachine(ST_MAX_STATES), params_(config),
    log_buffer_(el::Loggers::getLogger("data")), stop_waiting_cmd_recv_(false),
    is_waiting_(false), prev_state_(ST_MAX_STATES)
{
  printStateTransition(prev_state_, ST_IDLE);
  prev_state_ = ST_IDLE;

  // Setup EtherCAT network
  max_shutdown_wait_time_sec_ = 3.0; // [sec]
  quint8 slave_pos            = 0;
#if INCLUDE_EASYCAT
  easycat1_ptr_ = new grabec::TestEasyCAT1Slave(slave_pos++);
  slaves_ptrs_.push_back(easycat1_ptr_);
  easycat2_ptr_ = new grabec::TestEasyCAT2Slave(slave_pos++);
  slaves_ptrs_.push_back(easycat2_ptr_);
#endif
  for (uint i = 0; i < config.actuators.size(); i++)
  {
    rt_actuators_ptrs_.push_back(new Actuator(i, slave_pos++, config.actuators[i], this));
    slaves_ptrs_.push_back(rt_actuators_ptrs_[i]->GetWinch().GetServo());
    if (config.actuators[i].active)
    {
      active_actuators_id_.push_back(i);
      rt_active_actuators_ptrs_.push_back(rt_actuators_ptrs_[i]);
      connect(rt_actuators_ptrs_[i], SIGNAL(printToQConsole(QString)), this,
              SLOT(forwardPrintToQConsole(QString)));
    }
  }
  num_slaves_ = slaves_ptrs_.size();
  for (grabec::EthercatSlave* slave_ptr : slaves_ptrs_)
    num_domain_elements_ += slave_ptr->GetDomainEntriesNum();

  // Setup non-RT loop thread, where control and state estimation loops live
  loop_thread_ = new CableRobotLoopThread(this, &meas_buffer_, &ctrl_actions_buffer_,
                                          active_actuators_id_);
  loop_thread_->start(QThread::Priority::HighestPriority);

  // Setup data logging
  rt_logging_enabled_ = false;
  rt_logging_mod_     = 1;
  connect(this, SIGNAL(sendMsg(QByteArray)), &log_buffer_, SLOT(collectMsg(QByteArray)),
          Qt::QueuedConnection);
  log_buffer_.start();

  // Setup timers for components' status update
  motor_status_timer_ = new QTimer(this);
  connect(motor_status_timer_, SIGNAL(timeout()), this, SLOT(emitMotorStatus()));
  actuator_status_timer_ = new QTimer(this);
  connect(actuator_status_timer_, SIGNAL(timeout()), this, SLOT(emitActuatorStatus()));
}

CableRobot::~CableRobot()
{
  // Stop non-RT loop (state est. + control)
  loop_thread_->stop();
  loop_thread_->exit();

  // Close data logging
  log_buffer_.stop();
  disconnect(this, SIGNAL(sendMsg(QByteArray)), &log_buffer_,
             SLOT(collectMsg(QByteArray)));

  // Close timers for components' status update
  stopTimers();
  disconnect(motor_status_timer_, SIGNAL(timeout()), this, SLOT(emitMotorStatus()));
  disconnect(actuator_status_timer_, SIGNAL(timeout()), this, SLOT(emitActuatorStatus()));
  delete motor_status_timer_;
  delete actuator_status_timer_;

  // Stop RT thread before removing slaves
  thread_rt_.Stop();

  // Delete robot components (i.e. ethercat slaves)
#if INCLUDE_EASYCAT
  delete easycat1_ptr_;
  delete easycat2_ptr_;
#endif
  for (Actuator* actuator_ptr : rt_actuators_ptrs_)
  {
    if (actuator_ptr->IsActive())
      disconnect(actuator_ptr, SIGNAL(printToQConsole(QString)), this,
                 SLOT(forwardPrintToQConsole(QString)));
    delete actuator_ptr;
  }
}

//--------- Public Functions --------------------------------------------------------//

grabcdpr::RobotParams CableRobot::getActiveComponentsParams() const
{
  grabcdpr::RobotParams params;
  params.platform = params_.platform;
  for (const auto& actuator_params : params_.actuators)
    if (actuator_params.active)
      params.actuators.push_back(actuator_params);
  return params;
}

const Actuator* CableRobot::getActuator(const id_t motor_id)
{
  return rt_actuators_ptrs_[motor_id];
}

ActuatorStatus CableRobot::getActuatorStatus(const id_t motor_id)
{
  vect<ActuatorStatus> active_actuators_status = getActuatorsStatus();

  for (const ActuatorStatus& actuator_status : active_actuators_status)
    if (actuator_status.id == motor_id)
      return actuator_status;
  // If not found
  throw std::invalid_argument("invalid motor ID");
}

vect<ActuatorStatus> CableRobot::getActuatorsStatus()
{
  return loop_thread_->getActiveActuatorsStatus();
}

void CableRobot::updateHomeConfig(const double cable_len, const double pulley_angle)
{
  pthread_mutex_lock(&mutex_);
  for (Actuator* actuator_ptr : rt_active_actuators_ptrs_)
    actuator_ptr->UpdateHomeConfig(cable_len, pulley_angle);
  pthread_mutex_unlock(&mutex_);
}

void CableRobot::updateHomeConfig(const id_t motor_id, const double cable_len,
                                  const double pulley_angle)
{
  pthread_mutex_lock(&mutex_);
  rt_actuators_ptrs_[motor_id]->UpdateHomeConfig(cable_len, pulley_angle);
  pthread_mutex_unlock(&mutex_);
}

void CableRobot::setPlatformPose(const grabnum::VectorXd<POSE_DIM>& pose)
{
  loop_thread_->initCdprStatus(pose);
}

bool CableRobot::motorEnabled(const id_t motor_id)
{
  return rt_actuators_ptrs_[motor_id]->IsEnabled();
}

bool CableRobot::anyMotorEnabled()
{
  for (Actuator* actuator_ptr : rt_active_actuators_ptrs_)
    if (actuator_ptr->IsEnabled())
      return true;
  return false;
}

bool CableRobot::motorsEnabled()
{
  for (Actuator* actuator_ptr : rt_active_actuators_ptrs_)
    if (!actuator_ptr->IsEnabled())
      return false;
  return true;
}

void CableRobot::enableMotor(const id_t motor_id)
{
  if (rt_actuators_ptrs_[motor_id]->IsActive())
    rt_actuators_ptrs_[motor_id]->enable();
}

void CableRobot::enableMotors()
{
  for (Actuator* actuator_ptr : rt_active_actuators_ptrs_)
    if (!actuator_ptr->IsEnabled())
      actuator_ptr->enable();
}

void CableRobot::enableMotors(const vect<id_t>& motors_id)
{
  for (const id_t& motor_id : motors_id)
    if (rt_actuators_ptrs_[motor_id]->IsActive())
      rt_actuators_ptrs_[motor_id]->enable();
}

void CableRobot::disableMotor(const id_t motor_id)
{
  if (rt_actuators_ptrs_[motor_id]->IsActive())
    rt_actuators_ptrs_[motor_id]->disable();
}

void CableRobot::disableMotors()
{
  for (Actuator* actuator_ptr : rt_active_actuators_ptrs_)
    actuator_ptr->disable();
}

void CableRobot::disableMotors(const vect<id_t>& motors_id)
{
  for (const id_t& motor_id : motors_id)
    if (rt_actuators_ptrs_[motor_id]->IsActive())
      rt_actuators_ptrs_[motor_id]->disable();
}

void CableRobot::clearFaults()
{
  for (Actuator* actuator_ptr : rt_active_actuators_ptrs_)
    if (actuator_ptr->IsInFault())
      actuator_ptr->faultReset();
}

CdprMeasurement CableRobot::collectMeas()
{
  CdprMeasurement meas;
  while (!meas_buffer_.pop(meas))
    continue;
  return meas;
}

void CableRobot::collectAndDumpMeas()
{
  CdprMeasurement meas = collectMeas();
  for (size_t i = 0; i < active_actuators_id_.size(); i++)
    emit sendMsg(meas[i].serialized());
}

void CableRobot::collectAndDumpMeas(const id_t actuator_id)
{
  CdprMeasurement meas = collectMeas();
  for (size_t i = 0; i < active_actuators_id_.size(); i++)
    if (actuator_id == active_actuators_id_[i])
    {
      emit sendMsg(meas[i].serialized());
      break;
    }
}

void CableRobot::startRtLogging(const uint rt_cycle_multiplier)
{
  pthread_mutex_lock(&mutex_);
  rt_logging_enabled_ = true;
  rt_logging_mod_     = rt_cycle_multiplier;
  pthread_mutex_unlock(&mutex_);
}

void CableRobot::stopRtLogging()
{
  pthread_mutex_lock(&mutex_);
  rt_logging_enabled_ = false;
  pthread_mutex_unlock(&mutex_);
}

void CableRobot::flushDataLogs()
{
  log_buffer_.flush();
  CLOG(INFO, "event") << "Data logs flushed";
}

bool CableRobot::goHome()
{
  if (!motorsEnabled())
  {
    emit printToQConsole("WARNING: Cannot move to home position: not all motors enabled");
    return false;
  }
  emit printToQConsole("Moving to home position...");

  ControllerSingleDrive controller(GetRtCycleTimeNsec());
  // Temporarly switch to local controller for moving to home pos
  ControllerBase* prev_controller = loop_thread_->getController();
  loop_thread_->setController(&controller);

  for (Actuator* actuator_ptr : rt_active_actuators_ptrs_)
  {
    loop_thread_->lockMutex();
    controller.SetMotorID(actuator_ptr->ID());
    controller.SetMode(ControlMode::MOTOR_POSITION);
    controller.SetMotorPosTarget(actuator_ptr->GetWinch().GetServoHomePos(), true, 3.0);
    loop_thread_->unlockMutex();

    if (waitUntilTargetReached() != RetVal::OK)
    {
      emit printToQConsole("WARNING: Transition to home position interrupted");
      return false;
    }
  }
  loop_thread_->setController(prev_controller); // restore original controller

  emit printToQConsole("Daddy, I'm home!");
  return true;
}

void CableRobot::setController(ControllerBase* controller)
{
  loop_thread_->setController(controller);
}

RetVal CableRobot::waitUntilTargetReached(const double max_wait_time_sec)
{
  qmutex_.lock();
  is_waiting_ = true;
  qmutex_.unlock();
  grabrt::ThreadClock clock(grabrt::Sec2NanoSec(kCycleWaitTimeSec));
  while (1)
  {
    // Check if target is reached
    loop_thread_->lockMutex();
    if (loop_thread_->getController()->targetReached())
    {
      loop_thread_->unlockMutex();
      qmutex_.lock();
      is_waiting_ = false;
      qmutex_.unlock();
      return RetVal::OK;
    }
    loop_thread_->unlockMutex();
    // Check if external abort signal is received
    if (isExtAbortSigRecv("waiting for platform to reach target"))
      return RetVal::EINT;
    // Check if timeout expired (safety feature to prevent hanging in forever)
    if (isTimeoutExpired(max_wait_time_sec, clock, "reach target"))
      return RetVal::ETIMEOUT;
    clock.WaitUntilNext();
  }
}

void CableRobot::setStateEstimator(StateEstimatorBase* state_estimator)
{
  loop_thread_->setStateEstimator(state_estimator);
}

RetVal CableRobot::waitUntilPlatformSteady(const double max_wait_time_sec)
{
  // Compute these once for all
  static constexpr size_t kBuffSize =
    static_cast<size_t>(kBufferingTimeSec_ / kCycleWaitTimeSec);
  // LP filters setup
  static std::vector<grabnum::LowPassFilter> lp_filters(
    active_actuators_id_.size(), grabnum::LowPassFilter(kCutoffFreq_, kCycleWaitTimeSec));
  for (size_t i = 0; i < active_actuators_id_.size(); i++)
    lp_filters[i].Reset();

  // Init
  qmutex_.lock();
  is_waiting_ = true;
  qmutex_.unlock();
  bool swinging = true;
  std::vector<RingBufferD> pulleys_angles(active_actuators_id_.size(),
                                          RingBufferD(kBuffSize));
  grabrt::ThreadClock clock(grabrt::Sec2NanoSec(kCycleWaitTimeSec));
  // Start waiting
  while (swinging)
  {
    // Analyze platform status
    vect<ActuatorStatus> actuators_status = loop_thread_->getActiveActuatorsStatus();
    for (size_t i = 0; i < active_actuators_id_.size(); i++)
    {
      // Add filtered angle
      double current_pulley_angle = actuators_status[i].pulley_angle;
      pulleys_angles[i].Add(lp_filters[i].Filter(current_pulley_angle));
      if (!pulleys_angles[i].IsFull()) // wait at least until buffer is full
        continue;
      // Condition to detect steadyness
      swinging = grabnum::Std(pulleys_angles[i].Data()) > kMaxAngleDeviation_;
      if (swinging)
        break;
    }
    // Check if external abort signal is received
    if (isExtAbortSigRecv("waiting for platform to be steady"))
      return RetVal::EINT;
    // Check if timeout expired (safety feature to prevent hanging in forever)
    if (isTimeoutExpired(max_wait_time_sec, clock, "stabilize"))
      return RetVal::ETIMEOUT;
    clock.WaitUntilNext();
  }
  qmutex_.lock();
  is_waiting_ = false;
  qmutex_.unlock();
  return RetVal::OK;
}

bool CableRobot::isWaiting()
{
  qmutex_.lock();
  bool is_waiting = is_waiting_;
  qmutex_.unlock();
  return is_waiting;
}

//--------- External Events (Public slots) ------------------------------------------//

void CableRobot::stopWaiting()
{
  qmutex_.lock();
  stop_waiting_cmd_recv_ = true;
  qmutex_.unlock();
}

void CableRobot::enterCalibrationMode()
{
  CLOG(TRACE, "event");
  // clang-format off
  BEGIN_TRANSITION_MAP			            // - Current State -
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)          // ST_IDLE
      TRANSITION_MAP_ENTRY (ST_CALIBRATION)         // ST_ENABLED
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)          // ST_CALIBRATION
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)          // ST_HOMING
      TRANSITION_MAP_ENTRY (ST_CALIBRATION)         // ST_READY
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)          // ST_OPERATIONAL
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)          // ST_ERROR
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

void CableRobot::enterHomingMode()
{
  CLOG(TRACE, "event");
  // clang-format off
  BEGIN_TRANSITION_MAP			            // - Current State -
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)          // ST_IDLE
      TRANSITION_MAP_ENTRY (ST_HOMING)              // ST_ENABLED
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)          // ST_CALIBRATION
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)          // ST_HOMING
      TRANSITION_MAP_ENTRY (ST_HOMING)              // ST_READY
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)          // ST_OPERATIONAL
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)          // ST_ERROR
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

void CableRobot::eventSuccess()
{
  CLOG(TRACE, "event");
  // clang-format off
  BEGIN_TRANSITION_MAP                              // - Current State -
      TRANSITION_MAP_ENTRY (ST_ENABLED)             // ST_IDLE
      TRANSITION_MAP_ENTRY (ST_OPERATIONAL)         // ST_ENABLED
      TRANSITION_MAP_ENTRY (ST_ENABLED)             // ST_CALIBRATION
      TRANSITION_MAP_ENTRY (ST_READY)               // ST_HOMING
      TRANSITION_MAP_ENTRY (ST_OPERATIONAL)         // ST_READY
      TRANSITION_MAP_ENTRY (ST_READY)               // ST_OPERATIONAL
      TRANSITION_MAP_ENTRY (ST_ENABLED)             // ST_ERROR
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

void CableRobot::eventFailure()
{
  CLOG(TRACE, "event");
  // clang-format off
  BEGIN_TRANSITION_MAP			            // - Current State -
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)	    // ST_IDLE
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)          // ST_ENABLED
      TRANSITION_MAP_ENTRY (ST_ENABLED)             // ST_CALIBRATION
      TRANSITION_MAP_ENTRY (ST_ENABLED)             // ST_HOMING
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)          // ST_READY
      TRANSITION_MAP_ENTRY (ST_ERROR)               // ST_OPERATIONAL
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)          // ST_ERROR
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

void CableRobot::stop()
{
  CLOG(TRACE, "event");
  // clang-format off
  BEGIN_TRANSITION_MAP                              // - Current State -
      TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)          // ST_IDLE
      TRANSITION_MAP_ENTRY (EVENT_IGNORED)          // ST_ENABLED
      TRANSITION_MAP_ENTRY (ST_ENABLED)             // ST_CALIBRATION
      TRANSITION_MAP_ENTRY (ST_ENABLED)             // ST_HOMING
      TRANSITION_MAP_ENTRY (ST_ENABLED)             // ST_READY
      TRANSITION_MAP_ENTRY (ST_READY)               // ST_OPERATIONAL
      TRANSITION_MAP_ENTRY (ST_ENABLED)             // ST_ERROR
  END_TRANSITION_MAP(NULL)
  // clang-format on
}

//--------- States Actions Private --------------------------------------------------//

STATE_DEFINE(CableRobot, Idle, NoEventData)
{
  printStateTransition(prev_state_, ST_IDLE);
  prev_state_ = ST_IDLE;
}

STATE_DEFINE(CableRobot, Enabled, NoEventData)
{
  printStateTransition(prev_state_, ST_ENABLED);
  prev_state_ = ST_ENABLED;

  loop_thread_->reset();

  actuator_status_timer_->start(kGuiTimerIntervalMsec_);
}

STATE_DEFINE(CableRobot, Calibration, NoEventData)
{
  printStateTransition(prev_state_, ST_CALIBRATION);
  prev_state_ = ST_CALIBRATION;

  loop_thread_->reset();
}

STATE_DEFINE(CableRobot, Homing, NoEventData)
{
  printStateTransition(prev_state_, ST_HOMING);
  prev_state_ = ST_HOMING;

  loop_thread_->reset();

  stopTimers();
}

STATE_DEFINE(CableRobot, Ready, NoEventData)
{
  printStateTransition(prev_state_, ST_READY);
  prev_state_ = ST_READY;

  loop_thread_->reset();
  loop_thread_->setStateEstimator(new StateEstimatorBase(params_));

  actuator_status_timer_->start(kGuiTimerIntervalMsec_);
}

STATE_DEFINE(CableRobot, Operational, NoEventData)
{
  printStateTransition(prev_state_, ST_OPERATIONAL);
  prev_state_ = ST_OPERATIONAL;

  stopTimers();
}

STATE_DEFINE(CableRobot, Error, NoEventData)
{
  printStateTransition(prev_state_, ST_ERROR);
  prev_state_ = ST_ERROR;

  loop_thread_->reset();

  stopTimers();
}

//--------- Private slots -----------------------------------------------------------//

void CableRobot::forwardPrintToQConsole(const QString& text) const
{
  emit printToQConsole(text);
}

void CableRobot::emitMotorStatus()
{
  if (!(ec_network_valid_ && rt_thread_active_))
    return;

  vect<ActuatorStatus> actuators_status = loop_thread_->getActiveActuatorsStatus();
  for (const ActuatorStatus& actuator_status : actuators_status)
  {
    MotorStatus motor_status(actuator_status);
    emit motorStatus(motor_status);
  }
}

void CableRobot::emitActuatorStatus()
{
  if (!(ec_network_valid_ && rt_thread_active_))
    return;

  vect<ActuatorStatus> actuators_status = loop_thread_->getActiveActuatorsStatus();
  for (const ActuatorStatus& actuator_status : actuators_status)
    emit actuatorStatus(actuator_status);
}

//--------- Miscellaneous private ---------------------------------------------------//

void CableRobot::printStateTransition(const States current_state,
                                      const States new_state) const
{
  if (current_state == new_state)
    return;
  QString msg;
  if (current_state != ST_MAX_STATES)
    msg = QString("CableRobot state transition: %1 --> %2")
            .arg(kStatesStr_[current_state], kStatesStr_[new_state]);
  else
    msg = QString("CableRobot initial state: %1").arg(kStatesStr_[new_state]);
  emit printToQConsole(msg);
}

void CableRobot::stopTimers()
{
  motor_status_timer_->stop();
  actuator_status_timer_->stop();
}

bool CableRobot::isExtAbortSigRecv(const QString& operation_in_progress /*=""*/)
{
  QCoreApplication::processEvents();
  qmutex_.lock();
  if (stop_waiting_cmd_recv_)
  {
    is_waiting_            = false;
    stop_waiting_cmd_recv_ = false;
    qmutex_.unlock();
    QString addendum =
      operation_in_progress.isEmpty() ? "" : " while " + operation_in_progress;
    CLOG(INFO, "event") << "Stop waiting command received" + addendum;
    return true;
  }
  qmutex_.unlock();
  return false;
}

bool CableRobot::isTimeoutExpired(const double max_wait_time_sec,
                                  const grabrt::Clock& clock,
                                  const QString& operation_in_progress)
{
  if (max_wait_time_sec > 0 && clock.ElapsedFromStart() > max_wait_time_sec)
  {
    qmutex_.lock();
    is_waiting_ = false;
    qmutex_.unlock();
    emit printToQConsole(
      QString("WARNING: Platform is taking too long to %1: operation aborted")
        .arg(operation_in_progress));
    return true;
  }
  return false;
}

//--------- Ethercat related private functions --------------------------------------//

void CableRobot::EcStateChangedCb(const std::bitset<3>& new_state)
{
  ec_network_valid_ = new_state.all();
  emit ecStateChanged(new_state);
}

void CableRobot::EcPrintCb(const std::string& msg, const char color /* = 'w' */) const
{
  switch (color)
  {
    case 'r':
      if (msg.find("ERROR:") == std::string::npos)
        emit printToQConsole(QString("ERROR: %1").arg(msg.c_str()));
      else
        emit printToQConsole(msg.c_str());
      break;
    case 'y':
      if (msg.find("WARNING:") == std::string::npos)
        emit printToQConsole(QString("WARNING: %1").arg(msg.c_str()));
      else
        emit printToQConsole(msg.c_str());
      break;
    default:
      emit printToQConsole(msg.c_str());
      break;
  }
}

void CableRobot::EcRtThreadStatusChanged(const bool active)
{
  rt_thread_active_ = active;
  emit rtThreadStatusChanged(active);
}

void CableRobot::EcWorkFun()
{
  for (grabec::EthercatSlave* slave_ptr : slaves_ptrs_)
    slave_ptr->ReadInputs(); // read pdos

  rtCollectMeas();
  // Push on lock-free buffer latest measurements
  static std::array<ActuatorStatusMsg, 8> trash;
  while (!meas_buffer_.push(rt_meas_))
    meas_buffer_.pop(trash);

  // Pop from lock-free buffer latest control actions, if any
  ctrl_actions_buffer_.pop(rt_ctrl_actions_);
  rtControlStep(); // apply

  for (grabec::EthercatSlave* slave_ptr : slaves_ptrs_)
    slave_ptr->WriteOutputs(); // write all the necessary pdos
}

void CableRobot::EcEmergencyFun() {}

//--------- RT cyclic steps related private functions -------------------------------//

void CableRobot::rtCollectMeas()
{
  for (size_t i = 0; i < rt_active_actuators_ptrs_.size(); i++)
  {
    rt_meas_[i].body             = rt_active_actuators_ptrs_[i]->GetStatus();
    rt_meas_[i].header.timestamp = clock_.Elapsed();
  }
}

void CableRobot::rtCollectAndDumpMeas()
{
  for (size_t i = 0; i < rt_active_actuators_ptrs_.size(); i++)
  {
    rt_meas_[i].body             = rt_active_actuators_ptrs_[i]->GetStatus();
    rt_meas_[i].header.timestamp = clock_.Elapsed();
    emit sendMsg(rt_meas_[i].serialized());
  }
}

void CableRobot::rtControlStep()
{
  for (const ControlAction& ctrl_action : rt_ctrl_actions_)
  {
    // Safety check to see if given motor id is valid
    bool valid_id = false;
    for (const id_t& actuator_id : active_actuators_id_)
      if (ctrl_action.motor_id == actuator_id)
      {
        valid_id = true;
        break;
      }
    if (!valid_id)
      continue;

    if (!rt_actuators_ptrs_[ctrl_action.motor_id]->IsEnabled()) // safety check
      continue;

    switch (ctrl_action.ctrl_mode)
    {
      case CABLE_LENGTH:
        rt_actuators_ptrs_[ctrl_action.motor_id]->SetCableLength(
          ctrl_action.cable_length);
        break;
      case MOTOR_POSITION:
        rt_actuators_ptrs_[ctrl_action.motor_id]->SetMotorPos(ctrl_action.motor_position);
        break;
      case MOTOR_SPEED:
        rt_actuators_ptrs_[ctrl_action.motor_id]->SetMotorSpeed(ctrl_action.motor_speed);
        break;
      case MOTOR_TORQUE:
        rt_actuators_ptrs_[ctrl_action.motor_id]->SetMotorTorque(
          ctrl_action.motor_torque);
        break;
      case NONE:
        break;
    }
  }
}
