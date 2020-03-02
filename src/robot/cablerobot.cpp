/**
 * @file cablerobot.cpp
 * @author Simone Comari, Edoardo Idà
 * @date 02 Mar 2020
 * @brief File containing definitions of functions and class declared in cablerobot.h.
 */

#include "robot/cablerobot.h"

constexpr double CableRobot::kMaxWaitTimeSec;
constexpr double CableRobot::kCycleWaitTimeSec;
constexpr char* CableRobot::kStatesStr_[];
constexpr double CableRobot::kCutoffFreq_;

CableRobot::CableRobot(QObject* parent, const grabcdpr::RobotParams& params)
  : QObject(parent), StateMachine(ST_MAX_STATES), platform_(grabcdpr::TILT_TORSION),
    params_(params), log_buffer_(el::Loggers::getLogger("data")),
    stop_waiting_cmd_recv_(false), is_waiting_(false), prev_state_(ST_MAX_STATES)
{
  printStateTransition(prev_state_, ST_IDLE);
  prev_state_ = ST_IDLE;

  // Setup robot model
  params_active_        = params_.getOnlyActiveComponents();
  cdpr_status_.platform = platform_;
  cdpr_status_.cables.resize(params_.activeActuatorsNum());

  // Setup EtherCAT network
  max_shutdown_wait_time_sec_ = 3.0; // [sec]
  quint8 slave_pos            = 0;
#if INCLUDE_EASYCAT
  easycat1_ptr_ = new grabec::TestEasyCAT1Slave(slave_pos++);
  slaves_ptrs_.push_back(easycat1_ptr_);
  easycat2_ptr_ = new grabec::TestEasyCAT2Slave(slave_pos++);
  slaves_ptrs_.push_back(easycat2_ptr_);
#endif
  for (uint i = 0; i < params.actuators.size(); i++)
  {
    actuators_ptrs_.push_back(new Actuator(i, slave_pos++, params.actuators[i], this));
    slaves_ptrs_.push_back(actuators_ptrs_[i]->getWinch().getServo());
    if (params.actuators[i].active)
    {
      active_actuators_id_.push_back(i);
      active_actuators_ptrs_.push_back(actuators_ptrs_[i]);
      connect(actuators_ptrs_[i], SIGNAL(printToQConsole(QString)), this,
              SLOT(forwardPrintToQConsole(QString)));
    }
  }
  num_slaves_ = slaves_ptrs_.size();
  for (grabec::EthercatSlave* slave_ptr : slaves_ptrs_)
    num_domain_elements_ += slave_ptr->GetDomainEntriesNum();

  // Setup data logging
  rt_logging_enabled_ = false;
  rt_logging_mod_     = 1;
  meas_.resize(active_actuators_id_.size());
  connect(this, SIGNAL(sendMsg(QByteArray)), &log_buffer_, SLOT(collectMsg(QByteArray)),
          Qt::QueuedConnection);
  log_buffer_.start();

  // Setup timers for components' status update
  motor_status_timer_ = new QTimer(this);
  connect(motor_status_timer_, SIGNAL(timeout()), this, SLOT(emitMotorStatus()));
  active_actuators_status_.resize(active_actuators_id_.size());
  actuator_status_timer_ = new QTimer(this);
  connect(actuator_status_timer_, SIGNAL(timeout()), this, SLOT(emitActuatorStatus()));
}

CableRobot::~CableRobot()
{
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
  for (Actuator* actuator_ptr : actuators_ptrs_)
  {
    if (actuator_ptr->isActive())
      disconnect(actuator_ptr, SIGNAL(printToQConsole(QString)), this,
                 SLOT(forwardPrintToQConsole(QString)));
    delete actuator_ptr;
  }
}

//--------- Public Functions --------------------------------------------------------//

const grabcdpr::RobotParams& CableRobot::getActiveComponentsParams() const
{
  return params_active_;
}

const Actuator* CableRobot::getActuator(const id_t motor_id)
{
  return actuators_ptrs_[motor_id];
}

const ActuatorStatus CableRobot::getActuatorStatus(const id_t motor_id)
{
  pthread_mutex_lock(&mutex_);
  ActuatorStatus status = actuators_ptrs_[motor_id]->getStatus();
  pthread_mutex_unlock(&mutex_);
  return status;
}

void CableRobot::updateHomeConfig(const double cable_len, const double pulley_angle)
{
  for (Actuator* actuator_ptr : active_actuators_ptrs_)
    actuator_ptr->updateHomeConfig(cable_len, pulley_angle);
}

void CableRobot::updateHomeConfig(const id_t motor_id, const double cable_len,
                                  const double pulley_angle)
{
  actuators_ptrs_[motor_id]->updateHomeConfig(cable_len, pulley_angle);
}

void CableRobot::updateHomeConfig(const grabnum::Vector6d& home_pose)
{
  grabcdpr::updateIK0(home_pose, params_active_, cdpr_status_);
  std::vector<id_t> active_actuators_id = params_.activeActuatorsId();
  for (uint8_t i = 0; i < active_actuators_id.size(); ++i)
    updateHomeConfig(active_actuators_id[i], cdpr_status_.cables[i].length,
                     cdpr_status_.cables[i].swivel_ang);
}

void CableRobot::updateActiveActuatorsStatus()
{
  if (!active_actuators_status_updated_)
  {
    for (size_t i = 0; i < active_actuators_status_.size(); i++)
      active_actuators_status_[i] = active_actuators_ptrs_[i]->getStatus();
    active_actuators_status_updated_ = true;
  }
}

bool CableRobot::motorEnabled(const id_t motor_id)
{
  return actuators_ptrs_[motor_id]->isEnabled();
}

bool CableRobot::anyMotorEnabled()
{
  for (Actuator* actuator_ptr : active_actuators_ptrs_)
    if (actuator_ptr->isEnabled())
      return true;
  return false;
}

bool CableRobot::motorsEnabled()
{
  for (Actuator* actuator_ptr : active_actuators_ptrs_)
    if (!actuator_ptr->isEnabled())
      return false;
  return true;
}

void CableRobot::enableMotor(const id_t motor_id)
{
  if (actuators_ptrs_[motor_id]->isActive())
    actuators_ptrs_[motor_id]->enable();
}

void CableRobot::enableMotors()
{
  for (Actuator* actuator_ptr : active_actuators_ptrs_)
    if (!actuator_ptr->isEnabled())
      actuator_ptr->enable();
}

void CableRobot::enableMotors(const vect<id_t>& motors_id)
{
  for (const id_t& motor_id : motors_id)
    if (actuators_ptrs_[motor_id]->isActive())
      actuators_ptrs_[motor_id]->enable();
}

void CableRobot::disableMotor(const id_t motor_id)
{
  if (actuators_ptrs_[motor_id]->isActive())
    actuators_ptrs_[motor_id]->disable();
}

void CableRobot::disableMotors()
{
  for (Actuator* actuator_ptr : active_actuators_ptrs_)
  {
    actuator_ptr->disable();
  }
}

void CableRobot::disableMotors(const vect<id_t>& motors_id)
{
  for (const id_t& motor_id : motors_id)
    if (actuators_ptrs_[motor_id]->isActive())
    {
      actuators_ptrs_[motor_id]->disable();
    }
}

void CableRobot::setMotorOpMode(const id_t motor_id, const qint8 op_mode)
{
  actuators_ptrs_[motor_id]->setMotorOpMode(op_mode);
}

void CableRobot::setMotorsOpMode(const qint8 op_mode)
{
  for (Actuator* actuator_ptr : active_actuators_ptrs_)
    actuator_ptr->setMotorOpMode(op_mode);
}

void CableRobot::setMotorsOpMode(const vect<id_t>& motors_id, const qint8 op_mode)
{
  for (const id_t& motor_id : motors_id)
    actuators_ptrs_[motor_id]->setMotorOpMode(op_mode);
}

void CableRobot::clearFaults()
{
  for (Actuator* actuator_ptr : active_actuators_ptrs_)
    if (actuator_ptr->isInFault())
      actuator_ptr->faultReset();
}

void CableRobot::collectMeasRt()
{
  for (size_t i = 0; i < active_actuators_ptrs_.size(); i++)
  {
    meas_[i].body             = active_actuators_ptrs_[i]->getStatus();
    meas_[i].header.timestamp = clock_.Elapsed();
  }
}

void CableRobot::collectMeas()
{
  pthread_mutex_lock(&mutex_);
  collectMeasRt();
  pthread_mutex_unlock(&mutex_);
}

void CableRobot::dumpMeas() const
{
  for (size_t i = 0; i < active_actuators_ptrs_.size(); i++)
    emit sendMsg(meas_[i].serialized());
}

void CableRobot::collectAndDumpMeasRt()
{
  if (active_actuators_status_updated_)
    for (size_t i = 0; i < active_actuators_status_.size(); i++)
    {
      meas_[i].body             = active_actuators_status_[i];
      meas_[i].header.timestamp = clock_.Elapsed();
      emit sendMsg(meas_[i].serialized());
    }
  else
    for (size_t i = 0; i < active_actuators_ptrs_.size(); i++)
    {
      meas_[i].body             = active_actuators_ptrs_[i]->getStatus();
      meas_[i].header.timestamp = clock_.Elapsed();
      emit sendMsg(meas_[i].serialized());
    }
}

void CableRobot::collectAndDumpMeas()
{
  collectMeas();
  dumpMeas();
}

void CableRobot::collectAndDumpMeas(const id_t actuator_id)
{
  for (size_t i = 0; i < active_actuators_id_.size(); i++)
  {
    if (actuator_id != active_actuators_id_[i])
      continue;
    pthread_mutex_lock(&mutex_);
    ActuatorStatusMsg msg(clock_.Elapsed(), active_actuators_ptrs_[i]->getStatus());
    pthread_mutex_unlock(&mutex_);
    emit sendMsg(msg.serialized());
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
  ControllerBase* prev_controller = controller_;
  controller_                     = &controller;

  for (Actuator* actuator_ptr : active_actuators_ptrs_)
  {
    pthread_mutex_lock(&mutex_);
    controller.SetMotorID(actuator_ptr->ID());
    controller.SetMode(ControlMode::MOTOR_POSITION);
    controller.SetMotorPosTarget(actuator_ptr->getWinch().getServoHomePos(), true, 3.0);
    pthread_mutex_unlock(&mutex_);

    if (waitUntilTargetReached() != RetVal::OK)
    {
      emit printToQConsole("WARNING: Transition to home position interrupted");
      return false;
    }
  }
  controller_ = prev_controller; // restore original controller

  emit printToQConsole("Daddy, I'm home!");
  return true;
}

void CableRobot::setController(ControllerBase* controller)
{
  pthread_mutex_lock(&mutex_);
  controller_ = controller;
  pthread_mutex_unlock(&mutex_);
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
    pthread_mutex_lock(&mutex_);
    if (controller_->targetReached())
    {
      pthread_mutex_unlock(&mutex_);
      qmutex_.lock();
      is_waiting_ = false;
      qmutex_.unlock();
      return RetVal::OK;
    }
    pthread_mutex_unlock(&mutex_);
    // Check if external abort signal is received
    QCoreApplication::processEvents();
    qmutex_.lock();
    if (stop_waiting_cmd_recv_)
    {
      is_waiting_            = false;
      stop_waiting_cmd_recv_ = false;
      qmutex_.unlock();
      CLOG(INFO, "event") << "Stop waiting command received while target not yet reached";
      return RetVal::EINT;
    }
    qmutex_.unlock();
    // Check if timeout expired (safety feature to prevent hanging in forever)
    if (max_wait_time_sec > 0 && clock.ElapsedFromStart() > max_wait_time_sec)
    {
      qmutex_.lock();
      is_waiting_ = false;
      qmutex_.unlock();
      emit printToQConsole(
        "WARNING: Actuator is taking too long to reach target: operation aborted");
      return RetVal::ETIMEOUT;
    }
    clock.WaitUntilNext();
  }
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
    for (size_t i = 0; i < active_actuators_id_.size(); i++)
    {
      // Check if external abort signal is received
      QCoreApplication::processEvents();
      qmutex_.lock();
      if (stop_waiting_cmd_recv_)
      {
        is_waiting_            = false;
        stop_waiting_cmd_recv_ = false;
        qmutex_.unlock();
        CLOG(INFO, "event") << "Stop waiting command received while platform not yet"
                               " steady";
        return RetVal::EINT;
      }
      qmutex_.unlock();
      // Add filtered angle
      pthread_mutex_lock(&mutex_);
      double current_pulley_angle = active_actuators_ptrs_[i]->getStatus().pulley_angle;
      pthread_mutex_unlock(&mutex_);
      pulleys_angles[i].Add(lp_filters[i].Filter(current_pulley_angle));
      if (!pulleys_angles[i].IsFull()) // wait at least until buffer is full
        continue;
      // Condition to detect steadyness
      swinging = grabnum::Std(pulleys_angles[i].Data()) > kMaxAngleDeviation_;
      if (swinging)
        break;
    }
    // Check if timeout expired (safety feature to prevent hanging in forever)
    if (max_wait_time_sec > 0 && clock.ElapsedFromStart() > max_wait_time_sec)
    {
      qmutex_.lock();
      is_waiting_ = false;
      qmutex_.unlock();
      emit printToQConsole(
        "WARNING: Platform is taking too long to stabilize: operation aborted");
      return RetVal::ETIMEOUT;
    }
    clock.WaitUntilNext();
  }
  qmutex_.lock();
  is_waiting_ = false;
  qmutex_.unlock();
  return RetVal::OK;
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

  pthread_mutex_lock(&mutex_);
  if (state_estimator_ != nullptr)
  {
    delete state_estimator_;
    state_estimator_ = nullptr;
  }
  pthread_mutex_unlock(&mutex_);

  stopTimers();
  motor_status_timer_->start(kMotorStatusIntervalMsec_);
}

STATE_DEFINE(CableRobot, Calibration, NoEventData)
{
  printStateTransition(prev_state_, ST_CALIBRATION);
  prev_state_ = ST_CALIBRATION;

  stopTimers();
}

STATE_DEFINE(CableRobot, Homing, NoEventData)
{
  printStateTransition(prev_state_, ST_HOMING);
  prev_state_ = ST_HOMING;

  pthread_mutex_lock(&mutex_);
  if (state_estimator_ != nullptr)
  {
    delete state_estimator_;
    state_estimator_ = nullptr;
  }
  pthread_mutex_unlock(&mutex_);

  stopTimers();
  actuator_status_timer_->start(kActuatorStatusIntervalMsec_);
}

STATE_DEFINE(CableRobot, Ready, NoEventData)
{
  printStateTransition(prev_state_, ST_READY);
  prev_state_ = ST_READY;

  pthread_mutex_lock(&mutex_);
  if (state_estimator_ == nullptr)
    state_estimator_ = new StateEstimatorBase(params_active_);
  pthread_mutex_unlock(&mutex_);

  stopTimers();
  motor_status_timer_->start(kMotorStatusIntervalMsec_);
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

  stopTimers();
}

//--------- Private slots -----------------------------------------------------------//

void CableRobot::forwardPrintToQConsole(const QString& text) const
{
  emit printToQConsole(text);
}

void CableRobot::emitMotorStatus()
{
  static uint8_t counter = 0;

  if (!(ec_network_valid_ && rt_thread_active_))
    return;

  id_t id = active_actuators_id_[counter++];
  if (pthread_mutex_trylock(&mutex_) != 0)
    return;

  grabec::GSWDriveInPdos motor_status(
    actuators_ptrs_[id]->getWinch().getServo()->GetDriveStatus());
  pthread_mutex_unlock(&mutex_);

  emit motorStatus(id, motor_status);
  if (counter >= active_actuators_id_.size())
    counter = 0;
}

void CableRobot::emitActuatorStatus()
{
  static size_t idx = 0;

  if (!(ec_network_valid_ && rt_thread_active_))
    return;

  if (pthread_mutex_trylock(&mutex_) != 0)
    return;
  active_actuators_status_[idx] = active_actuators_ptrs_[idx]->getStatus();
  pthread_mutex_unlock(&mutex_);

  emit actuatorStatus(active_actuators_status_[idx++]);
  if (idx >= active_actuators_id_.size())
    idx = 0;
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
    slave_ptr->ReadInputs();                // read pdos
  active_actuators_status_updated_ = false; // reset

  if (state_estimator_ != nullptr)
    stateEstimationStep();

  if (controller_ != nullptr)
    controlStep();

  static uint log_counter = 0;
  if (rt_logging_enabled_ && (++log_counter % rt_logging_mod_ == 0))
  {
    collectAndDumpMeasRt();
    log_counter = 0;
  }

  for (grabec::EthercatSlave* slave_ptr : slaves_ptrs_)
    slave_ptr->WriteOutputs(); // write all the necessary pdos
}

void CableRobot::EcEmergencyFun() {}

//--------- RT cyclic steps related private functions -------------------------------//

void CableRobot::stateEstimationStep()
{
  updateActiveActuatorsStatus();

  if (!state_estimator_->estimatePlatformPose(active_actuators_status_, cdpr_status_))
    CLOG(WARNING, "event") << "State estimation step failed";
}

void CableRobot::controlStep()
{
  updateActiveActuatorsStatus();

  vect<ControlAction> ctrl_actions =
    controller_->calcCtrlActions(cdpr_status_, active_actuators_status_);
  for (const ControlAction& ctrl_action : ctrl_actions)
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

    if (!actuators_ptrs_[ctrl_action.motor_id]->isEnabled()) // safety check
      continue;

    switch (ctrl_action.ctrl_mode)
    {
      case CABLE_LENGTH:
        actuators_ptrs_[ctrl_action.motor_id]->setCableLength(ctrl_action.cable_length);
        break;
      case MOTOR_POSITION:
        actuators_ptrs_[ctrl_action.motor_id]->setMotorPos(ctrl_action.motor_position);
        break;
      case MOTOR_SPEED:
        actuators_ptrs_[ctrl_action.motor_id]->setMotorSpeed(ctrl_action.motor_speed);
        break;
      case MOTOR_TORQUE:
        actuators_ptrs_[ctrl_action.motor_id]->setMotorTorque(ctrl_action.motor_torque);
        break;
      case NONE:
        break;
    }
  }
}
