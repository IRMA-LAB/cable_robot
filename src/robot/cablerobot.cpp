/**
 * @file cablerobot.cpp
 * @author Simone Comari, Edoardo Idà
 * @date 17 Gen 2019
 * @brief File containing definitions of functions and class declared in cablerobot.h.
 */

#include "robot/cablerobot.h"

constexpr char* CableRobot::kStatesStr[];

CableRobot::CableRobot(QObject* parent, const grabcdpr::Params& config)
  : QObject(parent), StateMachine(ST_MAX_STATES), platform_(grabcdpr::TILT_TORSION),
    log_buffer_(el::Loggers::getLogger("data")), prev_state_(ST_MAX_STATES)
{
  PrintStateTransition(prev_state_, ST_IDLE);
  prev_state_ = ST_IDLE;

  status_.platform = &platform_;

  quint8 slave_pos = 0;

  for (size_t i = 0; i < config.actuators.size(); i++)
  {
    grabcdpr::CableVars cable;
    status_.cables.push_back(cable);
    actuators_ptrs_.push_back(new Actuator(i, slave_pos++, config.actuators[i], this));
    slaves_ptrs_.push_back(actuators_ptrs_[i]->GetWinch().GetServo());
    if (config.actuators[i].active)
    {
      active_actuators_ptrs_.push_back(actuators_ptrs_[i]);
      connect(actuators_ptrs_[i], SIGNAL(printToQConsole(QString)), this,
              SLOT(forwardPrintToQConsole(QString)));
    }
  }
  meas_.resize(active_actuators_ptrs_.size());

  for (grabec::EthercatSlave* slave_ptr : slaves_ptrs_)
    num_domain_elements_ += slave_ptr->GetDomainEntriesNum();

  connect(this, SIGNAL(sendMsg(QByteArray)), &log_buffer_, SLOT(collectMsg(QByteArray)));
  log_buffer_.start();
}

CableRobot::~CableRobot()
{
  log_buffer_.Stop();
  disconnect(this, SIGNAL(sendMsg(QByteArray)), &log_buffer_,
             SLOT(collectMsg(QByteArray)));

  for (Actuator* actuator_ptr : actuators_ptrs_)
  {
    if (actuator_ptr->IsActive())
    {
      disconnect(actuator_ptr, SIGNAL(printToQConsole(QString)), this,
                 SLOT(forwardPrintToQConsole(QString)));
    }
    delete actuator_ptr;
  }
}

//--------- Public Functions --------------------------------------------------//

const ActuatorStatus CableRobot::GetActuatorStatus(const id_t motor_id)
{
  pthread_mutex_lock(&mutex_);
  ActuatorStatus status = actuators_ptrs_[motor_id]->GetStatus();
  pthread_mutex_unlock(&mutex_);
  return status;
}

void CableRobot::UpdateHomeConfig(const double cable_len, const double pulley_angle)
{
  for (Actuator* actuator_ptr : active_actuators_ptrs_)
    actuator_ptr->UpdateHomeConfig(cable_len, pulley_angle);
}

void CableRobot::UpdateHomeConfig(const id_t motor_id, const double cable_len,
                                  const double pulley_angle)
{
  actuators_ptrs_[motor_id]->UpdateHomeConfig(cable_len, pulley_angle);
}

bool CableRobot::MotorEnabled(const id_t motor_id)
{
  return actuators_ptrs_[motor_id]->IsEnabled();
}

bool CableRobot::AnyMotorEnabled()
{
  for (Actuator* actuator_ptr : active_actuators_ptrs_)
    if (actuator_ptr->IsEnabled())
      return true;
  return false;
}

bool CableRobot::MotorsEnabled()
{
  for (Actuator* actuator_ptr : active_actuators_ptrs_)
    if (!actuator_ptr->IsEnabled())
      return false;
  return true;
}

void CableRobot::EnableMotor(const id_t motor_id)
{
  if (actuators_ptrs_[motor_id]->IsActive())
    actuators_ptrs_[motor_id]->Enable();
}

void CableRobot::EnableMotors()
{
  for (Actuator* actuator_ptr : active_actuators_ptrs_)
  {
    actuator_ptr->Enable();
  }
}

void CableRobot::EnableMotors(const vect<id_t>& motors_id)
{
  for (const id_t& motor_id : motors_id)
    if (actuators_ptrs_[motor_id]->IsActive())
    {
      actuators_ptrs_[motor_id]->Enable();
    }
}

void CableRobot::DisableMotor(const id_t motor_id)
{
  if (actuators_ptrs_[motor_id]->IsActive())
  {
    actuators_ptrs_[motor_id]->Disable();
  }
}

void CableRobot::DisableMotors()
{
  for (Actuator* actuator_ptr : active_actuators_ptrs_)
  {
    actuator_ptr->Disable();
  }
}

void CableRobot::DisableMotors(const vect<id_t>& motors_id)
{
  for (const id_t& motor_id : motors_id)
    if (actuators_ptrs_[motor_id]->IsActive())
    {
      actuators_ptrs_[motor_id]->Disable();
    }
}

void CableRobot::SetMotorOpMode(const id_t motor_id, const qint8 op_mode)
{
  actuators_ptrs_[motor_id]->SetMotorOpMode(op_mode);
}

void CableRobot::SetMotorsOpMode(const qint8 op_mode)
{
  for (Actuator* actuator_ptr : active_actuators_ptrs_)
    actuator_ptr->SetMotorOpMode(op_mode);
}

void CableRobot::SetMotorsOpMode(const vect<id_t>& motors_id, const qint8 op_mode)
{
  for (const id_t& motor_id : motors_id)
    actuators_ptrs_[motor_id]->SetMotorOpMode(op_mode);
}

vect<id_t> CableRobot::GetActiveMotorsID() const
{
  vect<id_t> motors_id;
  for (const Actuator* actuator_ptr : active_actuators_ptrs_)
    motors_id.push_back(actuator_ptr->ID());
  return motors_id;
}

void CableRobot::ClearFaults()
{
  for (Actuator* actuator_ptr : active_actuators_ptrs_)
    if (actuator_ptr->IsInFault())
      actuator_ptr->FaultReset();
}

void CableRobot::CollectMeas()
{
  size_t i = 0;
  for (Actuator* actuator_ptr : active_actuators_ptrs_)
  {
    meas_[i].body = actuator_ptr->GetStatus();
    meas_[i].header.timestamp = clock_.Elapsed();
    i++;
  }
  emit printToQConsole("Measurements collected");
}

void CableRobot::DumpMeas() const
{
  for (ActuatorStatusMsg msg : meas_)
    emit sendMsg(msg.serialized());
  emit printToQConsole("Measurements dumped onto log file");
}

bool CableRobot::GoHome()
{
  if (!MotorsEnabled())
  {
    emit printToQConsole("Cannot move to home position: not all motors enabled");
    return false;
  }
  emit printToQConsole("Moving to home position...");

  ControllerSingleDriveNaive controller;
  // temporarly switch to local controller for moving to home pos
  ControllerBase* prev_controller = controller_;
  controller_ = &controller;

  for (Actuator* actuator_ptr : active_actuators_ptrs_)
  {
    controller.SetMotorID(actuator_ptr->ID());
    controller.SetMotorPosTarget(actuator_ptr->GetWinch().GetServoHomePos());
    while (!controller.MotorPosTargetReached(actuator_ptr->GetStatus().motor_position))
      continue; // todo: inserisci un tempo di attesa qui
  }
  controller_ = prev_controller; // restore original controller

  emit printToQConsole("Daddy, I'm home!");
  return true;
}

void CableRobot::SetController(ControllerBase* controller)
{
  pthread_mutex_lock(&mutex_);
  controller_ = controller;
  pthread_mutex_unlock(&mutex_);
}

//--------- External Events Public --------------------------------------------------//

void CableRobot::enterCalibrationMode()
{
  CLOG(TRACE, "event");
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

void CableRobot::enterHomingMode()
{
  CLOG(TRACE, "event");
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

void CableRobot::eventSuccess()
{
  CLOG(TRACE, "event");
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

void CableRobot::eventFailure()
{
  CLOG(TRACE, "event");
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

void CableRobot::stop()
{
  CLOG(TRACE, "event");
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

//--------- States Actions Private --------------------------------------------------//

STATE_DEFINE(CableRobot, Idle, NoEventData)
{
  PrintStateTransition(prev_state_, ST_IDLE);
  prev_state_ = ST_IDLE;
}

STATE_DEFINE(CableRobot, Enabled, NoEventData)
{
  PrintStateTransition(prev_state_, ST_ENABLED);
  prev_state_ = ST_ENABLED;

  EmitMotorStatusSync();

  if (controller_ != NULL)
    ControlStep(); // take care of manual control when a motor is active, skip otherwise
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

  if (controller_ != NULL)
    ControlStep(); // take care of manual control when a motor is active, skip otherwise
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

//--------- Private slots --------------------------------------------------//

void CableRobot::forwardPrintToQConsole(const QString& text) const
{
  emit printToQConsole(text);
}

//--------- Miscellaneous private --------------------------------------------------//

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
    msg = QString("CableRobot initial state: %1").arg(kStatesStr[new_state]);
  emit printToQConsole(msg);
}

void CableRobot::EmitMotorStatusSync() const
{
  static const std::vector<id_t> active_motors_id = GetActiveMotorsID();
  static grabrt::Clock clock;
  static uint8_t counter = 0;

  if (!ec_network_valid_)
    return;

  if (clock.Elapsed() < kEmitPeriodSec_)
    return;

  id_t id = active_motors_id[counter++];
  emit motorStatus(id, actuators_ptrs_[id]->GetWinch().GetServo()->GetDriveStatus());
  clock.Reset();

  if (counter >= active_motors_id.size())
    counter = 0;
}

//--------- Ethercat related private functions ---------------------------------//

void CableRobot::EcStateChangedCb(const Bitfield8& new_state)
{
  ec_network_valid_ = (new_state.Count() == 3);
  emit ecStateChanged(new_state);
}

void CableRobot::EcPrintCb(const std::string& msg, const char color /* = 'w' */) const
{
  switch (color)
  {
  case 'r':
    emit printToQConsole(QString("<span style='color: red'>%1</span>").arg(msg.c_str()));
    break;
  case 'y':
      emit printToQConsole(QString("<span style='color: yellow'>%1</span>").arg(msg.c_str()));
    break;
  default:
    emit printToQConsole(msg.c_str());
    break;
  }
}

void CableRobot::EcRtThreadStatusChanged(const bool active) const
{
  emit rtThreadStatusChanged(active);
}

void CableRobot::EcWorkFun()
{
  for (grabec::EthercatSlave* slave_ptr : slaves_ptrs_)
    slave_ptr->ReadInputs(); // read pdos

  // clang-format off
  BEGIN_TRANSITION_MAP			              			// - Current State -
      TRANSITION_MAP_ENTRY (ST_IDLE)                                  // ST_IDLE
      TRANSITION_MAP_ENTRY (ST_ENABLED)                          // ST_ENABLED
      TRANSITION_MAP_ENTRY (ST_CALIBRATION)                    // ST_CALIBRATION
      TRANSITION_MAP_ENTRY (ST_HOMING)                            // ST_HOMING
      TRANSITION_MAP_ENTRY (ST_READY)                               // ST_READY
      TRANSITION_MAP_ENTRY (ST_OPERATIONAL)			// ST_OPERATIONAL
      TRANSITION_MAP_ENTRY (ST_ERROR)                              // ST_ERROR
  END_TRANSITION_MAP(NULL)
    // clang-format on

    for (grabec::EthercatSlave* slave_ptr : slaves_ptrs_)
      slave_ptr->WriteOutputs(); // write all the necessary pdos
}

void CableRobot::EcEmergencyFun() {}

//--------- Control related private functions ----------------------------------//

void CableRobot::ControlStep()
{
  std::vector<ControlAction> res = controller_->CalcCableSetPoint(status_);
  for (ControlAction& ctrl_action : res)
  {
    if (!actuators_ptrs_[ctrl_action.motor_id]->IsEnabled()) // safety check
      continue;

    switch (ctrl_action.ctrl_mode)
    {
    case CABLE_LENGTH:
      actuators_ptrs_[ctrl_action.motor_id]->SetCableLength(ctrl_action.cable_length);
      break;
    case MOTOR_POSITION:
      actuators_ptrs_[ctrl_action.motor_id]->SetMotorPos(ctrl_action.motor_position);
      break;
    case MOTOR_SPEED:
      actuators_ptrs_[ctrl_action.motor_id]->SetMotorSpeed(ctrl_action.motor_speed);
      break;
    case MOTOR_TORQUE:
      actuators_ptrs_[ctrl_action.motor_id]->SetMotorTorque(ctrl_action.motor_torque);
      break;
    case NONE:
      break;
    }
  }
}
