#include "robot/cablerobot.h"

constexpr char* CableRobot::kStatesStr[];

CableRobot::CableRobot(QObject* parent, const grabcdpr::Params& config)
  : QObject(parent), StateMachine(ST_MAX_STATES), platform_(grabcdpr::TILT_TORSION),
    prev_state_(ST_MAX_STATES)
{
  PrintStateTransition(prev_state_, ST_IDLE);
  prev_state_ = ST_IDLE;

  status_.platform = &platform_;

#if ECNTW
  quint8 slave_pos = 0;
  grabec::EasyCatSlave temp(slave_pos);
  easycat_slaves_.push_back(temp);
  ec_slaves_ptrs_.push_back(&easycat_slaves_[slave_pos++]);
#endif

  meas_.reserve(config.cables.size());
  for (size_t i = 0; i < config.cables.size(); i++)
  {
    grabcdpr::CableVars cable;
    status_.cables.push_back(cable);
#if ECNTW
    actuators_.push_back(Actuator(i, slave_pos++, config.cables[i]));
    ec_slaves_ptrs_.push_back(actuators_[i].GetWinch()->GetServo());
#endif
  }

  // todo: is this necessary? maybe fix ethercatmaster directly
  num_slaves_ = static_cast<int>(ec_slaves_ptrs_.size());
#if ECNTW
  slave_ = &ec_slaves_ptrs_[0];
  for (int i = 0; i < num_slaves_; i++)
    num_domain_elements_ += ec_slaves_ptrs_[i]->GetDomainEntriesNum();
#endif
}

////////////////////////////////////////////////////////////////////////////
//// Public functions
////////////////////////////////////////////////////////////////////////////

ActuatorStatus CableRobot::GetActuatorStatus(const quint8 motor_id) const
{
  return actuators_[motor_id].GetStatus();
}

void CableRobot::UpdateHomeConfig(const double cable_len, const double pulley_angle)
{
  for (Actuator& actuator : actuators_)
    actuator.UpdateHomeConfig(cable_len, pulley_angle);
}

void CableRobot::UpdateHomeConfig(const quint8 motor_id, const double cable_len,
                                  const double pulley_angle)
{
  actuators_[motor_id].UpdateHomeConfig(cable_len, pulley_angle);
}

bool CableRobot::AnyMotorEnabled()
{
  for (Actuator& actuator : actuators_)
    if (actuator.IsEnabled())
      return true;
  return false;
}

bool CableRobot::MotorsEnabled()
{
  for (Actuator& actuator : actuators_)
    if (!actuator.IsEnabled())
      return false;
  return true;
}

bool CableRobot::EnableMotor(const quint8 motor_id)
{
  actuators_[motor_id].Enable();
  if (!MotorEnabled(motor_id))
  {
    emit printToQConsole(QString("WARNING: Could not enable motor %1").arg(motor_id));
    return false;
  }
  emit printToQConsole(QString("Motor %1 enabled").arg(motor_id));
  return true;
}

bool CableRobot::EnableMotors()
{
  bool ret = false;
  for (Actuator& actuator : actuators_)
  {
    actuator.Enable();
    ret = actuator.IsEnabled();
    if (!ret)
      break;
  }
  if (ret)
    emit printToQConsole("All motors enabled");
  else
    emit printToQConsole("WARNING: Could not enable all motors");
  return ret;
}

bool CableRobot::EnableMotors(const std::vector<quint8>& motors_id)
{
  for (const quint8& motor_id : motors_id)
  {
    actuators_[motor_id].Enable();
    if (!MotorEnabled(motor_id))
    {
      emit printToQConsole(QString("WARNING: Could not enable motor %1").arg(motor_id));
      return false;
    }
    emit printToQConsole(QString("Motor %1 enabled").arg(motor_id));
  }
  return true;
}

bool CableRobot::DisableMotor(const quint8 motor_id)
{
  actuators_[motor_id].Disable();
  if (MotorEnabled(motor_id))
    return false;
  emit printToQConsole(QString("Motor %1 disabled").arg(motor_id));
  return true;
}

bool CableRobot::DisableMotors()
{
  for (Actuator& actuator : actuators_)
  {
    actuator.Disable();
    if (actuator.IsEnabled())
      return false;
  }
  emit printToQConsole("All motors disabled");
  return true;
}

bool CableRobot::DisableMotors(const std::vector<quint8>& motors_id)
{
  for (const quint8& motor_id : motors_id)
  {
    actuators_[motor_id].Disable();
    if (MotorEnabled(motor_id))
      return false;
    emit printToQConsole(QString("Motor %1 disabled").arg(motor_id));
  }
  return true;
}

void CableRobot::SetMotorOpMode(const quint8 motor_id, const qint8 op_mode)
{
  actuators_[static_cast<quint8>(motor_id)].SetMotorOpMode(op_mode);
}

void CableRobot::SetMotorsOpMode(const qint8 op_mode)
{
  for (Actuator& actuator : actuators_)
    actuator.SetMotorOpMode(op_mode);
}

void CableRobot::SetMotorsOpMode(const std::vector<quint8>& motors_id,
                                 const qint8 op_mode)
{
  for (const quint8& motor_id : motors_id)
    actuators_[static_cast<quint8>(motor_id)].SetMotorOpMode(op_mode);
}

vect<quint8> CableRobot::GetMotorsID() const
{
  std::vector<quint8> motors_id;
  for (const Actuator& actuator : actuators_)
    motors_id.push_back(actuator.GetActuatorID());
  return motors_id;
}

void CableRobot::CollectMeas()
{
  for (quint8 i = 0; i < meas_.size(); ++i)
    meas_[i] = actuators_[i].GetStatus();

  emit printToQConsole("Measurements collected");
}

void CableRobot::DumpMeas() const
{
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

  for (Actuator& actuator : actuators_)
  {
    controller.SetMotorID(actuator.ID());
    controller.SetMotorPosTarget(actuator.GetWinch()->GetServoHomePos());
    while (!controller.MotorPosTargetReached(actuator.GetStatus().motor_position))
      continue; // todo: inserisci un tempo di attesa qui
  }
  controller_ = prev_controller; // restore original controller

  emit printToQConsole("Daddy, I'm home!");
  return true;
}

////////////////////////////////////////////////////////////////////////////
//// External events public
////////////////////////////////////////////////////////////////////////////

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
//// States actions private
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
//// Miscellaneous private
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

////////////////////////////////////////////////////////////////////////////
//// Ethercat related private functions
////////////////////////////////////////////////////////////////////////////

void CableRobot::LoopFunction()
{
  for (size_t i = 0; i < ec_slaves_ptrs_.size(); i++)
    ec_slaves_ptrs_[i]->ReadInputs(); // read pdos, act accordingly

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

    for (size_t i = 0; i < ec_slaves_ptrs_.size(); i++)
      ec_slaves_ptrs_[i]->WriteOutputs(); // write all the necessary pdos
}

////////////////////////////////////////////////////////////////////////////
//// Control related private functions
////////////////////////////////////////////////////////////////////////////

void CableRobot::ControlStep()
{
  std::vector<ControlAction> res = controller_->CalcCableSetPoint(status_);
  for (ControlAction& ctrl_action : res)
  {
    emit motorStatus(
      ctrl_action.motor_id,
      actuators_[ctrl_action.motor_id].GetWinch()->GetServo()->GetDriveStatus());

    if (!actuators_[ctrl_action.motor_id].IsEnabled()) // safety check
      continue;

    switch (ctrl_action.ctrl_mode)
    {
    case CABLE_LENGTH:
      actuators_[ctrl_action.motor_id].SetCableLength(ctrl_action.cable_length);
      break;
    case MOTOR_POSITION:
      actuators_[ctrl_action.motor_id].SetMotorPos(ctrl_action.motor_position);
      break;
    case MOTOR_SPEED:
      actuators_[ctrl_action.motor_id].SetMotorSpeed(ctrl_action.motor_speed);
      break;
    case MOTOR_TORQUE:
      actuators_[ctrl_action.motor_id].SetMotorTorque(ctrl_action.motor_torque);
      break;
    case NONE:
      break;
    }
  }
}
