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
  uint8_t slave_pos = 0;
  grabec::EasyCatSlave temp(slave_pos);
  easycat_slaves_.push_back(temp);
  ec_slaves_ptrs_.push_back(&easycat_slaves_[slave_pos++]);
#endif

  for (size_t i = 0; i < config.cables.size(); i++)
  {
    grabcdpr::CableVars cable;
    status_.cables.push_back(cable);
#if ECNTW
    actuators_.push_back(Actuator(slave_pos++, config.cables[i]));
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

bool CableRobot::EnableMotors(const std::vector<uint8_t>& motors_id)
{
  for (uint8_t motor_id : motors_id)
  {
    actuators_[motor_id].Enable();
    if (!actuators_[motor_id].IsEnabled())
      return false;
  }
  return true;
}

bool CableRobot::DisableMotors(const std::vector<uint8_t>& motors_id)
{
  for (uint8_t motor_id : motors_id)
  {
    actuators_[motor_id].Disable();
    if (actuators_[motor_id].IsEnabled())
      return false;
  }
  return true;
}

void CableRobot::SetMotorOpMode(const uint8_t motor_id, const int8_t op_mode)
{
  actuators_[motor_id].SetMotorOpMode(op_mode);
}

MotorStatus CableRobot::GetMotorStatus(const uint8_t motor_id) const
{
  return actuators_[motor_id].GetWinchStatus();
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
  std::vector<MotorStatus> res = controller_->CalcCableSetPoint(status_);
  for (MotorStatus& ctrl_output : res)
  {
    emit motorStatus(
      ctrl_output.motor_id,
      actuators_[ctrl_output.motor_id].GetWinch()->GetServo()->GetDriveStatus());

    if (!actuators_[ctrl_output.motor_id].IsEnabled()) // safety check
      continue;

    switch (ctrl_output.op_mode)
    {
    case grabec::CYCLIC_POSITION:
      actuators_[ctrl_output.motor_id].SetCableLength(ctrl_output.length_target);
      break;
    case grabec::CYCLIC_VELOCITY:
      actuators_[ctrl_output.motor_id].SetMotorSpeed(ctrl_output.speed_target);
      break;
    case grabec::CYCLIC_TORQUE:
      actuators_[ctrl_output.motor_id].SetMotorTorque(ctrl_output.torque_target);
      break;
    default:
      break;
    }
  }
}
