#ifndef CABLEROBOT_H
#define CABLEROBOT_H

#include <QObject>

#include "StateMachine.h"
#include "libcdpr/inc/types.h"
#include "ethercatmaster.h"
#include "slaves/easycatslave.h"
#include "components/actuator.h"
#include "controller/controller_base.h"

class CableRobot : public QObject,
                   public StateMachine,
                   public virtual grabec::EthercatMaster
{
  Q_OBJECT
public:
  explicit CableRobot(QObject* parent, const grabcdpr::Params& config);
  ~CableRobot() {}

  enum States : BYTE
  {
    ST_IDLE,
    ST_ENABLED,
    ST_CALIBRATION,
    ST_HOMING,
    ST_READY,
    ST_OPERATIONAL,
    ST_ERROR,
    ST_MAX_STATES
  };

  bool EnableMotors(const std::vector<uint8_t>& motors_id);
  bool DisableMotors(const std::vector<uint8_t>& motors_id);
  void SetMotorOpMode(const uint8_t motor_id, const int8_t op_mode);
  MotorStatus GetMotorStatus(const uint8_t motor_id) const;

  void SetController(ControllerBase* controller) { controller_ = controller; }

  void EnterCalibrationMode();
  void EnterHomingMode();

public slots:
  void EventSuccess();
  void EventFailure();
  void Stop();

signals:
  void printToQConsole(const QString&) const;

private:
  grabcdpr::PlatformVars platform_;
  grabcdpr::Vars status_;

  // Ethercat related
  std::vector<grabec::EasyCatSlave> easycat_slaves_;
  std::vector<Actuator> actuators_;
  std::vector<grabec::EthercatSlave*> ec_slaves_ptrs_;

  void StartUpFunction() override final {}
  void LoopFunction() override final;

  // Control related
  ControllerBase* controller_;

  void ControlStep();

// State machine
private:
  // clang-format off
  static constexpr char* kStatesStr[] = {
    const_cast<char*>("IDLE"),
    const_cast<char*>("ENABLED"),
    const_cast<char*>("CALIBRATION"),
    const_cast<char*>("HOMING"),
    const_cast<char*>("READY"),
    const_cast<char*>("OPERATIONAL"),
    const_cast<char*>("ERROR")};
  // clang-format on

  States prev_state_;

  // Define the state machine state functions with event data type
  STATE_DECLARE(CableRobot, Idle, NoEventData)
  STATE_DECLARE(CableRobot, Enabled, NoEventData)
  STATE_DECLARE(CableRobot, Calibration, NoEventData)
  STATE_DECLARE(CableRobot, Homing, NoEventData)
  STATE_DECLARE(CableRobot, Ready, NoEventData)
  STATE_DECLARE(CableRobot, Operational, NoEventData)
  STATE_DECLARE(CableRobot, Error, NoEventData)

  // State map to define state object order. Each state map entry defines a state object.
  BEGIN_STATE_MAP
  // clang-format off
    STATE_MAP_ENTRY({&Idle})
    STATE_MAP_ENTRY({&Enabled})
    STATE_MAP_ENTRY({&Calibration})
    STATE_MAP_ENTRY({&Homing})
    STATE_MAP_ENTRY({&Ready})
    STATE_MAP_ENTRY({&Operational})
    STATE_MAP_ENTRY({&Error})
  // clang-format on
  END_STATE_MAP

  void PrintStateTransition(const States current_state, const States new_state) const;
};

#endif // CABLEROBOT_H
