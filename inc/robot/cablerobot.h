#ifndef CABLEROBOT_H
#define CABLEROBOT_H

#include <QObject>

#include "StateMachine.h"
#include "libcdpr/inc/types.h"
#include "ethercatmaster.h"
#include "slaves/easycatslave.h"
#include "components/actuator.h"
#include "ctrl/controller_base.h"

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

  bool MotorEnabled(const quint8 motor_id) { return actuators_[motor_id].IsEnabled(); }
  bool AnyMotorEnabled();
  bool MotorsEnabled();
  bool EnableMotor(const quint8 motor_id);
  bool EnableMotors();
  bool EnableMotors(const std::vector<quint8>& motors_id);
  bool DisableMotor(const quint8 motor_id);
  bool DisableMotors();
  bool DisableMotors(const std::vector<quint8>& motors_id);
  void SetMotorOpMode(const quint8 motor_id, const qint8 op_mode);
  void SetMotorsOpMode(const qint8 op_mode);
  void SetMotorsOpMode(const std::vector<quint8>& motors_id, const qint8 op_mode);
  ActuatorStatus GetActuatorStatus(const quint8 motor_id) const;
  vect<quint8> GetMotorsID() const;

  void CollectMeas();
  void DumpMeas() const;

  void SetController(ControllerBase* controller) { controller_ = controller; }

public slots:
  void EnterCalibrationMode();
  void EnterHomingMode();
  void EventSuccess();
  void EventFailure();
  void Stop();

signals:
  void printToQConsole(const QString&) const;
  void motorStatus(const quint8, const grabec::GSWDriveInPdos&) const;

private:
  grabcdpr::PlatformVars platform_;
  grabcdpr::Vars status_;
  vect<ActuatorStatus> meas_;

  // Ethercat related
  vect<grabec::EasyCatSlave> easycat_slaves_;
  vect<Actuator> actuators_;
  vect<grabec::EthercatSlave*> ec_slaves_ptrs_;

  void StartUpFunction() override final {}
  void LoopFunction() override final;

  // Control related
  ControllerBase* controller_ = NULL;

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
