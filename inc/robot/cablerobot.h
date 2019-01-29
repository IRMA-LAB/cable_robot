/**
 * @file cablerobot.cpp
 * @author Simone Comari, Edoardo Idà
 * @date 24 Gen 2019
 * @brief File containing ...
 */

#ifndef CABLE_ROBOT_CABLEROBOT_H
#define CABLE_ROBOT_CABLEROBOT_H

#define INCLUDE_EASYCAT 0

#include <QObject>
#include <QTimer>

#include "easylogging++.h"
#include "StateMachine.h"
#include "libcdpr/inc/types.h"
#include "libgrabec/inc/ethercatmaster.h"
#if INCLUDE_EASYCAT
#include "slaves/easycat/TestEasyCAT1_slave.h"
#include "slaves/easycat/TestEasyCAT2_slave.h"
#endif

#include "components/actuator.h"
#include "ctrl/controller_base.h"
#include "ctrl/controller_singledrive_naive.h"
#include "utils/easylog_wrapper.h"

class CableRobot : public QObject,
                   public virtual grabec::EthercatMaster,
                   public StateMachine
{
  Q_OBJECT

public:
  CableRobot(QObject* parent, const grabcdpr::Params& config);
  ~CableRobot();

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

  const ActuatorStatus GetActuatorStatus(const id_t motor_id);
  void UpdateHomeConfig(const double cable_len, const double pulley_angle);
  void UpdateHomeConfig(const id_t motor_id, const double cable_len,
                        const double pulley_angle);

  bool MotorEnabled(const id_t motor_id);
  bool AnyMotorEnabled();
  bool MotorsEnabled();
  void EnableMotor(const id_t motor_id);
  void EnableMotors();
  void EnableMotors(const vect<id_t>& motors_id);
  void DisableMotor(const id_t motor_id);
  void DisableMotors();
  void DisableMotors(const vect<id_t>& motors_id);
  void SetMotorOpMode(const id_t motor_id, const qint8 op_mode);
  void SetMotorsOpMode(const qint8 op_mode);
  void SetMotorsOpMode(const vect<id_t>& motors_id, const qint8 op_mode);
  vect<id_t> GetActiveMotorsID() const { return active_actuators_id_; }
  void ClearFaults();

  void CollectMeas();
  void DumpMeas() const;
  bool GoHome();

  void SetController(ControllerBase* controller);

public slots:
  void enterCalibrationMode();
  void enterHomingMode();
  void eventSuccess();
  void eventFailure();
  void stop();

signals:
  void motorStatus(const id_t&, const grabec::GSWDriveInPdos&) const;
  void actuatorStatus(const id_t&, const ActuatorStatus&) const;
  void sendMsg(const QByteArray) const;
  void printToQConsole(const QString&) const;
  void ecStateChanged(const Bitfield8&) const;
  void rtThreadStatusChanged(const bool) const;

private slots:
  void forwardPrintToQConsole(const QString&) const;

private:
  //-------- Pseudo-signals from EthercatMaster base class (live in RT thread) -------//

  void EcStateChangedCb(const Bitfield8& new_state) override final;
  void EcPrintCb(const std::string& msg, const char color = 'w') const override final;
  void EcRtThreadStatusChanged(const bool active) override final;

private:
  static constexpr int kMotorStatusIntervalMsec_ = 100;
  static constexpr int kActuatorStatusIntervalMsec_ = 10;
  QTimer* motor_status_timer_ = NULL;
  QTimer* actuator_status_timer_ = NULL;

  grabcdpr::PlatformVars platform_;
  grabcdpr::Vars status_;

  void EmitMotorStatus();
  void EmitActuatorStatus();
  void StopTimers();

  // Data logging
  vect<ActuatorStatusMsg> meas_;
  LogBuffer log_buffer_;
  grabrt::Clock clock_;

  // Ethercat related
#if INCLUDE_EASYCAT
  grabec::TestEasyCAT1Slave* easycat1_ptr_;
  grabec::TestEasyCAT2Slave* easycat2_ptr_;
#endif
  vect<Actuator*> actuators_ptrs_;
  vect<Actuator*> active_actuators_ptrs_;
  vect<id_t> active_actuators_id_;
  bool ec_network_valid_ = false;
  bool rt_thread_active_ = false;

  void EcWorkFun() override final;      // lives in the RT thread
  void EcEmergencyFun() override final; // lives in the RT thread

  // Control related
  ControllerBase* controller_ = NULL;

  void ControlStep();

private:
  //--------- State machine --------------------------------------------------//

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

#endif // CABLE_ROBOT_CABLEROBOT_H
