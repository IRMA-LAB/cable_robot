/**
 * @file cablerobot.cpp
 * @author Simone Comari, Edoardo Idà
 * @date 07 Mar 2019
 * @brief File containing the virtualization of the physical cable robot, in terms of
 * components, signalig and low level operations.
 */

#ifndef CABLE_ROBOT_CABLEROBOT_H
#define CABLE_ROBOT_CABLEROBOT_H

#define INCLUDE_EASYCAT 0

#include <QObject>
#include <QTimer>

#include "StateMachine.h"
#include "easylogging++.h"
#include "libcdpr/inc/types.h"
#include "libgrabec/inc/ethercatmaster.h"
#if INCLUDE_EASYCAT
#include "slaves/easycat/TestEasyCAT1_slave.h"
#include "slaves/easycat/TestEasyCAT2_slave.h"
#endif

#include "components/actuator.h"
#include "ctrl/controller_base.h"
#include "ctrl/controller_singledrive.h"
#include "utils/easylog_wrapper.h"

/**
 * @brief The CableRobot class
 */
class CableRobot: public QObject,
                  public virtual grabec::EthercatMaster,
                  public StateMachine
{
  Q_OBJECT

 public:
  /**
   * @brief CableRobot constructor.
   * @param parent The parent Qt object.
   * @param config Configuration parameters of the cable robot.
   */
  CableRobot(QObject* parent, const grabcdpr::Params& config);
  ~CableRobot() override;

  /**
   * @brief The cable robot states enum
   */
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

  // Tuning params for waiting functions
  static constexpr double kCycleWaitTimeSec = 0.02; /*<< [sec] Cycle time when waiting. */
  static constexpr double kMaxWaitTimeSec   = 25.0; /*<< [sec] Maximum waiting time. */

  /**
   * @brief Get inquired actuator status.
   * @param motor_id The ID of the inquired actuator.
   * @return The status of the inquired actuator.
   */
  const ActuatorStatus GetActuatorStatus(const id_t motor_id);
  /**
   * @brief Update home configuration of all actuators at once.
   *
   * When updating the home configuration new values for cable lengths and swivel pulley
   * angles are assigned for the current motor pose.
   * @param cable_len Cable lengths at homing position.
   * @param pulley_angle Swivel pulleys angles at homing position.
   */
  void UpdateHomeConfig(const double cable_len, const double pulley_angle);
  /**
   * @brief Update home configuration of a single actuator.
   * @param motor_id The ID of the actuator to update.
   * @param cable_len Cable length at homing position for specified actuator.
   * @param pulley_angle Swivel pulley angle at homing position for specified actuator.
   */
  void UpdateHomeConfig(const id_t motor_id, const double cable_len,
                        const double pulley_angle);

  /**
   * @brief Check if inquired motor is enabled.
   * @param motor_id The ID of the inquired motor.
   * @return _True_ if motor is enabled, _false_ otherwise.
   */
  bool MotorEnabled(const id_t motor_id);
  /**
   * @brief Check if any motor is enabled.
   * @return _True_ if at least one motor is enabled, _false_ otherwise.
   */
  bool AnyMotorEnabled();
  /**
   * @brief Check if all motors are enabled.
   * @return _True_ if all motors are enabled, _false_ otherwise.
   */
  bool MotorsEnabled();
  /**
   * @brief Enable a single motor.
   * @param motor_id The ID of the motor to enable.
   */
  void EnableMotor(const id_t motor_id);
  /**
   * @brief Enable all motors at once.
   */
  void EnableMotors();
  /**
   * @brief Enable a set of motors.
   * @param motors_id The IDs of the motor to enable.
   */
  void EnableMotors(const vect<id_t>& motors_id);
  /**
   * @brief Disable a single motor.
   * @param motor_id The ID of the motor to disable.
   */
  void DisableMotor(const id_t motor_id);
  /**
   * @brief Disable all motors at once.
   */
  void DisableMotors();
  /**
   * @brief Disable a set of motors.
   * @param motors_id The IDs of the motor to disable.
   */
  void DisableMotors(const vect<id_t>& motors_id);
  /**
   * @brief Set a single motor operational mode.
   * @param motor_id The ID of the motor whose operational mode is to be set.
   * @param op_mode See grabec::GoldSoloWhistleOperationModes
   */
  void SetMotorOpMode(const id_t motor_id, const qint8 op_mode);
  /**
   * @brief Set all motors operational mode at once.
   * @param op_mode See grabec::GoldSoloWhistleOperationModes
   */
  void SetMotorsOpMode(const qint8 op_mode);
  /**
   * @brief Set a set of motors operational mode at once.
   * @param motors_id The IDs of the motors whose operational mode is to be set.
   * @param op_mode See grabec::GoldSoloWhistleOperationModes
   */
  void SetMotorsOpMode(const vect<id_t>& motors_id, const qint8 op_mode);
  /**
   * @brief Get active motors ID.
   * @return IDs of active motors.
   */
  vect<id_t> GetActiveMotorsID() const { return active_actuators_id_; }
  /**
   * @brief Clear any motor's fault.
   */
  void ClearFaults();

  /**
   * @brief Collect current cable robot measurents.
   */
  void CollectMeas();
  /**
   * @brief Dump latest collected cable robot measurements onto data.log file.
   */
  void DumpMeas() const;
  /**
   * @brief Go to home position.
   * @return _True_ if operation was successful, _false_ otherwise.
   */
  bool GoHome();

  /**
   * @brief Set motors controller.
   * @param controller Pointer to a controller.
   * @note The pointer is to the ControllerBase whose virtual methods have to be override
   * by any derived class of it. In particular, CalcCtrlActions() is called at every cycle
   * of the real time thread while TargetReached() is typically used asynchronously to
   * inquire robot control status in waiting condition.
   */
  void SetController(ControllerBase* controller);
  /**
   * @brief Wait until controller target is reached.
   * @return 0 if target was reached, a positive number otherwise, yielding the error
   * type.
   */
  RetVal WaitUntilTargetReached();

 public slots:
  /**
   * @brief Stop waiting command, to be used to manually interrupt a waiting cycle.
   */
  void stopWaiting();

  /**
   * @brief Enter calibration mode trigger.
   */
  void enterCalibrationMode();
  /**
   * @brief Enter homing mode trigger.
   */
  void enterHomingMode();
  /**
   * @brief Trigger transitions in case of successful outcome of a generic operation.
   *
   * Triggers the following transitions:
   * - IDLE --> ENABLED
   * - CALIBRATION --> ENABLED
   * - HOMING --> READY
   * - OPERATIONAL --> READY
   */
  void eventSuccess();
  /**
   * @brief Trigger transitions in case of failure of a generic operation.
   *
   * Triggers the following transitions:
   * - CALIBRATION --> ENABLED
   * - HOMING --> ENABLED
   * - OPERATIONAL --> ERROR
   */
  void eventFailure();
  /**
   * @brief Stop command.
   *
   * Triggers the following transitions:
   * - CALIBRATION --> ENABLED
   * - HOMING --> ENABLED
   * - READY --> ENABLED
   * - OPERATIONAL --> READY
   * - ERROR --> ENABLED
   */
  void stop();

 signals:
  void motorStatus(const id_t&, const grabec::GSWDriveInPdos&) const;
  void actuatorStatus(const ActuatorStatus&) const;
  void sendMsg(const QByteArray) const;
  void printToQConsole(const QString&) const;
  void ecStateChanged(const Bitfield8&) const;
  void rtThreadStatusChanged(const bool) const;

 private slots:
  void forwardPrintToQConsole(const QString&) const;
  void emitMotorStatus();
  void emitActuatorStatus();

 private:
  //-------- Pseudo-signals from EthercatMaster base class (live in RT thread) --------//

  void EcStateChangedCb(const Bitfield8& new_state) override final;
  void EcPrintCb(const std::string& msg, const char color = 'w') const override final;
  void EcRtThreadStatusChanged(const bool active) override final;

 private:
  grabcdpr::PlatformVars platform_;
  grabcdpr::Vars cdpr_status_;

  // Timers for status updates
  static constexpr int kMotorStatusIntervalMsec_    = 100;
  static constexpr int kActuatorStatusIntervalMsec_ = 10;
  QTimer* motor_status_timer_                       = NULL;
  QTimer* actuator_status_timer_                    = NULL;

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
  vect<ActuatorStatus> active_actuators_status_;
  vect<id_t> active_actuators_id_;
  bool ec_network_valid_ = false;
  bool rt_thread_active_ = false;

  void EcWorkFun() override final;      // lives in the RT thread
  void EcEmergencyFun() override final; // lives in the RT thread

  // Control related
  ControllerBase* controller_ = NULL;
  QMutex qmutex_;
  bool stop_waiting_cmd_recv_ = false;

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
