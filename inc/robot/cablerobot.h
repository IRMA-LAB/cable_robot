/**
 * @file cablerobot.h
 * @author Simone Comari, Edoardo Idà
 * @date 10 JuL 2019
 * @brief File containing the virtualization of the physical cable robot, in terms of
 * components, signalig and low level operations.
 */

#ifndef CABLE_ROBOT_CABLEROBOT_H
#define CABLE_ROBOT_CABLEROBOT_H

#define INCLUDE_EASYCAT 0 /**< @todo remove this debug flag */

#include <QObject>
#include <QTimer>

#include "StateMachine.h"
#include "easylogging++.h"
#include "inc/filters.h"
#include "libcdpr/inc/cdpr_types.h"
#include "libgrabec/inc/ethercatmaster.h"
#if INCLUDE_EASYCAT
#include "slaves/easycat/TestEasyCAT1_slave.h"
#include "slaves/easycat/TestEasyCAT2_slave.h"
#endif

#include "components/actuator.h"
#include "ctrl/controller_base.h"
#include "ctrl/controller_singledrive.h"
#include "state_estimation/state_estimator_base.h"
#include "utils/easylog_wrapper.h"

/**
 * @brief The virtualization of physical GRAB CDPR.
 *
 * This class allows the user to interact with the physical robot on different levels.
 *
 * The cable robot operations are organized as a finite state machine, whose details can
 * be found on the relative documentation in a more friendly and schematic format.
 * Therefore most of its public slots consists of external transition events.
 *
 * CableRobot class is also a derived class of EthercatMaster, because it acts in fact as
 * master of the ethercat network, where its motors are the main slaves.
 * Most cable robot methods live in the main (non real time) thread, except for the ones
 * starting with "Ec", such as EcWorkFun(), and all functions called within those, for
 * instance ControlStep(). Because the real time thread cycles at 1ms, make sure to limit
 * the operations inside these functions to simple, fast operations, avoiding unnecessary
 * prints.
 *
 * This class also includes some timers to be able to synchronously emit useful
 * information to the extern at need, such as motors status.
 *
 * It also takes care of exception and error handling, such as real time deadline missed
 * or ethercat network failures.
 *
 * Robot status logging is also implemented here and can be exploited with CollectMeas()
 * and DumpMeas() functions.
 *
 * Last important remark regards the controller. The controller is called at every cycle
 * of the real time thread and provides commands to the motors, if present and its output
 * is valid. Any controller is a derived class of ControllerBase which provides the
 * virtual API which is used and called here. Make sure that the computational time of
 * your new controller stays largely within 1ms to have some margin for other cyclic
 * operations. Because the controller is a shared pointer between threads, be sure to lock
 * the robot mutex when accessing it from outside. There is no need to do so when calling
 * methods of this class as they already are thread safe, such as SetController().
 *
 * Please refer to the class public methods description for other ancillary functions,
 * such as GoHome().
 */
class CableRobot: public QObject,
                  public virtual grabec::EthercatMaster,
                  public StateMachine
{
  Q_OBJECT

 public:
  /**
   * @brief CableRobot constructor.
   * @param[in] parent The parent Qt object.
   * @param[in] config Configuration parameters of the cable robot.
   */
  CableRobot(QObject* parent, const grabcdpr::RobotParams& config);
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
  static constexpr double kCycleWaitTimeSec = 0.02; /**< [sec] Cycle time when waiting. */
  static constexpr double kMaxWaitTimeSec   = 5.0;  /**< [sec] Maximum waiting time. */

  /**
   * @brief Get all robot parameters.
   * @return All robot parameters.
   */
  const grabcdpr::RobotParams& GetParams() const { return params_; }
  /**
   * @brief Get the parameters of active robot components.
   *
   * This is a subset of all robot parameters. It includes robot platform parameters and
   * the parameters of each active actuator.
   * @return The parameters of active robot components
   */
  grabcdpr::RobotParams GetActiveComponentsParams() const;
  /**
   * @brief GetCdprStatus
   * @return
   */
  const grabcdpr::RobotVars& GetCdprStatus() const { return cdpr_status_; }
  /**
   * @brief Get a pointer to inquired actuator.
   * @param[in] motor_id The ID of the inquired actuator.
   * @return A pointer to inquired actuator.
   */
  const Actuator* GetActuator(const id_t motor_id);

  /**
   * @brief Get inquired actuator status.
   * @param[in] motor_id The ID of the inquired actuator.
   * @return The status of the inquired actuator.
   */
  const ActuatorStatus GetActuatorStatus(const id_t motor_id);

  /**
   * @brief Update home configuration of all actuators at once.
   *
   * When updating the home configuration new values for cable lengths and swivel pulley
   * angles are assigned for the current motor pose.
   * @param[in] cable_len Cable lengths at homing position.
   * @param[in] pulley_angle Swivel pulleys angles at homing position.
   */
  void UpdateHomeConfig(const double cable_len, const double pulley_angle);
  /**
   * @brief Update home configuration of a single actuator.
   * @param[in] motor_id The ID of the actuator to update.
   * @param[in] cable_len Cable length at homing position for specified actuator.
   * @param[in] pulley_angle Swivel pulley angle at homing position for specified
   * actuator.
   */
  void UpdateHomeConfig(const id_t motor_id, const double cable_len,
                        const double pulley_angle);

  /**
   * @brief Check if inquired motor is enabled.
   * @param[in] motor_id The ID of the inquired motor.
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
   * @param[in] motor_id The ID of the motor to enable.
   */
  void EnableMotor(const id_t motor_id);
  /**
   * @brief Enable all motors at once.
   */
  void EnableMotors();
  /**
   * @brief Enable a set of motors.
   * @param[in] motors_id The IDs of the motor to enable.
   */
  void EnableMotors(const vect<id_t>& motors_id);
  /**
   * @brief Disable a single motor.
   * @param[in] motor_id The ID of the motor to disable.
   */
  void DisableMotor(const id_t motor_id);
  /**
   * @brief Disable all motors at once.
   */
  void DisableMotors();
  /**
   * @brief Disable a set of motors.
   * @param[in] motors_id The IDs of the motor to disable.
   */
  void DisableMotors(const vect<id_t>& motors_id);
  /**
   * @brief Set a single motor operational mode.
   * @param[in] motor_id The ID of the motor whose operational mode is to be set.
   * @param[in] op_mode See grabec::GoldSoloWhistleOperationModes
   */
  void SetMotorOpMode(const id_t motor_id, const qint8 op_mode);
  /**
   * @brief Set all motors operational mode at once.
   * @param[in] op_mode See grabec::GoldSoloWhistleOperationModes
   */
  void SetMotorsOpMode(const qint8 op_mode);
  /**
   * @brief Set a set of motors operational mode at once.
   * @param[in] motors_id The IDs of the motors whose operational mode is to be set.
   * @param[in] op_mode See grabec::GoldSoloWhistleOperationModes
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
   * @brief Collect current cable robot measurents without locking the RT-thread mutex.
   */
  void CollectMeasRt();
  /**
   * @brief Collect current cable robot measurents locking the RT-thread mutex.
   */
  void CollectMeas();
  /**
   * @brief Dump latest collected cable robot measurements onto data.log file.
   */
  void DumpMeas() const;
  /**
   * @brief Collect and dump current cable robot measurements onto data.log file without
   * locking the RT-thread mutex.
   */
  void CollectAndDumpMeasRt(const bool active_actuators_status_updated = false);
  /**
   * @brief Collect and dump current cable robot measurements onto data.log file locking
   * the RT-thread mutex.
   */
  void CollectAndDumpMeas();
  /**
   * @brief Collect and dump current cable robot measurements of a single actuator onto
   * data.log file.
   * @param[in] actuator_id ID of inquired actuator.
   */
  void CollectAndDumpMeas(const id_t actuator_id);
  /**
   * @brief Start data logging inside real-time thread cycle.
   * @param[in] rt_cycle_multiplier Factor definining the number of cycles between one log
   * sample and the next one.
   */
  void StartRtLogging(const uint rt_cycle_multiplier);
  /**
   * @brief Stop data logging inside real-time thread cycle.
   */
  void StopRtLogging();
  /**
   * @brief Flush data logs up to now.
   */
  void FlushDataLogs();

  /**
   * @brief Go to home position.
   * @return _True_ if operation was successful, _false_ otherwise.
   */
  bool GoHome();

  /**
   * @brief Set motors controller.
   * @param[in] controller Pointer to a controller.
   * @note The pointer is to the ControllerBase whose virtual methods have to be override
   * by any derived class of it. In particular, ControllerBase::CalcCtrlActions() is
   * called at every cycle of the real time thread while ControllerBase::TargetReached()
   * is typically used asynchronously to inquire robot control status in waiting
   * condition.
   */
  void SetController(ControllerBase* controller);
  /**
   * @brief Wait until controller target is reached.
   * @return 0 if target was reached, a positive number otherwise, yielding the error
   * type.
   */
  RetVal WaitUntilTargetReached(const double max_wait_time_sec = kMaxWaitTimeSec);

  /**
   * @brief WaitUntilPlatformSteady
   * @return
   */
  RetVal WaitUntilPlatformSteady(const double max_wait_time_sec = kMaxWaitTimeSec);

  /**
   * @brief isWaiting
   * @return
   */
  bool isWaiting() const { return is_waiting_; }

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
  /**
   * @brief Signal including motor status, as GSWD input PDOs structure.
   *
   * @note This is a timer-triggered synchronous signal.
   * @see actuatorStatus
   */
  void motorStatus(const id_t&, const grabec::GSWDriveInPdos&) const;
  /**
   * @brief Signal including actuator status.
   *
   * @note This is a timer-triggered synchronous signal.
   * @see motorStatus
   */
  void actuatorStatus(const ActuatorStatus&) const;

  /**
   * @brief Signal including a serialized message to be logged.
   */
  void sendMsg(const QByteArray) const;

  /**
   * @brief Signal including a message to any QConsole, for instance a QTextBrowser.
   */
  void printToQConsole(const QString&) const;

  /**
   * @brief Signal including the changed state of the EtherCAT network.
   */
  void ecStateChanged(const std::bitset<3>&) const;
  /**
   * @brief Signal including the changed state of the real-time thread.
   */
  void rtThreadStatusChanged(const bool) const;

 private slots:
  void forwardPrintToQConsole(const QString&) const;
  void emitMotorStatus();
  void emitActuatorStatus();

 private:
  //-------- Pseudo-signals from EthercatMaster base class (live in RT thread) --------//

  void EcStateChangedCb(const std::bitset<3>& new_state) override final;
  void EcPrintCb(const std::string& msg, const char color = 'w') const override final;
  void EcRtThreadStatusChanged(const bool active) override final;

 private:
  grabcdpr::RobotVars cdpr_status_;
  grabcdpr::RobotParams params_;

  // Timers for status updates
  static constexpr int kMotorStatusIntervalMsec_    = 100;
  static constexpr int kActuatorStatusIntervalMsec_ = 10;
  QTimer* motor_status_timer_                       = nullptr;
  QTimer* actuator_status_timer_                    = nullptr;

  void StopTimers();

  // Data logging
  vect<ActuatorStatusMsg> meas_;
  LogBuffer log_buffer_;
  grabrt::Clock clock_;
  bool rt_logging_enabled_;
  uint rt_logging_mod_;

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

  // Waiting functions
  QMutex qmutex_;
  bool stop_waiting_cmd_recv_ = false;
  bool is_waiting_            = false;

  // RT cyclic steps related
  StateEstimatorBase* state_estimator_ = nullptr;
  ControllerBase* controller_          = nullptr;

  void StateEstimationStep(const bool active_actuators_status_updated);
  void ControlStep(const bool active_actuators_status_updated);

  // Tuning params for detecting platform steadyness
  static constexpr double kBufferingTimeSec_  = 3.0;     // [sec]
  static constexpr double kCutoffFreq_        = 20.0;    // [Hz]
  static constexpr double kMaxAngleDeviation_ = 0.00005; // [rad]

 private:
  //--------- State machine --------------------------------------------------//

  // clang-format off
  static constexpr char* kStatesStr_[] = {
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
