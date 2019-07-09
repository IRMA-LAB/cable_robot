/**
 * @file actuator.h
 * @author Simone Comari, Edoardo Idà
 * @date 11 Mar 2019
 * @brief File containing the virtualization of a single actuator of the cable robot.
 */

#ifndef CABLE_ROBOT_ACTUATOR_H
#define CABLE_ROBOT_ACTUATOR_H

#include <QObject>

#include "StateMachine.h"
#include "easylogging++.h"
#include "libcdpr/inc/types.h"
#include "libgrabrt/inc/clocks.h"

#include "pulleys_system.h"
#include "winch.h"

using GSWDStates = grabec::GoldSoloWhistleDriveStates; /**< Shortcut for GSWD states. */

/**
 * @brief The cable robot actuator class.
 *
 * Each actuator of GRAB CDPR is composed of a winch and a pulleys system.
 * This class provides the basic commands to interacts with these components, such as
 * motor target settings and configuration updates.
 *
 * Moreover, like the cable robot itself, also the actuator has its own state machine,
 * which is directly linked to its motor state, mapped in a simplified state here.
 * Therefore external transition events are available here too.
 */
class Actuator: public QObject, public StateMachine
{
  Q_OBJECT

 public:
  /**
   * @brief Actuator constructor.
   * @param[in] id Motor ID.
   * @param[in] slave_position Position of this ethercat slave in the network, more
   * precisely of the GoldSoloWhistle drive, which is one of its components.
   * @param[in] params Configuration parameters describing assembly details and technical
   * information about its components.
   * @param[in,out] parent The parent QObject, in this case the cable robot.
   */
  Actuator(const id_t id, const uint8_t slave_position,
           const grabcdpr::ActuatorParams& params, QObject* parent = nullptr);

  ~Actuator();

  /**
   * @brief The actuator states enum
   */
  enum States : BYTE
  {
    ST_IDLE,
    ST_ENABLED,
    ST_FAULT,
    ST_MAX_STATES
  };

 public:
  /**
   * @brief Return actuator ID.
   * @return Actuator ID.
   */
  id_t ID() const { return id_; }
  /**
   * @brief Get a constant reference to the winch component of the actuator.
   * @return A constant reference to the winch component of the actuator.
   */
  const Winch& GetWinch() const { return winch_; }
  /**
   * @brief Get a reference to the winch component of the actuator.
   * @return A reference to the winch component of the actuator.
   */
  Winch& GetWinch() { return winch_; }
  /**
   * @brief Get a constant reference to the pulleys system component of the actuator.
   * @return A constant reference to the pulleys system component of the actuator.
   */
  const PulleysSystem& GetPulley() const { return pulley_; }
  /**
   * @brief Get actuator status.
   * @return Most recent actuator status.
   */
  const ActuatorStatus GetStatus();

  /**
   * @brief Set cable length target and change its operational mode to CYCLIC_POSITION.
   *
   * The cable length target is then converted in motor encoder counts, by static winch
   * parameters and homing position record.
   * @param[in] target_length Cable length target in meters.
   */
  void SetCableLength(const double target_length);
  /**
   * @brief Set motor position and change its operational mode to CYCLIC_POSITION.
   * @param[in] target_pos Target motor position in motor encoder counts.
   */
  void SetMotorPos(const int32_t target_pos);
  /**
   * @brief Set motor speed and change its operational mode to CYCLIC_VELOCITY.
   * @param[in] target_speed Target motor speed in motor encoder counts/seconds.
   */
  void SetMotorSpeed(const int32_t target_speed);
  /**
   * @brief Set motor torque and change its operational mode to CYCLIC_TORQUE.
   * @param[in] target_torque Target motor torque in per thousand nominal points.
   * @note A negative value corresponds to a _pulling_ torque.
   */
  void SetMotorTorque(const int16_t target_torque);
  /**
   * @brief Set motor operational mode.
   * @param[in] op_mode Desired motor operational mode.
   */
  void SetMotorOpMode(const int8_t op_mode);

  /**
   * @brief Update actuator configuration at home position.
   *
   * Home configuration is nothing but the definition of actuator configuration in terms
   * of cable length and swivel pulley angle for a specific set of encoder values of motor
   * and pulley encoders respectively.
   * This unique correspondence defines a fixed reference for the movements of the cable
   * for this particular actuator.
   * @param[in] cable_len _[m]_ Cable length corresponding to the recorded home position
   * in motor counts.
   * @param[in] pulley_angle _[rad]_ Swivel pulley angle corresponding to the recorded
   * home position in encoder counts.
   */
  void UpdateHomeConfig(const double cable_len, const double pulley_angle);
  /**
   * @brief Update actuator configuration using internal most recent measurements.
   *
   * Actuator configuration consists in particular of cable length and swivel angle.
   */
  void UpdateConfig();

  /**
   * @brief Inquire if actuator is active.
   * @return _True_ if actuator is active, _false_ otherwise.
   */
  bool IsActive() const { return active_; }
  /**
   * @brief Inquire if actuator is idle.
   * @return _True_ if actuator is idle, _false_ otherwise.
   */
  bool IsIdle() { return GetCurrentState() == ST_IDLE; }
  /**
   * @brief Inquire if actuator is enabled.
   * @return _True_ if actuator is enabled, _false_ otherwise.
   */
  bool IsEnabled() { return GetCurrentState() == ST_ENABLED; }
  /**
   * @brief Inquire if actuator is in fault.
   * @return _True_ if actuator is in fault, _false_ otherwise.
   */
  bool IsInFault() { return GetCurrentState() == ST_FAULT; }

  /**
   * @brief Map DriveState to ActuatorState.
   * @param[in] drive_state
   * @return Corresponding ActuatorState of drive_state.
   */
  static Actuator::States DriveState2ActuatorState(const GSWDStates drive_state);

 public slots:
  //--------- External Events Public --------------------------------------------------//

  /**
   * @brief Enable command.
   *
   * Triggers following transition:
   * - IDLE --> ENABLED
   */
  void enable();
  /**
   * @brief Disable command.
   *
   * Triggers following transition:
   * - ENABLED --> IDLE
   */
  void disable();
  /**
   * @brief Fault trigger.
   *
   * Triggers following transition:
   * - IDLE    --> FAULT
   * - ENABLED --> FAULT
   */
  void faultTrigger();
  /**
   * @brief Fault reset command.
   *
   * Triggers following transition:
   * - FAULT --> IDLE
   */
  void faultReset();

 signals:
  /**
   * @brief Signal including the changed state of this object.
   */
  void stateChanged(const id_t&, const BYTE&) const;
  /**
   * @brief Signal including a message to any QConsole, for instance a QTextBrowser.
   */
  void printToQConsole(const QString&) const;

 private slots:
  void logServoMsg(const QString& msg) { CLOG(INFO, "event") << msg; }
  void forwardServoPrintMsg(const QString& msg) { emit printToQConsole(msg); }

 private:
  id_t id_;
  uint8_t slave_position_;
  bool active_;

  Winch winch_;
  PulleysSystem pulley_;

 private:
  //--------- State machine --------------------------------------------------//

  static constexpr double kMaxTransitionTimeSec_ = 5.0;
  static constexpr uint64_t kWaitCycleTimeNsec_  = 100000000UL; // = 100 ms
  // clang-format off
  static constexpr char* kStatesStr_[] = {
    const_cast<char*>("IDLE"),
    const_cast<char*>("ENABLED"),
    const_cast<char*>("FAULT"),
    const_cast<char*>("MAX_STATE")
  };
  // clang-format on

  States prev_state_;
  grabrt::ThreadClock clock_;

  // Define the state machine state functions with event data type
  GUARD_DECLARE(Actuator, GuardIdle, NoEventData)
  STATE_DECLARE(Actuator, Idle, NoEventData)
  GUARD_DECLARE(Actuator, GuardEnabled, NoEventData)
  STATE_DECLARE(Actuator, Enabled, NoEventData)
  GUARD_DECLARE(Actuator, GuardFault, NoEventData)
  STATE_DECLARE(Actuator, Fault, NoEventData)

  // State map to define state object order. Each state map entry defines a state object.
  BEGIN_STATE_MAP_EX
  // clang-format off
    STATE_MAP_ENTRY_ALL_EX(&Idle, &GuardIdle, 0, 0)
    STATE_MAP_ENTRY_ALL_EX(&Enabled, &GuardEnabled, 0, 0)
    STATE_MAP_ENTRY_ALL_EX(&Fault, &GuardFault, 0, 0)
  // clang-format on
  END_STATE_MAP_EX

  void PrintStateTransition(const States current_state) const;
};

#endif // CABLE_ROBOT_ACTUATOR_H
