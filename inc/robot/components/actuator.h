#ifndef CABLE_ROBOT_ACTUATOR_H
#define CABLE_ROBOT_ACTUATOR_H

#include <QObject>

#include "easylogging++.h"
#include "StateMachine.h"
#include "libgrabrt/inc/clocks.h"
#include "libcdpr/inc/types.h"

#include "winch.h"
#include "pulleys_system.h"

/**
 * @brief The Actuator class
 */
class Actuator : public QObject, public StateMachine
{
  Q_OBJECT

public:
  /**
   * @brief Actuator
   * @param[in] slave_position
   * @param[in] params
   */
  Actuator(const id_t id, const uint8_t slave_position,
           const grabcdpr::ActuatorParams& params, QObject* parent = NULL);

  ~Actuator();

  enum States : BYTE
  {
    ST_IDLE,
    ST_ENABLED,
    ST_FAULT,
    ST_MAX_STATES
  };

public slots:
  //--------- External Events Public --------------------------------------------------//

  /**
   * @brief Enable
   */
  void Enable();
  /**
   * @brief Disable
   */
  void Disable();
  /**
   * @brief FaultTrigger
   */
  void FaultTrigger();
  /**
   * @brief FaultReset
   */
  void FaultReset();

public:
  /**
   * @brief ID
   * @return
   */
  id_t ID() const { return id_; }
  /**
   * @brief GetWinch
   * @return
   */
  const Winch& GetWinch() const { return winch_; }
  /**
   * @brief GetWinch
   * @return
   */
  Winch& GetWinch() { return winch_; }
  /**
   * @brief GetPulley
   * @return
   */
  const PulleysSystem& GetPulley() const { return pulley_; }
  /**
   * @brief GetStatus
   * @return
   */
  const ActuatorStatus GetStatus();

  /**
   * @brief SetCableLength
   * @param target_length
   */
  void SetCableLength(const double target_length);
  /**
   * @brief SetMotorPos
   * @param target_speed
   */
  void SetMotorPos(const int32_t target_pos);
  /**
   * @brief SetCableSpeed
   * @param target_speed
   */
  void SetMotorSpeed(const int32_t target_speed);
  /**
   * @brief SetCableTorque
   * @param target_torque
   */
  void SetMotorTorque(const int16_t target_torque);
  /**
   * @brief SetOpMode
   * @param op_mode
   */
  void SetMotorOpMode(const int8_t op_mode);

  /**
   * @brief UpdateHomeConfig
   * @param cable_len
   * @param cable_len_true
   * @param pulley_angle
   */
  void UpdateHomeConfig(const double cable_len, const double pulley_angle);
  /**
   * @brief UpdateConfig
   */
  void UpdateConfig();

  /**
   * @brief IsActive
   * @return
   */
  bool IsActive() const { return active_; }
  /**
   * @brief IsIdle
   * @return
   */
  bool IsIdle() { return GetCurrentState() == ST_IDLE; }
  /**
   * @brief IsEnabled
   * @return
   */
  bool IsEnabled() { return GetCurrentState() == ST_ENABLED; }
  /**
   * @brief IsInFault
   * @return
   */
  bool IsInFault() { return GetCurrentState() == ST_FAULT; }

signals:
  void stateChanged(const id_t&, const BYTE&) const;
  void printToQConsole(const QString&) const;

private:
  id_t id_;
  uint8_t slave_position_;
  bool active_;

  Winch winch_;
  PulleysSystem pulley_;

private:
  static constexpr double kMaxTransitionTimeSec_ = 5.0;
  // clang-format off
  static constexpr char* kStatesStr_[] = {
    const_cast<char*>("IDLE"),
    const_cast<char*>("ENABLED"),
    const_cast<char*>("FAULT"),
    const_cast<char*>("MAX_STATE")
  };
  // clang-format on

  States prev_state_ = ST_IDLE;
  grabrt::Clock clock_;

  // Define the state machine state functions with event data type
  STATE_DECLARE(Actuator, Idle, NoEventData)
  GUARD_DECLARE(Actuator, GuardIdle, NoEventData)
  STATE_DECLARE(Actuator, Enabled, NoEventData)
  GUARD_DECLARE(Actuator, GuardEnabled, NoEventData)
  STATE_DECLARE(Actuator, Fault, NoEventData)
  GUARD_DECLARE(Actuator, GuardFault, NoEventData)

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
