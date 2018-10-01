#ifndef CABLE_ROBOT_WINCH_H
#define CABLE_ROBOT_WINCH_H

#include "slaves/goldsolowhistledrive.h"
#include "types.h"
#include "state_machine/inc/StateMachine.h"

class Cable
{
public:
  Cable() {}

  void SetHomeLength(const double length) { home_length_ = length; }
  void SetHomeLengthTrue(const double length) { home_length_true_ = length; }

  void UpdateCableLen(const double delta_length);

private:
  double home_length_ = 0.0;
  double length_ = 0.0;
  double home_length_true_ = 0.0;
};

/**
 * @brief The Winch class
 */
class Winch : StateMachine
{
public:
  /**
   * @brief Winch
   * @param slave_position
   */
  Winch(const uint8_t slave_position);

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  //// External events
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * @brief Enable
   */
  void Enable();
  /**
   * @brief Disable
   */
  void Disable();
  /**
   * @brief FaultReset
   */
  void FaultReset();

  /**
   * @brief GetServo
   * @return
   */
  const grabec::GoldSoloWhistleDrive* GetServo() const { return &servo_; }
  /**
   * @brief GetCable
   * @return
   */
  const Cable* GetCable() const { return &cable_; }

  /**
   * @brief SetServoPosByCableLen
   * @param target_length
   */
  void SetServoPosByCableLen(const double target_length);
  /**
   * @brief SetServoSpeed
   * @param target_speed
   */
  void SetServoSpeed(const int32_t target_speed);
  /**
   * @brief SetServoTorque
   * @param target_torque
   */
  void SetServoTorque(const int16_t target_torque);

  /**
   * @brief UpdateHomeConfig
   * @param cable_len
   * @param pulley_angle
   * @param cable_len_true
   */
  void UpdateHomeConfig(const double cable_len, const double pulley_angle,
                        const double cable_len_true);
  /**
   * @brief UpdateStartConfig
   */
  void UpdateStartConfig();
  /**
   * @brief UpdateConfig
   */
  void UpdateConfig();

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

private:
  static constexpr uint8_t kMaxTransitionCounter_ = 100;
  // clang-format off
  static constexpr char* kStatesStr[] = {
    const_cast<char*>("IDLE"),
    const_cast<char*>("ENABLED"),
    const_cast<char*>("FAULT"),
    const_cast<char*>("MAX_STATE")
  };
  // clang-format on

  enum States : BYTE
  {
    ST_IDLE,
    ST_ENABLED,
    ST_FAULT,
    ST_MAX_STATES
  };

  uint8_t slave_position_;

  WinchParams params_;
  grabec::GoldSoloWhistleDrive servo_;
  Cable cable_;
  Pulley pulley_;

  int servo_home_pos_ = 0;
  int servo_start_pos_ = 0;

  States prev_state_ = ST_IDLE;

  // Define the state machine state functions with event data type
  STATE_DECLARE(Winch, Idle, NoEventData)
  GUARD_DECLARE(Winch, GuardIdle, NoEventData)
  STATE_DECLARE(Winch, Enabled, NoEventData)
  GUARD_DECLARE(Winch, GuardEnabled, NoEventData)
  STATE_DECLARE(Winch, Fault, NoEventData)
  GUARD_DECLARE(Winch, GuardFault, NoEventData)

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

#endif // CABLE_ROBOT_WINCH_H
