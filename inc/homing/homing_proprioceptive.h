/**
 * @file homing_proprioceptive.h
 * @author Simone Comari
 * @date 17 Jun 2019
 * @brief This file includes classes necessary to implement the homing proprioceptive
 * algorithm.
 */

#ifndef CABLE_ROBOT_HOMING_PROPRIOCEPTIVE_H
#define CABLE_ROBOT_HOMING_PROPRIOCEPTIVE_H

#include <QObject>
#include <QString>

#include "json.hpp"

#include "StateMachine.h"
#include "easylogging++.h"

#include "ctrl/controller_singledrive.h"
#include "homing/matlab_thread.h"
#include "robot/cablerobot.h"
#include "utils/types.h"

#define HOMING_ACK true

using json = nlohmann::json; /**< Alias for JSON library support. */

/**
 * @brief The HomingProprioceptiveStartData class is a derived class of EventData, which
 * is used to pass initial data to the transition event handler when starting the homing
 * procedure.
 */
class HomingProprioceptiveStartData: public EventData
{
 public:
  /**
   * @brief HomingProprioceptiveStartData default constructor.
   *
   * When using this constructor make sure to manually assign values to this class
   * attributes before using it.
   */
  HomingProprioceptiveStartData();
  /**
   * @brief HomingProprioceptiveStartData
   * @param[in] _init_torques See init_torques.
   * @param[in] _max_torques See max_torques.
   * @param[in] _num_meas See num_meas.
   */
  HomingProprioceptiveStartData(const vect<qint16>& _init_torques,
                                const vect<qint16>& _max_torques, const quint8 _num_meas);

  /**
   * @brief init_torques Initial torque values to assign to each drives at the beginning
   * of homing procedure. At the end of the procedure the configuration of the robot at
   * this point will become the homing position.
   */
  vect<qint16> init_torques;
  /**
   * @brief max_torques Maximum torque values that each drive will reach at the end of
   * the coiling phase.
   */
  vect<qint16> max_torques;
  /**
   * @brief num_meas Number of measurments during the coiling phase.
   * @note The total number of measurements for each active cable is 2 * _num_meas - 1,
   * including both coiling and uncoiling phase.
   */
  quint8 num_meas;
};

/**
 * @brief operator << to nicely print HomingProprioceptiveStartData.
 * @param[in] stream
 * @param[in] data
 * @return New stream with HomingProprioceptiveStartData information.
 */
std::ostream& operator<<(std::ostream& stream, const HomingProprioceptiveStartData& data);


/**
 * @brief The HomingProprioceptiveHomeData class is a derived class of EventData, which
 * is used to pass homing data to the transition event handler when moving to the final
 * homing phase, which completes the homing procedure.
 */
class HomingProprioceptiveHomeData: public EventData
{
 public:
  /**
   * @brief init_lengths Cable lengths at homing position, resulting from optimization
   * step.
   */
  vect<double> init_lengths;
  /**
   * @brief init_angles Swivel pulley angles at homing position, resulting from
   * optimization step.
   */
  vect<double> init_angles;
};

/**
 * @brief operator << to nicely print HomingProprioceptiveHomeData.
 * @param[in] stream
 * @param[in] data
 * @return New stream with HomingProprioceptiveHomeData information.
 */
std::ostream& operator<<(std::ostream& stream, const HomingProprioceptiveHomeData& data);


/**
 * @brief This class implements the homing proprioceptive algorithm.
 *
 * This class follows a state machine structure and works jointly with its parent, the
 * corresponding widget HomingProprioceptiveInterface, which triggers all transitions,
 * both manually (from user end) and automatically, like in the acquisition phase.
 *
 * More details about the states and transition events and policy can be found in the
 * relative documentation in a more friendly and schematic format.
 */
class HomingProprioceptive: public QObject, public StateMachine
{
  Q_OBJECT

 public:
  /**
   * @brief HomingProprioceptive
   * @param[in] parent The parent Qt object, in this case the corresponding interface.
   * @param[in] robot Pointer to cable robot instance, to access robot features and
   * commands.
   */
  HomingProprioceptive(QObject* parent, CableRobot* robot);
  ~HomingProprioceptive() override;

  /**
   * @brief This class' states enum
   */
  enum States : BYTE
  {
    ST_IDLE,
    ST_ENABLED,
    ST_START_UP,
    ST_SWITCH_CABLE,
    ST_COILING,
    ST_UNCOILING,
    ST_OPTIMIZING,
    ST_HOME,
    ST_FAULT,
    ST_MAX_STATES
  };

  /**
   * @brief Check if robot is in acquisition phase.
   * @return _True_ if robot is collecting data, _false_ otherwise.
   */
  bool IsCollectingData();

  /**
   * @brief Get the active actuators IDs.
   * @return The active actuators IDs.
   */
  vect<id_t> GetActuatorsID() const { return active_actuators_id_; }

  /**
   * @brief Get status of active actuator corresponding to given ID.
   * @param[in] id The ID of the inquired actuator.
   * @return The status of the inquired actuator.
   */
  ActuatorStatus GetActuatorStatus(const id_t id);

  /**
   * @brief Parse an external file with homing data and fill the relative structure.
   * @param[in] filepath The filepath of the external file with the homing data, resulting
   * from an optimization process.
   * @param[out] home_data
   * @return _True_ if parsing was successful, _false_ otherwise
   */
  bool ParseExtFile(const QString& filepath, HomingProprioceptiveHomeData* home_data);

 public:
  //--------- External events -------------------------------------------------------//

  /**
   * @brief Start command, moves to next state in line.
   *
   * Triggers the following transitions:
   * - IDLE --> ENABLED
   * - ENABLED --> START_UP
   * - START_UP --> SWITCH_CABLE
   * - SWITCH_CABLE --> COILING
   * - COILING --> COILING (repeat)
   * - UNCOILING --> UNCOILING (repeat)
   * @param[in] data Data needed for starting the acquisition phase. This is only
   * necessary for 2nd transition (ENABLED --> START_UP), ignored otherwise (set to NULL).
   */
  void Start(HomingProprioceptiveStartData* data);
  /**
   * @brief Stop command, stop on going phase, if stoppable (like acquisition one).
   *
   * This command triggers a transition to ENABLED.
   */
  void Stop();
  /**
   * @brief Stop on going phase and disable motors.
   *
   * This command triggers a transition to IDLE.
   */
  void Disable();
  /**
   * @brief Optimize command, triggers a transition from ENABLED to OPTIMIZING, starting
   * this phase.
   */
  void Optimize();
  /**
   * @brief Go home command, moves the robot to home position and assign newly computed
   * homing values (cable lengths and swivel angles values at homing position).
   * @param[in] data
   */
  void GoHome(HomingProprioceptiveHomeData* data);
  /**
   * @brief Fault trigger.
   */
  void FaultTrigger();
  /**
   * @brief Fault reset command.
   */
  void FaultReset();

 signals:
  /**
   * @brief Signal including a message to any QConsole, for instance a QTextBrowser.
   */
  void printToQConsole(const QString&) const;
  /**
   * @brief Acquisition complete signal.
   */
  void acquisitionComplete() const;
  /**
   * @brief Homing complete signal.
   */
  void homingComplete() const;
  /**
   * @brief Signal including the changed state of this object.
   */
  void stateChanged(const quint8&) const;
  /**
   * @brief Signal including an operation progress value, from 0 to 100.
   */
  void progressValue(const int&) const;
  /**
   * @brief Stop waiting command.
   */
  void stopWaitingCmd() const;

 private slots:
  void handleActuatorStatusUpdate(const ActuatorStatus& actuator_status);
  void handleMatlabResultsReady();
  void updateOptimizationProgress();

 private:
  CableRobot* robot_ptr_ = nullptr;
  ControllerSingleDrive controller_;

  static constexpr size_t kNumMeasMin_     = 1;
  static constexpr qint16 kTorqueSsErrTol_ = 5;
  static constexpr double kPositionStepTransTime_ = 3.0;
  size_t num_meas_                         = kNumMeasMin_;
  size_t num_tot_meas_;
  size_t working_actuator_idx_;
  size_t meas_step_;
  vect<qint16> init_torques_;
  vect<qint16> max_torques_;
#if HOMING_ACK
  vectI positions_;
#else
  vect<qint16> torques_;
  vect<qint32> reg_pos_;
#endif

  bool disable_cmd_recv_;
  QMutex qmutex_;

  vect<id_t> active_actuators_id_;
  vect<ActuatorStatus> actuators_status_;

  static constexpr int kOptProgressIntervalMsec_ = 150;
  QTimer optimization_progess_timer_;
  int optimization_progress_counter_;

  void DumpMeasAndMoveNext();

 private:
  //--------- State machine ---------------------------------------------------------//

  // clang-format off
  static constexpr char* kStatesStr[] = {
    const_cast<char*>("IDLE"),
    const_cast<char*>("ENABLED"),
    const_cast<char*>("START_UP"),
    const_cast<char*>("SWITCH_CABLE"),
    const_cast<char*>("COILING"),
    const_cast<char*>("UNCOILING"),
    const_cast<char*>("OPTIMIZING"),
    const_cast<char*>("GO_HOME"),
    const_cast<char*>("FAULT")};
  // clang-format on

  States prev_state_;

  // Define the state machine state functions with event data type
  GUARD_DECLARE(HomingProprioceptive, GuardIdle, NoEventData)
  STATE_DECLARE(HomingProprioceptive, Idle, NoEventData)
  GUARD_DECLARE(HomingProprioceptive, GuardEnabled, NoEventData)
  STATE_DECLARE(HomingProprioceptive, Enabled, NoEventData)
  STATE_DECLARE(HomingProprioceptive, StartUp, HomingProprioceptiveStartData)
  GUARD_DECLARE(HomingProprioceptive, GuardSwitch, NoEventData)
  STATE_DECLARE(HomingProprioceptive, SwitchCable, NoEventData)
  ENTRY_DECLARE(HomingProprioceptive, EntryCoiling, NoEventData)
  STATE_DECLARE(HomingProprioceptive, Coiling, NoEventData)
  STATE_DECLARE(HomingProprioceptive, Uncoiling, NoEventData)
  STATE_DECLARE(HomingProprioceptive, Optimizing, NoEventData)
  STATE_DECLARE(HomingProprioceptive, Home, HomingProprioceptiveHomeData)
  STATE_DECLARE(HomingProprioceptive, Fault, NoEventData)

  // State map to define state object order
  BEGIN_STATE_MAP_EX
  // clang-format off
    STATE_MAP_ENTRY_ALL_EX(&Idle, &GuardIdle, nullptr, nullptr)
    STATE_MAP_ENTRY_ALL_EX(&Enabled, &GuardEnabled, nullptr, nullptr)
    STATE_MAP_ENTRY_EX(&StartUp)
    STATE_MAP_ENTRY_ALL_EX(&SwitchCable, &GuardSwitch, nullptr, nullptr)
    STATE_MAP_ENTRY_ALL_EX(&Coiling, nullptr, &EntryCoiling, nullptr)
    STATE_MAP_ENTRY_EX(&Uncoiling)
    STATE_MAP_ENTRY_EX(&Optimizing)
    STATE_MAP_ENTRY_EX(&Home)
    STATE_MAP_ENTRY_EX(&Fault)
  // clang-format on
  END_STATE_MAP_EX

  void PrintStateTransition(const States current_state, const States new_state) const;
};

#endif // CABLE_ROBOT_HOMING_PROPRIOCEPTIVE_H
