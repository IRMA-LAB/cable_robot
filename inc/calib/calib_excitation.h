/**
 * @file calib_excitation.h
 * @author Simone Comari
 * @date 14 May 2020
 * @brief This file includes the calibration support application when using an external
 * tracking system, such as Vicon.
 */

#ifndef CABLE_ROBOT_CALIB_EXCITATION_H
#define CABLE_ROBOT_CALIB_EXCITATION_H

#include <QObject>
#include <QTextStream>

#include "StateMachine.h"

#include "ctrl/controller_joints_pvt.h"
#include "robot/cablerobot.h"

/**
 * @brief The EventData derived class for CalibExcitation state machine.
 *
 * This data includes a torque value to be use when transitioning to torque control mode.
 */
class CalibExcitationData: public EventData
{
 public:
  /**
   * @brief Default constructor.
   *
   * This can be used to manually set torque value and/or trajectory filepath like in a
   * structure.
   */
  CalibExcitationData();
  /**
   * @brief Constructor for torque use case.
   * @param _torque Target torque value in nominal points to be assigned to all drives
   * when transitioning to torque control mode (i.e. freedrive mode).
   */
  CalibExcitationData(const qint16 _torque);
  /**
   * @brief Constructor for trajectory use case.
   * @param filepath Absolute path of trajectory text file.
   */
  CalibExcitationData(const QString& filepath);

  qint16 torque = 0; /**< Target torque value in nominal points. */
  QString traj_filepath; /**< Absolute path of trajectory text file. */
};

/**
 * @brief operator <<
 * @param stream
 * @param data Basically the torque value
 * @return Stream with current torque value in data.
 */
std::ostream& operator<<(std::ostream& stream, const CalibExcitationData& data);


/**
 * @brief The calibration support class when using an external tracking system.
 *
 * This class takes care of controlling the robot when performing a calibration through
 * an external tracking system, such as Vicon.
 * The support procedure is the following:
 * 1. Enable all motors;
 * 2. Manually move the platform in a desired position in freedrive mode (i.e. torque
 * control mode).
 * 3. Switch to position control to fix the cable length.
 * 4. Start logging actuators status while performing a predefined trajectory which should
 * excite most platform dynamics.
 */
class CalibExcitation: public QObject, public StateMachine
{
  Q_OBJECT

 public:
  /**
   * @brief Full constructor.
   * @param parent The parent QObject, in this case the corresponding interface.
   * @param robot A pointer to the cable robot object.
   * @param[in] params A vector of actuator parameters, with as many elements as the
   * active motors.
   */
  explicit CalibExcitation(QObject* parent, CableRobot* robot,
                           const vect<grabcdpr::ActuatorParams>& params);
  ~CalibExcitation() override;

  /**
   * @brief The States enum of this class' state machine
   */
  enum States : BYTE
  {
    ST_IDLE,
    ST_ENABLED,
    ST_POS_CONTROL,
    ST_TORQUE_CONTROL,
    ST_LOGGING,
    ST_MAX_STATES
  };

  /**
   * @brief Attempt to enable all active motors.
   * @param data Optional torque target for all motors, in case torque control mode is
   * selected upon enable call.
   */
  void enable(CalibExcitationData* data = nullptr);
  /**
   * @brief Change control mode to position control.
   */
  void changeControlMode();
  /**
   * @brief Change control mode to torque control.
   * @param data Torque target for all motors.
   */
  void changeControlMode(CalibExcitationData* data);
  /**
   * @brief Start logging while following an excitation trajectory.
   * @param data Filepath containg the excitation trajectories for each drive.
   * @note The operation starts as soon as the platform is considered to be steady.
   */
  void exciteAndLog(CalibExcitationData* data);
  /**
   * @brief Disable all active motors.
   */
  void disable();

 signals:
  /**
   * @brief Signal including a message to any QConsole, for instance a QTextBrowser.
   */
  void printToQConsole(const QString&) const;
  /**
   * @brief Signal including the changed state of this object.
   */
  void stateChanged(const quint8&) const;
  /**
   * @brief Stop waiting command.
   */
  void stopWaitingCmd() const;

 private slots:
  void stopLogging();

 private:
  static constexpr qint16 kTorqueSsErrTol_ = 5;

  CableRobot* robot_ptr_ = nullptr;
  ControllerSingleDrive controller_single_drive_;
  ControllerJointsPVT* controller_joints_ptv_ = nullptr;

  bool disable_cmd_recv_ = false;
  QMutex qmutex_;

  vect<id_t> active_actuators_id_;

  static constexpr uint kRtCycleMultiplier_ = 10; // logging T = cycle_time * multiplier
  vect<TrajectoryD> traj_cables_len_;
  bool readTrajectories(const QString& ifilepath);
  void setCablesLenTraj(const bool relative, const vect<id_t>& motors_id, QTextStream& s);

 private:
  //--------- State machine ---------------------------------------------------------//

  // clang-format off
  static constexpr char* kStatesStr[] = {
    const_cast<char*>("IDLE"),
    const_cast<char*>("ENABLED"),
    const_cast<char*>("POS_CONTROL"),
    const_cast<char*>("TORQUE_CONTROL"),
    const_cast<char*>("LOGGING")};
  // clang-format on

  States prev_state_;

  STATE_DECLARE(CalibExcitation, Idle, NoEventData)
  GUARD_DECLARE(CalibExcitation, GuardEnabled, NoEventData)
  STATE_DECLARE(CalibExcitation, Enabled, NoEventData)
  STATE_DECLARE(CalibExcitation, PosControl, NoEventData)
  STATE_DECLARE(CalibExcitation, TorqueControl, CalibExcitationData)
  GUARD_DECLARE(CalibExcitation, GuardLogging, CalibExcitationData)
  STATE_DECLARE(CalibExcitation, Logging, CalibExcitationData)
  EXIT_DECLARE(CalibExcitation, ExitLogging)

  // State map to define state object order
  BEGIN_STATE_MAP_EX
  // clang-format off
    STATE_MAP_ENTRY_EX(&Idle)
    STATE_MAP_ENTRY_ALL_EX(&Enabled, &GuardEnabled, nullptr, nullptr)
    STATE_MAP_ENTRY_EX(&PosControl)
    STATE_MAP_ENTRY_EX(&TorqueControl)
    STATE_MAP_ENTRY_ALL_EX(&Logging, &GuardLogging, nullptr, &ExitLogging)
  // clang-format on
  END_STATE_MAP_EX

  void printStateTransition(const States current_state, const States new_state) const;
};

#endif // CABLE_ROBOT_CALIB_EXCITATION_H
