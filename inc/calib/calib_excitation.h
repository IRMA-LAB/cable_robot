#ifndef CABLE_ROBOT_CALIB_EXCITATION_H
#define CABLE_ROBOT_CALIB_EXCITATION_H

#include <QObject>
#include <QTextStream>

#include "StateMachine.h"

#include "ctrl/controller_joints_pvt.h"
#include "robot/cablerobot.h"

class CalibExcitationData: public EventData
{
 public:
  CalibExcitationData();
  CalibExcitationData(const qint16 _torque);

  qint16 torque;
};

std::ostream& operator<<(std::ostream& stream, const CalibExcitationData& data);


class CalibExcitation: public QObject, public StateMachine
{
  Q_OBJECT

 public:
  explicit CalibExcitation(QObject* parent, CableRobot* robot = nullptr);
  ~CalibExcitation() override;

  enum States : BYTE
  {
    ST_IDLE,
    ST_ENABLED,
    ST_POS_CONTROL,
    ST_TORQUE_CONTROL,
    ST_LOGGING,
    ST_MAX_STATES
  };

  void enable(CalibExcitationData* data = nullptr);
  void changeControlMode();
  void changeControlMode(CalibExcitationData* data);
  void exciteAndLog();
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
  static const QString kExcitationTrajFilepath_;
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
  STATE_DECLARE(CalibExcitation, Logging, NoEventData)
  EXIT_DECLARE(CalibExcitation, ExitLogging)

  // State map to define state object order
  BEGIN_STATE_MAP_EX
  // clang-format off
    STATE_MAP_ENTRY_EX(&Idle)
    STATE_MAP_ENTRY_ALL_EX(&Enabled, &GuardEnabled, nullptr, nullptr)
    STATE_MAP_ENTRY_EX(&PosControl)
    STATE_MAP_ENTRY_EX(&TorqueControl)
    STATE_MAP_ENTRY_ALL_EX(&Logging, nullptr, nullptr, &ExitLogging)
  // clang-format on
  END_STATE_MAP_EX

  void printStateTransition(const States current_state, const States new_state) const;
};

#endif // CABLE_ROBOT_CALIB_EXCITATION_H
