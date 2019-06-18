#ifndef MANUAL_CONTROL_APP_H
#define MANUAL_CONTROL_APP_H

#include <QObject>
#include <QTextStream>

#include "StateMachine.h"

#include "ctrl/controller_joints_pvt.h"
#include "robot/cablerobot.h"

class MyData: public EventData
{
 public:
  MyData();
  MyData(const qint16 _torque);

  qint16 torque;
};

std::ostream& operator<<(std::ostream& stream, const MyData& data);


class ManualControlApp: public QObject, public StateMachine
{
  Q_OBJECT

 public:
  explicit ManualControlApp(QObject* parent, CableRobot* robot = nullptr);
  ~ManualControlApp() override;

  enum States : BYTE
  {
    ST_IDLE,
    ST_ENABLED,
    ST_POS_CONTROL,
    ST_TORQUE_CONTROL,
    ST_LOGGING,
    ST_MAX_STATES
  };

  void enable(MyData* data = nullptr);
  void changeControlMode();
  void changeControlMode(MyData* data);
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

  STATE_DECLARE(ManualControlApp, Idle, NoEventData)
  GUARD_DECLARE(ManualControlApp, GuardEnabled, NoEventData)
  STATE_DECLARE(ManualControlApp, Enabled, NoEventData)
  STATE_DECLARE(ManualControlApp, PosControl, NoEventData)
  STATE_DECLARE(ManualControlApp, TorqueControl, MyData)
  STATE_DECLARE(ManualControlApp, Logging, NoEventData)
  EXIT_DECLARE(ManualControlApp, ExitLogging)

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

  void PrintStateTransition(const States current_state, const States new_state) const;
};

#endif // MANUAL_CONTROL_APP_H
