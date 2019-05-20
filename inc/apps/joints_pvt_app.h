#ifndef CABLE_ROBOT_JOINTS_PVT_APP_H
#define CABLE_ROBOT_JOINTS_PVT_APP_H

#include <QObject>
#include <QTextStream>

#include "StateMachine.h"
#include "easylogging++.h"

#include "ctrl/controller_joints_pvt.h"
#include "robot/cablerobot.h"

struct TrajectorySet
{
  ControlMode traj_type;
  vect<TrajectoryD> traj_platform;
  vect<TrajectoryD> traj_cables_len;
  vect<TrajectoryI> traj_motors_pos;
  vect<TrajectoryI> traj_motors_vel;
  vect<TrajectoryS> traj_motors_torque;
};

class JointsPVTAppData: public EventData
{
 public:
  JointsPVTAppData();
  JointsPVTAppData(const int _traj_idx);

  int traj_idx = 0;
};


class JointsPVTApp: public QObject, public StateMachine
{
  Q_OBJECT

 public:
  explicit JointsPVTApp(QObject* parent, CableRobot* robot,
                        const vect<grabcdpr::ActuatorParams>& params);
  ~JointsPVTApp() override;

  void pause();
  bool isPaused() { return controller_.IsPaused(); }

  const TrajectorySet& getTrajectorySet(const int traj_idx) const;

 public:
  //--------- External events -------------------------------------------------------//

  void clearAllTrajectories();
  bool readTrajectories(const QString& ifilepath);
  void runTransition(const int traj_idx);
  void sendTrajectories(const int traj_idx);
  void stop();

 signals:
  /**
   * @brief Signal including a message to any QConsole, for instance a QTextBrowser.
   */
  void printToQConsole(const QString&) const;

  void transitionComplete() const;

  void trajectoryComplete() const;

  void trajectoryProgress(const int) const;
  /**
   * @brief Stop waiting command.
   */
  void stopWaitingCmd() const;

 private slots:
  // For signals emitted by controller
  void handleTrajectoryCompleted();
  void progressUpdate(const int progress_value);

  void logInfo(const QString&text) const;

 private:
  CableRobot* robot_ptr_;
  ControllerJointsPVT controller_;

  QVector<TrajectorySet> traj_sets_;

  void setCablesLenTraj(const bool relative, const vect<id_t>& motors_id, QTextStream& s,
                        TrajectorySet& traj_set);
  void setMotorPosTraj(const bool relative, const vect<id_t>& motors_id, QTextStream& s,
                       TrajectorySet& traj_set);
  void setMotorVelTraj(const vect<id_t>& motors_id, QTextStream& s,
                       TrajectorySet& traj_set);
  void setMotorTorqueTraj(const bool relative, const vect<id_t>& motors_id,
                          QTextStream& s, TrajectorySet& traj_set);

 private:
  //--------- State machine ---------------------------------------------------------//

  enum States : BYTE
  {
    ST_IDLE,
    ST_READY,
    ST_TRANSITION,
    ST_TRAJECTORY_FOLLOW,
    ST_MAX_STATES
  };

  // clang-format off
  static constexpr char* kStatesStr[] = {
    const_cast<char*>("IDLE"),
    const_cast<char*>("READY"),
    const_cast<char*>("TRANSITION"),
    const_cast<char*>("TRAJECTORY_FOLLOW")};
  // clang-format on

  States prev_state_;

  STATE_DECLARE(JointsPVTApp, Idle, NoEventData)
  STATE_DECLARE(JointsPVTApp, Ready, NoEventData)
  STATE_DECLARE(JointsPVTApp, Transition, JointsPVTAppData)
  STATE_DECLARE(JointsPVTApp, TrajectoryFollow, JointsPVTAppData)

  // State map to define state object order
  BEGIN_STATE_MAP
  // clang-format off
    STATE_MAP_ENTRY({&Idle})
    STATE_MAP_ENTRY({&Ready})
    STATE_MAP_ENTRY({&Transition})
    STATE_MAP_ENTRY({&TrajectoryFollow})
  // clang-format on
  END_STATE_MAP

  void PrintStateTransition(const States current_state, const States new_state) const;
};

#endif // CABLE_ROBOT_JOINTS_PVT_APP_H
