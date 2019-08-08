/**
 * @file joints_pvt_app.h
 * @author Simone Comari
 * @date 03 Jul 2019
 * @brief This file includes the implementation of the joints pvt app.
 */

#ifndef CABLE_ROBOT_JOINTS_PVT_APP_H
#define CABLE_ROBOT_JOINTS_PVT_APP_H

#include <QObject>
#include <QTextStream>

#include "StateMachine.h"
#include "easylogging++.h"

#include "ctrl/controller_joints_pvt.h"
#include "robot/cablerobot.h"

/**
 * @brief A convenient structure to include a generic joints trajectory type.
 *
 * The trajectory type defines which trajectory is not empty.
 * @note The platform trajectory is independent from the trajectory type.
 */
struct TrajectorySet
{
  ControlMode traj_type; /**< Trajectory type, defining which one is not empty. */
  vect<TrajectoryD> traj_platform;   /**< Platform trajectory in global 3D coordinates. */
  vect<TrajectoryD> traj_cables_len; /**< Cable lengths trajectories, one per actuator. */
  vect<TrajectoryI> traj_motors_pos; /**< Motor positions trajectories, one per motor. */
  vect<TrajectoryI> traj_motors_vel; /**< Motor velocities trajectories, one per motor. */
  vect<TrajectoryS> traj_motors_torque; /**< Motor torques trajectories, one per motor. */
};


/**
 * @brief The EventData derived class for JointsPVTApp state machine.
 *
 * This data includes the index of the next trajectories to be executed in the set.
 */
class JointsPVTAppData: public EventData
{
 public:
  /**
   * @brief Default constructor.
   *
   * This can be used to manually set the index value like in a structure.
   */
  JointsPVTAppData();
  /**
   * @brief Full constructor.
   * @param _traj_idx The index of the next trajectories to be executed in the set.
   */
  JointsPVTAppData(const int _traj_idx);

  int traj_idx = 0; /**< the index of the next trajectories to be executed in the set. */
};


/**
 * @brief The JointsPVTApp class
 */
class JointsPVTApp: public QObject, public StateMachine
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
  explicit JointsPVTApp(QObject* parent, CableRobot* robot,
                        const vect<grabcdpr::ActuatorParams>& params);
  ~JointsPVTApp() override;

  /**
   * @brief Pause trajectory following or transition phase.
   */
  void pause();
  /**
   * @brief Check if trajectory following or transition phase is pause.
   * @return _True_ if pause, _False_ otherwise.
   */
  bool isPaused() { return controller_.isPaused(); }

  /**
   * @brief Get a trajectory set given the index within the list of parsed trajectories.
   * @param traj_idx The index of desired trajectory set in list.
   * @return The desired trajectory set.
   */
  const TrajectorySet& getTrajectorySet(const int traj_idx) const;

 public:
  //--------- External events -------------------------------------------------------//

  /**
   * @brief Clear all stored trajectories.
   *
   * This triggers following state transition:
   * ST_READY --> ST_IDLE
   */
  void clearAllTrajectories();
  /**
   * @brief Read new trajectories from a file and append it to the list.
   *
   * This triggers following state transition:
   * any --> ST_READY
   * @param ifilepath The location of the text file containing the trajectory.
   * @return _True_ if the parsing was successful, _False_ otherwise.
   */
  bool readTrajectories(const QString& ifilepath);
  /**
   * @brief Run i-th transition, that is the one preceeding the i-th trajectory set in
   * list.
   *
   * This triggers following state transitions:
   * ST_READY --> ST_TRANSITION
   * ST_TRAJECTORY_FOLLOW --> ST_TRANSITION
   * @param traj_idx The index of next trajectory set to be excecuted.
   */
  void runTransition(const int traj_idx);
  /**
   * @brief Send i-th trajectory set to the controller for immediate execution.
   *
   * This triggers following state transition:
   * ST_TRANSITION --> ST_TRAJECTORY_FOLLOW
   * @param traj_idx The index of next trajectory set to be excecuted.
   */
  void sendTrajectories(const int traj_idx);
  /**
   * @brief Stop trajectory following or transition phase.
   *
   * This triggers following state transition:
   * ST_TRANSITION --> ST_READY
   * ST_TRAJECTORY_FOLLOW --> ST_READY
   */
  void stop();

 signals:
  /**
   * @brief Signal including a message to any QConsole, for instance a QTextBrowser.
   */
  void printToQConsole(const QString&) const;
  /**
   * @brief Signal notifying that the transition is complete.
   */
  void transitionComplete() const;
  /**
   * @brief Signal notifying that the trajectory is complete.
   */
  void trajectoryComplete() const;
  /**
   * @brief Signal carrying trajectory/transition progress status.
   */
  void trajectoryProgress(const int, const double) const;
  /**
   * @brief Stop waiting command.
   */
  void stopWaitingCmd() const;

 private slots:
  // For signals emitted by controller
  void handleTrajectoryCompleted();
  void progressUpdate(const int progress_value, const double timestamp);

  void logInfo(const QString& text) const;

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

 public:
  //--------- State machine ---------------------------------------------------------//

  /**
   * @brief The States enum of JointsPVTApp state machine.
   */
  enum States : BYTE
  {
    ST_IDLE,
    ST_READY,
    ST_TRANSITION,
    ST_TRAJECTORY_FOLLOW,
    ST_MAX_STATES
  };

 private:
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

  void printStateTransition(const States current_state, const States new_state) const;
};

#endif // CABLE_ROBOT_JOINTS_PVT_APP_H
