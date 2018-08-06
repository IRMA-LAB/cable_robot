#ifndef CABLEROBOT_H
#define CABLEROBOT_H

#include <QObject>
#include <iostream>
#include <QVector>

#include "goldsolowhistledrive.h"
#include "servomotor.h"

class CableRobot : public QObject
{
  Q_OBJECT
private:
  constexpr static uint8_t kNumActuators = 6;
  constexpr static uint8_t kNumStates = 8;
  constexpr static uint8_t kNumHomingStates = 8;
  constexpr static uint8_t kNumActuatorPvt33States = 4;
  constexpr static uint16_t kHomingDelay = 1000; // ms
  constexpr static double kIncrementalHomingLength = 0.10;
  constexpr static double kTransitionTime = 5.0;
  constexpr static double kGoHomeTime = 10.0;

  QVector<double> homing_data_;
  int num_pvt33data_;
  double* ptr2pvt33data_[3];
  uint8_t enable_pvt_ = 0;
  uint8_t actuator_pvt33process_finished_ = 0;
  int pvt_counter_ = 0;

  uint8_t robot_globally_enabled_ = 0;
  uint8_t homing_process_finished_ = 0;
  uint8_t homing_flag_ = 0;
  uint8_t homing_stage_ = 0;
  int8_t homing_actuator_ = 0;
  uint8_t meas_stage_ = 0;
  uint8_t meas_flag_ = 0;
  typedef void (CableRobot::*StateFunction)(); // Easy way to implement state machine
  StateFunction state_machine_[kNumStates] = {
    &CableRobot::IdleFun, &CableRobot::CalibrationFun, &CableRobot::HomingFun,
    &CableRobot::Robot66ManualFun, &CableRobot::Robot66DemoFun,
    &CableRobot::Robot33ActuatorPvtFun, &CableRobot::Robot33AutomaticFun,
    &CableRobot::Robot33ManualFun};
  StateFunction homing_state_machine_[kNumHomingStates] = {
    &CableRobot::IdleHomingFun, &CableRobot::SwitchToTensionMode, &CableRobot::GoToCenter,
    &CableRobot::MoveAway, &CableRobot::WaitForMeasurement, &CableRobot::MoveCentral,
    &CableRobot::SwitchActuatedCable, &CableRobot::GoingHome};
  StateFunction actuator_pvt33state_machine_[kNumActuatorPvt33States] = {
    &CableRobot::IdleActuatorPvt33Fun, &CableRobot::SwitchToPositionMode,
    &CableRobot::MoveActuatorPvt33, &CableRobot::GoingHomePvt33};
  StateFunction state_manager_[kNumStates] = {
    &CableRobot::IdleTransition, &CableRobot::CalibrationTransition,
    &CableRobot::HomingTransition, &CableRobot::Robot66ManualTransition,
    &CableRobot::Robot66DemoTransition, &CableRobot::Robot33ActuatorPvtTransition,
    &CableRobot::Robot33AutomaticTransition, &CableRobot::Robot33ManualTransition};
  StateFunction homing_state_manager_[kNumHomingStates] = {
    &CableRobot::IdleFunHomingTransition, &CableRobot::SwitchToTensionModeTransition,
    &CableRobot::GoToCenterTransition, &CableRobot::MoveAwayTransition,
    &CableRobot::WaitForMeasurementTransition, &CableRobot::MoveCentralTransition,
    &CableRobot::SwitchActuatedCableTransition, &CableRobot::GoingHomeTransition};
  StateFunction actuator_pvt33state_manager_[kNumActuatorPvt33States] = {
    &CableRobot::IdleActuatorPvt33Transition, &CableRobot::SwitchToPositionModeTransition,
    &CableRobot::MoveActuatorPvt33Transition, &CableRobot::GoingHomePvt33Transition};

  void IdleFun();
  void CalibrationFun();
  void HomingFun();
  void Robot66ManualFun();
  void Robot66DemoFun();
  void Robot33ActuatorPvtFun();
  void Robot33AutomaticFun();
  void Robot33ManualFun();
  void IdleTransition();
  void CalibrationTransition();
  void HomingTransition();
  void Robot66ManualTransition();
  void Robot66DemoTransition();
  void Robot33ActuatorPvtTransition();
  void Robot33AutomaticTransition();
  void Robot33ManualTransition();

  void IdleHomingFun();
  void SwitchToTensionMode();
  void GoToCenter();
  void MoveAway();
  void WaitForMeasurement();
  void MoveCentral();
  void SwitchActuatedCable();
  void GoingHome();
  void SwitchToTensionModeTransition();
  void GoToCenterTransition();
  void MoveAwayTransition();
  void WaitForMeasurementTransition();
  void MoveCentralTransition();
  void SwitchActuatedCableTransition();
  void IdleFunHomingTransition();
  void GoingHomeTransition();

  void IdleActuatorPvt33Fun();
  void SwitchToPositionMode();
  void MoveActuatorPvt33();
  void GoingHomePvt33();
  void IdleActuatorPvt33Transition();
  void SwitchToPositionModeTransition();
  void MoveActuatorPvt33Transition();
  void GoingHomePvt33Transition();

public:
  explicit CableRobot(QObject* parent = nullptr, GoldSoloWhistleDrive* drives = nullptr);

  ServoMotor servo_motor_[kNumActuators];
  uint8_t current_motor_ = 0;
  uint16_t internal_delay_counter_ = 0;

  enum RobotState
  {
    IDLE,
    CALIBRATION,
    HOMING,
    ROBOT66MANUAL,
    ROBOT66DEMO,
    ROBOT33ACTUATOR_PVT,
    ROBOT33AUTOMATIC,
    ROBOT33MANUAL,
    NULL_STATE
  } state_flags_,
    state_;

  enum HomingState
  {
    IDLE_HOMING,
    SWITCH2TENSION_MODE,
    GO2CENTER,
    MOVE_AWAY,
    WAIT_FOR_MEAS,
    MOVE_CENTRAL,
    SWITCH_ACTUATED_CABLE,
    GOING_HOME,
    NULL_STATE_HOMING
  } homing_state_flags_,
    homing_state_;

  enum ActuatorPvt33State
  {
    IDLE_ACTUATOR_PVT33,
    SWITCH2POSITION_MODE,
    MOVE_ACTUATOR_PVT33,
    GOING_HOME_PVT33,
    NULL_STATE_ACTUATOR_PVT33
  } actuator_pvt33state_flags_,
    actuator_pvt33state_;

  void StandardLoopFunction();
  void UserLoopFunction();

signals:
  void SendRobotRequestProcessed(int);
  void SendEnableRequestProcessed(int, int);
  void SendClearFaultRequestProcessed(int);
  void SendFaultPresentAdvice(int);
  void SendHomingControl(int);
  void SendHomingMeasurements(double);
  void SendMeasurement(QVector<double>);
  void SendData(double, double, double);
  void SendActuatorPvt33Control(int);

public slots:
  void CollectRobotRequest(int state);
  void CollectEnableRequest(int enable);
  void CollectClearFaultRequest();
  void CollectHomingProcessControl(int state);
  void CollectMeasurementRequest();
  void CollectHomingData(QVector<double> data);
  void CollectDataPointers(int n, double* p1, double* p2, double* p3);
  void CollectActuatorPvt33Control(int state);
  void CollectStartRequest();
};

#endif // CABLEROBOT_H
