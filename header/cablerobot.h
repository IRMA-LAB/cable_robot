#ifndef CABLEROBOT_H
#define CABLEROBOT_H

#include <QObject>
#include <iostream>
#include <QVector>

#include "goldsolowhistledrive.h"
#include "servomotor.h"

using namespace std;

class CableRobot : public QObject
{
    Q_OBJECT
private:
    constexpr static uint8_t numberOfActuators = 6;
    constexpr static uint8_t numberOfStates = 8;
    constexpr static uint8_t numberOfHomingStates = 8;
    constexpr static uint8_t numberOfActuatorPvt33States = 4;
    constexpr static uint16_t homingDelay = 1000; //ms
    constexpr static double incrementalHomingLength = 0.10;
    constexpr static double transitionTime = 5.0;
    constexpr static double goHomeTime = 10.0;

    QVector<double> homingData;
    int numberOfPvt33Data;
    double *pointerToPvt33Data[3];
    uint8_t enablePvt = 0;
    uint8_t actuatorPvt33ProcessFinished = 0;
    int pvtCounter = 0;

    uint8_t robotGloballyEnabled = 0;
    uint8_t homingProcessFinished = 0;
    uint8_t homingFlag = 0;
    uint8_t homingStage = 0;
    int8_t homingActuator = 0;
    uint8_t measurementStage = 0;
    uint8_t measurementFlag = 0;
    typedef void (CableRobot::*StateFunction)(); // Easy way to implement state machine
    StateFunction stateMachine[numberOfStates] = {&CableRobot::IdleFun,
                                                  &CableRobot::CalibrationFun,
                                                  &CableRobot::HomingFun,
                                                  &CableRobot::Robot66ManualFun,
                                                  &CableRobot::Robot66DemoFun,
                                                  &CableRobot::Robot33ActuatorPvtFun,
                                                  &CableRobot::Robot33AutomaticFun,
                                                  &CableRobot::Robot33ManualFun};
    StateFunction homingStateMachine[numberOfHomingStates] = {&CableRobot::IdleHomingFun,
                                                              &CableRobot::SwitchToTensionMode,
                                                              &CableRobot::GoToCenter,
                                                              &CableRobot::MoveAway,
                                                              &CableRobot::WaitForMeasurement,
                                                              &CableRobot::MoveCentral,
                                                              &CableRobot::SwitchActuatedCable,
                                                              &CableRobot::GoingHome};
    StateFunction actuatorPvt33StateMachine[numberOfActuatorPvt33States] = {&CableRobot::IdleActuatorPvt33Fun,
                                                              &CableRobot::SwitchToPositionMode,
                                                              &CableRobot::MoveActuatorPvt33,
                                                              &CableRobot::GoingHomePvt33};
    StateFunction stateManager[numberOfStates] = {&CableRobot::IdleTransition,
                                                  &CableRobot::CalibrationTransition,
                                                  &CableRobot::HomingTransition,
                                                  &CableRobot::Robot66ManualTransition,
                                                  &CableRobot::Robot66DemoTransition,
                                                  &CableRobot::Robot33ActuatorPvtTransition,
                                                  &CableRobot::Robot33AutomaticTransition,
                                                  &CableRobot::Robot33ManualTransition};
    StateFunction homingStateManager[numberOfHomingStates] = {&CableRobot::IdleFunHomingTransition,
                                                              &CableRobot::SwitchToTensionModeTransition,
                                                              &CableRobot::GoToCenterTransition,
                                                              &CableRobot::MoveAwayTransition,
                                                              &CableRobot::WaitForMeasurementTransition,
                                                              &CableRobot::MoveCentralTransition,
                                                              &CableRobot::SwitchActuatedCableTransition,
                                                              &CableRobot::GoingHomeTransition};
    StateFunction actuatorPvt33StateManager[numberOfActuatorPvt33States] = {&CableRobot::IdleActuatorPvt33Transition,
                                                              &CableRobot::SwitchToPositionModeTransition,
                                                              &CableRobot::MoveActuatorPvt33Transition,
                                                              &CableRobot::GoingHomePvt33Transition};

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
    explicit CableRobot(QObject *parent = nullptr, GoldSoloWhistleDrive *theDrives = nullptr);

    ServoMotor servoMotor[numberOfActuators];
    uint8_t currentMotor = 0;
    uint16_t internalDelayCounter = 0;

    enum RobotState {
        idle,
        calibration,
        homing,
        robot66Manual,
        robot66Demo,
        robot33ActuatorPvt,
        robot33Automatic,
        robot33Manual,
        nullState
    } stateFlags, state;

    enum HomingState {
        idleHoming,
        switchToTensionMode,
        goToCenter,
        moveAway,
        waitForMeasurement,
        moveCentral,
        switchActuatedCable,
        goingHome,
        nullStateHoming
    } homingStateFlags, homingState;

    enum ActuatorPvt33State {
        idleActuatorPvt33,
        switchToPositionMode,
        moveActuatorPvt33,
        goingHomePvt33,
        nullStateActuatorPvt33
    } actuatorPvt33StateFlags, actuatorPvt33State;

    void StandardLoopFunction();
    void UserLoopFunction();

signals:
    void SendRobotRequestProcessed(int);
    void SendEnableRequestProcessed(int,int);
    void SendClearFaultRequestProcessed(int);
    void SendFaultPresentAdvice(int);
    void SendHomingControl(int);
    void SendHomingMeasurements(double);
    void SendMeasurement(QVector<double>);
    void SendData(double,double,double);
    void SendActuatorPvt33Control(int);

public slots:
    void CollectRobotRequest(int state);
    void CollectEnableRequest(int enable);
    void CollectClearFaultRequest();
    void CollectHomingProcessControl(int state);
    void CollectMeasurementRequest();
    void CollectHomingData(QVector<double> theData);
    void CollectDataPointers(int n, double* p1, double* p2, double* p3);
    void CollectActuatorPvt33Control(int state);
    void CollectStartRequest();

};

#endif // CABLEROBOT_H
