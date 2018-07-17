#include "cablerobot.h"

void CableRobot::IdleFun() {}

void CableRobot::CalibrationFun() {}

void CableRobot::HomingFun()
{

  for (int i = 0; i < numberOfActuators; i++)
  {
    int flag = servoMotor[i].HasEnableRequestBeenProcessed();
    if (flag < 2)
    {
      emit SendEnableRequestProcessed(flag, i);
      if (flag == 1)
      {
        robotGloballyEnabled += 1;
        cout << static_cast<int>(robotGloballyEnabled) << endl;
      }
      else
      {
      }
    }
    else if (servoMotor[i].HasClearFaultRequestBeenProcessed())
    {
      servoMotor[i].Enable();
      emit SendClearFaultRequestProcessed(i);
    }
    else if (servoMotor[i].FaultPresent())
    {
      servoMotor[i].FaultReset();
      robotGloballyEnabled -= 1;
      emit SendFaultPresentAdvice(i);
    }
    servoMotor[i].UpdateState();
  }
  if (robotGloballyEnabled < numberOfActuators)
  {
    homingState = idleHoming;
    homingStateFlags = nullStateHoming;
    internalDelayCounter = 0;
    homingProcessFinished = 0;
    homingStage = 0;
  }
  (this->*homingStateManager[homingState])();
  (this->*homingStateMachine[homingState])();
}

void CableRobot::Robot66ManualFun() {}

void CableRobot::Robot66DemoFun() {}

void CableRobot::Robot33ActuatorPvtFun()
{
  for (int i = 0; i < numberOfActuators; i++)
  {
    int flag = servoMotor[i].HasEnableRequestBeenProcessed();
    if (flag < 2)
    {
      emit SendEnableRequestProcessed(flag, i);
      if (flag == 1)
      {
        robotGloballyEnabled += 1;
        cout << static_cast<int>(robotGloballyEnabled) << endl;
      }
      else
      {
      }
    }
    else if (servoMotor[i].HasClearFaultRequestBeenProcessed())
    {
      servoMotor[i].Enable();
      emit SendClearFaultRequestProcessed(i);
    }
    else if (servoMotor[i].FaultPresent())
    {
      servoMotor[i].FaultReset();
      robotGloballyEnabled -= 1;
      emit SendFaultPresentAdvice(i);
    }
    servoMotor[i].UpdateState();
  }
  if (robotGloballyEnabled < numberOfActuators)
  {
    actuatorPvt33State = idleActuatorPvt33;
    actuatorPvt33StateFlags = nullStateActuatorPvt33;
    internalDelayCounter = 0;
  }
  (this->*actuatorPvt33StateManager[actuatorPvt33State])();
  (this->*actuatorPvt33StateMachine[actuatorPvt33State])();
}

void CableRobot::Robot33AutomaticFun() {}

void CableRobot::Robot33ManualFun() {}

void CableRobot::IdleTransition()
{
  if (stateFlags == idle)
  {
    stateFlags = nullState;
    emit SendRobotRequestProcessed(state);
    cout << "Changing robot state to " << state << endl;
  }
  else if (stateFlags != nullState)
  {
    state = stateFlags;
    homingState = idleHoming;
    homingStateFlags = idleHoming;
    actuatorPvt33State = idleActuatorPvt33;
    actuatorPvt33StateFlags = idleActuatorPvt33;
    actuatorPvt33ProcessFinished = 0;
    internalDelayCounter = 0;
    enablePvt = 0;
    pvtCounter = 0;
  }
}

void CableRobot::CalibrationTransition()
{
  if (stateFlags == calibration)
  {
    stateFlags = nullState;
    emit SendRobotRequestProcessed(state);
    cout << "Changing robot state to " << state << endl;
  }
  else if (stateFlags != nullState)
  {
    state = stateFlags;
  }
}

void CableRobot::HomingTransition()
{
  if (stateFlags == homing)
  {
    stateFlags = nullState;
    internalDelayCounter = 0;
    emit SendRobotRequestProcessed(state);
    cout << "Changing robot state to " << state << endl;
  }
  else if (stateFlags != nullState)
  {
    state = stateFlags;
    robotGloballyEnabled = 0;
  }
}

void CableRobot::Robot66ManualTransition()
{
  if (stateFlags == robot66Manual)
  {
    stateFlags = nullState;
    emit SendRobotRequestProcessed(state);
    cout << "Changing robot state to " << state << endl;
  }
  else if (stateFlags != nullState)
  {
    state = stateFlags;
  }
}

void CableRobot::Robot66DemoTransition()
{
  if (stateFlags == robot66Demo)
  {
    stateFlags = nullState;
    emit SendRobotRequestProcessed(state);
    cout << "Changing robot state to " << state << endl;
  }
  else if (stateFlags != nullState)
  {
    state = stateFlags;
  }
}

void CableRobot::Robot33ActuatorPvtTransition()
{
  if (stateFlags == robot33ActuatorPvt)
  {
    stateFlags = nullState;
    internalDelayCounter = 0;
    emit SendRobotRequestProcessed(state);
    cout << "Changing robot state to " << state << endl;
  }
  else if (stateFlags != nullState)
  {
    state = stateFlags;
    robotGloballyEnabled = 0;
  }
}

void CableRobot::Robot33AutomaticTransition()
{
  if (stateFlags == robot33Automatic)
  {
    stateFlags = nullState;
    emit SendRobotRequestProcessed(state);
    cout << "Changing robot state to " << state << endl;
  }
  else if (stateFlags != nullState)
  {
    state = stateFlags;
  }
}

void CableRobot::Robot33ManualTransition()
{
  if (stateFlags == robot33Manual)
  {
    stateFlags = nullState;
    emit SendRobotRequestProcessed(state);
    cout << "Changing robot state to " << state << endl;
  }
  else if (stateFlags != nullState)
  {
    state = stateFlags;
  }
}

void CableRobot::IdleHomingFun()
{
  if (robotGloballyEnabled >= numberOfActuators && !homingProcessFinished)
  {
    if (internalDelayCounter < homingDelay)
    {
      internalDelayCounter++;
    }
    else
    {
      homingStateFlags = switchToTensionMode;
      internalDelayCounter = 0;
    }
  }
}

void CableRobot::SwitchToTensionMode()
{
  if (robotGloballyEnabled >= numberOfActuators)
  {
    uint8_t flagCompleted = 0;
    if (homingFlag == 0)
    {
      for (int i = 0; i < numberOfActuators; i++)
      {
        if (servoMotor[i].servoMotorOperationState !=
            GoldSoloWhistleDrive::cyclicTorque)
        {
          servoMotor[i].SetTargetDefaults();
          servoMotor[i].ChangeOperationMode(GoldSoloWhistleDrive::cyclicTorque);
        }
        else
          flagCompleted++;
      }
      if (flagCompleted == numberOfActuators)
      {
        homingStateFlags = goToCenter;
        emit SendHomingControl(1);
        cout << "here" << endl;
        internalDelayCounter = 0;
        homingStage = 0;
        homingActuator = 0;
      }
    }
    else
    {
      for (int i = 0; i < numberOfActuators; i++)
      {
        if (servoMotor[i].servoMotorOperationState !=
            GoldSoloWhistleDrive::cyclicPosition)
        {
          servoMotor[i].SetTargetDefaults();
          servoMotor[i].ChangeOperationMode(
            GoldSoloWhistleDrive::cyclicPosition);
        }
        else
          flagCompleted++;
      }
      if (flagCompleted == numberOfActuators)
      {
        homingStateFlags = goingHome;
        for (int i = 0; i < numberOfActuators; i += 2)
        {
          servoMotor[i].SetPoly7GoHomeParameters(goHomeTime);
        }
        emit SendHomingControl(0);
        cout << "here going home" << endl;
        internalDelayCounter = 0;
        homingActuator = 0;
      }
    }
  }
}

void CableRobot::GoToCenter()
{
  internalDelayCounter++;
  if (internalDelayCounter > 30)
  {
    internalDelayCounter = 0;
    for (int i = 0; i < numberOfActuators; i += 2)
      servoMotor[i].SetMaxTorque();
  }
  if (homingFlag)
  {
    cout << "here2" << endl;
    homingStateFlags = switchActuatedCable;
    for (int i = 0; i < numberOfActuators; i += 2)
      servoMotor[i].SetStartingWinchParameter();
    internalDelayCounter = 0;
    homingActuator -= 2;
    // Send Start Values
    homingData[0] = servoMotor[0].cableLength;
    homingData[1] = servoMotor[2].cableLength;
    homingData[2] = servoMotor[4].cableLength;
    homingData[3] = servoMotor[0].pulleyAngle;
    homingData[4] = servoMotor[2].pulleyAngle;
    homingData[5] = servoMotor[4].pulleyAngle;
    SendMeasurement(homingData);
  }
}

void CableRobot::SwitchActuatedCable()
{
  static uint8_t modIndex = 99;

  if (homingFlag && internalDelayCounter == 0)
  {
    homingActuator += 2;
    internalDelayCounter++;
    if (homingActuator < numberOfActuators)
    {
      servoMotor[homingActuator].SetTargetDefaults();
      for (int i = 0; i < numberOfActuators; i += 2)
      {
        if (i != homingActuator)
        {
          if (servoMotor[i].servoMotorOperationState !=
              GoldSoloWhistleDrive::cyclicTorque)
          {
            servoMotor[i].ChangeOperationMode(
              GoldSoloWhistleDrive::cyclicTorque);
            modIndex = static_cast<uint8_t>(i);
          }
        }
        else
        {
          servoMotor[i].ChangeOperationMode(
            GoldSoloWhistleDrive::cyclicPosition);
        }
        servoMotor[i].SetTargetDefaults();
      }
    }
    else
    {
      homingStateFlags = switchToTensionMode;
      homingActuator = 0;
      cout << "here" << endl;
      internalDelayCounter = 0;
    }
  }
  else if (homingFlag && homingActuator <= numberOfActuators)
  {
    uint8_t flag = 1;
    for (int i = 0; i < numberOfActuators; i += 2)
    {
      if (i == homingActuator &&
          servoMotor[i].servoMotorOperationState ==
            GoldSoloWhistleDrive::cyclicPosition)
        flag *= 1;
      else if (i != homingActuator &&
               servoMotor[i].servoMotorOperationState ==
                 GoldSoloWhistleDrive::cyclicTorque)
        flag *= 1;
      else
        flag *= 0;
    }
    if (flag)
    {
      homingStateFlags = moveAway;
      servoMotor[homingActuator].SetPoly7IncrementalParameters(
        -incrementalHomingLength, transitionTime);
      internalDelayCounter = 0;
      measurementStage = 0;
    }
  }
}

void CableRobot::MoveAway()
{
  if (homingFlag)
  {
    internalDelayCounter++;
    double timeInSeconds = (static_cast<double>(internalDelayCounter)) / 1000.0;
    if (timeInSeconds <= transitionTime)
      servoMotor[homingActuator].MovePoly7Incremental(timeInSeconds);
    else
    {
      homingStateFlags = waitForMeasurement;
    }
  }
}

void CableRobot::WaitForMeasurement()
{
  cout << servoMotor[0].pulleyAngle << '\t' << servoMotor[2].pulleyAngle << '\t'
       << servoMotor[4].pulleyAngle << endl;
  if (homingFlag && measurementFlag)
  {
    // Send Measurement To non RT Thread
    homingData[0] = servoMotor[0].cableLength;
    homingData[1] = servoMotor[2].cableLength;
    homingData[2] = servoMotor[4].cableLength;
    homingData[3] = servoMotor[0].pulleyAngle;
    homingData[4] = servoMotor[2].pulleyAngle;
    homingData[5] = servoMotor[4].pulleyAngle;
    SendMeasurement(homingData);
    measurementFlag = 0;
    measurementStage++;
    if (measurementStage < 1)
    {
      servoMotor[homingActuator].SetPoly7IncrementalParameters(
        -incrementalHomingLength, transitionTime);
      internalDelayCounter = 0;
      homingStateFlags = moveAway;
    }
    else if (measurementStage < 2)
    {
      homingStateFlags = moveCentral;
      servoMotor[homingActuator].SetPoly7IncrementalParameters(
        incrementalHomingLength, transitionTime);
      internalDelayCounter = 0;
    }
    else
    {
      homingStateFlags = switchActuatedCable;
      internalDelayCounter = 0;
    }
  }
}

void CableRobot::MoveCentral()
{
  if (homingFlag)
  {
    internalDelayCounter++;
    double timeInSeconds = (static_cast<double>(internalDelayCounter)) / 1000.0;
    if (timeInSeconds <= transitionTime)
      servoMotor[homingActuator].MovePoly7Incremental(timeInSeconds);
    else
    {
      homingStateFlags = waitForMeasurement;
    }
  }
}

void CableRobot::GoingHome()
{
  internalDelayCounter++;
  homingFlag = 0;
  double timeInSeconds = (static_cast<double>(internalDelayCounter)) / 1000.0;
  if (timeInSeconds <= goHomeTime)
    for (int i = 0; i < numberOfActuators; i += 2)
      servoMotor[i].MovePoly7Incremental(timeInSeconds);
  else
  {
    homingStateFlags = idleHoming;
    homingProcessFinished = 1;
  }
}

void CableRobot::SwitchToTensionModeTransition()
{
  if (homingStateFlags == switchToTensionMode)
  {
    homingStateFlags = nullStateHoming;
    cout << "Changing homing state to " << homingState << endl;
  }
  else if (homingStateFlags != nullStateHoming)
  {
    homingState = homingStateFlags;
    internalDelayCounter = 0;
  }
}

void CableRobot::GoToCenterTransition()
{
  if (homingStateFlags == goToCenter)
  {
    homingStateFlags = nullStateHoming;
    cout << "Changing homing state to " << homingState << endl;
  }
  else if (homingStateFlags != nullStateHoming)
  {
    homingState = homingStateFlags;
    internalDelayCounter = 0;
  }
}

void CableRobot::MoveAwayTransition()
{
  if (homingStateFlags == moveAway)
  {
    homingStateFlags = nullStateHoming;
    cout << "Changing homing state to " << homingState << endl;
  }
  else if (homingStateFlags != nullStateHoming)
  {
    homingState = homingStateFlags;
    internalDelayCounter = 0;
  }
}

void CableRobot::WaitForMeasurementTransition()
{
  if (homingStateFlags == waitForMeasurement)
  {
    homingStateFlags = nullStateHoming;
    cout << "Changing homing state to " << homingState << endl;
  }
  else if (homingStateFlags != nullStateHoming)
  {
    homingState = homingStateFlags;
    internalDelayCounter = 0;
  }
}

void CableRobot::MoveCentralTransition()
{
  if (homingStateFlags == moveCentral)
  {
    homingStateFlags = nullStateHoming;
    cout << "Changing homing state to " << homingState << endl;
  }
  else if (homingStateFlags != nullStateHoming)
  {
    homingState = homingStateFlags;
    internalDelayCounter = 0;
  }
}

void CableRobot::SwitchActuatedCableTransition()
{
  if (homingStateFlags == switchActuatedCable)
  {
    homingStateFlags = nullStateHoming;
    cout << "Changing homing state to " << homingState << endl;
  }
  else if (homingStateFlags != nullStateHoming)
  {
    homingState = homingStateFlags;
    internalDelayCounter = 0;
  }
}

void CableRobot::IdleFunHomingTransition()
{
  if (homingStateFlags == idleHoming)
  {
    homingStateFlags = nullStateHoming;
    cout << "Changing homing state to " << homingState << endl;
  }
  else if (homingStateFlags != nullStateHoming)
  {
    homingState = homingStateFlags;
  }
}

void CableRobot::GoingHomeTransition()
{
  if (homingStateFlags == goingHome)
  {
    homingStateFlags = nullStateHoming;
    cout << "Changing homing state to " << homingState << endl;
  }
  else if (homingStateFlags != nullStateHoming)
  {
    homingState = homingStateFlags;
    internalDelayCounter = 0;
  }
}

void CableRobot::IdleActuatorPvt33Fun()
{
  if (robotGloballyEnabled >= numberOfActuators &&
      !actuatorPvt33ProcessFinished)
  {
    if (internalDelayCounter < homingDelay)
    {
      internalDelayCounter++;
    }
    else
    {
      actuatorPvt33StateFlags = switchToPositionMode;
      internalDelayCounter = 0;
    }
  }
  if (actuatorPvt33ProcessFinished)
  {
    // emit
    // SendData(servoMotor[0].pulleyAngle,servoMotor[1].pulleyAngle,servoMotor[2].pulleyAngle);
  }
}

void CableRobot::SwitchToPositionMode()
{
  if (robotGloballyEnabled >= numberOfActuators)
  {
    uint8_t flagCompleted = 0;
    for (int i = 0; i < numberOfActuators; i++)
    {
      if (servoMotor[i].servoMotorOperationState !=
          GoldSoloWhistleDrive::cyclicPosition)
      {
        servoMotor[i].SetTargetDefaults();
        servoMotor[i].ChangeOperationMode(GoldSoloWhistleDrive::cyclicPosition);
      }
      else
        flagCompleted++;
    }
    if (flagCompleted == numberOfActuators)
    {
      enablePvt = 1;
      actuatorPvt33StateFlags = moveActuatorPvt33;
      emit SendActuatorPvt33Control(1);
      cout << "going to move..." << endl;
    }
  }
}

void CableRobot::MoveActuatorPvt33()
{
  if (enablePvt && robotGloballyEnabled >= numberOfActuators)
  {
    if (pvtCounter < numberOfPvt33Data)
    {
      pvtCounter++;
      emit SendData(servoMotor[0].pulleyAngle, servoMotor[2].pulleyAngle,
                    servoMotor[4].pulleyAngle);
      for (int i = 0; i < numberOfActuators; i += 2)
      {
        servoMotor[i].SetPosition(*pointerToPvt33Data[i / 2]);
        pointerToPvt33Data[i / 2]++;
      }
    }
    else
    {
      emit SendActuatorPvt33Control(2);
      actuatorPvt33StateFlags = goingHomePvt33;
      for (int i = 0; i < numberOfActuators; i += 2)
        servoMotor[i].SetPoly7GoStartParameters(goHomeTime);
    }
  }
  else
  {
    // emit SendActuatorPvt33Control(0);
  }
}

void CableRobot::GoingHomePvt33()
{
  if (robotGloballyEnabled >= numberOfActuators)
  {
    internalDelayCounter++;
    double timeInSeconds = (static_cast<double>(internalDelayCounter)) / 1000.0;
    if (timeInSeconds <= goHomeTime)
      for (int i = 0; i < numberOfActuators; i += 2)
        servoMotor[i].MovePoly7Incremental(timeInSeconds);
    else
    {
      actuatorPvt33StateFlags = idleActuatorPvt33;
      actuatorPvt33ProcessFinished = 1;
      enablePvt = 0;
    }
  }
}

void CableRobot::IdleActuatorPvt33Transition()
{
  if (actuatorPvt33StateFlags == idleActuatorPvt33)
  {
    actuatorPvt33StateFlags = nullStateActuatorPvt33;
    cout << "Changing actuatorPvt33 state to " << actuatorPvt33State << endl;
  }
  else if (actuatorPvt33StateFlags != nullStateActuatorPvt33)
  {
    actuatorPvt33State = actuatorPvt33StateFlags;
    internalDelayCounter = 0;
  }
}

void CableRobot::SwitchToPositionModeTransition()
{
  if (actuatorPvt33StateFlags == switchToPositionMode)
  {
    actuatorPvt33StateFlags = nullStateActuatorPvt33;
    cout << "Changing actuatorPvt33 state to " << actuatorPvt33State << endl;
  }
  else if (actuatorPvt33StateFlags != nullStateActuatorPvt33)
  {
    actuatorPvt33State = actuatorPvt33StateFlags;
    internalDelayCounter = 0;
  }
}

void CableRobot::MoveActuatorPvt33Transition()
{
  if (actuatorPvt33StateFlags == moveActuatorPvt33)
  {
    actuatorPvt33StateFlags = nullStateActuatorPvt33;
    cout << "Changing actuatorPvt33 state to " << actuatorPvt33State << endl;
  }
  else if (actuatorPvt33StateFlags != nullStateActuatorPvt33)
  {
    actuatorPvt33State = actuatorPvt33StateFlags;
    internalDelayCounter = 0;
  }
}

void CableRobot::GoingHomePvt33Transition()
{
  if (actuatorPvt33StateFlags == goingHomePvt33)
  {
    actuatorPvt33StateFlags = nullStateActuatorPvt33;
    cout << "Changing actuatorPvt33 state to " << actuatorPvt33State << endl;
  }
  else if (actuatorPvt33StateFlags != nullStateActuatorPvt33)
  {
    actuatorPvt33State = actuatorPvt33StateFlags;
    internalDelayCounter = 0;
  }
}

CableRobot::CableRobot(QObject* parent, GoldSoloWhistleDrive* theDrives)
  : QObject(parent)
{
  for (int i = 0; i < numberOfActuators; i++)
  {
    servoMotor[i].AssignDrive(&theDrives[i]);
  }

  state = idle;
  stateFlags = nullState;
  homingData.resize(6);
}

void CableRobot::StandardLoopFunction()
{
  (this->*stateManager[state])();
  (this->*stateMachine[state])();
}

void CableRobot::UserLoopFunction()
{
  (this->*stateManager[state])();
  (this->*stateMachine[state])();
}

void CableRobot::CollectRobotRequest(int state)
{
  stateFlags = CableRobot::RobotState(state);
}

void CableRobot::CollectEnableRequest(int enable)
{
  robotGloballyEnabled = 0;
  for (int i = 0; i < numberOfActuators; i++)
  {
    if (enable)
    {
      if (servoMotor[i].servoMotorState !=
          GoldSoloWhistleDrive::operationEnabled)
        servoMotor[i].Enable();
    }
    else
    {
      servoMotor[i].Disable();
    }
  }
}

void CableRobot::CollectClearFaultRequest()
{
  for (int i = 0; i < numberOfActuators; i++)
  {
    if (servoMotor[i].servoMotorState == GoldSoloWhistleDrive::fault)
      servoMotor[i].FaultReset();
  }
}

void CableRobot::CollectHomingProcessControl(int state)
{
  homingFlag = static_cast<uint8_t>(state);
}

void CableRobot::CollectMeasurementRequest() { measurementFlag = 1; }

void CableRobot::CollectHomingData(QVector<double> theData)
{
  for (int i = 0; i < numberOfActuators; i += 2)
  {
    servoMotor[i].SetHomeWinchParameters(theData[i / 2],
                                         theData[i / 2 + numberOfActuators / 2],
                                         theData[i / 2 + numberOfActuators]);
    servoMotor[i].SetPoly7GoStartParameters(goHomeTime);
  }
  homingStateFlags = goingHome;
  cout << "going to start position" << endl;
  internalDelayCounter = 0;
}

void CableRobot::CollectDataPointers(int n, double* p1, double* p2, double* p3)
{
  numberOfPvt33Data = n;
  pointerToPvt33Data[0] = p1;
  pointerToPvt33Data[1] = p2;
  pointerToPvt33Data[2] = p3;
  actuatorPvt33ProcessFinished = 0;
  internalDelayCounter = 0;
  enablePvt = 0;
  pvtCounter = 0;
}

void CableRobot::CollectActuatorPvt33Control(int /*state*/)
{
  enablePvt = 1;
  pvtCounter = 0;
}

void CableRobot::CollectStartRequest() {}
