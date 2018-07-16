#include "servomotor.h"

void ServoMotor::AssignDrive(GoldSoloWhistleDrive* theDrive)
{
  thisDrive = theDrive;
  servoMotorInputPdos = &thisDrive->inputPdos;
  servoMotorState = thisDrive->state;
  servoMotorOperationState = thisDrive->operationState;
  homeMotorCounts = 0;
  homePulleyCounts = 0;
  cableHomeLength = 0.0;
  pulleyHomeAngle = 0.0;
  cableEffectiveHomeLength = 0.0;
}

void ServoMotor::UpdateState()
{
  servoMotorState = thisDrive->state;
  servoMotorOperationState = thisDrive->operationState;
  cableLength = cableHomeLength +
                thisWinch.FromCountsToLength(
                  thisDrive->inputPdos.positionActualValue - homeMotorCounts);
  pulleyAngle =
    pulleyHomeAngle +
    thisWinch.FromCountsToPulleyAngleRad(
      thisDrive->inputPdos.auxiliaryPositionActualValue - homePulleyCounts);
}

void ServoMotor::LoopFunction()
{
  if (servoMotorState == GoldSoloWhistleDrive::operationEnabled)
  {
    switch (servoMotorOperationState)
    {
    case GoldSoloWhistleDrive::cyclicPosition:
    {
      if (motionStatus == set)
      {
        if (upCounter >= 10000)
          upCounter = 10000;
        if (downCounter <= 0)
          downCounter = 0;
        switch (typeOfMotion)
        {
        case PositionMovePlus:
        {
          thisDrive->outputPdos.TargetPosition +=
            thisWinch.FromLengthToCounts((double)upCounter * 0.0000001);
          break;
        }
        case PositionMoveMinus:
        {
          thisDrive->outputPdos.TargetPosition -=
            thisWinch.FromLengthToCounts((double)upCounter * 0.0000001);
          break;
        }
        case PositionMicroMovePlus:
        {
          thisDrive->outputPdos.TargetPosition +=
            thisWinch.FromLengthToCounts((double)upCounter * 0.000000001);
          break;
        }
        case PositionMicroMoveMinus:
        {
          thisDrive->outputPdos.TargetPosition -=
            thisWinch.FromLengthToCounts((double)upCounter * 0.000000001);
          break;
        }
        default:
          break;
        }
      }
      else
      {
        if (thisDrive->inputPdos.torqueActualValue > 10)
          thisDrive->outputPdos.TargetPosition -= (int)ceil(
            abs((double)thisDrive->inputPdos.velocityActualValue) * 0.00005);
        if (thisDrive->inputPdos.torqueActualValue < -10)
          thisDrive->outputPdos.TargetPosition += (int)ceil(
            abs((double)thisDrive->inputPdos.velocityActualValue) * 0.00005);
      }
      break;
    }
    case GoldSoloWhistleDrive::cyclicVelocity:
    {
      if (motionStatus == set)
      {
        switch (typeOfMotion)
        {
        case SpeedPlus:
        {
          thisDrive->outputPdos.TargetVelocity += 1000;
          break;
        }
        case SpeedMinus:
        {
          thisDrive->outputPdos.TargetVelocity -= 1000;
          break;
        }
        default:
          break;
        }
      }
      else
      {
        if (thisDrive->inputPdos.velocityActualValue > 2000)
          thisDrive->outputPdos.TargetVelocity -=
            thisDrive->outputPdos.TargetVelocity / 100;
        else if (thisDrive->inputPdos.velocityActualValue < -2000)
          thisDrive->outputPdos.TargetVelocity +=
            -thisDrive->outputPdos.TargetVelocity / 100;
      }
      break;
    }
    case GoldSoloWhistleDrive::cyclicTorque:
    {
      if (upCounter > 30)
      {
        upCounter = 0;
        if (motionStatus == set)
        {
          switch (typeOfMotion)
          {
          case TorquePlus:
          {
            thisDrive->outputPdos.TargetTorque += 1;
            break;
          }
          case TorqueMinus:
          {
            thisDrive->outputPdos.TargetTorque -= 1;
            break;
          }
          default:
            break;
          }
        }
        else
        {
          if (thisDrive->inputPdos.torqueActualValue > 10)
            thisDrive->outputPdos.TargetTorque -=
              thisDrive->outputPdos.TargetTorque / 10;
          else if (thisDrive->inputPdos.torqueActualValue < -10)
            thisDrive->outputPdos.TargetTorque +=
              -thisDrive->outputPdos.TargetTorque / 10;
        }
      }
      break;
    }
    default:
      break;
    }
  }
  upCounter++;
  downCounter--;
}

void ServoMotor::Enable()
{
  thisDrive->stateFlags = GoldSoloWhistleDrive::readyToSwitchOn;
}

void ServoMotor::Disable()
{
  thisDrive->stateFlags = GoldSoloWhistleDrive::switchOnDisabled;
}

void ServoMotor::FaultReset()
{
  thisDrive->stateFlags = GoldSoloWhistleDrive::switchOnDisabled;
}

void ServoMotor::ChangeOperationMode(int theMode)
{
  upCounter = 0;
  downCounter = 10000;
  thisDrive->operationStateFlags =
    (GoldSoloWhistleDrive::GoldSoloWhistleOperationState)theMode;
}

void ServoMotor::SetCommand(int theCommand, int theState)
{
  upCounter = 0;
  downCounter = 10000;
  typeOfMotion = (TypeOfMotions)theCommand;
  motionStatus = theState;
}

void ServoMotor::SetTargetDefaults()
{
  thisDrive->outputPdos.TargetTorque = thisDrive->inputPdos.torqueActualValue;
  thisDrive->outputPdos.TargetPosition =
    thisDrive->inputPdos.positionActualValue;
  thisDrive->outputPdos.TargetVelocity =
    thisDrive->inputPdos.velocityActualValue;
}

void ServoMotor::SetStartingWinchParameter()
{
  homeMotorCounts = thisDrive->inputPdos.positionActualValue;
  homePulleyCounts = thisDrive->inputPdos.auxiliaryPositionActualValue;
  cableHomeLength = 0.0;
  pulleyHomeAngle = 0.0;
  cableLength = 0.0;
  pulleyAngle = 0.0;
}

void ServoMotor::SetHomeWinchParameters(double theCable, double thePulley,
                                        double theCable2)
{
  cableHomeLength = theCable;
  pulleyHomeAngle = thePulley;
  cableEffectiveHomeLength = theCable2;
  startMotorCounts =
    homeMotorCounts +
    thisWinch.FromLengthToCounts(cableEffectiveHomeLength - cableHomeLength);
}

void ServoMotor::SetMaxTorque()
{
  if (abs(thisDrive->inputPdos.torqueActualValue) < maxTorque)
    thisDrive->outputPdos.TargetTorque--;
}

void ServoMotor::SetTorque(short theTorque)
{
  thisDrive->outputPdos.TargetTorque = theTorque;
}

void ServoMotor::SetPosition(double thePosition)
{
  thisDrive->outputPdos.TargetPosition =
    startMotorCounts + thisWinch.FromLengthToCounts(thePosition);
  // thisDrive->outputPdos.TargetPosition = startMotorCounts;
}

void ServoMotor::SetSpeed(int theVelocity)
{
  thisDrive->outputPdos.TargetVelocity = theVelocity;
}

void ServoMotor::SetPoly7IncrementalParameters(double endLength, double endT)
{
  if (endT >= 0.0)
  {
    startCounts = (double)thisDrive->inputPdos.positionActualValue;
    stopCounts = startCounts + (double)thisWinch.FromLengthToCounts(endLength);
    endTime = endT;
    poly7Flag = 1;
  }
}

void ServoMotor::SetPoly7GoHomeParameters(double endT)
{
  if (endT >= 0.0)
  {
    startCounts = (double)thisDrive->inputPdos.positionActualValue;
    stopCounts = (double)homeMotorCounts;
    endTime = endT;
    poly7Flag = 1;
  }
}

void ServoMotor::SetPoly7GoStartParameters(double endT)
{
  if (endT >= 0.0)
  {
    startCounts = (double)thisDrive->inputPdos.positionActualValue;
    stopCounts = (double)startMotorCounts;
    endTime = endT;
    poly7Flag = 1;
  }
}

void ServoMotor::MovePoly7Incremental(double t)
{
  double normalizedTime = t / endTime;
  if (normalizedTime <= 1.0 && poly7Flag)
  {
    thisDrive->outputPdos.TargetPosition =
      (int)(startCounts +
            (stopCounts - startCounts) *
              (poly7Coeff[0] * pow(normalizedTime, 4.0) +
               poly7Coeff[1] * pow(normalizedTime, 5.0) +
               poly7Coeff[2] * pow(normalizedTime, 6.0) +
               poly7Coeff[3] * pow(normalizedTime, 7.0)));
  }
  else
  {
    poly7Flag = 0;
  }
}
