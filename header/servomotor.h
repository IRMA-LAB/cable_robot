#ifndef SERVOMOTOR_H
#define SERVOMOTOR_H

#include "goldsolowhistledrive.h"
#include "winch.h"
#include "cmath"

class ServoMotor
{
private:
    GoldSoloWhistleDrive *thisDrive = NULL;
    Winch thisWinch;
    constexpr static uint8_t set = 1;
    constexpr static uint8_t reset = 0;
    constexpr static short maxTorque = 400;
    double stopCounts = 0.0;
    double startCounts = 0.0;
    double endTime = 1.0;
    int homeMotorCounts = 0;
    int homePulleyCounts = 0;
    int startMotorCounts = 0;
    int startPulleyCounts = 0;
    uint8_t poly7Flag = 0;

    double poly7Coeff[4] = {35.0,-84.0,70.0,-20.0};
public:

    enum TypeOfMotions {
        PositionMovePlus,
        PositionMoveMinus,
        PositionMicroMovePlus,
        PositionMicroMoveMinus,
        TorquePlus,
        TorqueMinus,
        SpeedPlus,
        SpeedMinus
    } typeOfMotion;
    uint8_t motionStatus = reset;
    GoldSoloWhistleDrive::InputPdos *servoMotorInputPdos = NULL;
    GoldSoloWhistleDrive::GoldSoloWhistleDriveState servoMotorState;
    GoldSoloWhistleDrive::GoldSoloWhistleOperationState servoMotorOperationState;

    ServoMotor() {}
    int updateIndex = 0;
    int upCounter = 0;
    int downCounter = 10000;
    double cableHomeLength = 0.0;
    double cableLength = 0.0;
    double cableEffectiveHomeLength = 0.0;
    double pulleyHomeAngle = 0.0;
    double pulleyAngle = 0.0;
    double motorTorque = 0.0;

    void AssignDrive(GoldSoloWhistleDrive *theDrive);
    void UpdateState();
    void LoopFunction();
    void Enable();
    void Disable();
    void FaultReset();
    void ChangeOperationMode(int theMode);
    void SetCommand(int theCommand,int theState);


    inline int HasEnableRequestBeenProcessed() {
        if (thisDrive->state == GoldSoloWhistleDrive::operationEnabled && servoMotorState == GoldSoloWhistleDrive::switchOn) {
            return 1;
        } else if (thisDrive->state == GoldSoloWhistleDrive::switchOnDisabled && servoMotorState == GoldSoloWhistleDrive::operationEnabled) {
            return 0;
        } else return 2;
    }
    inline int FaultPresent() {
        if (thisDrive->state == GoldSoloWhistleDrive::fault && servoMotorState != GoldSoloWhistleDrive::fault) {
            return 1;
        } else return 0;
    }
    inline int HasClearFaultRequestBeenProcessed() {
        if (thisDrive->state == GoldSoloWhistleDrive::switchOnDisabled && servoMotorState == GoldSoloWhistleDrive::fault) {
            return 1;
        } else return 0;
    }
    inline int HasOperationModeChangeRequestBeenProcessed() {
        if (thisDrive->operationState != servoMotorOperationState && thisDrive->operationState != GoldSoloWhistleDrive::nullOperation) {
            servoMotorOperationState = thisDrive->operationState;
            return 1;
        } else return 0;
    }

    void SetTargetDefaults();
    void SetStartingWinchParameter();
    void SetHomeWinchParameters(double theCable, double thePulley, double theCable2);
    void SetMaxTorque();
    void SetTorque(short theTorque);
    void SetPosition(double thePosition);
    void SetSpeed(int theVelocity);
    void SetPoly7IncrementalParameters(double endLength, double endT);
    void SetPoly7GoHomeParameters(double endT);
    void SetPoly7GoStartParameters(double endT);
    void MovePoly7Incremental(double t);

};

#endif // SERVOMOTOR_H
