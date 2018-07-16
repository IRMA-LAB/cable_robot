#include "cablerobotmaster.h"

void CableRobotMaster::IdleFun()
{

}

void CableRobotMaster::ActuatorControlFun()
{
    if (theServoMotor != NULL) {
        int flag = theServoMotor->HasEnableRequestBeenProcessed();
        if (flag< 2) {
            emit SendEnableRequestProcessed(flag);
        } else if (theServoMotor->HasClearFaultRequestBeenProcessed()) {
            emit SendClearFaultRequestProcessed();
        } else if (theServoMotor->HasOperationModeChangeRequestBeenProcessed()) {
            emit SendOperationModeChangeRequestProcessed((int)theServoMotor->servoMotorOperationState);
        } else if (theServoMotor->FaultPresent()) {
            emit SendFaultPresentAdvice();
        }
        theServoMotor->UpdateState();
        theServoMotor->LoopFunction();
        if (theServoMotor->updateIndex==(int)GoldSoloWhistleDrive::GoldSoloWhistleDomainInputs) theServoMotor->updateIndex = 0;
        emit SendGuiData(theServoMotor->servoMotorInputPdos,theServoMotor->updateIndex);
        theServoMotor->updateIndex++;
    }
}

void CableRobotMaster::EasyCatControlFun()
{
    for (int i=0; i<EasyCatNumber; i++) easyCatSlave[i].LoopFunction();
}

void CableRobotMaster::StandardRobotOperationFun()
{
    cableRobot.StandardLoopFunction();
}

void CableRobotMaster::UserRobotOperationFun()
{
    cableRobot.UserLoopFunction();
}

void CableRobotMaster::IdleTransition()
{
    if (stateFlags == idle) {
        stateFlags = nullState;
        emit SendMasterRequestProcessed(state);
        cout << "Changing state to " << state << endl;
    } else if (stateFlags != nullState) {
        state = stateFlags;
        for (int i=0; i<GoldSoloWhistleNumber; i++) goldSoloWhistleSlave[i].SetTargetDefaults();
    }

}

void CableRobotMaster::ActuatorControlTransition()
{
    if (stateFlags == actuatorControl) {
        stateFlags = nullState;
        emit SendMasterRequestProcessed(state);
        cout << "Changing state to " << state << endl;
    } else if (stateFlags != nullState) {
        state = stateFlags;
        theServoMotor = NULL;
    }
}

void CableRobotMaster::EasyCatControlTransition()
{
    if (stateFlags == easyCatControl) {
        stateFlags = nullState;
        emit SendMasterRequestProcessed(state);
        cout << "Changing state to " << state << endl;
    } else if (stateFlags != nullState) {
        state = stateFlags;
    }
}

void CableRobotMaster::StandardRobotOperationTransition()
{
    if (stateFlags == standardRobotOperation) {
        stateFlags = nullState;
        emit SendMasterRequestProcessed(state);
        cout << "Changing state to " << state << endl;
    } else if (stateFlags != nullState) {
        state = stateFlags;
    }
}

void CableRobotMaster::UserRobotOperationTransition()
{
    if (stateFlags == userRobotOperation) {
        stateFlags = nullState;
        emit SendMasterRequestProcessed(state);
        cout << "Changing state to " << state << endl;
    } else if (stateFlags != nullState) {
        state = stateFlags;
    }
}

CableRobotMaster::CableRobotMaster(QObject *parent) :
    QObject(parent),
    cableRobot(nullptr,goldSoloWhistleSlave)
{
    /* It is FUNDAMENTAL that the constructor has this form. There are several other ways we can program
     * the slave interface so that we do not have to deal with the assignment of this variables in the constructor,
     * but they will require the user to initialize some of the variables in the main file and then pass the as
     * input variable in the constructor. I find that this way it is easy for "not experienced user" to write all their code
     * in just two files, the .h and .cpp of their actual implementation of the ethercat master
    */

    numberOfSlaves = slavesNumber;
    slave = &etherCatSlave[0];
    for (int i=0; i<numberOfSlaves; i++) domainElementsNumber+= etherCatSlave[i]->numberOfDomainEntries;

    state = idle;
    stateFlags = nullState;

}

void CableRobotMaster::StartUpFunction()
{
    emit SendStartUp();
}

void CableRobotMaster::LoopFunction()
{
    for (int i=0;i<slavesNumber; i++) etherCatSlave[i]->ReadInputs(); // Read pdos, act accordingly
    (this->*stateManager[state])(); // check if the state has to be changed
    (this->*stateMachine[state])(); // do your job
    for (int i=0;i<slavesNumber; i++) etherCatSlave[i]->WriteOutputs(); // Write all the necessary pdos
}

void CableRobotMaster::CollectMasterRequest(int state)
{
    stateFlags = CableRobotMaster::MasterState(state);

}

void CableRobotMaster::CollectMotorNumber(int theNumber)
{
    theServoMotor = &cableRobot.servoMotor[theNumber];
}

void CableRobotMaster::CollectEnableRequest(int enable)
{
    if (enable) theServoMotor->Enable();
    else theServoMotor->Disable();
}

void CableRobotMaster::CollectClearFaultRequest()
{
    theServoMotor->FaultReset();
}

void CableRobotMaster::CollectOperationModeChangeRequest(int theOperation)
{
    theServoMotor->ChangeOperationMode(theOperation);
}

void CableRobotMaster::CollectCommandUpdateRequest(int theCommand, int theState)
{
    theServoMotor->SetCommand(theCommand,theState);
    emit SendCommandUpdateRequestProcessed(theCommand,theState);
}
