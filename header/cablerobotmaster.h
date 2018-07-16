#ifndef CABLEROBOTMASTER_H
#define CABLEROBOTMASTER_H

#include <QObject>

#include "ethercatmaster.h"
#include "easycatslave.h"
#include "goldsolowhistledrive.h"
#include "cablerobot.h"

class CableRobotMaster : public QObject, public EthercatMaster
{
  Q_OBJECT
private:
  constexpr static uint8_t slavesNumber =
    7; // Some easycat master specific infos
  // constexpr static uint8_t slavesNumber = 2;
  constexpr static uint8_t EasyCatNumber = 1;
  constexpr static uint8_t GoldSoloWhistleNumber = 6;
  constexpr static uint8_t numberOfMasterStates = 5;

  ServoMotor* theServoMotor = NULL;

  typedef void (
    CableRobotMaster::*StateFunction)(); // Easy way to implement state machine
  StateFunction stateMachine[numberOfMasterStates] = {
    &CableRobotMaster::IdleFun, &CableRobotMaster::ActuatorControlFun,
    &CableRobotMaster::EasyCatControlFun,
    &CableRobotMaster::StandardRobotOperationFun,
    &CableRobotMaster::UserRobotOperationFun};
  StateFunction stateManager[numberOfMasterStates] = {
    &CableRobotMaster::IdleTransition,
    &CableRobotMaster::ActuatorControlTransition,
    &CableRobotMaster::EasyCatControlTransition,
    &CableRobotMaster::StandardRobotOperationTransition,
    &CableRobotMaster::UserRobotOperationTransition};

  void IdleFun();
  void ActuatorControlFun();
  void EasyCatControlFun();
  void StandardRobotOperationFun();
  void UserRobotOperationFun();

  void IdleTransition();
  void ActuatorControlTransition();
  void EasyCatControlTransition();
  void StandardRobotOperationTransition();
  void UserRobotOperationTransition();

public:
  explicit CableRobotMaster(QObject* parent = nullptr);
  ~CableRobotMaster() {}
  enum MasterState
  {
    idle,
    actuatorControl,
    easyCatControl,
    standardRobotOperation,
    userRobotOperation,
    nullState
  } stateFlags,
    state;

  /* The next 2 array are fundamental. The first one contains all the slaves
   * of a specified type (easycat, in this case), and for each kind of ethercat
   * slave
   * we have to define a separate array.
   * The *etherCatSlave pointer array contains the addresses of all the slave on
   * the
   * ethercat bus, in the order they are mapped in the bus. So the master base
   * class can deal with
   * everything ethercat related, without the user having to worry about it
  */
  EasyCatSlave easyCatSlave[EasyCatNumber] = {
    {EasyCatSlave(0)},
  };
  GoldSoloWhistleDrive goldSoloWhistleSlave[GoldSoloWhistleNumber] = {
    {GoldSoloWhistleDrive(1)},
    {GoldSoloWhistleDrive(2)},
    {GoldSoloWhistleDrive(3)},
    {GoldSoloWhistleDrive(4)},
    {GoldSoloWhistleDrive(5)},
    {GoldSoloWhistleDrive(6)}};
  EthercatSlave* etherCatSlave[slavesNumber] = {{&easyCatSlave[0]},
                                                {&goldSoloWhistleSlave[0]},
                                                {&goldSoloWhistleSlave[1]},
                                                {&goldSoloWhistleSlave[2]},
                                                {&goldSoloWhistleSlave[3]},
                                                {&goldSoloWhistleSlave[4]},
                                                {&goldSoloWhistleSlave[5]}};
  //    EthercatSlave *etherCatSlave[slavesNumber] = {{&easyCatSlave[0]},
  //                                                  {&easyCatSlave[1]}};

  CableRobot cableRobot;

  virtual void StartUpFunction(); // overloaded master ethercat functions, we
                                  // have to overload them
  virtual void LoopFunction();

signals:
  void SendStartUp();
  void SendMasterRequestProcessed(int);
  void SendEnableRequestProcessed(int);
  void SendClearFaultRequestProcessed();
  void SendOperationModeChangeRequestProcessed(int);
  void SendCommandUpdateRequestProcessed(int, int);
  void SendFaultPresentAdvice();
  void SendGuiData(GoldSoloWhistleDrive::InputPdos*, int);

public slots:
  void CollectMasterRequest(int state);
  void CollectMotorNumber(int theNumber);
  void CollectEnableRequest(int enable);
  void CollectClearFaultRequest();
  void CollectOperationModeChangeRequest(int theOperation);
  void CollectCommandUpdateRequest(int theCommand, int theState);
};

#endif // CABLEROBOTMASTER_H
