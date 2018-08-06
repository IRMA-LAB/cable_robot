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
  // Some easycat master specific infos
  constexpr static uint8_t kSlavesNum_ = 7;
  // constexpr static uint8_t slavesNumber = 2;
  constexpr static uint8_t kEasyCatNum_ = 1;
  constexpr static uint8_t kGoldSoloWhistleNum_ = 6;
  constexpr static uint8_t kMasterStatesNum_ = 5;

  ServoMotor* servo_motor_ = NULL;

  // Easy way to implement state machine
  typedef void (CableRobotMaster::*StateFunction)();
  StateFunction state_machine_[kMasterStatesNum_] = {
    &CableRobotMaster::IdleFun, &CableRobotMaster::ActuatorControlFun,
    &CableRobotMaster::EasyCatControlFun, &CableRobotMaster::StandardRobotOperationFun,
    &CableRobotMaster::UserRobotOperationFun};
  StateFunction state_manager_[kMasterStatesNum_] = {
    &CableRobotMaster::IdleTransition, &CableRobotMaster::ActuatorControlTransition,
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
  virtual ~CableRobotMaster() {}
  enum MasterState
  {
    IDLE,
    ACTUATOR_CTRL,
    EASYCAT_CTRL,
    STD_ROBOT_OPERATION,
    USER_ROBOT_OPERATION,
    NULL_STATE
  } state_flags_,
    state_;

  /* The next 2 array are fundamental. The first one contains all the slaves
   * of a specified type (easycat, in this case), and for each kind of ethercat
   * slave we have to define a separate array.
   * The *etherCatSlave pointer array contains the addresses of all the slave on
   * the ethercat bus, in the order they are mapped in the bus. So the master base
   * class can deal with everything ethercat related, without the user having to worry
   * about it
  */
  EasyCatSlave easycat_slave_[kEasyCatNum_] = {
    {EasyCatSlave(0)},
  };
  GoldSoloWhistleDrive gold_solo_whistle_slave_[kGoldSoloWhistleNum_] = {
    {GoldSoloWhistleDrive(1)},
    {GoldSoloWhistleDrive(2)},
    {GoldSoloWhistleDrive(3)},
    {GoldSoloWhistleDrive(4)},
    {GoldSoloWhistleDrive(5)},
    {GoldSoloWhistleDrive(6)}};
  EthercatSlave* etherCatSlave_[kSlavesNum_] = {{&easycat_slave_[0]},
                                                  {&gold_solo_whistle_slave_[0]},
                                                  {&gold_solo_whistle_slave_[1]},
                                                  {&gold_solo_whistle_slave_[2]},
                                                  {&gold_solo_whistle_slave_[3]},
                                                  {&gold_solo_whistle_slave_[4]},
                                                  {&gold_solo_whistle_slave_[5]}};
  //    EthercatSlave *etherCatSlave[slavesNumber] = {{&easyCatSlave[0]},
  //                                                  {&easyCatSlave[1]}};

  CableRobot cable_robot_;

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
  void CollectMotorNumber(int motor_num);
  void CollectEnableRequest(int enable);
  void CollectClearFaultRequest();
  void CollectOperationModeChangeRequest(int target_op_mode);
  void CollectCommandUpdateRequest(int cmd, int state);
};

#endif // CABLEROBOTMASTER_H
