#ifndef CABLE_ROBOT_HOMING_PROPRIOCEPTIVE_H
#define CABLE_ROBOT_HOMING_PROPRIOCEPTIVE_H

#include <QObject>
#include <QString>

#include "StateMachine.h"
#include "easylogging++.h"

#include "robot/cablerobot.h"
#include "ctrl/controller_singledrive_naive.h"


class HomingProprioceptiveStartData : public EventData
{
public:
  HomingProprioceptiveStartData();
  HomingProprioceptiveStartData(const vect<qint16>& _init_torques,
                                const vect<qint16>& _max_torques, const quint8 _num_meas);

  vect<qint16> init_torques;
  vect<qint16> max_torques;
  quint8 num_meas;
};

std::ostream& operator<<(std::ostream& stream, const HomingProprioceptiveStartData& data);

class HomingProprioceptiveHomeData : public EventData
{
public:
  vect<double> init_lengths;
  vect<double> init_angles;
};

std::ostream& operator<<(std::ostream& stream, const HomingProprioceptiveHomeData& data);

class HomingProprioceptive : public QObject, public StateMachine
{
  Q_OBJECT

public:
  explicit HomingProprioceptive(QObject* parent, CableRobot* robot);

  enum States : BYTE
  {
    ST_IDLE,
    ST_ENABLED,
    ST_START_UP,
    ST_SWITCH_CABLE,
    ST_COILING,
    ST_UNCOILING,
    ST_OPTIMIZING,
    ST_HOME,
    ST_FAULT,
    ST_MAX_STATES
  };

  bool IsCollectingData();

public:
  void Start(HomingProprioceptiveStartData* data);
  void Stop();
  void Next();
  void Optimize();
  void GoHome(HomingProprioceptiveHomeData *data);
  void FaultTrigger();
  void FaultReset();

signals:
  void printToQConsole(const QString&) const;
  void acquisitionComplete() const;
  void homingComplete() const;

private:
  static constexpr quint8 kNumMeasMin = 1U;

  CableRobot* robot_ = NULL;
  ControllerSingleDriveNaive controller_;
  quint8 num_meas_ = kNumMeasMin;
  quint16 meas_step_;
  vect<qint16> init_torques_;
  vect<qint16> max_torques_;
  vect<qint16> torques_;

private:
  // clang-format off
  static constexpr char* kStatesStr[] = {
    const_cast<char*>("IDLE"),
    const_cast<char*>("ENABLED"),
    const_cast<char*>("START_UP"),
    const_cast<char*>("SWITCH_CABLE"),
    const_cast<char*>("COILING"),
    const_cast<char*>("UNCOILING"),
    const_cast<char*>("OPTIMIZING"),
    const_cast<char*>("GO_HOME"),
    const_cast<char*>("FAULT")};
  // clang-format on

  States prev_state_;

  // Define the state machine state functions with event data type
  STATE_DECLARE(HomingProprioceptive, Idle, NoEventData)
  GUARD_DECLARE(HomingProprioceptive, GuardEnabled, NoEventData)
  STATE_DECLARE(HomingProprioceptive, Enabled, NoEventData)
  STATE_DECLARE(HomingProprioceptive, StartUp, HomingProprioceptiveStartData)
  GUARD_DECLARE(HomingProprioceptive, GuardSwitch, NoEventData)
  STATE_DECLARE(HomingProprioceptive, SwitchCable, NoEventData)
  GUARD_DECLARE(HomingProprioceptive, GuardCoiling, NoEventData)
  STATE_DECLARE(HomingProprioceptive, Coiling, NoEventData)
  GUARD_DECLARE(HomingProprioceptive, GuardUncoiling, NoEventData)
  STATE_DECLARE(HomingProprioceptive, Uncoiling, NoEventData)
  STATE_DECLARE(HomingProprioceptive, Optimizing, NoEventData)
  STATE_DECLARE(HomingProprioceptive, Home, HomingProprioceptiveHomeData)
  STATE_DECLARE(HomingProprioceptive, Fault, NoEventData)

  // State map to define state object order. Each state map entry defines a state object.
  BEGIN_STATE_MAP_EX
  // clang-format off
    STATE_MAP_ENTRY_EX(&Idle)
    STATE_MAP_ENTRY_ALL_EX(&Enabled, &GuardEnabled, 0, 0)
    STATE_MAP_ENTRY_EX(&StartUp)
    STATE_MAP_ENTRY_ALL_EX(&SwitchCable, &GuardSwitch, 0, 0)
    STATE_MAP_ENTRY_ALL_EX(&Coiling, &GuardCoiling, 0, 0)
    STATE_MAP_ENTRY_ALL_EX(&Uncoiling, &GuardUncoiling, 0, 0)
    STATE_MAP_ENTRY_EX(&Optimizing)
    STATE_MAP_ENTRY_EX(&Home)
    STATE_MAP_ENTRY_EX(&Fault)
  // clang-format on
  END_STATE_MAP_EX

  void PrintStateTransition(const States current_state, const States new_state) const;
};

#endif // CABLE_ROBOT_HOMING_PROPRIOCEPTIVE_H
