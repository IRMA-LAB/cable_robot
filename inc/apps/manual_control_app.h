#ifndef MANUAL_CONTROL_APP_H
#define MANUAL_CONTROL_APP_H

#include <QObject>

#include "StateMachine.h"

#include "robot/cablerobot.h"

class ManualControlApp: public QObject, public StateMachine
{
  Q_OBJECT

 public:
  explicit ManualControlApp(QObject* parent, CableRobot* robot = nullptr);
  ~ManualControlApp() override {}

  void next();

 signals:
  /**
   * @brief Signal including a message to any QConsole, for instance a QTextBrowser.
   */
  void printToQConsole(const QString&) const;

 private:
  CableRobot* robot_ptr_;

 private:
  //--------- State machine ---------------------------------------------------------//

  enum States : BYTE
  {
    ST_IDLE,
    ST_READY,
    ST_MAX_STATES
  };

  // clang-format off
  static constexpr char* kStatesStr[] = {
    const_cast<char*>("IDLE"),
    const_cast<char*>("READY")};
  // clang-format on

  States prev_state_;

  STATE_DECLARE(ManualControlApp, Idle, NoEventData)
  STATE_DECLARE(ManualControlApp, Ready, NoEventData)

  // State map to define state object order
  BEGIN_STATE_MAP
  // clang-format off
   STATE_MAP_ENTRY({&Idle})
   STATE_MAP_ENTRY({&Ready})
  // clang-format on
  END_STATE_MAP

  void PrintStateTransition(const States current_state, const States new_state) const;
};

#endif // MANUAL_CONTROL_APP_H
