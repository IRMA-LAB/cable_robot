#ifndef CABLE_ROBOT_MSGS_H
#define CABLE_ROBOT_MSGS_H

#include <QDataStream>

#include "utils/types.h"
#include "utils/macros.h"

//----------------------  MESSAGE LIST -------------------------//

/**
 * @brief The MsgType enum
 */
enum MsgType : quint32
{
  NULL_MSG,
  MOTOR_STATUS,
  WINCH_STATUS,
  ACTUATOR_STATUS,
  // ... add new message type here, e.g. MY_TYPE, ...
};

//----------------------  MESSAGE HEADER -------------------------//

/**
 * @brief The HeaderMsg struct
 */
struct HeaderMsg
{
  HeaderMsg() : timestamp(-1.0), msg_type(NULL_MSG) {}
  HeaderMsg(const double time_sec) : timestamp(time_sec), msg_type(NULL_MSG) {}
  HeaderMsg(const MsgType msg_t) : timestamp(-1.0), msg_type(msg_t) {}
  HeaderMsg(const double time_sec, const MsgType msg_t)
    : timestamp(time_sec), msg_type(msg_t)
  {
  }

  double timestamp;
  MsgType msg_type;
};

QDataStream& operator<<(QDataStream& ostream, const HeaderMsg& data);
QDataStream& operator>>(QDataStream& istream, HeaderMsg& data);

//----------------------  MESSAGES -------------------------//

/**
 * @brief The BaseMsg struct
 */
struct BaseMsg
{
  BaseMsg(const MsgType msg_t) : header(msg_t) {}
  BaseMsg(const double time_sec, const MsgType msg_t) : header(time_sec, msg_t) {}

  HeaderMsg header;
};

extern const size_t kMaxMsgSize;

MSG_STRUCT_DECLARE(MOTOR_STATUS, MotorStatus)
MSG_STRUCT_DECLARE(WINCH_STATUS, WinchStatus)
MSG_STRUCT_DECLARE(ACTUATOR_STATUS, ActuatorStatus)
// ... and here, e.g. MSG_STRUCT_DECLARE(MY_TYPE, MyTypeStruct), ...

MSG_SERIALIZATION_DECLARE(MotorStatus)
MSG_SERIALIZATION_DECLARE(WinchStatus)
MSG_SERIALIZATION_DECLARE(ActuatorStatus)
// ... and here, e.g. MSG_SERIALIZATION_DECLARE(MyTypeStruct)

#endif // CABLE_ROBOT_MSGS_H
