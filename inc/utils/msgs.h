#ifndef MSGS_H
#define MSGS_H

#include <QDataStream>

#include "types.h"

//----------------------  MACROS -------------------------//

#ifndef MSG_SERIALIZATION_DECLARE
#define MSG_SERIALIZATION_DECLARE(_MsgType)                                              \
  QDataStream& operator<<(QDataStream& ostream, const _MsgType& data);                   \
  QDataStream& operator>>(QDataStream& istream, _MsgType& data);                         \
  QByteArray serialize(const _MsgType##Msg& data);
#endif


#ifndef MSG_STRUCT_DECLARE
#define MSG_STRUCT_DECLARE(_MSG_TYPE, _BodyType)                                         \
  struct _BodyType##Msg : BaseMsg                                                        \
  {                                                                                      \
    _BodyType##Msg() : BaseMsg(_MSG_TYPE) {}                                             \
    _BodyType##Msg(const _BodyType& data) : BaseMsg(_MSG_TYPE), body(data) {}            \
    _BodyType##Msg(const double time_sec, const _BodyType& data)                         \
      : BaseMsg(time_sec, _MSG_TYPE), body(data)                                         \
    {                                                                                    \
    }                                                                                    \
    _BodyType##Msg(const QByteArray data) : BaseMsg(_MSG_TYPE) { deserialize(data); }    \
    QByteArray serialized() const;                                                       \
    void deserialize(QByteArray data);                                                   \
    _BodyType body;                                                                      \
  };
#endif

//----------------------  MESSAGE LIST -------------------------//

/**
 * @brief The MsgType enum
 */
enum MsgType : quint32
{
  NONE,
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
  HeaderMsg() : timestamp(-1.0), msg_type(NONE) {}
  HeaderMsg(const double time_sec) : timestamp(time_sec), msg_type(NONE) {}
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

#endif // MSGS_H
