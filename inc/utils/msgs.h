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

#ifndef MSG_SERIALIZATION_DEFINE
#define MSG_SERIALIZATION_DEFINE(_MSG_TYPE, _BodyType)                                   \
  QByteArray _BodyType##Msg::serialized() const { return serialize(*this); }             \
                                                                                         \
  void _BodyType##Msg::deserialize(QByteArray data)                                      \
  {                                                                                      \
    QDataStream stream(&data, QIODevice::ReadOnly);                                      \
    stream.setVersion(QDataStream::Qt_4_5);                                              \
    HeaderMsg tmp_header;                                                                \
    stream >> tmp_header;                                                                \
    if (header.msg_type != _MSG_TYPE)                                                    \
      return;                                                                            \
    stream >> body;                                                                      \
    header = tmp_header;                                                                 \
  }                                                                                      \
                                                                                         \
  QByteArray serialize(const _BodyType##Msg& data)                                       \
  {                                                                                      \
    QByteArray buf;                                                                      \
    QDataStream strm(&buf, QIODevice::WriteOnly);                                        \
    strm.setVersion(QDataStream::Qt_4_5);                                                \
    strm << data.header << data.body;                                                    \
    return buf;                                                                          \
  }
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

enum MsgType : quint32
{
  NONE,
  MOTOR_STATUS,
  WINCH_STATUS,
  ACTUATOR_STATUS
};

//----------------------  MESSAGE HEADER -------------------------//

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

struct BaseMsg
{
  BaseMsg(const MsgType msg_t) : header(msg_t) {}
  BaseMsg(const double time_sec, const MsgType msg_t) : header(time_sec, msg_t) {}

  HeaderMsg header;
};

MSG_STRUCT_DECLARE(MOTOR_STATUS, MotorStatus)
MSG_STRUCT_DECLARE(WINCH_STATUS, WinchStatus)
MSG_STRUCT_DECLARE(ACTUATOR_STATUS, ActuatorStatus)

MSG_SERIALIZATION_DECLARE(MotorStatus)
MSG_SERIALIZATION_DECLARE(WinchStatus)
MSG_SERIALIZATION_DECLARE(ActuatorStatus)

#endif // MSGS_H
