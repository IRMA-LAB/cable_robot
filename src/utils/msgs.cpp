#include "utils/msgs.h"

//----------------------  MACROS -------------------------//

// Make a FOREACH macro
#define FE_1(WHAT, X) WHAT(X)
#define FE_2(WHAT, X, ...) WHAT(X) FE_1(WHAT, __VA_ARGS__)
#define FE_3(WHAT, X, ...) WHAT(X) FE_2(WHAT, __VA_ARGS__)
#define FE_4(WHAT, X, ...) WHAT(X) FE_3(WHAT, __VA_ARGS__)
#define FE_5(WHAT, X, ...) WHAT(X) FE_4(WHAT, __VA_ARGS__)
#define FE_6(WHAT, X, ...) WHAT(X) FE_5(WHAT, __VA_ARGS__)
#define FE_7(WHAT, X, ...) WHAT(X) FE_6(WHAT, __VA_ARGS__)
#define FE_8(WHAT, X, ...) WHAT(X) FE_7(WHAT, __VA_ARGS__)
//... repeat as needed
#define GET_MACRO(_1, _2, _3, _4, _5, _6, _7, _8, NAME, ...) NAME
#define FOR_EACH(action, ...)                                                            \
  GET_MACRO(__VA_ARGS__, FE_8, FE_7, FE_6, FE_5, FE_4, FE_3, FE_2, FE_1)(action,         \
                                                                         __VA_ARGS__)
// Actions (WHAT)
#define OSTREAM_MSG_FIELD(_field) << data._field
#define ISTREAM_MSG_FIELD(_field) >> data._field

#ifndef MSG_FIELDS_ORDER_DEFINE
#define MSG_FIELDS_ORDER_DEFINE(_MsgType, ...)                                           \
  QDataStream& operator<<(QDataStream& ostream, const _MsgType& data)                    \
  {                                                                                      \
    return ostream FOR_EACH(OSTREAM_MSG_FIELD, __VA_ARGS__);                             \
  }                                                                                      \
                                                                                         \
  QDataStream& operator>>(QDataStream& istream, _MsgType& data)                          \
  {                                                                                      \
    istream FOR_EACH(ISTREAM_MSG_FIELD, __VA_ARGS__);                                    \
    return istream;                                                                      \
  }
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

//----------------------  MESSAGE HEADER -------------------------//

QDataStream& operator<<(QDataStream& ostream, const HeaderMsg& data)
{
  return ostream << data.msg_type << data.timestamp;
}

QDataStream& operator>>(QDataStream& istream, HeaderMsg& data)
{
  quint32 msg_type;
  istream >> msg_type >> data.timestamp;
  data.msg_type = static_cast<MsgType>(msg_type);
  return istream;
}

//----------------------  MESSAGES -------------------------//

const size_t kMaxMsgSize = 44; // todo: compute automatically somehow..

// clang-format off
MSG_FIELDS_ORDER_DEFINE(MotorStatus,
                        op_mode,
                        motor_position,
                        motor_speed,
                        motor_torque)
MSG_FIELDS_ORDER_DEFINE(WinchStatus,
                        op_mode,
                        motor_position,
                        motor_speed,
                        motor_torque,
                        cable_length,
                        aux_position)
MSG_FIELDS_ORDER_DEFINE(ActuatorStatus,
                        op_mode,
                        motor_position,
                        motor_speed,
                        motor_torque,
                        cable_length,
                        aux_position,
                        pulley_angle,
                        id)
// ... message order goes here, this is how you need to handle data in parser for example.
// E.g. MSG_FIELDS_ORDER_DEFINE(MyTypeStruct, field1, field2, ...)
// clang-format on

MSG_SERIALIZATION_DEFINE(MOTOR_STATUS, MotorStatus)
MSG_SERIALIZATION_DEFINE(WINCH_STATUS, WinchStatus)
MSG_SERIALIZATION_DEFINE(ACTUATOR_STATUS, ActuatorStatus)
// ... and definition here, e.g. MSG_SERIALIZATION_DEFINE(MY_TYPE, MyTypeStruct)
