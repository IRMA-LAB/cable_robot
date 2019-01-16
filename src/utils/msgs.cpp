#include "utils/msgs.h"

//----------------------  QDataStream operators for basic types -------------------------//

QDataStream& operator<<(QDataStream& ostream, const size_t& value)
{
  return ostream << static_cast<quint32>(value);
}

QDataStream& operator>>(QDataStream& istream, size_t& value)
{
  quint32 qvalue;
  istream >> qvalue;
  value = static_cast<size_t>(qvalue);
  return istream;
}

QDataStream& operator>>(QDataStream& istream, MsgType& value)
{
  quint32 qvalue;
  istream >> qvalue;
  value = static_cast<MsgType>(qvalue);
  return istream;
}

//----------------------  MESSAGE HEADER -------------------------//

QDataStream& operator<<(QDataStream& ostream, const HeaderMsg& data)
{
  return ostream << data.msg_type << data.timestamp;
}

QDataStream& operator>>(QDataStream& istream, HeaderMsg& data)
{
  istream >>  data.msg_type >> data.timestamp;
  return istream;
}

//----------------------  MESSAGES -------------------------//

const size_t kMaxMsgSize = 45; // todo: compute automatically somehow..

// clang-format off
MSG_FIELDS_ORDER_DEFINE(MotorStatus,
                        id,
                        op_mode,
                        motor_position,
                        motor_speed,
                        motor_torque)
MSG_FIELDS_ORDER_DEFINE(WinchStatus,
                        id,
                        op_mode,
                        motor_position,
                        motor_speed,
                        motor_torque,
                        cable_length,
                        aux_position)
MSG_FIELDS_ORDER_DEFINE(ActuatorStatus,
                        id,
                        op_mode,
                        motor_position,
                        motor_speed,
                        motor_torque,
                        cable_length,
                        aux_position,
                        pulley_angle)
// ... message order goes here, this is how you need to handle data in parser for example.
// E.g. MSG_FIELDS_ORDER_DEFINE(MyTypeStruct, field1, field2, ...)
// clang-format on

MSG_SERIALIZATION_DEFINE(MOTOR_STATUS, MotorStatus)
MSG_SERIALIZATION_DEFINE(WINCH_STATUS, WinchStatus)
MSG_SERIALIZATION_DEFINE(ACTUATOR_STATUS, ActuatorStatus)
// ... and definition here, e.g. MSG_SERIALIZATION_DEFINE(MY_TYPE, MyTypeStruct)
