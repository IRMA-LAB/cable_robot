#include "utils/msgs.h"

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

QDataStream& operator<<(QDataStream& ostream, const MotorStatus& data)
{
  return ostream << data.op_mode << data.motor_position << data.motor_speed
                 << data.motor_torque;
}

QDataStream& operator>>(QDataStream& istream, MotorStatus& data)
{
  istream >> data.op_mode >> data.motor_position >> data.motor_speed >> data.motor_torque;
  return istream;
}

QDataStream& operator<<(QDataStream& ostream, const WinchStatus& data)
{
  return ostream << data.op_mode << data.motor_position << data.motor_speed
                 << data.motor_torque << data.cable_length << data.aux_position;
}

QDataStream& operator>>(QDataStream& istream, WinchStatus& data)
{
  istream >> data.op_mode >> data.motor_position >> data.motor_speed >>
    data.motor_torque >> data.cable_length >> data.aux_position;
  return istream;
}

QDataStream& operator<<(QDataStream& ostream, const ActuatorStatus& data)
{
  return ostream << data.op_mode << data.motor_position << data.motor_speed
                 << data.motor_torque << data.cable_length << data.aux_position
                 << data.pulley_angle << data.id;
}

QDataStream& operator>>(QDataStream& istream, ActuatorStatus& data)
{
  istream >> data.op_mode >> data.motor_position >> data.motor_speed >>
    data.motor_torque >> data.cable_length >> data.aux_position >> data.pulley_angle >>
    data.id;
  return istream;
}

MSG_SERIALIZATION_DEFINE(MOTOR_STATUS, MotorStatus)
MSG_SERIALIZATION_DEFINE(WINCH_STATUS, WinchStatus)
MSG_SERIALIZATION_DEFINE(ACTUATOR_STATUS, ActuatorStatus)
