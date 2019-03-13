/**
 * @file msgs.cpp
 * @author Simone Comari
 * @date 11 Mar 2019
 * @brief This file includes definitions of functions declared in msgs.h.
 * @todo Compute kMaxMsgSize automatically somehow..
 */

#include "utils/msgs.h"

//------- QDataStream operators for non-Qt types -------------------------------------//

/**
 * @brief operator << for size_t type serialization.
 * @param[out] ostream
 * @param[in] value Basic type value to be serialized
 * @return
 */
QDataStream& operator<<(QDataStream& ostream, const size_t& value)
{
  return ostream << static_cast<quint32>(value);
}

/**
 * @brief operator >> for size_t type deserialization.
 * @param[in] istream
 * @param[out] value Basic type to be assigned.
 * @return
 */
QDataStream& operator>>(QDataStream& istream, size_t& value)
{
  quint32 qvalue;
  istream >> qvalue;
  value = static_cast<size_t>(qvalue);
  return istream;
}

/**
 * @brief operator >> for MsgType type deserialization.
 * @param[in] istream
 * @param[out] value Custom type to be assigned.
 * @return
 */
QDataStream& operator>>(QDataStream& istream, MsgType& value)
{
  quint32 qvalue;
  istream >> qvalue;
  value = static_cast<MsgType>(qvalue);
  return istream;
}

//------- MESSAGE HEADER -------------------------------------------------------------//

QDataStream& operator<<(QDataStream& ostream, const HeaderMsg& data)
{
  return ostream << data.msg_type << data.timestamp;
}

QDataStream& operator>>(QDataStream& istream, HeaderMsg& data)
{
  istream >> data.msg_type >> data.timestamp;
  return istream;
}

//------- MESSAGES -------------------------------------------------------------------//

const size_t kMaxMsgSize = 48; // TODO: compute automatically somehow..

// clang-format off
// @cond DO_NOT_DOCUMENT
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
                        state,
                        pulley_angle)
// ... message order goes here, this is how you need to handle data in parser for example.
// E.g. MSG_FIELDS_ORDER_DEFINE(MyTypeStruct, field1, field2, ...)
// @endcond
// clang-format on

MSG_SERIALIZATION_DEFINE(MOTOR_STATUS, MotorStatus)
MSG_SERIALIZATION_DEFINE(WINCH_STATUS, WinchStatus)
MSG_SERIALIZATION_DEFINE(ACTUATOR_STATUS, ActuatorStatus)
// ... and definition here, e.g. MSG_SERIALIZATION_DEFINE(MY_TYPE, MyTypeStruct)
