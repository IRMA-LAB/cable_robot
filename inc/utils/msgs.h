/**
 * @file msgs.h
 * @author Simone Comari
 * @date 11 Mar 2019
 * @brief File containing the list of cable robot messages that can be logged.
 */

#ifndef CABLE_ROBOT_MSGS_H
#define CABLE_ROBOT_MSGS_H

#include <QDataStream>

#include "utils/macros.h"
#include "utils/types.h"

//----------------------  MESSAGE LIST -----------------------------------------------//

/**
 * @brief The list of available loggable message types.
 */
enum MsgType : quint32
{
  NULL_MSG,
  MOTOR_STATUS,
  WINCH_STATUS,
  ACTUATOR_STATUS,
  // ... add new message type here, e.g. MY_TYPE, ...
};

//----------------------  MESSAGE HEADER ---------------------------------------------//

/**
 * @brief The default header message struct, common to all messages.
 *
 * It contains information about when the message was created and its type.
 */
struct HeaderMsg
{
  /**
   * @brief HeaderMsg default constructor.
   */
  HeaderMsg() : timestamp(-1.0), msg_type(NULL_MSG) {}
  /**
   * @brief HeaderMsg partial constructor.
   * @param[in] time_sec Time of measurement at source in seconds. This is NOT the logging
   * time, which occurs typically shortly after and is anyway in a different time
   * reference. In the cable robot app, this time is the elapsed time since robot object
   * was instatiated.
   */
  HeaderMsg(const double time_sec) : timestamp(time_sec), msg_type(NULL_MSG) {}
  /**
   * @brief HeaderMsg partial constructor.
   * @param[in] msg_t Message type.
   */
  HeaderMsg(const MsgType msg_t) : timestamp(-1.0), msg_type(msg_t) {}
  /**
   * @brief HeaderMsg full constructor.
   * @param[in] time_sec Time of measurement at source in seconds. This is NOT the logging
   * time, which occurs typically shortly after and is anyway in a different time
   * reference. In the cable robot app, this time is the elapsed time since robot object
   * was instatiated.
   * @param[in] msg_t Message type.
   */
  HeaderMsg(const double time_sec, const MsgType msg_t)
    : timestamp(time_sec), msg_type(msg_t)
  {}

  double timestamp; /**< Time of measurement at source in seconds. */
  MsgType msg_type; /**< Message type. */
};

/**
 * @brief operator << for serialization of HeaderMsg.
 * @param[out] ostream
 * @param[in] data Header message to be serialized.
 * @return Serialized HeaderMsg.
 */
QDataStream& operator<<(QDataStream& ostream, const HeaderMsg& data);
/**
 * @brief operator >> for deserialization of HeaderMsg.
 * @param[in] istream
 * @param[out] data Header message to be filled.
 * @return
 */
QDataStream& operator>>(QDataStream& istream, HeaderMsg& data);

//----------------------  MESSAGES ---------------------------------------------------//

/**
 * @brief The base message structure.
 *
 * This structure is common to each loggable message and includes a header.
 */
struct BaseMsg
{
  /**
   * @brief BaseMsg default constructor.
   * @param[in] msg_t Message type.
   */
  BaseMsg(const MsgType msg_t) : header(msg_t) {}
  /**
   * @brief BaseMsg full constructor.
   * @param[in] time_sec Time of measurement at source in seconds. This is NOT the logging
   * time, which occurs typically shortly after and is anyway in a different time
   * reference. In the cable robot app, this time is the elapsed time since robot object
   * was instatiated.
   * @param[in] msg_t Message type.
   */
  BaseMsg(const double time_sec, const MsgType msg_t) : header(time_sec, msg_t) {}

  HeaderMsg header; /**< Header message including timestamp and message type. */
};

extern const size_t kMaxMsgSize; /**< Maximum serialized message size in bytes. */

MSG_STRUCT_DECLARE(MOTOR_STATUS, MotorStatus)
MSG_STRUCT_DECLARE(WINCH_STATUS, WinchStatus)
MSG_STRUCT_DECLARE(ACTUATOR_STATUS, ActuatorStatus)
// ... and here, e.g. MSG_STRUCT_DECLARE(MY_TYPE, MyTypeStruct), ...

MSG_SERIALIZATION_DECLARE(MotorStatus)
MSG_SERIALIZATION_DECLARE(WinchStatus)
MSG_SERIALIZATION_DECLARE(ActuatorStatus)
// ... and here, e.g. MSG_SERIALIZATION_DECLARE(MyTypeStruct)

#endif // CABLE_ROBOT_MSGS_H
