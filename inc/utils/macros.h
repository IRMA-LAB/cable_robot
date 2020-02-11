/**
 * @file macros.h
 * @author Simone Comari
 * @date 11 Feb 2020
 * @brief File containing useful macros used troughout cable robot firmware.
 */

#ifndef CABLE_ROBOT_MACROS_H
#define CABLE_ROBOT_MACROS_H

#include "grabcommon.h"

//---------------------- DECLARATIONS ------------------------------------------------//

// @cond DO_NOT_DOCUMENT
#ifndef MSG_SERIALIZATION_DECLARE
#define MSG_SERIALIZATION_DECLARE(_MsgType)                                              \
  QDataStream& operator<<(QDataStream& ostream, const _MsgType& data);                   \
  QDataStream& operator>>(QDataStream& istream, _MsgType& data);                         \
  QByteArray serialize(const _MsgType##Msg& data);
#endif


#ifndef MSG_STRUCT_DECLARE
#define MSG_STRUCT_DECLARE(_MSG_TYPE, _BodyType)                                         \
  struct _BodyType##Msg: BaseMsg                                                         \
  {                                                                                      \
    _BodyType##Msg() : BaseMsg(_MSG_TYPE) {}                                             \
    _BodyType##Msg(const _BodyType& data) : BaseMsg(_MSG_TYPE), body(data) {}            \
    _BodyType##Msg(const double time_sec, const _BodyType& data)                         \
      : BaseMsg(time_sec, _MSG_TYPE), body(data)                                         \
    {}                                                                                   \
    _BodyType##Msg(const QByteArray data) : BaseMsg(_MSG_TYPE) { deserialize(data); }    \
    QByteArray serialized() const;                                                       \
    void deserialize(QByteArray data);                                                   \
    _BodyType body;                                                                      \
  };
#endif

//---------------------- DEFINITIONS -------------------------------------------------//

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
// @endcond

#endif // CABLE_ROBOT_MACROS_H
