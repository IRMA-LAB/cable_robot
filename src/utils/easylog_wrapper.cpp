#include "utils/easylog_wrapper.h"

void LogMotorStatusMsg(el::Logger* data_logger, const MotorStatusMsg& msg)
{
  // clang-format off
  data_logger->info("%v,%v,%v,%v,%v,%v,%v",
                                                                msg.header.msg_type,
                                                                msg.header.timestamp,
                                     static_cast<int>(msg.body.id),
                                     static_cast<int>(msg.body.op_mode),
                                                                msg.body.motor_position,
                                                                msg.body.motor_speed,
                                                                msg.body.motor_torque);
  // clang-format on
}

void LogWinchStatusMsg(el::Logger* data_logger, const WinchStatusMsg& msg)
{
  // clang-format off
  data_logger->info("%v,%v,%v,%v,%v,%v,%v,%v,%v",
                                                                msg.header.msg_type,
                                                                msg.header.timestamp,
                                     static_cast<int>(msg.body.id),
                                     static_cast<int>(msg.body.op_mode),
                                                                msg.body.motor_position,
                                                                msg.body.motor_speed,
                                                                msg.body.motor_torque,
                                                                msg.body.cable_length,
                                                                msg.body.aux_position);
  // clang-format on
}

void LogActuatorStatusMsg(el::Logger* data_logger, const ActuatorStatusMsg& msg)
{
  // clang-format off
  data_logger->info("%v,%v,%v,%v,%v,%v,%v,%v,%v,%v",
                                                                msg.header.msg_type,
                                                                msg.header.timestamp,
                                     static_cast<int>(msg.body.id),
                                     static_cast<int>(msg.body.op_mode),
                                                                msg.body.motor_position,
                                                                msg.body.motor_speed,
                                                                msg.body.motor_torque,
                                                                msg.body.cable_length,
                                                                msg.body.aux_position,
                                                                msg.body.pulley_angle);
  // clang-format on
}

// ... add new message log function definition here

////////////////////////////////////////  LOG BUFFER
/////////////////////////////////////////

void LogBuffer::Stop()
{
  mutex_.lock();
  stop_requested_ = true;
  buffer_not_empty.wakeAll();
  mutex_.unlock();
  wait();
}

void LogBuffer::collectMsg(QByteArray msg)
{
  static quint64 linear_cnt = 0;

  // Wait until buffer is no longer full
  mutex_.lock();
  if (circular_cnt_ == buffer_.size())
    buffer_not_full.wait(&mutex_);
  mutex_.unlock();

  // Add cast data to buffer
  quint16 index = static_cast<quint16>(linear_cnt % buffer_.size());
  buffer_[index].replace(0, msg.size(), msg);
  linear_cnt++;

  // Update circular counter and notify buffer is no longer empty
  mutex_.lock();
  ++circular_cnt_;
  buffer_not_empty.wakeAll();
  mutex_.unlock();
}

void LogBuffer::run()
{
  quint64 linear_cnt = 0;
  while (1)
  {
    // If buffer is empty, wait until new data is available or user stop request
    mutex_.lock();
    if (circular_cnt_ == 0)
    {
      if (!stop_requested_) // need this is case buffer empties out after a stop request
        buffer_not_empty.wait(&mutex_);
      if (stop_requested_)
      {
        mutex_.unlock();
        break;
      }
    }
    mutex_.unlock();

    QCoreApplication::processEvents();

    // Actual logging step
    quint16 index = static_cast<quint16>(linear_cnt % buffer_.size());
    LogData(index);
    linear_cnt++;

    // Update circular counter and notify buffer is no longer full
    mutex_.lock();
    --circular_cnt_;
    buffer_not_full.wakeAll();
    mutex_.unlock();

    QCoreApplication::processEvents();
  }
}

void LogBuffer::LogData(const quint16 index)
{
  QDataStream stream(buffer_[index]);
  stream.setVersion(QDataStream::Qt_4_5);
  stream >> header_;
  switch (header_.msg_type)
  {
  case NULL_MSG:
    break;
  case MOTOR_STATUS:
    motor_status_.deserialize(buffer_[index]);
    LogMotorStatusMsg(logger_, motor_status_);
    break;
  case WINCH_STATUS:
    winch_status_.deserialize(buffer_[index]);
    LogWinchStatusMsg(logger_, winch_status_);
    break;
  case ACTUATOR_STATUS:
    actuator_status_.deserialize(buffer_[index]);
    LogActuatorStatusMsg(logger_, actuator_status_);
    break;
  // ... add new case here
  }
}
