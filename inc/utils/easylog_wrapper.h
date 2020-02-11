/**
 * @file easylog_wrapper.h
 * @author Simone Comari
 * @date 06 Feb 2020
 * @brief File containing the implementation of a custom wrapper to log cable robot data
 * employing easylogging++ package.
 */

#ifndef CABLE_ROBOT_EASYLOG_WRAPPER_H
#define CABLE_ROBOT_EASYLOG_WRAPPER_H

#include <QCoreApplication>
#include <QMutex>
#include <QThread>
#include <QWaitCondition>

#include "easylogging++.h"

#include "utils/msgs.h"
#include "utils/types.h"

//---------------------- MESSAGE LOG FUNCTIONS ---------------------------------------//

/**
 * @brief Log MotorStatusMsg.
 * @param[in] data_logger Pointer to easylogger employed.
 * @param[in] msg Motor status message to be logged.
 */
void LogMotorStatusMsg(el::Logger* data_logger, const MotorStatusMsg& msg);
/**
 * @brief Log WinchStatusMsg.
 * @param[in] data_logger Pointer to easylogger employed.
 * @param[in] msg Winch status message to be logged.
 */
void LogWinchStatusMsg(el::Logger* data_logger, const WinchStatusMsg& msg);
/**
 * @brief Log ActuatorStatusMsg.
 * @param[in] data_logger Pointer to easylogger employed.
 * @param[in] msg Actuator status message to be logged.
 */
void LogActuatorStatusMsg(el::Logger* data_logger, const ActuatorStatusMsg& msg);

// ... add new message log function declaration here

/**
 * @brief A log buffer to log data using easylogging++ in a thread safe way.
 *
 * Logging, despite being made very easy thanks to easylogging++ package, still takes some
 * time, which can be critical in real time thread with short cycle periods.
 * To account for this, this log buffer has been developed.
 * It takes serialized information, which are thus received and copied very fast at high
 * rates, and dumps them onto the log file whenever it can, without clogging the more
 * demanding source thread.
 *
 * @note To the developer: the queued message needs to be serialized, have a _deserialize_
 * option and for each message a new message-specific log function must be present, such
 * as LogActuatorStatusMsg. Moreover a new case to LogData private function must be add,
 * with the new message enum value.
 */
class LogBuffer: public QThread
{
  Q_OBJECT
 public:
  /**
   * @brief LogBuffer constructor.
   * @param[in] data_logger Pointer to easylogger employed.
   * @param[in] buffer_size Maximum buffer size, i.e. maximum queued messages number.
   */
  LogBuffer(el::Logger* data_logger, const size_t buffer_size = 2000)
    : logger_(data_logger), stop_requested_(false),
      buffer_(buffer_size, QByteArray(static_cast<int>(kMaxMsgSize), 0)), circular_cnt_(0)
  {}

  /**
   * @brief Flush data log up to now.
   */
  void flush();

  /**
   * @brief Stop logging command.
   */
  void stop();

 public slots:
  /**
   * @brief Collect an incoming message to be logged.
   * @param[in] msg The serialized message to be logged
   */
  void collectMsg(QByteArray msg);

 private:
  el::Logger* logger_ = nullptr;

  QMutex mutex_;
  QWaitCondition buffer_not_empty;
  QWaitCondition buffer_not_full;
  bool stop_requested_;

  std::vector<QByteArray> buffer_;
  quint16 circular_cnt_;

  HeaderMsg header_;
  // Full messages
  MotorStatusMsg motor_status_;
  WinchStatusMsg winch_status_;
  ActuatorStatusMsg actuator_status_;
  // ... add new message here

  void run() override;

  void logData(const quint16 index);
};

#endif // CABLE_ROBOT_EASYLOG_WRAPPER_H
