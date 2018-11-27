#ifndef CABLE_ROBOT_EASYLOG_WRAPPER_H
#define CABLE_ROBOT_EASYLOG_WRAPPER_H

#include <QCoreApplication>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>

#include "easylogging++.h"

#include "utils/types.h"
#include "utils/msgs.h"

//---------------------- MESSAGE LOG FUNCTIONS -------------------------//

/**
 * @brief LogMotorStatusMsg
 * @param data_logger
 * @param msg
 */
void LogMotorStatusMsg(el::Logger* data_logger, const MotorStatusMsg& msg);
/**
 * @brief LogWinchStatusMsg
 * @param data_logger
 * @param msg
 */
void LogWinchStatusMsg(el::Logger* data_logger, const WinchStatusMsg& msg);
/**
 * @brief LogActuatorStatusMsg
 * @param data_logger
 * @param msg
 */
void LogActuatorStatusMsg(el::Logger* data_logger, const ActuatorStatusMsg& msg);

// ... add new message log function declaration here

/**
 * @brief The LogBuffer class
 */
class LogBuffer : public QThread
{
  Q_OBJECT
public:
  /**
   * @brief LogBuffer
   * @param data_logger
   * @param buffer_size
   */
  LogBuffer(el::Logger* data_logger, const size_t buffer_size = 2000)
    : logger_(data_logger), stop_requested_(false),
      buffer_(buffer_size, QByteArray(static_cast<int>(kMaxMsgSize), 0))
  {}

  /**
   * @brief Stop
   */
  void Stop();

public slots:
  /**
   * @brief CollectMsg
   * @param msg
   */
  void CollectMsg(QByteArray msg);

protected:
  el::Logger* logger_ = NULL;

private:
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

  void LogData(const quint16 index);
};

#endif // CABLE_ROBOT_EASYLOG_WRAPPER_H
