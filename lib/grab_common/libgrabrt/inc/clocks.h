/**
 * @file clocks.h
 * @author Simone Comari
 * @date 14 Sep 2018
 * @brief ...
 */

#ifndef GRABCOMMON_LIBGRABRT_CLOCKS_H
#define GRABCOMMON_LIBGRABRT_CLOCKS_H

#include <iostream>
#include "grabcommon.h"

namespace grabrt
{

  /**
 * @brief Sec2NanoSec
 * @param seconds
 * @return
 */
uint64_t Sec2NanoSec(const double seconds);
/**
 * @brief NanoSec2Sec
 * @param nanoseconds
 * @return
 */
double NanoSec2Sec(const long nanoseconds);

/**
 * @brief The ThreadClock class
 */
class ThreadClock
{
public:
  /**
   * @brief ThreadClock
   * @param cycle_time_nsec
   * @param clk_name
   */
  ThreadClock(const uint64_t cycle_time_nsec = 1000LL,
              const std::string& clk_name = "ThreadClock")
    : name_(clk_name), period_nsec_(cycle_time_nsec)
  {
  }

  /**
   * @brief SetCycleTime
   * @param cycle_time_nsec
   */
  void SetCycleTime(const uint64_t cycle_time_nsec) { period_nsec_ = cycle_time_nsec; }
  /**
   * @brief Reset
   */
  void Reset();
  /**
   * @brief Next
   */
  void Next();
  /**
   * @brief WaitUntilNext
   */
  void WaitUntilNext();

  /**
   * @brief GetName
   * @return
   */
  std::string GetName() const { return name_; }
  /**
   * @brief GetCycleTime
   * @return
   */
  uint64_t GetCycleTime() const { return period_nsec_; }
  /**
   * @brief GetCurrentTime
   * @return
   */
  struct timespec GetCurrentTime() const { return time_; }
  /**
   * @brief GetNextTime
   * @return
   */
  struct timespec GetNextTime();

  /**
   * @brief DispCurrentTime
   */
  void DispCurrentTime() const;
  /**
   * @brief DispNextTime
   */
  void DispNextTime();

private:
  static constexpr uint64_t kNanoSec2Sec = 1000000000L;

  std::string name_;
  struct timespec time_;
  uint64_t period_nsec_;

  /**
   *
   */
  [[noreturn]] void HandleErrorEnWrapper(const int en, const char* msg) const;
};

} // end namespace grabrt

#endif // GRABCOMMON_LIBGRABRT_CLOCKS_H
