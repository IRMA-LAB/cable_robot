#ifndef GRABCOMMON_LIBGRABRT_THREADS_H
#define GRABCOMMON_LIBGRABRT_THREADS_H

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/syscall.h>
#include <iostream>

namespace grabrt
{

class ThreadClock
{
public:
  ThreadClock(const uint64_t cycle_time_nsec = 1000LL) : period_nsec_(cycle_time_nsec) {}

  void SetCycleTime(const uint64_t cycle_time_nsec) { period_nsec_ = cycle_time_nsec; }
  void Reset() { clock_gettime(CLOCK_MONOTONIC, &time_); }
  void Next();
  void WaitUntilNext();

private:
  static constexpr uint64_t kNanoSec2Sec_ = 1000000000L;

  struct timespec time_;
  uint64_t period_nsec_;
};

class Thread
{
public:
  Thread() { InitDefault(); }
  Thread(pthread_attr_t* attr) { SetAttr(attr); }
  Thread(const cpu_set_t* cpu_set);
  Thread(const int policy, const int priority = -1);
  Thread(const cpu_set_t* cpu_set, const int policy, const int priority = -1);
  ~Thread();

  void SetAttr(const pthread_attr_t* attr);
  void SetCPUs(const cpu_set_t* cpu_set);
  void SetPolicy(const int policy, int priority = -1);

  void SetInitFunc(void (*fun_ptr)(void*), void* args);
  void SetLoopFunc(void (*fun_ptr)(void*), void* args);
  void SetEndFunc(void (*fun_ptr)(void*), void* args);

  long GetTID() const;
  pthread_t GetPID() const;
  cpu_set_t GetCPUs() const { return cpu_set_; }
  int GetPolicy() const;
  int GetPriority() const { return sched_param_.sched_priority; }
  pthread_attr_t GetAttr() const { return attr_; }

  void Start(const uint64_t cycle_time_nsec = 1000LL);
  void Stop() { run_ = false; }
  void Close();

  bool IsRunning() const { return run_; }
  bool IsActive() const { return active_; }

  void DispAttr() const;

private:
  static constexpr unsigned long kStackSize_ = 10 * 1024 * 1024; // 10 Mb

  long tid_;
  pthread_t thread_id_;
  pthread_attr_t attr_;
  struct sched_param sched_param_;
  cpu_set_t cpu_set_;
  uint64_t cycle_time_nsec_;

  void (*init_fun_ptr_)(void*) = NULL;
  void (*loop_fun_ptr_)(void*) = NULL;
  void (*end_fun_ptr_)(void*) = NULL;
  void* init_fun_args_ptr_ = NULL;
  void* loop_fun_args_ptr_ = NULL;
  void* end_fun_args_ptr_ = NULL;

  bool run_ = false;
  bool active_ = false;

  pthread_mutex_t mutex_ = PTHREAD_MUTEX_INITIALIZER;

  void InitDefault();
  void HandleError(const int en, const char* msg) const;
  void TargetFun();

  static void* StaticTargetFun(void* args)
  {
    static_cast<Thread*>(args)->TargetFun();
    return NULL;
  }
};

} // end namespace grabrt

#endif // GRABCOMMON_LIBGRABRT_THREADS_H
