#ifndef GRABCOMMON_LIBGRABRT_THREADS_H
#define GRABCOMMON_LIBGRABRT_THREADS_H

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/syscall.h>
#include <iostream>
#include <vector>
#include <limits.h>

#ifndef CPU_CORES_NUM
#define CPU_CORES_NUM sysconf(_SC_NPROCESSORS_ONLN)
#endif

namespace grabrt
{

#ifndef THREAD_RUN
#define THREAD_RUN(t)                                                                    \
  int ret = pthread_create(&(t.thread_id_), t.GetAttrPtr(), t.StaticTargetFun, &t);      \
  if (ret != 0)                                                                          \
    t.HandleErrorEnWrapper(ret, "pthread_create ");
#endif

enum CpuCores
{
  END_CORE = -1,
  ALL_CORES = -2
};

uint64_t Sec2NanoSec(const double seconds);

double NanoSec2Sec(const long nanoseconds);

[[noreturn]] void HandleErrorEn(const int en, const char* msg);

cpu_set_t BuildCPUSet(const int cpu_core = ALL_CORES);

cpu_set_t BuildCPUSet(const std::vector<size_t>& cpu_cores);

void SetThreadCPUs(const cpu_set_t& cpu_set, const pthread_t thread_id = pthread_self());

void SetThreadPolicy(const int policy, const int priority = -1,
                     const pthread_t thread_id = pthread_self());

void displayThreadAffinitySet();

void displaySchedAttr(const int policy, const struct sched_param& param);

void displayThreadSchedAttr(const std::string& msg);

class ThreadClock
{
public:
  ThreadClock(const uint64_t cycle_time_nsec = 1000LL,
              const std::string& clk_name = "ThreadClock")
    : name_(clk_name), period_nsec_(cycle_time_nsec)
  {
  }

  void SetCycleTime(const uint64_t cycle_time_nsec) { period_nsec_ = cycle_time_nsec; }
  void Reset();
  void Next();
  void WaitUntilNext();

  std::string GetName() const { return name_; }
  uint64_t GetCycleTime() const { return period_nsec_; }
  struct timespec GetCurrentTime() const { return time_; }
  struct timespec GetNextTime();

  void DispCurrentTime() const;
  void DispNextTime();

private:
  static constexpr uint64_t kNanoSec2Sec = 1000000000L;

  std::string name_;
  struct timespec time_;
  uint64_t period_nsec_;

  [[noreturn]] void HandleErrorEnWrapper(const int en, const char* msg) const;
};

class Thread
{
public:
  Thread(const std::string& thread_name = "Thread") : name_(thread_name)
  {
    InitDefault();
  }
  Thread(pthread_attr_t& attr, const std::string& thread_name = "Thread")
    : name_(thread_name)
  {
    SetAttr(attr);
  }
  Thread(const cpu_set_t& cpu_set, const std::string& thread_name = "Thread");
  Thread(const int policy, const int priority = -1,
         const std::string& thread_name = "Thread");
  Thread(const cpu_set_t& cpu_set, const int policy, const int priority = -1,
         const std::string& thread_name = "Thread");
  ~Thread();

  pthread_t thread_id_;
  pthread_attr_t attr_;

  void SetAttr(const pthread_attr_t& attr);
  void SetCPUs(const cpu_set_t& cpu_set);
  void SetCPUs(const int cpu_core = ALL_CORES);
  void SetCPUs(const std::vector<size_t>& cpu_cores);
  void SetPolicy(const int policy, int priority = -1);

  void SetInitFunc(void (*fun_ptr)(void*), void* args);
  void SetLoopFunc(void (*fun_ptr)(void*), void* args);
  void SetEndFunc(void (*fun_ptr)(void*), void* args);

  long GetTID() const;
  pthread_t GetPID() const;
  cpu_set_t GetCPUs();
  int GetPolicy() const;
  int GetPriority() const { return sched_param_.sched_priority; }
  pthread_attr_t GetAttr() const { return attr_; }
  const pthread_attr_t* GetAttrPtr() const { return &attr_; }
  std::string GetName() const { return name_; }

  int GetReady(const uint64_t cycle_time_nsec = 1000LL);
  void Pause() { run_ = false; }
  void Unpause();
  void Stop();

  bool IsRunning() const { return run_; }
  bool IsActive() const { return active_; }

  void DispAttr() const;

  static void* StaticTargetFun(void* obj)
  {
    static_cast<Thread*>(obj)->TargetFun();
    return NULL;
  }

  [[noreturn]] void HandleErrorEnWrapper(const int en, const char* msg) const;

private:
  static constexpr unsigned long kStackSize_ = 10 * 1024 * 1024; // 10 Mb

  std::string name_;
  long tid_;
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
  void TargetFun();
};

} // end namespace grabrt

#endif // GRABCOMMON_LIBGRABRT_THREADS_H
