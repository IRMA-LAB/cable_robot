/**
 * @file threads.h
 * @author Simone Comari
 * @date 14 Sep 2018
 * @brief ...
 */

#ifndef GRABCOMMON_LIBGRABRT_THREADS_H
#define GRABCOMMON_LIBGRABRT_THREADS_H

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <iostream>
#include <vector>
#include <limits.h>

#include "grabcommon.h"
#include "clocks.h"

#ifndef CPU_CORES_NUM
/**
 * @brief ...
 */
#define CPU_CORES_NUM sysconf(_SC_NPROCESSORS_ONLN)
#endif

/**
 * @brief
 */
namespace grabrt
{

#ifndef THREAD_RUN
/**
 * @brief ...
 */
#define THREAD_RUN(t)                                                                    \
  int ret = pthread_create(t.GetThreadIDPtr(), t.GetAttrPtr(), t._StaticTargetFun, &t);  \
  if (ret != 0)                                                                          \
    t.HandleErrorEnWrapper(ret, "pthread_create ");
#endif

/**
*@brief The CpuCores enum
*/
enum CpuCores
{
  END_CORE = -1,
  ALL_CORES = -2
};

/**
 * @brief BuildCPUSet
 * @param cpu_core
 * @return
 */
cpu_set_t BuildCPUSet(const int cpu_core = ALL_CORES);

/**
 * @brief BuildCPUSet
 * @param cpu_cores
 * @return
 */
cpu_set_t BuildCPUSet(const std::vector<size_t>& cpu_cores);

/**
 * @brief SetThreadCPUs
 * @param cpu_set
 * @param thread_id
 */
void SetThreadCPUs(const cpu_set_t& cpu_set, const pthread_t thread_id = pthread_self());

/**
 * @brief SetThreadPolicy
 * @param policy
 * @param priority
 * @param thread_id
 */
void SetThreadPolicy(const int policy, const int priority = -1,
                     const pthread_t thread_id = pthread_self());

/**
 * @brief displayThreadAffinitySet
 */
void displayThreadAffinitySet();

/**
 * @brief displaySchedAttr
 * @param policy
 * @param param
 */
void displaySchedAttr(const int policy, const struct sched_param& param);

/**
 * @brief displayThreadSchedAttr
 * @param msg
 */
void displayThreadSchedAttr(const std::string& msg);

/**
 * @brief The Thread class
 */
class Thread
{
public:
  /**
   * @brief Thread
   * @param thread_name
   */
  Thread(const std::string& thread_name = "Thread") : name_(thread_name)
  {
    InitDefault();
  }
  /**
   * @brief Thread
   * @param attr
   * @param thread_name
   */
  Thread(pthread_attr_t& attr, const std::string& thread_name = "Thread")
    : name_(thread_name)
  {
    SetAttr(attr);
  }
  /**
   * @brief Thread
   * @param cpu_set
   * @param thread_name
   */
  Thread(const cpu_set_t& cpu_set, const std::string& thread_name = "Thread");
  /**
   * @brief Thread
   * @param policy
   * @param priority
   * @param thread_name
   */
  Thread(const int policy, const int priority = -1,
         const std::string& thread_name = "Thread");
  /**
   * @brief Thread
   * @param cpu_set
   * @param policy
   * @param priority
   * @param thread_name
   */
  Thread(const cpu_set_t& cpu_set, const int policy, const int priority = -1,
         const std::string& thread_name = "Thread");
  /**
   * @brief ...
   */
  ~Thread();

  /**
   * @brief SetAttr
   * @param attr
   */
  void SetAttr(const pthread_attr_t& attr);
  /**
   * @brief SetCPUs
   * @param cpu_set
   */
  void SetCPUs(const cpu_set_t& cpu_set);
  /**
   * @brief SetCPUs
   * @param cpu_core
   */
  void SetCPUs(const int cpu_core = ALL_CORES);
  /**
   * @brief SetCPUs
   * @param cpu_cores
   */
  void SetCPUs(const std::vector<size_t>& cpu_cores);
  /**
   * @brief SetPolicy
   * @param policy
   * @param priority
   */
  void SetPolicy(const int policy, int priority = -1);

  /**
   * @brief SetInitFunc
   * @param fun_ptr
   * @param args
   */
  void SetInitFunc(void (*fun_ptr)(void*), void* args);
  /**
   * @brief SetLoopFunc
   * @param fun_ptr
   * @param args
   */
  void SetLoopFunc(void (*fun_ptr)(void*), void* args);
  /**
   * @brief SetEndFunc
   * @param fun_ptr
   * @param args
   */
  void SetEndFunc(void (*fun_ptr)(void*), void* args);

  /**
   * @brief GetTID
   * @return
   */
  long GetTID() const;
  /**
   * @brief GetPID
   * @return
   */
  pthread_t GetPID() const;
  /**
   * @brief GetThreadIDPtr
   * @return
   */
  pthread_t* GetThreadIDPtr() { return &thread_id_; }
  /**
   * @brief GetCPUs
   * @return
   */
  cpu_set_t GetCPUs();
  /**
   * @brief GetPolicy
   * @return
   */
  int GetPolicy() const;
  /**
   * @brief GetPriority
   * @return
   */
  int GetPriority() const { return sched_param_.sched_priority; }
  /**
   * @brief GetAttr
   * @return
   */
  pthread_attr_t GetAttr() const { return attr_; }
  /**
   * @brief GetAttrPtr
   * @return
   */
  const pthread_attr_t* GetAttrPtr() const { return &attr_; }
  /**
   * @brief GetName
   * @return
   */
  std::string GetName() const { return name_; }
  /**
   * @brief GetNameCstr
   * @return
   */
  const char* GetNameCstr() const { return name_.c_str(); }

  /**
   * @brief GetReady
   * @param cycle_time_nsec
   * @return
   */
  int GetReady(const uint64_t cycle_time_nsec = 1000LL);
  /**
   * @brief Pause
   */
  void Pause() { run_ = false; }
  /**
   * @brief Unpause
   */
  void Unpause();
  /**
   * @brief Stop
   */
  void Stop();

  /**
   * @brief IsRunning
   * @return
   */
  bool IsRunning() const { return run_; }
  /**
   * @brief IsActive
   * @return
   */
  bool IsActive() const { return active_; }

  /**
   * @brief DispAttr
   */
  void DispAttr() const;

  /**
   * @brief StaticTargetFun
   * @param obj
   * @return
   */
  static void* _StaticTargetFun(void* obj)
  {
    static_cast<Thread*>(obj)->TargetFun();
    return NULL;
  }

  /**
   * @brief ...
   */
  [[noreturn]] void HandleErrorEnWrapper(const int en, const char* msg) const;

private:
  static constexpr unsigned long kStackSize = 10 * 1024 * 1024; // 10 Mb

  pthread_t thread_id_;
  pthread_attr_t attr_;
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

  /**
   * @brief InitDefault
   */
  void InitDefault();
  /**
   * @brief TargetFun
   */
  void TargetFun();
};

} // end namespace grabrt

#endif // GRABCOMMON_LIBGRABRT_THREADS_H
