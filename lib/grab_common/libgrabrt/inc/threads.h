/**
 * @file threads.h
 * @author Simone Comari
 * @date 14 Sep 2018
 * @brief This file collects utilities to create a new thread in a user-friendly way, hiding most
 * of the complexity linked to multi-threading. It also allows the setup of a real-time thread.
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
 * @brief Number of physical CPU cores on Linux-based systems.
 * @note Not cross-platform valid!
 */
#define CPU_CORES_NUM sysconf(_SC_NPROCESSORS_ONLN)
#endif

/**
 * @brief Namespace for real-time and multi-threading utilities of GRAB software.
 */
namespace grabrt
{

#ifndef THREAD_RUN
/**
 * @brief User-friendly macro to start a new thread after setting up the Thread object.
 * @param[in] t An instance of Thread class with the attributes of the new thread.
 * @see Thread
 * @todo add usage example
 */
#define THREAD_RUN(t)                                                                    \
  int ret = pthread_create(t.GetThreadIDPtr(), t.GetAttrPtr(), t._StaticTargetFun, &t);  \
  if (ret != 0)                                                                          \
    t.HandleErrorEnWrapper(ret, "pthread_create ");
#endif

/**
* @brief Enum to collect particular core entries as argument to BuildCPUSet().
* @see BuildCPUSet()
*/
enum CpuCores
{
  END_CORE = -1, /**< select end physical core on the machine (#CPU_CORES_NUM - 1). */
  ALL_CORES = -2 /**< select all physical cores on the machine. */
};

/**
 * @brief Build a valid CPU set with a single core or all of them.
 * @param[in] cpu_core (Optional) The single core to be included in the set. Valid values
 * are:
 * - #ALL_CORES : all CPU cores will be included in the set. This is the _default_ option.
 * - #END_CORE : end core only will be selected.
 * - @f$k \in@f$ [0, #CPU_CORES_NUM - 1] : include selected core only from available ones.
 * @return A CPU set.
 * @see CpuCores
 * @note For further details about the implementation, click
 * <a href="https://www.systutorials.com/docs/linux/man/3-CPU_SET/">here</a>.
 */
cpu_set_t BuildCPUSet(const int cpu_core = ALL_CORES);

/**
 * @brief Build a valid CPU set with an arbitrary, yet valid, number of cores.
 * @param[in] cpu_cores A vector of cores to be included in the set. Valid values inside
 * the vector are:
 * - @f$k \in@f$ [0, #CPU_CORES_NUM - 1]
 * - #END_CORE
 * @return A CPU set.
 * @see CpuCores BuildCPUSet()
 */
cpu_set_t BuildCPUSet(const std::vector<size_t>& cpu_cores);

/**
 * @brief Set a given CPU set for a specific thread.
 * @param[in] cpu_set A valid CPU set.
 * @param[in] thread_id (Optional) The PID of the thread to be set. By _default_, calling
 * thread is selected.
 * @see BuildCPUSet()
 * @note For further details about the implementation, click <a
 * href="http://man7.org/linux/man-pages/man3/pthread_setaffinity_np.3.html">here</a>.
 */
void SetThreadCPUs(const cpu_set_t& cpu_set, const pthread_t thread_id = pthread_self());

/**
 * @brief Set scheduling attributes for a specific thread.
 * @param[in] policy A scheduling policy. Valid options are @b SCHED_FIFO or @b SCHED_RR
 * for _real-time_ policies, and @b SCHED_OTHER for normal scheduling policies.
 * @param[in] priority (Optional) A scheduling priority. Valid options are [1, 99] for
 * real-time policies and 0 for normal policies. Default value is 1 for the former and 0
 * for the latter.
 * @param[in] thread_id (Optional) The PID of the thread to be set. By _default_, the
 * calling thread is selected.
 * @note For further details about the implementation, click <a
 * href="http://man7.org/linux/man-pages/man3/pthread_setschedparam.3.html">here</a>.
 * For further details about _scheduling policy_ and _priority_ instead click
 * <a href="http://man7.org/linux/man-pages/man7/sched.7.html">here</a>.
 */
void SetThreadSchedAttr(const int policy, const int priority = -1,
                        const pthread_t thread_id = pthread_self());

/**
 * @brief Display CPU set for a specific thread.
 *
 * Example output:
 * @verbatim
    CPU set of thread 140695503214336:
      CPU 2
      CPU 3
    @endverbatim
 * @param[in] thread_id (Optional) The PID of the thread whose CPU set is displayed. By
 * _default_, the calling thread is selected.
 */
void DisplayThreadAffinitySet(const pthread_t thread_id = pthread_self());

/**
 * @brief Display given scheduling policy and priority in a pretty format.
 *
 * Example output:
 * @verbatim
          policy=SCHED_FIFO, priority=15
    @endverbatim
 * @param[in] policy A scheduling policy. Valid options are @b SCHED_FIFO or @b SCHED_RR
 * for _real-time_ policies, and @b SCHED_OTHER for normal scheduling policies.
 * @param[in] param A scheduling parameter structure including scheduling priority.
 * @note For further details about _scheduling policy_ and _priority_ click
 * <a href="http://man7.org/linux/man-pages/man7/sched.7.html">here</a>.
 */
void DisplaySchedAttr(const int policy, const struct sched_param& param);

/**
 * @brief Display scheduling policy and priority of a specific thread in a pretty format.
 *
 * Example output:
 * @verbatim
    Scheduling attributes of thread 139940789114624:
          policy=SCHED_FIFO, priority=15
    @endverbatim
 * @param[in] thread_id (Optional) The PID of the thread whose CPU set is displayed. By
 * _default_, the calling thread is selected.
 */
void DisplayThreadSchedAttr(const pthread_t thread_id = pthread_self());

/**
 * @brief This class provides a simple interface to setup and create a new thread, with
 * normal or real-time characteristics.
 */
class Thread
{
public:
  /**
   * @brief Default constructor.
   *
   * Setup thread with default attributes:
   * @verbatim
        Detach state        = PTHREAD_CREATE_JOINABLE
        Scope               = PTHREAD_SCOPE_SYSTEM
        Inherit scheduler   = PTHREAD_EXPLICIT_SCHED
        Scheduling policy   = SCHED_OTHER
        Scheduling priority = 0
        Guard size          = 4096 bytes
        Stack address       = 0xffffffffff5fc000
        Stack size          = 0xa04000 bytes
     @endverbatim
   * and all physical cores in the CPU set.
   * @param[in] thread_name (Optional) Instance name.
   */
  Thread(const std::string& thread_name = "Thread") : name_(thread_name)
  {
    InitDefault();
  }
  /**
   * @brief Constructor with attributes.
   *
   * Setup thread with given attributes @c attr and all physical cores in the CPU set.
   * @param[in] attr Desired thread attributes.
   * @param[in] thread_name (Optional) Instance name.
   */
  Thread(pthread_attr_t& attr, const std::string& thread_name = "Thread")
    : name_(thread_name)
  {
    SetAttr(attr);
  }
  /**
   * @brief Constructor with CPU set.
   *
   * Setup thread with default attributes:
   * @verbatim
        Detach state        = PTHREAD_CREATE_JOINABLE
        Scope               = PTHREAD_SCOPE_SYSTEM
        Inherit scheduler   = PTHREAD_EXPLICIT_SCHED
        Scheduling policy   = SCHED_OTHER
        Scheduling priority = 0
        Guard size          = 4096 bytes
        Stack address       = 0xffffffffff5fc000
        Stack size          = 0xa04000 bytes
     @endverbatim
   * and given CPU set @c cpu_set.
   * @param[in] cpu_set A valid CPU set.
   * @param[in] thread_name (Optional) Instance name.
   * @see BuildCPUSet()
   */
  Thread(const cpu_set_t& cpu_set, const std::string& thread_name = "Thread");
  /**
   * @brief Constructor with scheduling attributes.
   *
   * Setup thread with given scheduling attributes and all physical cores in the CPU set.
   * @param[in] policy A scheduling policy. Valid options are @b SCHED_FIFO or @b SCHED_RR
   * for _real-time_ policies, and @b SCHED_OTHER for normal scheduling policies.
   * @param[in] priority (Optional) A scheduling priority. Valid options are [1, 99] for
   * real-time policies and 0 for normal policies. Default value is 1 for the former and 0
   * for the latter.
   * @param[in] thread_name (Optional) Instance name.
   * @note For further details about _scheduling policy_ and _priority_ click
   * <a href="http://man7.org/linux/man-pages/man7/sched.7.html">here</a>.
   */
  Thread(const int policy, const int priority = -1,
         const std::string& thread_name = "Thread");
  /**
   * @brief Full constructor.
   * Setup thread with given scheduling attributes and given CPU set @c cpu_set.
   * @param[in] cpu_set A valid CPU set.
   * @param[in] policy A scheduling policy. Valid options are @b SCHED_FIFO or @b SCHED_RR
   * for _real-time_ policies, and @b SCHED_OTHER for normal scheduling policies.
   * @param[in] priority (Optional) A scheduling priority. Valid options are [1, 99] for
   * real-time policies and 0 for normal policies. Default value is 1 for the former and 0
   * for the latter.
   * @param[in] thread_name (Optional) Instance name.
   * @note For further details about _scheduling policy_ and _priority_ click
   * <a href="http://man7.org/linux/man-pages/man7/sched.7.html">here</a>.
   * @see BuildCPUSet()
   */
  Thread(const cpu_set_t& cpu_set, const int policy, const int priority = -1,
         const std::string& thread_name = "Thread");
  ~Thread();

  /**
   * @brief Set thread attributes before it is created.
   * @param[in] attr Desired thread attributes.
   * @note Calling this setter after creating the thread is ineffective. If you wish to
   * set new
   * attributes first close the running thread invoking Stop() and start over.
   */
  void SetAttr(const pthread_attr_t& attr);
  /**
   * @brief Set thread CPU set.
   * @param cpu_set A valid CPU set.
   * @see BuildCPUSet()
   */
  void SetCPUs(const cpu_set_t& cpu_set);
  /**
   * @brief Build a valid CPU set with a single core or all of them and assign it to the
   * thread.
   * @param[in] cpu_core (Optional) The single core to be included in the set. Valid
   * values are:
   * - #ALL_CORES : all CPU cores will be included in the set. This is the _default_
   * option.
   * - #END_CORE : end core only will be selected.
   * - @f$k \in@f$ [0, #CPU_CORES_NUM - 1] : include selected core only from available
   * ones.
   * @see CpuCores BuildCPUSet()
   */
  void SetCPUs(const int cpu_core = ALL_CORES);
  /**
   * @brief Build a valid CPU set with an arbitrary, yet valid, number of cores and assign
   * it to the thread.
   * @param[in] cpu_cores A vector of cores to be included in the set. Valid values inside
   * the vector are:
   * - @f$k \in@f$ [0, #CPU_CORES_NUM - 1]
   * - #END_CORE
   * @see CpuCores BuildCPUSet()
   */
  void SetCPUs(const std::vector<size_t>& cpu_cores);
  /**
   * @brief Set thread scheduling attributes.
   * @param[in] policy A scheduling policy. Valid options are @b SCHED_FIFO or @b SCHED_RR
   * for _real-time_ policies, and @b SCHED_OTHER for normal scheduling policies.
   * @param[in] priority (Optional) A scheduling priority. Valid options are [1, 99] for
   * real-time policies and 0 for normal policies. Default value is 1 for the former and 0
   * for the latter.
   * @note For further details about _scheduling policy_ and _priority_ click
   * <a href="http://man7.org/linux/man-pages/man7/sched.7.html">here</a>.
   */
  void SetSchedAttr(const int policy, int priority = -1);

  /**
   * @brief Set thread _initial_ function.
   *
   * The _initial_ function is called once after the creation of the new thread, before its
   * main loop.
   * @param fun_ptr Pointer to function to be called at the beginning of the new thread.
   * @param args Pointer to optional arguments to the initial function. Set it to @c NULL if
   * not needed.
   * @see SetLoopFunc() SetEndFunc()
   */
  void SetInitFunc(void (*fun_ptr)(void*), void* args);
  /**
   * @brief Set thread _loop_ function.
   *
   * The _loop_ function is called continuosly after the initial function until thread is paused or
   * stopped, either from an internal or external call to the thread.
   * @param fun_ptr Pointer to function to be looped forever.
   * @param args Pointer to optional arguments to the loop function. Set it to @c NULL if
   * not needed.
   * @see SetInitFunc() SetEndFunc()
   */
  void SetLoopFunc(void (*fun_ptr)(void*), void* args);
  /**
   * @brief Set thread _end_ function.
   *
   * The _end_ function is called once after the end of main loop, consequent to a stopping call,
   * right before thread is closed.
   * @param fun_ptr Pointer to function to be called before thread is closed.
   * @param args Pointer to optional arguments to the end function. Set it to @c NULL if
   * not needed.
   * @see SetLoopFunc() SetInitFunc()
   */
  void SetEndFunc(void (*fun_ptr)(void*), void* args);

  /**
   * @brief Get the integral thread of @c this. This is equivalent to its LWP ID in Linux.
   * @return Integral thread ID (aka LWP ID) if thread is active, -1 otherwise.
   * @note Not cross-platform valid!
   * @see GetPID()
   */
  long GetTID() const;
  /**
   * @brief Get the POSIX thread ID of @c *this.
   * @return POSIX thread ID if thread is active, 0 otherwise.
   * @note This is different from its LWP ID in Linux!
   * @see GetTID() GetThreadIDPtr()
   */
  pthread_t GetPID() const;
  /**
   * @brief Get the pointer to POSIX thread ID of @c *this.
   *
   * This function is used to assign the thread ID to this instance upon thread creation.
   * @return A pointer to POSIX thread ID.
   * @see THREAD_RUN
   */
  pthread_t* GetThreadIDPtr() { return &thread_id_; }
  /**
   * @brief Get thread CPU set.
   * @return CPU set of @c *this;
   */
  cpu_set_t GetCPUs();
  /**
   * @brief Get thread scheduling policy.
   * @return %Thread scheduling policy. Output can be @b SCHED_FIFO or @b SCHED_RR
   * for _real-time_ policies, and @b SCHED_OTHER for normal scheduling policies.
   * @note For further details about _scheduling policy_ click
   * <a href="http://man7.org/linux/man-pages/man7/sched.7.html">here</a>.
   * @see GetPriority() SetSchedAttr()
   */
  int GetPolicy() const;
  /**
   * @brief Get thread scheduling priority.
   * @return %Thread scheduling priority. Valid options are [1, 99] for real-time policies and 0
   * for normal policies. Default value is 1 for the former and 0 for the latter.
   * @note For further details about _scheduling priority_ click
   * <a href="http://man7.org/linux/man-pages/man7/sched.7.html">here</a>.
   * @see GetPolicy() SetSchedAttr()
   */
  int GetPriority() const { return sched_param_.sched_priority; }
  /**
   * @brief Get thread attributes.
   * @return %Thread attributes.
   */
  pthread_attr_t GetAttr() const { return attr_; }
  /**
   * @brief Get a pointer to thread attributes.
   *
   * This function is used to pass the thread attributes to the actual thread creator.
   * @return A pointer thread attributes.
   * @see THREAD_RUN
   */
  const pthread_attr_t* GetAttrPtr() const { return &attr_; }
  /**
   * @brief Get thread user-set name in string format.
   * @return A standard string.
   */
  std::string GetName() const { return name_; }
  /**
   * @brief Get thread user-set name in char pointer format.
   * @return A pointer to char string.
   */
  const char* GetNameCstr() const { return name_.c_str(); }

  /**
   * @brief Set thread loop cycle time and verifies if thread setup is completed before creation.
   * @param cycle_time_nsec (Optional) %Thread loop cycle time in nanoseconds.
   * @return 0 if thread is ready to be created. Non-zero error number otherwise.
   * @exception EFAULT Loop function is not set.
   * @see SetInitFunc()
   * @note For further details about error number convention, click
   * <a href="http://man7.org/linux/man-pages/man3/errno.3.html">here</a>.
   */
  int GetReady(const uint64_t cycle_time_nsec = 1000LL);
  /**
   * @brief Pauses a running thread. This stops the loop until thread is unpaused (or closed).
   * @see Unpause()
   */
  void Pause() { run_ = false; }
  /**
   * @brief Unpauses a previously paused thread. This operation resets the cycle clock.
   * @see Pause()
   */
  void Unpause();
  /**
   * @brief Stops and terminates the thread.
   *
   * This operation is not reversible and terminates the thread after completion of last cycle.
   * @see THREAD_RUN Pause()
   */
  void Stop();

  /**
   * @brief Check if thread is up and running.
   * @return True if thread is running.
   */
  bool IsRunning() const { return run_; }
  /**
   * @brief Check if thread is up. This is true also when thread is paused.
   * @return True if thread is up.
   */
  bool IsActive() const { return active_; }

  /**
   * @brief Display thread attributes in a pretty format.
   *
   * Example output:
   * @verbatim
        Detach state        = PTHREAD_CREATE_JOINABLE
        Scope               = PTHREAD_SCOPE_SYSTEM
        Inherit scheduler   = PTHREAD_EXPLICIT_SCHED
        Scheduling policy   = SCHED_OTHER
        Scheduling priority = 0
        Guard size          = 4096 bytes
        Stack address       = 0xffffffffff5fc000
        Stack size          = 0xa04000 bytes
     @endverbatim
   * @todo make this an independent function and replace it with a wrapper
   */
  void DispAttr() const;

  /**
   * @brief This function is a work-around to pass a class member to @c pthread_create().
   * @param obj A pointer to @c *this.
   * @return @c NULL
   * @note This function should not be used by the end-user, even if declared as _public_!
   * @see THREAD_RUN
   */
  static void* _StaticTargetFun(void* obj)
  {
    static_cast<Thread*>(obj)->TargetFun();
    return NULL;
  }

  /**
   * @brief This wrapper adds the instance name before the error message displayed by
   * HandleErrorEn().
   * @see HandleErrorEn()
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
   * @brief Initializes thread with default attributes and CPU set.
   */
  void InitDefault();
  /**
   * @brief The actual function linked to the new thread.
   */
  void TargetFun();
};

} // end namespace grabrt

#endif // GRABCOMMON_LIBGRABRT_THREADS_H
