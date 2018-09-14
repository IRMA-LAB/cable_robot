/**
 * @file threads.cpp
 * @author Simone Comari
 * @date 14 Sep 2018
 * @brief File containing definitions of functions and class declared in threads.h.
 */

#include "threads.h"

namespace grabrt
{

cpu_set_t BuildCPUSet(const int cpu_core /*= ALL_CORES*/)
{
  // init
  cpu_set_t cpu_set;
  CPU_ZERO(&cpu_set);
  // validity check
  if (cpu_core < ALL_CORES || cpu_core >= static_cast<int>(CPU_CORES_NUM))
    HandleErrorEn(EINVAL, "BuildCPUSet ");
  switch (cpu_core)
  {
  case ALL_CORES: // set all cores
    for (size_t i = 0; i < static_cast<size_t>(CPU_CORES_NUM); i++)
      CPU_SET(i, &cpu_set);
    break;
  case END_CORE:                                               // set affine single core
    CPU_SET(static_cast<size_t>(CPU_CORES_NUM) - 1, &cpu_set); // last core
    break;
  default: // set affine single core
    CPU_SET(static_cast<size_t>(cpu_core), &cpu_set);
    break;
  }
  return cpu_set;
}

cpu_set_t BuildCPUSet(const std::vector<size_t>& cpu_cores)
{
  // init
  cpu_set_t cpu_set;
  CPU_ZERO(&cpu_set);
  // validity check on set size
  if (cpu_cores.size() > static_cast<size_t>(CPU_CORES_NUM))
    HandleErrorEn(EINVAL, "BuildCPUSet ");
  // set affine cores
  for (auto const& core : cpu_cores)
  {
    // validity check on single core
    if (core >= static_cast<size_t>(CPU_CORES_NUM))
      HandleErrorEn(EINVAL, "BuildCPUSet ");

    CPU_SET(core, &cpu_set);
  }
  return cpu_set;
}

void SetThreadCPUs(const cpu_set_t& cpu_set,
                   const pthread_t thread_id /*= pthread_self()*/)
{
  int ret = pthread_setaffinity_np(thread_id, sizeof(cpu_set_t), &cpu_set);
  if (ret != 0)
    HandleErrorEn(ret, "pthread_setaffinity_np ");
}

void SetThreadPolicy(const int policy, const int priority /*= -1*/,
                     const pthread_t thread_id /*= pthread_self()*/)
{
  struct sched_param param;
  if (priority < 0)
  {
    // Default priority value depends on policy type
    if (policy == SCHED_FIFO || policy == SCHED_RR)
      param.sched_priority = 1;
    else
      param.sched_priority = 0;
  }
  else
  {
    if (policy == SCHED_OTHER)
    {
      printf(ANSI_COLOR_YELLOW
             "WARNING: Priority for SCHED_OTHER policy must be 0. Ignoring invalid "
             "user-set priority: %d." ANSI_COLOR_RESET "\n",
             priority);
      param.sched_priority = 0;
    }
    else
      param.sched_priority = priority;
  }

  int ret = pthread_setschedparam(thread_id, policy, &param);
  if (ret != 0)
    HandleErrorEn(ret, "pthread_setschedparam ");
}

void displayThreadAffinitySet()
{
  cpu_set_t cpuset;
  int s = pthread_getaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
  if (s != 0)
    HandleErrorEn(s, "pthread_getaffinity_np ");

  printf("Set returned by pthread_getaffinity_np() contained:\n");
  for (size_t j = 0; j < CPU_SETSIZE; j++)
    if (CPU_ISSET(j, &cpuset))
      printf("    CPU %lu\n", j);
}

void displaySchedAttr(const int policy, const struct sched_param& param)
{
  printf("    policy=%s, priority=%d\n",
         (policy == SCHED_FIFO)
           ? "SCHED_FIFO"
           : (policy == SCHED_RR) ? "SCHED_RR" : (policy == SCHED_OTHER) ? "SCHED_OTHER"
                                                                         : "???",
         param.sched_priority);
}

void displayThreadSchedAttr(const std::string& msg)
{
  int policy, s;
  struct sched_param param;

  s = pthread_getschedparam(pthread_self(), &policy, &param);
  if (s != 0)
    HandleErrorEn(s, "pthread_getschedparam ");

  printf("%s\n", msg.c_str());
  displaySchedAttr(policy, param);
}

/////////////////////////////////////////////////
/// Thread Class Methods
/////////////////////////////////////////////////

Thread::Thread(const cpu_set_t& cpu_set, const std::string& thread_name /*= "Thread"*/)
{
  name_ = thread_name;
  InitDefault();
  SetCPUs(cpu_set);
}

Thread::Thread(const int policy, const int priority /*= -1*/,
               const std::string& thread_name /*= "Thread"*/)
{
  name_ = thread_name;
  InitDefault();
  SetPolicy(policy, priority);
}

Thread::Thread(const cpu_set_t& cpu_set, const int policy, const int priority /*= -1*/,
               const std::string& thread_name /*= "Thread"*/)
{
  name_ = thread_name;
  InitDefault();
  SetCPUs(cpu_set);
  SetPolicy(policy, priority);
}

Thread::~Thread()
{
  Stop();
  pthread_attr_destroy(&attr_);
}

void Thread::SetAttr(const pthread_attr_t& attr)
{
  int ret;
  attr_ = attr; // copy values
  // Update class members
  ret = pthread_attr_getaffinity_np(&attr_, sizeof(cpu_set_t), &cpu_set_);
  if (ret != 0)
    HandleErrorEnWrapper(ret, "pthread_attr_getaffinity_np ");
  ret = pthread_attr_getschedparam(&attr_, &sched_param_);
  if (ret != 0)
    HandleErrorEnWrapper(ret, "pthread_attr_getschedparam ");
}

void Thread::SetCPUs(const cpu_set_t& cpu_set)
{
  pthread_mutex_lock(&mutex_);
  cpu_set_ = cpu_set;
  if (IsActive())
    SetThreadCPUs(cpu_set_, thread_id_);
  pthread_mutex_unlock(&mutex_);
}

void Thread::SetCPUs(const int cpu_core /*= ALL_CORES*/)
{
  pthread_mutex_lock(&mutex_);
  cpu_set_ = BuildCPUSet(cpu_core);
  if (IsActive())
    SetThreadCPUs(cpu_set_, thread_id_);
  pthread_mutex_unlock(&mutex_);
}

void Thread::SetCPUs(const std::vector<size_t>& cpu_cores)
{
  pthread_mutex_lock(&mutex_);
  cpu_set_ = BuildCPUSet(cpu_cores);
  if (IsActive())
    SetThreadCPUs(cpu_set_, thread_id_);
  pthread_mutex_unlock(&mutex_);
}

void Thread::SetPolicy(const int policy, const int priority /*= -1*/)
{
  int ret;
  ret = pthread_attr_setschedpolicy(&attr_, policy);
  if (ret != 0)
    HandleErrorEnWrapper(ret, "pthread_attr_setschedpolicy ");

  if (priority < 0)
  {
    // Default priority value depends on policy type
    if (policy == SCHED_FIFO || policy == SCHED_RR)
      sched_param_.sched_priority = 1;
    else
      sched_param_.sched_priority = 0;
  }
  else
  {
    if (policy == SCHED_OTHER)
    {
      printf(ANSI_COLOR_YELLOW
             "WARNING: Priority for SCHED_OTHER policy must be 0. Ignoring invalid "
             "user-set priority: %d." ANSI_COLOR_RESET "\n",
             priority);
      sched_param_.sched_priority = 0;
    }
    else
      sched_param_.sched_priority = priority;
  }

  ret = pthread_attr_setschedparam(&attr_, &sched_param_);
  if (ret != 0)
    HandleErrorEnWrapper(ret, "pthread_attr_setschedparam ");

  if (IsActive())
    SetThreadPolicy(policy, sched_param_.sched_priority, thread_id_);
}

void Thread::SetInitFunc(void (*fun_ptr)(void*), void* args)
{
  if (IsRunning())
    printf(
      ANSI_COLOR_YELLOW
      "WARNING: Thread is running. New InitFunc set but not effective!" ANSI_COLOR_RESET
      "\n");

  init_fun_ptr_ = fun_ptr;
  init_fun_args_ptr_ = args;
}

void Thread::SetLoopFunc(void (*fun_ptr)(void*), void* args)
{
  if (IsRunning())
  {
    printf(ANSI_COLOR_YELLOW
           "WARNING: Thread is running. Cannot set new LoopFunc now!" ANSI_COLOR_RESET
           "\n");
  }
  else
  {
    loop_fun_ptr_ = fun_ptr;
    loop_fun_args_ptr_ = args;
  }
}

void Thread::SetEndFunc(void (*fun_ptr)(void*), void* args)
{
  if (IsActive())
  {
    printf(
      ANSI_COLOR_YELLOW
      "WARNING: Thread is closed. New EndFunc set but not effective!" ANSI_COLOR_RESET
      "\n");
  }
  end_fun_ptr_ = fun_ptr;
  end_fun_args_ptr_ = args;
}

long Thread::GetTID() const { return IsRunning() ? tid_ : -1; }

pthread_t Thread::GetPID() const
{
  if (IsRunning())
    return thread_id_;
  else
  {
    printf(ANSI_COLOR_YELLOW "WARNING: Thread is not running!" ANSI_COLOR_RESET "\n");
    return 0;
  }
}

cpu_set_t Thread::GetCPUs()
{
  pthread_mutex_lock(&mutex_);
  cpu_set_t cpu_set_copy = cpu_set_;
  pthread_mutex_unlock(&mutex_);
  return cpu_set_copy;
}

int Thread::GetPolicy() const
{
  int policy;
  int ret = pthread_attr_getschedpolicy(&attr_, &policy);
  if (ret != 0)
    HandleErrorEnWrapper(ret, "pthread_attr_getschedpolicy ");
  return policy;
}

int Thread::GetReady(const uint64_t cycle_time_nsec /*= 1000LL*/)
{
  if (loop_fun_ptr_ == NULL)
    return EFAULT;

  cycle_time_nsec_ = cycle_time_nsec;
  active_ = true;
  run_ = true;
  return 0;
}

void Thread::Unpause()
{
  if (IsActive())
    run_ = true;
}

void Thread::Stop()
{
  if (IsActive())
  {
    Pause();
    active_ = false;
    pthread_join(thread_id_, NULL);
  }
}

void Thread::DispAttr() const
{
  int ret, i;
  size_t v;
  void* stkaddr;
  const char* prefix = "\t";

  ret = pthread_attr_getdetachstate(&attr_, &i);
  if (ret != 0)
    HandleErrorEnWrapper(ret, "pthread_attr_getdetachstate");
  printf("%sDetach state        = %s\n", prefix,
         (i == PTHREAD_CREATE_DETACHED)
           ? "PTHREAD_CREATE_DETACHED"
           : (i == PTHREAD_CREATE_JOINABLE) ? "PTHREAD_CREATE_JOINABLE" : "???");

  ret = pthread_attr_getscope(&attr_, &i);
  if (ret != 0)
    HandleErrorEnWrapper(ret, "pthread_attr_getscope");
  printf("%sScope               = %s\n", prefix,
         (i == PTHREAD_SCOPE_SYSTEM)
           ? "PTHREAD_SCOPE_SYSTEM"
           : (i == PTHREAD_SCOPE_PROCESS) ? "PTHREAD_SCOPE_PROCESS" : "???");

  ret = pthread_attr_getinheritsched(&attr_, &i);
  if (ret != 0)
    HandleErrorEnWrapper(ret, "pthread_attr_getinheritsched");
  printf("%sInherit scheduler   = %s\n", prefix,
         (i == PTHREAD_INHERIT_SCHED)
           ? "PTHREAD_INHERIT_SCHED"
           : (i == PTHREAD_EXPLICIT_SCHED) ? "PTHREAD_EXPLICIT_SCHED" : "???");

  ret = pthread_attr_getschedpolicy(&attr_, &i);
  if (ret != 0)
    HandleErrorEnWrapper(ret, "pthread_attr_getschedpolicy");
  printf("%sScheduling policy   = %s\n", prefix,
         (i == SCHED_OTHER) ? "SCHED_OTHER" : (i == SCHED_FIFO)
                                                ? "SCHED_FIFO"
                                                : (i == SCHED_RR) ? "SCHED_RR" : "???");

  printf("%sScheduling priority = %d\n", prefix, sched_param_.sched_priority);

  ret = pthread_attr_getguardsize(&attr_, &v);
  if (ret != 0)
    HandleErrorEnWrapper(ret, "pthread_attr_getguardsize");
  printf("%sGuard size          = %d bytes\n", prefix, static_cast<int>(v));

  ret = pthread_attr_getstack(&attr_, &stkaddr, &v);
  if (ret != 0)
    HandleErrorEnWrapper(ret, "pthread_attr_getstack");
  printf("%sStack address       = %p\n", prefix, stkaddr);
  printf("%sStack size          = 0x%zx bytes\n", prefix, v);
}

void Thread::InitDefault()
{
  int ret;
  ret = pthread_attr_init(&attr_);
  if (ret != 0)
    HandleErrorEnWrapper(ret, "pthread_attr_init ");

  ret = pthread_attr_setstacksize(&attr_, PTHREAD_STACK_MIN + kStackSize);
  if (ret != 0)
    HandleErrorEnWrapper(ret, "pthread_attr_setstacksize ");

  ret = pthread_attr_setinheritsched(&attr_, PTHREAD_EXPLICIT_SCHED);
  if (ret != 0)
    HandleErrorEnWrapper(ret, "pthread_attr_setinheritsched ");

  cpu_set_ = BuildCPUSet(); // using all cores by default
}

void Thread::TargetFun()
{
  pthread_mutex_lock(&mutex_);
  tid_ = syscall(__NR_gettid);
  SetThreadCPUs(cpu_set_);
  pthread_mutex_unlock(&mutex_);
  ThreadClock clock(cycle_time_nsec_);
  clock.Reset();

  if (init_fun_ptr_ != NULL)
  {
    pthread_mutex_lock(&mutex_);
    init_fun_ptr_(init_fun_args_ptr_);
    pthread_mutex_unlock(&mutex_);
  }

  while (active_)
  {
    while (run_)
    {
      clock.WaitUntilNext();

      pthread_mutex_lock(&mutex_);
      loop_fun_ptr_(loop_fun_args_ptr_);
      pthread_mutex_unlock(&mutex_);
    }
    clock.Reset();
  }

  if (end_fun_ptr_ != NULL)
  {
    pthread_mutex_lock(&mutex_);
    end_fun_ptr_(end_fun_args_ptr_);
    pthread_mutex_unlock(&mutex_);
  }
}

[[noreturn]] void Thread::HandleErrorEnWrapper(const int en, const char* msg) const
{
  std::string full_msg = "[";
  full_msg.append(name_);
  full_msg.append("] ");
  full_msg.append(msg);
  HandleErrorEn(en, full_msg.c_str());
}

} // end namespace grabrt
