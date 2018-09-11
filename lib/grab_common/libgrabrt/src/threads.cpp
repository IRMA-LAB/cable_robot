#include "threads.h"

namespace grabrt
{

void ThreadClock::Next()
{
  time_.tv_sec = (static_cast<uint64_t>(time_.tv_nsec) + period_nsec_) / kNanoSec2Sec_;
  time_.tv_nsec = (static_cast<uint64_t>(time_.tv_nsec) + period_nsec_) % kNanoSec2Sec_;
}

void ThreadClock::WaitUntilNext()
{
  Next();
  int ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &time_, NULL);
  if (ret != 0)
  {
    std::string err, msg;
    switch (ret)
    {
    case EFAULT:
      err = "EFAULT";
      msg = "invalid address";
      break;
    case EINTR:
      err = "EINTR";
      msg = "received interrupt";
      break;
    case EINVAL:
      err = "EFAULT";
      msg = "invalid value";
      break;
    default:
      err = "Unknown";
      msg = "???";
    }
    printf("%s error: %s\n", err.c_str(), msg.c_str());
  }
}

Thread::Thread(const cpu_set_t* cpu_set)
{
  InitDefault();
  SetCPUs(cpu_set);
}

Thread::Thread(const int policy, const int priority /*= -1*/)
{
  InitDefault();
  SetPolicy(policy, priority);
}

Thread::Thread(const cpu_set_t* cpu_set, const int policy, const int priority /*= -1*/)
{
  InitDefault();
  SetCPUs(cpu_set);
  SetPolicy(policy, priority);
}

Thread::~Thread()
{
  Close();
  pthread_attr_destroy(&attr_);
  free(init_fun_args_ptr_);
  free(loop_fun_args_ptr_);
  free(end_fun_args_ptr_);
}

void Thread::SetAttr(const pthread_attr_t* attr)
{
  int s;
  attr_ = *attr; // copy values
  // Update class members
  s = pthread_attr_getaffinity_np(&attr_, sizeof(cpu_set_t), &cpu_set_);
  if (s != 0)
    HandleError(s, "pthread_attr_getaffinity_np ");
  s = pthread_attr_getschedparam(&attr_, &sched_param_);
  if (s != 0)
    HandleError(s, "pthread_attr_getschedparam ");
}

void Thread::SetCPUs(const cpu_set_t* cpu_set)
{
  cpu_set_ = *cpu_set;
  if (IsActive())
  {
    int s = pthread_setaffinity_np(thread_id_, sizeof(cpu_set_t), &cpu_set_);
    if (s != 0)
      HandleError(s, "pthread_setaffinity_np ");
  }
}

void Thread::SetPolicy(const int policy, const int priority /*= -1*/)
{
  int s;
  s = pthread_attr_setschedpolicy(&attr_, policy);
  if (s != 0)
    HandleError(s, "pthread_attr_setschedpolicy ");

  if (priority < 0)
  {
    // Default priority value depends on policy type
    if (policy == SCHED_FIFO || policy == SCHED_RR)
      sched_param_.sched_priority = 1;
    else
      sched_param_.sched_priority = 0;
  }
  else
    sched_param_.sched_priority = priority;

  s = pthread_attr_setschedparam(&attr_, &sched_param_);
  if (s != 0)
    HandleError(s, "pthread_attr_setschedparam ");
}

void Thread::SetInitFunc(void (*fun_ptr)(void*), void* args)
{
  if (IsRunning())
    std::cerr << "[WARNING] Thread is running. New InitFunc set but not effective!"
              << std::endl;

  init_fun_ptr_ = fun_ptr;
  init_fun_args_ptr_ = args;
}

void Thread::SetLoopFunc(void (*fun_ptr)(void*), void* args)
{
  if (IsRunning())
    std::cerr << "[WARNING] Thread is running. Cannot set new LoopFunc now!" << std::endl;
  else
  {
    loop_fun_ptr_ = fun_ptr;
    loop_fun_args_ptr_ = args;
  }
}

void Thread::SetEndFunc(void (*fun_ptr)(void*), void* args)
{
  if (IsActive())
    std::cerr << "[WARNING] Thread is closed. New EndFunc set but not effective!"
              << std::endl;

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
    std::cerr << "[WARNING] Thread is not running!" << std::endl;
    return 0;
  }
}

int Thread::GetPolicy() const
{
  int policy;
  int s = pthread_attr_getschedpolicy(&attr_, &policy);
  if (s != 0)
    HandleError(s, "pthread_attr_getschedpolicy ");
  return policy;
}

void Thread::Start(const uint64_t cycle_time_nsec /*= 1000LL*/)
{
  cycle_time_nsec_ = cycle_time_nsec;
  active_ = true;
  run_ = true;
  int s = pthread_create(&thread_id_, &attr_, StaticTargetFun, NULL);
  if (s != 0)
    HandleError(s, "pthread_create ");
}

void Thread::Close()
{
  Stop();
  active_ = false;
  pthread_join(thread_id_, NULL);
}

void Thread::DispAttr() const
{
  int s, i;
  size_t v;
  void* stkaddr;
  const char* prefix = "\t";

  s = pthread_attr_getdetachstate(&attr_, &i);
  if (s != 0)
    HandleError(s, "pthread_attr_getdetachstate");
  printf("%sDetach state        = %s\n", prefix,
         (i == PTHREAD_CREATE_DETACHED)
           ? "PTHREAD_CREATE_DETACHED"
           : (i == PTHREAD_CREATE_JOINABLE) ? "PTHREAD_CREATE_JOINABLE" : "???");

  s = pthread_attr_getscope(&attr_, &i);
  if (s != 0)
    HandleError(s, "pthread_attr_getscope");
  printf("%sScope               = %s\n", prefix,
         (i == PTHREAD_SCOPE_SYSTEM)
           ? "PTHREAD_SCOPE_SYSTEM"
           : (i == PTHREAD_SCOPE_PROCESS) ? "PTHREAD_SCOPE_PROCESS" : "???");

  s = pthread_attr_getinheritsched(&attr_, &i);
  if (s != 0)
    HandleError(s, "pthread_attr_getinheritsched");
  printf("%sInherit scheduler   = %s\n", prefix,
         (i == PTHREAD_INHERIT_SCHED)
           ? "PTHREAD_INHERIT_SCHED"
           : (i == PTHREAD_EXPLICIT_SCHED) ? "PTHREAD_EXPLICIT_SCHED" : "???");

  s = pthread_attr_getschedpolicy(&attr_, &i);
  if (s != 0)
    HandleError(s, "pthread_attr_getschedpolicy");
  printf("%sScheduling policy   = %s\n", prefix,
         (i == SCHED_OTHER) ? "SCHED_OTHER" : (i == SCHED_FIFO)
                                                ? "SCHED_FIFO"
                                                : (i == SCHED_RR) ? "SCHED_RR" : "???");

  printf("%sScheduling priority = %d\n", prefix, sched_param_.sched_priority);

  s = pthread_attr_getguardsize(&attr_, &v);
  if (s != 0)
    HandleError(s, "pthread_attr_getguardsize");
  printf("%sGuard size          = %d bytes\n", prefix, static_cast<int>(v));

  s = pthread_attr_getstack(&attr_, &stkaddr, &v);
  if (s != 0)
    HandleError(s, "pthread_attr_getstack");
  printf("%sStack address       = %p\n", prefix, stkaddr);
  printf("%sStack size          = 0x%zx bytes\n", prefix, v);
}

void Thread::InitDefault()
{
  int s;
  s = pthread_attr_init(&attr_);
  if (s != 0)
    HandleError(s, "pthread_attr_init ");

  s = pthread_attr_setstacksize(&attr_, PTHREAD_STACK_MIN + kStackSize_);
  if (s != 0)
    HandleError(s, "pthread_attr_setstacksize ");
}

void Thread::TargetFun()
{
  int s = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpu_set_);
  if (s != 0)
    HandleError(s, "pthread_setaffinity_np ");

  ThreadClock clock(cycle_time_nsec_);
  clock.Reset();

  pthread_mutex_lock(&mutex_);
  init_fun_ptr_(init_fun_args_ptr_);
  pthread_mutex_unlock(&mutex_);

  while (active_)
  {
    while (run_)
    {
      clock.WaitUntilNext();

      pthread_mutex_lock(&mutex_);
      loop_fun_ptr_(loop_fun_args_ptr_);
      pthread_mutex_unlock(&mutex_);
    }
  }

  pthread_mutex_lock(&mutex_);
  end_fun_ptr_(end_fun_args_ptr_);
  pthread_mutex_unlock(&mutex_);
}

} // end namespace grabrt
