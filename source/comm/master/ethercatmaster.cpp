#include "ethercatmaster.h"

static EthercatMaster* this_instance;

void EthercatMaster::ConfigureMemoryLocks()
{
  if (mlockall(MCL_CURRENT | MCL_FUTURE))
    perror("mlockall failed:");  // Memory Lock, no page faults
  mallopt(M_TRIM_THRESHOLD, -1); // Turn off memory trimming
  mallopt(M_MMAP_MAX, 0);        // Turn off shared memory usage
}

void EthercatMaster::LockProcessMemory(uint32_t size)
{
  char* buffer;
  buffer = static_cast<char*>(malloc(size));
  for (uint32_t i = 0; i < size; i += sysconf(_SC_PAGESIZE))
    buffer[i] = 0; // Send this memory to RAM and lock it there
  free(buffer);
}

void EthercatMaster::PeriodIncrement(PeriodInfo* periodInfo)
{
  periodInfo->next_period.tv_nsec +=
    periodInfo->period_nsec; // Incrementing the NextPeriod interval for sleeping,
                             // the timer source is monotonic
  while (periodInfo->next_period.tv_nsec >= nSecondsInSeconds)
  { // tv_nsec has to be less than nSecondsInSEconds, because seconds are dealt
    // with in tv_sec
    periodInfo->next_period.tv_sec++;
    periodInfo->next_period.tv_nsec -= nSecondsInSeconds;
  }
}

uint8_t EthercatMaster::WaitUntilPeriodElapsed(PeriodInfo* periodInfo)
{
  PeriodIncrement(periodInfo); // periodInfo structure handling
  uint8_t msg = static_cast<uint8_t>(
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &periodInfo->next_period,
                    NULL)); // Just Sleep until the end of the time period required
  if (msg != 0)
  {
    std::cout << "An Error Occurred! Error number = " << msg << std::endl;
  }
  return msg;
}

void* EthercatMaster::TheRtThread(void* args)
{
  PeriodInfo period_info;
  uint8_t cycle_flag = 1;

  this_instance->flags_.not_sync = 0;
  this_instance->SetSchedulerParameters(this_instance->master_data_.rt_cpu_id,
                                        this_instance->master_data_.rt_priority);
  period_info.period_nsec = this_instance->master_data_.cycle_cime_nsec;
  clock_gettime(CLOCK_MONOTONIC, &(period_info.next_period));
  this_instance->StartUpFunction();
  while (cycle_flag)
  {
    pthread_mutex_lock(
      &this_instance->rt_mutex_); // Lock the resources while we are using it
    ecrt_master_receive(this_instance->master_ptr_); // Receive data
    ecrt_domain_process(this_instance->domain_ptr_);
    this_instance->CheckConfigState(); // Check ethercat State Machine
    this_instance->CheckMasterState();
    this_instance->CheckDomainState();
    if (!this_instance->flags_.not_sync && this_instance->flags_.config_state &&
        this_instance->flags_.master_state && this_instance->flags_.domain_state)
    {
      this_instance->LoopFunction(); // If everything is in order, execute
                                     // loopfunction of master
    }
    else
    {
      cycle_flag = this_instance->FlagManagement();
    }
    ecrt_domain_queue(this_instance->domain_ptr_); // Write data
    ecrt_master_send(this_instance->master_ptr_);
    pthread_mutex_unlock(&this_instance->rt_mutex_); // Unlock resources
    this_instance->flags_.not_sync = WaitUntilPeriodElapsed(&period_info); // sleep
  }
  return args;
}

void EthercatMaster::CheckDomainState() // Check ethercat domain state machine
{
  ec_domain_state_t domain_state_local;

  ecrt_domain_state(domain_ptr_, &domain_state_local);

  if (domain_state_local.working_counter != domain_state_.working_counter)
  {
    std::cout << "Domain: WC " << domain_state_local.working_counter << std::endl;
  }
  if (domain_state_local.wc_state != domain_state_.wc_state)
  {
    std::cout << "Domain: State " << domain_state_local.wc_state << std::endl;
    if (domain_state_local.wc_state == kDomainOperational)
      flags_.domain_state = kOperationalState;
    else
      flags_.domain_state = kNotOperationalState;
  }

  domain_state_ = domain_state_local;
}

void EthercatMaster::CheckMasterState() // Check ethercat master state machine
{
  ec_master_state_t master_state_local;

  ecrt_master_state(master_ptr_, &master_state_local);

  if (master_state_local.slaves_responding != master_state_.slaves_responding)
  {
    std::cout << master_state_local.slaves_responding << " slave(s) on the bus"
              << std::endl;
  }
  if (master_state_local.al_states != master_state_.al_states)
  {
    std::cout << "Master states: " << master_state_local.al_states << std::endl;
    if (master_state_local.al_states == kMasterOperational)
      flags_.master_state = kOperationalState;
    else
      flags_.master_state = kNotOperationalState;
  }
  if (master_state_local.link_up != master_state_.link_up)
  {
    std::cout << "Master Link is " << (master_state_local.link_up ? "up" : "down")
              << std::endl;
  }

  master_state_ = master_state_local;
}

void EthercatMaster::CheckConfigState() // Check ethercat slave configuration
                                        // state machine
{
  ec_slave_config_state_t config_state_local;

  ecrt_slave_config_state(config_ptr_, &config_state_local);

  if (config_state_local.al_state != config_state_.al_state)
  {
    std::cout << "Slaves State " << config_state_local.al_state << std::endl;
    if (config_state_local.al_state == kSlaveOperational)
      flags_.config_state = kOperationalState;
    else
      flags_.config_state = kNotOperationalState;
  }
  if (config_state_local.online != config_state_.online)
  {
    std::cout << "Slaves: " << (config_state_local.online ? "online" : "offline")
              << std::endl;
  }
  if (config_state_local.operational != config_state_.operational)
  {
    std::cout << "Slaves: " << (config_state_local.operational ? "" : "Not ")
              << "operational" << std::endl;
  }

  config_state_ = config_state_local;
}

void EthercatMaster::GetDomainElements(ec_pdo_entry_reg_t* regs) // Wrapper..
{
  uint16_t index = 0;
  for (int i = 0; i < num_slaves_; i++)
  {
    for (uint8_t j = 0; j < slave_[i]->num_domain_entries_; j++)
    {
      regs[index] = slave_[i]->domain_registers_ptr_[j];
      index++;
    }
  }
}

// Thread utility
void EthercatMaster::SetSchedulerParameters(uint8_t thread_cpu, uint8_t thread_priority)
{
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(thread_cpu, &cpuset);
  struct sched_param schedulerParameters;
  schedulerParameters.sched_priority = thread_priority;
  sched_setaffinity(0, sizeof(cpuset), &cpuset);
  sched_setscheduler(0, SCHED_RR, &schedulerParameters);
}

uint8_t EthercatMaster::FlagManagement()
{
  // Nothing is currently implemented for ethercat state transition
  // Even though we do nothings, if no damage is present in the hardware
  // and our installation, the transition to operational gets 10 ms
  if (flags_.not_sync)
  { // We check for
    return kNotValidate_;
  }
  return kValidate_;
}

uint8_t EthercatMaster::InitProtocol()
{
  ec_pdo_entry_reg_t domain_registers_local[num_domain_elements_];

  if (!(master_ptr_ = ecrt_request_master(0)))
  { // Requesting to initialize master 0
    std::cout << "Error requesting master" << std::endl;
    return kNotValidate_;
  }
  if (!(domain_ptr_ = ecrt_master_create_domain(master_ptr_)))
  { // Creating Domain Process associated with master 0
    std::cout << "Error Creating Domain" << std::endl;
    return kNotValidate_;
  }

  for (int i = 0; i < num_slaves_; i++)
  { // Configuring Slaves
    if (!(config_ptr_ =
            ecrt_master_slave_config(master_ptr_, slave_[i]->alias_, slave_[i]->position_,
                                     slave_[i]->vendor_id_, slave_[i]->product_code_)))
    {
      std::cout << "Error Configuring Slave Devices" << std::endl;
      return kNotValidate_;
    }
    std::cout << "Configuring PDOs" << std::endl;
    if (ecrt_slave_config_pdos(config_ptr_, EC_END, slave_[i]->slave_sync_ptr_))
    {
      std::cout << "Error Configuring PDOs" << std::endl;
      return kNotValidate_;
    }
    if (slave_[i]->SdoRequests(sdo_ptr_, config_ptr_))
    {
      return kNotValidate_;
    }
  }

  GetDomainElements(domain_registers_local); // Configuring Domain
  if (ecrt_domain_reg_pdo_entry_list(domain_ptr_, domain_registers_local))
  {
    std::cout << "Error Registering PDOs' entries" << std::endl;
    return kNotValidate_;
  }

  std::cout << "Activating master" << std::endl;
  if (ecrt_master_activate(master_ptr_))
  { // Activating Master
    std::cout << "Error activating Master" << std::endl;
    return kNotValidate_;
  }
  if (!(domain_data_ptr_ = ecrt_domain_data(domain_ptr_)))
  { // Activating Domain
    std::cout << "Error Initializing Domain Data" << std::endl;
    return kNotValidate_;
  }

  for (int i = 0; i < num_slaves_; i++)
    slave_[i]->Init(domain_data_ptr_); // Performing Slave initialization
                                       // function, default is empty function

  return kValidate_;
}

EthercatMaster::EthercatMaster()
{
  this_instance = this; // we need the address of the actual master, we'll need
                        // it in the static functions
}

EthercatMaster::~EthercatMaster() {}

void EthercatMaster::Start()
{

  pthread_t thread_rt;
  pthread_attr_t thread_attributes;

  ConfigureMemoryLocks();                 // Call immediately
  LockProcessMemory(kPreAllocationSize_); // Call immediately

  /* init to default values */
  if (pthread_attr_init(&thread_attributes))
  {
    std::cout << "pthread_attr_init failed" << std::endl;
  }
  else
  {
    if (pthread_attr_setstacksize(&thread_attributes, PTHREAD_STACK_MIN + kThisStackSize_))
    {
      std::cout << "pthread_attr_setstacksize failed" << std::endl;
    }
    else
    {
      if (this_instance->InitProtocol())
      { // If everything is ok, start the master thread on a differrent thread
        SetSchedulerParameters(master_data_.gui_cpu_id,
                               master_data_.gui_priority); // Set priority for gui thread
        pthread_create(&thread_rt, &thread_attributes, TheRtThread,
                       NULL); // master thread start
      }
      else
        {
          std::cout << "Could not initialize Ethercat Devices" << std::endl;
        }
    }
  }
}
