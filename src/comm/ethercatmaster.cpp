#include "ethercatmaster.h"

EthercatMaster* thisInstance;

void EthercatMaster::ConfigureMemoryLocks()
{
  if (mlockall(MCL_CURRENT | MCL_FUTURE))
    perror("mlockall failed:");  // Memory Lock, no page faults
  mallopt(M_TRIM_THRESHOLD, -1); // Turn off memory trimming
  mallopt(M_MMAP_MAX, 0);        // Turn off shared memory usage
}

void EthercatMaster::LockProcessMemory(uint32_t size)
{
  uint32_t i;
  char* buffer;
  buffer = static_cast<char*>(malloc(size));
  for (i = 0; i < size; i += sysconf(_SC_PAGESIZE))
    buffer[i] = 0; // Send this memory to RAM and lock it there
  free(buffer);
}

void EthercatMaster::PeriodIncrement(PeriodInfo* periodInfo)
{
  periodInfo->nextPeriod.tv_nsec +=
    periodInfo->periodNs; // Incrementing the NextPeriod interval for sleeping,
                          // the timer source is monotonic
  while (periodInfo->nextPeriod.tv_nsec >= nSecondsInSeconds)
  { // tv_nsec has to be less than nSecondsInSEconds, because seconds are dealt
    // with in tv_sec
    periodInfo->nextPeriod.tv_sec++;
    periodInfo->nextPeriod.tv_nsec -= nSecondsInSeconds;
  }
}

uint8_t EthercatMaster::WaitUntilPeriodElapsed(PeriodInfo* periodInfo)
{
  PeriodIncrement(periodInfo); // periodInfo structure handling
  uint8_t msg = static_cast<uint8_t>(
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &periodInfo->nextPeriod,
                    NULL)); // Just Sleep until the end of the time period required
  if (msg != 0)
  {
    std::cout << "An Error Occurred! Error number = " << msg << std::endl;
  }
  return msg;
}

void* EthercatMaster::TheRtThread(void* args)
{
  PeriodInfo periodInfo;
  uint8_t cycleFlag = 1;

  thisInstance->flags.notSync = 0;
  thisInstance->SetSchedulerParameters(thisInstance->masterData.rtCpuId,
                                       thisInstance->masterData.rtPriority);
  periodInfo.periodNs = thisInstance->masterData.nSecondsCycleTime;
  clock_gettime(CLOCK_MONOTONIC, &(periodInfo.nextPeriod));
  thisInstance->StartUpFunction();
  while (cycleFlag)
  {
    pthread_mutex_lock(
      &thisInstance->rtMutex); // Lock the resources while we are using it
    ecrt_master_receive(thisInstance->masterPointer); // Receive data
    ecrt_domain_process(thisInstance->domainPointer);
    thisInstance->CheckConfigState(); // Check ethercat State Machine
    thisInstance->CheckMasterState();
    thisInstance->CheckDomainState();
    if (!thisInstance->flags.notSync && thisInstance->flags.configState &&
        thisInstance->flags.masterState && thisInstance->flags.domainState)
    {
      thisInstance->LoopFunction(); // If everything is in order, execute
                                    // loopfunction of master
    }
    else
    {
      cycleFlag = thisInstance->FlagManagement();
    }
    ecrt_domain_queue(thisInstance->domainPointer); // Write data
    ecrt_master_send(thisInstance->masterPointer);
    pthread_mutex_unlock(&thisInstance->rtMutex);                      // Unlock resources
    thisInstance->flags.notSync = WaitUntilPeriodElapsed(&periodInfo); // sleep
  }
  return args;
}

void EthercatMaster::CheckDomainState() // Check ethercat domain state machine
{
  ec_domain_state_t domainStateLocal;

  ecrt_domain_state(domainPointer, &domainStateLocal);

  if (domainStateLocal.working_counter != domainState.working_counter)
  {
    std::cout << "Domain: WC " << domainStateLocal.working_counter << std::endl;
  }
  if (domainStateLocal.wc_state != domainState.wc_state)
  {
    std::cout << "Domain: State " << domainStateLocal.wc_state << std::endl;
    if (domainStateLocal.wc_state == domainOperational)
      flags.domainState = operationalState;
    else
      flags.domainState = notOperationalState;
  }

  domainState = domainStateLocal;
}

void EthercatMaster::CheckMasterState() // Check ethercat master state machine
{
  ec_master_state_t masterStateLocal;

  ecrt_master_state(masterPointer, &masterStateLocal);

  if (masterStateLocal.slaves_responding != masterState.slaves_responding)
  {
    std::cout << masterStateLocal.slaves_responding << " slave(s) on the bus"
              << std::endl;
  }
  if (masterStateLocal.al_states != masterState.al_states)
  {
    std::cout << "Master states: " << masterStateLocal.al_states << std::endl;
    if (masterStateLocal.al_states == masterOperational)
      flags.masterState = operationalState;
    else
      flags.masterState = notOperationalState;
  }
  if (masterStateLocal.link_up != masterState.link_up)
  {
    std::cout << "Master Link is " << (masterStateLocal.link_up ? "up" : "down")
              << std::endl;
  }

  masterState = masterStateLocal;
}

void EthercatMaster::CheckConfigState() // Check ethercat slave configuration
                                        // state machine
{
  ec_slave_config_state_t configStateLocal;

  ecrt_slave_config_state(configPointer, &configStateLocal);

  if (configStateLocal.al_state != configState.al_state)
  {
    std::cout << "Slaves State " << configStateLocal.al_state << std::endl;
    if (configStateLocal.al_state == slaveOperational)
      flags.configState = operationalState;
    else
      flags.configState = notOperationalState;
  }
  if (configStateLocal.online != configState.online)
  {
    std::cout << "Slaves: " << (configStateLocal.online ? "online" : "offline")
              << std::endl;
  }
  if (configStateLocal.operational != configState.operational)
  {
    std::cout << "Slaves: " << (configStateLocal.operational ? "" : "Not ")
              << "operational" << std::endl;
  }

  configState = configStateLocal;
}

void EthercatMaster::GetDomainElements(ec_pdo_entry_reg_t* regs) // Wrapper..
{
  uint16_t index = 0;
  for (int i = 0; i < numberOfSlaves; i++)
  {
    for (int j = 0; j < slave[i]->numberOfDomainEntries; j++)
    {
      regs[index] = slave[i]->domainRegistersPointer[j];
      index++;
    }
  }
}

void EthercatMaster::SetSchedulerParameters(uint8_t threadCpu,
                                            uint8_t threadPriority) // Thread utility..
{
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(threadCpu, &cpuset);
  struct sched_param schedulerParameters;
  schedulerParameters.sched_priority = threadPriority;
  sched_setaffinity(0, sizeof(cpuset), &cpuset);
  sched_setscheduler(0, SCHED_RR, &schedulerParameters);
}

uint8_t EthercatMaster::FlagManagement()
{
  // Nothing is currently implemented for ethercat state transition
  // Even though we do nothings, if no damage is present in the hardware
  // and our installation, the transition to operational gets 10 ms
  if (flags.notSync)
  { // We check for
    return notValidate;
  }
  return validate;
}

uint8_t EthercatMaster::InitProtocol()
{
  ec_pdo_entry_reg_t domainRegistersLocal[domainElementsNumber];

  if (!(masterPointer = ecrt_request_master(0)))
  { // Requesting to initialize master 0
    std::cout << "Error requesting master" << std::endl;
    return notValidate;
  }
  if (!(domainPointer = ecrt_master_create_domain(masterPointer)))
  { // Creating Domain Process associated with master 0
    std::cout << "Error Creating Domain" << std::endl;
    return notValidate;
  }

  for (int i = 0; i < numberOfSlaves; i++)
  { // Configuring Slaves
    if (!(configPointer =
            ecrt_master_slave_config(masterPointer, slave[i]->alias, slave[i]->position,
                                     slave[i]->vendor_id, slave[i]->product_code)))
    {
      std::cout << "Error Configuring Slave Devices" << std::endl;
      return notValidate;
    }
    std::cout << "Configuring PDOs" << std::endl;
    if (ecrt_slave_config_pdos(configPointer, EC_END, slave[i]->slaveSyncsPointer))
    {
      std::cout << "Error Configuring PDOs" << std::endl;
      return notValidate;
    }
    if (slave[i]->SdoRequests(sdoPointer, configPointer))
    {
      return notValidate;
    }
  }

  GetDomainElements(domainRegistersLocal); // Configuring Domain
  if (ecrt_domain_reg_pdo_entry_list(domainPointer, domainRegistersLocal))
  {
    std::cout << "Error Registering PDOs' entries" << std::endl;
    return notValidate;
  }

  std::cout << "Activating master" << std::endl;
  if (ecrt_master_activate(masterPointer))
  { // Activating Master
    std::cout << "Error activating Master" << std::endl;
    return notValidate;
  }
  if (!(domainDataPointer = ecrt_domain_data(domainPointer)))
  { // Activating Domain
    std::cout << "Error Initializing Domain Data" << std::endl;
    return notValidate;
  }

  for (int i = 0; i < numberOfSlaves; i++)
    slave[i]->Init(domainDataPointer); // Performing Slave initialization
                                       // function, default is empty function

  return validate;
}

EthercatMaster::EthercatMaster()
{
  thisInstance = this; // we need the address of the actual master, we'll need
                       // it in the static functions
}

EthercatMaster::~EthercatMaster() {}

void EthercatMaster::Start()
{

  pthread_t threadRt;
  pthread_attr_t threadAttributes;

  ConfigureMemoryLocks();               // Call immediately
  LockProcessMemory(preAllocationSize); // Call immediately

  /* init to default values */
  if (pthread_attr_init(&threadAttributes))
  {
    std::cout << "pthread_attr_init failed" << std::endl;
  }
  else
  {
    if (pthread_attr_setstacksize(&threadAttributes, PTHREAD_STACK_MIN + thisStackSize))
    {
      std::cout << "pthread_attr_setstacksize failed" << std::endl;
    }
    else
    {
      if (thisInstance->InitProtocol())
      { // If everything is ok, start the master thread on a differrent thread
        SetSchedulerParameters(masterData.guiCpuId,
                               masterData.guiPriority); // Set priority for gui thread
        pthread_create(&threadRt, &threadAttributes, TheRtThread,
                       NULL); // master thread start
      }
      else
        std::cout << "Could not initialize Ethercat Devices" << std::endl;
    }
  }
}
