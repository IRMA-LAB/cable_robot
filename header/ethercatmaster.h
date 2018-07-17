#ifndef ETHERCATMASTER_H
#define ETHERCATMASTER_H

/* Ethercat Master interface. This class is used as base class
   in our master. So we won't need every time to deal with:
   - real time stuff
   - memory locking
   - ethercat initialization process
   - how to cyclically loop
   This way all the effort can be put in the design of our specific
   master, with in mind that the ethercat master interface requires
   to overload some function, which are actually called every loop
   This functions are the ones marked as virtual
*/

#include <signal.h>
#include <unistd.h>
#include <malloc.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <pthread.h>
#include <limits.h>
#include <iostream>
#include <string>
#include <cstring>
#include <stdlib.h>
#include <unistd.h>

#include "ecrt.h"          // ethercat library
#include "ethercatslave.h" // ethercat slave degisn interface

struct PeriodInfo
{
  struct timespec nextPeriod;
  long periodNs;
};

using namespace std;

class EthercatMaster
{
private:
  // static function in a class are not tied to any instance of that class,
  // They just reside in the same namespace, so you need to call them in a
  // specific way
  static void ConfigureMemoryLocks();                  // Memory managment
  static void LockProcessMemory(uint32_t size);        // Memory locks
  static void PeriodIncrement(PeriodInfo* periodInfo); // Timer utility
  static void*
  TheRtThread(void* args); // The actual real time thread that is executed
  static uint8_t
  WaitUntilPeriodElapsed(PeriodInfo* periodInfo); // put thread to sleep

  void CheckDomainState();                          // ethercat utility
  void CheckMasterState();                          // ethercat utility
  void CheckConfigState();                          // ethercat utility
  void GetDomainElements(ec_pdo_entry_reg_t* regs); // ethercat utility
  void SetSchedulerParameters(
    uint8_t threadCpu, uint8_t threadPriority); // real time processing utility
  uint8_t FlagManagement();                     // real time process management
  uint8_t InitProtocol();                       // ethercat utility

public:
  EthercatMaster();
  virtual ~EthercatMaster() = 0;

  // constexpr static members of a class can be viewed as #define replacement
  // #definines have the problem to live outside the
  constexpr static uint32_t preAllocationSize =
    100 * 1024 * 1024; /* 100MB pagefault free buffer */
  constexpr static uint32_t thisStackSize = 10 * 1024 * 1024; // 10 Mb
  constexpr static long nSecondsInMillis = 1000000;
  constexpr static long long nSecondsInSeconds = 1000000000;
  constexpr static uint8_t validate = 1;
  constexpr static uint8_t notValidate = 0;
  constexpr static uint8_t masterOperational = 8;
  constexpr static uint8_t slaveOperational = 8;
  constexpr static uint8_t domainOperational = 2;
  constexpr static uint8_t operationalState = 1;
  constexpr static uint8_t notOperationalState = 0;
  // Variables related to our project, must be gived as input
  class MasterData
  {
  public:
    uint8_t guiCpuId = 0;
    uint8_t rtCpuId = 1;
    uint8_t guiPriority = 60;
    uint8_t rtPriority = 98;
    long nSecondsCycleTime = 1000000;
  } masterData;                             // Default Values
  ec_master_t* masterPointer = NULL;        // ethercat utility
  ec_master_state_t masterState = {};       // ethercat utility
  ec_domain_t* domainPointer = NULL;        // ethercat utility
  ec_domain_state_t domainState = {};       // ethercat utility
  ec_slave_config_t* configPointer = NULL;  // ethercat utility
  ec_slave_config_state_t configState = {}; // ethercat utility
  ec_sdo_request_t* sdoPointer = NULL;      // ethercat utility
  uint8_t* domainDataPointer = NULL;        // ethercat utility

  struct EthercatFlags
  {
    uint8_t domainState;
    uint8_t masterState;
    uint8_t configState;
    uint8_t notSync;
  } flags; // ethercat utility

  pthread_mutex_t rtMutex =
    PTHREAD_MUTEX_INITIALIZER;      // real time process utility
  EthercatSlave** slave;            // master utility
  uint8_t domainElementsNumber = 0; // master utility
  int numberOfSlaves;               // master utility

  void Start(); // the only thing you need to callnin the main
  virtual void StartUpFunction() = 0; // called before the cycle begins
  virtual void LoopFunction() = 0;    // called every cycle
};

#endif // ETHERCATMASTER_H
