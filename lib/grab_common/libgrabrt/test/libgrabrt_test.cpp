/**
 * @file libgrabrt_test.cpp
 * @author Simone Comari
 * @date 14 Sep 2018
 * @brief ...
 */

#include <QString>
#include <QtTest>

#include "common.h"
#include "threads.h"
#include "clocks.h"

/**
 * @brief The LibgrabrtTest class
 */
class LibgrabrtTest : public QObject
{
  Q_OBJECT

private Q_SLOTS:
  /**
   * @brief testCPUSetBuilders
   */
  void testCPUSetBuilders();
  /**
   * @brief testSetThisThread
   */
  void testSetThisThread();
  /**
   * @brief testThreadClock
   */
  void testThreadClock();
  /**
   * @brief testNewThread
   */
  void testNewThread();

private:
  /**
   * @brief loopFun
   * @param obj
   */
  static void loopFun(void* obj)
  {
    static uint counter = 1;
    char msg[50];
    sprintf(msg, "New thread %s scheduler attributes:",
            static_cast<grabrt::Thread*>(obj)->GetNameCstr());
    grabrt::displayThreadSchedAttr(msg);
    grabrt::displayThreadAffinitySet();
    printf("counter:\n\t%d\n\n", counter++);
  }
};

void LibgrabrtTest::testCPUSetBuilders()
{
  std::cout << "CPU no. of cores: " << CPU_CORES_NUM << std::endl;

  cpu_set_t cpu_set = grabrt::BuildCPUSet();
  QCOMPARE(CPU_COUNT(&cpu_set), static_cast<int>(CPU_CORES_NUM));

  cpu_set = grabrt::BuildCPUSet(-1);
  QCOMPARE(CPU_COUNT(&cpu_set), 1);
  QVERIFY(CPU_ISSET(CPU_CORES_NUM - 1, &cpu_set));

  const size_t c = 3;
  cpu_set = grabrt::BuildCPUSet(c);
  QCOMPARE(CPU_COUNT(&cpu_set), 1);
  QVERIFY(CPU_ISSET(c, &cpu_set));

  const std::vector<size_t> cores = {2, 1, 1, 2};
  cpu_set = grabrt::BuildCPUSet(cores);
  QCOMPARE(CPU_COUNT(&cpu_set), 2);
  for (auto const& val : cores)
    QVERIFY(CPU_ISSET(val, &cpu_set));

  const std::vector<size_t> cores2 = {2, 1, 3, 0};
  cpu_set = grabrt::BuildCPUSet(cores2);
  QCOMPARE(CPU_COUNT(&cpu_set), static_cast<int>(cores2.size()));
  for (auto const& val : cores2)
    QVERIFY(CPU_ISSET(val, &cpu_set));
}

void LibgrabrtTest::testSetThisThread()
{
  grabrt::displayThreadSchedAttr(
    std::string("Original scheduler settings of this thread"));
  grabrt::displayThreadAffinitySet();
  grabrt::SetThreadCPUs(grabrt::BuildCPUSet(2));
  grabrt::SetThreadPolicy(SCHED_RR, 25);
  grabrt::displayThreadSchedAttr(
    std::string("\nModified scheduler settings of this thread (RT)"));
  grabrt::displayThreadAffinitySet();
  grabrt::SetThreadCPUs(grabrt::BuildCPUSet(3));
  grabrt::SetThreadPolicy(SCHED_OTHER, 4);
  grabrt::displayThreadSchedAttr(
    std::string("\nModified scheduler settings of this thread (Non-RT)"));
  grabrt::displayThreadAffinitySet();
}

void LibgrabrtTest::testThreadClock()
{
  const double period = 0.145;
  grabrt::ThreadClock clock(grabrt::Sec2NanoSec(period));
  // test waiting time
  struct timespec ts_start, ts_end;
  double t_start, t_end;
  for (uint i = 0; i < 20; ++i)
  {
    timespec_get(&ts_start, TIME_UTC);
    clock.Reset();
    clock.WaitUntilNext();
    timespec_get(&ts_end, TIME_UTC);
    t_start = ts_start.tv_sec + grabrt::NanoSec2Sec(ts_start.tv_nsec);
    t_end = ts_end.tv_sec + grabrt::NanoSec2Sec(ts_end.tv_nsec);
    QVERIFY(grabnum::IsClose(t_end - t_start, period, 0.001));
  }
  // test displaying functions
  clock.DispCurrentTime();
  clock.DispNextTime();
  // test getters
  ts_start = clock.GetCurrentTime();
  ts_end = clock.GetNextTime();
  t_start = ts_start.tv_sec + grabrt::NanoSec2Sec(ts_start.tv_nsec);
  t_end = ts_end.tv_sec + grabrt::NanoSec2Sec(ts_end.tv_nsec);
  QVERIFY(grabnum::IsClose(t_end - t_start, period));
}

void LibgrabrtTest::testNewThread()
{
  grabrt::Thread t("TestSubThread");
  t.SetCPUs(std::vector<size_t>{2, 3});
  t.SetPolicy(SCHED_RR, 25);
  t.DispAttr();
  QCOMPARE(t.GetPID(), 0UL);
  QCOMPARE(t.GetTID(), -1L);
  t.SetLoopFunc(&loopFun, &t);
  QCOMPARE(t.GetReady(grabrt::Sec2NanoSec(0.2)), 0);
  THREAD_RUN(t)
  QVERIFY(t.IsRunning());
  sleep(1);
  t.SetCPUs(1);
  t.SetPolicy(SCHED_FIFO, 15);
  sleep(1);
  t.Pause();
  QVERIFY(!t.IsRunning());
  sleep(1);
  t.Unpause();
  QVERIFY(t.IsRunning());
  sleep(1);
  printf("%s IDs:\n\tPID = %lu\n\tTID = %ld\n", t.GetNameCstr(), t.GetPID(), t.GetTID());
  t.Stop();
  QVERIFY(!t.IsActive());
}

QTEST_APPLESS_MAIN(LibgrabrtTest)

#include "libgrabrt_test.moc"
