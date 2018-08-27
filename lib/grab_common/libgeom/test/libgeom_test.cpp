#include <QString>
#include <QtTest>

#include "rotations.h"

/**
 * @brief The LibgeomTest class
 */
class LibgeomTest : public QObject
{
  Q_OBJECT

private Q_SLOTS:
  /**
   * @brief testCase1
   */
  void testCase1();
};

void LibgeomTest::testCase1()
{
  QVERIFY2(true, "Failure");
}

QTEST_APPLESS_MAIN(LibgeomTest)

#include "libgeom_test.moc"
