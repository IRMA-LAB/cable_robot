#include <QString>
#include <QtTest>

/**
 * @brief The LibcdprTest class
 */
class LibcdprTest : public QObject
{
  Q_OBJECT

private Q_SLOTS:
  /**
   * @brief testCase1
   */
  void testCase1();
};

void LibcdprTest::testCase1()
{
  QVERIFY2(true, "Failure");
}

QTEST_APPLESS_MAIN(LibcdprTest)

#include "libcdpr_test.moc"
