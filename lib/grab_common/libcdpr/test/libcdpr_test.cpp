#include <QString>
#include <QtTest>

class LibcdprTest : public QObject
{
  Q_OBJECT

public:
  LibcdprTest();

private Q_SLOTS:
  void testCase1();
};

LibcdprTest::LibcdprTest()
{
}

void LibcdprTest::testCase1()
{
  QVERIFY2(true, "Failure");
}

QTEST_APPLESS_MAIN(LibcdprTest)

#include "libcdpr_test.moc"
