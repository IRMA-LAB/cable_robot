#include <QtTest/QtTest>
#include "matrix.h"

class TestMatrix: public QObject
{
    Q_OBJECT
private slots:
    void Constructors();
};

void TestMatrix::Constructors()
{
  grabnum::Matrix<int, 3, 2> Mat32i;
  std::cout << Mat32i << std::endl;
  grabnum::Matrix<double, 3, 3> Mat34d(4);
  std::cout << Mat34d << std::endl;
  float vec[4] = {3.45f, 3.1f, 6.7f, 54.3f};
  grabnum::Matrix<float, 2, 2> Mat35f(vec, ARRAY_SIZE(vec));
  std::cout << Mat35f << std::endl;
  std::vector<long> vec2 = {5463454, 543542222};
  grabnum::Matrix<long, 2, 1> Mat25l(&vec2.front(), vec2.size());
  std::cout << Mat25l << std::endl;
  grabnum::Matrix<float, 2, 1> MatA = Mat25l;
  std::cout << MatA << std::endl;
  Mat25l.SetZero();
  std::cout << Mat25l << std::endl;
  std::cout << MatA << std::endl;

}

QTEST_MAIN(TestMatrix)
#include "matrix_test.moc"
