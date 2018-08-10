#include <QtTest/QtTest>
#include <cmath>

#include "matrix.h"

class TestMatrix : public QObject
{
  Q_OBJECT

private:
  static constexpr uint8_t kDim1_ = 2;
  static constexpr uint8_t kDim2_ = 3;
  const double DValues_[6] = {0.1, 1.1, 2.1, 3.1, 4.1, 5.1};
  const int IValues_[6] = {0, 1, 2, 3, 4, 5};

  grabnum::Matrix<double, kDim1_, kDim1_> mat22d_;
  grabnum::Matrix<int, kDim1_, kDim1_> mat22i_;
  grabnum::Matrix<double, kDim1_, kDim2_> mat23d_;
  grabnum::Matrix<int, kDim1_, kDim2_> mat23i_;
  grabnum::Matrix<double, kDim1_, kDim1_> identity2d_;
  grabnum::Matrix<int, kDim1_, kDim1_> identity2i_;
  grabnum::Matrix<double, kDim1_, kDim1_> zeros22d_;
  grabnum::Matrix<int, kDim1_, kDim1_> zeros22i_;

private slots:
  /**
   * Called before each test function is executed (overloaded).
   */
  void init();
  /**
   * Test @a Matrix constructors.
   */
  void Constructors();
  /**
   * Test @a Matrix operators overloadings.
   * @todo TBF
   */
  void ClassOperators();
  /**
   * Test @a Matrix setters.
   * @todo TBD
   */
  void Setters();
  /**
   * Test @a Matrix getters.
   * @todo TBD
   */
  void Getters();
  /**
   * Test @a Matrix manipulating functions.
   * @todo TBD
   */
  void Manipulations();
};

void TestMatrix::init()
{
  // Reset data.
  mat22d_.Fill(DValues_, mat22d_.Size());
  mat22i_.Fill(IValues_, mat22i_.Size());
  mat23d_.Fill(DValues_, mat23d_.Size());
  mat23i_.Fill(IValues_, mat23i_.Size());
  identity2d_.SetIdentity();
  identity2i_.SetIdentity();
  zeros22d_.SetZero();
  zeros22i_.SetZero();
}

void TestMatrix::Constructors()
{
  const uint8_t m = 3;
  const uint8_t n = 2;

  // Default constructor.
  grabnum::Matrix<int, m, n> mat_mxn_i;
  for (uint8_t i = 1; i <= m; ++i)
    for (uint8_t j = 1; j <= n; ++j)
      QCOMPARE(mat_mxn_i(i, j), 0);

  // Identity constructor.
  double diag_value = 4.3;
  grabnum::Matrix<double, m, m> mat_mxm_d(diag_value);
  for (uint8_t i = 1; i <= m; ++i)
    for (uint8_t j = 1; j <= n; ++j)
      if (i == j)
        QCOMPARE(mat_mxm_d(i, j), diag_value);
      else
        QCOMPARE(mat_mxm_d(i, j), 0.);

  // Constructor from array.
  float vec[n * m] = {3.45f, 3.1f, 6.7f, 54.3f, 5.74f, 0.45f};
  grabnum::Matrix<float, n, m> mat_nxm_f(vec, ARRAY_SIZE(vec));
  for (uint8_t i = 1; i <= n; ++i)
    for (uint8_t j = 1; j <= m; ++j)
      QCOMPARE(mat_nxm_f(i, j), vec[(i - 1) * m + (j - 1)]);

  // Constructor from standard array.
  std::vector<float> std_vec(vec, vec + sizeof(vec) / sizeof(float));
  grabnum::Matrix<float, n, m> mat_nxm_f2(&std_vec.front(), std_vec.size());
  for (uint8_t i = 1; i <= n; ++i)
    for (uint8_t j = 1; j <= m; ++j)
      QCOMPARE(mat_nxm_f2(i, j), std_vec[(i - 1) * m + (j - 1)]);

  // Constructor from copy.
  grabnum::Matrix<float, n, m> mat_nxm_f3 = mat_nxm_f2;
  QVERIFY(mat_nxm_f2 == mat_nxm_f3);

  // Assignment with explicit casting.
  grabnum::Matrix<double, n, m> mat_nxm_d;
  mat_nxm_d = static_cast<grabnum::Matrix<double, n, m>>(mat_nxm_f2);
  QVERIFY(mat_nxm_d == mat_nxm_f2);

  // Constructor from copy with implicit casting.
  grabnum::Matrix<int, n, m> mat_nxm_i(mat_nxm_f2);
  QVERIFY(mat_nxm_i != mat_nxm_f2);
  QVERIFY(mat_nxm_i.Type() != mat_nxm_f2.Type());

  // Constructor from direct assignment with implicit casting.
  grabnum::Matrix<long, n, m> mat_nxm_i2 = mat_nxm_f2;
  QVERIFY(mat_nxm_i2 != mat_nxm_f2);
  QVERIFY(mat_nxm_i2.Type() != mat_nxm_f2.Type());
}

void TestMatrix::ClassOperators()
{
  double scalar = 0.1;
  double values[ARRAY_SIZE(DValues_)];
  for (size_t i = 0; i < ARRAY_SIZE(DValues_); ++i)
    values[i] = DValues_[i] + scalar;
  grabnum::Matrix<double, kDim1_, kDim2_> other(values, mat23d_.Size());
  grabnum::Matrix<double, kDim1_, kDim2_> mat23d_copy(mat23d_);
  grabnum::Matrix<double, kDim1_, kDim2_> other_copy(other);

  // Test equal operator
  bool res = true;
  for (uint8_t i = 0; i < kDim1_; ++i)
  {
    if (!res)
      break;
    for (uint8_t j = 0; j < kDim2_; ++j)
      if (std::abs(DValues_[i * kDim2_ + j] - mat23d_copy(i + 1, j + 1)) >
          std::numeric_limits<double>::epsilon())
      {
        res = false;
        break;
      }
  }
  QVERIFY(res == (mat23d_.IsApprox(mat23d_copy)));

  // Test not equal operator
  res = true;
  for (uint8_t i = 1; i <= kDim1_; ++i)
  {
    if (!res)
      break;
    for (uint8_t j = 1; j <= kDim2_; ++j)
      if (std::abs(mat23d_(i, j) - other(i, j)) <= std::numeric_limits<double>::epsilon())
      {
        res = false;
        break;
      }
  }
  QVERIFY(res == (!mat23d_.IsApprox(other)));

  // Test difference operator with scalar
  other -= scalar;
  QVERIFY(other.IsApprox(mat23d_));

  // Test addition operator with scalar
  other += scalar;
  QVERIFY(other.IsApprox(other_copy));

  // Test scalar multiplication
  for (size_t i = 0; i < ARRAY_SIZE(DValues_); ++i)
    values[i] = DValues_[i] * scalar;
  grabnum::Matrix<double, kDim1_, kDim2_> mat23d_scaled(values, mat23d_.Size());
  mat23d_copy *= scalar;
  QVERIFY(mat23d_copy.IsApprox(mat23d_scaled));

  // Test division by scalar
  mat23d_copy /= scalar;
  QVERIFY(mat23d_copy.IsApprox(mat23d_));
}

void TestMatrix::Setters() {}

void TestMatrix::Getters() {}

void TestMatrix::Manipulations() {}

QTEST_MAIN(TestMatrix)
#include "matrix_test.moc"
