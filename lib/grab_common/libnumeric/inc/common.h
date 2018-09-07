/**
 * @file common.h
 * @author Simone Comari
 * @date 07 Sep 2018
 * @brief File containing common basic utilities to be included in the GRAB numeric
 * library.
 */
#ifndef GRABCOMMON_LIBNUMERIC_COMMON_H
#define GRABCOMMON_LIBNUMERIC_COMMON_H

namespace grabnum
{

static constexpr double EPSILON = 1e-7; /**< tolerance for floating point comparison */

/**
 * @brief Check whether two floating numbers are close within a certain tolerance.
 *
 * This function is safer when comparing two floating numbers than using the standard `==`
 * operator, which could deliver misleading result due to numerical errors.
 * @param[in] a First floating point value to be compared.
 * @param[in] b Second floating point value to be compared.
 * @param[in] tol (Optional) Maximum tolerance on the absolute difference in order to
 * consider the two values close (aka "equal").
 * return _True_ if their absolute difference is within the threshold.
 * @see EPSILON
 */
template <typename T>
inline bool IsClose(const T a, const T b, const double tol = EPSILON)
{
  return fabs(a - b) <= tol;
}

} //  end namespace grabnum
#endif // GRABCOMMON_LIBNUMERIC_COMMON_H
