/**
 * @file quaternions.h
 * @author Edoardo Idà, Simone Comari
 * @date 06 Sep 2018
 * @brief File containing quaternions utilities to be included in the GRAB geometric
 * library.
 * @todo documentation of external operators.
 */

#ifndef GRABCOMMON_LIBGEOM_QUATERNIONS_H
#define GRABCOMMON_LIBGEOM_QUATERNIONS_H

#include "matrix.h"

namespace grabgeom
{

/**
*@brief A minimal implementation of a quaternion structure.
*
* In this implementation we represent a quaternion as a 4-vector
* @f$\mathbf{q} \in \mathbb{R}^4@f$, defined as
* @f[
* \mathbf{q} := ( q_w, \mathbf{q}_v^T)^T = (q_w, q_x, q_y, q_z)^T
* @f]
* For the sake of simpicity, we use here the subscripts directly to refer to its components:
* @f[
* \mathbf{q} := ( w, \mathbf{v}^T)^T = (w, x, y, z)^T
* @f]
*/
struct Quaternion
{
  double w; /**< _w_ component */
  double x; /**< _x_ component  */
  double y; /**< _y_ component */
  double z; /**< _z_ component  */

  /**
   * @brief Default constructor. Initializes all components to 0.
   */
  Quaternion() { Quaternion(0., 0., 0., 0.); }
  /**
   * @brief Constructor to initialize all components individually.
   * @param[in] _w @a w component.
   * @param[in] _x @a x component.
   * @param[in] _y @a y component.
   * @param[in] _z @a z component.
   */
  Quaternion(const double _w, const double _x, const double _y, const double _z)
  {
    w = _w;
    x = _x;
    y = _y;
    z = _z;
  }
  /**
   * @brief Constructor to initialize all components from a 4-vector.
   * @param[in] vect A 4-vector @f$(w, x, y, z)^T@f$.
   */
  Quaternion(const grabnum::VectorXd<4>& vect)
  {
    w = vect(1);
    x = vect(2);
    y = vect(3);
    z = vect(4);
  }
  /**
   * @brief Constructor to initialize real and imaginary parts separately.
   * @param[in] _w @a w component, i.e. the real part.
   * @param[in] _v A 3-vector @f$(x, y, z)^T@f$, i.e. the imaginary parts.
   */
  Quaternion(const double _w, const grabnum::Vector3d& _v)
  {
    w = _w;
    x = _v(1);
    y = _v(2);
    z = _v(3);
  }

  /**
   * @brief operator ==
   * @param[in] other The quaternion to be compared against.
   * @return _True_ if quaternions are exactly the same.
   * @note This methos is not safe due to numerical imprecition of floating points.
   * @see IsApprox()
   */
  bool operator==(const Quaternion& other) const { return v() == other.v(); }

  /**
   * @brief operator !=
   * @param other The quaternion to be compared against.
   * @return _True_ if quaternions are not exactly the same.
   * @note This methos is not safe due to numerical imprecition of floating points.
   * @see IsApprox()
   */
  bool operator!=(const Quaternion& other) const { return v() != other.v(); }

  /**
   * @brief Replaces @c *this by @c *this + the _other_ quaternion.
   *
   * Quaternions addition is simply an element-wise addition:
   * @f[
   * \mathbf{p} + \mathbf{q} = \begin{bmatrix} p_w \\ \mathbf{p}_v \end{bmatrix} +
   *    \begin{bmatrix} q_w \\ \mathbf{q}_v \end{bmatrix} =
   *    \begin{bmatrix} p_w + q_ w \\ \mathbf{p}_v + \mathbf{q}_v\end{bmatrix}
   * @f]
   * @param[in] other The quaternion to be added.
   * @return A reference to @c *this.
   */
  Quaternion& operator+=(const Quaternion& other)
  {
    w += other.w;
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }

  /**
   * @brief Replaces @c *this by @c *this + the scalar value _scalar_.
   * @param[in] scalar The scalar value to be added to each element of the quaternion.
   * @return A reference to @c *this.
   */
  Quaternion& operator+=(const double scalar)
  {
    w += scalar;
    x += scalar;
    y += scalar;
    z += scalar;
    return *this;
  }

  /**
   * @brief Replaces @c *this by @c *this - the _other_ quaternion.
   *
   * Quaternions subtraction is simply an element-wise subtraction:
   * @f[
   * \mathbf{p} - \mathbf{q} = \begin{bmatrix} p_w \\ \mathbf{p}_v \end{bmatrix} -
   *    \begin{bmatrix} q_w \\ \mathbf{q}_v \end{bmatrix} =
   *    \begin{bmatrix} p_w - q_ w \\ \mathbf{p}_v - \mathbf{q}_v\end{bmatrix}
   * @f]
   * @param[in] other The quaternion to be subtracted.
   * @return A reference to @c *this.
   */
  Quaternion& operator-=(const Quaternion& other)
  {
    w -= other.w;
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
  }

  /**
   * @brief Replaces @c *this by @c *this - the scalar value _scalar_.
   * @param[in] scalar The scalar value to be subtracted to each element of the quaternion.
   * @return A reference to @c *this.
   */
  Quaternion& operator-=(const double scalar)
  {
    w -= scalar;
    x -= scalar;
    y -= scalar;
    z -= scalar;
    return *this;
  }

  /**
   * @brief Replaces @c *this by the quaternion product of @c *this and the _other_ quaternion.
   *
   * Quaternions product is defined as:
   * @f[
   * \mathbf{p} \otimes \mathbf{q} = \begin{bmatrix}
   *    p_w q_ w - \mathbf{p}_v \cdot \mathbf{q}_v \\
   *    p_w \mathbf{q}_v + q_w\mathbf{p}_v + \mathbf{p}_v \times \mathbf{q}_v
   *    \end{bmatrix}
   * @f]
   * @param[in] other The quaternion to be multiplied by.
   * @return A reference to @c *this.
   * @note The quaternion product **is not** _commutative_, except when one quaternion is
   * real, or when both imaginary parts are parallel. It is, however, _associative_ and
   * _distributive over the sum_.
   */
  Quaternion& operator*=(const Quaternion& other)
  {
    grabnum::Vector3d old_v = v();
    grabnum::Vector3d other_v = other.v();
    grabnum::Vector3d new_v =
      w * other_v + other.w * old_v + grabnum::Cross(old_v, other_v);
    w = w + other.w - grabnum::Dot(old_v, other_v);
    x = new_v(1);
    y = new_v(2);
    z = new_v(3);
    return *this;
  }

  /**
   * @brief Replaces @c *this by @c *this times the scalar value _scalar_.
   * @param[in] scalar The scalar value to be multiplied by to each element of the quaternion.
   * @return A reference to @c *this.
   */
  Quaternion& operator*=(const double scalar)
  {
    w *= scalar;
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
  }

  /**
   * @brief Replaces @c *this by @c *this divided by the scalar value _scalar_.
   * @param[in] scalar The scalar value to be divided to each element of the quaternion.
   * @return A reference to @c *this.
   */
  Quaternion& operator/=(const double scalar)
  {
    w /= scalar;
    x /= scalar;
    y /= scalar;
    z /= scalar;
    return *this;
  }

  /**
   * @brief Returns @c *this divided by the scalar value _scalar_.
   * @param[in] scalar The scalar value to be divided to each element of the quaternion.
   * @return The resulting quaternion.
   */
  Quaternion operator/(const double scalar) const
  {
    Quaternion res;
    res.w = w / scalar;
    res.x = x / scalar;
    res.y = y / scalar;
    res.z = z / scalar;
    return res;
  }

  /**
   * @brief Returns the imaginary parts vector component @f$\mathbf{q}_v@f$.
   * @return The 3D vector @f$\mathbf{q}_v@f$.
   */
  grabnum::Vector3d v() const
  {
    grabnum::Vector3d v;
    v(1) = x;
    v(2) = y;
    v(3) = z;
    return v;
  }

  /**
   * @brief Returns the whole quaternion in vector format.
   * @return The 4D vector @f$\mathbf{q}@f$.
   */
  grabnum::VectorXd<4> q() const
  {
    grabnum::VectorXd<4> q;
    q(1) = w;
    q(2) = x;
    q(3) = y;
    q(4) = z;
    return q;
  }

  /**
   * @brief Returns the identity element of quaternions
   * @f$\mathbf{q}_1 = (1, \mathbf{0}_v)^T@f$.
   * @return The identity quaternion @f$\mathbf{q}_1@f$.
   */
  Quaternion Identity() const { return Quaternion(1., 0., 0., 0.); }

  /**
   * @brief Returns the _conjugate_ of @c *this.
   *
   * By definition, the conjugate of a quaternion @f$\mathbf{q}@f$ is:
   * @f[
   * \mathbf{q}^* := \begin{bmatrix} q_ w \\ -\mathbf{q}_v \end{bmatrix}
   * @f]
   * @return The conjugate quaternion @f$\mathbf{q}^*@f$.
   */
  Quaternion Conj() const { return Quaternion(w, -x, -y, -z); }

  /**
   * @brief Returns the _inverse_ of @c *this.
   *
   * By definition, the inverse of a quaternion @f$\mathbf{q}@f$ is such that:
   * @f[
   * \mathbf{q} \otimes \mathbf{q}^{-1} = \mathbf{q}^{-1} \otimes \mathbf{q} =
   *    \mathbf{q}_1
   * @f]
   * and therefore it can be computed with:
   * @f[
   * \mathbf{q}^{-1} = \mathbf{q}^* / \|\mathbf{q}\|^2
   * @f]
   * @return The inverse quaternion @f$\mathbf{q}^{-1}@f$.
   */
  Quaternion Inv() const { return Conj() / Norm(); }

  /**
   * @brief Calculate the norm of @c *this.
   *
   * The norm of a quaternion is the standard L2 norm of the 4-vector @f$\mathbf{q}@f$.
   * @return The norm of @a *this.
   * @see Normalized() Normalize()
   */
  double Norm() const { return grabnum::Norm(q()); }

  /**
   * @brief Normalize @a *this.
   *
   * Normalizing a quaternion is done by diving all its elements by its norm. The result is a
   * unitary quaternion.
   * @return A reference to @c *this.
   * @see Normalized() Norm()
   * @note This function replaces @c *this with its normalized version. If you do not want to
   * change the original quaternion, please use Normalized() instead.
   */
  Quaternion& Normalize()
  {
    *this /= Norm();
    return *this;
  }

  /**
   * @brief Returns the normalized version of @a *this.
   *
   * Normalizing a quaternion is done by diving all its elements by its norm. The result is a
   * unitary quaternion.
   * @return The normalized quaternion.
   * @see Normalize() Norm()
   */
  Quaternion Normalized() const { return *this / Norm(); }

  /**
   * @brief Check whether @a *this is a unitary quaternion.
   * @return _True_ if it is unitary.
   */
  bool IsUnitary() const { return grabnum::IsClose(Norm(), 1.); }

  /**
   * @brief Check whether 2 quaternions are approximately equal, within a certain threshold.
   * @param other The other quaternion to be compared against.
   * @param[in] tol (Optional) The tolerance for element-wise comparison for being equal.
   * @return _True_ if they are approximately the same.
   */
  bool IsApprox(const Quaternion& other, const double tol = grabnum::EPSILON)
  { return v().IsApprox(other.v(), tol); }
};

/**
 * @brief operator +
 * @param lhs
 * @param rhs
 * @return
 */
Quaternion operator+(Quaternion lhs, const Quaternion& rhs) { return lhs += rhs; }

/**
 * @brief operator +
 * @param lhs
 * @param rhs
 * @return
 */
Quaternion operator+(Quaternion lhs, const double rhs) { return lhs += rhs; }

/**
 * @brief operator +
 * @param lhs
 * @param rhs
 * @return
 */
Quaternion operator+(const double lhs, Quaternion rhs) { return rhs += lhs; }

/**
 * @brief operator -
 * @param lhs
 * @param rhs
 * @return
 */
Quaternion operator-(Quaternion lhs, const Quaternion& rhs) { return lhs -= rhs; }

/**
 * @brief operator -
 * @param lhs
 * @param rhs
 * @return
 */
Quaternion operator-(Quaternion lhs, const double rhs) { return lhs -= rhs; }

/**
 * @brief operator -
 * @param lhs
 * @param rhs
 * @return
 */
Quaternion operator-(const double lhs, Quaternion rhs) { return rhs -= lhs; }

/**
 * @brief operator *
 * @param lhs
 * @param rhs
 * @return
 */
Quaternion operator*(Quaternion lhs, const Quaternion& rhs) { return lhs *= rhs; }

/**
 * @brief operator *
 * @param lhs
 * @param rhs
 * @return
 */
Quaternion operator*(Quaternion lhs, const double rhs) { return lhs *= rhs; }

/**
 * @brief operator *
 * @param lhs
 * @param rhs
 * @return
 */
Quaternion operator*(const double lhs, Quaternion rhs) { return rhs *= lhs; }

/**
 * @brief Returns the _conjugate_ of a quaternion.
 *
 * By definition, the conjugate of a quaternion @f$\mathbf{q}@f$ is:
 * @f[
 * \mathbf{q}^* := \begin{bmatrix} q_ w \\ -\mathbf{q}_v \end{bmatrix}
 * @f]
 * @param quat The original quaternion.
 * @return The conjugate quaternion @f$\mathbf{q}^*@f$.
 */
Quaternion QuatConjugate(const Quaternion& quat) { return quat.Conj(); }

/**
 * @brief Returns the _inverse_ of a quaternion.
 *
 * By definition, the inverse of a quaternion @f$\mathbf{q}@f$ is such that:
 * @f[
 * \mathbf{q} \otimes \mathbf{q}^{-1} = \mathbf{q}^{-1} \otimes \mathbf{q} =
 *    \mathbf{q}_1
 * @f]
 * and therefore it can be computed with:
 * @f[
 * \mathbf{q}^{-1} = \mathbf{q}^* / \|\mathbf{q}\|^2
 * @f]
 * @param quat The original quaternion.
 * @return The inverse quaternion @f$\mathbf{q}^{-1}@f$.
 */
Quaternion QuatInverse(const Quaternion& quat) { return quat.Inv(); }

/**
 * @brief Calculate the norm of a quaternion.
 *
 * The norm of a quaternion is the standard L2 norm of the 4-vector @f$\mathbf{q}@f$.
 * @param quat A quaternion.
 * @return The norm of @a quat.
 * @see QuatNormalized()
 */
double QuatNorm(const Quaternion& quat) { return quat.Norm(); }

/**
 * @brief Returns the normalized version of a quaternion.
 *
 * Normalizing a quaternion is done by diving all its elements by its norm. The result is a
 * unitary quaternion.
 * @return The normalized quaternion.
 * @see QuatNorm()
 */
Quaternion QuatNormalized(const Quaternion& quat) { return quat.Normalized(); }

/**
 * @brief Determines the rotation matrix corresponding to a given quaternion.
 *
 * @param[in] quaternion The orientation expressed by a unitary quaternion
 * @f$(q_w, q_x, q_y, q_z)@f$.
 * @return A 3x3 orthogonal matrix (double).
 */
grabnum::Matrix3d Quat2Rot(const Quaternion& quaternion);

/**
 * @brief Determines the quaternion corresponding to a given rotation matrix.
 * @param[in] rot_mat A rotation matrix.
 * @return A unitary quaternion @f$(q_w, q_x, q_y, q_z)@f$.
 */
Quaternion Rot2Quat(const grabnum::Matrix3d& rot_mat);

/**
 * @brief QuatRotate
 * @param quat
 * @param vect
 * @return
 */
grabnum::Vector3d QuatRotate(const Quaternion& quat, const grabnum::Vector3d& vect);

/**
 * @brief Transformation matrix @f$\mathbf{H}@f$ between the derivative of a quaternion
 * and angular velocity vector @f$\boldsymbol\omega@f$.
 *
 * @param[in] quaternion The orientation expressed by a unitary quaternion
 * @f$(q_w, q_x, q_y, q_z)@f$.
 * @return A 3x3 matrix (double).
 */
grabnum::MatrixXd<3, 4> HtfQuat(const Quaternion& quaternion);

/**
 * @brief Time-derivative of transformation matrix @f$\mathbf{H}@f$ between the
 * derivative of a quaternion and angular velocity vector @f$\boldsymbol\omega@f$.
 *
 * @f[
 * \mathbf{\dot{H}}(\boldsymbol{\epsilon}_q, \boldsymbol{\dot{\epsilon}}_q) =
 * \frac{d\mathbf{H}(\boldsymbol{\epsilon}_q)}{dt}
 * @f]
 *
 * @param[in] quaternion_dot The orientation speed expressed by a quaternion
 * @f$\dot{\boldsymbol{\epsilon}}_q@f$.
 * @return A 3x3 matrix (double).
 */
grabnum::MatrixXd<3, 4> DHtfQuat(const Quaternion& quaternion_dot)
{
  return HtfQuat(quaternion_dot);
}

} // end namespace grabgeom

#endif // GRABCOMMON_LIBGEOM_QUATERNIONS_H
