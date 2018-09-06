/**
 * @file quaternions.cpp
 * @author Edoardo Idà, Simone Comari
 * @date 06 Sep 2018
 * @brief File containing definitions of quaternions.h.
 */

#include "quaternions.h"

namespace grabgeom
{

grabnum::Matrix3d Quat2Rot(const Quaternion& q)
{
  assert(q.IsUnitary());

  grabnum::Matrix3d rot;
  rot(1, 1) = q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z;
  rot(1, 2) = 2 * (q.x * q.y - q.w * q.z);
  rot(1, 3) = 2 * (q.x * q.z + q.w * q.y);
  rot(2, 1) = 2 * (q.x * q.y + q.w * q.z);
  rot(2, 2) = q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z;
  rot(2, 3) = 2 * (q.y * q.z - q.w * q.x);
  rot(3, 1) = 2 * (q.x * q.z - q.w * q.y);
  rot(3, 2) = 2 * (q.y * q.z + q.w * q.x);
  rot(3, 3) = q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z;
  return rot;
}

Quaternion Rot2Quat(const grabnum::Matrix3d& rot_mat)
{
  Quaternion q_square;
  Quaternion q;
  q_square.w = 0.25 * (1 + rot_mat(1, 1) + rot_mat(2, 2) + rot_mat(3, 3));
  q_square.x = 0.25 * (1 + rot_mat(1, 1) - rot_mat(2, 2) - rot_mat(3, 3));
  q_square.y = 0.25 * (1 - rot_mat(1, 1) + rot_mat(2, 2) - rot_mat(3, 3));
  q_square.z = 0.25 * (1 - rot_mat(1, 1) - rot_mat(2, 2) + rot_mat(3, 3));
  switch (q_square.q().MaxIdx())
  {
  case 1:
    q.w = sqrt(q_square.w);
    q.x = 0.25 * (rot_mat(3, 2) - rot_mat(2, 3)) / q.w;
    q.y = 0.25 * (rot_mat(1, 3) - rot_mat(3, 1)) / q.w;
    q.z = 0.25 * (rot_mat(2, 1) - rot_mat(1, 2)) / q.w;
    break;
  case 2:
    q.x = sqrt(q_square.x);
    q.w = 0.25 * (rot_mat(3, 2) - rot_mat(2, 3)) / q.x;
    q.y = 0.25 * (rot_mat(1, 3) - rot_mat(3, 1)) / q.w;
    q.z = 0.25 * (rot_mat(2, 1) - rot_mat(1, 2)) / q.w;
    break;
  case 3:
    q.y = sqrt(q_square.y);
    q.w = 0.25 * (rot_mat(1, 3) - rot_mat(3, 1)) / q.y;
    q.x = 0.25 * (rot_mat(3, 2) - rot_mat(2, 3)) / q.w;
    q.z = 0.25 * (rot_mat(2, 1) - rot_mat(1, 2)) / q.w;
    break;
  case 4:
    q.z = sqrt(q_square.z);
    q.w = 0.25 * (rot_mat(2, 1) - rot_mat(1, 2)) / q.z;
    q.x = 0.25 * (rot_mat(3, 2) - rot_mat(2, 3)) / q.w;
    q.y = 0.25 * (rot_mat(1, 3) - rot_mat(3, 1)) / q.w;
    break;
  }
  return q;
}

grabnum::Vector3d QuatRotate(const Quaternion& quat, const grabnum::Vector3d& vect)
{
  assert(quat.IsUnitary());

  Quaternion vect_ext(0., vect);  // pure quaternion
  return (quat * vect_ext * quat.Conj()).v();
}

grabnum::MatrixXd<3, 4> HtfQuat(const Quaternion& quat)
{
  grabnum::MatrixXd<3, 4> hmat;
  hmat.SetCol(1, -quat.v());
  hmat.SetBlock(1, 2, grabnum::Anti(quat.v()) + grabnum::Matrix3d(quat.w));
  return 2. * hmat;
}

} // end namespace grabgeom
