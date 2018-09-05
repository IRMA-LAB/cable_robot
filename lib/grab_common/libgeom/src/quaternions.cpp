/**
 * @file quaternions.cpp
 * @author Edoardo Idà, Simone Comari
 * @date 03 Sep 2018
 * @brief File containing definitions of quaternions.h.
 */

#include "quaternions.h"

namespace grabgeom
{

grabnum::Matrix3d Quat2Rot(const Quaternion& q)
{
  grabnum::Matrix3d rot;
  rot(1, 1) = q(1) * q(1) + q(2) * q(2) - q(3) * q(3) - q(4) * q(4);
  rot(1, 2) = 2 * (q(2) * q(3) - q(1) * q(4));
  rot(1, 3) = 2 * (q(2) * q(4) + q(1) * q(3));
  rot(2, 1) = 2 * (q(2) * q(3) + q(1) * q(4));
  rot(2, 2) = q(1) * q(1) - q(2) * q(2) + q(3) * q(3) - q(4) * q(4);
  rot(2, 3) = 2 * (q(3) * q(4) - q(1) * q(2));
  rot(3, 1) = 2 * (q(2) * q(4) - q(1) * q(3));
  rot(3, 2) = 2 * (q(3) * q(4) + q(1) * q(2));
  rot(3, 3) = q(1) * q(1) - q(2) * q(2) - q(3) * q(3) + q(4) * q(4);
  return rot;
}

grabnum::VectorXd<4> Rot2Quat(const grabnum::Matrix3d& rot_mat)
{
  Quaternion q_square;
  Quaternion q;
  q_square(1) = 0.25 * (1 + rot_mat(1, 1) + rot_mat(2, 2) + rot_mat(3, 3));
  q_square(2) = 0.25 * (1 + rot_mat(1, 1) - rot_mat(2, 2) - rot_mat(3, 3));
  q_square(3) = 0.25 * (1 - rot_mat(1, 1) + rot_mat(2, 2) - rot_mat(3, 3));
  q_square(4) = 0.25 * (1 - rot_mat(1, 1) - rot_mat(2, 2) + rot_mat(3, 3));
  switch (q_square.MaxIdx())
  {
  case 1:
    q(1) = sqrt(q_square(1));
    q(2) = 0.25 * (rot_mat(3, 2) - rot_mat(2, 3)) / q(1);
    q(3) = 0.25 * (rot_mat(1, 3) - rot_mat(3, 1)) / q(1);
    q(4) = 0.25 * (rot_mat(2, 1) - rot_mat(1, 2)) / q(1);
    break;
  case 2:
    q(2) = sqrt(q_square(2));
    q(1) = 0.25 * (rot_mat(3, 2) - rot_mat(2, 3)) / q(2);
    q(3) = 0.25 * (rot_mat(1, 3) - rot_mat(3, 1)) / q(1);
    q(4) = 0.25 * (rot_mat(2, 1) - rot_mat(1, 2)) / q(1);
    break;
  case 3:
    q(3) = sqrt(q_square(3));
    q(1) = 0.25 * (rot_mat(1, 3) - rot_mat(3, 1)) / q(3);
    q(2) = 0.25 * (rot_mat(3, 2) - rot_mat(2, 3)) / q(1);
    q(4) = 0.25 * (rot_mat(2, 1) - rot_mat(1, 2)) / q(1);
    break;
  case 4:
    q(4) = sqrt(q_square(4));
    q(1) = 0.25 * (rot_mat(2, 1) - rot_mat(1, 2)) / q(4);
    q(2) = 0.25 * (rot_mat(3, 2) - rot_mat(2, 3)) / q(1);
    q(3) = 0.25 * (rot_mat(1, 3) - rot_mat(3, 1)) / q(1);
    break;
  }
  return q;
}

grabnum::MatrixXd<3,4> HtfQuat(const Quaternion& quat)
{
  grabnum::MatrixXd<3,4> hmat;
  hmat.SetCol(1, quat.GetBlock<3,1>(2,1));
  hmat.SetBlock(1, 2, grabnum::Anti(quat.GetBlock<3,1>(2,1)) + grabnum::Matrix3d(quat(1)));
  return 2. * hmat;
}

} // end namespace grabgeom
