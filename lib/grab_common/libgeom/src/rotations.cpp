/**
 * @file rotations.cpp
 * @author Edoardo Idà, Simone Comari
 * @date 27 Aug 2018
 * @brief File containing definitions of rotations.h.
 */

#include "rotations.h"

namespace grabgeom
{

grabnum::Matrix3d RotX(const double angle)
{
  grabnum::Matrix3d rot(1.0);
  rot(2, 2) = cos(angle);
  rot(3, 3) = cos(angle);
  rot(2, 3) = -sin(angle);
  rot(3, 2) = sin(angle);
  return rot;
}

grabnum::Matrix3d RotY(const double angle)
{
  grabnum::Matrix3d rot(1.0);
  rot(1, 1) = cos(angle);
  rot(3, 3) = cos(angle);
  rot(1, 3) = sin(angle);
  rot(3, 1) = -sin(angle);
  return rot;
}

grabnum::Matrix3d RotZ(const double angle)
{
  grabnum::Matrix3d rot(1.0);
  rot(1, 1) = cos(angle);
  rot(2, 2) = cos(angle);
  rot(1, 2) = -sin(angle);
  rot(2, 1) = sin(angle);
  return rot;
}

grabnum::Matrix3d RotXYZ(const double alpha, const double beta, const double gamma)
{
  grabnum::Matrix3d rot;
  rot = RotX(alpha) * RotY(beta) * RotZ(gamma);
  return rot;
}

grabnum::Matrix3d RotRPY(const double roll, const double pitch, const double yaw)
{
  grabnum::Matrix3d rot;
  rot = RotZ(yaw) * RotY(pitch) * RotX(roll);
  return rot;
}

grabnum::Matrix3d RotZYZ(const double alpha, const double beta, const double gamma)
{
  grabnum::Matrix3d rot;
  rot = RotZ(alpha) * RotY(beta) * RotZ(gamma);
  return rot;
}

grabnum::Matrix3d RotTiltTorsion(const double tilt_azimuth, const double tilt,
                                 const double torsion)
{
  return RotZYZ(tilt_azimuth, tilt, torsion - tilt_azimuth);
}

grabnum::Matrix3d HtfXYZ(const double alpha, const double beta)
{
  grabnum::Matrix3d hmat(1.0);
  hmat(1, 3) = sin(beta);
  hmat(2, 2) = cos(alpha);
  hmat(3, 2) = sin(alpha);
  hmat(2, 3) = -hmat(3, 2) * cos(beta);
  hmat(3, 3) = hmat(2, 2) * cos(beta);
  return hmat;
}

grabnum::Matrix3d HtfTiltTorsion(const double tilt_azimuth, const double tilt)
{
  grabnum::Matrix3d hmat;
  double c1 = cos(tilt_azimuth);
  double s1 = sin(tilt_azimuth);
  double c2 = cos(tilt);
  double s2 = sin(tilt);
  hmat(1, 1) = -c1 * s2;
  hmat(1, 2) = -s1;
  hmat(1, 3) = -hmat(1, 1);
  hmat(2, 1) = -s1 * s2;
  hmat(2, 2) = c1;
  hmat(2, 3) = -hmat(2, 1);
  hmat(3, 1) = 1.0 - c2;
  hmat(3, 2) = 0.0;
  hmat(3, 3) = c2;
  return hmat;
}

grabnum::Matrix3d HtfZYZ(const double alpha, const double beta)
{
  grabnum::Matrix3d hmat;
  hmat(1, 2) = -sin(alpha);
  hmat(2, 2) = cos(alpha);
  hmat(1, 3) = hmat(2, 2) * sin(beta);
  hmat(2, 3) = -hmat(1, 2) * sin(beta);
  hmat(3, 1) = 1.0;
  hmat(3, 3) = cos(beta);
  return hmat;
}

grabnum::Matrix3d DHtfXYZ(const double alpha, const double beta, const double alpha_dot,
                          const double beta_dot)
{
  grabnum::Matrix3d hmat_dot;
  hmat_dot(1, 3) = cos(beta) * beta_dot;
  hmat_dot(2, 2) = -sin(alpha) * alpha_dot;
  hmat_dot(3, 2) = cos(alpha) * alpha_dot;
  hmat_dot(2, 3) =
    -cos(alpha) * cos(beta) * alpha_dot + sin(alpha) * sin(beta) * beta_dot;
  hmat_dot(3, 3) =
    -sin(alpha) * cos(beta) * alpha_dot - cos(alpha) * sin(beta) * beta_dot;
  return hmat_dot;
}

grabnum::Matrix3d DHtfTiltTorsion(const double tilt_azimuth, const double tilt,
                                  const double tilt_azimuth_dot, const double tilt_dot)
{
  grabnum::Matrix3d hmat_dot;
  double c1 = cos(tilt_azimuth);
  double s1 = sin(tilt_azimuth);
  double c2 = cos(tilt);
  double s2 = sin(tilt);
  hmat_dot(1, 1) = s1 * s2 * tilt_azimuth_dot - c1 * c2 * tilt_dot;
  hmat_dot(1, 2) = -c1 * tilt_azimuth_dot;
  hmat_dot(1, 3) = -hmat_dot(1, 1);
  hmat_dot(2, 1) = -c1 * s2 * tilt_azimuth_dot - s1 * c2 * tilt_dot;
  hmat_dot(2, 2) = -s1 * tilt_azimuth_dot;
  hmat_dot(2, 3) = -hmat_dot(2, 1);
  hmat_dot(3, 1) = s2 * tilt_dot;
  hmat_dot(3, 2) = 0.0;
  hmat_dot(3, 3) = -s2 * tilt_dot;
  return hmat_dot;
}

} // end namespace grabgeom
