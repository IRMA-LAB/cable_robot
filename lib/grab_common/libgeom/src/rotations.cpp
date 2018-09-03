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
  rot(3, 3) = rot(2, 2);
  rot(3, 2) = sin(angle);
  rot(2, 3) = -rot(3, 2);
  return rot;
}

grabnum::Matrix3d RotY(const double angle)
{
  grabnum::Matrix3d rot(1.0);
  rot(1, 1) = cos(angle);
  rot(3, 3) = rot(1, 1);
  rot(1, 3) = sin(angle);
  rot(3, 1) = -rot(1, 3);
  return rot;
}

grabnum::Matrix3d RotZ(const double angle)
{
  grabnum::Matrix3d rot(1.0);
  rot(1, 1) = cos(angle);
  rot(2, 2) = rot(2, 2);
  rot(2, 1) = sin(angle);
  rot(1, 2) = -rot(2, 1);
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
  double cos_beta = cos(beta);
  hmat(1, 3) = sin(beta);
  hmat(2, 2) = cos(alpha);
  hmat(3, 2) = sin(alpha);
  hmat(2, 3) = -hmat(3, 2) * cos_beta;
  hmat(3, 3) = hmat(2, 2) * cos_beta;
  return hmat;
}

grabnum::Matrix3d HtfRPY(const double roll, const double pitch)
{
  grabnum::Matrix3d hmat;
  double c1 = cos(roll);
  double s1 = sin(roll);
  double c2 = cos(pitch);
  hmat(1, 2) = -s1;
  hmat(1, 3) = c1 * c2;
  hmat(2, 2) = c1;
  hmat(2, 3) = s1 * c2;
  hmat(3, 1) = 1.0;
  hmat(3, 3) = sin(pitch);
  return hmat;
}

grabnum::Matrix3d HtfZYZ(const double alpha, const double beta)
{
  grabnum::Matrix3d hmat;
  double sin_beta = sin(beta);
  hmat(1, 2) = -sin(alpha);
  hmat(2, 2) = cos(alpha);
  hmat(1, 3) = hmat(2, 2) * sin_beta;
  hmat(2, 3) = -hmat(1, 2) * sin_beta;
  hmat(3, 1) = 1.0;
  hmat(3, 3) = cos(beta);
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

grabnum::Matrix3d DHtfXYZ(const double alpha, const double beta, const double alpha_dot,
                          const double beta_dot)
{
  grabnum::Matrix3d hmat_dot;
  double c1 = cos(alpha);
  double s1 = sin(alpha);
  double c2 = cos(beta);
  double s2 = sin(beta);
  hmat_dot(1, 3) = c2 * beta_dot;
  hmat_dot(2, 2) = -s1 * alpha_dot;
  hmat_dot(3, 2) = c1 * alpha_dot;
  hmat_dot(2, 3) = -c1 * c2 * alpha_dot + s1 * s2 * beta_dot;
  hmat_dot(3, 3) = -s1 * c2 * alpha_dot - c1 * s2 * beta_dot;
  return hmat_dot;
}

grabnum::Matrix3d DHtfRPY(const double roll, const double pitch, const double roll_dot,
                          const double pitch_dot)
{
  grabnum::Matrix3d hmat_dot;
  double c1 = cos(roll);
  double s1 = sin(roll);
  double c2 = cos(pitch);
  double s2 = sin(pitch);
  hmat_dot(1, 2) = -c1 * roll_dot;
  hmat_dot(1, 3) = -s1 * c2 * roll_dot - c1 * s2 * pitch_dot;
  hmat_dot(2, 2) = -s1 * roll_dot;
  hmat_dot(2, 3) = c1 * c2 * roll_dot - s1 * s2 * pitch_dot;
  hmat_dot(3, 3) = -c2 * pitch_dot;
  return hmat_dot;
}

grabnum::Matrix3d DHtfZYZ(const double alpha, const double beta, const double alpha_dot,
                          const double beta_dot)
{
  grabnum::Matrix3d hmat_dot;
  double c1 = cos(alpha);
  double s1 = sin(alpha);
  double c2 = cos(beta);
  double s2 = sin(beta);
  hmat_dot(1, 2) = -c1 * alpha_dot;
  hmat_dot(1, 3) = -s1 * s2 * alpha_dot + c1 * c2 * beta_dot;
  hmat_dot(2, 2) = -s1 * alpha_dot;
  hmat_dot(2, 3) = c1 * s2 * alpha_dot + s1 * s2 * beta_dot;
  hmat_dot(3, 3) = -s2 * beta_dot;
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
  hmat_dot(3, 3) = -hmat_dot(3, 1);
  return hmat_dot;
}

} // end namespace grabgeom
