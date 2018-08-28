/**
 * @file kinematics.h
 * @author Edoardo Idà, Simone Comari
 * @date 28 Aug 2018
 * @brief File containing kinematics-related functions to be included in the GRAB CDPR
 * library.
 */

#ifndef GRABCOMMON_LIBCDPR_KINEMATICS_H
#define GRABCOMMON_LIBCDPR_KINEMATICS_H

#include "rotations.h"
#include "matrix.h"

/**
 * @brief namespace for CDPR-related utilities.
 */
namespace grabcdpr
{

///////////////////////////////////////////////////////////////////////////////
/// Enums
///////////////////////////////////////////////////////////////////////////////

/**
* @brief Angle parametrization enum.
* @todo description of single elements.
*/
typedef enum RotParametrizationEnum
{
  EULER_ZYZ,
  TAIT_BRYAN,
  RPY,
  TILT_TORSION,
  QUATERNION
} RotParametrization;

///////////////////////////////////////////////////////////////////////////////
/// Structs
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Structure collecting all variables related to a generic 6DoF platform.
 * @todo description of single elements.
 */
typedef struct PlatformVarsStruct
{
  grabnum::Vector3d position; /**< [m] 3D global position of the platform */
  grabnum::Vector3d
    orientation; /**< [rad] global orientation of the platform expressed by 3 angles */
  grabnum::VectorXd<4>
    quaternion; /**< global orientation of the platform expressed by quaternion */
  grabnum::Matrix3d
    rot_mat; /**< global orientation of the platform expressed by rotation matrix */
  grabnum::VectorXd<6>
    pose; /**< global pose (position + orientation), i.e. generalized variables  */
  grabnum::VectorXd<7>
    pose_q; /**< global pose (position + quaternion), i.e. generalized variables  */

  /**
   * @brief PlatformVarsStruct
   * @param _position
   * @param _orientation
   * @param angles_type
   */
  PlatformVarsStruct(const grabnum::Vector3d& _position,
                     const grabnum::Vector3d& _orientation,
                     const RotParametrization angles_type = TILT_TORSION)
  {
    Update(_position, _orientation, angles_type);
  }
  /**
   * @brief PlatformVarsStruct
   * @param _position
   * @param _orientation
   */
  PlatformVarsStruct(const grabnum::Vector3d& _position,
                     const grabnum::VectorXd<4>& _orientation)
  {
    Update(_position, _orientation);
  }

  /**
   * @brief Update
   * @param _position
   * @param _quaternion
   * @todo doc + conversion from quaternion to some angles
   */
  void Update(const grabnum::Vector3d& _position, const grabnum::VectorXd<4>& _quaternion)
  {
    position = _position;
    quaternion = _quaternion;
    for (uint8_t i = 1; i <= 3; ++i)
    {
      pose(i) = position(i);
      pose(2 * i) = quaternion(i);
    }
    pose(7) = quaternion(4);
    rot_mat = grabgeom::Quat2Rot(quaternion);
  }
  /**
   * @brief Update
   * @param _position
   * @param _orientation
   * @param angles_type
   * @todo doc
   */
  void Update(const grabnum::Vector3d& _position, const grabnum::Vector3d& _orientation,
              const RotParametrization angles_type = TILT_TORSION)
  {
    position = _position;
    orientation = _orientation;
    for (uint8_t i = 1; i <= 3; ++i)
    {
      pose(i) = position(i);
      pose(2 * i) = orientation(i);
    }
    switch (angles_type)
    {
    case EULER_ZYZ:
      rot_mat = grabgeom::RotZYZ(orientation(1), orientation(2), orientation(3));
      break;
    case TAIT_BRYAN:
      rot_mat = grabgeom::RotXYZ(orientation(1), orientation(2), orientation(3));
      break;
    case RPY:
      rot_mat = grabgeom::RotRPY(orientation(1), orientation(2), orientation(3));
      break;
    case TILT_TORSION:
      rot_mat = grabgeom::RotTiltTorsion(orientation(1), orientation(2), orientation(3));
      break;
    default:
      // TODO: handle this case better
      break;
    }
    quaternion = grabgeom::Rot2Quat(rot_mat);
  }
} PlatformVars;

/**
 * @brief Structure collecting variable related to a single generic cable of a CDPR.
 * @todo description of single elements
 */
typedef struct CableVarsStruct
{
  double length;     /**< [m] cable length defined as @f$l_i = \|A_i - B_i\| +
                        \overset{\large\frown}{B_i D_i}@f$ */
  double swivel_ang; /**< [rad] swivel angle, i.e. the angle between @f$\hat{x_i}@f$ and
                        @f$\hat{u_i}@f$ */
  double tan_ang;    /**< [rad] angle between @f$\hat{u_i}@f$ and @f$\hat{n_i}@f$ */

  /**
   * @brief CableVarsStruct
   * @param _length
   * @param _swivel_ang
   * @param _tan_ang
   */
  CableVarsStruct(const double _length, const double _swivel_ang, const double _tan_ang)
  {
    Update(_length, _swivel_ang, _tan_ang);
  }

  /**
   * @brief Update
   * @param _length
   * @param _swivel_ang
   * @param _tan_ang
   */
  void Update(const double _length, const double _swivel_ang, const double _tan_ang)
  {
    length = _length;
    swivel_ang = _swivel_ang;
    tan_ang = _tan_ang;
  }
} CableVars;

/**
 * @brief Structure collecting all variables related to a generic 6DoF CDPR.
 * @todo description of single elements.
 */
typedef struct VarsStruct
{
  PlatformVars platform;         /**< variables of a generic 6DoF platform. */
  std::vector<CableVars> cables; /**< vector of variables of a single cables in a CDPR. */
} Vars;

/**
 * @brief Structure collecting parameters related to a generic 6DoF platform.
 */
typedef struct PlatformParamsStruct
{
  double mass = 0.0; /**< [Kg] platform mass. */
  grabnum::Vector3d
    ext_torque_loc; /**< [Nm] external torque vector expressed in the local frame. */
  grabnum::Vector3d
    ext_force_loc; /**< [N] external force vector expressed in the local frame. */
  grabnum::Vector3d pos_G_loc; /**< [m] CoG position expressed in the local frame. */
  grabnum::Matrix3d
    inertia_mat_G_loc; /**< Inertia matrix expressed in the local frame. */
} PlatformParams;

/**
 * @brief Structure collecting parameters related to a single generic cable of a CDPR.
 */
typedef struct CableParamsStruct
{
  double l0 =
    0.0; /**< length between @f$D_i@f$ and the exit point from the corresponding winch. */
  double motor_cable_tau = 0.0; /**< cable length-to-motor revolution ratio. */
  uint32_t motor_encoder_res =
    0; /**< motor encoder resolution in count per revolution. */
  uint32_t swivel_pulley_encoder_res =
    0; /**< swivel pully encoder resolution in count per revolution. */
  double swivel_pulley_r = 0.0; /**< [m] swivel pulley radius */
  grabnum::Vector3d pos_D_glob; /**< global position of point @f$D_i@f$. */
  grabnum::Vector3d pos_A_loc;  /**< local position of point @f$A_i@f$. */
  grabnum::Vector3d
    vers_i; /**< versor @f$\hat{x_i}@f$ of _i-th_ pulley expressed in global frame. */
  grabnum::Vector3d
    vers_j; /**< versor @f$\hat{y_i}@f$ of _i-th_ pulley expressed in global frame. */
  grabnum::Vector3d
    vers_k; /**< versor @f$\hat{z_i}@f$ of _i-th_ pulley expressed in global frame. */
} CableParams;

/**
 * @brief Structure collecting all parameters related to a generic 6DoF CDPR.
 */
typedef struct ParamsStruct
{
  PlatformParams platform; /**< parameters of a generic 6DoF platform. */
  std::vector<CableParams>
    cables; /**< vector of parameters of a single cables in a CDPR. */
} Params;

///////////////////////////////////////////////////////////////////////////////
/// Functions
///////////////////////////////////////////////////////////////////////////////


} // end namespace grabcdpr

#endif // GRABCOMMON_LIBCDPR_KINEMATICS_H
