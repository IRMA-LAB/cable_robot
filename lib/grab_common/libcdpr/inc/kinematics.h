/**
 * @file kinematics.h
 * @author Edoardo Idà, Simone Comari
 * @date 28 Aug 2018
 * @brief File containing kinematics-related functions to be included in the GRAB CDPR
 * library.
 *
 * @note
 * <table>
 * <caption id="multi_row">Legend</caption>
 * <tr><th>Name     <th>Symbol     <th>Expression  <th>Description
 * <tr><td> _Global frame_  <td>@f$\mathcal{O}@f$ <td> -<td> World frame, centered on
 *point
 * @f$O@f$ and vertically aligned with gravitational axis.
 * <tr><td> _Local frame_  <td>@f$\mathcal{P}@f$ <td> -<td> Body-fixed frame centered on
 *an
 * arbitrary point @f$P@f$ belonging to the platform and arbitrarly oriented.
 * <tr><td> _World origin_  <td>@f$O@f$ <td> -<td> Origin of global frame_
 *@f$\mathcal{O}@f$.
 * <tr><td> _Local origin_  <td>@f$P@f$ <td> -<td> An arbitrary point rigidly fixed to the
 * platform, taken as the origin of _local frame_ @f$\mathcal{P}@f$.
 * <tr><td> _CoG_  <td>@f$G@f$ <td> - <td> The center of gravity (or baricenter) of the
 *platform.
 * <tr><td> _Swivel pulley system origin_  <td> @f$D_i@f$ <td> - <td> A fixed point around
 *which
 * the _i-th_ swivel pulley swivels. It also coincides with the entry point on the pulley
 *of the
 * cable unwinding from the _i-th_ winch. It is taken as origin of a fixed frame used to
 * parametrize the position of the pulley wrt global frame.
 * <tr><td> _Swivel pulley exit point_  <td>@f$B_i@f$ <td> - <td> The exit point of the
 *cable from
 * the _i-th_ swivel pulley, going to the platform. At this point the cable is assumed to
 *be
 * tangent to the pulley and belonging to the pulley plane (always true in static
 *conditions).
 * <tr><td> _Platform attach point_  <td>@f$A_i@f$ <td> - <td> The attaching point of the
 * _i-th_ cable to the platform. This point is fixed wrt the local frame.
 * <tr><td> _Pulley center_  <td>@f$C_i@f$ <td> - <td> The revolving center of the _i-th_
 * pulley.
 * <tr><td> _Position_  <td>@f$\mathbf{p}@f$ <td> @f$^\mathcal{O}(P - O)@f$ <td> Global
 * position of the platform expressed in global frame coordinates, i.e. of point @f$P@f$.
 * <tr><td> -  <td>@f$\mathbf{r}@f$ <td> @f$^\mathcal{O}(G - O)@f$ <td> Global
 * position of the platform's CoG @f$G@f$ expressed in global frame coordinates.
 * <tr><td> -  <td>@f$\mathbf{r}'@f$ <td> @f$^\mathcal{O}(G - P)@f$ <td> Local
 * position of the platform's CoG @f$G@f$ expressed in global frame coordinates.
 * <tr><td> -  <td>@f$^\mathcal{P}\mathbf{r}'@f$ <td> @f$^\mathcal{P}(G - P)@f$
 * <td> Local position of the platform's CoG @f$G@f$ expressed in local frame coordinates.
 * <tr><td> -  <td>@f$\mathbf{a}_i@f$ <td> @f$^\mathcal{O}(A_i - O)@f$ <td> Global
 * position of the attaching point of the _i-th_ cable to the platform expressed in global
 *frame
 * coordinates.
 * <tr><td> -  <td>@f$\mathbf{a}'_i@f$ <td> @f$^\mathcal{O}(A_i - P)@f$ <td> Local
 * position of the attaching point of the _i-th_ cable to the platform expressed in global
 *frame
 * coordinates.
 * <tr><td> -  <td>@f$^\mathcal{P}\mathbf{a}'_i@f$ <td> @f$^\mathcal{P}(A_i - P)@f$
 * <td> Local position of the attaching point of the _i-th_ cable to the platform
 *expressed in
 * local frame coordinates.
 * <tr><td> -  <td>@f$\mathbf{d}_i@f$ <td> @f$^\mathcal{O}(D_i - O)@f$ <td> Global
 * position of point @f$D_i@f$ expressed in global frame coordinates. This is a fixed
 *vector.
 * <tr><td> -  <td>@f$\mathbf{f}_i@f$ <td> @f$^\mathcal{O}(A_i - D_i)@f$ <td>
 * Time-variant vector expressed in global frame coordinates.
 * <tr><td> -  <td>@f$\mathbf{n}_i@f$ <td> @f$^\mathcal{O}(B_i - C_i)@f$ <td>
 * Time-variant vector expressed in global frame coordinates. Used to define tangent
 *angle.
 * <tr><td> _Cable vector_  <td>@f$\boldsymbol{\rho}_i@f$ <td>
 * @f$^\mathcal{O}(A_i - B_i)@f$ <td> Cable vector expressed in global frame coordinates.
 * <tr><td> _Cable length_  <td>@f$l_i@f$ <td> @f$\|A_i - B_i\| +
 * \overset{\large\frown}{B_i D_i}@f$ <td> Length of _i-th_ cable from point @f$D_i@f$ to
 * point @f$A_i@f$, including winding around swivel pulley.
 * <tr><td> _Pulley radius_ <td>@f$r_i@f$ <td> - <td> _i-th_ swivel pulley radius length.
 * <tr><td> _Swivel angle_ <td>@f$\sigma_i@f$ <td> - <td> _i-th_ pulley swivel angle, i.e.
 * the angle between @f$\hat{\mathbf{x}}_i@f$ and @f$\hat{\mathbf{u}}_i@f$.
 * <tr><td> _Tangent angle_ <td>@f$\psi_i@f$ <td> - <td> _i-th_ pulley tangent angle, i.e.
 * the angle between @f$\hat{\mathbf{u}}_i@f$ and @f$\hat{\mathbf{n}}_i@f$.
 * <tr><td> _Orientation_ <td> @f$\boldsymbol{\varepsilon}@f$ <td> - <td> Global
 * orientation of the platform expressed by 3 angles.
 * <tr><td> _Quaternion_ <td> @f$\boldsymbol{\varepsilon}_q@f$ <td> - <td> Global
 * orientation of the platform expressed by a quaternion @f$(q_w, q_x, q_y, q_z)^T@f$.
 * <tr><td> _Rotation matrix_ <td> @f$\mathbf{R}@f$ <td>
 * @f$\mathbf{R}(\boldsymbol{\varepsilon})@f$ <td> Rotation matrix from local to global
 * frame, function of the selected parametrization for the platform orientation.
 * <tr><td> _Pose_ <td> @f$\mathbf{q}@f$ <td>
 * @f$(\mathbf{p}^T, \boldsymbol{\varepsilon}^T)^T@f$ <td> Platform pose or generalized
 * variables using angles.
 * <tr><td> _Pose_ <td> @f$\mathbf{q}_q@f$ <td>
 * @f$(\mathbf{p}^T, \boldsymbol{\varepsilon}_q^T)^T@f$ <td> Platform pose or
 * generalized variables using quaternion.
 * </table>
 */

#ifndef GRABCOMMON_LIBCDPR_KINEMATICS_H
#define GRABCOMMON_LIBCDPR_KINEMATICS_H

#include "rotations.h"
#include "matrix.h"

/**
 * @brief namespace for CDPR-related utilities, such as kinematics and dynamics.
 */
namespace grabcdpr
{

///////////////////////////////////////////////////////////////////////////////
/// Enums
///////////////////////////////////////////////////////////////////////////////

/**
* @brief Rotation parametrization enum.
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
 */
typedef struct PlatformVarsStruct
{
  grabnum::Vector3d position;      /**< [m] 3D global position of the platform */
  grabnum::Vector3d orientation;   /**< [rad] global orientation expressed by 3 angles */
  grabnum::VectorXd<4> quaternion; /**< global orientation expressed by quaternion */
  grabnum::Matrix3d rot_mat;       /**< global orientation expressed by rotation matrix */
  grabnum::VectorXd<6> pose;       /**< global pose (position + orientation)  */
  grabnum::VectorXd<7> pose_q;     /**< global pose (position + quaternion) */

  grabnum::Vector3d pos_PG_glob; /**< [m] see @f$\mathbf{r}'_i@f$ */
  grabnum::Vector3d pos_OG_glob; /**< [m] see @f$\mathbf{r}_i@f$ */

  /**
   * @brief Constructor to initialize platform pose with position and angles.
   * @param[in] _position Platform global position.
   * @param[in] _orientation Platform global orientation expressed by angles.
   * @param[in] angles_type Desired rotation parametrization. Default is _TILT_TORSION_.
   */
  PlatformVarsStruct(const grabnum::Vector3d& _position,
                     const grabnum::Vector3d& _orientation,
                     const RotParametrization angles_type = TILT_TORSION)
  {
    UpdatePose(_position, _orientation, angles_type);
  }
  /**
   * @brief Constructor to initialize platform pose with position and quaternion.
   * @param[in] _position Platform global position.
   * @param[in] _orientation Platform global orientation expressed by quaternion.
   */
  PlatformVarsStruct(const grabnum::Vector3d& _position,
                     const grabnum::VectorXd<4>& _orientation)
  {
    UpdatePose(_position, _orientation);
  }

  /**
   * @brief Update platform pose with position and angles.
   * @param[in] _position Platform global position.
   * @param[in] _orientation Platform global orientation expressed by angles.
   * @param[in] angles_type Desired rotation parametrization. Default is _TILT_TORSION_.
   * @todo handle default case better
   */
  void UpdatePose(const grabnum::Vector3d& _position,
                  const grabnum::Vector3d& _orientation,
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
      rot_mat = grabgeom::RotZYZ(orientation);
      break;
    case TAIT_BRYAN:
      rot_mat = grabgeom::RotXYZ(orientation);
      break;
    case RPY:
      rot_mat = grabgeom::RotRPY(orientation);
      break;
    case TILT_TORSION:
      rot_mat = grabgeom::RotTiltTorsion(orientation);
      break;
    default:
      break;
    }
    quaternion = grabgeom::Rot2Quat(rot_mat);
  }
  /**
   * @brief Update platform pose with position and quaternion.
   * @param[in] _position Platform global position.
   * @param[in] _quaternion Platform global orientation expressed by quaternion.
   * @todo automatically update orientation from quaternion.
   */
  void UpdatePose(const grabnum::Vector3d& _position,
                  const grabnum::VectorXd<4>& _quaternion)
  {
    position = _position;
    quaternion = _quaternion;
    for (uint8_t i = 1; i <= 3; ++i)
    {
      pose_q(i) = position(i);
      pose(i) = position(i);
      pose_q(2 * i) = quaternion(i);
    }
    pose_q(7) = quaternion(4);
    rot_mat = grabgeom::Quat2Rot(quaternion);
  }
} PlatformVars;

/**
 * @brief Structure collecting variable related to a single generic cable of a CDPR.
 */
typedef struct CableVarsStruct
{
  double length; /**< [m] cable length @f$l_i@f$ */

  double swivel_ang; /**< [rad] swivel angle @f$\sigma_i@f$ */
  double tan_ang;    /**< [rad] tangent angle @f$\psi_i@f$ */

  grabnum::Vector3d pos_PA_glob; /**< [m] segment @f$\mathbf{a}'_i@f$ */
  grabnum::Vector3d pos_OA_glob; /**< [m] segment @f$\mathbf{a}_i@f$ */
  grabnum::Vector3d pos_DA_glob; /**< [m] segment @f$\mathbf{f}_i@f$ */
  grabnum::Vector3d pos_BA_glob; /**< [m] segment @f$\boldsymbol{\rho}_i@f$ */
  grabnum::Vector3d vers_u;   /**< _i-th_ swivel pulley versor @f$\hat{\mathbf{u}}_i@f$ */
  grabnum::Vector3d vers_w;   /**< _i-th_ swivel pulley versor @f$\hat{\mathbf{w}}_i@f$ */
  grabnum::Vector3d vers_n;   /**< _i-th_ swivel pulley versor @f$\hat{\mathbf{n}}_i@f$ */
  grabnum::Vector3d vers_rho; /**< _i-th_ cable versor @f$\hat{\boldsymbol{\rho}}_i@f$ */
} CableVars;

/**
 * @brief Structure collecting all variables related to a generic 6DoF CDPR.
 */
typedef struct VarsStruct
{
  PlatformVars* platform;        /**< variables of a generic 6DoF platform. */
  std::vector<CableVars> cables; /**< vector of variables of a single cables in a CDPR. */
} Vars;

/**
 * @brief Structure collecting parameters related to a generic 6DoF platform.
 */
typedef struct PlatformParamsStruct
{
  double mass = 0.0; /**< [Kg] platform mass (@f$m@f$). */
  grabnum::Vector3d
    ext_torque_loc; /**< [Nm] external torque vector expressed in the local frame. */
  grabnum::Vector3d
    ext_force_loc; /**< [N] external force vector expressed in the local frame. */
  grabnum::Vector3d pos_PG_loc; /**< [m] local CoG position (see @f$^P\mathbf{r}'@f$). */
  grabnum::Matrix3d inertia_mat_G_loc; /**< inertia matrix. */
} PlatformParams;

/**
 * @brief Structure collecting parameters related to a single generic cable of a CDPR.
 */
typedef struct CableParamsStruct
{
  double l0 = 0.0; /**< [m] length between @f$D_i@f$ and the exit point of the _i-th_
                      cable from the corresponding winch. */
  double motor_cable_tau = 0.0; /**< cable length-to-motor revolution ratio. */
  uint32_t motor_encoder_res =
    0; /**< motor encoder resolution in counts per revolution. */
  uint32_t swivel_pulley_encoder_res =
    0; /**< _i-th_ swivel pulley encoder resolution in counts per revolution. */
  double swivel_pulley_r = 0.0;  /**< [m] _i-th_ swivel pulley radius length @f$r_i@f$ */
  grabnum::Vector3d pos_OD_glob; /**< [m] see @f$\mathbf{d}_i@f$. */
  grabnum::Vector3d pos_PA_loc;  /**< see @f$\mathbf{a}_i'@f$. */
  grabnum::Vector3d vers_i; /**< versor @f$\hat{\mathbf{i}}_i@f$ of _i-th_ swivel pulley
                               expressed in global frame. */
  grabnum::Vector3d vers_j; /**< versor @f$\hat{\mathbf{j}}_i@f$ of _i-th_ swivel pulley
                               expressed in global frame. */
  grabnum::Vector3d vers_k; /**< versor @f$\hat{\mathbf{k}}_i@f$ of _i-th_ swivel pulley
                               expressed in global frame. */
} CableParams;

/**
 * @brief Structure collecting all parameters related to a generic 6DoF CDPR.
 */
typedef struct ParamsStruct
{
  PlatformParams* platform;  /**< parameters of a generic 6DoF platform. */
  std::vector<CableParams>
    cables;  /**< vector of parameters of a single cables in a CDPR. */
} Params;

///////////////////////////////////////////////////////////////////////////////
/// Functions
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Update platform-related quantities.
 * @param[in] position Platform global position.
 * @param[in] orientation Platform global orientation expressed by angles.
 * @param[in] angles_type Desired rotation parametrization. Default is _TILT_TORSION_.
 * @param[in] pos_PG_loc [m] local CoG position (see @f$^P\mathbf{r}'@f$).
 * @param[out] platform A pointer to the platform structure to be updated.
 */
void UpdatePlatform(const grabnum::Vector3d& position,
                    const grabnum::Vector3d& orientation,
                    const RotParametrization angles_type,
                    const grabnum::Vector3d& pos_PG_loc, PlatformVars* platform);

/**
 * @brief Update global position of point @f$A_i@f$ and relative segments.
 * @param[in] params A pointer to _i-th_ cable parameters.
 * @param[in] platform A pointer to the updated platform structure.
 * @param[out] cable A pointer to the cable structure including the positions to be updated.
 */
void UpdatePosA(const CableParams* params, const PlatformVars* platform,
                CableVars* cable);

/**
 * @brief Calculate swivel pulley versors @f$\hat{\mathbf{u}}_i, \hat{\mathbf{w}}_i@f$.
 * @param[in] params A pointer to _i-th_ cable parameters.
 * @param[in] swivel_ang Swivel angle of _i-th_ pulley.
 * @param[out] cable A pointer to the cable structure including the versors to be updated.
 */
void CalcPulleyVersors(const CableParams* params, const double swivel_ang,
                       CableVars* cable);

/**
 * @brief Calculate pulley swivel angle @f$\sigma_i@f$.
 * @param[in] params A pointer to _i-th_ cable parameters.
 * @param[in] pos_DA_glob Vector @f$\mathbf{f}_i@f$.
 * @return Swivel angle in radians.
 */
double CalcSwivelAngle(const CableParams* params, const grabnum::Vector3d& pos_DA_glob);

/**
 * @brief Calculate pulley tangent angle @f$\psi_i@f$.
 * @param[in] params A pointer to _i-th_ cable parameters.
 * @param[in] vers_u Versor @f$\hat{\mathbf{u}}_i@f$.
 * @param[in] pos_DA_glob Vector @f$\mathbf{f}_i@f$.
 * @return Tangent angle in radians.
 */
double CalcTangentAngle(const CableParams* params, const grabnum::Vector3d& vers_u,
                        const grabnum::Vector3d& pos_DA_glob);

/**
 * @brief Calculate cable versors @f$\hat{\mathbf{n}}_i, \hat{\boldsymbol{\rho}}_i@f$ and cable vector.
 * @param[in] params A pointer to _i-th_ cable parameters.
 * @param[in] vers_u Versor @f$\hat{\mathbf{u}}_i@f$.
 * @param[in] pos_DA_glob Vector @f$\mathbf{f}_i@f$.
 * @param[in] tan_ang Tangent angle in radians.
 * @param[out] cable A pointer to the cable structure including the variables to be calculated.
 */
void CalcCableVectors(const CableParams* params, const grabnum::Vector3d& vers_u,
                      const grabnum::Vector3d& pos_DA_glob, const double tan_ang,
                      CableVars* cable);

/**
 * @brief Calculate cable length.
 * @param[in] pos_BA_glob [m] Vector @f$\boldsymbol{\rho}_i@f$.
 * @param[in] pulley_radius Swivel pulley radius in meters.
 * @param[in] tan_ang Tangent angle in radians.
 * @return Cable length in meters.
 */
double CalcCableLen(const grabnum::Vector3d& pos_BA_glob, const double pulley_radius,
                    const double tan_ang)
{
  return pulley_radius * (M_PI - tan_ang) + grabnum::Norm(pos_BA_glob);
}

/**
 * @brief Update all cable variables at once.
 * @param[in] platform A pointer to the updated platform structure.
 * @param[in] params A pointer to _i-th_ cable parameters.
 * @param[out] cable A pointer to the cable structure to be updated.
 */
void UpdateCable(const CableParams* params, const PlatformVars* platform,
                 CableVars* cable);

/**
* @brief Update all robots variables at once (inverse kinematics problem).
* @param[in] position Platform global position.
* @param[in] orientation Platform global orientation expressed by angles.
* @param[in] angles_type Desired rotation parametrization. Default is _TILT_TORSION_.
* @param[in] params A pointer to the robot parameters structure.
* @param[out] vars A pointer to the robot variables structure to be updated.
*/
void UpdateIK(const grabnum::Vector3d& position, const grabnum::Vector3d& orientation,
              const RotParametrization angles_type, const Params* params, Vars* vars);

} // end namespace grabcdpr

#endif // GRABCOMMON_LIBCDPR_KINEMATICS_H
