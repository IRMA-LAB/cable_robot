/**
 * @file kinematics.h
 * @author Edoardo Idà, Simone Comari
 * @date 31 Aug 2018
 * @brief File containing kinematics-related functions to be included in the GRAB CDPR
 * library.
 *
 * @note
 * <table>
 * <caption id="legend">Legend</caption>
 * <tr><th>Name     <th>Symbol     <th>Expression  <th>Description
 * <tr><td> _Global frame_  <td>@f$\mathcal{O}@f$ <td> -<td> World frame, centered on
 *point
 * @f$O@f$ and vertically aligned with gravitational axis.
 * <tr><td> _Local frame_  <td>@f$\mathcal{P}@f$ <td> -<td> Body-fixed frame centered
 * on an arbitrary point @f$P@f$ belonging to the platform and arbitrarly oriented.
 * <tr><td> _Swivel pulley frame_  <td>@f$\mathcal{D}_i@f$ <td> -<td> Fixed frame
 * centered on point @f$D@f$. Its @f$z@f$-axis represents the swiveling axis of the _i-th_
 * pulley.
 * <tr><td> _World origin_  <td>@f$O@f$ <td> -<td> Origin of global frame_
 *@f$\mathcal{O}@f$.
 * <tr><td> _Local origin_  <td>@f$P@f$ <td> -<td> An arbitrary point rigidly fixed to the
 * platform, taken as the origin of _local frame_ @f$\mathcal{P}@f$.
 * <tr><td> _CoG_  <td>@f$G@f$ <td> - <td> The center of gravity (or baricenter) of the
 *platform.
 * <tr><td> _Swivel pulley frame origin_  <td> @f$D_i@f$ <td> - <td> A fixed point around
 * which the _i-th_ swivel pulley swivels. It also coincides with the entry point on the
 *pulley
 * of the cable unwinding from the _i-th_ winch. It is taken as origin of a fixed frame
 *used to
 * parametrize the position of the pulley wrt global frame.
 * <tr><td> _Swivel pulley exit point_  <td>@f$B_i@f$ <td> - <td> The exit point of the
 * cable from the _i-th_ swivel pulley, going to the platform. At this point the cable is
 *assumed
 * to be tangent to the pulley and belonging to the pulley plane (always true in static
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
 * <tr><td> _Transformation matrix_ <td> @f$\mathbf{H}@f$ <td>
 * @f$\mathbf{H}(\boldsymbol{\varepsilon})@f$ <td> Transformation matrix from angles
 * velocities @f$\dot{\boldsymbol{\varepsilon}}@f$ to angular speed vector
 * @f$\boldsymbol{\omega}@f$, function of the selected parametrization for the platform
 * orientation.
 * <tr><td> _Pose_ <td> @f$\mathbf{q}@f$ <td>
 * @f$(\mathbf{p}^T, \boldsymbol{\varepsilon}^T)^T@f$ <td> Platform pose or generalized
 * variables using angles.
 * <tr><td> _Pose variant_ <td> @f$\mathbf{q}_q@f$ <td>
 * @f$(\mathbf{p}^T, \boldsymbol{\varepsilon}_q^T)^T@f$ <td> Platform pose or
 * generalized variables using quaternion.
 * <tr><td> - <td>@f$\hat{\mathbf{i}}_i@f$ <td> - <td> @f$x@f$-axis versor of _i-th_
 * swivel pulley frame @f$\mathcal{D}_i@f$.
 * <tr><td> - <td>@f$\hat{\mathbf{j}}_i@f$ <td> - <td> @f$y@f$-axis versor of _i-th_
 * swivel pulley frame @f$\mathcal{D}_i@f$.
 * <tr><td> - <td>@f$\hat{\mathbf{k}}_i@f$ <td> - <td> @f$z@f$-axis versor of _i-th_
 * swivel pulley frame @f$\mathcal{D}_i@f$. This is also the swiveling axis of the pulley.
 * <tr><td> - <td>@f$\hat{\mathbf{u}}_i@f$ <td>
 * @f$\hat{\mathbf{u}}_i \perp \hat{\mathbf{w}}_i \perp \hat{\mathbf{k}}_i@f$ <td>
 * @f$x@f$-axis versor of a time-variant body-fixed frame to the _i-th_ swivel pulley. In
 * particular, @f$\hat{\mathbf{u}}_i@f$ belongs to the pulley plane and it is
 *perpendicular to
 * @f$\hat{\mathbf{k}}_i@f$.
 * <tr><td> - <td>@f$\hat{\mathbf{w}}_i@f$ <td>
 * @f$\hat{\mathbf{u}}_i \perp \hat{\mathbf{w}}_i \perp \hat{\mathbf{k}}_i@f$ <td>
 * @f$y@f$-axis versor of a time-variant body-fixed frame to the _i-th_ swivel pulley. In
 * particular, @f$\hat{\mathbf{w}}_i@f$ is perpendicular to the pulley plane and to
 * @f$\hat{\mathbf{k}}_i@f$.
 * <tr><td> - <td>@f$\hat{\mathbf{n}}_i@f$ <td> - <td> Time-variant versor denoting the
 * direction of @f$\mathbf{n}_i@f$ and used to define the tangent angle @f$\psi_i@f$. Note
 * that @f$\mathbf{n}_i = r_i \hat{\mathbf{n}}_i@f$.
 * <tr><td> - <td>@f$\hat{\boldsymbol{\rho}}_i@f$ <td> - <td> Time-variant versor
 * denoting the direction of @f$\boldsymbol{\rho}_i@f$. Note that
 * @f$\hat{\boldsymbol{\rho}}_i \perp \hat{\mathbf{n}}_i@f$.
 * <tr><td> _Time-derivative_ <td>@f$\dot{(\cdot)}@f$ <td> @f$\frac{d(\cdot)}{dt}@f$
 * <td> -
 * <tr><td> _Angular speed_ <td>@f$\boldsymbol{\omega}@f$ <td>
 * @f$\mathbf{H}(\boldsymbol{\varepsilon})\dot{\boldsymbol{\varepsilon}}@f$ <td> Angular
 * speed vector of the platform expressed in global coordinates.
 * <tr><td> _Angular acceleration_ <td>@f$\boldsymbol{\alpha}@f$ <td>
 * @f$\dot{\mathbf{H}}(\dot{\boldsymbol{\varepsilon}}, \boldsymbol{\varepsilon})
 * \dot{\boldsymbol{\varepsilon}} + \mathbf{H}(\boldsymbol{\varepsilon})
 * \ddot{\boldsymbol{\varepsilon}} @f$ <td> Angular acceleration vector of the platform
 * expressed in global coordinates.
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
*
* Defines the way a rotation matrix is determined according to the convention used.
*/
typedef enum RotParametrizationEnum
{
  EULER_ZYZ,    /**< _Euler_ angles convention with @f$Z_1Y_2Z_3@f$ order. */
  TAIT_BRYAN,   /**< _Tait-Bryan_ angles convention and @f$X_1Y_2Z_3@f$. */
  RPY,          /**< _Roll, Pitch, Yaw_ angles convention (from aviation). */
  TILT_TORSION, /**< _Tilt-and-torsion_ angles, a variation of _Euler_ angles convention.
                   */
  QUATERNION    /**< Use _quaternion_ to determine rotation. */
} RotParametrization;

///////////////////////////////////////////////////////////////////////////////
/// Structs
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Structure collecting all variables related to a generic 6DoF platform.
 * @note See @ref legend for symbols reference.
 * @todo implement quaternion 1st-2nd order stuff.
 */
typedef struct PlatformVarsStruct
{
  RotParametrization angles_type = TILT_TORSION; /**< rotation parametrization used. */

  /** @addtogroup ZeroOrderKinematics
   * @{
   */
  grabnum::Vector3d position;    /**< [_m_] vector @f$\mathbf{p}@f$. */
  grabnum::Vector3d orientation; /**< [_rad_] vector @f$\boldsymbol{\varepsilon}@f$. */
  grabnum::VectorXd<4> quaternion; /**< vector @f$\boldsymbol{\varepsilon}_q@f$. */

  grabnum::Matrix3d rot_mat;   /**< matrix @f$\mathbf{R}@f$. */
  grabnum::VectorXd<6> pose;   /**< vector @f$\mathbf{q}@f$.  */
  grabnum::VectorXd<7> pose_q; /**< vector @f$\mathbf{q}_q@f$. */

  grabnum::Vector3d pos_PG_glob; /**< [_m_] vector @f$\mathbf{r}'@f$.*/
  grabnum::Vector3d pos_OG_glob; /**< [_m_] vector @f$\mathbf{r}@f$.*/
  /** @} */                      // end of ZeroOrderKinematics group

  /** @addtogroup FirstOrderKinematics
   * @{
   */
  grabnum::Vector3d velocity; /**< [_m/s_] vector @f$\dot{\mathbf{p}}@f$. */
  grabnum::Vector3d
    angles_speed; /**< [_rad/s_] vector @f$\dot{\boldsymbol{\varepsilon}}@f$. */

  grabnum::Vector3d angular_vel; /**< vector @f$\boldsymbol\omega@f$. */

  grabnum::Vector3d vel_OG_glob; /**< [_m/s_] vector @f$\dot{\mathbf{r}}@f$. */
  /** @} */                      // end of FirstOrderKinematics group

  /** @addtogroup SecondOrderKinematics
   * @{
   */
  grabnum::Vector3d
    acceleration; /**< [_m/s<sup>2</sup>_] vector @f$\ddot{\mathbf{p}}@f$. */
  grabnum::Vector3d
    angles_acc; /**< [_rad/s<sup>2</sup>_] vector @f$\ddot{\boldsymbol{\varepsilon}}@f$. */

  grabnum::Vector3d angular_acc; /**< vector @f$\boldsymbol\alpha@f$.*/

  grabnum::Vector3d
    acc_OG_glob; /**< [_m/s<sup>2</sup>_] vector @f$\ddot{\mathbf{r}}@f$.*/
  /** @} */      // end of SecondOrderKinematics group

  /**
   * @brief Constructor to explicitly declare rotation parametrization desired only.
   * @param[in] _angles_type Desired rotation parametrization.
   */
  PlatformVarsStruct(const RotParametrization _angles_type)
  {
    angles_type = _angles_type;
  }
  /**
   * @brief Constructor to initialize platform vars with position and angles and their
   * first and
   * second derivatives.
   * @param[in] _position [m] Platform global position @f$\mathbf{p}@f$.
   * @param[in] _velocity [m/s] Platform global linear velocity @f$\dot{\mathbf{p}}@f$.
   * @param[in] _acceleration [m/s<sup>2</sup>] Platform global linear acceleration
   * @f$\ddot{\mathbf{p}}@f$.
   * @param[in] _orientation [rad] Platform global orientation expressed by angles
   * @f$\boldsymbol{\varepsilon}@f$.
   * @param[in] _angles_speed [rad/s] Platform orientation time-derivative
   * @f$\dot{\boldsymbol{\varepsilon}}@f$.
   * @param[in] _angles_acc [rad/s<sup>2</sup>] Platform orientation 2nd time-derivative
   * @f$\ddot{\boldsymbol{\varepsilon}}@f$.
   * @param[in] _angles_type Desired rotation parametrization. Default is @a TILT_TORSION.
   * @note See @ref legend for more details.
   */
  PlatformVarsStruct(const grabnum::Vector3d& _position,
                     const grabnum::Vector3d& _velocity,
                     const grabnum::Vector3d& _acceleration,
                     const grabnum::Vector3d& _orientation,
                     const grabnum::Vector3d& _angles_speed,
                     const grabnum::Vector3d& _angles_acc,
                     const RotParametrization _angles_type = TILT_TORSION)
  {
    angles_type = _angles_type;
    UpdatePose(_position, _orientation);
    UpdateVel(_velocity, _angles_speed);
    UpdateAcc(_acceleration, _angles_acc);
  }
  /**
   * @brief Constructor to initialize platform pose with position and quaternion.
   * @param[in] _position [m] Platform global position @f$\mathbf{p}@f$.
   * @param[in] _orientation Platform global orientation expressed by quaternion
   * @f$\boldsymbol{\varepsilon}_q@f$.
   * @todo include velocities and speed for quaternion case too.
   * @note See @ref legend for more details.
   */
  PlatformVarsStruct(const grabnum::Vector3d& _position,
                     const grabnum::VectorXd<4>& _orientation)
  {
    angles_type = QUATERNION;
    UpdatePose(_position, _orientation);
  }

  /**
   * @brief Update platform pose with position and angles.
   * @param[in] _position [m] Platform global position @f$\mathbf{p}@f$.
   * @param[in] _orientation [rad] Platform global orientation expressed by angles
   * @f$\boldsymbol{\varepsilon}@f$.
   * @todo handle default case better
   * @ingroup ZeroOrderKinematics
   * @see UpdateVel() UpdateAcc()
   * @note See @ref legend for more details.
   */
  void UpdatePose(const grabnum::Vector3d& _position,
                  const grabnum::Vector3d& _orientation)
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
   * @param[in] _position [m] Platform global position @f$\mathbf{p}@f$.
   * @param[in] _quaternion Platform global orientation expressed by quaternion
   * @f$\boldsymbol{\varepsilon}_q = (q_w, q_x, q_y, q_z)@f$.
   * @todo automatically update orientation from quaternion.
   * @ingroup ZeroOrderKinematics
   * @see UpdateVel() UpdateAcc()
   * @note See @ref legend for more details.
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

  /**
   * @brief Update platform velocities with linear velocity and angles speed.
   * @param[in] _velocity [m/s] Platform global linear velocity @f$\dot{\mathbf{p}}@f$.
   * @param[in] _angles_speed [rad/s] Vector @f$\dot{\boldsymbol{\varepsilon}}@f$.
   * @todo implement HtfRPY()
   * @ingroup FirstOrderKinematics
   * @see UpdatePose() UpdateAcc()
   * @note See @ref legend for more details.
   */
  void UpdateVel(const grabnum::Vector3d& _velocity,
                 const grabnum::Vector3d& _angles_speed)
  {
    velocity = _velocity;
    angles_speed = _angles_speed;
    switch (angles_type)
    {
    case EULER_ZYZ:
      angular_vel = grabgeom::HtfZYZ(orientation) * angles_speed;
      break;
    case TAIT_BRYAN:
      angular_vel = grabgeom::HtfXYZ(orientation) * angles_speed;
      break;
    case TILT_TORSION:
      angular_vel = grabgeom::HtfTiltTorsion(orientation) * angles_speed;
      break;
    default:
      break;
    }
  }

  /**
   * @brief Update platform accelerations with linear and angles acceleration.
   * @param[in] _acceleration [m/s<sup>2</sup>] Vector @f$\ddot{\mathbf{p}}@f$.
   * @param[in] _angles_acc [rad/s<sup>2</sup>] Vector
   * @f$\ddot{\boldsymbol{\varepsilon}}@f$.
   * @todo implement DHtfRPY(), DHtfZYZ()
   * @ingroup SecondOrderKinematics
   * @see UpdateVel() UpdateAcc()
   * @note See @ref legend for more details.
   */
  void UpdateAcc(const grabnum::Vector3d& _acceleration,
                 const grabnum::Vector3d& _angles_acc)
  {
    acceleration = _acceleration;
    angles_acc = _angles_acc;
    switch (angles_type)
    {
    case TAIT_BRYAN:
      angular_acc = grabgeom::DHtfXYZ(orientation, angles_speed) * angles_speed +
                    grabgeom::HtfXYZ(orientation) * angles_acc;
      break;
    case TILT_TORSION:
      angular_acc = grabgeom::DHtfTiltTorsion(orientation, angles_speed) * angles_speed +
                    grabgeom::HtfTiltTorsion(orientation) * angles_acc;
      break;
    default:
      break;
    }
  }
} PlatformVars;

/**
 * @brief Structure collecting variable related to a single generic cable of a CDPR.
 * @note See @ref legend for symbols reference.
 */
typedef struct CableVarsStruct
{
  /** @addtogroup ZeroOrderKinematics
   * @{
   */
  double length; /**< [_m_] cable length @f$l_i@f$. */

  double swivel_ang; /**< [_rad_] _i-th_ pulley swivel angle @f$\sigma_i@f$. */
  double tan_ang;    /**< [_rad_] _i-th_ pulley tangent angle @f$\psi_i@f$. */

  grabnum::Vector3d pos_PA_glob; /**< [_m_] vector @f$\mathbf{a}'_i@f$. */
  grabnum::Vector3d pos_OA_glob; /**< [_m_] vector @f$\mathbf{a}_i@f$. */
  grabnum::Vector3d pos_DA_glob; /**< [_m_] vector @f$\mathbf{f}_i@f$. */
  grabnum::Vector3d pos_BA_glob; /**< [_m_] vector @f$\boldsymbol{\rho}_i@f$. */

  grabnum::Vector3d vers_u; /**< _i-th_ swivel pulley versor @f$\hat{\mathbf{u}}_i@f$. */
  grabnum::Vector3d vers_w; /**< _i-th_ swivel pulley versor @f$\hat{\mathbf{w}}_i@f$. */
  grabnum::Vector3d vers_n; /**< _i-th_ swivel pulley versor @f$\hat{\mathbf{n}}_i@f$. */
  grabnum::Vector3d vers_rho; /**< _i-th_ cable versor @f$\hat{\boldsymbol{\rho}}_i@f$. */
  /** @} */                   // end of ZeroOrderKinematics group

  /** @addtogroup FirstOrderKinematics
   * @{
   */
  double speed; /**< [_m/s_] _i-th_ cable speed @f$\dot{l}_i@f$. */

  double
    swivel_ang_vel; /**< [_rad/s_] _i-th_ pulley swivel angle speed @f$\dot{\sigma}_i@f$. */
  double
    tan_ang_vel; /**< [_rad/s_] _i-th_ pulley tangent angle speed @f$\dot{\psi}_i@f$. */

  grabnum::Vector3d vel_OA_glob; /**< [_m/s_] vector @f$\dot{\mathbf{a}}_i@f$. */
  grabnum::Vector3d vel_BA_glob; /**< [_m/s_] vector @f$\dot{\boldsymbol{\rho}}_i@f$. */

  grabnum::Vector3d vers_u_dot;   /**< versor @f$\dot{\hat{\mathbf{u}}}_i@f$. */
  grabnum::Vector3d vers_w_dot;   /**< versor @f$\dot{\hat{\mathbf{w}}}_i@f$. */
  grabnum::Vector3d vers_n_dot;   /**< versor @f$\dot{\hat{\mathbf{n}}}_i@f$. */
  grabnum::Vector3d vers_rho_dot; /**< versor @f$\dot{\hat{\boldsymbol{\rho}}}_i@f$. */
  /** @} */                       // end of FirstOrderKinematics group

  /** @addtogroup SecondOrderKinematics
   * @{
   */
  double acceleration; /**< [_m/s<sup>2</sup>_] cable acceleration @f$\ddot{l}_i@f$. */

  double swivel_ang_acc; /**< [_rad/s<sup>2</sup>_] _i-th_ pulley @f$\ddot{\sigma}_i@f$. */
  double tan_ang_acc;    /**< [_rad/s<sup>2</sup>_] _i-th_ pulley @f$\ddot{\psi}_i@f$. */

  grabnum::Vector3d
    acc_OA_glob; /**< [_m/s<sup>2</sup>_] vector @f$\ddot{\mathbf{a}}_i@f$. */
  /** @} */      // end of SecondOrderKinematics group

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
  grabnum::Vector3d pos_PG_loc;        /**< [m] vector @f$^\mathcal{P}\mathbf{r}'@f$. */
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
  grabnum::Vector3d pos_OD_glob; /**< [m] vector @f$\mathbf{d}_i@f$. */
  grabnum::Vector3d pos_PA_loc;  /**< vector @f$\mathbf{a}_i'@f$. */
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
  PlatformParams* platform; /**< parameters of a generic 6DoF platform. */
  std::vector<CableParams>
    cables; /**< vector of parameters of a single cables in a CDPR. */
} Params;

///////////////////////////////////////////////////////////////////////////////
//// Functions
///////////////////////////////////////////////////////////////////////////////

/** @defgroup ZeroOrderKinematics Zero Order Kinematics
 * This group collects all elements related to zero-order kinematics of a generic 6DoF
 * CDPR.
 * @{
 */

/**
 * @brief Update platform-related zero-order quantities (explicit).
 *
 * Given a new pose of the platform
 * @f$\mathbf{q} = (\mathbf{p}^T, \boldsymbol{\varepsilon}^T)^T@f$, the following
 * quantities are updated:
 * @f[
 * \mathbf{r}' = \mathbf{R}(\boldsymbol{\varepsilon}) ^\mathcal{P}\mathbf{r}' \\
 * \mathbf{r} = \mathbf{p} + \mathbf{r}'
 * @f]
 * being @f$^\mathcal{P}\mathbf{r}'@f$ a known parameter.
 * @param[in] position [m] Platform global position @f$\mathbf{p}@f$.
 * @param[in] orientation [rad] Platform global orientation expressed by angles
 * @f$\boldsymbol{\varepsilon}@f$.
 * @param[in] pos_PG_loc [m] Local CoG position @f$^\mathcal{P}\mathbf{r}'@f$.
 * @param[out] platform A pointer to the platform variables structure to be updated.
 * @note See @ref legend for symbols reference.
 */
void UpdatePlatformPose(const grabnum::Vector3d& position,
                        const grabnum::Vector3d& orientation,
                        const grabnum::Vector3d& pos_PG_loc, PlatformVars* platform);
/**
 * @brief Update platform-related zero-order quantities (implicit).
 * @param[in] position [m] Platform global position @f$\mathbf{p}@f$.
 * @param[in] orientation [rad] Platform global orientation expressed by angles
 * @f$\boldsymbol{\varepsilon}@f$.
 * @param[in] params A pointer to the platform parameters structure.
 * @param[out] platform A pointer to the platform variables structure to be updated.
 * @see UpdatePlatformPose()
 */
void UpdatePlatformPose(const grabnum::Vector3d& position,
                        const grabnum::Vector3d& orientation,
                        const PlatformParams* params, PlatformVars* platform);

/**
 * @brief Update global position of point @f$A_i@f$ and relative segments.
 *
 * Given current platform variables @f$\mathbf{R}, \mathbf{p}@f$, the following quantities
 * are updated:
 * @f[
 * \mathbf{a}'_i = \mathbf{R}(\boldsymbol{\varepsilon}) ^\mathcal{P}\mathbf{a}'_i \\
 * \mathbf{a}_i = \mathbf{p} + \mathbf{a}'_i \\
 * \mathbf{f}_i = \mathbf{a}_i - \mathbf{d}_i
 * @f]
 * being @f$^\mathcal{P}\mathbf{a}'_i, \mathbf{d}_i@f$ known parameters.
 * @param[in] params A pointer to cable parameters.
 * @param[in] platform A pointer to the updated platform structure.
 * @param[out] cable A pointer to the cable structure including the positions to be
 * updated.
 * @note See @ref legend for symbols reference.
 */
void UpdatePosA(const CableParams* params, const PlatformVars* platform,
                CableVars* cable);

/**
 * @brief Calculate swivel pulley versors @f$\hat{\mathbf{u}}_i, \hat{\mathbf{w}}_i@f$.
 *
 * Given current swivel angle @f$\sigma_i@f$, the following versors are updated:
 * @f[
 * \hat{\mathbf{u}}_i = \hat{\mathbf{i}}_i \cos(\sigma_i) + \hat{\mathbf{j}}_i \sin(\sigma_i) \\
 * \hat{\mathbf{w}}_i = -\hat{\mathbf{i}}_i \sin(\sigma_i) + \hat{\mathbf{j}}_i \cos(\sigma_i)
 * @f]
 * being @f$\hat{\mathbf{i}}_i, \hat{\mathbf{j}}_i@f$ known parameters.
 * @param[in] params A pointer to cable parameters.
 * @param[in] swivel_ang [rad] Swivel angle @f$\sigma_i@f$.
 * @param[out] cable A pointer to the cable structure including the versors to be updated.
 * @note See @ref legend for symbols reference.
 * @note This expression results from the fact that, by definition,
 * @f$ \hat{\mathbf{u}}_i \perp \hat{\mathbf{w}}_i \perp \hat{\mathbf{k}}_i @f$.
 */
void CalcPulleyVersors(const CableParams* params, const double swivel_ang,
                       CableVars* cable);
/**
 * @brief Calculate swivel pulley versors @f$\hat{\mathbf{u}}_i, \hat{\mathbf{w}}_i@f$.
 * @param[in] params A pointer to cable parameters.
 * @param[in,out] cable A pointer to the cable structure including the versors to be
 * updated.
 * @see CalcPulleyVersors()
 */
void CalcPulleyVersors(const CableParams* params, CableVars* cable);

/**
 * @brief Calculate pulley swivel angle @f$\sigma_i@f$.
 *
 * Given current vector @f$\mathbf{f}_i@f$, the swivel angle is calculated as follows:
 * @f[
 * \sigma_i = \arctan_2(\hat{\mathbf{i}}_i \cdot \mathbf{f}_i , \hat{\mathbf{j}}_i \cdot \mathbf{f}_i)
 * @f]
 * being @f$\hat{\mathbf{i}}_i, \hat{\mathbf{j}}_i@f$ known parameters.
 * @param[in] params A pointer to cable parameters.
 * @param[in] pos_DA_glob [m] Vector @f$\mathbf{f}_i@f$.
 * @return Swivel angle @f$\sigma_i@f$ in radians.
 * @note See @ref legend for symbols reference.
 * @note This expression results from the application of the 1<sup>st</sup> kinematic
 * constraint
 * @f[ \hat{\mathbf{w}}_i \cdot \mathbf{f}_i = 0 @f]
 */
double CalcSwivelAngle(const CableParams* params, const grabnum::Vector3d& pos_DA_glob);
/**
 * @brief Calculate pulley swivel angle @f$\sigma_i@f$.
 * @param[in] params A pointer to cable parameters.
 * @param[in] cable A pointer to the cable structure.
 * @return Swivel angle @f$\sigma_i@f$ in radians.
 * @see CalcSwivelAngle()
 */
double CalcSwivelAngle(const CableParams* params, const CableVars* cable);

/**
 * @brief Calculate pulley tangent angle @f$\psi_i@f$.
 *
 * Given current vectors @f$\mathbf{f}_i, \hat{\mathbf{u}}_i@f$, the tangent angle is
 * calculated as follows:
 * @f[
 * \psi_i = 2 \arctan\left[
 *      \frac{\hat{\mathbf{k}}_i \cdot \mathbf{f}_i}{\hat{\mathbf{u}}_i \cdot \mathbf{f}_i} +
 *      \sqrt{1 - \frac{2r_i}{\hat{\mathbf{u}}_i \cdot \mathbf{f}_i} + \left(
 *      \frac{\hat{\mathbf{k}}_i \cdot \mathbf{f}_i}{\hat{\mathbf{u}}_i \cdot \mathbf{f}_i}
 *      \right)^2}\right]
 * @f]
 * being @f$\hat{\mathbf{k}}_i, r_i@f$ known parameters.
 * @param[in] params A pointer to cable parameters.
 * @param[in] vers_u Versor @f$\hat{\mathbf{u}}_i@f$.
 * @param[in] pos_DA_glob [m] Vector @f$\mathbf{f}_i@f$.
 * @return Tangent angle @f$\psi_i@f$  in radians.
 * @note See @ref legend for symbols reference.
 * @note This expression results from the application of the 2<sup>nd</sup> kinematic
 * constraint
 * @f[ \hat{\mathbf{n}}_i \cdot \boldsymbol{\rho}_i = 0 @f]
 */
double CalcTangentAngle(const CableParams* params, const grabnum::Vector3d& vers_u,
                        const grabnum::Vector3d& pos_DA_glob);
/**
 * @brief Calculate pulley tangent angle @f$\psi_i@f$.
 * @param[in] params A pointer to cable parameters.
 * @param[in] cable A pointer to the cable structure.
 * @return Tangent angle @f$\psi_i@f$  in radians.
 * @see CalcTangentAngle()
 */
double CalcTangentAngle(const CableParams* params, const CableVars* cable);

/**
 * @brief Calculate cable versors @f$\hat{\mathbf{n}}_i, \hat{\boldsymbol{\rho}}_i@f$ and
 * cable vector @f$\boldsymbol{\rho}_i@f$.
 *
 * Given current tangent angle @f$\psi_i@f$ and versor @f$\hat{\mathbf{u}}_i@f$, the
 * following versors are calculated:
 * @f[
 * \hat{\mathbf{n}}_i = \hat{\mathbf{u}}_i \cos(\psi_i) + \hat{\mathbf{k}}_i \sin(\psi_i) \\
 * \hat{\boldsymbol{\rho}}_i = -\hat{\mathbf{u}}_i \sin(\psi_i) + \hat{\mathbf{k}}_i \cos(\psi_i)
 * @f]
 * being @f$\hat{\mathbf{k}}_i@f$ a known parameter.
 * Then, from @f$ \mathbf{f}_i@f$ cable vector is calculated as:
 * @f[
 * \boldsymbol{\rho}_i = \mathbf{f}_i - r_i (\hat{\mathbf{u}}_i + \hat{\mathbf{n}}_i )
 * @f]
 * being @f$r_i@f$ a known parameter too.
 * @param[in] params A pointer to cable parameters.
 * @param[in] vers_u Versor @f$\hat{\mathbf{u}}_i@f$.
 * @param[in] pos_DA_glob [m] Vector @f$\mathbf{f}_i@f$.
 * @param[in] tan_ang [rad] Tangent angle @f$\psi_i@f$.
 * @param[out] cable A pointer to the cable structure including the variables to be
 * calculated.
 * @note See @ref legend for symbols reference.
 * @note This expressions result from the application of the 1<sup>st</sup> kinematic
 * constraint
 * @f[ \hat{\mathbf{w}}_i \cdot \mathbf{f}_i = 0 @f]
 * together with the fact that, by definition,
 * @f$ \hat{\mathbf{w}}_i \perp \hat{\boldsymbol{\rho}}_i \perp \hat{\mathbf{n}}_i @f$.
 */
void CalcCableVectors(const CableParams* params, const grabnum::Vector3d& vers_u,
                      const grabnum::Vector3d& pos_DA_glob, const double tan_ang,
                      CableVars* cable);
/**
 * @brief Calculate cable versors @f$\hat{\mathbf{n}}_i, \hat{\boldsymbol{\rho}}_i@f$ and
 * cable vector @f$\boldsymbol{\rho}_i@f$.
 * @param[in] params A pointer to cable parameters.
 * @param[in,out] cable A pointer to the cable structure including the variables to be
 * calculated.
 * @see CalcCableVectors()
 */
void CalcCableVectors(const CableParams* params, CableVars* cable);

/**
 * @brief Calculate cable length @f$l_i@f$.
 *
 * Given current vector @f$\boldsymbol{\rho}_i@f$ and tangent angle @f$\psi_i@f$, cable
 * lenght is calculated as follows:
 * @f[
 * l_i = r_i(\pi - \psi_i) + \|\boldsymbol{\rho}_i\|
 * @f]
 * being @f$r_i@f$ a known parameter.
 * @param[in] pos_BA_glob [m] Cable vector @f$\boldsymbol{\rho}_i@f$.
 * @param[in] pulley_radius [m] Swivel pulley radius @f$r_i@f$.
 * @param[in] tan_ang [rad] Tangent angle @f$\psi_i@f$.
 * @return Cable length @f$l_i@f$ in meters.
 * @note See @ref legend for symbols reference.
 * @note This expression results from the application of the 3<sup>rd</sup> kinematic
 * constraint
 * @f[ \boldsymbol{\rho}_i \cdot \boldsymbol{\rho}_i = \|\boldsymbol{\rho}_i\|^2 @f]
 */
double CalcCableLen(const grabnum::Vector3d& pos_BA_glob, const double pulley_radius,
                    const double tan_ang);
/**
 * @brief Calculate cable length @f$l_i@f$.
 * @param[in] params A pointer to cable parameters structure.
 * @param[in] cable A pointer to cable variables structure.
 * @return Cable length @f$l_i@f$ in meters.
 * @see CalcCableLen()
 */
double CalcCableLen(const CableParams* params, const CableVars* cable);

/**
 * @brief Update all zero-order variables of a single cable at once.
 * @param[in] platform A pointer to the updated platform structure.
 * @param[in] params A pointer to _i-th_ cable parameters.
 * @param[out] cable A pointer to _i-th_ cable variables structure to be updated.
 */
void UpdateCableZeroOrd(const CableParams* params, const PlatformVars* platform,
                        CableVars* cable);

/**
* @brief Update all robots zero-order variables at once (inverse kinematics problem).
* @param[in] position [m] Platform global position @f$\mathbf{p}@f$.
* @param[in] orientation [rad] Platform global orientation expressed by angles
* @f$\boldsymbol{\varepsilon}@f$.
* @param[in] params A pointer to the robot parameters structure.
* @param[out] vars A pointer to the robot variables structure to be updated.
*/
void UpdateIK0(const grabnum::Vector3d& position, const grabnum::Vector3d& orientation,
               const Params* params, Vars* vars);

/** @} */ // end of ZeroOrderKinematics group

} // end namespace grabcdpr

#endif // GRABCOMMON_LIBCDPR_KINEMATICS_H
