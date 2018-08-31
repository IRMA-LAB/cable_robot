/**
 * @file kinematics.h
 * @author Edoardo Idà, Simone Comari
 * @date 31 Aug 2018
 * @brief File containing kinematics-related functions to be included in the GRAB CDPR
 * library.
 */

#ifndef GRABCOMMON_LIBCDPR_KINEMATICS_H
#define GRABCOMMON_LIBCDPR_KINEMATICS_H

#include "types.h"
#include "rotations.h"
#include "matrix.h"

/**
 * @brief namespace for CDPR-related utilities, such as kinematics and dynamics.
 */
namespace grabcdpr
{
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
