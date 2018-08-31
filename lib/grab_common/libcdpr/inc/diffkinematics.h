/**
 * @file diffkinematics.h
 * @author Edoardo Idà, Simone Comari
 * @date 30 Aug 2018
 * @brief File containing differential kinematics-related functions to be included in the
 * GRAB CDPR library.
 */

#ifndef GRABCOMMON_LIBCDPR_DIFFKINEMATICS_H
#define GRABCOMMON_LIBCDPR_DIFFKINEMATICS_H

#include "kinematics.h"

/**
 * @brief namespace for CDPR-related utilities, such as kinematics and dynamics.
 */
namespace grabcdpr
{

/** @defgroup FirstOrderKinematics First Order Kinematics
 * This group collects all elements related to first-order kinematics of a generic 6DoF
 * CDPR.
 * @{
 */

/**
 * @brief Update platform-related first-order quantities.
 * @param[in] velocity [m/s] Platform global linear velocity @f$\dot{\mathbf{p}}@f$.
 * @param[in] angles_speed [rad/s] Platform orientation time-derivative
 * @f$\dot{\boldsymbol{\varepsilon}}@f$.
 * @param[in] pos_PG_glob [m] Global CoG position @f$\mathbf{r}'@f$.
 * @param[out] platform A pointer to the platform structure including vars to be updated.
 */
void UpdatePlatformVel(const grabnum::Vector3d& velocity,
                       const grabnum::Vector3d& angles_speed,
                       const grabnum::Vector3d& pos_PG_glob, PlatformVars* platform);
/**
 * @brief Update platform-related first-order quantities.
 * @param[in] velocity [m/s] Platform global linear velocity @f$\dot{\mathbf{p}}@f$.
 * @param[in] angles_speed [rad/s] Platform orientation time-derivative
 * @f$\dot{\boldsymbol{\varepsilon}}@f$.
 * @param[in,out] platform A pointer to the platform structure including vars to be
 * updated.
 */
void UpdatePlatformVel(const grabnum::Vector3d& velocity,
                       const grabnum::Vector3d& angles_speed, PlatformVars* platform);

/**
 * @brief Update global velocity of point @f$A_i@f$ and relative segments.
 * @param[in] pos_PA_glob [m] See @f$\mathbf{a}'_i@f$.
 * @param[in] platform A pointer to the updated platform variables structure.
 * @param[out] cable A pointer to the cable variables structure including the velocities
 * to be
 * updated.
 */
void UpdateVelA(const grabnum::Vector3d& pos_PA_glob, const PlatformVars* platform,
                CableVars* cable);
/**
 * @brief Update global velocity of point @f$A_i@f$ and relative segments.
 * @param[in] platform A pointer to the updated platform variables structure.
 * @param[in,out] cable A pointer to the cable variables structure including the
 * velocities
 * to be
 * updated.
 */
void UpdateVelA(const PlatformVars* platform, CableVars* cable);

/**
 * @brief Calculate swivel pulley versors speed.
 * @f$\dot{\hat{\mathbf{u}}}_i, \dot{\hat{\mathbf{w}}}_i@f$.
 * @param[in] vers_u Versor @f$\hat{\mathbf{u}}_i@f$.
 * @param[in] vers_w Versor @f$\hat{\mathbf{w}}_i@f$.
 * @param[in] swivel_ang_vel [rad/s] Swivel angle speed @f$\dot{\sigma}@f$.
 * @param[out] cable A pointer to the cable structure including the versors derivatives to
 * be
 * updated.
 */
void CalcPulleyVersorsDot(const grabnum::Vector3d& vers_u,
                          const grabnum::Vector3d& vers_w, const double swivel_ang_vel,
                          CableVars* cable);
/**
 * @brief Calculate swivel pulley versors speed.
 * @f$\dot{\hat{\mathbf{u}}}_i, \dot{\hat{\mathbf{w}}}_i@f$.
 * @param[in,out] cable A pointer to the cable structure including the updated versors and
 * their derivatives to be updated.
 */
void CalcPulleyVersorsDot(CableVars* cable);

/**
 * @brief Calculate pulley swivel angle speed @f$\dot{\sigma}_i@f$.
 * @param[in] vers_u Versor @f$\hat{\mathbf{u}}_i@f$.
 * @param[in] vers_w Versor @f$\hat{\mathbf{w}}_i@f$.
 * @param[in] vel_OA_glob [m/s] See @f$\dot{\mathbf{a}}_i@f$.
 * @param[in] pos_DA_glob [m/s] See @f$\mathbf{f}_i@f$.
 * @return Swivel angle speed @f$\dot{\sigma}_i@f$ in _rad/s_.
 */
double CalcSwivelAngSpeed(const grabnum::Vector3d& vers_u,
                          const grabnum::Vector3d& vers_w,
                          const grabnum::Vector3d& vel_OA_glob,
                          const grabnum::Vector3d& pos_DA_glob);
/**
 * @brief Calculate pulley swivel angle speed @f$\dot{\sigma}_i@f$.
 * @param[in] cable A pointer to the cable variables structure.
 * @return Swivel angle speed @f$\dot{\sigma}_i@f$ in _rad/s_.
 */
double CalcSwivelAngSpeed(const CableVars* cable);

/**
 * @brief Calculate pulley tangent angle speed @f$\dot{\psi}_i@f$.
 * @param[in] vers_n Versor @f$\hat{\mathbf{n}}_i@f$.
 * @param[in] vel_OA_glob [m/s] Vector @f$\dot{\mathbf{a}}_i@f$.
 * @param[in] pos_BA_glob [m] Vector @f$\boldsymbol{\rho}_i@f$.
 * @return Tangent angle speed @f$\dot{\psi}_i@f$ in _rad/s_.
 */
double CalcTangAngSpeed(const grabnum::Vector3d& vers_n,
                        const grabnum::Vector3d& vel_OA_glob,
                        const grabnum::Vector3d& pos_BA_glob);
/**
 * @brief Calculate pulley tangent angle speed @f$\dot{\psi}_i@f$.
 * @param[in] cable A pointer to the cable variables structure.
 * @return Tangent angle speed @f$\dot{\psi}_i@f$ in _rad/s_.
 */
double CalcTangAngSpeed(const CableVars* cable);

/**
 * @brief Calculate cable versors @f$\dot{\hat{\mathbf{n}}}_i,
 * \dot{\hat{\boldsymbol{\rho}}}_i@f$.
 * @param[in] vers_w Versor @f$\hat{\mathbf{w}}_i@f$.
 * @param[in] vers_n Versor @f$\hat{\mathbf{n}}_i@f$.
 * @param[in] vers_rho Versor @f$\hat{\boldsymbol{\rho}}_i@f$.
 * @param[in] tan_ang [rad] Tangent angle @f$\psi_i@f$.
 * @param[in] tan_ang_vel [rad/s] Tangent angle speed @f$\dot{\psi}_i@f$.
 * @param[in] swivel_ang_vel [rad/s] Swivel angle speed in @f$\dot{\sigma}_i@f$.
 * @param[out] cable A pointer to the cable structure including the versors to be
 * calculated.
 */
void CalcCableVersorsDot(const grabnum::Vector3d& vers_w, const grabnum::Vector3d& vers_n,
                         const grabnum::Vector3d& vers_rho, const double tan_ang,
                         const double tan_ang_vel, const double swivel_ang_vel,
                         CableVars* cable);
/**
 * @brief Calculate cable versors @f$\dot{\hat{\mathbf{n}}}_i,
 * \dot{\hat{\boldsymbol{\rho}}}_i@f$.
 * @param[in,out] cable A pointer to the cable structure including the versors to be
 * calculated and the updated input quantities.
 */
void CalcCableVersorsDot(CableVars* cable);

/**
 * @brief Calculate cable speed @f$\dot{l}_i@f$.
 * @param[in] vers_rho Versor @f$\hat{\boldsymbol{\rho}}_i@f$.
 * @param[in] vel_OA_glob Vector @f$\dot{\mathbf{a}}_i@f$.
 * @return Cable speed @f$\dot{l}_i@f$ in _m/s_.
 */
double CalcCableSpeed(const grabnum::Vector3d& vers_rho,
                      const grabnum::Vector3d& vel_OA_glob);
/**
 * @brief Calculate cable speed @f$\dot{l}_i@f$.
 * @param[in] cable A pointer to the cable variables structure.
 * @return Cable speed @f$\dot{l}_i@f$ in _m/s_.
 */
double CalcCableSpeed(const CableVars* cable);

/**
 * @brief Update all first-order variables of a single cable at once.
 * @param[in] platform A pointer to the updated platform structure.
 * @param[in,out] cable A pointer to the cable structure with updated zero-order variables
 * and first-order variables to be updated.
 */
void UpdateCableFirstOrd(const PlatformVars* platform, CableVars* cable);

/**
* @brief Update all robots first-order variables at once (inverse kinematics problem).
 * @param[in] velocity [m/s] Platform global linear velocity.
 * @param[in] angles_speed [rad/s] Platform orientation time-derivative.
* @param[in,out] vars A pointer to the robot structure with updated zero-order variables
 * and first-order variables to be updated.
*/
void UpdateIK1(const grabnum::Vector3d& velocity, const grabnum::Vector3d& angles_speed,
               Vars* vars);

/** @} */ // end of FirstOrderKinematics group

/** @defgroup SecondOrderKinematics Second Order Kinematics
 * This group collects all elements related to second-order kinematics of a generic 6DoF
 * CDPR.
 * @{
 */

/**
 * @brief Update platform-related first-order quantities.
 * @param[in] acceleration [m/s<sup>2</sup>] Platform global linear acceleration.
 * @param[in] angles_acc [rad/s] Platform orientation time-derivative.
 * @param[in] pos_PG_glob [m] Global CoG position @f$\mathbf{r}'@f$.
 * @param[out] platform A pointer to the platform structure including vars to be updated.
 */
void UpdatePlatformAcc(const grabnum::Vector3d& acceleration,
                       const grabnum::Vector3d& angles_acc,
                       const grabnum::Vector3d& pos_PG_glob, PlatformVars* platform);
/**
 * @brief Update platform-related first-order quantities.
 * @param[in] acceleration [m/s<sup>2</sup>] Platform global linear acceleration.
 * @param[in] angles_acc [rad/s] Platform orientation time-derivative.
 * @param[in,out] platform A pointer to the platform structure including vars to be
 * updated.
 */
void UpdatePlatformAcc(const grabnum::Vector3d& acceleration,
                       const grabnum::Vector3d& angles_acc, PlatformVars* platform);

/**
 * @brief Update global velocity of point @f$A_i@f$ and relative segments.
 * @param[in] pos_PA_glob [m] See @f$\mathbf{a}'_i@f$.
 * @param[in] platform A pointer to the updated platform structure.
 * @param[out] cable A pointer to the cable structure including the accelerations to be
 * updated.
 */
void UpdateAccA(const grabnum::Vector3d& pos_PA_glob, const PlatformVars* platform,
                CableVars* cable);
/**
 * @brief Update global velocity of point @f$A_i@f$ and relative segments.
 * @param[in] platform A pointer to the updated platform structure.
 * @param[in,out] cable A pointer to the cable structure including the accelerations to be
 * updated.
 */
void UpdateAccA(const PlatformVars* platform, CableVars* cable);

/**
 * @brief Calculate pulley swivel angle acceleration @f$\ddot{\sigma}_i@f$.
 * @param[in] vers_u Versor @f$\hat{\mathbf{u}}_i@f$.
 * @param[in] vers_w Versor @f$\hat{\mathbf{w}}_i@f$.
 * @param[in] vel_OA_glob [m/s] See @f$\dot{\mathbf{a}}_i@f$.
 * @param[in] pos_DA_glob [m] See @f$\mathbf{f}_i@f$.
 * @param[in] acc_OA_glob [m/s<sup>2</sup>] See @f$\ddot{\mathbf{a}}_i@f$.
 * @param[in] swivel_ang_vel [rad/s] See @f$\dot{\sigma}_i@f$.
 * @return Swivel angle acceleration @f$\ddot{\sigma}_i@f$ in _rad/s<sup>2</sup>_.
 */
double CalcSwivelAngAcc(const grabnum::Vector3d& vers_u, const grabnum::Vector3d& vers_w,
                        const grabnum::Vector3d& vel_OA_glob,
                        const grabnum::Vector3d& pos_DA_glob,
                        const grabnum::Vector3d& acc_OA_glob,
                        const double swivel_ang_vel);
/**
 * @brief Calculate pulley swivel angle acceleration @f$\ddot{\sigma}_i@f$.
 * @param[in] cable A pointer to the updated cable variables structure.
 * @return Swivel angle acceleration @f$\ddot{\sigma}_i@f$ in _rad/s<sup>2</sup>_.
 */
double CalcSwivelAngAcc(const CableVars* cable);

/**
 * @brief Calculate pulley tangent angle acceleration @f$\ddot{\psi}_i@f$.
 * @param[in] pulley_radius [m] Swivel pulley radius @f$r_i@f$.
 * @param[in] vers_u Versor @f$\hat{\mathbf{u}}_i@f$.
 * @param[in] vers_n Versor @f$\hat{\mathbf{n}}_i@f$.
 * @param[in] pos_DA_glob [m] See @f$\mathbf{f}_i@f$.
 * @param[in] pos_BA_glob [m] See @f$\boldsymbol{\rho}_i@f$.
 * @param[in] acc_OA_glob [m/s<sup>2</sup>] See @f$\ddot{\mathbf{a}}_i@f$.
 * @param[in] speed [m/s] Cable speed @f$\dot{l}_i@f$.
 * @param[in] tan_ang [rad] Tangent angle @f$\psi_i@f$.
 * @param[in] tan_ang_vel [rad/s] Tangent angle speed @f$\dot{\psi}_i@f$.
 * @param[in] swivel_ang_vel [rad/s] See @f$\dot{\sigma}_i@f$.
 * @return Tangent angle acceleration @f$\ddot{\psi}_i@f$ in _rad/s<sup>2</sup>_.
 */
double CalcTangAngAcc(const double pulley_radius, const grabnum::Vector3d& vers_u,
                      const grabnum::Vector3d& vers_n,
                      const grabnum::Vector3d& pos_DA_glob,
                      const grabnum::Vector3d& pos_BA_glob,
                      const grabnum::Vector3d& acc_OA_glob, const double speed,
                      const double tan_ang, const double tan_ang_vel,
                      const double swivel_ang_vel);
/**
 * @brief Calculate pulley tangent angle acceleration @f$\ddot{\psi}_i@f$.
 * @param[in] pulley_radius [m] Swivel pulley radius @f$r_i@f$.
 * @param[in] cable A pointer to the updated cable variables structure.
 * @return Tangent angle acceleration @f$\ddot{\psi}_i@f$ in _rad/s<sup>2</sup>_.
 */
double CalcTangAngAcc(const double pulley_radius, const CableVars* cable);

/**
 * @brief Calculate cable acceleration.
 * @param[in] vers_u Versor @f$\hat{\mathbf{u}}_i@f$.
 * @param[in] vers_rho Versor @f$\hat{\boldsymbol{\rho}_i}_i@f$.
 * @param[in] pos_DA_glob [m] See @f$\mathbf{f}_i@f$.
 * @param[in] pos_BA_glob [m] See @f$\boldsymbol{\rho}_i@f$.
 * @param[in] acc_OA_glob [m/s<sup>2</sup>] See @f$\ddot{\mathbf{a}}_i@f$.
 * @param[in] tan_ang [rad] Tangent angle @f$\psi_i@f$.
 * @param[in] tan_ang_vel [rad/s] Tangent angle speed @f$\dot{\psi}_i@f$.
 * @param[in] swivel_ang_vel [rad/s] See @f$\dot{\sigma}_i@f$.
 * @return Cable acceleration in _m/s<sup>2</sup>_.
 */
double CalcCableAcc(const grabnum::Vector3d& vers_u, const grabnum::Vector3d& vers_rho,
                    const grabnum::Vector3d& pos_DA_glob,
                    const grabnum::Vector3d& pos_BA_glob,
                    const grabnum::Vector3d& acc_OA_glob, const double tan_ang,
                    const double tan_ang_vel, const double swivel_ang_vel);
/**
 * @brief Calculate cable acceleration.
 * @param[in] cable A pointer to the updated cable variables structure.
 * @return Cable acceleration in _m/s<sup>2</sup>_.
 */
double CalcCableAcc(const CableVars* cable);

/**
 * @brief Update all second-order variables of a single cable at once.
 * @param[in] params A pointer to the cable parameters structure.
 * @param[in] platform A pointer to the updated platform structure.
 * @param[in,out] cable A pointer to the cable structure with updated zero and first-order
 * variables and second-order variables to be updated.
 */
void UpdateCableSecondOrd(const CableParams* params, const PlatformVars* platform,
                          CableVars* cable);

/**
* @brief Update all robots second-order variables at once (inverse kinematics problem).
* @param[in] acceleration [m/s<sup>2</sup>] Platform global linear acceleration.
* @param[in] angles_acc [rad/s] Platform orientation time-derivative.
* @param[in] params A pointer to the robot parameters structure.
* @param[in,out] vars A pointer to the robot structure with updated zero and first-order
 * variables and second-order variables to be updated.
*/
void UpdateIK2(const grabnum::Vector3d& acceleration, const grabnum::Vector3d& angles_acc,
               const Params* params, Vars* vars);

/** @} */ // end of SecondOrderKinematics group

/**
* @brief Update all robots variables at once (full inverse kinematics problem).
* @param[in] position Platform global position.
* @param[in] orientation Platform global orientation expressed by angles.
* @param[in] velocity [m/s] Platform global linear velocity.
* @param[in] angles_speed [rad/s] Platform orientation time-derivative.
* @param[in] acceleration [m/s<sup>2</sup>] Platform global linear acceleration.
* @param[in] angles_acc [rad/s] Platform orientation time-derivative.
* @param[in] params A pointer to the robot parameters structure.
* @param[out] vars A pointer to the robot structure to be updated.
*/
void UpdateIK(const grabnum::Vector3d& position, const grabnum::Vector3d& orientation,
              const grabnum::Vector3d& velocity, const grabnum::Vector3d& angles_speed,
              const grabnum::Vector3d& acceleration, const grabnum::Vector3d& angles_acc,
              const Params* params, Vars* vars);

} // end namespace grabcdpr

#endif // GRABCOMMON_LIBCDPR_DIFFKINEMATICS_H
