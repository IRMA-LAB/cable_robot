/**
 * @file diffkinematics.cpp
 * @author Edoardo Idà, Simone Comari
 * @date 05 Sep 2018
 * @brief File containing definitions of functions declared in diffkinematics.h.
 */

#include "diffkinematics.h"

namespace grabcdpr
{

///////////////////////////////////////////////////////////////////////////////
//// Functions first-order kinematics
///////////////////////////////////////////////////////////////////////////////

template <class OrientationType, class PlatformVarsType>
void UpdatePlatformVel(const grabnum::Vector3d& velocity,
                       const OrientationType& orientation_dot,
                       const grabnum::Vector3d& pos_PG_glob, PlatformVarsType* platform)
{
  // Update platform velocities.
  platform->UpdateVel(velocity, orientation_dot);
  // Calculate platform baricenter velocity expressed in global frame.
  platform->vel_OG_glob =
    platform->velocity + grabnum::Cross(platform->angular_vel, pos_PG_glob);
}

template <class OrientationType, class PlatformVarsType>
void UpdatePlatformVel(const grabnum::Vector3d& velocity,
                       const OrientationType& orientation_dot, PlatformVarsType* platform)
{
  UpdatePlatformVel(velocity, orientation_dot, platform->pos_PG_glob, platform);
}

template <class PlatformVarsType>
void UpdateVelA(const grabnum::Vector3d& pos_PA_glob, const PlatformVarsType* platform,
                CableVars* cable)
{
  cable->vel_OA_glob =
    platform->velocity + grabnum::Dot(platform->angular_vel, pos_PA_glob);
}

template <class PlatformVarsType>
void UpdateVelA(const PlatformVarsType* platform, CableVars* cable)
{
  UpdateVelA(cable->pos_PA_glob, platform, cable);
}

void CalcPulleyVersorsDot(const grabnum::Vector3d& vers_u,
                          const grabnum::Vector3d& vers_w, const double swivel_ang_vel,
                          CableVars* cable)
{
  cable->vers_u_dot = vers_w * swivel_ang_vel;
  cable->vers_w_dot = -vers_u * swivel_ang_vel;
}

void CalcPulleyVersorsDot(CableVars* cable)
{
  CalcPulleyVersorsDot(cable->vers_u, cable->vers_w, cable->swivel_ang_vel, cable);
}

double CalcSwivelAngSpeed(const grabnum::Vector3d& vers_u,
                          const grabnum::Vector3d& vers_w,
                          const grabnum::Vector3d& vel_OA_glob,
                          const grabnum::Vector3d& pos_DA_glob)
{
  return grabnum::Dot(vers_w, vel_OA_glob) / grabnum::Dot(vers_u, pos_DA_glob);
}

double CalcSwivelAngSpeed(const CableVars* cable)
{
  return CalcSwivelAngSpeed(cable->vers_u, cable->vers_w, cable->vel_OA_glob,
                            cable->pos_DA_glob);
}

double CalcTangAngSpeed(const grabnum::Vector3d& vers_n,
                        const grabnum::Vector3d& vel_OA_glob,
                        const grabnum::Vector3d& pos_BA_glob)
{
  return grabnum::Dot(vers_n, vel_OA_glob) / grabnum::Norm(pos_BA_glob);
}

double CalcTangAngSpeed(const CableVars* cable)
{
  return CalcTangAngSpeed(cable->vers_n, cable->vel_OA_glob, cable->pos_BA_glob);
}

void CalcCableVersorsDot(const grabnum::Vector3d& vers_w, const grabnum::Vector3d& vers_n,
                         const grabnum::Vector3d& vers_rho, const double tan_ang,
                         const double tan_ang_vel, const double swivel_ang_vel,
                         CableVars* cable)
{
  cable->vers_n_dot = vers_w * cos(tan_ang) * tan_ang_vel - vers_rho * tan_ang_vel;
  cable->vers_n_dot = vers_w * sin(tan_ang) * swivel_ang_vel + vers_n * tan_ang_vel;
}

void CalcCableVersorsDot(CableVars* cable)
{
  CalcCableVersorsDot(cable->vers_w, cable->vers_n, cable->vers_rho, cable->tan_ang,
                      cable->tan_ang_vel, cable->swivel_ang_vel, cable);
}

double CalcCableSpeed(const grabnum::Vector3d& vers_rho,
                      const grabnum::Vector3d& vel_OA_glob)
{
  return grabnum::Dot(vers_rho, vel_OA_glob);
}

double CalcCableSpeed(const CableVars* cable)
{
  return CalcCableSpeed(cable->vers_rho, cable->vel_OA_glob);
}

template <class PlatformVarsType>
void UpdateCableFirstOrd(const PlatformVarsType* platform, CableVars* cable)
{
  UpdateVelA(platform, cable);
  cable->swivel_ang_vel = CalcSwivelAngSpeed(cable);
  CalcPulleyVersorsDot(cable);
  cable->tan_ang_vel = CalcTangAngSpeed(cable);
  CalcCableVersorsDot(cable);
  cable->speed = CalcCableSpeed(cable);
}

template <class OrientationType, class VarsType>
void UpdateIK1(const grabnum::Vector3d& velocity, const OrientationType& orientation_dot,
               VarsType* vars)
{
  UpdatePlatformVel(velocity, orientation_dot, vars->platform);
  for (uint8_t i = 0; i < vars->cables.size(); ++i)
    UpdateCableFirstOrd(vars->platform, &(vars->cables[i]));
}

///////////////////////////////////////////////////////////////////////////////
//// Functions second-order kinematics
///////////////////////////////////////////////////////////////////////////////

template <class OrientationType, class PlatformVarsType>
void UpdatePlatformAcc(const grabnum::Vector3d& acceleration,
                       const OrientationType& orientation_ddot,
                       const grabnum::Vector3d& pos_PG_glob, PlatformVarsType* platform)
{
  // Update platform velocities.
  platform->UpdateAcc(acceleration, orientation_ddot);
  // Calculate platform baricenter velocity expressed in global frame.
  grabnum::Matrix3d Omega = grabnum::Anti(platform->angular_vel);
  platform->acc_OG_glob =
    acceleration + (grabnum::Anti(platform->angular_acc) + Omega * Omega) * pos_PG_glob;
}

template <class OrientationType, class PlatformVarsType>
void UpdatePlatformAcc(const grabnum::Vector3d& acceleration,
                       const OrientationType& orientation_ddot,
                       PlatformVarsType* platform)
{
  UpdatePlatformAcc(acceleration, orientation_ddot, platform->pos_PG_glob, platform);
}

template <class PlatformVarsType>
void UpdateAccA(const grabnum::Vector3d& pos_PA_glob, const PlatformVarsType* platform,
                CableVars* cable)
{
  grabnum::Matrix3d Omega = grabnum::Anti(platform->angular_vel);
  cable->acc_OA_glob =
    platform->acceleration +
    (grabnum::Anti(platform->angular_acc) + Omega * Omega) * pos_PA_glob;
}

template <class PlatformVarsType>
void UpdateAccA(const PlatformVarsType* platform, CableVars* cable)
{
  UpdateAccA(cable->pos_PA_glob, platform, cable);
}

double CalcSwivelAngAcc(const grabnum::Vector3d& vers_u, const grabnum::Vector3d& vers_w,
                        const grabnum::Vector3d& vel_OA_glob,
                        const grabnum::Vector3d& pos_DA_glob,
                        const grabnum::Vector3d& acc_OA_glob, const double swivel_ang_vel)
{
  return (grabnum::Dot(vers_w, acc_OA_glob) -
          2. * grabnum::Dot(vers_u, vel_OA_glob) * swivel_ang_vel) /
         grabnum::Dot(vers_u, pos_DA_glob);
}

double CalcSwivelAngAcc(const CableVars* cable)
{
  return CalcSwivelAngAcc(cable->vers_u, cable->vers_w, cable->vel_OA_glob,
                          cable->pos_DA_glob, cable->acc_OA_glob, cable->swivel_ang_vel);
}

double CalcTangAngAcc(const double pulley_radius, const grabnum::Vector3d& vers_u,
                      const grabnum::Vector3d& vers_n,
                      const grabnum::Vector3d& pos_DA_glob,
                      const grabnum::Vector3d& pos_BA_glob,
                      const grabnum::Vector3d& acc_OA_glob, const double speed,
                      const double tan_ang, const double tan_ang_vel,
                      const double swivel_ang_vel)
{
  return (grabnum::Dot(vers_n, acc_OA_glob) +
          grabnum::Dot(vers_u, pos_DA_glob) * cos(tan_ang) * swivel_ang_vel *
            swivel_ang_vel -
          (2. * speed + pulley_radius * tan_ang_vel) * tan_ang_vel) /
         grabnum::Norm(pos_BA_glob);
}

double CalcTangAngAcc(const double pulley_radius, const CableVars* cable)
{
  return CalcTangAngAcc(pulley_radius, cable->vers_u, cable->vers_n, cable->pos_DA_glob,
                        cable->pos_BA_glob, cable->acc_OA_glob, cable->speed,
                        cable->tan_ang, cable->tan_ang_vel, cable->swivel_ang_vel);
}

double CalcCableAcc(const grabnum::Vector3d& vers_u, const grabnum::Vector3d& vers_rho,
                    const grabnum::Vector3d& pos_DA_glob,
                    const grabnum::Vector3d& pos_BA_glob,
                    const grabnum::Vector3d& acc_OA_glob, const double tan_ang,
                    const double tan_ang_vel, const double swivel_ang_vel)
{
  return grabnum::Dot(vers_u, pos_DA_glob) * sin(tan_ang) * SQUARE(swivel_ang_vel) +
         grabnum::Norm(pos_BA_glob) * SQUARE(tan_ang_vel) +
         grabnum::Dot(vers_rho, acc_OA_glob);
}

double CalcCableAcc(const CableVars* cable)
{
  return CalcCableAcc(cable->vers_u, cable->vers_rho, cable->pos_DA_glob,
                      cable->pos_BA_glob, cable->acc_OA_glob, cable->tan_ang,
                      cable->tan_ang_vel, cable->swivel_ang_vel);
}

template <class PlatformVarsType>
void UpdateCableSecondOrd(const CableParams* params, const PlatformVarsType* platform,
                          CableVars* cable)
{
  UpdateAccA(cable->pos_PA_glob, platform, cable);
  cable->tan_ang_acc = CalcTangAngAcc(params->swivel_pulley_r, cable);
  cable->swivel_ang_acc = CalcSwivelAngAcc(cable);
  cable->acceleration = CalcCableAcc(cable);
}

template <class OrientationType, class VarsType>
void UpdateIK2(const grabnum::Vector3d& acceleration,
               const OrientationType& orientation_ddot, const Params* params,
               VarsType* vars)
{
  UpdatePlatformAcc(acceleration, orientation_ddot, vars->platform);
  for (uint8_t i = 0; i < vars->cables.size(); ++i)
    UpdateCableSecondOrd(&(params->cables[i]), vars->platform, &(vars->cables[i]));
}

template <class OrientationType, class VarsType>
void UpdateIK(const grabnum::Vector3d& position, const OrientationType& orientation,
              const grabnum::Vector3d& velocity, const OrientationType& orientation_dot,
              const grabnum::Vector3d& acceleration,
              const OrientationType& orientation_ddot, const Params* params,
              VarsType* vars)
{
  UpdatePlatformPose(position, orientation, params->platform, vars->platform);
  UpdatePlatformVel(velocity, orientation_dot, vars->platform);
  UpdatePlatformAcc(acceleration, orientation_ddot, vars->platform);
  for (uint8_t i = 0; i < vars->cables.size(); ++i)
  {
    UpdateCableZeroOrd(&(params->cables[i]), vars->platform, &(vars->cables[i]));
    UpdateCableFirstOrd(vars->platform, &(vars->cables[i]));
    UpdateCableSecondOrd(&(params->cables[i]), vars->platform, &(vars->cables[i]));
  }
}

} // end namespace grabcdpr
