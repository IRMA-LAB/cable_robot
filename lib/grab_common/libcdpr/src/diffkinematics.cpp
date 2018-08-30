/**
 * @file diffkinematics.cpp
 * @author Edoardo Idà, Simone Comari
 * @date 30 Aug 2018
 * @brief File containing definitions of functions declared in diffkinematics.h.
 */

#include "diffkinematics.h"

#ifndef SQUARE
#define SQUARE(x) (x * x) /**< returns the square of an element*/
#endif

namespace grabcdpr
{

///////////////////////////////////////////////////////////////////////////////
/// Functions first-order kinematics
///////////////////////////////////////////////////////////////////////////////

void UpdatePlatformVel(const grabnum::Vector3d& velocity,
                       const grabnum::Vector3d& angles_speed,
                       const RotParametrization angles_type,
                       const grabnum::Vector3d& pos_PG_glob, PlatformVars* platform)
{
  // Update platform velocities.
  platform->UpdateVel(velocity, angles_speed, angles_type);
  // Calculate platform baricenter velocity expressed in global frame.
  platform->vel_OG_glob =
    platform->velocity + grabnum::Cross(platform->angular_vel, pos_PG_glob);
}

void UpdateVelA(const grabnum::Vector3d& pos_PA_glob, const PlatformVars* platform,
                CableVars* cable)
{
  cable->vel_OA_glob =
    platform->velocity + grabnum::Dot(platform->angular_vel, pos_PA_glob);
}

void CalcPulleyVersorsDot(const double swivel_ang_dot, CableVars* cable)
{
  cable->vers_u_dot = cable->vers_w * swivel_ang_dot;
  cable->vers_w_dot = -cable->vers_u * swivel_ang_dot;
}

double CalcSwivelAngSpeed(const CableVars* cable)
{
  return grabnum::Dot(cable->vers_w, cable->vel_OA_glob) /
         grabnum::Dot(cable->vers_u, cable->pos_DA_glob);
}

double CalcTangAngSpeed(const grabnum::Vector3d& vers_n,
                        const grabnum::Vector3d& vel_OA_glob,
                        const grabnum::Vector3d& pos_BA_glob)
{
  return grabnum::Dot(vers_n, vel_OA_glob) / grabnum::Norm(pos_BA_glob);
}

void CalcCableVersorsDot(const grabnum::Vector3d& vers_w, const grabnum::Vector3d& vers_n,
                         const grabnum::Vector3d& vers_rho, const double tan_ang,
                         const double tan_ang_dot, const double swivel_ang_dot,
                         CableVars* cable)
{
  cable->vers_n_dot = vers_w * cos(tan_ang) * tan_ang_dot - vers_rho * tan_ang_dot;
  cable->vers_n_dot = vers_w * sin(tan_ang) * swivel_ang_dot + vers_n * tan_ang_dot;
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

void UpdateCableFirstOrd(const PlatformVars* platform, CableVars* cable)
{
  UpdateVelA(cable->pos_PA_glob, platform, cable);
  cable->tan_ang_vel =
    CalcTangAngSpeed(cable->vers_n, cable->vel_OA_glob, cable->pos_BA_glob);
  cable->swivel_ang_vel = CalcSwivelAngSpeed(cable);
  CalcCableVersorsDot(cable);
  cable->speed = CalcCableSpeed(cable->vers_rho, cable->vel_OA_glob);
}

void UpdateIK1(const grabnum::Vector3d& velocity, const grabnum::Vector3d& angles_speed,
               const RotParametrization angles_type, Vars* vars)
{
  UpdatePlatformVel(velocity, angles_speed, angles_type, vars->platform->pos_PG_glob,
                    vars->platform);
  for (uint8_t i = 0; i < vars->cables.size(); ++i)
    UpdateCableFirstOrd(vars->platform, &(vars->cables[i]));
}

///////////////////////////////////////////////////////////////////////////////
/// Functions second-order kinematics
///////////////////////////////////////////////////////////////////////////////

void UpdatePlatformAcc(const grabnum::Vector3d& acceleration,
                       const grabnum::Vector3d& angles_acc,
                       const RotParametrization angles_type,
                       const grabnum::Vector3d& pos_PG_glob, PlatformVars* platform)
{
  // Update platform velocities.
  platform->UpdateAcc(acceleration, angles_acc, angles_type);
  // Calculate platform baricenter velocity expressed in global frame.
  grabnum::Matrix3d Omega = grabnum::Anti(platform->angular_vel);
  platform->acc_OG_glob =
    acceleration + (grabnum::Anti(platform->angular_acc) + Omega * Omega) * pos_PG_glob;
}

void UpdateAccA(const grabnum::Vector3d& pos_PA_glob, const PlatformVars* platform,
                CableVars* cable)
{
  grabnum::Matrix3d Omega = grabnum::Anti(platform->angular_vel);
  cable->acc_OA_glob =
    platform->acceleration +
    (grabnum::Anti(platform->angular_acc) + Omega * Omega) * pos_PA_glob;
}

double CalcSwivelAngAcc(const CableVars* cable)
{
  return (grabnum::Dot(cable->vers_w, cable->acc_OA_glob) -
          2. * grabnum::Dot(cable->vers_u, cable->vel_OA_glob) * cable->swivel_ang_vel) /
         grabnum::Dot(cable->vers_u, cable->pos_DA_glob);
}

double CalcTangAngAcc(const double pulley_radius, const CableVars* cable)
{
  return (grabnum::Dot(cable->vers_n, cable->acc_OA_glob) +
          grabnum::Dot(cable->vers_u, cable->pos_DA_glob) * cos(cable->tan_ang) *
            cable->swivel_ang_vel * cable->swivel_ang_vel -
          (2. * cable->speed + pulley_radius * cable->tan_ang_vel) * cable->tan_ang_vel) /
         grabnum::Norm(cable->pos_BA_glob);
}

double CalcCableAcc(const CableVars* cable)
{
  return grabnum::Dot(cable->vers_u, cable->pos_DA_glob) * sin(cable->tan_ang) *
           SQUARE(cable->swivel_ang_vel) +
         grabnum::Norm(cable->pos_BA_glob) * SQUARE(cable->tan_ang_vel) +
         grabnum::Dot(cable->vers_rho, cable->acc_OA_glob);
}

void UpdateCableSecondOrd(const CableParams* params, const PlatformVars* platform,
                          CableVars* cable)
{
  UpdateAccA(cable->pos_PA_glob, platform, cable);
  cable->tan_ang_acc = CalcTangAngAcc(params->swivel_pulley_r, cable);
  cable->swivel_ang_acc = CalcSwivelAngAcc(cable);
  cable->acceleration = CalcCableAcc(cable);
}

void UpdateIK2(const grabnum::Vector3d& acceleration, const grabnum::Vector3d& angles_acc,
               const RotParametrization angles_type, const Params* params, Vars* vars)
{
  UpdatePlatformAcc(acceleration, angles_acc, angles_type, vars->platform->pos_PG_glob,
                    vars->platform);
  for (uint8_t i = 0; i < vars->cables.size(); ++i)
    UpdateCableSecondOrd(&(params->cables[i]), vars->platform, &(vars->cables[i]));
}

void UpdateIK(const grabnum::Vector3d& position, const grabnum::Vector3d& orientation,
              const grabnum::Vector3d& velocity, const grabnum::Vector3d& angles_speed,
              const grabnum::Vector3d& acceleration, const grabnum::Vector3d& angles_acc,
              const RotParametrization angles_type, const Params* params, Vars* vars)
{
  UpdatePlatformPose(position, orientation, angles_type, params->platform->pos_PG_loc,
                     vars->platform);
  UpdatePlatformVel(velocity, angles_speed, angles_type, vars->platform->pos_PG_glob,
                    vars->platform);
  UpdatePlatformAcc(acceleration, angles_acc, angles_type, vars->platform->pos_PG_glob,
                    vars->platform);
  for (uint8_t i = 0; i < vars->cables.size(); ++i)
  {
    UpdateCableZeroOrd(&(params->cables[i]), vars->platform, &(vars->cables[i]));
    UpdateCableFirstOrd(vars->platform, &(vars->cables[i]));
    UpdateCableSecondOrd(&(params->cables[i]), vars->platform, &(vars->cables[i]));
  }
}

} // end namespace grabcdpr
