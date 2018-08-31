/**
 * @file kinematics.cpp
 * @author Edoardo Idà, Simone Comari
 * @date 28 Aug 2018
 * @brief File containing definitions of functions declared in kinematics.h.
 */

#include "kinematics.h"

namespace grabcdpr
{

void UpdatePlatformPose(const grabnum::Vector3d& position,
                        const grabnum::Vector3d& orientation,
                        const grabnum::Vector3d& pos_PG_loc, PlatformVars* platform)
{
  // Update platform pose.
  platform->UpdatePose(position, orientation);
  // Calculate platform baricenter positions expressed in global frame.
  platform->pos_PG_glob = platform->rot_mat * pos_PG_loc;
  platform->pos_OG_glob = platform->position + platform->pos_PG_glob;
}

void UpdatePlatformPose(const grabnum::Vector3d& position,
                        const grabnum::Vector3d& orientation,
                        const PlatformParams* params, PlatformVars* platform)
{
  UpdatePlatformPose(position, orientation, params->pos_PG_loc, platform);
}

void UpdatePosA(const CableParams* params, const PlatformVars* platform, CableVars* cable)
{
  cable->pos_PA_glob = platform->rot_mat * params->pos_PA_loc;
  cable->pos_OA_glob = platform->position + cable->pos_PA_glob;
  cable->pos_DA_glob = cable->pos_OA_glob - params->pos_OD_glob;
}

void CalcPulleyVersors(const CableParams* params, const double swivel_ang,
                       CableVars* cable)
{
  double cos_sigma = cos(swivel_ang);
  double sin_sigma = sin(swivel_ang);
  cable->vers_u = params->vers_i * cos_sigma + params->vers_j * sin_sigma;
  cable->vers_w = -params->vers_i * sin_sigma + params->vers_j * cos_sigma;
}

void CalcPulleyVersors(const CableParams* params, CableVars* cable)
{
  CalcPulleyVersors(params, cable->swivel_ang, cable);
}

double CalcSwivelAngle(const CableParams* params, const grabnum::Vector3d& pos_DA_glob)
{
  return atan2(grabnum::Dot(params->vers_j, pos_DA_glob),
               grabnum::Dot(params->vers_i, pos_DA_glob));
}

double CalcSwivelAngle(const CableParams* params, const CableVars* cable)
{
  return CalcSwivelAngle(params, cable->pos_DA_glob);
}

double CalcTangentAngle(const CableParams* params, const grabnum::Vector3d& vers_u,
                        const grabnum::Vector3d& pos_DA_glob)
{
  double app_var =
    grabnum::Dot(params->vers_k, pos_DA_glob) / grabnum::Dot(vers_u, pos_DA_glob);
  double psi =
    2. * atan(app_var +
              sqrt(1. - 2. * params->swivel_pulley_r / grabnum::Dot(vers_u, pos_DA_glob) +
                   app_var * app_var));
  return psi;
}

double CalcTangentAngle(const CableParams* params, const CableVars* cable)
{
  return CalcTangentAngle(params, cable->vers_u, cable->pos_DA_glob);
}

void CalcCableVectors(const CableParams* params, const grabnum::Vector3d& vers_u,
                      const grabnum::Vector3d& pos_DA_glob, const double tan_ang,
                      CableVars* cable)
{
  // Versors describing cable exit direction from swivel pulley.
  double cos_psi = cos(tan_ang);
  double sin_psi = sin(tan_ang);
  cable->vers_n = vers_u * cos_psi + params->vers_k * sin_psi;
  cable->vers_rho = vers_u * sin_psi - params->vers_k * cos_psi;
  // Vector from swivel pulley exit point to platform attaching point.
  cable->pos_BA_glob = pos_DA_glob - params->swivel_pulley_r * (vers_u + cable->vers_n);
}

void CalcCableVectors(const CableParams* params, CableVars* cable)
{
  CalcCableVectors(params, cable->vers_u, cable->pos_DA_glob, cable->tan_ang, cable);
}

double CalcCableLen(const grabnum::Vector3d& pos_BA_glob, const double pulley_radius,
                    const double tan_ang)
{
  return pulley_radius * (M_PI - tan_ang) + grabnum::Norm(pos_BA_glob);
}

double CalcCableLen(const CableParams* params, const CableVars* cable)
{
  return CalcCableLen(cable->pos_BA_glob, params->swivel_pulley_r, cable->tan_ang);
}

void UpdateCableZeroOrd(const CableParams* params, const PlatformVars* platform,
                        CableVars* cable)
{  
  UpdatePosA(params, platform, cable);             // update segments ending with point A_i.
  cable->swivel_ang = CalcSwivelAngle(params, cable);  // from 1st kinematic constraint.
  CalcPulleyVersors(params, cable);
  cable->tan_ang = CalcTangentAngle(params, cable);    // from 2nd kinematic constraint.
  CalcCableVectors(params, cable);                                  // from 1st kinematic constraint.
  cable->length = CalcCableLen(params, cable);             // from 3rd kinematic constraint.
}

void UpdateIK0(const grabnum::Vector3d& position, const grabnum::Vector3d& orientation,
               const Params* params, Vars* vars)
{
  UpdatePlatformPose(position, orientation, params->platform, vars->platform);
  for (uint8_t i = 0; i < vars->cables.size(); ++i)
    UpdateCableZeroOrd(&(params->cables[i]), vars->platform, &(vars->cables[i]));
}

} // end namespace grabcdpr
