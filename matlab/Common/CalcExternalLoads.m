function platform_v = CalcExternalLoads(platform_v, platform_p, R)
%CALCEXTERNALLOADS computes the external loads acting on the platform.
%
%   CALCEXTERNALLOADS computes the components of the external loads in terms of
%   external forces and moments acting on the platform.
%
%   PLATFORM_P is a structure containing static parameters of the platform.
%   R_MAT is the rotation matrix (size[3,3]).
%   POS_PG_GLOB is a vector(size[3,1], [m]), containing the components of
%   the position vector (G-P), projected on the global frame.
%   R is a matrix (size[6,6]), that premultiplies the equation of dynamic
%   equilibrium in order to make the mass matrix symmetric.
%
%   EXT_LOAD is a vector([6,1]), containing the components of external
%   forces and moments, projected on the global frame.

platform_v.ext_load(1:3, 1) = R * (platform_p.mass .* ...
  platform_p.gravity_acceleration + platform_v.rot_mat * ...
  platform_p.ext_force_loc);
platform_v.ext_load(4:6, 1) = Anti(platform_v.pos_PG_glob) * ...
  platform_v.ext_load(1:3, 1) + platform_v.rot_mat * platform_p.ext_moments_loc;

end