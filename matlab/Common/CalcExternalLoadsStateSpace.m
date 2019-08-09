function platform_v = CalcExternalLoadsStateSpace(platform_v, platform_p, R)
%CALCEXTERNALLOADSSTATESPACE computes the external loads acting on the platform
%projected onto the state space.
%
%   CALCEXTERNALLOADS computes the components of the external loads in terms of
%   external forces and moments acting on the platform, projected onto the state space.
%
%   PLATFORM_P is a structure containing static parameters of the platform.
%   R_MAT is the rotation matrix (size[3,3]).
%   H_MAT is the matrix (size[3,3]) that transforms the 1th order time
%   derivatives of the angles of rotation into the components of the
%   angular velocity vector.
%   POS_PG_GLOB is a vector(size[3,1], [m]), containing the components of
%   the position vector (G-P), projected on the global frame.
%   R is a matrix (size[6,6]), that premultiplies the equation of dynamic
%   equilibrium in order to make the mass matrix symmetric.
%
%   EXT_LOAD is a vector([6,1]), containing the components of external
%   forces and moments, projected on the global frame.

platform_v = CalcExternalLoads(platform_v, platform_p, R);
platform_v.ext_load_ss(1:3, 1) = platform_v.ext_load(1:3, 1);
platform_v.ext_load_ss(4:6, 1) = platform_v.H_mat' * platform_v.ext_load(4:6, 1);

end