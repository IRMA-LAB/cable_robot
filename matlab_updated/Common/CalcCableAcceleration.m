function acceleration = CalcCableAcceleration(vers_rho,vers_rho_deriv,vel_OA_glob,acc_OA_glob)
%CALCCABLEACCELERATION computes the cable acceleration.
%
%   CALCCABLEACCELERATION computes the 2nd order time derivative of the cable 
%   length.
%
%   VERS_RHO is a vector (size[3,1]), containing the components of the 
%   unit vector equidirectional and equiverse to the position vector
%   (A-B). The aforementioned components are projected on the global frame.
%   VERS_RHO_DERIV is a vector (size[3,1]), containing the components of the 
%   time derivative of the unit vector equidirectional and equiverse to the position vector
%   (A-B). The aforementioned components are projected on the global frame.
%   VEL_OA_GLOB is a vector (size[3,1],[m/s]), containing the components
%   of the velocity vector of the distal anchor point A, projected on the
%   global frame.
%   ACC_OA_GLOB is a vector (size[3,1],[m/s]), containing the components
%   of the acceleration vector of the distal anchor point A, projected on the
%   global frame.

  acceleration = dot(vers_rho_deriv,vel_OA_glob)+dot(vers_rho,acc_OA_glob);
  
end