function scalar = CalcSwivelConstraint(cable_v)
%CALCSWIVELCONSTRAINT computes the vector w projection on the pulley plane.
%
%   CALCSWIVELCONSTRAINT computes a scalar quantity that has to be a null 
%   value so that the unit vector w is chosen normal to the pulley plane.  
% 
%   CABLE_V is a structure containing time dependent variables of the cable
%   and the swivel pulley.

scalar = dot(cable_v.vers_w,cable_v.pos_DA_glob);

end