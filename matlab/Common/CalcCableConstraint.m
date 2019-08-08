function scalar = CalcCableConstraint(cable_v,r)
%CALCCABLECONSTRAINT computes a scalar value connected to the cable length constraint. 
%
%   CALCCABLECONSTRAINT computes a scalar quantity that has to be a null
%   value in order to fulfill the cable constraint equation.
%   The aforementioned constraint equation links the norm of the position 
%   vector (A-B) with the cable length, the pulley radius and the wrapped  
%   angle of the cable inside the pulley groove.
%
%   CABLE_V is a structure containing time dependent variables of the  
%   cable and its swivel pulley.
%   R is the pulley radius.

scalar = dot(cable_v.pos_BA_glob,cable_v.pos_BA_glob) -...
  (cable_v.length-r.*(pi-cable_v.tan_ang)).^2;

end