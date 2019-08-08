function cdpr_v = UnderactuatedDynamicsAcceleration(cdpr_v)
%UNDERACTUATEDDYNAMICSACCELERATION computes a subset of the acceleration vector of the cdpr.
%
%   UNDERACTUATEDDYNAMICSACCELERATION computes the 2nd order time
%   derivatives of the unactuated coordinates of an underactuated system.
%   The following expression arises from the partition of the system of  
%   equations of dynamic equilibrium. 
%
%   CDPR_V is a structure containing time dependent variables of the cdpr.

    cdpr_v.tension_vector = CalcCablesTensionDynUnderAct(cdpr_v);
    cdpr_v.platform.orientation_deriv_2 = linsolve(cdpr_v.platform.mass_matrix_global_uu,cdpr_v.total_load_u...
        -cdpr_v.analitic_jacobian_u'*cdpr_v.tension_vector...
        -cdpr_v.platform.mass_matrix_global_ua*cdpr_v.platform.acceleration);

end