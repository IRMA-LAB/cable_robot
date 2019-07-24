function cdpr_v = UnderactuatedDynamicsPartition(cdpr_v)
%UNDERACTUATEDDYNAMICSPARTITION performs the partition of quantities affecting the dynamic equilibrium of an underactuated system.  
% 
%   The number of actuated coordinates of underactuated systems is less
%   than the total number of the generalized coordinates. It is convenient
%   to perform a partition of physical quantities in order to decouple the
%   system of dynamic equations. Quantities showing the subscript "a" are
%   referred to actuated coordinates, on the contrary the subscript "u" to
%   the unactuated ones.
%  
%   CDPR_V is a structure containing time dependent variables o the cdpr.

n = length(cdpr_v.cable);

cdpr_v.platform.mass_matrix_global_aa = cdpr_v.platform.mass_matrix_global(1:n,1:n);
cdpr_v.platform.mass_matrix_global_au = cdpr_v.platform.mass_matrix_global(1:n,n+1:end);
cdpr_v.platform.mass_matrix_global_ua = cdpr_v.platform.mass_matrix_global(n+1:end,1:n);
cdpr_v.platform.mass_matrix_global_uu = cdpr_v.platform.mass_matrix_global(n+1:end,n+1:end);
    
cdpr_v.analitic_jacobian_a = cdpr_v.analitic_jacobian(:,1:n);
cdpr_v.analitic_jacobian_u = cdpr_v.analitic_jacobian(:,n+1:end);
    
cdpr_v.total_load_a = cdpr_v.total_load(1:n,1);
cdpr_v.total_load_u = cdpr_v.total_load(n+1:end,1);

end