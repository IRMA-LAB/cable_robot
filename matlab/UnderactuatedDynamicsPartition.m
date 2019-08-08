function cdpr_v = UnderactuatedDynamicsPartition(cdpr_v)

    n = length(cdpr_v.cable);
    cdpr_v.platform.mass_matrix_global_aa = cdpr_v.platform.mass_matrix_global_ss(1:n,1:n);
    cdpr_v.platform.mass_matrix_global_au = cdpr_v.platform.mass_matrix_global_ss(1:n,n+1:end);
    cdpr_v.platform.mass_matrix_global_ua = cdpr_v.platform.mass_matrix_global_ss(n+1:end,1:n);
    cdpr_v.platform.mass_matrix_global_uu = cdpr_v.platform.mass_matrix_global_ss(n+1:end,n+1:end);
    
    cdpr_v.analitic_jacobian_a = cdpr_v.analitic_jacobian(:,1:n);
    cdpr_v.analitic_jacobian_u = cdpr_v.analitic_jacobian(:,n+1:end);
    
    cdpr_v.total_load_a = cdpr_v.total_load_ss(1:n,1);
    cdpr_v.total_load_u = cdpr_v.total_load_ss(n+1:end,1);

end