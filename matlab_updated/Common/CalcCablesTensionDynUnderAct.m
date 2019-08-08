function vector = CalcCablesTensionDynUnderAct(cdpr_v)
%CALCCABLESTENSIONDYNUNDERACT computes the cables tension 
%   CALCCABLESTENSIONDYN
%
%   CDPR_V is a structure containing time dependent variables of the cdpr.
%
%   VECTOR is the vector (size[])
    inv_mass_matrix_uu = MyInv(cdpr_v.platform.mass_matrix_global_uu);
    A = cdpr_v.analitic_jacobian_a'-cdpr_v.platform.mass_matrix_global_au*...
        inv_mass_matrix_uu*cdpr_v.analitic_jacobian_u';
    b = (cdpr_v.platform.mass_matrix_global_aa-cdpr_v.platform.mass_matrix_global_au*...
        inv_mass_matrix_uu*cdpr_v.platform.mass_matrix_global_ua)*cdpr_v.platform.acceleration+...
        cdpr_v.platform.mass_matrix_global_au*inv_mass_matrix_uu*cdpr_v.total_load_u-cdpr_v.total_load_a;
    vector = linsolve(A,-b);

    for i=1:length(vector)
        if (vector(i)<=0)
            vector(i)=NaN;
        end
    end

end