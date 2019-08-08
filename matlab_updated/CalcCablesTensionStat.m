function cdpr_v = CalcCablesTensionStat(cdpr_v)
%CALCCABLESTENSIONDYNUNDERACT computes the cables tension 
%   CALCCABLESTENSIONDYN
%
%   CDPR_V is a structure containing time dependent variables of the cdpr.
%
%   VECTOR is the vector (size[])
    A = -cdpr_v.analitic_jacobian*cdpr_v.analitic_jacobian';
    b = cdpr_v.analitic_jacobian*(-cdpr_v.ext_load_ss);
    vector = linsolve(A,b);

    for i=1:length(vector)
        if (vector(i)<=0)
            vector = NaN.*ones(length(vector));
        end
    end
    cdpr_v.tension_vector = vector;
end