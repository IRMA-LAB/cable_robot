function J_opt = CalcGsJacobians(cdpr_v, Ja, Ju, mg)
n = length(Ja);
m = length([Ja Ju]);
J_sl = zeros(2*n, m);
dJT = zeros(m, m);
for i=1:n
  dadq = [eye(3,3) -Anti(cdpr_v.cable(i).pos_PA_glob) * cdpr_v.platform.H_mat];
  dsdq(1,:) = cdpr_v.cable(i).vers_w' * dadq./...
    dot(cdpr_v.cable(i).pos_DA_glob, cdpr_v.cable(i).vers_u);
  dphidq(1,:) = cdpr_v.cable(i).vers_n' * dadq./...
    norm(cdpr_v.cable(i).pos_BA_glob);
  dldq(1,:) = cdpr_v.cable(i).vers_rho' * dadq;
  drhodq = sin(cdpr_v.cable(i).tan_ang) .* cdpr_v.cable(i).vers_w * dsdq(1,:) +...
    cdpr_v.cable(i).vers_n * dphidq(1,:);
  dadq(1:3,1:3) = zeros(3);
  dJT = dJT + [drhodq; Anti(cdpr_v.cable(i).pos_PA_glob) * drhodq - ...
    Anti(cdpr_v.cable(i).vers_rho) * dadq] .* cdpr_v.tension_vector(i);
  J_sl(i,:) = dsdq;
  J_sl(n+i,:) = dldq;
end

dfdq = [zeros(3, 6);
  zeros(3,3)   ...
  Anti(mg) * Anti(cdpr_v.platform.pos_PG_glob) * cdpr_v.platform.H_mat];

% assume the geomectric static mask is 1 up to the n-th coordinate, then 0.
% this helps with the selection of the columns of dJ and dfdq
J_opt = dJT(n+1:m, n+1:end) + Ju' * linsolve(Ja', dfdq(1:n, n+1:end) - ...
  dJT(1:n, n+1:end)) - dfdq(n+1:end, n+1:end);
