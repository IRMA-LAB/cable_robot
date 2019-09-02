function J_opt = CalcGsJacobians(cdpr_v,Ja,Ju,mg)

% n = length(Ja);
% m = length([Ja Ju]);
% J_sl = zeros(2*n,m);
% for i=1:n
%   dadq = [eye(3,3) -Anti(cdpr_v.cable(i).pos_PA_glob)*cdpr_v.platform.H_mat];
%   dsdq(i,:) = cdpr_v.cable(i).vers_w'*dadq./...
%     dot(cdpr_v.cable(i).pos_DA_glob,cdpr_v.cable(i).vers_u);
%   %dphidq(i,:) = cdpr_v.cable(i).vers_n'*dadq./...
%   %  norm(cdpr_v.cable(i).pos_BA_glob);
%   dphidq(i,:) = cdpr_v.cable(i).vers_n'*dadq./...
%     norm(cdpr_v.cable(i).pos_BA_glob);
%   dldq(i,:) = cdpr_v.cable(i).vers_rho'*dadq;
%   %jj = sin(cdpr_v.cable(i).tan_ang).*cdpr_v.cable(i).vers_w*dsdq(i,:);
%   drhodq = sin(cdpr_v.cable(i).tan_ang).*cdpr_v.cable(i).vers_w*dsdq(i,:)+...
%       cdpr_v.cable(i).vers_n*dphidq(i,:);
%   dJTp(:,i,:) = drhodq;
%   dadql = [zeros(3,3) -Anti(cdpr_v.cable(i).pos_PA_glob)*cdpr_v.platform.H_mat];
%   dJTo(:,i,:) = Anti(cdpr_v.cable(i).pos_PA_glob)*drhodq-Anti(cdpr_v.cable(i).vers_rho)*dadql;
% end
% 
% J_sl(1:n,1:m) = dsdq;
% J_sl(n+1:end,1:m) = dldq;
% dJdq = [dJTp;dJTo];
% dJa(:,:,:) = dJdq(1:n,:,:);
% dJu(:,:,:) = dJdq(n+1:end,:,:);
% dfdq = [zeros(3,6);zeros(3,3) Anti(mg)*Anti(cdpr_v.platform.pos_PG_glob)*cdpr_v.platform.H_mat];
% dWa(:,:) = dfdq(1:n,:);
% dWu(:,:) = dfdq(n+1:end,:);
% 
% j0u = zeros(m-n,m);
% j0a = zeros(n,m);
% for i=1:m
%   j0u(:,i) = dJu(:,:,i)*cdpr_v.tension_vector;
%   j0a(:,i) = dJa(:,:,i)*cdpr_v.tension_vector;
% end
% 
% J_q = j0u+Ju'*linsolve(Ja',dWa-j0a)-dWu;
% J_q(:,1:n) = [];
% 
% clear dadq dsdq dphidq dldq drhodq 

n = length(Ja);
m = length([Ja Ju]);
J_sl = zeros(2*n,m);
dJT = zeros(m,m);
for i=1:n
  dadq = [eye(3,3) -Anti(cdpr_v.cable(i).pos_PA_glob)*cdpr_v.platform.H_mat];
  dsdq(1,:) = cdpr_v.cable(i).vers_w'*dadq./...
    dot(cdpr_v.cable(i).pos_DA_glob,cdpr_v.cable(i).vers_u);
  dphidq(1,:) = cdpr_v.cable(i).vers_n'*dadq./...
    norm(cdpr_v.cable(i).pos_BA_glob);
  dldq(1,:) = cdpr_v.cable(i).vers_rho'*dadq;
  drhodq = sin(cdpr_v.cable(i).tan_ang).*cdpr_v.cable(i).vers_w*dsdq(1,:)+...
      cdpr_v.cable(i).vers_n*dphidq(1,:);
  dadq(1:3,1:3) = zeros(3);
  dJT = dJT+[drhodq;Anti(cdpr_v.cable(i).pos_PA_glob)*drhodq-Anti(cdpr_v.cable(i).vers_rho)*dadq].*cdpr_v.tension_vector(i);
  J_sl(i,:) = dsdq;
  J_sl(n+i,:) = dldq;
end

dfdq = [zeros(3,6);zeros(3,3) Anti(mg)*Anti(cdpr_v.platform.pos_PG_glob)*cdpr_v.platform.H_mat];
%assume the geomectric static mask is 1 up to the n-th coordinate, then 0.
% this elps with the selection of the columns of dJ and dfdq
J_opt = dJT(n+1:m,n+1:end) + Ju'*linsolve(Ja',dfdq(1:n,n+1:end)-dJT(1:n,n+1:end))-dfdq(n+1:end,n+1:end);





end