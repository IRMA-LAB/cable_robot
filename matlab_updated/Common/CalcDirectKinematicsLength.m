function vector = CalcDirectKinematicsLength(cdpr_p,record,parameters,R,variables)

cdpr_v = CdprVar(cdpr_p.n_cables);
cdpr_v = UpdateIKZeroOrd(variables(1:3,1),variables(4:end,1),cdpr_p,cdpr_v);
cdpr_v = UpdateIKFirstOrd(zeros(3,1),zeros(3,1),cdpr_p,cdpr_v);
cdpr_v.ext_load = CalcExternalLoads(cdpr_p.platform,cdpr_v.platform.rot_mat,...
  cdpr_v.platform.H_mat,cdpr_v.platform.pos_PG_glob,R);

  Ja(:,1:cdpr_p.n_cables) = cdpr_v.analitic_jacobian(:,1:3);
  Wa(1:cdpr_p.n_cables,1) = cdpr_v.ext_load(1:3,1);
  Ju(:,1:cdpr_p.pose_dim-cdpr_p.n_cables) = cdpr_v.analitic_jacobian(:,4:cdpr_p.pose_dim);
  Wu(1:6-cdpr_p.n_cables,1) = cdpr_v.ext_load(4:end,1);
  cdpr_v.tension_vector = linsolve(Ja',Wa);
for i=1:cdpr_p.n_cables
  cdpr_v.cable(i).length = parameters(i);
  vector(i,1) = CalcCableConstraint(cdpr_v.cable(i),cdpr_p.cable(i).swivel_pulley_r);
end
vector(cdpr_p.n_cables+1:cdpr_p.pose_dim,1) = Ju'*cdpr_v.tension_vector -Wu;

end