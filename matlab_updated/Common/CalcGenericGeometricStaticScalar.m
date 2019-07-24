function scalar = CalcGenericGeometricStaticScalar(cdpr_p,record,parameters,variables,mask)

parameters_index = 1;
variables_index = 1;

pose = zeros(cdpr_p.pose_dim,1);
for i=1:cdpr_p.pose_dim
  if (mask(i,1) == 1)
    pose(i,1) = parameters(parameters_index,1);
    parameters_index = parameters_index + 1;
  else
    pose(i,1) = variables(variables_index,1);
    variables_index = variables_index + 1;
  end
end
cdpr_v = CdprVar(cdpr_p.n_cables);
cdpr_v = UpdateIKZeroOrd(pose(1:3,1),pose(4:end,1),cdpr_p,cdpr_v);
cdpr_v = UpdateIKFirstOrd(zeros(3,1),zeros(3,1),cdpr_p,cdpr_v);

cdpr_v.ext_load = CalcExternalLoads(cdpr_p.platform,cdpr_v.platform.rot_mat,...
  cdpr_v.platform.H_mat,cdpr_v.platform.pos_PG_glob);

Ja = zeros(cdpr_p.n_cables,cdpr_p.n_cables);
Ju = zeros(cdpr_p.n_cables,cdpr_p.pose_dim-cdpr_p.n_cables);
Wa = zeros(cdpr_p.n_cables,1);
Wu = zeros(cdpr_p.pose_dim-cdpr_p.n_cables,1);
actuated_index = 1;
not_actuated_index = 1;

for i=1:cdpr_p.pose_dim
  if (mask(i,1) == 1)
    Ja(:,actuated_index) = cdpr_v.analitic_jacobian(:,i);
    Wa(actuated_index,1) = cdpr_v.ext_load(i,1);
    actuated_index = actuated_index + 1;
  else
    Ju(:,not_actuated_index) = cdpr_v.analitic_jacobian(:,i);
    Wu(not_actuated_index,1) = cdpr_v.ext_load(i,1);
    not_actuated_index = not_actuated_index + 1;
  end
end

cdpr_v.tension_vector = linsolve(Ja',Wa);
vector = Ju'*cdpr_v.tension_vector -Wu;
scalar = dot(vector,vector);
record.SetFrame(cdpr_v,cdpr_p);

end