function vector = CalcPoseFromHybridGeoStatic(cdpr_p,index,parameters,variables)

cdpr_v = CdprVar(cdpr_p.n_cables);
cdpr_v = UpdateIKZeroOrd(variables(1:3,1),variables(4:end,1),cdpr_p,cdpr_v);
cdpr_v = UpdateIKFirstOrd(zeros(3,1),zeros(3,1),cdpr_p,cdpr_v);
cdpr_v.ext_load = CalcExternalLoads(cdpr_p.platform,cdpr_v.platform.rot_mat,...
  cdpr_v.platform.H_mat,cdpr_v.platform.pos_PG_glob,eye(3));

ext_index = 1;
for i=1:cdpr_p.n_cables
  if (i==index)
    cdpr_v.cable(i).length = parameters(1);
    cdpr_v.tension_vector(i,1) = variables(end,1);
  else
    ext_index = ext_index + 1;
    cdpr_v.tension_vector(i,1) = parameters(ext_index,1);
  end
end

vector(1:6,1) = cdpr_v.analitic_jacobian'*cdpr_v.tension_vector-cdpr_v.ext_load;
vector(7,1) = CalcCableConstraint(cdpr_v.cable(index),cdpr_p.cable(index).swivel_pulley_r);

end