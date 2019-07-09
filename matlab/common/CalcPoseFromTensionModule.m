function vect = CalcPoseFromTensionModule(cdpr_p,record,parameters,variables)

cdpr_v = CdprVar(cdpr_p.n_cables);
cdpr_v = UpdateIKZeroOrd(variables(1:3,1),variables(4:end,1),cdpr_p,cdpr_v);
cdpr_v = UpdateIKFirstOrd(zeros(3,1),zeros(3,1),cdpr_p,cdpr_v);
cdpr_v.ext_load = CalcExternalLoads(cdpr_p.platform,cdpr_v.platform.rot_mat,...
  cdpr_v.platform.H_mat,cdpr_v.platform.pos_PG_glob,eye(3));
cdpr_v.tension_vector = parameters.*ones(cdpr_p.n_cables,1);    
vect = cdpr_v.analitic_jacobian'*cdpr_v.tension_vector-cdpr_v.ext_load;
%record.SetFrame(cdpr_v,cdpr_p);

end