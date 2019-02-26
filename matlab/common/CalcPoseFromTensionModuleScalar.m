function [vect,mat] = CalcPoseFromTensionModuleScalar(cdpr_p,record,tension_vector,variables)

cdpr_v = CdprVar(cdpr_p.n_cables);
cdpr_v = UpdateIKZeroOrd(variables(1:3,1),variables(4:end,1),cdpr_p,cdpr_v);
cdpr_v = UpdateIKFirstOrd(zeros(3,1),zeros(3,1),cdpr_p,cdpr_v);
cdpr_v.ext_load = CalcExternalLoads2(cdpr_p.platform,cdpr_v.platform.rot_mat,...
  cdpr_v.platform.pos_PG_glob,eye(3));
cdpr_v.tension_vector = tension_vector;    
vect = cdpr_v.geometric_jacobian'*cdpr_v.tension_vector-cdpr_v.ext_load;
%scalar = dot(vect,vect);
Ja(:,1:cdpr_p.n_cables) = cdpr_v.geometric_jacobian(:,1:3);
Wa(1:cdpr_p.n_cables,1) = cdpr_v.ext_load(1:3,1);
Ju(:,1:cdpr_p.pose_dim-cdpr_p.n_cables) = cdpr_v.geometric_jacobian(:,4:cdpr_p.pose_dim);
Wu(1:6-cdpr_p.n_cables,1) = cdpr_v.ext_load(4:end,1);
mat = CalcOptimizationJacobiansGS(cdpr_v,Ja,Ju,cdpr_p.platform.mass.*cdpr_p.platform.gravity_acceleration);
%record.SetFrame(cdpr_v,cdpr_p);

end