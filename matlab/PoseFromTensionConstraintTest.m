function [c,ceq] = PoseFromTensionConstraintTest(parameters,tension_vector,v)

x_axis = [1;0;0];
y_axis = [0;1;0];
z_axis = [0;0;1];
cdpr_v = CdprVar(parameters.n_cables);
cdpr_v.platform = UpdatePlatformPose(v(1:3),v(4:end),...
  parameters.rotation_parametrization,parameters.platform.pos_G_loc,cdpr_v.platform);
cx = dot(cdpr_v.platform.rot_mat(:,1),x_axis);
sx = norm(cross(cdpr_v.platform.rot_mat(:,1),x_axis));
x_angle = atan2(sx,cx);
cy = dot(cdpr_v.platform.rot_mat(:,2),y_axis);
sy = norm(cross(cdpr_v.platform.rot_mat(:,2),y_axis));
y_angle = atan2(sy,cy);
cz = dot(cdpr_v.platform.rot_mat(:,3),z_axis);
sz = norm(cross(cdpr_v.platform.rot_mat(:,3),z_axis));
z_angle = atan2(sz,cz);

cdpr_v = CdprVar(parameters.n_cables);
cdpr_v = UpdateIKZeroOrd(v(1:3,1),v(4:end,1),parameters,cdpr_v);
cdpr_v = UpdateIKFirstOrd(zeros(3,1),zeros(3,1),parameters,cdpr_v);
cdpr_v.ext_load = CalcExternalLoads(parameters.platform,cdpr_v.platform.rot_mat,...
  cdpr_v.platform.H_mat,cdpr_v.platform.pos_PG_glob);
cdpr_v.tension_vector = tension_vector;    
vect = cdpr_v.analitic_jacobian'*cdpr_v.tension_vector-cdpr_v.ext_load;

c = [x_angle-pi/2;
  -x_angle-pi/2;
  y_angle-pi/2;
  -y_angle-pi/2;
  z_angle-pi/2;
  -z_angle-pi/2];

ceq = vect;

end