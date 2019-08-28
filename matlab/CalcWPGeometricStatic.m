function [vector, mat] = CalcWPGeometricStatic(cdpr_p, parameters, variables, mask)

parameters_index = 1;
variables_index = 1;

pose = zeros(cdpr_p.platform.pose_dim, 1);
for i = 1:cdpr_p.platform.pose_dim
  if (mask(i, 1) == 1)
    pose(i, 1) = parameters(parameters_index, 1);
    parameters_index = parameters_index + 1;
  else
    pose(i, 1) = variables(variables_index, 1);
    variables_index = variables_index + 1;
  end
end
num_cables = length(cdpr_p.activeActuatorsId);
cdpr_v = CdprVar(num_cables);
cdpr_v = UpdateIKZeroOrd(pose(1:3, 1), pose(4:end, 1), cdpr_p, cdpr_v);
cdpr_v.platform = CalcExternalLoads(cdpr_v.platform, cdpr_p.platform, eye(3));

Ja = zeros(num_cables, num_cables);
Ju = zeros(num_cables, cdpr_p.platform.pose_dim - num_cables);
Wa = zeros(num_cables, 1);
Wu = zeros(cdpr_p.platform.pose_dim - num_cables, 1);
actuated_index = 1;
not_actuated_index = 1;

for i = 1:cdpr_p.platform.pose_dim
  if (mask(i, 1) == 1)
    Ja(:, actuated_index) = cdpr_v.geometric_jacobian(:, i);
    Wa(actuated_index, 1) = cdpr_v.platform.ext_load(i, 1);
    actuated_index = actuated_index + 1;
  else
    Ju(:, not_actuated_index) = cdpr_v.geometric_jacobian(:, i);
    Wu(not_actuated_index, 1) = cdpr_v.platform.ext_load(i, 1);
    not_actuated_index = not_actuated_index + 1;
  end
end

cdpr_v.tension_vector = linsolve(Ja', Wa);
vector = Ju' * cdpr_v.tension_vector - Wu;
mat = CalcGsJacobians(cdpr_v, Ja, Ju, cdpr_p.platform.mass .* ...
  cdpr_p.platform.gravity_acceleration);
