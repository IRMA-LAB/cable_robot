clc; clear; close all;

[cdpr_parameters, cdpr_variables, ~, record,utilities] = ...
    LoadConfigAndInit("my_config_copt.json", "homing");

reference_position = cdpr_parameters.workspace_center-[0; 0; 1.5];
geometric_static_mask = [1; 1; 1; 0; 0; 0];
reference_angle = fsolve(@(v) CalcGenericGeometricStatic(...
    cdpr_parameters, record, reference_position, v, geometric_static_mask),...
    zeros(cdpr_parameters.pose_dim-cdpr_parameters.n_cables, 1), ...
    utilities.fsolve_options);
cdpr_variables = UpdateIKZeroOrd(reference_position, reference_angle,...
    cdpr_parameters,cdpr_variables);
start_pose = [reference_position; reference_angle];
tension_module = [40; 40; 40];
home_pose = fsolve(@(v) CalcPoseFromTensionModule(...
    cdpr_parameters, record, tension_module, v),...
    start_pose, utilities.fsolve_options);
cdpr_variables = UpdateIKZeroOrd(home_pose(1:cdpr_parameters.n_cables),...
    home_pose(cdpr_parameters.n_cables + 1:cdpr_parameters.pose_dim),...
    cdpr_parameters, cdpr_variables);
home_l = zeros(cdpr_parameters.n_cables, 1);
home_swivel_ang = home_l;
for i=1:cdpr_parameters.n_cables
    home_l(i,1) = cdpr_variables.cable(i).length;
    home_swivel_ang(i,1) = cdpr_variables.cable(i).swivel_ang;
end

% Here automatic initial guess for the homing algorithm is generated,
% making use of banal extimation of the workspace center (geometrical
% property  of the robot) and acquired data. No need for user interaction.

[imported_data_coarse, ~] = parseCableRobotLogFile('data.log');
% [imported_data_coarse, ~] = parseCableRobotLogFile('/tmp/cable-robot-logs/data.log');
imported_data = Reparse(imported_data_coarse.actuator_status.values,...
    cdpr_parameters);
sizes=size(imported_data);
vLength = imported_data(1, 1:sizes(2)/2)';
vSwivelAngle = imported_data(1, sizes(2)/2+1:sizes(2))';
for j=2:sizes(1)
    vLength = [vLength ; imported_data(j, 1:sizes(2)/2)'];
    vSwivelAngle = [vSwivelAngle ;
        imported_data(j, sizes(2)/2+1:sizes(2))'];
end
[meas_stage, ~] = size(vLength);
meas_stage = meas_stage / cdpr_parameters.n_cables;
cables_acquisitions = meas_stage / cdpr_parameters.n_cables;

% Automatic Initial Guess Generation for Homing
for i = 1:meas_stage
    cdpr_v_ideal = cdpr_variables;
    l_current = home_l + vLength(cdpr_parameters.n_cables*(i-1) + ...
        1:i*cdpr_parameters.n_cables);
    if(mod(i-1,cables_acquisitions) == 0)
        start_sol = home_pose;
    end
    [sol,~] = fsolve(@(v)CalcDirectKinematicsLength(cdpr_parameters,...
        record,l_current,eye(3),v),start_sol,utilities.fsolve_options);
    start_sol = sol;
    vPose(cdpr_parameters.pose_dim*(i-1)+1:i*cdpr_parameters.pose_dim,1) = sol;
    cdpr_v_ideal = UpdateIKZeroOrd(sol(1:3,1),...
        sol(4:cdpr_parameters.pose_dim,1),cdpr_parameters,cdpr_v_ideal);
    cdpr_v_ideal = UpdateIKFirstOrd(zeros(3,1),zeros(3,1),...
        cdpr_parameters,cdpr_v_ideal);
    cdpr_v_ideal.ext_load = CalcExternalLoads(cdpr_parameters.platform,...
        cdpr_v_ideal.platform.rot_mat, cdpr_v_ideal.platform.H_mat, ...
        cdpr_v_ideal.platform.pos_PG_glob, eye(3));
    cdpr_v_ideal.tension_vector = CalcCablesTension(cdpr_v_ideal);
    record.SetFrame(cdpr_v_ideal,cdpr_parameters);
    
end

% Contruct external patameter and initial guess
initial_guess1 = [home_swivel_ang;home_l;vPose];

tau = [0.961590794366752; 0.961590794366752; 0.962250315307843];
for i=1:length(vLength)/cdpr_parameters.n_cables
    idx = ((i - 1) * cdpr_parameters.n_cables + 1):...
        (i * cdpr_parameters.n_cables);
    vLength(idx) = vLength(idx) .* tau(1:cdpr_parameters.n_cables);
end
[sol2,~,~,~,~] = lsqnonlin(@(v)HomingOptimizationFunction6...
    (cdpr_parameters, record, vLength, vSwivelAngle, v), ...
    initial_guess1, [], [], utilities.lsqnonlin_options_grad);
angles = sol2(1:3);
lenghts = sol2(4:6);
res = [ angles lenghts ];

j_struct.init_angles = res(:, 1)';
j_struct.init_lengths = res(:, 2)';
json.startup
json.write(j_struct, 'res.json')
fprintf('Results dumped in %s\n', strcat(pwd, '/res.json'))