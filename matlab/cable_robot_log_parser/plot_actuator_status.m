clc; clear; close all;

%% Parse file
filepath = '/tmp/cable-robot-logs/data.log';
data = parseCableRobotLogFile(filepath);

%% Pack according to actuator ID
[sorted_id, sorting_idx] = sort(data.actuator_status.values.id);

actuators_id = unique(sorted_id);
num_actuators = length(actuators_id);
MSG_NUM = floor(length(sorted_id)/num_actuators);
sorted_ts = zeros(num_actuators, MSG_NUM);
torques_mat = sorted_ts;
cable_len_mat = sorted_ts;
motor_pos_mat = sorted_ts;
motor_vel_mat = sorted_ts;
pulley_enc_mat = sorted_ts;
for i = 1:num_actuators
    idx = sorting_idx(sorted_id == actuators_id(i));
    pulley_enc_mat(i, :) = data.actuator_status.values.aux_position(idx(1:MSG_NUM));
    cable_len_mat(i, :) = data.actuator_status.values.cable_length(idx(1:MSG_NUM));
    motor_pos_mat(i, :) = data.actuator_status.values.motor_position(idx(1:MSG_NUM));
    motor_vel_mat(i, :) = data.actuator_status.values.motor_speed(idx(1:MSG_NUM));
    torques_mat(i, :) = data.actuator_status.values.motor_torque(idx(1:MSG_NUM));
    sorted_ts(i, :) = data.actuator_status.timestamp(idx(1:MSG_NUM));
end

%% Plot pulley encoder values
figure('units','normalized','outerposition',[0 0 1 1])
plot(diff(sorted_ts(1,:)))
grid on
title('Times between consecutive log samples of actuator 0')
xlabel('log sample')
ylabel('time [sec]')

%% Plot pulley encoder values
figure('units','normalized','outerposition',[0 0 1 1])
for i = 1:num_actuators
    subplot(3,1,i)
    plot(sorted_ts(i,:), pulley_enc_mat(i,:))
    grid on
    title(sprintf('Actuator #%d pulleys encoder values', actuators_id(i)))
    xlabel('[sec]')
    xlim([sorted_ts(i,1), sorted_ts(i,end)])
end

%% Plot torques
figure('units','normalized','outerposition',[0 0 1 1])
for i = 1:num_actuators
    subplot(3,1,i)
    plot(sorted_ts(i,:), torques_mat(i,:))
    grid on
    title(sprintf('Actuator #%d torques', actuators_id(i)))
    xlabel('[sec]')
    xlim([sorted_ts(i,1), sorted_ts(i,end)])
end

%% Plot cable lengths
figure('units','normalized','outerposition',[0 0 1 1])
for i = 1:num_actuators
    subplot(3,1,i)
    plot(sorted_ts(i,:), cable_len_mat(i,:))
    grid on
    title(sprintf('Actuator #%d cable length', actuators_id(i)))
    xlabel('[sec]')
    xlim([sorted_ts(i,1), sorted_ts(i,end)])
end

%% Plot motor positions
figure('units','normalized','outerposition',[0 0 1 1])
for i = 1:num_actuators
    subplot(3,1,i)
    plot(sorted_ts(i,:), motor_pos_mat(i,:))
    grid on
    title(sprintf('Actuator #%d motor position', actuators_id(i)))
    xlabel('[sec]')
    xlim([sorted_ts(i,1), sorted_ts(i,end)])
end

%% Plot motor velocities
figure('units','normalized','outerposition',[0 0 1 1])
for i = 1:num_actuators
    subplot(3,1,i)
    plot(sorted_ts(i,:), motor_vel_mat(i,:))
    grid on
    title(sprintf('Actuator #%d motor velocity', actuators_id(i)))
    xlabel('[sec]')
    xlim([sorted_ts(i,1), sorted_ts(i,end)])
end