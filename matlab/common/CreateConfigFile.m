clear all;
clc;

s.rotation_parametrization = char(RotationParametrizations.TAYT_BRYAN); 

s.platform.mass = 8.0064014;
s.platform.ext_force_loc = [0;0;0];
s.platform.ext_moments_loc = [0;0;0];
s.platform.inertia_mat_G_loc = [0.13974096 0 0;
                                     0 0.13974096 0;
                                     0 0 0.21622689];
s.platform.pos_G_loc = [0;0;0.18185573];

s.cable(1).pos_A_loc = [0;-0.2668;0.2700];
s.cable(1).pos_D_glob = [0.1597;-0.8359;-0.0230];
s.cable(1).vers_i = [0;1;0];
s.cable(1).vers_j = [-1;0;0];
s.cable(1).vers_k = [0;0;1];
s.cable(1).swivel_pulley_r =  0.025;
s.cable(1).swivel_pulley_encoder_res = 2^18;
s.cable(1).motor_cable_tau = 2;
s.cable(1).l0 = 1;
s.cable(1).motor_encoder_res = 2^20;

s.cable(2).pos_A_loc= [0.2311;0.1334;0.2700];
s.cable(2).pos_D_glob = [2.1753;0.1817;-0.0351];
s.cable(2).vers_i = [-1;0;0];
s.cable(2).vers_j = [0;-1;0];
s.cable(2).vers_k = [0;0;1];
s.cable(2).swivel_pulley_r =  0.025;
s.cable(2).swivel_pulley_encoder_res = 2^18;
s.cable(2).motor_cable_tau = 2;
s.cable(2).l0 = 1;
s.cable(2).motor_encoder_res = 2^20;

s.cable(3).pos_A_loc = [-0.2311;0.1334;0.2700];
s.cable(3).pos_D_glob = [0.2591;1.2883;-0.0430];
s.cable(3).vers_i = [0;-1;0];
s.cable(3).vers_j = [1;0;0];
s.cable(3).vers_k = [0;0;1];
s.cable(3).swivel_pulley_r =  0.025;
s.cable(3).swivel_pulley_encoder_res = 2^18;
s.cable(3).motor_cable_tau = 2;
s.cable(3).l0 = 1;
s.cable(3).motor_encoder_res = 2^20;

cd('../Configuration Files');
json.startup;
json.write(s,"my_config_1.json",'Indent',2);
cd ('../Common');

clear all
