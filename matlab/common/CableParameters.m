classdef CableParameters
  properties
    pos_A_loc;
    pos_D_glob;
    vers_i;
    vers_j;
    vers_k;
    rot_mat
    swivel_pulley_r;
    swivel_pulley_encoder_res;
    motor_cable_tau;
    l0;
    motor_encoder_res;
  end
  methods
    function obj = CableParameters(cable_parameters_struct)
      obj.pos_A_loc = cable_parameters_struct.pos_A_loc;
      obj.pos_D_glob = cable_parameters_struct.pos_D_glob;
      obj.vers_i = cable_parameters_struct.vers_i;
      obj.vers_j = cable_parameters_struct.vers_j;
      obj.vers_k = cable_parameters_struct.vers_k;
      obj.swivel_pulley_r =  cable_parameters_struct.swivel_pulley_r;
      obj.swivel_pulley_encoder_res = cable_parameters_struct.swivel_pulley_encoder_res;
      obj.motor_cable_tau = cable_parameters_struct.motor_cable_tau;
      obj.l0 = cable_parameters_struct.l0;
      obj.motor_encoder_res = cable_parameters_struct.motor_encoder_res;
      obj.rot_mat = [obj.vers_i obj.vers_j obj.vers_k];
    end
  end
end