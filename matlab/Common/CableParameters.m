classdef CableParameters
  %CABLEPARAMETERS is a class containing the parameters of each cable.
  %
  %   CABLEPARAMETERS links the geometrical parameter values of each cable
  %   and its corresponding swivel pulley from an input structure to the
  %   homonymous object's properties.
  %
  properties
    active;
    pos_A_loc;%is a vector (size[3,1], [m]), containing the components of the distal anchor point A, projected in the local frame.
    pos_D_glob;%is a vector (size[3,1], [m]), containing the components of the proximal anchor point D, projected in the fixed frame.
    vers_i;%
    vers_j;%
    vers_k;%
    rot_mat;%
    swivel_pulley_r;% is the swivel pulley radius.
    swivel_pulley_encoder_res;% is the swivel pulley encoder resolution.
    motor_cable_tau;% is the transmission ratio
    l0;% is the initial
    motor_encoder_res;% is the motor encoder resolution.
  end
  methods
    function obj = CableParameters(actuator_p_struct)
      %   CABLEPARAMETERS instantiates an object of CableParameters type.
      %   CABLE_PARAMETERS_STRUCT is a structure containing the cable
      %   parameters, arranged in different fields.
      obj.active = actuator_p_struct.active;
      obj.pos_A_loc = actuator_p_struct.winch.pos_PA_loc;
      obj.pos_D_glob = actuator_p_struct.pulley.pos_OD_glob;
      obj.vers_i = actuator_p_struct.pulley.vers_i;
      obj.vers_j = actuator_p_struct.pulley.vers_j;
      obj.vers_k = actuator_p_struct.pulley.vers_k;
      obj.swivel_pulley_r =  actuator_p_struct.pulley.radius;
      obj.swivel_pulley_encoder_res = actuator_p_struct.pulley.encoder_res;
      obj.motor_cable_tau = sqrt((pi * actuator_p_struct.winch.drum_diameter)^2 ...
        + actuator_p_struct.winch.drum_pitch^2 ...
        - actuator_p_struct.winch.drum_diameter * 0.1) / ...
        (actuator_p_struct.winch.motor_encoder_res * ...
        actuator_p_struct.winch.gear_ratio);
      obj.l0 = actuator_p_struct.winch.l0;
      obj.motor_encoder_res = actuator_p_struct.winch.motor_encoder_res;
      obj.rot_mat = [obj.vers_i obj.vers_j obj.vers_k];
    end
  end
end