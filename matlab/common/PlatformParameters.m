classdef PlatformParameters
  properties
    rotation_parametrization;
    ext_moments_loc;
    mass;
    inertia_mat_G_loc;
    pos_G_loc; 
    ext_force_loc;
    gravity_module = -9.80665;
    gravity_acceleration ;
    pose_dim;
  end
  methods
    function obj = PlatformParameters(platform_parameters_struct)
      obj.rotation_parametrization = platform_parameters_struct.rotation_parametrization;
      obj.ext_moments_loc = platform_parameters_struct.ext_torque_loc;
      obj.mass = platform_parameters_struct.mass;
      obj.inertia_mat_G_loc = platform_parameters_struct.inertia_mat_G_loc;
      obj.pos_G_loc = platform_parameters_struct.pos_PG_loc; 
      obj.ext_force_loc = platform_parameters_struct.ext_force_loc;
      obj.gravity_acceleration = obj.gravity_module.*platform_parameters_struct.gravity_axis;
      if (obj.rotation_parametrization ==  RotationParametrizations.QUATERNION)
        obj.pose_dim = 7;
      else
        obj.pose_dim = 6;
      end
    end
  end
end