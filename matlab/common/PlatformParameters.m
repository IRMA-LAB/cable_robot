classdef PlatformParameters
  properties
    ext_moments_loc;
    mass;
    inertia_mat_G_loc;
    pos_G_loc; 
    ext_force_loc;
    gravity_module = -9.80665;
    gravity_acceleration ;
  end
  methods
    function obj = PlatformParameters(platform_parameters_struct)
      obj.ext_moments_loc = platform_parameters_struct.ext_moments_loc;
      obj.mass = platform_parameters_struct.mass;
      obj.inertia_mat_G_loc = platform_parameters_struct.inertia_mat_G_loc;
      obj.pos_G_loc = platform_parameters_struct.pos_G_loc; 
      obj.ext_force_loc = platform_parameters_struct.ext_force_loc;
      obj.gravity_acceleration = obj.gravity_module.*platform_parameters_struct.grav_ax;
    end
  end
end