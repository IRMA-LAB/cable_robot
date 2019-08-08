classdef PlatformParameters
  %PLATFORMPARAMETERS is a class containing static parameters of the platform.
  %
  %   PLATFORMPARAMETERS links the parameter values of the platform from an
  %   input structure to the homonymous object's properties.
  %   Parameters consist of inertial properties of the platform and the
  %   wrench components acting on it. The wrench is evaluated wrt the center
  %   of mass G.
  %
  properties
    ext_moments_loc;% is a vector (size[3,1], [Nm]), containing the components of the external moments acting on the platform, about the center of mass G, projected in the local frame.
    mass; % is the mass([kg]) of the platform.
    inertia_mat_G_loc;% is a matrix (size [3,3], [kg m^2]), containing the elements of the inertia matrix about G, projected in the local frame.
    pos_G_loc; % is a vector (size[3,1],[m]), containing the components of the center of mass position, projected in the local frame.
    ext_force_loc;% is a vector (size[3,1], [N]), containing the components of the external forces acting on the platform, projected in the local frame.
    rotation_parametrization;% is a string containing the name of the adopted method for rotation parameterization.
    pose_dim;%is the number of parameters used for platform pose parameterization.
    
    
    gravity_acceleration = [0; 0; -9.80665]; %[m/s^2] unnecessary !!!!!
  end
  methods
    function obj = PlatformParameters(platform_parameters_struct)
      %   PLATFORMPARAMETERS instantiates an object of PlatformParameters type.
      %   PLATFORM_PARAMETERS_STRUCT is a structure containing the platform
      %   parameters, arranged in different fields.
      obj.ext_moments_loc = platform_parameters_struct.ext_torque_loc;
      obj.mass = platform_parameters_struct.mass;
      obj.inertia_mat_G_loc = platform_parameters_struct.inertia_mat_G_loc;
      obj.pos_G_loc = platform_parameters_struct.pos_PG_loc;
      obj.ext_force_loc = platform_parameters_struct.ext_force_loc;
      obj.gravity_acceleration = -norm(obj.gravity_acceleration) * ...
        platform_parameters_struct.gravity_axis;
      obj.rotation_parametrization = platform_parameters_struct.rotation_parametrization;
      if (obj.rotation_parametrization ==  RotationParametrizations.QUATERNION)
        obj.pose_dim = 7;
      else
        obj.pose_dim = 6;
      end
    end
  end
end