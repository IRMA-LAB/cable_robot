classdef CdprVar
%CDPRVAR is a class containing time dependent variables of the cdpr.

  properties
    platform = PlatformVar;% is an object containing time dependent variables of the platform.
    cable = CableVar;% is an object containing time dependent variables of a cable and its swivel pulley.
    
    geometric_jacobian;%
    analitic_jacobian;%
    
    geometric_jacobian_d;%
    analitic_jacobian_d;%
    
    analitic_jacobian_a;%
    analitic_jacobian_u;%
    
    dyn_load;%
    dyn_load_ss;%
    total_load;%
    total_load_ss;%
    total_load_a;%
    total_load_u;%
    
    tension_vector;%
    
  end
  methods
    function obj = CdprVar(n)
        % CDPRVAR instantiates an object of CableParameters type.
        % CDPRVAR defines the object CABLE containing an object for each
        % cable that stores time dependent variables of the cable and its
        % swivel pulley.
      for i=1:n
        obj.cable(i, 1) = CableVar;
      end
    end
  end
end