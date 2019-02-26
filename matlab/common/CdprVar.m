classdef CdprVar
  properties
    platform = PlatformVar;
    cable = CableVar;
    
    geometric_jacobian;
    analitic_jacobian;
    
    ext_load;
    tension_vector;
    
  end
  methods
    function obj = CdprVar(n)
      for i=1:n
        obj.cable(i,1) = CableVar;
      end
    end
  end
end