classdef CdprParameter
  properties
    platform;
    cable;
    n_cables;
    workspace_center;
    
  end
  methods
    function obj = CdprParameter(location,name)
      json.startup;
      filepath = fullfile(location, name);
      p =  json.read(filepath);
      workspace_center = zeros(3,1);
      obj.n_cables = length(p.actuator);
      obj.platform = PlatformParameters(p.platform);
      for i=1:obj.n_cables
        cable(i,1) = CableParameters(p.actuator(i));
        workspace_center = workspace_center + cable(i,1).pos_D_glob;
      end   
      obj.workspace_center = workspace_center./3;
      obj.cable = cable;
    end
  end
end