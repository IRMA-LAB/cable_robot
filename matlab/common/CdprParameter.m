classdef CdprParameter
  properties
    platform;
    cable;
    rotation_parametrization;
    n_cables;
    pose_dim;
    workspace_center;
    
  end
  methods
    function obj = CdprParameter(location,name)
      json.startup;
      p =  json.read(fullfile(location, name));
      workspace_center = zeros(3,1);
      obj.n_cables = length(p.cable);
      obj.rotation_parametrization = p.rotation_parametrization;
      if (obj.rotation_parametrization ==  RotationParametrizations.QUATERNION)
        obj.pose_dim = 7;
      else
        obj.pose_dim = 6;
      end
      obj.platform = PlatformParameters(p.platform);
      for i=1:obj.n_cables
        cable(i,1) = CableParameters(p.cable(i));
        workspace_center = workspace_center + cable(i,1).pos_D_glob;
      end   
      obj.workspace_center = workspace_center./3;
      obj.cable = cable;
    end
  end
end