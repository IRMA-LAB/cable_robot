classdef CdprParameter
  %CDPRPARAMETER is a class containing static parameters of the cdpr.
  %   CDPRPARAMETER decodes data from .json format to matlab objects.
  %   It needs a .json configuration file in order to define the set
  %   of static parameters as object's properties.
  %
  properties
    platform;% is an object containing the parameters of the platform.
    cable;% is an object containing the parameters of each cable and its corresponding swivel pulley.
    n_cables;%is the number of all cables.
    workspace_center;% is a vector (size[3,1], [m]) containing the components of the workspace center, projected in the fixed frame.
    
  end
  
  methods
    function obj = CdprParameter(location,name)
      %   CDPRPARAMETER instantiates an object of CdprParameters type.
      %   LOCATION is a string containing the path where the file NAME is
      %   located.
      %   NAME is a string containing the name of a .json config file.
      current_folder = pwd;
      cd(location)
      json.startup;
      p =  json.read(name);
      workspace_center = zeros(3, 1);
      obj.n_cables = length(p.actuator);
      obj.platform = PlatformParameters(p.platform);
      for i = 1:obj.n_cables
        cable(i, 1) = CableParameters(p.actuator(i));
        workspace_center = workspace_center + cable(i, 1).pos_D_glob;
      end
      obj.workspace_center = workspace_center./3;
      obj.workspace_center(3) = 0;
      obj.cable = cable;
      cd(current_folder);
    end
  end
end