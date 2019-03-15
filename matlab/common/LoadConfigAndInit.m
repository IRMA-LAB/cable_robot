function [parameters,variables,outputs,record,utilities] = ...
  LoadConfigAndInit(config_name,simulation_title)

parameters = CdprParameter('config',config_name);
variables = CdprVar(parameters.n_cables);

outputFieldsPlatform = fieldnames(variables.platform);
n_elem = parameters.n_cables;
outputFieldsCable = fieldnames(variables.cable(1));
  
  for i=1:numel(outputFieldsPlatform)
    outputs.platform.(outputFieldsPlatform{i}) = [];
  end
  
  for j = 1:n_elem
    for i = 1:numel(outputFieldsCable)
      outputs.cable(j).(outputFieldsCable{i}) = [];
    end
  end
  outputs.index = [];
record = RecordType(parameters,simulation_title);
utilities = UtilitiesType;

end