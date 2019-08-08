% InverseDynamicPlanner33
clear all
clc
addpath('Common')

[cdpr_parameters, cdpr_variables, cdpr_outputs,record,utilities] = ...
  LoadConfigAndInit("my_config_calib_mod.json","DynamicPlanning");

simulationData = struct();
geometricFunction = @LineFunction;
simulationData = NormalizedPoly7Coefficients(1,simulationData);
simulationData = GetDestinations(simulationData,cdpr_parameters,record,utilities);
  
[outputDataRTR] = RestToRestCoefficients33(cdpr_parameters,cdpr_variables,...
     simulationData,geometricFunction,utilities,record);
[outputDataSTD] = StandardInverseSimulator(cdpr_parameters,cdpr_variables,...
     simulationData,geometricFunction,utilities);
DataLoggerStruct(outputDataRTR,'InverseRTR',true,cdpr_parameters,cdpr_variables,record,utilities);
DataLoggerStruct(outputDataSTD,'InverseSTD',true,cdpr_parameters,cdpr_variables,record,utilities);
 