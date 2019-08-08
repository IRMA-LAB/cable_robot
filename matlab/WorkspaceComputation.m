clear all
clc
addpath('Common')

[cdpr_parameters, cdpr_variables, cdpr_outputs,record,utilities] = ...
  LoadConfigAndInit("my_config_calib_mod.json","DynamicPlanning");

cdpr_outputs = CalcWorkspace(cdpr_parameters,cdpr_variables,cdpr_outputs,utilities,1,[0;0;0]);
