clear all
clc
addpath('Common')

[cdpr_parameters, cdpr_variables, cdpr_outputs,record,utilities] = ...
  LoadConfigAndInit("my_config_calib_mod.json","DynamicPlanning"); 

traj(1) = load('Results\InverseRTR1.mat');
traj(2) = load('Results\InverseRTR2.mat');
traj(3) = load('Results\InverseRTR3.mat');
traj(4) = load('Results\InverseRTR4.mat');
traj(5) = load('Results\InverseRTR5.mat');
traj(6) = load('Results\InverseRTR6.mat');
traj(7) = load('Results\InverseRTR7.mat');
traj(8) = load('Results\InverseRTR8.mat');
traj(9) = load('Results\InverseRTR9.mat');
traj(10) = load('Results\InverseRTR10.mat');
traj(11) = load('Results\InverseRTR11.mat');
traj(12) = load('Results\InverseRTR12.mat');
[t,cables,p0] = JoinTrajectories(traj,cdpr_parameters.n_cables,utilities.t_interval);

i=0;
for tt = t
    i=i+1;
    for j=1:cdpr_parameters.n_cables
        l(j,i) = cables(j).length(i)+(2.*rand(1,1)-1).*0.0002;
        if i==1
            dl(j,i) = FiniteDifferentiation(l(j,i),l(j,i),0,t(i),utilities);
            ddl(j,i) = FiniteDifferentiation(dl(j,i),dl(j,i),0,t(i),utilities);
        else
            dl(j,i) = FiniteDifferentiation(l(j,i),l(j,i-1),dl(j,i-1),t(i),utilities);
            ddl(j,i) = FiniteDifferentiation(dl(j,i),dl(j,i-1),ddl(j,i-1),t(i),utilities);
        end
    end
end

for j=1:cdpr_parameters.n_cables
   spline_id.l(j) = spline(t,cables(j).length);
   spline_id.l_d(j) = spline(t,cables(j).speed);
   spline_id.l_d2(j) = spline(t,cables(j).acceleration);
   
end

for j=1:cdpr_parameters.n_cables
   spline_real.l(j) = spline(t,l(j,:));
   spline_real.l_d(j) = spline(t,dl(j,:));
   spline_real.l_d2(j) = spline(t,ddl(j,:));
   
end
 
%  tic
%  sol_id = HuenDiscreteSolver(@(time,state) IntegrableDirectDynamics33(cdpr_parameters,...
%         cdpr_variables,utilities,spline_id,time,state),...
%         0:utilities.t_interval:t(end),p0);  
%  t1 = toc
 
 tic
 sol_real = HuenDiscreteSolver(@(time,state) IntegrableDirectDynamics33(cdpr_parameters,...
        cdpr_variables,utilities,spline_id,time,state),...
        0:utilities.t_interval:t(end),p0);  
 t2 = toc
