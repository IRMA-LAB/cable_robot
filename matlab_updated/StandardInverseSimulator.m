function [output] = StandardInverseSimulator(cdpr_p,cdpr_v,...
     sim_data,geom_fun,ut)

for i = 1:sim_data.pNumber-1
    
 sim_data.coeff(:,i) = zeros(6,1);
 out = GenerateOutputUnderActuated(i,cdpr_p,cdpr_v,sim_data,geom_fun,ut);
 output(i).t = out.t;
 output(i).platform = out.platform;
 output(i).cables(:) = out.cable(:);
 output(i).coefficients = sim_data.coeff(:,i);
       
end

end