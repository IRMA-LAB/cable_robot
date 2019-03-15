function vector = CalcDirectKinematicsLengthSwivel(cdpr_p,record,l,s,variables)

cdpr_v = CdprVar(cdpr_p.n_cables);
cdpr_v = UpdateIKZeroOrd(variables(1:3,1),variables(4:end,1),cdpr_p,cdpr_v);
vector = zeros(2*cdpr_p.n_cables,1);

for i=1:cdpr_p.n_cables
  vector(2*i-1,1) = 100.*(cdpr_v.cable(i).length-l(i));
  vector(2*i,1) = 10.*(cdpr_v.cable(i).swivel_ang-s(i));
end

end