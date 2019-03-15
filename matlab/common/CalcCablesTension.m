function vector = CalcCablesTension(cdpr_v)

vector = linsolve(cdpr_v.analitic_jacobian*cdpr_v.analitic_jacobian',...
  cdpr_v.analitic_jacobian*cdpr_v.ext_load); 

end