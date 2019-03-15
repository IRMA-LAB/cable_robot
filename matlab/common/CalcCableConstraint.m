function scalar = CalcCableConstraint(cable_v,r)

scalar = dot(cable_v.pos_BA_glob,cable_v.pos_BA_glob) -...
  (cable_v.length-r.*(pi-cable_v.tan_ang)).^2;

end