function scalar = CalcSwivelConstraint(cable_v)

scalar = dot(cable_v.vers_w,cable_v.pos_DA_glob);

end