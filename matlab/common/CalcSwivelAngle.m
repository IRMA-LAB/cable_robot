function angle = CalcSwivelAngle(vers_i,vers_j,pos_DA_glob)

angle = atan2(dot(vers_j,pos_DA_glob),dot(vers_i,pos_DA_glob));

end