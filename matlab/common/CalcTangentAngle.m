function angle = CalcTangentAngle(vers_k,sw_r,vers_u,pos_DA_glob)

app_v = dot(vers_k,pos_DA_glob)/dot(vers_u,pos_DA_glob);
angle = 2*atan(app_v+sqrt(1 - 2*sw_r/dot(vers_u,pos_DA_glob) + app_v*app_v));

end