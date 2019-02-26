function angle_d = CalcSviwelAngleD(vers_u,vers_w,vel_OA_glob,pos_DA_glob)

angle_d = dot(vers_w,vel_OA_glob)/dot(vers_u,pos_DA_glob);

end