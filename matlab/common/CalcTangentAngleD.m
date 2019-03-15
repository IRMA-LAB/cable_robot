function angle_d = CalcTangentAngleD(vers_n,vel_OA_glob,pos_BA_glob)

angle_d = dot(vers_n,vel_OA_glob)/norm(pos_BA_glob);

end