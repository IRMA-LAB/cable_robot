function cable_v = CalcCableVectors(sw_r,vers_k,cable_v)

cos_p = cos(cable_v.tan_ang);
sin_p = sin(cable_v.tan_ang);
cable_v.vers_n = cable_v.vers_u*cos_p + vers_k*sin_p;
cable_v.vers_rho = cable_v.vers_u*sin_p - vers_k*cos_p;
cable_v.pos_BA_glob = cable_v.pos_DA_glob - sw_r*(cable_v.vers_u+cable_v.vers_n);

end