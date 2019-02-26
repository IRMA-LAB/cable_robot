function cable_v = CalcPulleyVersors(vers_i,vers_j,cable_v)

cos_s = cos(cable_v.swivel_ang);
sin_s = sin(cable_v.swivel_ang);
cable_v.vers_u = vers_i*cos_s + vers_j*sin_s;
cable_v.vers_w = -vers_i*sin_s + vers_j*cos_s;

end