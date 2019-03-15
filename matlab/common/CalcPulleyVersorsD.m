function cable_v = CalcPulleyVersorsD(cable_v)

cable_v.vers_u_deriv = cable_v.vers_w.*cable_v.swivel_ang_vel;
cable_v.vers_w_deriv = -cable_v.vers_u.*cable_v.swivel_ang_vel;

end