function cable_v = UpdateCableZeroOrd(cable_p,platform_v,cable_v)

cable_v = UpdatePosA(cable_p.pos_A_loc,cable_p.pos_D_glob,...
  platform_v.position,platform_v.rot_mat,cable_v);
cable_v.swivel_ang = CalcSwivelAngle(cable_p.vers_i,...
  cable_p.vers_j,cable_v.pos_DA_glob);
cable_v = CalcPulleyVersors(cable_p.vers_i,cable_p.vers_j,cable_v);
cable_v.tan_ang = CalcTangentAngle(cable_p.vers_k,...
  cable_p.swivel_pulley_r,cable_v.vers_u,cable_v.pos_DA_glob);
cable_v = CalcCableVectors(cable_p.swivel_pulley_r,cable_p.vers_k,cable_v);
cable_v.length = CalcCableLen(cable_p.swivel_pulley_r,...
  cable_v.tan_ang,cable_v.pos_BA_glob);

end