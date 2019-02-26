function cable_v = UpdateCableFirstOrd(cable_p,platform_v,cable_v)

  cable_v = UpdateVelA(cable_v.pos_PA_glob,platform_v.velocity,...
    platform_v.angular_vel,cable_v);
  cable_v.swivel_ang_vel = CalcSviwelAngleD(cable_v.vers_u,cable_v.vers_w,...
    cable_v.vel_OA_glob,cable_v.pos_DA_glob);
  cable_v = CalcPulleyVersorsD(cable_v);
  cable_v.tan_ang_vel = CalcTangentAngleD(cable_v.vers_n,...
    cable_v.vel_OA_glob,cable_v.pos_BA_glob);
  cable_v = CalcCableVectorsD(cable_p,cable_v);
  cable_v.speed = CalcCableSpeed(cable_v.vers_rho,cable_v.vel_OA_glob);
  [cable_v.geometric_jacobian_row, cable_v.analitic_jacobian_row] = ...
    CalcPlatformJacobianRow(cable_v.vers_rho,cable_v.pos_PA_glob,platform_v.H_mat);

end