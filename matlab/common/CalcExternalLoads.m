function ext_load = CalcExternalLoads(platform_p,R_mat,H_mat,pos_PG_glob,R)

  ext_load(1:3,1) = R*(platform_p.mass.*platform_p.gravity_acceleration + ...
    R_mat*platform_p.ext_force_loc);
  ext_load(4:6,1) = H_mat'*(Anti(pos_PG_glob)*ext_load(1:3,1) + ...
    R_mat*platform_p.ext_moments_loc);

end