function cable_v = UpdatePosA(pos_A_loc,pos_D_glob,position,rot_mat,cable_v)

cable_v.pos_PA_glob = rot_mat*pos_A_loc;
cable_v.pos_OA_glob = position + cable_v.pos_PA_glob;
cable_v.pos_DA_glob = cable_v.pos_OA_glob - pos_D_glob;

end