function platform_v = UpdatePlatformPose(position,orientation,rot_par,pos_G_loc,platform_v)

platform_v = platform_v.UpdatePose(position,orientation,rot_par);
platform_v.pos_PG_glob = platform_v.rot_mat*pos_G_loc;
platform_v.pos_OG_glob = platform_v.position + platform_v.pos_PG_glob;

end