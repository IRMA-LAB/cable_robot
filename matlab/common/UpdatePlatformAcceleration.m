function platform_v = UpdatePlatformAcceleration(acceleration,orientation_deriv_2,rot_par,platform_v)

platform_v = platform_v.UpdateAcceleration(acceleration,orientation_deriv_2,rot_par);
platform_v.vel_OG_glob = platform_v.acceleration + cross(platform_c.angular_acc,platform_v.pos_PG_glob)+...
  cross(platform_c.angular_vel,cross(platform_c.angular_vel,platform_v.pos_PG_glob));

end