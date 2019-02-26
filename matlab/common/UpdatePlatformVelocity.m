function platform_v = UpdatePlatformVelocity(velocity,orientation_deriv,rot_par,platform_v)

platform_v = platform_v.UpdateVelocity(velocity,orientation_deriv,rot_par);
platform_v.vel_OG_glob = platform_v.velocity + cross(platform_v.angular_vel,platform_v.pos_PG_glob);

end