function cable_v = UpdateVelA(pos_PA_glob,velocity,angular_velocity,cable_v)

cable_v.vel_OA_glob = velocity + cross(angular_velocity,pos_PA_glob);

end