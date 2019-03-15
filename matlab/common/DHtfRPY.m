function mat = DHtfRPY(angle,angle_d)

c1 = cos(angle(1)); s1 = sin(angle(1));
c2 = cos(angle(2)); s2 = sin(angle(2));
mat = zeros(3);
mat(1,2) = -c1*angle_d(1);
mat(1,3) = -s1*c2*angle_d(1) - c1*s2*angle_d(2);
mat(2,2) = -s1*angle_d(1);
mat(2,3) = c1*c2*angle_d(1) - s1*s2*angle_d(2);
mat(3,3) = -c2*angle_d(2);

end