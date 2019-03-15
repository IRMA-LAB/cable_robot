function R = RotZ(angle)

R = eye(3);
R(1,1) = cos(angle);
R(2,2) = cos(angle);
R(1,2) = -sin(angle);
R(2,1) = sin(angle);

end