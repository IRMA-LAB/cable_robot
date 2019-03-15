function R = RotY(angle)

R = eye(3);
R(1,1) = cos(angle);
R(3,3) = cos(angle);
R(1,3) = sin(angle);
R(3,1) = -sin(angle);

end