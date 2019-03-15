function R = RotX(angle)

R = eye(3);
R(2,2) = cos(angle);
R(3,3) = cos(angle);
R(2,3) = -sin(angle);
R(3,2) = sin(angle);

end