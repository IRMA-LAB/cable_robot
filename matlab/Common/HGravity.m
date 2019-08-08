function mat = HGravity(angle)

mat = eye(3,2);
mat(2,2) = cos(angle);
mat(3,2) = sin(angle);

end