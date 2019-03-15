function mat = HtfTiltTorsion(angle)

c1 = cos(angle(1)); s1 = sin(angle(1));
c2 = cos(angle(2)); s2 = sin(angle(2));
mat = zeros(3);
mat(1,1) = -c1*s2;
mat(1,2) = -s1;
mat(1,3) = -mat(1,1);
mat(2,1) = -s1*s2;
mat(2,2) = c1;
mat(2,3) = -mat(2,1);
mat(3,1) = 1-c2;
mat(3,3) = c2;

end