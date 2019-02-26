function R = RotTiltTorsion(v)


R = RotZYZ([v(1);v(2);v(3)-v(1)]);

end