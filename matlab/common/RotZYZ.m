function R = RotZYZ(v)

R = RotZ(v(1))*RotY(v(2))*RotZ(v(3));

end