function mat = HtfQuaternion(q)

 mat(1,1) = -q(2);
 mat(1,2) = q(1);
 mat(1,3) = -q(4);
 mat(1,4) = q(3);
 
 mat(2,1) = -q(3);
 mat(2,2) = q(4);
 mat(2,3) = q(1);
 mat(2,4) = -q(2);
 
 mat(3,1) = -q(4);
 mat(3,2) = -q(3);
 mat(3,3) = q(2);
 mat(3,4) = q(1);

end