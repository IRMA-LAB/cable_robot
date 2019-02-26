function mat = DHtfQuaternion(dq)

 mat(1,1) = -dq(2);
 mat(1,2) = dq(1);
 mat(1,3) = -dq(4);
 mat(1,4) = dq(3);
 
 mat(2,1) = -dq(3);
 mat(2,2) = dq(4);
 mat(2,3) = dq(1);
 mat(2,4) = -dq(2);
 
 mat(3,1) = -dq(4);
 mat(3,2) = -dq(3);
 mat(3,3) = dq(2);
 mat(3,4) = dq(1);

end