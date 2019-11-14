function RPY = mat2rpy(C) % rotation matrix to euler angles conversion
 RPY = zeros(3,1);
 RPY(1) = atan2( C(3,2), C(3,3));
 RPY(2) = -(pi/2 - atan2( norm(C(1:2,1)) , C(3,1)));
 RPY(3) = atan2(C(2,1), C(1,1));
 