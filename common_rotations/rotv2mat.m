%% Rotation vector to rotation matrix conversion
% FROM:
% Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors
% James Diebel
% Stanford University
% 20 October 2006
%% 
function C = rotv2mat(R) 
 C = eye(3);
 v = norm(R);
 if (v == 0.0)
    return;
 else
   s2 = sin(v/2);
   c2 = cos(v/2);
   v2 = v*v;
   C(1,1) = (R(1)*R(1) - R(2)*R(2) - R(3)*R(3))*s2*s2/v2 + c2*c2;
   C(1,2) = (R(1)*R(2)*s2 - v*R(3)*c2)*2*s2/v2;
   C(1,3) = (R(1)*R(3)*s2 + v*R(2)*c2)*2*s2/v2;
   
   C(2,1) = (R(1)*R(2)*s2 + v*R(3)*c2)*2*s2/v2;
   C(2,2) = (R(2)*R(2) - R(3)*R(3) - R(1)*R(1))*s2*s2/v2 + c2*c2;
   C(2,3) = (R(2)*R(3)*s2 - v*R(1)*c2)*2*s2/v2;
   
   C(3,1) = (R(1)*R(3)*s2 - v*R(2)*c2)*2*s2/v2;
   C(3,2) = (R(2)*R(3)*s2 + v*R(1)*c2)*2*s2/v2;
   C(3,3) = (R(3)*R(3) - R(1)*R(1) - R(2)*R(2))*s2*s2/v2 + c2*c2;
end
return
