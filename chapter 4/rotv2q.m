function Q = rotv2q(R) % rotation vector to quaternion conversion
 Q = zeros(4,1);
 theta = norm(R);
 if (theta==0.0)
    Q(1) = 1;
    return ;
 else 
   Sinc = sin(theta/2.0)/theta;
   Q(1) = cos(theta/2.0);
   Q(2) = R(1)*Sinc;
   Q(3) = R(2)*Sinc;
   Q(4) = R(3)*Sinc;
end
