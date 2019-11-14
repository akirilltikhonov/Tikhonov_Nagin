function R = q2rotv(Q) % 2016 Nagin -  quaternion to rotation vector conversion

R = zeros(3,1);

if ((abs(Q(1)) - 1.0) >= 100*eps)
    Q = Q/norm(Q);
end

if (abs(abs(Q(1))-1.0) < 100*eps)
    R(1) = Q(2)*2;
    R(2) = Q(3)*2;
    R(3) = Q(4)*2;
else
    theta = 2*acos(Q(1));
    iSinc = theta/sin(theta/2.0);
    R(1) = Q(2)*iSinc;
    R(2) = Q(3)*iSinc;
    R(3) = Q(4)*iSinc;
end
