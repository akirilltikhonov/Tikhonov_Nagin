%% special orthogonal matrix to quaternion conversion (det(C) must be 1)
function Q = mat2q(C) 
    Q = [1;0;0;0];
    Q(1) = 0.5*sqrt(1 + C(1,1) + C(2,2) + C(3,3));
    ia = 1/(4*Q(1));
    Q(2) = (C(3,2)-C(2,3))*ia;
    Q(3) = (C(1,3)-C(3,1))*ia;
    Q(4) = (C(2,1)-C(1,2))*ia;
return
