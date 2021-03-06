function [Point_estim] = Point_estim_correct(Point_estim,ENU2RPY_with_error_mas,Y1,k);
ENU2RPY = eye(3);
ENU2RPY = ENU2RPY_with_error_mas(:,3*k-2:3*k);
% Xrpy(1)
Point_estim.filter.a = ENU2RPY(1)*(Point_estim.filter.x1_extr(1)-Point_estim.filter.x1_extr(4));
Point_estim.filter.b = ENU2RPY(4)*(Point_estim.filter.x1_extr(2)-Point_estim.filter.x1_extr(5));
Point_estim.filter.c = ENU2RPY(7)*(Point_estim.filter.x1_extr(3)-Point_estim.filter.x1_extr(6));
X1RPY = (Point_estim.filter.a+Point_estim.filter.b+Point_estim.filter.c);

% Xrpy(2)
Point_estim.filter.d = ENU2RPY(2)*(Point_estim.filter.x1_extr(1)-Point_estim.filter.x1_extr(4));
Point_estim.filter.e = ENU2RPY(5)*(Point_estim.filter.x1_extr(2)-Point_estim.filter.x1_extr(5));
Point_estim.filter.f = ENU2RPY(8)*(Point_estim.filter.x1_extr(3)-Point_estim.filter.x1_extr(6));
X2RPY = (Point_estim.filter.d+Point_estim.filter.e+Point_estim.filter.f);

% Xrpy(3)
Point_estim.filter.g = ENU2RPY(3)*(Point_estim.filter.x1_extr(1)-Point_estim.filter.x1_extr(4));
Point_estim.filter.h = ENU2RPY(6)*(Point_estim.filter.x1_extr(2)-Point_estim.filter.x1_extr(5));
Point_estim.filter.i = ENU2RPY(9)*(Point_estim.filter.x1_extr(3)-Point_estim.filter.x1_extr(6));
X3RPY = (Point_estim.filter.g+Point_estim.filter.h+Point_estim.filter.i);

%% Matrix of partial derivative
Top_dSdX1 = (Point_estim.camera.Cam_F/X3RPY^2)*[ENU2RPY(1)*X3RPY-ENU2RPY(3)*X1RPY  ENU2RPY(4)*X3RPY-ENU2RPY(6)*X1RPY  ENU2RPY(7)*X3RPY-ENU2RPY(9)*X1RPY  -ENU2RPY(1)*X3RPY+ENU2RPY(3)*X1RPY  -ENU2RPY(4)*X3RPY+ENU2RPY(6)*X1RPY  -ENU2RPY(7)*X3RPY+ENU2RPY(9)*X1RPY; ENU2RPY(2)*X3RPY-ENU2RPY(3)*X2RPY  ENU2RPY(5)*X3RPY-ENU2RPY(6)*X2RPY  ENU2RPY(8)*X3RPY-ENU2RPY(9)*X2RPY  -ENU2RPY(2)*X3RPY+ENU2RPY(3)*X2RPY  -ENU2RPY(5)*X3RPY+ENU2RPY(6)*X2RPY  -ENU2RPY(8)*X3RPY+ENU2RPY(9)*X2RPY];
Button_dSdX1 = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1];
Point_estim.filter.dSdX1 = [Top_dSdX1; Button_dSdX1];

%% Kalman filter coefficient
Point_estim.filter.K = Point_estim.filter.Dx1_extr*Point_estim.filter.dSdX1'*inv(Point_estim.filter.dSdX1*Point_estim.filter.Dx1_extr*Point_estim.filter.dSdX1'+Point_estim.filter.Dn1);

%%
%predicted special point coordinates in RPY
Xrpy = ENU2RPY*(Point_estim.filter.x1_extr(1:3) - Point_estim.filter.x1_extr(4:6));

%predicted frame coordinates special point
predicted_FramePoint(1:2,1) = Point_estim.camera.Cam_F/Xrpy(3)*[Xrpy(1); Xrpy(2)];

%predicted observation vector
predicted_observation_vector = [predicted_FramePoint; Point_estim.filter.x1_extr(4:6)];

%new estimate coordinates state vector
Point_estim.filter.x1 = Point_estim.filter.x1_extr + Point_estim.filter.K*(Y1 - predicted_observation_vector);

%% Variance error of new estimate coordinates of state vector
I=eye(6);       %
Point_estim.filter.Dx1 = (I-Point_estim.filter.K*Point_estim.filter.dSdX1)*Point_estim.filter.Dx1_extr;

return