function [Point_estim] = Point_estim_correct(Point_estim,Xcam_mas,ENU2RPY_with_error,Y,k)
ENU2RPY = eye(3);
ENU2RPY = ENU2RPY_with_error;
% Xrpy(1)
Point_estim.filter.a = ENU2RPY(1)*(Point_estim.filter.xoc(1)-Xcam_mas(1,k));
Point_estim.filter.b = ENU2RPY(4)*(Point_estim.filter.xoc(2)-Xcam_mas(2,k));
Point_estim.filter.c = ENU2RPY(7)*(Point_estim.filter.xoc(3)-Xcam_mas(3,k));
X1RPY = (Point_estim.filter.a+Point_estim.filter.b+Point_estim.filter.c);

% Xrpy(2)
Point_estim.filter.d = ENU2RPY(2)*(Point_estim.filter.xoc(1)-Xcam_mas(1,k));
Point_estim.filter.e = ENU2RPY(5)*(Point_estim.filter.xoc(2)-Xcam_mas(2,k));
Point_estim.filter.f = ENU2RPY(8)*(Point_estim.filter.xoc(3)-Xcam_mas(3,k));
X2RPY = (Point_estim.filter.d+Point_estim.filter.e+Point_estim.filter.f);

% Xrpy(3)
Point_estim.filter.g = ENU2RPY(3)*(Point_estim.filter.xoc(1)-Xcam_mas(1,k));
Point_estim.filter.h = ENU2RPY(6)*(Point_estim.filter.xoc(2)-Xcam_mas(2,k));
Point_estim.filter.i = ENU2RPY(9)*(Point_estim.filter.xoc(3)-Xcam_mas(3,k));
X3RPY = (Point_estim.filter.g+Point_estim.filter.h+Point_estim.filter.i);

%% Matrix of partial derivative
Point_estim.filter.dSdX = Point_estim.camera.Cam_F/(X3RPY)^2*[ENU2RPY(1)*X3RPY-ENU2RPY(3)*X1RPY ENU2RPY(4)*X3RPY-ENU2RPY(6)*X1RPY ENU2RPY(7)*X3RPY-ENU2RPY(9)*X1RPY; ENU2RPY(2)*X3RPY-ENU2RPY(3)*X2RPY ENU2RPY(5)*X3RPY-ENU2RPY(6)*X2RPY ENU2RPY(8)*X3RPY-ENU2RPY(9)*X2RPY];

%% Kalman filter coefficient
Point_estim.filter.K = Point_estim.filter.Dx_extr*Point_estim.filter.dSdX'*inv(Point_estim.filter.dSdX*Point_estim.filter.Dx_extr*Point_estim.filter.dSdX'+Point_estim.filter.Dn);

%% 
Xrpy = ENU2RPY*(Point_estim.filter.x_extr - Xcam_mas(1:3,k));
%predicted special point coordinates in RPY
predicted_FramePoint(1:2,1) = Point_estim.camera.Cam_F/Xrpy(3)*[Xrpy(1); Xrpy(2)];
%predicted frame coordinates special point
Point_estim.filter.xoc = Point_estim.filter.x_extr + Point_estim.filter.K*(Y - predicted_FramePoint);
%new estimate coordinates special point in ENU

%% Variance error of new estimate coordinates in ENU
I=eye(3);       %
Point_estim.filter.Dx = (I-Point_estim.filter.K*Point_estim.filter.dSdX)*Point_estim.filter.Dx_extr;
return