function [Point_estim] = Point_estim_correct(Point_estim,Xcam,Options,Y)

ENU2RPY = q2mat(Point_estim.filter.x2_extr);        %transfer vector state (quaternion) to rotation matrix

% Xrpy(1)
Point_estim.filter.a = ENU2RPY(1)*(Options.PointZ1_RTK(1)-Xcam(1));
Point_estim.filter.b = ENU2RPY(4)*(Options.PointZ1_RTK(2)-Xcam(2));
Point_estim.filter.c = ENU2RPY(7)*(Options.PointZ1_RTK(3)-Xcam(3));
X1RPY = (Point_estim.filter.a+Point_estim.filter.b+Point_estim.filter.c);

% Xrpy(2)
Point_estim.filter.d = ENU2RPY(2)*(Options.PointZ1_RTK(1)-Xcam(1));
Point_estim.filter.e = ENU2RPY(5)*(Options.PointZ1_RTK(2)-Xcam(2));
Point_estim.filter.f = ENU2RPY(8)*(Options.PointZ1_RTK(3)-Xcam(3));
X2RPY = (Point_estim.filter.d+Point_estim.filter.e+Point_estim.filter.f);

% Xrpy(3)
Point_estim.filter.g = ENU2RPY(3)*(Options.PointZ1_RTK(1)-Xcam(1));
Point_estim.filter.h = ENU2RPY(6)*(Options.PointZ1_RTK(2)-Xcam(2));
Point_estim.filter.i = ENU2RPY(9)*(Options.PointZ1_RTK(3)-Xcam(3));
X3RPY = (Point_estim.filter.g+Point_estim.filter.h+Point_estim.filter.i);

%% vector Xrpy derirative of vector state X2 (q)
dCdQ = dQ2MATxAdQ(Point_estim.filter.x2_extr,Options.PointZ1_RTK);

%% Matrix of partial derivative
Point_estim.filter.dSdX = Point_estim.camera.Cam_F/(X3RPY)^2*[dCdQ(1,1)*X3RPY-dCdQ(3,1)*X1RPY dCdQ(1,2)*X3RPY-dCdQ(3,2)*X1RPY dCdQ(1,3)*X3RPY-dCdQ(3,3)*X1RPY dCdQ(1,4)*X3RPY-dCdQ(3,4)*X1RPY; dCdQ(2,1)*X3RPY-dCdQ(3,1)*X2RPY dCdQ(2,2)*X3RPY-dCdQ(3,2)*X2RPY dCdQ(2,3)*X3RPY-dCdQ(3,3)*X2RPY dCdQ(2,4)*X3RPY-dCdQ(3,4)*X2RPY];

%% Kalman filter coefficient
Point_estim.filter.K = Point_estim.filter.Dx2_extr*Point_estim.filter.dSdX'*inv(Point_estim.filter.dSdX*Point_estim.filter.Dx2_extr*Point_estim.filter.dSdX'+Point_estim.filter.Dn);

%% 
Xrpy = ENU2RPY*(Options.PointZ1_RTK - Xcam);
%predicted special point coordinates in RPY
predicted_FramePoint(1:2,1) = Point_estim.camera.Cam_F/Xrpy(3)*[Xrpy(1); Xrpy(2)];
%predicted frame coordinates special point
Point_estim.filter.x2 = Point_estim.filter.x2_extr + Point_estim.filter.K*(Y - predicted_FramePoint);
%new estimate coordinates special point in ENU

%% Variance error of new estimate coordinates in ENU
I=eye(4);       %
Point_estim.filter.Dx2 = (I-Point_estim.filter.K*Point_estim.filter.dSdX)*Point_estim.filter.Dx2_extr;
return