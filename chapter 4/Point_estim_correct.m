function [Point_estim] = Point_estim_correct(Point_estim,Xcam,Options,Y_1,Y_2)

ENU2RPY = q2mat(Point_estim.filter.x2_extr);        %transfer vector state (quaternion) to rotation matrix

%% Special points 1 and 2 hit into camera lens
if (Options.phantomZ1==1 && Options.phantomZ2==1)
% 1 Matrix of partial derivative and coordinates in RPY of special point 1     
[dS1dX2, Xrpy1] = dSdX(Options.PointZ1_RTK, Xcam, ENU2RPY, Point_estim);
%predicted frame coordinates special point 1
predicted_FramePoint1(1:2,1) = Point_estim.camera.Cam_F/Xrpy1(3)*[Xrpy1(1); Xrpy1(2)];

% 2 Matrix of partial derivative and coordinates in RPY of special point 2    
[dS2dX2, Xrpy2] = dSdX(Options.PointZ2_RTK, Xcam, ENU2RPY, Point_estim);
%predicted frame coordinates special point 2
predicted_FramePoint2(1:2,1) = Point_estim.camera.Cam_F/Xrpy2(3)*[Xrpy2(1); Xrpy2(2)];


% Matrix of partial derivative, predicted frame coordinates and variance matrix of observations on a two-dimensional (camera) image if special points 1 and 2 hit into camera lens
Point_estim.filter.dSdX2 = [dS1dX2; dS2dX2];
predicted_FramePoint = [predicted_FramePoint1; predicted_FramePoint2];
Y2 = [Y_1;Y_2];
Point_estim.filter.Dn2 = Point_estim.filter.sko_Frame_Meas^2*diag([1,1,1,1]);

%% Only special points 1 hit into camera lens
elseif (Options.phantomZ1==1 && Options.phantomZ2==0)
%% 1 Matrix of partial derivative and coordinates in RPY of special point 1     
[dS1dX2, Xrpy1] = dSdX(Options.PointZ1_RTK, Xcam, ENU2RPY, Point_estim);
%predicted frame coordinates special point 1
predicted_FramePoint1(1:2,1) = Point_estim.camera.Cam_F/Xrpy1(3)*[Xrpy1(1); Xrpy1(2)];

% Matrix of partial derivative, predicted frame coordinates and variance matrix of observations on a two-dimensional (camera) image if special points 1 and 2 hit into camera lens
Point_estim.filter.dSdX2 = dS1dX2;
predicted_FramePoint = predicted_FramePoint1;
Y2 = Y_1;
Point_estim.filter.Dn2 = Point_estim.filter.sko_Frame_Meas^2*diag([1,1]);

else (Options.phantomZ1==0 && Options.phantomZ2==1)
%% 2 Matrix of partial derivative and coordinates in RPY of special point 2    
[dS2dX2, Xrpy2] = dSdX(Options.PointZ2_RTK, Xcam, ENU2RPY, Point_estim);
%predicted frame coordinates special point 2
predicted_FramePoint2(1:2,1) = Point_estim.camera.Cam_F/Xrpy2(3)*[Xrpy2(1); Xrpy2(2)];


% Matrix of partial derivative, predicted frame coordinates and variance matrix of observations on a two-dimensional (camera) image if special points 1 and 2 hit into camera lens
Point_estim.filter.dSdX2 = dS2dX2;
predicted_FramePoint = predicted_FramePoint2;
Y2 = Y_2;
Point_estim.filter.Dn2 = Point_estim.filter.sko_Frame_Meas^2*diag([1,1]);
end

%% Kalman filter coefficient
Point_estim.filter.K = Point_estim.filter.Dx2_extr*Point_estim.filter.dSdX2'*inv(Point_estim.filter.dSdX2*Point_estim.filter.Dx2_extr*Point_estim.filter.dSdX2' + Point_estim.filter.Dn2);

%% New estimate of state vector X2 (quaternion)
Point_estim.filter.x2 = Point_estim.filter.x2_extr + Point_estim.filter.K*(Y2 - predicted_FramePoint);

%% Variance error of new estimate coordinates in ENU
I=eye(4);       %
Point_estim.filter.Dx2 = (I-Point_estim.filter.K*Point_estim.filter.dSdX2)*Point_estim.filter.Dx2_extr;
return