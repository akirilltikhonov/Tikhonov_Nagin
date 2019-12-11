function [Point_estim] = Point_estim_correct(Point_estim,Xcam,Options,Y_1,Y_2,Y_3,Y_4,Y_5,Y_6)

ENU2RPY = q2mat(Point_estim.filter.x2_extr);        %transfer vector state (quaternion) to rotation matrix

%% What special points hit into camera lens?
% 1
if (Options.phantomZ1 == 1)         % special point 1 hit into camera lens
    [dS1dX2, Xrpy1] = dSdX(Options.PointZ1_RTK, Xcam, ENU2RPY, Point_estim);                    % matrix of partial derivative and coordinates in RPY of special point 1  
    predicted_FramePoint1(1:2,1) = Point_estim.camera.Cam_F/Xrpy1(3)*[Xrpy1(1); Xrpy1(2)];      % predicted frame coordinates special point 1

    Point_estim.filter.dSdX2 = dS1dX2;                      % matrix of partial derivative if special point 1 hit into camera lens
    predicted_FramePoint = predicted_FramePoint1;           % predicted frame coordinates special points if special point 1 hit into camera lens
    Y2 = Y_1;                                               % observation vector if special point 1 hit into camera lens
end
%% 2
if (Options.phantomZ1 == 0 && Options.phantomZ2 == 1)       % special point 2 hit into camera lens, but special point 1 don't 
    [dS2dX2, Xrpy2] = dSdX(Options.PointZ2_RTK, Xcam, ENU2RPY, Point_estim);                    % matrix of partial derivative and coordinates in RPY of special point 2  
    predicted_FramePoint2(1:2,1) = Point_estim.camera.Cam_F/Xrpy2(3)*[Xrpy2(1); Xrpy2(2)];      % predicted frame coordinates special point 2
    
    Point_estim.filter.dSdX2 = dS2dX2;                      % matrix of partial derivative if special point 2 hit into camera lens but special point 1 don't
    predicted_FramePoint = predicted_FramePoint2;           % predicted frame coordinates special points if special point 2 hit into camera lens but special point 1 don't
    Y2 = Y_2;                                               % observation vector if special point 2 hit into camera lens but special point 1 don't
elseif (Options.phantomZ2 == 1)       % special points 1 and 2 hit into camera lens
    [dS2dX2, Xrpy2] = dSdX(Options.PointZ2_RTK, Xcam, ENU2RPY, Point_estim);                    % matrix of partial derivative and coordinates in RPY of special point 2 
    predicted_FramePoint2(1:2,1) = Point_estim.camera.Cam_F/Xrpy2(3)*[Xrpy2(1); Xrpy2(2)];      % predicted frame coordinates special point 2

    Point_estim.filter.dSdX2 = [Point_estim.filter.dSdX2; dS2dX2];          % matrix of partial derivative if special point 1 and 2 hit into camera lens
    predicted_FramePoint = [predicted_FramePoint; predicted_FramePoint2];    % predicted frame coordinates special points if special point 1 and 2 hit into camera lens
    Y2 = [Y2; Y_2];                                                         % observation vector if special point 1 and 2 hit into camera lens
    end
%% 3
if (Options.phantomZ1 == 0 && Options.phantomZ2 == 0 && Options.phantomZ3 == 1)       % special point 3 hit into camera lens, but special point 2 don't
    [dS3dX2, Xrpy3] = dSdX(Options.PointZ3_RTK, Xcam, ENU2RPY, Point_estim);                    % matrix of partial derivative and coordinates in RPY of special point 3 
    predicted_FramePoint3(1:2,1) = Point_estim.camera.Cam_F/Xrpy3(3)*[Xrpy3(1); Xrpy3(2)];      % predicted frame coordinates special point 3

    Point_estim.filter.dSdX2 = dS3dX2;                      % matrix of partial derivative if special point 3 hit into camera lens but special point 2 don't
    predicted_FramePoint = predicted_FramePoint3;           % predicted frame coordinates special points if special point 3 hit into camera lens but special point 2 don't
    Y2 = Y_3;                                               % observation vector if special point 3 hit into camera lens but special point 2 don't
elseif (Options.phantomZ3 == 1)
    [dS3dX2, Xrpy3] = dSdX(Options.PointZ3_RTK, Xcam, ENU2RPY, Point_estim);                    % matrix of partial derivative and coordinates in RPY of special point 3 
    predicted_FramePoint3(1:2,1) = Point_estim.camera.Cam_F/Xrpy3(3)*[Xrpy3(1); Xrpy3(2)];      % predicted frame coordinates special point 3
    
    Point_estim.filter.dSdX2 = [Point_estim.filter.dSdX2; dS3dX2];          % matrix of partial derivative if special point 2 and 3 hit into camera lens
    predicted_FramePoint = [predicted_FramePoint; predicted_FramePoint3];    % predicted frame coordinates special points if special point 2 and 3 hit into camera lens
    Y2 = [Y2; Y_3];                                                         % observation vector if special point 2 and 3 hit into camera lens
end
%% 4
if (Options.phantomZ1 == 0 && Options.phantomZ2 == 0 && Options.phantomZ3 == 0 && Options.phantomZ4 == 1) 
    [dS4dX2, Xrpy4] = dSdX(Options.PointZ4_RTK, Xcam, ENU2RPY, Point_estim);
    predicted_FramePoint4(1:2,1) = Point_estim.camera.Cam_F/Xrpy4(3)*[Xrpy4(1); Xrpy4(2)];
    
    Point_estim.filter.dSdX2 = dS4dX2;
    predicted_FramePoint = predicted_FramePoint4;
    Y2 = Y_4; 
elseif(Options.phantomZ4 == 1)
    [dS4dX2, Xrpy4] = dSdX(Options.PointZ4_RTK, Xcam, ENU2RPY, Point_estim);
    predicted_FramePoint4(1:2,1) = Point_estim.camera.Cam_F/Xrpy4(3)*[Xrpy4(1); Xrpy4(2)];
    
    Point_estim.filter.dSdX2 = [Point_estim.filter.dSdX2; dS4dX2];
    predicted_FramePoint = [predicted_FramePoint; predicted_FramePoint4];
    Y2 = [Y2; Y_4];
end
%% 5
if (Options.phantomZ1 == 0 && Options.phantomZ2 == 0 && Options.phantomZ3 == 0 && Options.phantomZ4 == 0 && Options.phantomZ5 == 1)
    [dS5dX2, Xrpy5] = dSdX(Options.PointZ5_RTK, Xcam, ENU2RPY, Point_estim);
    predicted_FramePoint5(1:2,1) = Point_estim.camera.Cam_F/Xrpy5(3)*[Xrpy5(1); Xrpy5(2)];
    
    Point_estim.filter.dSdX2 = dS5dX2;
    predicted_FramePoint = predicted_FramePoint5;
    Y2 = Y_5; 
elseif (Options.phantomZ5 == 1)
    [dS5dX2, Xrpy5] = dSdX(Options.PointZ5_RTK, Xcam, ENU2RPY, Point_estim);
    predicted_FramePoint5(1:2,1) = Point_estim.camera.Cam_F/Xrpy5(3)*[Xrpy5(1); Xrpy5(2)];
    
    Point_estim.filter.dSdX2 = [Point_estim.filter.dSdX2; dS5dX2];
    predicted_FramePoint = [predicted_FramePoint; predicted_FramePoint5];
    Y2 = [Y2; Y_5];
end

%% 6
if (Options.phantomZ1 == 0 && Options.phantomZ2 == 0 && Options.phantomZ3 == 0 && Options.phantomZ4 == 0 && Options.phantomZ5 == 0 && Options.phantomZ6 == 1)  
    [dS6dX2, Xrpy6] = dSdX(Options.PointZ6_RTK, Xcam, ENU2RPY, Point_estim);
    predicted_FramePoint6(1:2,1) = Point_estim.camera.Cam_F/Xrpy6(3)*[Xrpy6(1); Xrpy6(2)];
    
    Point_estim.filter.dSdX2 = dS6dX2;
    predicted_FramePoint = predicted_FramePoint6;
    Y2 = Y_6; 
elseif (Options.phantomZ6 == 1)
    [dS6dX2, Xrpy6] = dSdX(Options.PointZ6_RTK, Xcam, ENU2RPY, Point_estim);
    predicted_FramePoint6(1:2,1) = Point_estim.camera.Cam_F/Xrpy6(3)*[Xrpy6(1); Xrpy6(2)];
    
    Point_estim.filter.dSdX2 = [Point_estim.filter.dSdX2; dS6dX2];
    predicted_FramePoint = [predicted_FramePoint; predicted_FramePoint6];
    Y2 = [Y2; Y_6];
end
%% Variance matrix of observations on a two-dimensional (camera) image. Its dimension depends on quantity special points which hit in camera lens
n = Options.phantomZ1 + Options.phantomZ2 + Options.phantomZ3 + Options.phantomZ4 + Options.phantomZ5 + Options.phantomZ6;
Point_estim.filter.Dn2 = Point_estim.filter.sko_Frame_Meas^2*diag([1:2*n]);

%% Kalman filter coefficient
Point_estim.filter.K = Point_estim.filter.Dx2_extr*Point_estim.filter.dSdX2'*inv(Point_estim.filter.dSdX2*Point_estim.filter.Dx2_extr*Point_estim.filter.dSdX2' + Point_estim.filter.Dn2);

%% New estimate of state vector X2 (quaternion)
Point_estim.filter.x2 = Point_estim.filter.x2_extr + Point_estim.filter.K*(Y2 - predicted_FramePoint);

%% Variance error of new estimate coordinates in ENU
I=eye(4);       %
Point_estim.filter.Dx2 = (I-Point_estim.filter.K*Point_estim.filter.dSdX2)*Point_estim.filter.Dx2_extr;
return