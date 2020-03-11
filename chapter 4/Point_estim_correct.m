function [Point_estim] = Point_estim_correct(Point_estim,Xcam,Options,Y2)

ENU2RPY = q2mat(Point_estim.filter.x2_extr);        %transfer vector state (quaternion) to rotation matrix

% ENU2RPY = Options.ENU2RPY(1:3,3*k-2:3*k);

%% What special points hit into camera lens?
flag = 0;   % marker which indicated that no one special point don't hit into camera lens
for N = 1:1:Options.Number_Z
       
    if (Options.phantomZ(N)==1 && N==1 && flag==0)  % special point N=1 hit into camera lens
        
        [dSndX2, Xrpy_N] = dSdX(Options.PointsZ_RTK(3*N-2:3*N), Xcam, ENU2RPY, Point_estim);        % partial derivative and coordinates in RPY of special point N=1
        predicted_FramePoint_N(1:2,1) = Point_estim.camera.Cam_F/Xrpy_N(3)*[Xrpy_N(1); Xrpy_N(2)];  % predicted frame coordinates special point N=1
        
        Point_estim.filter.dSdX2 = dSndX2;                  % matrix of partial derivative if special point N=1 hit into camera lens
        predicted_FramePoint = predicted_FramePoint_N;      % matrix predicted frame coordinates special points if special point N=1 hit into camera lens
        Y2_obs = Y2(2*N-1:2*N);                                     % observation vector if special point N=1 hit into camera lens
        
        flag = 1;       % marker which indicated that one of special point hit into camera lensin in first time
        
    elseif (Options.phantomZ(N)==1 && N~=1 && flag==0)  % special point N (but N not equal 1) hit into camera lens and flag=0 (it is first point)

        [dSndX2, Xrpy_N] = dSdX(Options.PointsZ_RTK(3*N-2:3*N), Xcam, ENU2RPY, Point_estim);        % partial derivative and coordinates in RPY of special point N
        predicted_FramePoint_N(1:2,1) = Point_estim.camera.Cam_F/Xrpy_N(3)*[Xrpy_N(1); Xrpy_N(2)];  % predicted frame coordinates special point N
        
        Point_estim.filter.dSdX2 = dSndX2;                  % matrix of partial derivative if special point N hit into camera lens
        predicted_FramePoint = predicted_FramePoint_N;      % matrix predicted frame coordinates special points if special point N hit into camera lens
        Y2_obs = Y2(2*N-1:2*N);                                     % observation vector if special point N hit into camera lens
        
        flag = 1;       % marker which indicated that one of special point hit into camera lensin in first time
        
    elseif (Options.phantomZ(N)==1 && flag==1)          % special point N hit into camera lens and flag=1 (it isn't first point)
        
        [dSndX2, Xrpy_N] = dSdX(Options.PointsZ_RTK(3*N-2:3*N), Xcam, ENU2RPY, Point_estim);        % partial derivative and coordinates in RPY of special point N
        predicted_FramePoint_N(1:2,1) = Point_estim.camera.Cam_F/Xrpy_N(3)*[Xrpy_N(1); Xrpy_N(2)];  % predicted frame coordinates special point N
        
        Point_estim.filter.dSdX2 = [Point_estim.filter.dSdX2; dSndX2];              % matrix of partial derivative if special point N hit into camera lens 
        predicted_FramePoint = [predicted_FramePoint; predicted_FramePoint_N];      % matrix predicted frame coordinates special points if special point N hit into camera lens  
        Y2_obs = [Y2_obs; Y2(2*N-1:2*N)];                                                   % observation vector if special point N hit into camera lens
    end      
end  

%% Variance matrix of observations on a two-dimensional (camera) image. Its dimension depends on quantity special points which hit in camera lens
% amount special points which hit into camera lens
n = length(find(Options.phantomZ==true));     
Point_estim.filter.Dn2 = Point_estim.filter.sko_Frame_Meas^2*diag([1:2*n]);

%% Kalman filter coefficient
Point_estim.filter.K = Point_estim.filter.Dx2_extr*Point_estim.filter.dSdX2'*inv(Point_estim.filter.dSdX2*Point_estim.filter.Dx2_extr*Point_estim.filter.dSdX2' + Point_estim.filter.Dn2);

%% New estimate of state vector X2 (quaternion)
Point_estim.filter.x2 = Point_estim.filter.x2_extr + Point_estim.filter.K*(Y2_obs - predicted_FramePoint);

%% Normalization of quartenion
if ((abs(Point_estim.filter.x2(1)) - 1.0) >= 100*eps)
        Point_estim.filter.x2 = Point_estim.filter.x2/norm(Point_estim.filter.x2);
end

%% Variance error of new estimate coordinates in ENU
I=eye(4);       %
Point_estim.filter.Dx2 = (I-Point_estim.filter.K*Point_estim.filter.dSdX2)*Point_estim.filter.Dx2_extr;
return