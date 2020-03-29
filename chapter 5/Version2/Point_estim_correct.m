function [Point_estim] = Point_estim_correct(Point_estim, Number_Z, Y2)
%% Rotation matrix
ENU2RPY = q2mat(Point_estim.filter.x3_extr(4:7, 1));

%% Initialization of signal function and full matrix of derivative dSdX3
S = [];
dSdX3 = [];


for k = 1:Number_Z
    
    %% Find coorninates keypoints in RPY coordinate system
    % Xrpy(1)
    a = ENU2RPY(1)*(Point_estim.filter.x3_extr(7+3*k-2)-Point_estim.filter.x3_extr(1));
    b = ENU2RPY(4)*(Point_estim.filter.x3_extr(7+3*k-1)-Point_estim.filter.x3_extr(2));
    c = ENU2RPY(7)*(Point_estim.filter.x3_extr(7+3*k)-Point_estim.filter.x3_extr(3));
    X1RPY = (a + b + c);

    % Xrpy(2)
    d = ENU2RPY(2)*(Point_estim.filter.x3_extr(7+3*k-2)-Point_estim.filter.x3_extr(1));
    e = ENU2RPY(5)*(Point_estim.filter.x3_extr(7+3*k-1)-Point_estim.filter.x3_extr(2));
    f = ENU2RPY(8)*(Point_estim.filter.x3_extr(7+3*k)-Point_estim.filter.x3_extr(3));
    X2RPY = (d + e + f);

    % Xrpy(3)
    g = ENU2RPY(3)*(Point_estim.filter.x3_extr(7+3*k-2)-Point_estim.filter.x3_extr(1));
    h = ENU2RPY(6)*(Point_estim.filter.x3_extr(7+3*k-1)-Point_estim.filter.x3_extr(2));
    i = ENU2RPY(9)*(Point_estim.filter.x3_extr(7+3*k)-Point_estim.filter.x3_extr(3));
    X3RPY = (g + h + i);
    
    %% Si
    Si = Point_estim.camera.Cam_F/(X3RPY)^2*[X1RPY; X2RPY];
    
    %% Partial derivative dSi/dXcam
    dSdXcam = -Point_estim.camera.Cam_F/(X3RPY)^2*[ENU2RPY(1)*X3RPY-ENU2RPY(3)*X1RPY ENU2RPY(4)*X3RPY-ENU2RPY(6)*X1RPY ENU2RPY(7)*X3RPY-ENU2RPY(9)*X1RPY;
                                                   ENU2RPY(2)*X3RPY-ENU2RPY(3)*X2RPY ENU2RPY(5)*X3RPY-ENU2RPY(6)*X2RPY ENU2RPY(8)*X3RPY-ENU2RPY(9)*X2RPY];
                                             
    %% Vector Xrpy derirative of vector state qcam
    dXrpydQcam = dQ2MATxAdQ(Point_estim.filter.x3_extr(4:7, 1), Point_estim.filter.x3_extr(7+3*k-2:7+3*k, 1));

    % Partial derivative dSi/dqcam
    dSdQcam = Point_estim.camera.Cam_F/(X3RPY)^2*[dXrpydQcam(1,1)*X3RPY-dXrpydQcam(3,1)*X1RPY dXrpydQcam(1,2)*X3RPY-dXrpydQcam(3,2)*X1RPY dXrpydQcam(1,3)*X3RPY-dXrpydQcam(3,3)*X1RPY dXrpydQcam(1,4)*X3RPY-dXrpydQcam(3,4)*X1RPY;
                                                  dXrpydQcam(2,1)*X3RPY-dXrpydQcam(3,1)*X2RPY dXrpydQcam(2,2)*X3RPY-dXrpydQcam(3,2)*X2RPY dXrpydQcam(2,3)*X3RPY-dXrpydQcam(3,3)*X2RPY dXrpydQcam(2,4)*X3RPY-dXrpydQcam(3,4)*X2RPY];                                 
    
    %% Partial derivative dSi/dXi
    dSdXn = Point_estim.camera.Cam_F/(X3RPY)^2*[ENU2RPY(1)*X3RPY-ENU2RPY(3)*X1RPY ENU2RPY(4)*X3RPY-ENU2RPY(6)*X1RPY ENU2RPY(7)*X3RPY-ENU2RPY(9)*X1RPY;
                                                ENU2RPY(2)*X3RPY-ENU2RPY(3)*X2RPY ENU2RPY(5)*X3RPY-ENU2RPY(6)*X2RPY ENU2RPY(8)*X3RPY-ENU2RPY(9)*X2RPY];
    
    %% Matrix of derivative dSidX3 for i-th keypoints
    dSidX3(1:2, 1:7+3*Number_Z) = 0;
    dSidX3(1:2, 1:7) = [dSdXcam, dSdQcam];
    dSidX3(1:2, 7+3*k-2: 7+3*k) = dSdXn;
    
    %% Signal function
    S = [S ; Si];
    
    %% Full matrix of derivative dSdX3
    dSdX3 = [dSdX3 ; dSidX3];
end

%% Kalman filter coefficient
K = Point_estim.filter.Dx3_extr*dSdX3'*inv(dSdX3*Point_estim.filter.Dx3_extr*dSdX3' + Point_estim.filter.Dn3);

%% New estimate of state vector X3 (quaternion)
Point_estim.filter.x3 = Point_estim.filter.x3_extr + K*(Y2 - S);

%% Normalization of quartenion qcam
if ((abs(Point_estim.filter.x3(4)) - 1.0) >= 100*eps)
        Point_estim.filter.x3(4:7) = Point_estim.filter.x3(4:7)/norm(Point_estim.filter.x3(4:7));
end

%% Variance error of new estimate coordinates in ENU
I=eye(7+3*Number_Z);       %
Point_estim.filter.Dx3 = (I-K*dSdX3)*Point_estim.filter.Dx3_extr;

return