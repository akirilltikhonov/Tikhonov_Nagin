function [dSdX, Xrpy] = dSdX(PointZ, Xcam, ENU2RPY, Point_estim)

%% Find coorninates special point in RPY coordinate system
% Xrpy(1)
a = ENU2RPY(1)*(PointZ(1)-Xcam(1));
b = ENU2RPY(4)*(PointZ(2)-Xcam(2));
c = ENU2RPY(7)*(PointZ(3)-Xcam(3));
X1RPY = (a + b + c);

% Xrpy(2)
d = ENU2RPY(2)*(PointZ(1)-Xcam(1));
e = ENU2RPY(5)*(PointZ(2)-Xcam(2));
f = ENU2RPY(8)*(PointZ(3)-Xcam(3));
X2RPY = (d + e + f);

% Xrpy(3)
g = ENU2RPY(3)*(PointZ(1)-Xcam(1));
h = ENU2RPY(6)*(PointZ(2)-Xcam(2));
i = ENU2RPY(9)*(PointZ(3)-Xcam(3));
X3RPY = (g + h + i);
% Xrpy(1:3)
Xrpy = [X1RPY; X2RPY; X3RPY];

%% Vector Xrpy derirative of vector state X2 (q)
dXrpydQ = dQ2MATxAdQ(Point_estim.filter.x2_extr,PointZ);

%% Matrix of partial derivative
dSdX = Point_estim.camera.Cam_F/(X3RPY)^2*[dXrpydQ(1,1)*X3RPY-dXrpydQ(3,1)*X1RPY dXrpydQ(1,2)*X3RPY-dXrpydQ(3,2)*X1RPY dXrpydQ(1,3)*X3RPY-dXrpydQ(3,3)*X1RPY dXrpydQ(1,4)*X3RPY-dXrpydQ(3,4)*X1RPY; dXrpydQ(2,1)*X3RPY-dXrpydQ(3,1)*X2RPY dXrpydQ(2,2)*X3RPY-dXrpydQ(3,2)*X2RPY dXrpydQ(2,3)*X3RPY-dXrpydQ(3,3)*X2RPY dXrpydQ(2,4)*X3RPY-dXrpydQ(3,4)*X2RPY];


end

