function [Xcam, myX] = Dynamic(Vku, wVu, sko_Coordinate_Meas, tt)


%% camera dynamic
myX = [Vku*1/wVu*sin(wVu*tt); Vku*1/wVu*cos(wVu*tt); 0];    % coordinates
myV = [Vku*cos(wVu*tt); -Vku*sin(wVu*tt); 0];               % velocity
myA = [-Vku*wVu*sin(wVu*tt); -Vku*wVu*cos(wVu*tt); 0];      % acceleration

%% 
Xcam = myX+randn(3,1)*sko_Coordinate_Meas;    %coordinate of camera with RTK solution