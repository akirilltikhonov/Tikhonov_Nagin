clear all
close all
clc

amount = 100;     %Number of runs. Minimum 2
for i = 1:1:amount
[error,MODEL_TIME_SEC,F_frame,T,N_MODEL] = Basic()

error_XYZ (3*i:3*i, 1:N_MODEL)=0;
error_XYZ(3*i-2:3*i, 1:N_MODEL) = error;

error_X (i,1:N_MODEL)=0;
error_Y (i,1:N_MODEL)=0;
error_Z (i,1:N_MODEL)=0;

error_X(i,:) = error(1,:);
error_Y(i,:) = error(2,:);
error_Z(i,:) = error(3,:);

end

t=1:N_MODEL;          %all observations
l=t/F_frame; 

%% X coordinate
figure
plot(l,error_X)
legend ('Ошибка по координате X для N прогонов')
xlabel('Время,с')
ylabel('Ошибка оценивания,м')
grid on
title('Зависимость ошибки оценивания координат особой точки от времени')
ylim([min(error_X(:))-1 max(error_X(:))+1])


%% Y coordinate
figure
plot(l,error_Y)
legend ('Ошибка по координате Y для N прогонов')
xlabel('Время,с')
ylabel('Ошибка оценивания,м')
grid on
title('Зависимость ошибки оценивания координат особой точки от времени')
ylim([min(error_Y(:))-1 max(error_Y(:))+1])

%% Z coordinate
figure
plot(l,error_Z)
legend ('Ошибка по координате Z для N прогонов')
xlabel('Время,с')
ylabel('Ошибка оценивания,м')
grid on
title('Зависимость ошибки оценивания координат особой точки от времени')
ylim([min(error_Z(:))-1 max(error_Z(:))+1])


%% All 3 coordinates
% figure
% plot(l,error_XYZ)
% legend ('Ошибка по координате X для N прогонов','Ошибка по координате Y для N прогонов','Ошибка по координате Z для N прогонов')
% xlabel('Время,с')
% ylabel('Ошибка оценивания,м')
% grid on
% title('Зависимость ошибки оценивания координат особой точки от времени')
% ylim([min(error_XYZ(:))-1 max(error_XYZ(:))+1])