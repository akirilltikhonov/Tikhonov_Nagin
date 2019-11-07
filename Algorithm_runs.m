clear all
close all
clc

amount = 10;     %Number of runs. Minimum 2
for i = 1:1:amount
[error,MODEL_TIME_SEC,F_frame,T,N_MODEL,seed(i,1)] = Basic();

error_XYZ(3*i-2:3*i, 1:N_MODEL) = error;

error_X(i,1:N_MODEL) = error(1,:);
error_Y(i,1:N_MODEL) = error(2,:);
error_Z(i,1:N_MODEL) = error(3,:);
end

for j = 1:1:N_MODEL
    %% СКО по X, Y, Z для каждого момента времени для N реализаций
error_X_std(j:N_MODEL) = std(error_X(:,j));
%error_X_std_mean = mean(error_X_std);

error_Y_std(j:N_MODEL) = std(error_Y(:,j));
%error_Y_std_mean = mean(error_Y_std);

error_Z_std(j:N_MODEL) = std(error_Z(:,j));
%error_Z_std_mean = mean(error_Z_std);
error_XYZ_std = [error_X_std; error_Y_std; error_Z_std];

% % expected value = 0
EV = 0;
error_X_std_EV0(j:N_MODEL) = sqrt((sum(error_X(:,j).^2)/(amount-1)));
error_Y_std_EV0(j:N_MODEL) = sqrt((sum(error_Y(:,j).^2)/(amount-1)));
error_Z_std_EV0(j:N_MODEL) = sqrt((sum(error_Z(:,j).^2)/(amount-1)));
end


%% Mean error X, Y, Z
% error_XYZ_mean = [mean(error_X); mean(error_Y); mean(error_Z)];

%%
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

%% X coordinate RMSE
figure
plot(l,error_X_std)
legend ('СКО по координате X для N прогонов')
xlabel('Время,с')
ylabel('СКО, м')
grid on
title('СКО по X для каждого момента времени для N реализаций')
ylim([min(error_X_std(:))-1 max(error_X_std(:))+1])

%% Y coordinate RMSE
figure
plot(l,error_Y_std)
legend ('СКО по координате Y для N прогонов')
xlabel('Время,с')
ylabel('СКО, м')
grid on
title('СКО по Y для каждого момента времени для N реализаций')
ylim([min(error_Y_std(:))-1 max(error_Y_std(:))+1])

%% Z coordinate RMSE
figure
plot(l,error_Z_std)
legend ('СКО по Z для каждого момента времени для N реализаций')
xlabel('Время,с')
ylabel('СКО, м')
grid on
title('СКО по Y для каждого момента времени для N реализаций')
ylim([min(error_Z_std(:))-1 max(error_Z_std(:))+1])

% %% All 3 coordinates RMSE
% figure
% plot(l,error_XYZ_std)
% legend ('СКО по X', 'СКО по Y', 'СКО по Z для каждого момента времени для N реализаций')
% xlabel('Время,с')
% ylabel('СКО, м')
% grid on
% title('Зависимость ошибки оценивания координат особой точки от времени')
% ylim([min(error_XYZ_std(:))-1 max(error_XYZ_std(:))+1])

%% All 3 coordinates
% figure
% plot(l,error_XYZ)
% legend ('Ошибка по координате X для N прогонов','Ошибка по координате Y для N прогонов','Ошибка по координате Z для N прогонов')
% xlabel('Время,с')
% ylabel('Ошибка оценивания,м')
% grid on
% title('Зависимость ошибки оценивания координат особой точки от времени')
% ylim([min(error_XYZ(:))-1 max(error_XYZ(:))+1])

%% Mean error X, Y, Z
% figure
% plot(l,error_XYZ_mean())
% legend ('Усредненная ошибка по координате X для N прогонов', 'Усредненная ошибка по координате Y для N прогонов', 'Усредненная ошибка по координате Z для N прогонов')
% xlabel('Время,с')
% ylabel('Ошибка оценивания,м')
% grid on
% title('Зависимость средней ошибки оценивания координат особой точки от времени')
% ylim([min(error_XYZ_mean(:))-1 max(error_XYZ_mean(:))+1])

%% Expected value 0
figure
plot(l,error_X_std_EV0)
legend ('СКО при EV=0 по координате Y')
xlabel('Время,с')
ylabel('СКО при EV=0, м')
grid on
title('Зависимость СКО по X при EV=0 в каждый момент времени')
ylim([min(error_X_std_EV0(:))-1 max(error_X_std_EV0(:))+1])

figure
plot(l,error_Y_std_EV0)
legend ('СКО при EV=0 по координате Y')
xlabel('Время,с')
ylabel('СКО при EV=0, м')
grid on
title('Зависимость СКО по Y при EV=0 в каждый момент времени')
ylim([min(error_Y_std_EV0(:))-1 max(error_Y_std_EV0(:))+1])

figure
plot(l,error_Z_std_EV0)
legend ('СКО при EV=0 по координате Z')
xlabel('Время,с')
ylabel('СКО при EV=0, м')
grid on
title('Зависимость СКО по Z при EV=0 в каждый момент времени')
ylim([min(error_Z_std_EV0(:))-1 max(error_Z_std_EV0(:))+1])