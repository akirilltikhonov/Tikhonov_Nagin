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
    %% ��� �� X, Y, Z ��� ������� ������� ������� ��� N ����������
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
legend ('������ �� ���������� X ��� N ��������')
xlabel('�����,�')
ylabel('������ ����������,�')
grid on
title('����������� ������ ���������� ��������� ������ ����� �� �������')
ylim([min(error_X(:))-1 max(error_X(:))+1])

%% Y coordinate
figure
plot(l,error_Y)
legend ('������ �� ���������� Y ��� N ��������')
xlabel('�����,�')
ylabel('������ ����������,�')
grid on
title('����������� ������ ���������� ��������� ������ ����� �� �������')
ylim([min(error_Y(:))-1 max(error_Y(:))+1])

%% Z coordinate
figure
plot(l,error_Z)
legend ('������ �� ���������� Z ��� N ��������')
xlabel('�����,�')
ylabel('������ ����������,�')
grid on
title('����������� ������ ���������� ��������� ������ ����� �� �������')
ylim([min(error_Z(:))-1 max(error_Z(:))+1])

%% X coordinate RMSE
figure
plot(l,error_X_std)
legend ('��� �� ���������� X ��� N ��������')
xlabel('�����,�')
ylabel('���, �')
grid on
title('��� �� X ��� ������� ������� ������� ��� N ����������')
ylim([min(error_X_std(:))-1 max(error_X_std(:))+1])

%% Y coordinate RMSE
figure
plot(l,error_Y_std)
legend ('��� �� ���������� Y ��� N ��������')
xlabel('�����,�')
ylabel('���, �')
grid on
title('��� �� Y ��� ������� ������� ������� ��� N ����������')
ylim([min(error_Y_std(:))-1 max(error_Y_std(:))+1])

%% Z coordinate RMSE
figure
plot(l,error_Z_std)
legend ('��� �� Z ��� ������� ������� ������� ��� N ����������')
xlabel('�����,�')
ylabel('���, �')
grid on
title('��� �� Y ��� ������� ������� ������� ��� N ����������')
ylim([min(error_Z_std(:))-1 max(error_Z_std(:))+1])

% %% All 3 coordinates RMSE
% figure
% plot(l,error_XYZ_std)
% legend ('��� �� X', '��� �� Y', '��� �� Z ��� ������� ������� ������� ��� N ����������')
% xlabel('�����,�')
% ylabel('���, �')
% grid on
% title('����������� ������ ���������� ��������� ������ ����� �� �������')
% ylim([min(error_XYZ_std(:))-1 max(error_XYZ_std(:))+1])

%% All 3 coordinates
% figure
% plot(l,error_XYZ)
% legend ('������ �� ���������� X ��� N ��������','������ �� ���������� Y ��� N ��������','������ �� ���������� Z ��� N ��������')
% xlabel('�����,�')
% ylabel('������ ����������,�')
% grid on
% title('����������� ������ ���������� ��������� ������ ����� �� �������')
% ylim([min(error_XYZ(:))-1 max(error_XYZ(:))+1])

%% Mean error X, Y, Z
% figure
% plot(l,error_XYZ_mean())
% legend ('����������� ������ �� ���������� X ��� N ��������', '����������� ������ �� ���������� Y ��� N ��������', '����������� ������ �� ���������� Z ��� N ��������')
% xlabel('�����,�')
% ylabel('������ ����������,�')
% grid on
% title('����������� ������� ������ ���������� ��������� ������ ����� �� �������')
% ylim([min(error_XYZ_mean(:))-1 max(error_XYZ_mean(:))+1])

%% Expected value 0
figure
plot(l,error_X_std_EV0)
legend ('��� ��� EV=0 �� ���������� Y')
xlabel('�����,�')
ylabel('��� ��� EV=0, �')
grid on
title('����������� ��� �� X ��� EV=0 � ������ ������ �������')
ylim([min(error_X_std_EV0(:))-1 max(error_X_std_EV0(:))+1])

figure
plot(l,error_Y_std_EV0)
legend ('��� ��� EV=0 �� ���������� Y')
xlabel('�����,�')
ylabel('��� ��� EV=0, �')
grid on
title('����������� ��� �� Y ��� EV=0 � ������ ������ �������')
ylim([min(error_Y_std_EV0(:))-1 max(error_Y_std_EV0(:))+1])

figure
plot(l,error_Z_std_EV0)
legend ('��� ��� EV=0 �� ���������� Z')
xlabel('�����,�')
ylabel('��� ��� EV=0, �')
grid on
title('����������� ��� �� Z ��� EV=0 � ������ ������ �������')
ylim([min(error_Z_std_EV0(:))-1 max(error_Z_std_EV0(:))+1])