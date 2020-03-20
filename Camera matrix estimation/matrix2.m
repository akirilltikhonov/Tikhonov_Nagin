clear
clc
% camera matrix

%% horizontal
F = 3.7e-3; % focus
D = 1620e-3; % distance to closest point
D1 = (1845e-3+1775e-3)/2; % distance to the edge of image
L = 1626e-3; % object size

ca = cos(D/D1);
F1 = F/ca; % distance to the edge of matrix
d_h = F1*L/(D1+F1)*1000
% d_h = 5.3140 mm

%% vertical
F = 3.7e-3; % focus
D = 1620e-3; % distance to closest point
D1 = (1710e-3+1740e-3)/2; % distance to the edge of image
L = 1220e-3; % object size

ca = cos(D/D1);
F1 = F/ca; % distance to the edge of matrix
d_v = F1*L/D1*1000
% d_v = 4.4316 mm

% Correction
% d_h/d_v
% % d_h/d_v = 1.19, должно быть 4/3=1.33
% 
% % переводить из px в мм Khoriz = 5.3140/640
% % переводить из px в мм Kvert = 4.4316/480
% 
% % или привести к 4/3
% % (d_h+a)/(d_v-a) = 4/3
% a = (4/3*d_v-d_h)/7*3
% 
% d_h = d_h+a
% d_v = d_v-a
% d_h/d_v
% % приведенные:
% % переводить из px в мм Khoriz = 5.5689/640
% % переводить из px в мм Kvert = 4.1767/480
% (d_h^2+d_v^2)^0.5


% % или привести к 4/3
% % (d_h-a)/(d_v) = 4/3
% a = d_h - (4/3*d_v)
% d_h = d_h-a
% d_v = d_v
% 
% d_h/d_v

a = d_h - (4/3*d_v)
d_h = d_h-a
d_v = d_v

d_h/d_v
(d_h^2+d_v^2)^0.5


% % % или привести к 4/3
% % % (d_h+a)/ = 4/3
% b = 10;
% % (d_h+a) = (d_v-a/b)*(4/3)
% % ((3*b)/(3*b)*a+4/(3*b)*a) = (4/3)*d_v-d_h
% % ((3*b)+4)/(3*b)*a = (4/3)*d_v-d_h
% a = ((4/3)*d_v-d_h)*(3*b)/((3*b)+4)
% 
% d_h = d_h+a
% d_v = d_v-a/10
% 
% d_h/d_v
% 
% (d_h^2+d_v^2)^0.5
