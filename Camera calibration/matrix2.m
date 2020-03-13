% camera matrix

%% horizontal
F = 3.7e-3; % focus
D = 1620e-3; % distance to closest point
D1 = (1845e-3+1775e-3)/2; % distance to the edge of image
L = 1626e-3; % object size

ca = cos(D/D1);
F1 = F/ca; % distance to the edge of matrix
d = F1*L/D1*1000
% d = 5.3140 mm

%% vertical
F = 3.7e-3; % focus
D = 1620e-3; % distance to closest point
D1 = (1710e-3+1740e-3)/2; % distance to the edge of image
L = 1220e-3; % object size

ca = cos(D/D1);
F1 = F/ca; % distance to the edge of matrix
d2 = F1*L/D1*1000
% d2 = 4.4316 mm

d/d2
% d/d2 = 1.19, должно быть 4/3=1.33

% переводить из px в мм Khoriz = 5.3140/640
% переводить из px в мм Kvert = 4.4316/480

% или привести к 4/3
% (d+a)/(d2-a) = 4/3
a = (4/3*d2-d)/7*3

d = d+a
d2 = d2-a
d/d2
% приведенные:
% переводить из px в мм Khoriz = 5.5689/640
% переводить из px в мм Kvert = 4.1767/480
(d^2+d2^2)^0.5
