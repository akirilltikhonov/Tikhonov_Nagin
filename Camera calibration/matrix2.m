% camera matrix

%% horizontal
F = 3.7e-3; % focus
D = 1620e-3; % distance to closest point
D1 = (1845e-3+1775e-3)/2; % distance to the edge of image
L = 1626e-3; % object size

ca = cos(D/D1);
F1 = F/ca; % distance to the edge of matrix
d = F1*L/D1*1000

% d = 5.3272 mm

%% vertical
F = 3.7e-3; % focus
D = 1620e-3; % distance to closest point
D1 = (1710e-3+1740e-3)/2; % distance to the edge of image
L = 1220e-3; % object size

ca = cos(D/D1);
F1 = F/ca; % distance to the edge of matrix
d2 = F1*L/D1*1000

d/d2
% d = 5.3272 mm