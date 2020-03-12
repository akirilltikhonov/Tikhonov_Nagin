% camera matrix

%% horizontal
F = 3.7e-3; % focus
D = 2370e-3; % distance to closest point
D1 = 2656e-3; % distance to the edge of image
L = 2400e-3; % object size

ca = cos(D/D1);
F1 = F/ca; % distance to the edge of matrix
d = F1*L/D1*1000

% d = 5.3272 mm

%% vertical
F = 3.7e-3; % focus
D = 2750e-3; % distance to closest point
D1 = 2790e-3; % distance to the edge of image
L = 2*1020e-3; % object size

ca = cos(D/D1);
F1 = F/ca; % distance to the edge of matrix
d2 = F1*L/D1*1000

% d = 5.3272 mm