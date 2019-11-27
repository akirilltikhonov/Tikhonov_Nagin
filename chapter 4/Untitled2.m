clear 
clc

% A1_deg = 180;
% A2_deg = 0;
% A3_deg = 0;
% 
% %%
% A1 = deg2rad(A1_deg);
% A2 = deg2rad(A2_deg);
% A3 = deg2rad(A3_deg);
% A123=[A1;A2;A3];
% 
% A = sqrt(A1^2 + A2^2 + A3^2);
% a1 = cos(A/2);
% a2 = (A1/A)*sin(A/2);
% a3 = (A2/A)*sin(A/2);
% a4 = (A3/A)*sin(A/2);
% 
% %
% BB = rand(1,1);
% BB_SKO1 = 0.09;
% BB_SKO = sqrt(BB_SKO1^2 + BB_SKO1^2 + BB_SKO1^2);
% 
% B1_deg = BB*BB_SKO1;
% B2_deg = BB*BB_SKO1;
% B3_deg = BB*BB_SKO1;
% 
% B1 = deg2rad(B1_deg);
% B2 = deg2rad(B2_deg);
% B3 = deg2rad(B3_deg);
% B = sqrt(B1^2 + B2^2 + B3^2);
% 
% 
% 
% b1 = cos(B/2);
% b2 = (B1/B)*sin(B/2);
% b3 = (B2/B)*sin(B/2);
% b4 = (B3/B)*sin(B/2);
% b = [b1;b2;b3;b4]
% ab = [a1*b1-a2*b2-a3*b3-a4*b4; a2*b1+a1*b2-a4*b3+a3*b4; a3*b1+a4*b2+a1*b3-a2*b4; a4*b1-a3*b2+a2*b3+a1*b4]
% 
% 
% c1 = deg2rad(BB*BB_SKO);
% c2 =deg2rad(BB*BB_SKO1);
% c3 = deg2rad(BB*BB_SKO1);
% c4 = deg2rad(BB*BB_SKO1);
% c = [c1; c2; c3; c4];
% a_plus_c = a+c

a = [1 2 3 4; 5 6 7 8];

b = a(1:2, 1:2)

