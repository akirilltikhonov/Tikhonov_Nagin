function [Options] = Main_options ()
%% MAIN OPTIONS
Options.MODEL_TIME_SEC = 200;           % observation time
Options.F_frame = 24;                   % frames per second
Options.T = 1/Options.F_frame;          % frame duration
Options.N_MODEL = ceil(Options.MODEL_TIME_SEC/Options.T);   % number of observations

Options.sko_Coordinate_Meas = 0.05;     % RTK solution
Options.wVu = 0.1;                      % radial frequency
Options.R = 4;                            % radius of curcus movement
Options.Vku = Options.R*Options.wVu;     % velocity (uniform movement around the circle)

Options.Number_Z = 24;           % Numbers of special points

Options.PointsZ (1:3*Options.Number_Z, 1) = 0;
Options.PointsZ = [
%                      
                     
                     4; 0; 10;      % true coordinates of special point 1 in ENU frame
                     0; 4; 10;      % 2     
                     0; -4; 10;     % 3                     
                     -4; 0; 10;     % 4
                     
                     6; 6; 15;      % 5
                     6; -6; 15;     % 6
                     -6; 6; 15;     % 7   
                     -6; -6; 15;
                     
                       0; 6; 15;
                     0; -6; 15;
                     -6; 0; 15;
                     6; 0; 15; 
                     
                     -7; 8; 11;
                     -8; 6; 9;
                     -10.5; 8.5; 10;
                     -9; 10; 9;
                     -12; 6; 8;
                     -9; 8; 15;
                     
                     
                     7; -8; 11;
                     8; -6; 9;
                     10.5; -8.5; 10;
                     9; -10; 9;
                     12; -6; 8;
                     9; -8; 15;
                     
                     ];   
                            
Options.ENU2RPY = eye(3);               % rotation matrix RPY to ENU
Options.RPY2ENU = Options.ENU2RPY';     % rotation matrix ENU to RPY


%% Camera rotation and framepoint
Options.Tturn = 200;               % period of turn
Options.Ufi1deg = 30;           % amplitude of turn (deg) relative to X (Roll)
Options.Ufi2deg = 30;           % amplitude of turn (deg) relative to Y (Pitch)
Options.Ufi3deg = 0;           % amplitude of turn (deg) relative to Z (Yaw)

return

