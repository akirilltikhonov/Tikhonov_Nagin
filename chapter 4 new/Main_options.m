function [Options] = Main_options ()
%% MAIN OPTIONS
Options.MODEL_TIME_SEC = 200;           % observation time
Options.F_frame = 24;                   % frames per second
Options.T = 1/Options.F_frame;          % frame duration
Options.N_MODEL = ceil(Options.MODEL_TIME_SEC/Options.T);   % number of observations

Options.sko_Coordinate_Meas = 0.05;     % RTK solution
Options.wVu = 0.5;                      % radial frequency
Options.Vku = 2;                        % max velocity

Options.Number_Z = 10;           % Numbers of special points

Options.PointsZ (1:3*Options.Number_Z, 1) = 0;
Options.PointsZ = [0; 1; 20;      % true coordinates of special point1 in ENU frame
                     0; 2; 20;      % 2
                     0; 1; 20;      % 3     
                     0; 2; 20;      % 4
                     1; 0; 20;      % 5
                     2; 0; 20;      % 6
                     2; 1; 20;
                     1; 1; 20;
                     -2; 0; 20;
                     -2; -1; 20;
                              ];   


%                     [0; 0.5; 50;      % true coordinates of special point1 in ENU frame
%                     0.5; 0; 50;      % 2
%                     0; -0.5; 50;      % 3     
%                     -0.5; 0; 50;      % 4
%                     0; 0.7; 50;      % 5
%                     0.7; 0; 50;      % 6
%                     0; -0.7; 50;
%                     -0.7; 0; 50;
%                     0; 0.5; 50;
%                     0.5; 0; 50;
%                              ];    
           
Options.ENU2RPY = eye(3);               % rotation matrix RPY to ENU
Options.RPY2ENU = Options.ENU2RPY';     % rotation matrix ENU to RPY


%% Camera rotation and framepoint
Options.Tturn=300;               % period of turn
Options.Ufi1deg = 0;           % amplitude of turn (deg) relative to X (Roll)
Options.Ufi2deg = 0;           % amplitude of turn (deg) relative to Y (Pitch)
Options.Ufi3deg = 10;           % amplitude of turn (deg) relative to Z (Yaw)

%% ENU2RPY error   
Options.error_deg = 0;          % by the end of simulation time error wiil be "error_deg" deg

return

