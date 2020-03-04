function [Options] = Main_options ()
%% MAIN OPTIONS
Options.MODEL_TIME_SEC = 200;           % observation time
Options.F_frame = 24;                   % frames per second
Options.T = 1/Options.F_frame;          % frame duration
Options.N_MODEL = ceil(Options.MODEL_TIME_SEC/Options.T);   % number of observations

Options.sko_Coordinate_Meas = 0.05;     % RTK solution
Options.wVu = 0.5;                      % radial frequency
Options.Vku = 2;                        % max velocity

Options.Number_Z = 2;           % Numbers of special points

Options.PointsZ (1:3*Options.Number_Z, 1) = 0;
Options.PointsZ = [
%                      3; 0; 20;      % true coordinates of special point1 in ENU frame
%                      -3; 0; 20;      % 2
%                      0; 3; 20;      % 3     
%                      0; -3; 20;      % 4
%                      3; 0; 20;      % 5
%                      2; 2; 20;      % 6
%                      2; -2; 20;
%                      -2; 2; 20;
%                      -2; -2; 20;
%                      0; 0; 20;
%                               ];   


                     0; 4; 20;      % 1
                     0; -4; 20;      % 2                    
                     ];   

          
Options.ENU2RPY = eye(3);               % rotation matrix RPY to ENU
Options.RPY2ENU = Options.ENU2RPY';     % rotation matrix ENU to RPY


%% Camera rotation and framepoint
Options.Tturn = 300;               % period of turn
Options.Ufi1deg = 0;           % amplitude of turn (deg) relative to X (Roll)
Options.Ufi2deg = 10;           % amplitude of turn (deg) relative to Y (Pitch)
Options.Ufi3deg = 0;           % amplitude of turn (deg) relative to Z (Yaw)

return

