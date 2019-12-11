function [Options] = Main_options ()
%% MAIN OPTIONS
Options.MODEL_TIME_SEC = 100;   % observation time
Options.F_frame = 24;           % frames per second
Options.T = 1/Options.F_frame;          % frame duration
Options.N_MODEL = ceil(Options.MODEL_TIME_SEC/Options.T);   % number of observations

Options.sko_Coordinate_Meas = 0.05;     % RTK solution
Options.wVu = 0.5;                      % radial frequency
Options.Vku = 2;                        % max velocity

Options.Number_Z = 6;                           % Numbers of special points
Options.PointZ1 = [4;0;10];            % true coordinates of special point1 in ENU frame
Options.PointZ2 = [0;4;10];            % true coordinates of special point2 in ENU frame
Options.PointZ3 = [-4;0;10]; 
Options.PointZ4 = [0;-4;10]; 
Options.PointZ5 = [3;3;10]; 
Options.PointZ6 = [-3;-3;10];
Options.PointZ = [Options.PointZ1; Options.PointZ2; Options.PointZ3; Options.PointZ4; Options.PointZ5; Options.PointZ6];

Options.ENU2RPY = eye(3);               % rotation matrix RPY to ENU
Options.RPY2ENU = Options.ENU2RPY';     % rotation matrix ENU to RPY


%% Camera rotation and framepoint
Options.Tturn=60;                % period of turn
Options.Ufi1deg = 0;            % amplitude of turn (deg) relative to X
Options.Ufi2deg = 0;            % amplitude of turn (deg) relative to Y
Options.Ufi3deg = 0;           % amplitude of turn (deg) relative to Z

%% ENU2RPY error   
Options.error_deg = 1;          % by the end of simulation time error wiil be "error_deg" deg

return

