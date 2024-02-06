%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% VEHICLE PARAMETERS %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% REFERENCE POINT - Basically ground level when rocket is idle on the ground

ct_pos = [0 0 0.2]; % Vertical Distance of Centre of Thrust from the reference point
cg_pos = [0.03 0 0.7]; % Vertical Distance of Centre of Gravity from the reference point

Mass = 2; % Mass in kg

%%% Aerodynamics %%%
C_d = 1.05;   % Drag coefficient
Area = 0.1;   % Area

%%% Sensor Placement %%%
%dist_pos = % Distance Sensor Position

%%% Kalman Filter Setup %%%
load("Kalman_SS");
Q = eye(16)*0.02;
R = eye(6)*0.05;