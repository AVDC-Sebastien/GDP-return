%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% VEHICLE PARAMETERS %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% REFERENCE POINT - Basically ground level when rocket is idle on the ground

ct_pos = [0 0 0.2]; % Vertical Distance of Centre of Thrust from the reference point
cg_pos = [0.0 0 0.7]; % Vertical Distance of Centre of Gravity from the reference point

Mass = 5; % Mass in kg
Inertia = eye(3);

%%% Aerodynamics %%%
C_d = 1.05;   % Drag coefficient
Area = 0.1;   % Area

%%% Actuator Data %%%
TVC_Range = 10; % Max Nozzle Deflection (deg)
Max_Thrust = 22.4; % Max EDF Thrust (N)

%%% Sensor Placement %%%
%dist_pos = % Distance Sensor Position

K_tf = tf([-254.7 -81.76 -6.563], [1 227.4 5004 0]);