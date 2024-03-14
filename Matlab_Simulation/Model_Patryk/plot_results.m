data = sin01to2;

bias_thrust = 0.031225;
bias_torque = -0.056332;

%thrust_scale = 1;
thrust_scale = 1/(0.57/2.28);

% Remove bias
data(:, 3) = data(:, 3) - bias_torque;
data(:, 4) = data(:, 4) - bias_thrust;

%Convert throttle to 0..1 range
data(:, 2) = data(:, 2) - 1000;
data(:, 2) = data(:, 2)./1000;

% Scale thrust
data(:, 4) = data(:, 4).*thrust_scale;

plot(data, "Time_s_", ["ESCSignal__s_", "Thrust_kgf_", "Torque_N_m_"]);