sympref('FloatingPointOutput',true);
syms F_t F_tx F_ty F_tz % Force Total Body
syms M_t M_tx M_ty M_tz % Moment Total Body
syms F_p F_px F_py F_pz % Force Propulsion Body
syms M_p M_px M_py M_pz % Moments Propulsion Body
syms F_g F_gx F_gy F_gz % Force Gravity Body
syms F_a F_ax F_ay F_az % Force Aerodynamic Body
syms F_d F_dx F_dy F_dz % Force Drag Body
syms Vb Vbx Vby Vbz     % Velocity Body
syms C_d A              % Drag Coeff, Area
syms Phi Theta Psi      % Roll, Pitch, Yaw
syms m g                % Mass
syms T Mz               % Thrust, Yaw Propulsion Torque
syms CTPosx CTPosy CTPosz % Centre of thrust position
syms CGPosx CGPosy CGPosz % Centre of gravity position
syms d_T d_yaw d_roll d_pitch % Commands: Throttle, Yaw, TVC Roll TVC Pitch

%% Actuator Model !!!TODO!!!


%% Aero Model
Vb_xy_len = sqrt(Vbx^2 + Vby^2);
Vb_xy_normx = Vbx / Vb_xy_len;
Vb_xy_normy = Vby / Vb_xy_len;

F_d_len = Vb_xy_len * C_d * A * 0.5 * 1.23;
F_dx = Vb_xy_normx * F_d_len;
F_dy = Vb_xy_normy * F_d_len;
F_dz = 0;

F_ax = F_dx;
F_ay = F_dy;
F_az = F_dz;

%% Gravity Model
F_gez = g * m; % Force of Gravity - Earth
F_gx = (cos(Psi)*sin(Theta)*cos(Phi) + sin(Psi)*sin(Phi)) * F_gez;
F_gy = (sin(Psi)*sin(Theta)*cos(Phi) - cos(Psi)*sin(Phi)) * F_gez;
F_gz = (cos(Theta)*cos(Phi)) * F_gez;

%% Propulsion Model
Tdirx = (cos(Psi)*sin(Theta)*cos(Phi) + sin(Psi)*sin(Phi)) * -1;
Tdiry = (sin(Psi)*sin(Theta)*cos(Phi) - cos(Psi)*sin(Phi)) * -1;
Tdirz = (cos(Theta)*cos(Phi)) * -1;

F_px = Tdirx * T;
F_py = Tdiry * T;
F_pz = Tdirz * T;

TPosx = CTPosx - CGPosx;
TPosy = CTPosy - CGPosy;
TPosz = CTPosz - CGPosz;

% Cross product for moments calculation
M_px = TPosy * F_pz - TPosz * F_py;
M_py = TPosz * F_px - TPosx * F_pz;
M_pz = TPosx * F_py - TPosy * F_px + Mz;


%% Total Forces and Moments
F_tx = F_px + F_gx + F_ax;
F_ty = F_py + F_gy + F_ay;
F_tz = F_pz + F_gz + F_az;

M_tx = M_px;
M_ty = M_py;
M_tz = M_pz;

%% Equations of Motion
syms Vbx_dot Vby_dot Vbz_dot; % u_dot, v_dot, w_dot
syms p q r p_dot q_dot r_dot;

Vbx_dot = F_tx / m; % u
Vby_dot = F_ty / m; % v
Vbz_dot = F_tz / m; % w

p_dot = 

