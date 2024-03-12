sympref('FloatingPointOutput',true);
syms F_t F_tx F_ty F_tz % Force Total Body
syms F_p F_px F_py F_pz % Force Propulsion Body
syms F_g F_gx F_gy F_gz % Force Gravity Body
syms F_a F_ax F_ay F_az % Force Aerodynamic Body
syms F_d F_dx F_dy F_dz % Force Drag Body
syms Vb Vbx Vby Vbz     % Velocity Body
syms C_d A              % Drag Coeff, Area
syms Phi Theta Psi      % Roll, Pitch, Yaw
syms m                  % Mass

%% Aero Model
Vb_xy_len = sqrt(Vbx^2 + Vby^2);
Vb_xy_normx = Vbx / Vb_xy_len;
Vb_xy_normy = Vby / Vb_xy_len;

F_d_len = Vb_xy_len * C_d * A * 0.5 * 1.23;
F_dx = Vb_xy_normx * F_d_len;
F_dy = Vb_xy_normy * F_d_len;
F_dz = 0;

%% Gravity Model
a_gz = 9.81; % Acceleration of Gravity - Earth
F_gez = a_gz * m; % Force of Gravity - Earth
F_gx = (cos(Psi)*sin(Theta)*cos(Phi) + sin(Psi)*sin(Phi)) * F_gez;
F_gy = (sin(Psi)*sin(Theta)*cos(Phi) - cos(Psi)*sin(Phi)) * F_gez;
F_gz = (cos(Theta)*cos(Phi)) * F_gez;

%% Total
F_tx = F_px + F_gx + F_ax;
F_ty = F_py + F_gy + F_ay;
F_tz = F_pz + F_gz + F_az;