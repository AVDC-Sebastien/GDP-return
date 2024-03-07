%%% Equations of Motion %%%
syms L M N Fx Fy Fz;
syms p q r p_dot q_dot r_dot;
syms u v w u_dot v_dot w_dot;
syms I Ixx Ixy Ixz Iyx Iyy Iyz Izx Izy Izz;
syms m;

I = [Ixx Ixy Ixz;
    Iyx Iyy Iyz;
    Izx Izy Izz];

L = p_dot*Ixx