% =========================================================================
% AEROSONDE UAV PARAMETERS
% =========================================================================

%% --- GRAVITY AND MASS ---
P.g    = 9.81;       % Gravitational constant (m/s^2)
P.mass = 11.0;       % Aircraft mass (kg)

%% --- MOMENTS AND PRODUCTS OF INERTIA ---
% These determine the aircraft's resistance to angular acceleration.
P.Jx  = 0.824;       % Roll moment of inertia (kg-m^2)
P.Jy  = 1.135;       % Pitch moment of inertia (kg-m^2)
P.Jz  = 1.759;       % Yaw moment of inertia (kg-m^2)
P.Jxz = 0.120;       % Product of inertia (kg-m^2)

%% --- INERTIA KINEMATIC CONSTANTS ---
% Pre-computed constants used in the rigid-body rotational equations.
Gamma    = P.Jx*P.Jz - P.Jxz^2;
P.Gamma1 = P.Jxz*(P.Jx - P.Jy + P.Jz) / Gamma;
P.Gamma2 = (P.Jz*(P.Jz - P.Jy) + P.Jxz^2) / Gamma;
P.Gamma3 = P.Jz / Gamma;
P.Gamma4 = P.Jxz / Gamma;
P.Gamma5 = (P.Jz - P.Jx) / P.Jy;
P.Gamma6 = P.Jxz / P.Jy;
P.Gamma7 = ((P.Jx - P.Jy)*P.Jx + P.Jxz^2) / Gamma;
P.Gamma8 = P.Jx / Gamma;

%% --- AIRCRAFT GEOMETRY ---
P.S = 0.55;          % Wing planform area (m^2)
P.b = 2.9;           % Wingspan (m)
P.c = 0.19;          % Mean aerodynamic chord (m)

%% --- NONLINEAR STALL AND DRAG PARAMETERS ---
P.M      = 50;       % Stall blending function slope
P.alpha0 = 0.47;     % Stall angle of attack (rad)
P.e       = 0.9;     % Oswald efficiency factor

%% --- LONGITUDINAL AERODYNAMIC COEFFICIENTS ---
% Subscripts:
%   0      = zero angle of attack
%   alpha  = angle of attack derivative
%   q      = pitch-rate derivative
%   de     = elevator deflection derivative

% Lift
P.C_L0     = 0.23;
P.C_Lalpha = 5.61;
P.C_Lq     = 7.95;
P.C_Lde    = 0.13;

% Drag
P.C_D0     = 0.0424;
P.C_Dalpha = 0.132;
P.C_Dq     = 0;
P.C_Dde    = 0.0135;
P.C_Dp     = 0.043;

% Pitch moment
P.C_My0     = 0.0135;
P.C_Myalpha = -2.74;
P.C_Myq     = -38.21;
P.C_Myde    = -0.99;

%% --- LATERAL AERODYNAMIC COEFFICIENTS ---
% Subscripts:
%   beta = sideslip angle
%   p    = roll rate
%   r    = yaw rate
%   da   = aileron deflection
%   dr   = rudder deflection

% Side force
P.C_Y0     = 0;
P.C_Ybeta  = -0.83;
P.C_Yp     = 0;
P.C_Yr     = 0;
P.C_Yda    = 0.075;
P.C_Ydr    = 0.19;

% Roll moment
P.C_Mx0     = 0;
P.C_Mxbeta  = -0.13;
P.C_Mxp     = -0.51;
P.C_Mxr     = 0.25;
P.C_Mxda    = 0.17;
P.C_Mxdr    = 0.0024;

% Yaw moment
P.C_Mz0     = 0;
P.C_Mzbeta  = 0.073;
P.C_Mzp     = -0.069;
P.C_Mzr     = -0.095;
P.C_Mzda    = -0.011;
P.C_Mzdr    = -0.069;

%% --- PROPULSION PARAMETERS (MOTOR & BATTERY) ---
P.V_max   = 44.4;    % Maximum battery voltage (V)
P.k_V     = 0.0659;  % Motor constant (V·s/rad)
P.k_Q     = 0.0659;  % Motor torque constant (N·m/A)
P.R_motor = 0.042;   % Motor winding resistance (Ohms)
P.i0      = 1.5;     % No-load current (A)

%% --- PROPELLER AERODYNAMIC COEFFICIENTS ---
% Torque coefficient polynomial
P.C_Q0 = 0.005230;
P.C_Q1 = 0.004970;
P.C_Q2 = -0.01664;

% Thrust coefficient polynomial
P.C_T0 = 0.09357;
P.C_T1 = -0.06044;
P.C_T2 = -0.1079;

P.D_prop = 0.508;    % Propeller diameter (m)

%% --- ATMOSPHERIC PARAMETERS ---
P.T0    = 288.15;      % Standard temperature at sea level (K)
P.p0    = 101325;      % Standard pressure at sea level (Pa)
P.L0    = -0.0065;     % Temperature lapse rate (K/m)
P.R     = 8.31432;     % Universal gas constant (J/(mol·K))
P.Molar = 0.0289644;   % Molar mass of air (kg/mol)
P.rho   = 1.268;       % Air density (kg/m^3)