function TF = compute_tf_coeffs(Va, alpha, theta, de, dt, P)
    % computes longitudinal and lateral transfer function coefficients
    % inputs:
    %   Va:    [1x1] trim airspeed (m/s)
    %   alpha: [1x1] trim angle of attack (rad)
    %   theta: [1x1] trim pitch angle (rad)
    %   de:    [1x1] trim elevator (rad)
    %   dt:    [1x1] trim throttle (unitless)
    %   P:     [1x1 struct] physical parameters
    % outputs:
    %   TF:    [1x1 struct] transfer function coefficients

    % 1. dynamic pressure and air density (assuming sea level for trim)
    rho = P.rho; 
    
    % 2. lateral coefficients (roll subsystem)
    cp_p  = P.Gamma3 * P.C_Mxp + P.Gamma4 * P.C_Mzp;
    cp_da = P.Gamma3 * P.C_Mxda + P.Gamma4 * P.C_Mzda;
    
    TF.a_phi1 = -0.5 * rho * Va^2 * P.S * P.b * cp_p * (P.b / (2 * Va));
    TF.a_phi2 = 0.5 * rho * Va^2 * P.S * P.b * cp_da;
    
    % 3. longitudinal coefficients (pitch subsystem)
    TF.a_theta1 = -(rho * Va^2 * P.c * P.S / (2 * P.Jy)) * P.C_Myq * (P.c / (2 * Va));
    TF.a_theta2 = -(rho * Va^2 * P.c * P.S / (2 * P.Jy)) * P.C_Myalpha;
    TF.a_theta3 = (rho * Va^2 * P.c * P.S / (2 * P.Jy)) * P.C_Myde;
    
    % 4. longitudinal coefficients (airspeed subsystem)
    eps_num = 1e-4;
    
    % numerical derivative of thrust w.r.t airspeed (dTp_dVa)
    [f_p_plus, ~]  = get_propulsion(Va + eps_num, dt, rho, P);
    [f_p_minus, ~] = get_propulsion(Va - eps_num, dt, rho, P);
    dtp_dva = (f_p_plus(1) - f_p_minus(1)) / (2 * eps_num);
    
    % numerical derivative of thrust w.r.t throttle (dTp_ddt)
    [f_p_plus, ~]  = get_propulsion(Va, dt + eps_num, rho, P);
    [f_p_minus, ~] = get_propulsion(Va, dt - eps_num, rho, P);
    dtp_ddt = (f_p_plus(1) - f_p_minus(1)) / (2 * eps_num);
    
    % linear drag coefficient derivative
    ar = (P.b^2) / P.S;
    cd_alpha = 2 * (P.C_L0 + P.C_Lalpha * alpha) * P.C_Lalpha / (pi * P.e * ar);
    
    TF.a_V1 = (rho * Va * P.S / P.mass) * (P.C_D0 + cd_alpha * alpha + P.C_Dde * de) - (1 / P.mass) * dtp_dva;
    TF.a_V2 = (1 / P.mass) * dtp_ddt;
    TF.a_V3 = P.g * cos(theta - alpha);
end

function [F_p, M_p] = get_propulsion(Va, dt, rho, P)
    % localized propulsion logic for numerical derivative calculations
    if dt == 0
        F_p = [0;0;0]; M_p = [0;0;0]; return;
    end
    v_in = P.V_max * dt;
    a = (rho * P.D_prop^5) / ((2 * pi)^2) * P.C_Q0;
    b = (rho * P.D_prop^4) / (2 * pi) * P.C_Q1 * Va + (P.k_Q * P.k_V) / P.R_motor;
    c = rho * P.D_prop^3 * P.C_Q2 * Va^2 - (P.k_Q / P.R_motor) * v_in + P.k_Q * P.i0;
    omega_p = (-b + sqrt(b^2 - 4 * a * c)) / (2 * a);
    
    t_p = (rho*P.D_prop^4)/((2*pi)^2)*omega_p^2*P.C_T0 + (rho*P.D_prop^3)/(2*pi)*omega_p*Va*P.C_T1 + rho*P.D_prop^2*Va^2*P.C_T2;
    q_p = (rho*P.D_prop^5)/((2*pi)^2)*omega_p^2*P.C_Q0 + (rho*P.D_prop^4)/(2*pi)*omega_p*Va*P.C_Q1 + rho*P.D_prop^3*Va^2*P.C_Q2;
    F_p = [t_p; 0; 0]; M_p = [-q_p; 0; 0];
end