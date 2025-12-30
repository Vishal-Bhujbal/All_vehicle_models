% Loading Suspensions parameters: 

% suspension and damper constants.

% Initialize structure
sus_par = struct();

% Spring data
sus_par.Ks_f = 17472; % spring stiffness for front spring in N/m.
sus_par.Ks_r = 20816; % spring stiffness for rear spring in N/m.

% ARB Data
sus_par.K_arb_f = 3669; % ARB stiffness_front.
sus_par.K_arb_r = 805.8663; % ARB stiffness_rear.

% Dampers data
sus_par.C_damp_f = 14143; % Front damper coefficient in Ns/m.
sus_par.C_damp_r = 16358; % Rear damper coefficient in Ns/m.

% Motion ratio data
sus_par.Motion_ratio_f = 0.98; % Motion ratio_front.
sus_par.Motion_ratio_r = 0.99; % Motion ratio_rear.

% Rolling stiffness
sus_par.K_phi_f = 10014; % Front rolling stiffness.
sus_par.K_phi_r = 11466; % Rear rolling stiffness.

% Static values of angles(Camber, Caster and Toe)
sus_par.Camber_static = 0; % Static camber.
sus_par.Toe_f_static = 0; % Static toe angle_front.
sus_par.Toe_r_static = 0; % Static toe angle_rear.
sus_par.Caster_static = 0; % Static Caster angle.

% Compliance coefficients for FX,Fy and Mz
sus_par.C_Fx_toe = 0; % Toe compliance for Fx.
sus_par.C_Fy_toe = 0; % Toe compliance for Fy.
sus_par.C_Mz_toe = 0; % Toe compliance for Mz.
