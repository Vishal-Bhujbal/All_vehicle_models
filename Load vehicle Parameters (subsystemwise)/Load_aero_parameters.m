% Aero Parameters: 
aero_par = struct();

aero_par.Cl = -0.5;  % Lift coefficient.
aero_par.Cd = 0.8;  % Drag coefficient.
aero_par.Area = 1.3;     % Reference area in square meters.
aero_par.CP_rear = 0.5;   % COP rear ratio.
aero_par.rho = 1.225;  % Density of air.
aero_par.C_side = 0; % Sideways_drag.

assignin("base","aero_par",aero_par)
