% Loading vehicle parameters: 

%initiliaze structure 
Par = struct();

Par.g = 9.81; % Acceleration due to gravity.
Par.R0 = 0.228; % Wheel radius in m.
Par.m = 270; % Vehicle mass in kg.
Par.d_Cg_Cp = 0; % Distance between COP and CG.
Par.WD_f = 0.49; % Front weight distribution.
Par.WD_r = 0.51; % Rear weight distribution. 
Par.C_rr = 0.0150; % Rolling resistance coefficient.

% Wheel base and track width
Par.WB = 1.535; % Wheel base of the vehicle.
Par.Lf = Par.WB*0.51; % Front wheel base.
Par.Lr = Par.WB*0.49; % Rear wheel base.
Par.Tf = 1.17; % Front track width.
Par.Tr = 1.17; % Rear track width.

% CG and RC heights
Par.RC_h = 0.0767; % Roll center height in m.
Par.CG_h = 0.2050; % CG height in m.

% Inertia values
Par.Ix = 62; %Inertia about roll axis
Par.Iy = 105; %Inertia about pitch axis
Par.Iz = 47.563; %Inertia about yaw axis
Par.Iw = 1; % Wheel inertia

% Static vertical load
Par.Fz0 = 270*9.81; 

