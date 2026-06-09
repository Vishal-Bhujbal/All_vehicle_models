%% Skidpad Centerline Generator - Formula Student (ISO 8855 ALIGNED)
% Vehicle starts at (0,0) heading in +X direction.
% +X is Forward, +Y is Left (Left circle center is at +Y).

clear; clc; close all;

r_cl = 9.125;           % Centerline radius of each circle (m)
cx_L = 0;               % Center of LEFT circle (x)
cy_L = r_cl;            % Center of LEFT circle (y) -> +9.125m

cx_R = 0;               % Center of RIGHT circle (x)
cy_R = -r_cl;           % Center of RIGHT circle (y) -> -9.125m

N_straight = 50;         % Points along straight (Keep 0 if just doing circles)
N_circle   = 360;       % Points per full lap

%% 1. Entry straight: from -X up to origin (0,0)
% Car drives along the X-axis heading towards +X
x_entry = -r_cl;        
x_mid   =  0;           
y_s1 = zeros(1, N_straight);
x_s1 = linspace(x_entry, x_mid, N_straight);

%% 2. Two laps of LEFT circle (counterclockwise = positive direction)
% Center is at (0, +9.125). Car enters at (0,0), which is the BOTTOM of this circle.
% The bottom of the circle corresponds to an angle of -90 deg (-pi/2).
theta_L = linspace(-pi/2, -pi/2 + 4*pi, 2*N_circle);   
x_L = cx_L + r_cl * cos(theta_L);
y_L = cy_L + r_cl * sin(theta_L);

%% 3. Two laps of RIGHT circle (clockwise = negative direction)
% Center is at (0, -9.125). Car crosses origin (0,0), which is the TOP of this circle.
% The top of the circle corresponds to an angle of +90 deg (+pi/2).
theta_R = linspace(pi/2, pi/2 - 4*pi, 2*N_circle);   
x_R = cx_R + r_cl * cos(theta_R);
y_R = cy_R + r_cl * sin(theta_R);

%% 4. Exit straight: from origin (0,0) continuing to +X
y_s2 = zeros(1, N_straight);
x_s2 = linspace(x_mid, r_cl, N_straight);

%% Assemble full path in sequence
x_all = [x_s1, x_L, x_R, x_s2];
y_all = [y_s1, y_L, y_R, y_s2];

%% Generate Track Boundaries for Racing Line Optimizer
% Track width is 3.0m, so left and right boundaries are 1.5m from centerline
wl = 1.5 * ones(size(x_all)); 
wr = 1.5 * ones(size(x_all)); 

%% Write to 4-Column CSV
skidpad_data = table(x_all', y_all', wl', wr', 'VariableNames', {'x', 'y', 'wl', 'wr'});
writetable(skidpad_data, 'skidpad_centerline.csv');
disp('CSV written: skidpad_centerline.csv (4-column format)');
disp(['Total points: ', num2str(length(x_all))]);

%% Plot to verify
figure('Name', 'Skidpad Map (ISO 8855)', 'Color', 'w');
plot(x_all, y_all, 'b-', 'LineWidth', 1.5); hold on;
plot(x_all(1), y_all(1), 'go', 'MarkerSize', 10, 'DisplayName', 'Start');
plot(x_all(end), y_all(end), 'rs', 'MarkerSize', 10, 'DisplayName', 'End');

% Mark circle centers
plot(cx_L, cy_L, 'k+', 'MarkerSize', 12, 'DisplayName', 'Left Center');
plot(cx_R, cy_R, 'k+', 'MarkerSize', 12, 'DisplayName', 'Right Center');

% Draw reference circles
theta_ref = linspace(0, 2*pi, 360);
% Left Boundaries
plot(cx_L + 7.625*cos(theta_ref), cy_L + 7.625*sin(theta_ref), 'k--', 'LineWidth', 0.5);  
plot(cx_L + 10.625*cos(theta_ref), cy_L + 10.625*sin(theta_ref), 'k--', 'LineWidth', 0.5); 
% Right Boundaries
plot(cx_R + 7.625*cos(theta_ref), cy_R + 7.625*sin(theta_ref), 'k--', 'LineWidth', 0.5);  
plot(cx_R + 10.625*cos(theta_ref), cy_R + 10.625*sin(theta_ref), 'k--', 'LineWidth', 0.5); 

axis equal; grid on;
xlabel('Global X [m] (Forward)'); 
ylabel('Global Y [m] (Left)');
title('Skidpad Centerline - Vehicle Aligned');
legend('Location', 'best');