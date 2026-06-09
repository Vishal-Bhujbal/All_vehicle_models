%% Skidpad Centerline Generator - Formula Student (ISO 8855 ALIGNED)
% Track dimensions from rulebook:
%   Circle diameter = 15.25m → inner edge radius = 7.625m
%   Track width = 3.0m
%   Centerline radius = 7.625 + 1.5 = 9.125m
%   Vehicle Coordinate System: +X is Forward, +Y is Left.

r_cl = 9.125;           % Centerline radius of each circle (m)
cx_R =  0;              % Center of RIGHT circle (x)
cy_R = -9.125;          % Center of RIGHT circle (y) -> Physically to the right
cx_L =  0;              % Center of LEFT circle (x)
cy_L =  9.125;          % Center of LEFT circle (y) -> Physically to the left

N_straight = 50;        % Points along straight
N_circle   = 360;       % Points per full lap
L_straight = 12;        % Length of entry/exit straights (m)

%% 1. Entry straight: 12m straight up to origin (0,0)
% Runs along y=0, from x = -12m up to x = 0m (Car driving Forward in +X)
y_s1 = zeros(1, N_straight);
x_s1 = linspace(-L_straight, 0, N_straight);

%% 2. Two laps of RIGHT circle (clockwise = negative direction)
% Right circle center: (0, -9.125)
% Car enters at (0, 0) which is the TOP of the right circle → theta_start = +pi/2
% Turning right means heading clockwise: theta goes from pi/2 → pi/2 - 4*pi
theta_R = linspace(pi/2, pi/2 - 4*pi, 2*N_circle);
x_R = cx_R + r_cl * cos(theta_R);
y_R = cy_R + r_cl * sin(theta_R);

%% 3. Two laps of LEFT circle (counter-clockwise = positive direction)
% Left circle center: (0, +9.125)
% Car crosses back to (0, 0) (BOTTOM point of left circle) → theta_start = -pi/2
% Turning left means heading counter-clockwise: theta goes from -pi/2 → -pi/2 + 4*pi
theta_L = linspace(-pi/2, -pi/2 + 4*pi, 2*N_circle);
x_L = cx_L + r_cl * cos(theta_L);
y_L = cy_L + r_cl * sin(theta_L);

%% 4. Exit straight: 12m straight from origin continuing to +X
% Runs along y=0, from x = 0m up to x = +12m
y_s2 = zeros(1, N_straight);
x_s2 = linspace(0, L_straight, N_straight);

%% Assemble full path in sequence
% Using (2:end) for subsequent segments avoids generating duplicate (0,0) coordinates
x_all = [x_s1, x_R(2:end), x_L(2:end), x_s2(2:end)];
y_all = [y_s1, y_R(2:end), y_L(2:end), y_s2(2:end)];

%% Generate Track Boundaries for Racing Line Optimizer
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
plot(cx_L, cy_L, 'k+', 'MarkerSize', 12, 'HandleVisibility', 'off');
plot(cx_R, cy_R, 'k+', 'MarkerSize', 12, 'HandleVisibility', 'off');

% Draw reference circles (inner edge, centerline, outer edge)
theta_ref = linspace(0, 2*pi, 360);
for cy = [cy_L, cy_R]
    plot(7.625*cos(theta_ref), cy + 7.625*sin(theta_ref), 'k--', 'LineWidth', 0.5, 'HandleVisibility', 'off');
    plot(9.125*cos(theta_ref), cy + 9.125*sin(theta_ref), 'r--', 'LineWidth', 0.5, 'HandleVisibility', 'off');
    plot(10.625*cos(theta_ref), cy + 10.625*sin(theta_ref), 'k--', 'LineWidth', 0.5, 'HandleVisibility', 'off');
end

axis equal; grid on;
xlabel('Global X [m] (Forward)'); 
ylabel('Global Y [m] (Left)');
title('Skidpad Centerline - Vehicle Aligned');
legend('Centerline path', 'Entry', 'Exit', 'Location', 'best');