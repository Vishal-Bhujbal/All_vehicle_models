%% plot_steady_state_YMD.m
% Grabs existing workspace variables to run the steady-state YMD solver
% and plots the standard Constant Steer / Constant Sideslip mesh.

% 1. Safety check to ensure variables exist in the workspace
if ~exist('Par', 'var') || ~exist('sus_par', 'var') || ...
   ~exist('aero_par', 'var') || ~exist('TireData', 'var')
    error('Missing required structs! Please load Par, sus_par, aero_par, and TireData into your workspace first.');
end

% 2. Execute Your Solver Function
fprintf('Running steady-state YMD solver...\n');
doPlot = false; % We will handle the clean plotting in this script
[res_Ay, res_Mz, Steer_Deg, Beta_Deg] = yaw_moment_diagram(Par, sus_par, aero_par, TireData, doPlot);
fprintf('Solver finished. Generating plot...\n');

% 3. Setup the YMD Figure
figure('Name', 'Steady-State Yaw Moment Diagram', 'Color', 'w', 'Position', [100, 100, 900, 700]);
hold on;
grid on;
box on;

% 4. Plot the Mesh
% Plot lines of constant Steering Angle (Iterating through rows)
for i = 1:length(Steer_Deg)
    plot(res_Ay(i, :), res_Mz(i, :), 'Color', [0.2 0.6 1.0], 'LineWidth', 1.0);
end

% Plot lines of constant Sideslip Angle (Iterating through columns)
for j = 1:length(Beta_Deg)
    plot(res_Ay(:, j), res_Mz(:, j), 'Color', [1.0 0.4 0.4], 'LineWidth', 1.0);
end

% 5. Highlight the Zero-Lines for easy reading
idx_steer_0 = find(Steer_Deg == 0);
if ~isempty(idx_steer_0)
    plot(res_Ay(idx_steer_0, :), res_Mz(idx_steer_0, :), 'b-', 'LineWidth', 2.5);
end

idx_beta_0 = find(Beta_Deg == 0);
if ~isempty(idx_beta_0)
    plot(res_Ay(:, idx_beta_0), res_Mz(:, idx_beta_0), 'r-', 'LineWidth', 2.5);
end

% 6. Add Origin Axes
xline(0, 'k-', 'LineWidth', 1.5);
yline(0, 'k-', 'LineWidth', 1.5);

% 7. Formatting & Labels
title('Steady-State Yaw Moment Diagram', 'FontSize', 16, 'FontWeight', 'bold');
xlabel('Lateral Acceleration, A_y [g]', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Normalized Yaw Moment, M_z', 'FontSize', 14, 'FontWeight', 'bold');

% Create a clean legend using dummy plots
h1 = plot(nan, nan, 'Color', [0.2 0.6 1.0], 'LineWidth', 1.0);
h2 = plot(nan, nan, 'Color', [1.0 0.4 0.4], 'LineWidth', 1.0);
h3 = plot(nan, nan, 'b-', 'LineWidth', 2.5);
h4 = plot(nan, nan, 'r-', 'LineWidth', 2.5);
legend([h1, h2, h3, h4], ...
    {'Constant Steer (\delta)', 'Constant Sideslip (\beta)', ...
    '\delta = 0^\circ', '\beta = 0^\circ'}, ...
    'Location', 'best', 'FontSize', 11);

hold off;
fprintf('YMD generated successfully!\n');